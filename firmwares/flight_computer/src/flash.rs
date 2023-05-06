//! Driver for the Winbond W25Q flash chip, including buffering of telemetry messages.
//! This should maybe be separated into the chip driver and a higher level implementation
//! of settings/blackbox storage.
//!
//! Datasheet: https://www.mouser.com/datasheet/2/949/w25q256jv_spi_revg_08032017-1489574.pdf

use alloc::sync::Arc;
use alloc::vec::Vec;
use embedded_hal_one::digital::blocking::OutputPin;
use embedded_hal_one::spi::blocking::SpiBus;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use hal::dma::config::DmaConfig;
use hal::dma::{MemoryToPeripheral, Transfer, StreamX};
use hal::pac::{interrupt, Interrupt, DMA1, NVIC};
use hal::spi::Tx;
use stm32f4xx_hal as hal;

use crc::{Crc, CRC_16_IBM_SDLC};

use crate::prelude::*;
use crate::telemetry::*;
use crate::usb::*;

#[cfg(feature = "rev1")]
type DmaStream = StreamX<DMA1, 4>;
#[cfg(feature = "rev2")]
type DmaStream = StreamX<DMA1, 7>;

#[cfg(feature = "rev1")]
type DmaTx = Tx<hal::pac::SPI2>;
#[cfg(feature = "rev2")]
type DmaTx = Tx<hal::pac::SPI3>;

type SpiDmaTransfer = Transfer<
    DmaStream,
    0,
    DmaTx,
    MemoryToPeripheral,
    &'static mut [u8; DMA_BUFFER_SIZE],
>;

const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4096;

const DMA_BUFFER_SIZE: usize = 256 + 1 + 4;
static DMA_TRANSFER: Mutex<RefCell<Option<SpiDmaTransfer>>> = Mutex::new(RefCell::new(None));
static DMA_RESULT: Mutex<RefCell<Option<Result<(), ()>>>> = Mutex::new(RefCell::new(None));
static mut DMA_BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];

#[derive(Debug, PartialEq, Eq)]
enum FlashState {
    Init,
    Idle,
    Erasing,
    Writing,
}

pub struct Flash<SPI, CS> {
    state: FlashState,
    spi: Arc<Mutex<RefCell<SPI>>>,
    cs: CS,
    size: u32,
    pub pointer: u32,
    write_buffer: Vec<u8>,
}

#[derive(Debug)]
pub enum FlashError<E> {
    Spi(E),
    Busy
}

impl<E> From<E> for FlashError<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

fn on_interrupt() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::DMA1_STREAM4);

    free(|cs| {
        let transfer = &mut *DMA_TRANSFER.borrow(cs).borrow_mut();
        if let Some(ref mut transfer) = transfer.as_mut() {
            transfer.clear_fifo_error_interrupt();
            transfer.clear_transfer_complete_interrupt();
        }

        let result = Some(Ok(())); // TODO: properly track errors
        *DMA_RESULT.borrow(cs).borrow_mut() = result;
    })
}

#[cfg(feature = "rev1")]
#[interrupt]
fn DMA1_STREAM4() {
    on_interrupt();
}

#[cfg(feature = "rev2")]
#[interrupt]
fn DMA1_STREAM7() {
    on_interrupt();
}

impl<SPI: SpiBus, CS: OutputPin> Flash<SPI, CS> {
    pub fn init(
        spi: Arc<Mutex<RefCell<SPI>>>,
        cs: CS,
        dma_tx: DmaTx,
        dma_stream: DmaStream
    ) -> Self {
        let transfer = unsafe {
            Transfer::init_memory_to_peripheral(
                dma_stream,
                dma_tx,
                &mut DMA_BUFFER,
                None,
                DmaConfig::default()
                    .memory_increment(true)
                    .fifo_enable(true)
                    .fifo_error_interrupt(true)
                    .transfer_complete_interrupt(true),
            )
        };

        // Hand off transfer to interrupt handler
        free(|cs| *DMA_TRANSFER.borrow(cs).borrow_mut() = Some(transfer));

        let mut flash = Self {
            state: FlashState::Init,
            spi,
            cs,
            size: 0,
            pointer: 0,
            write_buffer: Vec::with_capacity((PAGE_SIZE * 2) as usize),
        };

        let ids = flash.command(W25OpCode::JedecId, &[], 3).unwrap_or([0; 3].to_vec());
        let (man_id, dev_id) = (ids[0], ((ids[1] as u16) << 8) + (ids[2] as u16));
        let (name, size_mbit) = match (man_id, dev_id) {
            (0xef, 0x4019) => ("Winbond W25Q256JV-IQ", 256),
            (0xef, 0x7019) => ("Winbond W25Q256JV-IM", 256),
            _ => ("unknown", 0),
        };

        if size_mbit > 0 {
            log!(Info, "Initialized flash ({}, {}Mb, 0x{:02x}, 0x{:04x}).", name, size_mbit, man_id, dev_id);
        } else {
            log!(Error, "Failed to connect to flash (0x{:02x}, 0x{:04x}).", man_id, dev_id);
        }

        flash.size = size_mbit * 1024 * 1024 / 8;
        flash
    }

    fn command(&mut self, opcode: W25OpCode, params: &[u8], response_len: usize) -> Result<Vec<u8>, FlashError<SPI::Error>> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

            self.cs.set_low().unwrap();
            let res = spi.transfer_in_place(&mut payload);
            self.cs.set_high().unwrap();
            res?;

            Ok(payload[(1 + params.len())..].to_vec())
        })
    }

    fn command_dma(&mut self, opcode: W25OpCode, params: &[u8]) -> () {
        if params.len() > DMA_BUFFER_SIZE - 1 {
            return;
        }

        self.cs.set_low().unwrap();

        free(|cs| {
            let msg = [&[opcode as u8], params].concat();

            let mut transfer = DMA_TRANSFER.borrow(cs).borrow_mut();
            if let Some(ref mut transfer) = transfer.as_mut() {
                unsafe {
                    DMA_BUFFER = msg.try_into().unwrap();
                    transfer.next_transfer(&mut DMA_BUFFER).unwrap();
                    transfer.start(|_tx| {});
                }
            }

            *DMA_RESULT.borrow(cs).borrow_mut() = None;

            unsafe {
                #[cfg(feature = "rev1")]
                NVIC::unmask(hal::pac::Interrupt::DMA1_STREAM4);
                #[cfg(feature = "rev2")]
                NVIC::unmask(hal::pac::Interrupt::DMA1_STREAM7);
            }
        })
    }

    fn is_busy(&mut self) -> bool {
        let mut response = self.command(W25OpCode::ReadStatusRegister1, &[], 1);
        // Same issue as in lora.rs. When combining DMA and regular SPI, the first
        // non-DMA command after a DMA transfer (which is always this one) needs 3
        // tries not to end with an overrun. Weirdly reproducible.
        for _i in 0..3 {
            if response.is_ok() {
                break;
            }
            response = self.command(W25OpCode::ReadStatusRegister1, &[], 1);
        }
        response.map(|resp| resp[0] & 0x01 > 0).unwrap_or(true)
    }

    fn read(&mut self, address: u32, len: u32) -> Result<Vec<u8>, FlashError<SPI::Error>> {
        if self.is_busy() {
            return Err(FlashError::Busy);
        }

        let address = address.to_be_bytes();
        self.command(W25OpCode::ReadData4BAddress, &address, len as usize)
    }

    fn write_dma(&mut self, address: usize, data: &[u8]) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy() {
            return Err(FlashError::Busy);
        }

        self.command(W25OpCode::WriteEnable, &[], 0)?;
        let cmd = [&address.to_be_bytes(), data].concat();
        self.command_dma(W25OpCode::PageProgram4BAddress, &cmd);
        Ok(())
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), FlashError<SPI::Error>> {
        self.command(W25OpCode::WriteEnable, &[], 0)?;
        let cmd = address.to_be_bytes();
        self.command(W25OpCode::SectorErase4KB4BAddress, &cmd, 0)?;
        Ok(())
    }

    fn flush_page(&mut self) -> Result<(), FlashError<SPI::Error>> {
        let data: Vec<u8> = self.write_buffer.drain(..((PAGE_SIZE - 3) as usize)).collect();

        if self.pointer >= FLASH_SIZE {
            log_every_nth_time!(1000, Error, "Flash storage full, skipping page flush.");
            return Ok(());
        }

        let crc = X25.checksum(&data);

        let mut page = Vec::with_capacity(PAGE_SIZE as usize);
        page.push(0x00);
        page.extend(data);
        page.push((crc >> 8) as u8);
        page.push(crc as u8);

        let result = self.write_dma(self.pointer as usize, &page);
        self.pointer += PAGE_SIZE;

        if result.is_ok() {
            self.state = FlashState::Writing;
        }

        result
    }

    pub fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), FlashError<SPI::Error>> {
        if self.state != FlashState::Idle {
            log_every_nth_time!(1000, Error, "Flash not idle.");
            return Ok(());
        }

        let serialized = msg.serialize().unwrap_or_default();
        if serialized.len() > (2 * PAGE_SIZE as usize) - self.write_buffer.len() {
            log!(Error, "Flash message too big.");
            return Ok(());
        }

        self.write_buffer.extend(serialized);
        if self.write_buffer.len() > (PAGE_SIZE - 3) as usize {
            self.flush_page()
        } else {
            Ok(())
        }
    }

    fn configure(&mut self) -> Result<(), FlashError<SPI::Error>> {
        // Determine first unwritten page by binary search
        let (mut a, mut b) = (0, self.size);
        while b - a > PAGE_SIZE {
            let mid = (a + b) / 2;
            if self.read(mid, 1)?[0] == 0xff {
                b = mid;
            } else {
                a = mid;
            }
        }
        self.pointer = u32::max(b, FLASH_HEADER_SIZE);
        log!(Info, "Determined Flash pointer: 0x{:02x?}", self.pointer);
        Ok(())
    }

    fn tick_erase(&mut self) {
        if self.pointer == FLASH_HEADER_SIZE {
            log!(Info, "Flash erase done, reinitializing.");
            self.state = FlashState::Init;
            self.write_buffer.truncate(0);
            return;
        }

        if self.is_busy() {
            return;
        }

        self.pointer -= self.pointer % SECTOR_SIZE;

        for address in (self.pointer..(self.pointer + SECTOR_SIZE)).step_by(PAGE_SIZE as usize) {
            let page_erased = self.read(address, 256)
                .map(|page_content| !page_content.iter().any(|b| *b != 0xff))
                .unwrap_or(false);

            if !page_erased {
                if let Err(e) = self.erase_sector(self.pointer) {
                    log!(Error, "Error erasing sector 0x{:x}: {:?}", self.pointer, e);
                }
                return;
            }
        }

        self.pointer -= SECTOR_SIZE;
    }

    fn tick_writing(&mut self) {
        let result = free(|cs| DMA_RESULT.borrow(cs).replace(None));
        if result.is_some() {
            self.state = FlashState::Idle;
            self.cs.set_high().unwrap();
        }
    }

    pub fn tick(&mut self, _time: u32, msg: Option<DownlinkMessage>) {
        match self.state {
            FlashState::Init => {
                if let Err(e) = self.configure() {
                    log!(Error, "Error configuring Flash memory: {:?}", e);
                } else {
                    self.state = FlashState::Idle;
                }
            }
            FlashState::Erasing => self.tick_erase(),
            FlashState::Writing => self.tick_writing(),
            FlashState::Idle => {}
        }

        if let Some(msg) = msg {
            if let Err(e) = self.write_message(msg) {
                log!(Error, "Failed to write message to flash: {:?}", e);
            }
        }
    }

    pub fn downlink(&mut self, usb_link: &mut UsbLink, address: u32, size: u32) {
        match self.read(address, size) {
            Ok(data) => {
                let msg = DownlinkMessage::FlashContent(address as u32, data);
                usb_link.send_message(msg);
            }
            Err(e) => log!(Error, "Failed to read flash: {:?}", e),
        }
    }

    pub fn erase(&mut self) {
        log!(Info, "Erasing flash.");
        self.state = FlashState::Erasing;
        self.pointer = self.size - 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum W25OpCode {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,

    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,

    ReleasePowerDown = 0xab,
    ManufacturerDeviceId = 0x90,
    JedecId = 0x9f,
    ReadUniqueId = 0x48,

    ReadData = 0x03,
    ReadData4BAddress = 0x13,
    FastRead = 0x0b,
    FastRead4BAddress = 0x0c,

    PageProgram = 0x02,
    PageProgram4BAddress = 0x12,

    SectorErase4KB = 0x20,
    SectorErase4KB4BAddress = 0x21,
    BlockErase32KB = 0x52,
    BlockErase64KB = 0xd8,
    BlockErase64KB4BAddress = 0xdc,
    ChipErase = 0xc7,
}
