use alloc::vec::Vec;

use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI2;
use stm32f4xx_hal as hal;

use crc::{Crc, CRC_16_IBM_SDLC};

use crate::prelude::*;
use crate::telemetry::*;
use crate::usb::*;

type CsPin = Pin<'B', 12, Output>;
type SclPin = Pin<'B', 13, Alternate<5>>;
type MisoPin = Pin<'C', 2, Alternate<5>>;
type MosiPin = Pin<'C', 3, Alternate<5>>;
type Spi = hal::spi::Spi<SPI2, (SclPin, MisoPin, MosiPin)>;

const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);
const PAGE_SIZE: u32 = 256;
const SECTOR_SIZE: u32 = 4096;

#[derive(Debug, PartialEq, Eq)]
enum FlashState {
    Init,
    Idle,
    Erasing
}

pub struct Flash {
    state: FlashState,
    spi: Spi,
    cs: CsPin,
    size: u32,
    pub pointer: u32,
    write_buffer: Vec<u8>,
}

impl Flash {
    pub fn init(spi: Spi, cs: CsPin) -> Self {
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

    fn command(&mut self, opcode: W25OpCode, params: &[u8], response_len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        free(|_cs| {
            let mut msg = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

            self.cs.set_low();
            let response = self.spi.transfer(&mut msg);
            self.cs.set_high();

            Ok(response?[(1 + params.len())..].to_vec())
        })
    }

    fn is_busy(&mut self) -> bool {
        let response = self.command(W25OpCode::ReadStatusRegister1, &[], 1);
        response.map(|resp| resp[0] & 0x01 > 0).unwrap_or(true)
    }

    fn read(&mut self, address: u32, len: u32) -> Result<Vec<u8>, hal::spi::Error> {
        if self.is_busy() {
            return Err(hal::spi::Error::Overrun);
        }

        let address = address.to_be_bytes();
        self.command(W25OpCode::ReadData4BAddress, &address, len as usize)
    }

    fn write(&mut self, address: usize, data: &[u8]) -> Result<(), hal::spi::Error> {
        if self.is_busy() {
            return Err(hal::spi::Error::Overrun);
        }

        self.command(W25OpCode::WriteEnable, &[], 0)?;
        let cmd = [&address.to_be_bytes(), data].concat();
        self.command(W25OpCode::PageProgram4BAddress, &cmd, 0)?;
        Ok(())
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), hal::spi::Error> {
        self.command(W25OpCode::WriteEnable, &[], 0)?;
        let cmd = address.to_be_bytes();
        self.command(W25OpCode::SectorErase4KB4BAddress, &cmd, 0)?;
        Ok(())
    }

    fn flush_page(&mut self) -> Result<(), hal::spi::Error> {
        let data: Vec<u8> = self.write_buffer.drain(..((PAGE_SIZE-3) as usize)).collect();

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

        let result = self.write(self.pointer as usize, &page);
        self.pointer += PAGE_SIZE;
        result
    }

    pub fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), hal::spi::Error> {
        if self.state != FlashState::Idle {
            log!(Error, "Flash not idle.");
            return Ok(());
        }

        let serialized = msg.wrap();
        if serialized.len() > (2*PAGE_SIZE as usize) - self.write_buffer.len() {
            log!(Error, "Flash message too big.");
            return Ok(());
        }

        self.write_buffer.extend(serialized);
        if self.write_buffer.len() > (PAGE_SIZE-3) as usize {
            self.flush_page()
        } else {
            Ok(())
        }
    }

    fn configure(&mut self) -> Result<(), hal::spi::Error> {
        // Determine first unwritten page by binary search
        let (mut a, mut b) = (0, self.size);
        while b - a > PAGE_SIZE {
            let mid = (a+b)/2;
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
        let first_byte = self.read(self.pointer, 1)
            .map(|resp| resp[0])
            .unwrap_or(0x00);

        if first_byte != 0xff {
            if let Err(e) = self.erase_sector(self.pointer) {
                log!(Error, "Error erasing sector 0x{:x}: {:?}", self.pointer, e);
            }
        } else {
            self.pointer -= SECTOR_SIZE;
        }
    }

    pub fn tick(&mut self, _time: u32, msg: Option<DownlinkMessage>) {
        if self.state == FlashState::Init {
            if let Err(e) = self.configure() {
                log!(Error, "Error configuring Flash memory: {:?}", e);
            } else {
                self.state = FlashState::Idle;
            }
        }

        if self.state == FlashState::Erasing {
            self.tick_erase();
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
            },
            Err(e) => log!(Error, "Failed to read flash: {:?}", e)
        }
    }

    pub fn erase(&mut self) {
        log!(Info, "Erasing flash.");
        self.state = FlashState::Erasing;
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
