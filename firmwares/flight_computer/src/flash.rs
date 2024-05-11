//! Flash storage implementation
//!
//! The first sector (4KiB) of the flash memory is reserved for storing settings, the rest is used
//! for telemetry messages. Telemetry messages are buffered and written to memory in pages (256B).
//!
//! For reading, the flash implementation holds its own handle to the USB connection, which allows
//! faster reading of flash.

use alloc::vec;
use alloc::vec::Vec;

use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SpiDeviceImpl;
use embassy_sync::signal::Signal;
use embedded_hal_async::spi::SpiDevice;

use crc::{Crc, CRC_16_IBM_SDLC};
use static_cell::make_static;

use defmt::*;

use shared_types::*;

use crate::drivers::flash::W25Q;
use crate::usb::FlashUsbHandle;

const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

const PAGE_SIZE: usize = 256;
const SECTOR_SIZE: u32 = 4096;

/// Signal for sending flash pointer to flash handle, in order to pass it on via telemetry. There
/// is probably a better way to do this.
static FLASH_POINTER_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

/// Request sent to background flash task.
enum FlashRequest {
    WriteMessage(DownlinkMessage),
    WriteSettings(Settings),
    Read(u32, u32),
    Erase,
}

/// Main flash struct. This is moved to a background task and handles interaction with the physical
/// flash chip.
pub struct Flash<SPI> {
    request_receiver: Receiver<'static, CriticalSectionRawMutex, FlashRequest, 3>,
    driver: W25Q<SPI>,
    usb: FlashUsbHandle,
    pointer: u32,
    write_buffer: Vec<u8>,
}

/// Flash handle returned by initialization and used by the rest of the firmware to interact with
/// the flash.
pub struct FlashHandle {
    request_sender: Sender<'static, CriticalSectionRawMutex, FlashRequest, 3>,
    pub pointer: u32,
}

#[derive(Debug)]
pub enum FlashError<E> {
    Spi(E),
    Serialization(postcard::Error),
    Busy,
    Crc,
}

impl<E: Sized> From<E> for FlashError<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

// Embassy tasks cannot be generic for some reason, so for now we have to have these ugly type
// signatures and a task outside of the struct here.
type SpiInst = Spi<'static, SPI3, DMA1_CH7, DMA1_CH0>;
type FlashInst = Flash<SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PD2>>>;

#[embassy_executor::task]
pub async fn run(mut flash: FlashInst) -> ! {
    flash.run().await
}

impl FlashHandle {
    pub async fn tick(&mut self) {
        if FLASH_POINTER_SIGNAL.signaled() {
            self.pointer = FLASH_POINTER_SIGNAL.wait().await;
        }
    }

    pub fn write_settings(&mut self, settings: Settings) -> Result<(), ()> {
        self.request_sender.try_send(FlashRequest::WriteSettings(settings)).map_err(|_e| ())
    }

    pub fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), ()> {
        self.request_sender.try_send(FlashRequest::WriteMessage(msg)).map_err(|_e| ())
    }

    pub fn read(&mut self, address: u32, size: u32) -> Result<(), ()> {
        self.request_sender.try_send(FlashRequest::Read(address, size)).map_err(|_e| ())
    }

    pub fn erase(&mut self) -> Result<(), ()>{
        self.request_sender.try_send(FlashRequest::Erase).map_err(|_e| ())
    }
}

impl<SPI: SpiDevice> Flash<SPI> {
    pub async fn init(spi: SPI, usb: FlashUsbHandle) -> Result<(Self, FlashHandle, Settings), FlashError<SPI::Error>> {
        let request_channel = make_static!(Channel::<CriticalSectionRawMutex, FlashRequest, 3>::new());

        let driver = W25Q::init(spi).await?;

        let mut flash = Self {
            request_receiver: request_channel.receiver(),
            driver,
            usb,
            pointer: 0,
            write_buffer: Vec::with_capacity((PAGE_SIZE * 2) as usize),
        };

        flash.determine_pointer().await?;

        let mut retries = 2;
        let settings = loop {
            match flash.read_settings().await {
                Ok(settings) => break settings,
                Err(e) if retries > 0 => {
                    error!("Failed to read settings from flash ({:?}), retrying {} more times...", Debug2Format(&e), retries);
                    retries -= 1;
                }
                Err(e) => {
                    error!("Failed to read settings from flash ({:?}), reverting to defaults.", Debug2Format(&e));
                    break Settings::default();
                }
            }
        };

        let flash_handle = FlashHandle {
            request_sender: request_channel.sender(),
            pointer: flash.pointer,
        };

        Ok((flash, flash_handle, settings))
    }

    async fn determine_pointer(&mut self) -> Result<(), FlashError<SPI::Error>> {
        // Determine first unwritten page by binary search
        let (mut a, mut b) = (FLASH_HEADER_SIZE, self.driver.size());
        while b - a > PAGE_SIZE as u32 {
            let mid = (a + b) / 2;
            if self.driver.read(mid, 1).await?[0] == 0xff {
                b = mid;
            } else {
                a = mid;
            }
        }

        self.pointer = u32::max(b, FLASH_HEADER_SIZE);
        info!("Determined Flash pointer: {:?}", self.pointer);

        Ok(())
    }

    fn update_pointer(&mut self, pointer: u32) {
        self.pointer = pointer;
        FLASH_POINTER_SIGNAL.signal(self.pointer);
    }

    async fn flush_page(&mut self) -> Result<(), FlashError<SPI::Error>> {
        let data: Vec<u8> = self.write_buffer.drain(..(PAGE_SIZE - 3)).collect();

        // We're full, do nothing
        if self.pointer >= FLASH_SIZE {
            return Ok(());
        }

        let crc = X25.checksum(&data);

        let mut page = Vec::with_capacity(PAGE_SIZE);
        page.push(0x00);
        page.extend(data);
        page.push((crc >> 8) as u8);
        page.push(crc as u8);

        let result = self.driver.write(self.pointer as usize, &page).await;
        self.update_pointer(self.pointer + PAGE_SIZE as u32);

        result
    }

    pub async fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), FlashError<SPI::Error>> {
        let serialized = msg.serialize().unwrap_or_default();
        if serialized.len() > 2 * PAGE_SIZE - self.write_buffer.len() {
            error!("Flash message too big.");
            return Ok(());
        }

        self.write_buffer.extend(serialized);
        if self.write_buffer.len() > PAGE_SIZE - 3 {
            self.flush_page().await
        } else {
            Ok(())
        }
    }

    pub async fn read_settings(&mut self) -> Result<Settings, FlashError<SPI::Error>> {
        let page = self.driver.read(0x00, FLASH_HEADER_SIZE).await?;

        let (settings, crc) = page.split_at(FLASH_HEADER_SIZE as usize - 2);
        let crc = u16::from_be_bytes([crc[0], crc[1]]);

        if crc != X25.checksum(&settings) {
            return Err(FlashError::Crc);
        }

        Ok(postcard::from_bytes(&settings).map_err(|e| FlashError::Serialization(e))?)
    }

    async fn write_settings(&mut self, settings: &Settings) -> Result<(), FlashError<SPI::Error>> {
        // TODO: pretty this up a bit, especially get rid of the is_busy loops

        let mut buf: [u8; 1024] = [0x00; 1024];
        let serialized = postcard::to_slice(settings, &mut buf).unwrap();

        let mut sector: [u8; FLASH_HEADER_SIZE as usize] = [0x00; FLASH_HEADER_SIZE as usize];
        sector[..serialized.len()].copy_from_slice(serialized);

        let crc = X25.checksum(&sector[..(FLASH_HEADER_SIZE as usize - 2)]);
        sector[FLASH_HEADER_SIZE as usize - 2] = (crc >> 8) as u8;
        sector[FLASH_HEADER_SIZE as usize - 1] = crc as u8;

        let mut res = Ok(());
        for _i in 0..3 {
            res = self.driver.erase_sector(0x00).await;
            if res.is_ok() {
                break;
            }
        }
        res?;

        // Wait for flash to finish erasing before writing
        for _i in 0..2000 {
            if !self.driver.is_busy().await {
                break;
            }
        }

        // We can only write a single page at a time
        for i in 0..(FLASH_HEADER_SIZE as usize / PAGE_SIZE) {
            self.driver.write(PAGE_SIZE * i, &sector[(PAGE_SIZE * i)..(PAGE_SIZE * (i+1))]).await?;

            // Wait for flash to finish writing
            for _i in 0..100 {
                if !self.driver.is_busy().await {
                    break;
                }
            }
        }

        // Read back what we have written to make sure the changes are persisted.
        if &self.read_settings().await? != settings {
            Err(FlashError::Crc)
        } else {
            Ok(())
        }
    }

    async fn erase(&mut self) {
        self.update_pointer(self.driver.size() - SECTOR_SIZE);

        loop {
            debug!("{:?}", self.pointer);

            if self.pointer == FLASH_HEADER_SIZE {
                info!("Flash erase done, reinitializing.");
                self.write_buffer.truncate(0);
                break;
            }

            if self.driver.is_busy().await {
                debug!("busy");
                continue;
            }

            let next_pointer = self.pointer - SECTOR_SIZE;

            let mut sector_needs_erasing = false;
            for address in (next_pointer..(next_pointer + SECTOR_SIZE)).step_by(PAGE_SIZE) {
                let content = self.driver.read(address, 256).await.unwrap_or(vec![0x00]);
                if content.iter().any(|b| *b != 0xff) {
                    sector_needs_erasing = true;
                    break;
                }
            }

            if sector_needs_erasing {
                if let Err(e) = self.driver.erase_sector(next_pointer).await {
                    error!("Error erasing sector {}: {:?}", next_pointer, Debug2Format(&e));
                } else {
                    self.update_pointer(next_pointer);
                }
            } else {
                self.update_pointer(next_pointer);
            }
        }
    }

    async fn run(&mut self) -> ! {
        loop {
            let request = self.request_receiver.receive().await;
            match request {
                FlashRequest::WriteMessage(msg) => {
                    if let Err(e) = self.write_message(msg).await {
                        error!("Failed to write message to flash ({:?}), discarding.", Debug2Format(&e));
                    }
                },
                FlashRequest::WriteSettings(settings) => {
                    for _i in 0..10 {
                        if self.write_settings(&settings).await.is_ok() {
                            break;
                        }
                    }

                    // reboot to apply settings
                    cortex_m::peripheral::SCB::sys_reset();
                },
                FlashRequest::Read(address, size) => {
                    match self.driver.read(address, size).await {
                        Ok(data) => {
                            // split up the data into smaller chunks for USB transfer
                            const CHUNK_SIZE: usize = 256;
                            for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
                                let msg = DownlinkMessage::FlashContent(address + (i * CHUNK_SIZE) as u32, chunk.to_vec());
                                self.usb.send_message(msg).await;
                            }
                        },
                        Err(e) => {
                            error!("Error reading from flash: {:?}", Debug2Format(&e));
                        }
                    }
                },
                FlashRequest::Erase => {
                    info!("Erasing flash.");
                    self.erase().await
                }
            }
        }
    }
}
