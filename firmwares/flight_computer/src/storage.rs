//! Flash storage implementation
//!
//! The first sector (4KiB) of the flash memory is reserved for storing settings, the rest is used
//! for telemetry messages. Telemetry messages are buffered and written to memory in pages (256B).
//!
//! For reading, the flash implementation holds its own handle to the USB connection, which allows
//! faster reading of flash.

use heapless::Vec;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SpiDeviceImpl;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embassy_stm32::{Peri, peripherals::*};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use embedded_hal_async::spi::SpiDevice;
use embedded_storage_async::nor_flash::NorFlash;

use crc::{CRC_16_IBM_SDLC, Crc};
use static_cell::StaticCell;

use defmt::*;

use shared_types::*;

use crate::drivers::flash::{FlashError, W25Q128};
// use crate::usb::FlashUsbHandle;

const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

const PAGE_SIZE: usize = 256;
const BUFFER_SIZE: usize = PAGE_SIZE * 2;
const SECTOR_SIZE: u32 = 4096;

static REQUEST_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, FlashRequest, 3>> = StaticCell::new();

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
    driver: W25Q128<SPI>,
    // usb: FlashUsbHandle,
    pointer: u32,
    write_buffer: Vec<u8, BUFFER_SIZE>,
}

/// Flash handle returned by initialization and used by the rest of the firmware to interact with
/// the flash.
pub struct FlashHandle {
    request_sender: Sender<'static, CriticalSectionRawMutex, FlashRequest, 3>,
    pub pointer: u32,
}

#[derive(Debug)]
#[allow(dead_code)]
pub enum StorageError<E> {
    Spi(E),
    Init,
    Serialization(postcard::Error),
    Busy,
    Crc,
    Other,
}

impl<E: Sized> From<E> for StorageError<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

impl<E: Sized> From<FlashError<E>> for StorageError<E> {
    fn from(value: FlashError<E>) -> Self {
        match value {
            FlashError::Init => StorageError::Init,
            FlashError::Busy => StorageError::Busy,
            FlashError::Spi(e) => StorageError::Spi(e),
            FlashError::NotAligned => StorageError::Other,
            FlashError::OutOfBounds => StorageError::Other,
            FlashError::MultipleCommandsRequired => StorageError::Other,
        }
    }
}

// Embassy tasks cannot be generic for some reason, so for now we have to have these ugly type
// signatures and a task outside of the struct here.
// type SpiBusType = Spi<'static, Peri<'static, SPI3>>;
// type FlashInst = Flash<SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PD2>>>;
// type FlashInst = Flash<SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiBusType, Output<'static>>>;

// Note: CriticalSectionRawMutex vs NoopRawMutex
pub type FlashSpiBusType = Spi<'static, embassy_stm32::mode::Async>;
pub type FlashSpiDeviceType = SpiDeviceImpl<'static, CriticalSectionRawMutex, FlashSpiBusType, Output<'static>>;
pub type FlashType = Flash<FlashSpiDeviceType>;

#[embassy_executor::task]
pub async fn run(mut flash: FlashType) -> ! {
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

    pub fn erase(&mut self) -> Result<(), ()> {
        self.request_sender.try_send(FlashRequest::Erase).map_err(|_e| ())
    }
}

impl<SPI: SpiDevice> Flash<SPI> {
    pub async fn init(
        spi: SPI,
        // usb: FlashUsbHandle,
    ) -> Result<(Self, FlashHandle, Settings), StorageError<SPI::Error>> {
        let request_channel = REQUEST_CHANNEL.init(Channel::new());

        let driver = W25Q128::new(spi).await?;

        let mut flash = Self {
            request_receiver: request_channel.receiver(),
            driver,
            // usb,
            pointer: 0,
            write_buffer: Vec::new(),
        };

        flash.determine_pointer().await?;

        let mut retries = 2;
        let settings = loop {
            match flash.read_settings().await {
                Ok(settings) => break settings,
                Err(_e) if retries > 0 => {
                    retries -= 1;
                }
                Err(e) => {
                    // info!("writing settings now");
                    // flash.write_settings(&Settings::default()).await?;
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

    async fn determine_pointer(&mut self) -> Result<(), StorageError<SPI::Error>> {
        // Determine first unwritten page by binary search
        let (mut a, mut b) = (FLASH_HEADER_SIZE, self.driver.capacity() as u32);
        while b - a > 2 * PAGE_SIZE as u32 {
            let mid = (a + b) / 2;
            let mid = mid - (mid % PAGE_SIZE as u32);
            let mut buf: [u8; 1] = [0];
            self.driver.read(mid, &mut buf).await?;
            if buf[0] == 0xff {
                b = mid;
            } else {
                a = mid;
            }
        }

        self.pointer = u32::max(b, FLASH_HEADER_SIZE);
        defmt::info!("Flash pointer: 0x{:02x}", self.pointer);
        Ok(())
    }

    fn update_pointer(&mut self, pointer: u32) {
        self.pointer = pointer;
        FLASH_POINTER_SIGNAL.signal(self.pointer);
    }

    async fn flush_page(&mut self) -> Result<(), StorageError<SPI::Error>> {
        // We're full, do nothing
        if self.pointer >= FLASH_SIZE {
            return Ok(());
        }

        let result = {
            let data = &self.write_buffer[..(PAGE_SIZE - 3)];
            let crc = X25.checksum(data);

            let mut page = [0x00; PAGE_SIZE];
            page[1..PAGE_SIZE - 2].copy_from_slice(data);
            page[PAGE_SIZE - 2] = (crc >> 8) as u8;
            page[PAGE_SIZE - 1] = crc as u8;

            const CHUNK_SIZE: usize = 32;

            let mut result = Ok(());
            for i in 0..(PAGE_SIZE / CHUNK_SIZE) {
                let chunk = &page[(i * CHUNK_SIZE)..((i + 1) * CHUNK_SIZE)];

                for attempt in 0..3 {
                    result = self.driver.write(self.pointer + ((i * CHUNK_SIZE) as u32), chunk).await;
                    if result.is_err() {
                        continue;
                    }

                    let mut read_back: [u8; CHUNK_SIZE] = [0; CHUNK_SIZE];
                    let read_successful =
                        self.driver.read(self.pointer + (i * CHUNK_SIZE) as u32, &mut read_back).await;
                    if read_successful.is_ok() && read_back == chunk {
                        break;
                    }

                    Timer::after(Duration::from_micros(100)).await;

                    defmt::warn!(
                        "Chunk write 0x{:02x}+0x{} failed. ({}/{})",
                        self.pointer,
                        i * CHUNK_SIZE,
                        attempt + 1,
                        3
                    );
                }

                if result.is_err() {
                    break;
                }
            }

            self.update_pointer(self.pointer + PAGE_SIZE as u32);
            result
        };

        let mut new_buffer: Vec<u8, BUFFER_SIZE> = Vec::new();
        let _ = new_buffer.extend_from_slice(&self.write_buffer[(PAGE_SIZE - 3)..]);
        self.write_buffer = new_buffer;

        result?;
        Ok(())
    }

    pub async fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), StorageError<SPI::Error>> {
        let serialized = msg.serialize().unwrap_or_default();
        if serialized.len() > 2 * PAGE_SIZE - self.write_buffer.len() {
            //error!("Flash message too big.");
            return Ok(());
        }

        self.write_buffer.extend(serialized);
        if self.write_buffer.len() > PAGE_SIZE - 3 {
            self.flush_page().await
        } else {
            Ok(())
        }
    }

    pub async fn read_settings(&mut self) -> Result<Settings, StorageError<SPI::Error>> {
        const DATA_SIZE: usize = FLASH_SETTINGS_SIZE as usize;
        let mut settings_data: Vec<u8, DATA_SIZE> = Vec::new();
        for i in 0..(DATA_SIZE / 256) {
            let mut page: [u8; PAGE_SIZE] = [0; PAGE_SIZE];
            self.driver.read((i * 256) as u32, &mut page).await?;
            settings_data.extend(page);
        }

        let (settings, crc) = settings_data.split_at(FLASH_SETTINGS_SIZE as usize - 2);
        let crc = u16::from_be_bytes([crc[0], crc[1]]);

        if crc != X25.checksum(settings) {
            return Err(StorageError::Crc);
        }

        postcard::from_bytes(settings).map_err(StorageError::Serialization)
    }

    async fn write_settings(&mut self, settings: &Settings) -> Result<(), StorageError<SPI::Error>> {
        info!("write_settings");
        let mut res = Ok(());
        for _i in 0..3 {
            res = self.driver.erase_sector(0x00).await;
            if res.is_ok() {
                break;
            }
        }
        res?;
        info!("erase started");

        // Wait for flash to finish erasing before writing
        // TODO: change
        for _i in 0..10 {
            if !self.driver.is_busy().await? {
                break;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        info!("erase completed");

        // separate scope so we get rid of these buffers before reading back
        {
            let mut buffer = [0u8; 512];
            let serialized = postcard::to_slice(settings, &mut buffer).unwrap();

            let mut sector: [u8; FLASH_SETTINGS_SIZE as usize] = [0x00; FLASH_SETTINGS_SIZE as usize];
            sector[..serialized.len()].copy_from_slice(serialized);

            let crc = X25.checksum(&sector[..(FLASH_SETTINGS_SIZE as usize - 2)]);
            sector[FLASH_SETTINGS_SIZE as usize - 2] = (crc >> 8) as u8;
            sector[FLASH_SETTINGS_SIZE as usize - 1] = crc as u8;

            // We can only write a single page at a time
            for i in 0..(FLASH_SETTINGS_SIZE as usize / PAGE_SIZE) {
                // TODO: fix flash busy timing
                for _i in 0..100 {
                    if !self.driver.is_busy().await? {
                        break;
                    }
                    info!("driver busy check (storage)");
                    Timer::after(Duration::from_millis(1)).await;
                }
                info!("driver write 1");
                self.driver.write((PAGE_SIZE * i) as u32, &sector[(PAGE_SIZE * i)..(PAGE_SIZE * (i + 1))]).await?;
                info!("driver write 2");

                // Wait for flash to finish writing
                // TODO: change
                for _i in 0..10 {
                    if !self.driver.is_busy().await? {
                        break;
                    }
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
        }
        info!("written settings");

        // Read back what we have written to make sure the changes are persisted.
        if &self.read_settings().await? != settings {
            Err(StorageError::Crc)
        } else {
            Ok(())
        }
    }

    async fn erase(&mut self) {
        self.update_pointer(self.driver.capacity() as u32 - SECTOR_SIZE);

        loop {
            if self.pointer == FLASH_HEADER_SIZE {
                info!("Flash erase done, reinitializing.");
                self.write_buffer.truncate(0);
                break;
            }

            if self.driver.is_busy().await.unwrap_or(true) {
                continue;
            }

            let next_pointer = self.pointer - SECTOR_SIZE;

            let mut sector_needs_erasing = false;
            for address in (next_pointer..(next_pointer + SECTOR_SIZE)).step_by(PAGE_SIZE) {
                // TODO
                let mut content: [u8; PAGE_SIZE] = [0; PAGE_SIZE];
                let success = self.driver.read(address, &mut content).await.ok().is_some();
                if !success || content.into_iter().any(|b| b != 0xff) {
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
                        error!("Failed to write flash msg: {:?}", Debug2Format(&e));
                    }
                }
                FlashRequest::WriteSettings(settings) => {
                    for _i in 0..10 {
                        if self.write_settings(&settings).await.is_ok() {
                            break;
                        }
                    }

                    // reboot to apply settings
                    cortex_m::peripheral::SCB::sys_reset();
                }
                // TODO: think about size limit
                FlashRequest::Read(address, size) => {
                    let size: u32 = { core::cmp::min(size, 256) };
                    let mut content: Vec<u8, 256> = Vec::new();
                    let _ = content.resize(size as usize, 0);

                    match self.driver.read(address, content.as_mut_slice()).await {
                        Ok(_) => {
                            // split up the data into smaller chunks for USB transfer
                            const CHUNK_SIZE: usize = 256;
                            for (i, chunk) in content.chunks(CHUNK_SIZE).enumerate() {
                                let _msg = DownlinkMessage::FlashContent(
                                    address + (i * CHUNK_SIZE) as u32,
                                    Vec::from_slice(chunk).unwrap_or_default(),
                                );
                                // TODO: implement
                                // self.usb.send_message(msg).await;
                            }
                        }
                        Err(e) => {
                            error!("Failed to read flash: {:?}", Debug2Format(&e));
                        }
                    }
                }
                FlashRequest::Erase => {
                    info!("Erasing flash.");
                    self.erase().await
                }
            }
        }
    }
}
