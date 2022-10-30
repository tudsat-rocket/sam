use alloc::vec::Vec;

use cortex_m::interrupt::free;
use embedded_hal::prelude::*;
use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI2;
use stm32f4xx_hal as hal;

use crate::prelude::*;
use crate::telemetry::*;

type CsPin = Pin<'B', 12, Output>;
type SclPin = Pin<'B', 13, Alternate<5>>;
type MisoPin = Pin<'C', 2, Alternate<5>>;
type MosiPin = Pin<'C', 3, Alternate<5>>;
type Spi = hal::spi::Spi<SPI2, (SclPin, MisoPin, MosiPin)>;

const PAGE_SIZE: usize = 256;
const RESERVED_SIZE: usize = 4096;

#[derive(Debug, PartialEq, Eq)]
enum FlashState {
    Init,
    Idle
}

pub struct Flash {
    state: FlashState,
    spi: Spi,
    cs: CsPin,
    size: usize,
    pointer: usize,
    write_buffer: [u8; PAGE_SIZE*2],
    write_buffer_pointer: usize
}

impl Flash {
    pub fn init(spi: Spi, cs: CsPin) -> Self {
        let mut flash = Self {
            state: FlashState::Init,
            spi,
            cs,
            size: 0,
            pointer: 0,
            write_buffer: [0; PAGE_SIZE*2],
            write_buffer_pointer: 0
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

    fn read(&mut self, address: usize, len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        // TODO: check if busy
        let address = address.to_be_bytes();
        self.command(W25OpCode::ReadData4BAddress, &address, len)
    }

    fn write(&mut self, address: usize, data: &[u8]) -> Result<(), hal::spi::Error> {
        self.command(W25OpCode::WriteEnable, &[], 0)?;
        let cmd = [&address.to_be_bytes(), data].concat();
        self.command(W25OpCode::PageProgram4BAddress, &cmd, 0)?;
        Ok(())
    }

    fn write_message(&mut self, msg: DownlinkMessage) -> Result<(), hal::spi::Error> {
        let serialized = msg.wrap();
        if serialized.len() > (self.write_buffer.len() - self.write_buffer_pointer) {
            return Ok(());
        }

        // TODO
        for i in 0..serialized.len() {
            self.write_buffer[self.write_buffer_pointer + i] = serialized[i];
        }
        self.write_buffer_pointer += serialized.len();

        if self.write_buffer_pointer > (PAGE_SIZE-1) {
            self.pointer += PAGE_SIZE;
            let page = [&[0x0a], &self.write_buffer[..(PAGE_SIZE-1)]].concat();
            self.write_buffer.rotate_left(PAGE_SIZE-1);
            self.write_buffer_pointer -= PAGE_SIZE-1;
            self.write(self.pointer, &page)?;
        }

        Ok(())
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
        self.pointer = b;
        log!(Info, "Determined Flash pointer: 0x{:02x?}", self.pointer);
        Ok(())
    }

    pub fn tick(&mut self, _time: u32, _data: Option<bool>) {
        if self.state == FlashState::Init {
            if let Err(e) = self.configure() {
                log!(Error, "Error configuring Flash memory: {:?}", e);
            } else {
                self.state = FlashState::Idle;
            }
        }
        // TODO
    }


}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum W25OpCode {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,

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
