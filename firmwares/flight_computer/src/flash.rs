use alloc::vec::Vec;

use stm32f4xx_hal as hal;
use hal::pac::SPI2;
use hal::gpio::{Alternate, Output, Pin};
use cortex_m::interrupt::free;
use embedded_hal::prelude::*;

use crate::prelude::*;

type CsPin = Pin<'B', 12, Output>;
type SclPin = Pin<'B', 13, Alternate<5>>;
type MisoPin = Pin<'C', 2, Alternate<5>>;
type MosiPin = Pin<'C', 3, Alternate<5>>;
type Spi = hal::spi::Spi<SPI2, (SclPin, MisoPin, MosiPin)>;

pub struct Flash {
    spi: Spi,
    cs: CsPin,
    size: usize
}

impl Flash {
    pub fn init(spi: Spi, cs: CsPin) -> Self {
        let mut flash = Self {
            spi,
            cs,
            size: 0
        };

        let ids = flash.command(W25OpCode::JedecId, &[], 3).unwrap_or([0; 3].to_vec());
        let (man_id, dev_id) = (ids[0], ((ids[1] as u16) << 8) + (ids[2] as u16));
        let (name, size_mbit) = match (man_id, dev_id) {
            (0xef, 0x4019) => ("Winbond W25Q256JV-IQ", 256),
            (0xef, 0x7019) => ("Winbond W25Q256JV-IM", 256),
            _ => ("unknown", 0)
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

    pub fn tick(&mut self, _time: u32, _data: Option<bool>) {
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
}
