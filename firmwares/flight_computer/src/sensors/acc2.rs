use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::pac::SPI3;
use hal::gpio::{Pin, Alternate, Output};
use hal::spi::{TransferModeNormal, Master};

type Spi = hal::spi::Spi<SPI3, (
    Pin<'C', 10, Alternate<6>>,
    Pin<'C', 11, Alternate<6>>,
    Pin<'C', 12, Alternate<6>>,
), TransferModeNormal, Master>;
type CsPin = hal::gpio::Pin<'D', 2, Output>;

pub struct Accelerometer {
    spi: Spi,
    cs: CsPin
}

impl Accelerometer {
    pub fn init(spi: Spi, cs: CsPin) -> Self {
        Self {
            spi,
            cs
        }
    }

    fn read_u8(&mut self, address: u8) -> Result<u8, hal::spi::Error> {
        let mut payload = [address | 0x80, 0];

        self.cs.set_low();
        let response = self.spi.transfer(&mut payload);
        self.cs.set_high();

        //log_every_nth_time!(100, Info, "response: {:?}", response);

        Ok(response?[1])
    }

    pub fn tick(&mut self) {
        self.read_u8(0x00);
    }
}
