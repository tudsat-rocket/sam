// https://www.lcsc.com/datasheet/lcsc_datasheet_2407240928_Bosch-Sensortec-BMP580_C22391138.pdf

use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use num_traits::float::Float;

use defmt::*;

pub struct BMP580<SPI: SpiDevice<u8>> {
    spi: SPI,
    pressure: Option<f32>,
    temperature: Option<f32>,
}

impl<SPI: SpiDevice<u8>> BMP580<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut baro = Self {
            spi,
            pressure: None,
            temperature: None,
        };

        let mut whoami = 0x00;
        for _i in 0..10 {
            whoami = baro.read_u8(BMP580Register::WhoAmI).await?;
            if whoami == 0x50 {
                break
            }

            Timer::after(Duration::from_micros(100)).await;
        }

        if whoami != 0x50 {
            error!("Failed to initialize BMP580 (0x{:02x} != 0x50)", whoami);
        } else {
            info!("BMP580 initialized");
        }

        baro.configure().await?;

        Ok(baro)
    }

    async fn read_u8(&mut self, address: BMP580Register) -> Result<u8, SPI::Error> {
        let mut payload = [address as u8 | 0x80, 0]; // TODO
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(payload[1])
    }

    async fn write_u8(&mut self, address: BMP580Register, value: u8) -> Result<(), SPI::Error> {
        let mut payload = [address as u8, value];
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(())
    }

    async fn configure(&mut self) -> Result<(), SPI::Error> {
        self.write_u8(BMP580Register::OsrConfig, 0b01000000).await?;
        self.write_u8(BMP580Register::OdrConfig, 0b00000001).await?;
        //self.write_u8(BMP580Register::CtrlReg2, 0b00010000).await?;
        Ok(())
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        // TODO: limit to ODR

        let mut payload = [(BMP580Register::TempXl as u8) | 0x80, 0, 0, 0, 0, 0, 0];
        self.spi.transfer_in_place(&mut payload).await?;

        let press = ((payload[6] as u32) << 16) + ((payload[5] as u32) << 8) + (payload[4] as u32);
        // using i32 because of 2s complement, divided by 2^8 later
        let temp = ((payload[3] as i32) << 24) + ((payload[2] as i32) << 16) + ((payload[1] as i32) << 8);

        self.pressure = Some(press as f32 / 6400.0);
        self.temperature = Some(temp as f32 / 2.0.powi(24));

        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(_) = self.read_sensor_data().await {
            self.pressure = None;
            self.temperature = None;
        }
    }

    pub fn pressure(&self) -> Option<f32> {
        self.pressure
    }

    pub fn temperature(&self) -> Option<f32> {
        self.temperature
    }

    pub fn altitude(&self) -> Option<f32> {
        self.pressure()
            .map(|p| 44330.769 * (1.0 - (p / 1012.5).powf(0.190223)))
    }
}

#[derive(Clone, PartialEq, Eq)]
enum BMP580Register {
    WhoAmI = 0x01,
    OsrConfig = 0x36,
    OdrConfig = 0x37,
    TempXl = 0x1d,
}
