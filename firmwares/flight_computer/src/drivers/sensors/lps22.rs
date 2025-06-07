// https://www.lcsc.com/datasheet/lcsc_datasheet_2209222100_STMicroelectronics-LPS22HHTR_C2827824.pdf

use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use num_traits::float::Float;

use defmt::*;

pub struct LPS22<SPI: SpiDevice<u8>> {
    spi: SPI,
    pressure: Option<f32>,
    temperature: Option<f32>,
}

impl<SPI: SpiDevice<u8>> LPS22<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut baro = Self {
            spi,
            pressure: None,
            temperature: None,
        };

        let mut whoami = 0x00;
        for _i in 0..10 {
            whoami = baro.read_u8(LPS22Register::WhoAmI).await?;
            if whoami == 0xb3 {
                break
            }

            Timer::after(Duration::from_micros(100)).await;
        }

        if whoami != 0xb3 {
            error!("Failed to initialize LPS22 (0x{:02x} != 0xb3)", whoami);
        } else {
            info!("LPS22 initialized");
        }

        baro.configure().await?;

        Ok(baro)
    }

    async fn read_u8(&mut self, address: LPS22Register) -> Result<u8, SPI::Error> {
        let mut payload = [address as u8 | 0x80, 0]; // TODO
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(payload[1])
    }

    async fn write_u8(&mut self, address: LPS22Register, value: u8) -> Result<(), SPI::Error> {
        let mut payload = [address as u8, value];
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(())
    }

    async fn configure(&mut self) -> Result<(), SPI::Error> {
        self.write_u8(LPS22Register::CtrlReg1, 0b01110010).await?;
        self.write_u8(LPS22Register::CtrlReg2, 0b00010000).await?;
        Ok(())
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        // TODO: limit to ODR

        let mut payload = [(LPS22Register::PressOutXl as u8) | 0x80, 0, 0, 0, 0, 0];
        self.spi.transfer_in_place(&mut payload).await?;

        let press = ((payload[3] as u32) << 16) + ((payload[2] as u32) << 8) + (payload[1] as u32);
        let temp = ((payload[5] as i16) << 8) + (payload[4] as i16);

        self.pressure = Some(press as f32 / 4096.0);
        self.temperature = Some(temp as f32 / 100.0);

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
enum LPS22Register {
    WhoAmI = 0x0f,
    CtrlReg1 = 0x10,
    CtrlReg2 = 0x11,
    FifoCtrl = 0x13,
    PressOutXl = 0x28,
}
