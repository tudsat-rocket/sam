use alloc::collections::VecDeque;
use alloc::vec::Vec;

use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use num_traits::float::Float;

use defmt::*;

const TEMP_FILTER_LEN: usize = 64;

#[derive(Debug)]
struct MS5611CalibrationData {
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temp_coef_pressure_sensitivity: u16,
    temp_coef_pressure_offset: u16,
    reference_temperature: u16,
    temp_coef_temperature: u16,
}

impl MS5611CalibrationData {
    pub fn valid(&self) -> bool {
        // We assume that every value needs to be non-zero and non-0xffff.
        self.pressure_sensitivity != 0x0000 &&
            self.pressure_offset != 0x0000 &&
            self.temp_coef_pressure_sensitivity != 0x0000 &&
            self.temp_coef_pressure_offset != 0x0000 &&
            self.reference_temperature != 0x0000 &&
            self.temp_coef_temperature != 0x0000 &&
            self.pressure_sensitivity != 0xffff &&
            self.pressure_offset != 0xffff &&
            self.temp_coef_pressure_sensitivity != 0xffff &&
            self.temp_coef_pressure_offset != 0xffff &&
            self.reference_temperature != 0xffff &&
            self.temp_coef_temperature != 0xffff
    }
}

pub struct MS5611<SPI: SpiDevice<u8>> {
    spi: SPI,
    calibration_data: Option<MS5611CalibrationData>,
    read_temp: bool,
    dt_filter: VecDeque<i32>,
    dt: Option<i32>,
    temp: Option<i32>,
    raw_pressure: Option<i32>,
    pressure: Option<i32>,
}

impl<SPI: SpiDevice<u8>> MS5611<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut baro = Self {
            spi,
            calibration_data: None,
            read_temp: true,
            dt_filter: VecDeque::with_capacity(TEMP_FILTER_LEN),
            dt: None,
            temp: None,
            raw_pressure: None,
            pressure: None,
        };

        'outer: for _i in 0..3 { // did you know that rust has loop labels?
            baro.reset().await?;

            for _j in 0..50 {
                Timer::after(Duration::from_micros(10)).await;

                baro.read_calibration_values().await?;
                if baro.calibration_data.as_ref().map(|d| d.valid()).unwrap_or(false) {
                    break 'outer;
                }
            }
        }

        if baro.calibration_data.as_ref().map(|d| d.valid()).unwrap_or(false) {
            info!("MS5611 successfully initialized.");
        } else {
            error!("Failed to initialize MS5611");
        }

        Ok(baro)
    }

    async fn command(&mut self, command: MS5611Command, response_len: usize) -> Result<Vec<u8>, SPI::Error> {
        let mut payload = [alloc::vec![command.into()], [0x00].repeat(response_len)].concat();
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(payload[1..].to_vec())
    }

    async fn reset(&mut self) -> Result<(), SPI::Error> {
        self.command(MS5611Command::Reset, 0).await?;
        Ok(())
    }

    async fn read_calibration_values(&mut self) -> Result<(), SPI::Error> {
        let c1 = self.command(MS5611Command::ReadProm(1), 2).await?;
        let c2 = self.command(MS5611Command::ReadProm(2), 2).await?;
        let c3 = self.command(MS5611Command::ReadProm(3), 2).await?;
        let c4 = self.command(MS5611Command::ReadProm(4), 2).await?;
        let c5 = self.command(MS5611Command::ReadProm(5), 2).await?;
        let c6 = self.command(MS5611Command::ReadProm(6), 2).await?;

        self.calibration_data = Some(MS5611CalibrationData {
            pressure_sensitivity: ((c1[0] as u16) << 8) + (c1[1] as u16),
            pressure_offset: ((c2[0] as u16) << 8) + (c2[1] as u16),
            temp_coef_pressure_sensitivity: ((c3[0] as u16) << 8) + (c3[1] as u16),
            temp_coef_pressure_offset: ((c4[0] as u16) << 8) + (c4[1] as u16),
            reference_temperature: ((c5[0] as u16) << 8) + (c5[1] as u16),
            temp_coef_temperature: ((c6[0] as u16) << 8) + (c6[1] as u16),
        });

        Ok(())
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let response = self.command(MS5611Command::ReadAdc, 3).await?;
        let value = ((response[0] as i32) << 16) + ((response[1] as i32) << 8) + (response[2] as i32);

        let cal = self.calibration_data.as_ref().unwrap();

        if self.read_temp {
            let dt = (value as i32) - ((cal.reference_temperature as i32) << 8);
            self.dt_filter.truncate(TEMP_FILTER_LEN - 1);
            self.dt_filter.push_front(dt);
            let filtered = self.dt_filter.iter()
                .map(|dt| *dt as i64)
                .sum::<i64>() / (self.dt_filter.len() as i64);
            self.dt = Some(filtered as i32);
        } else {
            self.raw_pressure = Some(value);
        }

        if let Some((dt, raw_pressure)) = self.dt.zip(self.raw_pressure) {
            let mut temp = 2000 + (((dt as i64) * (cal.temp_coef_temperature as i64)) >> 23);
            let mut offset =
                ((cal.pressure_offset as i64) << 16) + ((cal.temp_coef_pressure_offset as i64 * dt as i64) >> 7);
            let mut sens = ((cal.pressure_sensitivity as i64) << 15)
                + (((cal.temp_coef_pressure_sensitivity as i64) * (dt as i64)) >> 8);

            // second order temp compensation
            if temp < 2000 {
                let t2 = ((dt as i64) * (dt as i64)) >> 31;
                let temp_offset = temp - 2000;
                let mut off2 = (5 * temp_offset * temp_offset) >> 1;
                let mut sens2 = off2 >> 1;

                if temp < -1500 { // brrrr
                    let temp_offset = temp + 1500;
                    off2 += 7 * temp_offset * temp_offset;
                    sens2 += (11 * temp_offset * temp_offset) >> 1;
                }

                temp -= t2;
                offset -= off2;
                sens -= sens2;
            }

            self.temp = Some(temp as i32);
            let p = (((raw_pressure as i64 * sens) >> 21) - offset) >> 15;
            self.pressure = Some(p as i32);
        }

        Ok(())
    }

    async fn start_next_conversion(&mut self) -> Result<(), SPI::Error> {
        let osr = MS5611OSR::OSR256;
        if self.read_temp {
            self.command(MS5611Command::StartTempConversion(osr), 0).await?;
        } else {
            self.command(MS5611Command::StartPressureConversion(osr), 0).await?;
        }
        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(_) = self.read_sensor_data().await {
            self.dt = None;
            self.temp = None;
            self.raw_pressure = None;
            self.pressure = None;
            self.read_temp = true;
        } else {
            self.read_temp = !self.read_temp;
        }

        if let Err(_) = self.start_next_conversion().await {
            self.dt = None;
            self.temp = None;
            self.raw_pressure = None;
            self.pressure = None;
            self.read_temp = true;
        }
    }

    pub fn temperature(&self) -> Option<f32> {
        self.temp.map(|t| (t as f32) / 100.0)
    }

    pub fn pressure(&self) -> Option<f32> {
        self.pressure.map(|p| (p as f32) / 100.0)
    }

    pub fn altitude(&self) -> Option<f32> {
        self.pressure()
            .map(|p| 44330.769 * (1.0 - (p / 1012.5).powf(0.190223)))
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MS5611Command {
    Reset,
    StartPressureConversion(MS5611OSR),
    StartTempConversion(MS5611OSR),
    ReadAdc,
    ReadProm(u8),
}

impl Into<u8> for MS5611Command {
    fn into(self: Self) -> u8 {
        match self {
            Self::Reset => 0x1e,
            Self::StartPressureConversion(osr) => 0x40 + ((osr as u8) << 1),
            Self::StartTempConversion(osr) => 0x50 + ((osr as u8) << 1),
            Self::ReadAdc => 0x00,
            Self::ReadProm(adr) => 0xa0 + (adr << 1),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MS5611OSR {
    OSR256 = 0b000,
    OSR512 = 0b001,
    OSR1024 = 0b010,
    OSR2048 = 0b011,
    OSR4096 = 0b100,
}
