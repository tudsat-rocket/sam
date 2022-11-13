use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI1;
use hal::prelude::*;
use hal::spi::{Master, Spi, TransferModeNormal};
use stm32f4xx_hal as hal;

use num_traits::float::Float;

type RawSpi = Spi<
    SPI1,
    (
        Pin<'A', 5, Alternate<5>>,
        Pin<'B', 4, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    ),
    TransferModeNormal,
    Master,
>;
type SharedSpi = Arc<Mutex<RefCell<RawSpi>>>;
type CsPin = hal::gpio::Pin<'C', 6, Output>;

struct MS5611CalibrationData {
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temp_coef_pressure_sensitivity: u16,
    temp_coef_pressure_offset: u16,
    reference_temperature: u16,
    temp_coef_temperature: u16,
}

pub struct Barometer {
    spi: SharedSpi,
    cs: CsPin,
    calibration_data: Option<MS5611CalibrationData>,
    read_temp: bool,
    dt: Option<i32>,
    temp: Option<i32>,
    raw_pressure: Option<i32>,
    pressure: Option<i32>,
}

impl Barometer {
    pub fn init(spi: SharedSpi, cs: CsPin) -> Result<Self, hal::spi::Error> {
        let mut baro = Self {
            spi,
            cs,
            calibration_data: None,
            read_temp: true,
            dt: None,
            temp: None,
            raw_pressure: None,
            pressure: None,
        };

        baro.read_calibration_values()?;

        Ok(baro)
    }

    fn command(&mut self, command: MS5611Command, response_len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [alloc::vec![command.into()], [0x00].repeat(response_len)].concat();

            self.cs.set_low();
            let response = spi.transfer(&mut payload);
            self.cs.set_high();

            Ok(response?[1..].to_vec())
        })
    }

    fn read_calibration_values(&mut self) -> Result<(), hal::spi::Error> {
        let c1 = self.command(MS5611Command::ReadProm(1), 2)?;
        let c2 = self.command(MS5611Command::ReadProm(2), 2)?;
        let c3 = self.command(MS5611Command::ReadProm(3), 2)?;
        let c4 = self.command(MS5611Command::ReadProm(4), 2)?;
        let c5 = self.command(MS5611Command::ReadProm(5), 2)?;
        let c6 = self.command(MS5611Command::ReadProm(6), 2)?;

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

    fn read_sensor_data(&mut self) -> Result<(), hal::spi::Error> {
        let response = self.command(MS5611Command::ReadAdc, 3)?;
        let value = ((response[0] as i32) << 16) + ((response[1] as i32) << 8) + (response[2] as i32);

        let cal = self.calibration_data.as_ref().unwrap();

        if self.read_temp {
            let dt = (value as i32) - ((cal.reference_temperature as i32) << 8);
            let temp = 2000 + ((dt as i64) * (cal.temp_coef_temperature as i64) >> 23) as i32;
            self.dt = Some(dt);
            self.temp = Some(temp);
        } else {
            self.raw_pressure = Some(value);
        }

        if self.dt.is_some() && self.raw_pressure.is_some() {
            let dt = self.dt.unwrap();
            let raw_pressure = self.raw_pressure.unwrap();

            let offset =
                ((cal.pressure_offset as i64) << 16) + ((cal.temp_coef_pressure_offset as i64 * dt as i64) >> 7);
            let sens = ((cal.pressure_sensitivity as i64) << 15)
                + ((cal.temp_coef_pressure_sensitivity as i64 * dt as i64) >> 8);
            let p = (raw_pressure as i64 * (sens >> 21) - offset) >> 15;
            self.pressure = Some(p as i32);

            // TODO: second order temp compensation

            // If the value is too drastically different from the last, skip it
            self.pressure = if let Some(last_pressure) = self.pressure {
                if ((p as i32) - last_pressure).abs() > 100 {
                    Some(last_pressure)
                } else {
                    Some(p as i32)
                }
            } else {
                Some(p as i32)
            }
        }

        Ok(())
    }

    fn start_next_conversion(&mut self) -> Result<(), hal::spi::Error> {
        let osr = MS5611OSR::OSR256;
        if self.read_temp {
            self.command(MS5611Command::StartTempConversion(osr), 0)?;
        } else {
            self.command(MS5611Command::StartPressureConversion(osr), 0)?;
        }
        Ok(())
    }

    pub fn tick(&mut self) {
        if let Err(_) = self.read_sensor_data() {
            self.dt = None;
            self.temp = None;
            self.raw_pressure = None;
            self.pressure = None;
            self.read_temp = true;
        } else {
            self.read_temp = !self.read_temp;
        }

        if let Err(_) = self.start_next_conversion() {
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
            .map(|p| 44307.694 * (1.0 - (p / 1013.25).powf(0.190284)))
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
