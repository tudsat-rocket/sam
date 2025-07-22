use heapless::{Deque, Vec};

use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiDevice;

use num_traits::float::Float;

use defmt::*;

const BARO_MEDIAN_FILTER_LENGTH: usize = 20;

#[derive(PartialEq, Clone, Copy)]
pub enum MS56Variant {
    MS5607,
    MS5611,
}

struct MS56CalibrationData {
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temp_coef_pressure_sensitivity: u16,
    temp_coef_pressure_offset: u16,
    reference_temperature: u16,
    temp_coef_temperature: u16,
}

impl MS56CalibrationData {
    pub fn valid(&self) -> bool {
        // We assume that every value needs to be non-zero and non-0xffff.
        self.pressure_sensitivity != 0x0000
            && self.pressure_offset != 0x0000
            && self.temp_coef_pressure_sensitivity != 0x0000
            && self.temp_coef_pressure_offset != 0x0000
            && self.reference_temperature != 0x0000
            && self.temp_coef_temperature != 0x0000
            && self.pressure_sensitivity != 0xffff
            && self.pressure_offset != 0xffff
            && self.temp_coef_pressure_sensitivity != 0xffff
            && self.temp_coef_pressure_offset != 0xffff
            && self.reference_temperature != 0xffff
            && self.temp_coef_temperature != 0xffff
    }
}

pub struct MS56<SPI: SpiDevice<u8>> {
    variant: MS56Variant,
    spi: SPI,
    calibration_data: Option<MS56CalibrationData>,
    read_temp: bool,
    dt: Option<i32>,
    temp: Option<i32>,
    raw_pressure: Option<i32>,
    pressure: Option<i32>,
    baro_filter: BaroFilter,
}

impl<SPI: SpiDevice<u8>> MS56<SPI> {
    pub async fn init(variant: MS56Variant, spi: SPI) -> Result<Self, SPI::Error> {
        let mut baro = Self {
            variant,
            spi,
            calibration_data: None,
            read_temp: true,
            dt: None,
            temp: None,
            raw_pressure: None,
            pressure: None,
            baro_filter: BaroFilter::new(),
        };

        'outer: for _i in 0..3 {
            // did you know that rust has loop labels?
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
            info!("MS56xx initialized");
        } else {
            error!("Failed to initialize MS56xx");
        }

        Ok(baro)
    }

    async fn command(&mut self, command: MS56Command, response_len: usize) -> Result<Vec<u8, 32>, SPI::Error> {
        let mut payload = [0x00; 32];
        payload[0] = command.into();
        self.spi.transfer_in_place(&mut payload[..1 + response_len]).await?;
        Ok(Vec::from_slice(&payload[1..1 + response_len]).unwrap_or_default())
    }

    async fn reset(&mut self) -> Result<(), SPI::Error> {
        self.command(MS56Command::Reset, 0).await?;
        Ok(())
    }

    async fn read_calibration_values(&mut self) -> Result<(), SPI::Error> {
        let c1 = self.command(MS56Command::ReadProm(1), 2).await?;
        let c2 = self.command(MS56Command::ReadProm(2), 2).await?;
        let c3 = self.command(MS56Command::ReadProm(3), 2).await?;
        let c4 = self.command(MS56Command::ReadProm(4), 2).await?;
        let c5 = self.command(MS56Command::ReadProm(5), 2).await?;
        let c6 = self.command(MS56Command::ReadProm(6), 2).await?;

        self.calibration_data = Some(MS56CalibrationData {
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
        let response = self.command(MS56Command::ReadAdc, 3).await?;
        let value = ((response[0] as i32) << 16) + ((response[1] as i32) << 8) + (response[2] as i32);
        let cal = self.calibration_data.as_ref().unwrap();

        if self.read_temp {
            let dt = (value as i32) - ((cal.reference_temperature as i32) << 8);
            let filtered = self.baro_filter.filter(dt);
            self.dt = Some(filtered);
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

                if temp < -1500 {
                    // brrrr
                    let temp_offset = temp + 1500;
                    off2 += 7 * temp_offset * temp_offset;
                    sens2 += (11 * temp_offset * temp_offset) >> 1;
                }

                temp -= t2;
                offset -= off2;
                sens -= sens2;
            }

            self.temp = Some(temp as i32);

            let shift = match self.variant {
                MS56Variant::MS5607 => 14,
                MS56Variant::MS5611 => 15,
            };
            let p = (((raw_pressure as i64 * sens) >> 21) - offset) >> shift;
            self.pressure = Some(p as i32);
        }

        Ok(())
    }

    async fn start_next_conversion(&mut self) -> Result<(), SPI::Error> {
        let osr = MS56OSR::OSR256;
        if self.read_temp {
            self.command(MS56Command::StartTempConversion(osr), 0).await?;
        } else {
            self.command(MS56Command::StartPressureConversion(osr), 0).await?;
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
        self.pressure().map(|p| 44330.769 * (1.0 - (p / 1012.5).powf(0.190223)))
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MS56Command {
    Reset,
    StartPressureConversion(MS56OSR),
    StartTempConversion(MS56OSR),
    ReadAdc,
    ReadProm(u8),
}

impl Into<u8> for MS56Command {
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
enum MS56OSR {
    OSR256 = 0b000,
    OSR512 = 0b001,
    OSR1024 = 0b010,
    OSR2048 = 0b011,
    OSR4096 = 0b100,
}

pub struct BaroFilter {
    previous_raw_values: Deque<i32, BARO_MEDIAN_FILTER_LENGTH>,
    last_spike_warning_counter: u32,
}

impl BaroFilter {
    pub fn new() -> Self {
        Self {
            previous_raw_values: Deque::new(),
            last_spike_warning_counter: 0,
        }
    }

    //median filter
    pub fn filter(&mut self, input_value: i32) -> i32 {
        const SPIKE_WARNING_THRESHOLD: i32 = 8000000;

        while self.previous_raw_values.len() > (BARO_MEDIAN_FILTER_LENGTH - 1) {
            let _ = self.previous_raw_values.pop_front();
        }

        let _ = self.previous_raw_values.push_back(input_value);

        let mut sorted: Vec<_, BARO_MEDIAN_FILTER_LENGTH> = self.previous_raw_values.iter().collect();
        sorted.sort_unstable();

        if self.last_spike_warning_counter <= 100 {
            self.last_spike_warning_counter += 1;
        }

        let diff = *sorted.last().unwrap() - *sorted.first().unwrap();
        if diff > SPIKE_WARNING_THRESHOLD && self.last_spike_warning_counter > 100 {
            defmt::warn!("Baro temp spike: {}", diff);
            self.last_spike_warning_counter = 0;
        }

        *sorted[sorted.len() / 2]
    }
}
