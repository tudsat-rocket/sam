use alloc::sync::Arc;
use alloc::vec::Vec;
use nalgebra::Vector3;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI1;
use hal::prelude::*;
use hal::spi::{Master, Spi, TransferModeNormal};
use hal::timer::SysDelay;
use stm32f4xx_hal as hal;

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
type CsPin = hal::gpio::Pin<'B', 14, Output>;

struct BMM150TrimData {
    x1: i8,
    y1: i8,
    x2: i8,
    y2: i8,
    z1: u16,
    z2: i16,
    z3: i16,
    z4: i16,
    xy1: u8,
    xy2: i8,
    xyz1: u16,
}

pub struct Compass {
    spi: SharedSpi,
    cs: CsPin,
    trim_data: Option<BMM150TrimData>,
    mag: Option<Vector3<f32>>,
}

impl Compass {
    pub fn init(spi: SharedSpi, cs: CsPin, delay: &mut SysDelay) -> Result<Self, hal::spi::Error> {
        let mut bmm = Self {
            spi,
            cs,
            trim_data: None,
            mag: None,
        };

        bmm.set_power_control(true)?;
        delay.delay_ms(3u32);
        bmm.configure_operation(BMM150DataRate::Odr30Hz, BMM150OpMode::Normal)?;
        delay.delay_ms(3u32);
        bmm.read_trim_data()?;

        // TODO: set reps?

        Ok(bmm)
    }

    fn read_registers(&mut self, address: BMM150Register, response_len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [alloc::vec![(address as u8) | 0x80], [0x00].repeat(response_len)].concat();

            self.cs.set_low();
            let response = spi.transfer(&mut payload);
            self.cs.set_high();

            Ok(response?[1..].to_vec())
        })
    }

    fn write_u8(&mut self, address: BMM150Register, value: u8) -> Result<(), hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [address as u8, value];

            self.cs.set_low();
            let res = spi.transfer(&mut payload);
            self.cs.set_high();

            res?;

            Ok(())
        })
    }

    fn read_trim_data(&mut self) -> Result<(), hal::spi::Error> {
        let trim_x1y1 = self.read_registers(BMM150Register::DigX1, 2)?;
        let trim_xyz = self.read_registers(BMM150Register::DigZ4L, 4)?;
        let trim_xy1xy2 = self.read_registers(BMM150Register::DigZ2L, 10)?;

        self.trim_data = Some(BMM150TrimData {
            x1: trim_x1y1[0] as i8,
            y1: trim_x1y1[1] as i8,
            x2: trim_xyz[2] as i8,
            y2: trim_xyz[3] as i8,
            z1: ((trim_xy1xy2[3] as u16) << 8) + (trim_xy1xy2[2] as u16),
            z2: ((trim_xy1xy2[1] as i16) << 8) + (trim_xy1xy2[0] as i16),
            z3: ((trim_xy1xy2[7] as i16) << 8) + (trim_xy1xy2[6] as i16),
            z4: ((trim_xyz[1] as i16) << 8) + (trim_xyz[0] as i16),
            xy1: trim_xy1xy2[9],
            xy2: trim_xy1xy2[8] as i8,
            xyz1: (((trim_xy1xy2[5] & 0x7f) as u16) << 8) + (trim_xy1xy2[4] as u16),
        });

        Ok(())
    }

    fn compensate_values(&self, raw_mag: (i16, i16, i16), rhall: i16) -> (f32, f32, f32) {
        let rhall = rhall as f32;
        let trim_data = self.trim_data.as_ref().unwrap();

        // adapted from https://github.com/BoschSensortec/BMM150-Sensor-API/blob/master/bmm150.c#L1614-L1712

        let xy0 = (trim_data.xyz1 as f32) * 16384.0 / rhall - 16384.0;
        let xy1 = (trim_data.xy2 as f32) * xy0 * xy0 / 268435456.0;
        let xy2 = xy1 + xy0 * (trim_data.xy1 as f32) / 16384.0;

        let x3 = (trim_data.x2 as f32) + 100.0;
        let x4 = (raw_mag.0 as f32) * (xy2 + 256.0) * x3;
        let x = (x4 / 8192.0) + (trim_data.x1 as f32) * 8.0 / 16.0;

        let y3 = (trim_data.y2 as f32) + 100.0;
        let y4 = (raw_mag.1 as f32) * (xy2 + 256.0) * y3;
        let y = (y4 / 8192.0) + (trim_data.y1 as f32) * 8.0 / 16.0;

        let z0 = (raw_mag.2 as f32) - (trim_data.z4 as f32);
        let z1 = rhall - (trim_data.xyz1 as f32);
        let z2 = (trim_data.z3 as f32) * z1;
        let z3 = (trim_data.z1 as f32) * rhall / 32768.0;
        let z4 = (trim_data.z2 as f32) + z3;
        let z5 = z0 * 131072.0 - z2;
        let z = (z5 / (z4 * 4.0)) / 16.0;

        (x, y, z)
    }

    fn read_sensor_data(&mut self) -> Result<(), hal::spi::Error> {
        let response = self.read_registers(BMM150Register::DataXL, 8)?;

        // we first have to shift all the way to the left to fill the i16 and
        // then back to the right to make sure the 2's complement survives
        let mag_x = (((response[1] as i16) << 8) + ((response[0] & 0xf8) as i16)) >> 3;
        let mag_y = (((response[3] as i16) << 8) + ((response[2] & 0xf8) as i16)) >> 3;
        let mag_z = (((response[5] as i16) << 8) + ((response[4] & 0xfe) as i16)) >> 1;
        let rhall = (((response[7] as i16) << 8) + ((response[6] & 0xfc) as i16)) >> 2;

        // TODO: check for overflows?

        let compensated = self.compensate_values((mag_x, mag_y, mag_z), rhall);
        self.mag = Some(Vector3::new(-compensated.1, -compensated.2, compensated.0));

        Ok(())
    }

    fn set_power_control(&mut self, power_control: bool) -> Result<(), hal::spi::Error> {
        self.write_u8(BMM150Register::PowerControl, power_control as u8)
    }

    fn configure_operation(&mut self, data_rate: BMM150DataRate, opmode: BMM150OpMode) -> Result<(), hal::spi::Error> {
        let reg = ((data_rate as u8) << 3) + ((opmode as u8) << 1);
        self.write_u8(BMM150Register::OpControl, reg)
    }

    pub fn tick(&mut self) {
        if let Err(_) = self.read_sensor_data() {
            self.mag = None;
        }
    }

    pub fn magnetometer(&self) -> Option<Vector3<f32>> {
        self.mag
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum BMM150Register {
    ChipId = 0x40,
    DataXL = 0x42,
    DataXH = 0x43,
    DataYL = 0x44,
    DataYH = 0x45,
    DataZL = 0x46,
    DataZH = 0x47,
    HallL = 0x48,
    HallH = 0x49,
    IntStatus = 0x4a,
    PowerControl = 0x4b,
    OpControl = 0x4c,
    InterruptAxisEnableControl1 = 0x4d,
    InterruptAxisEnableControl2 = 0x4e,
    LowThresholdInterruptControl = 0x4f,
    HighThresholdInterruptControl = 0x50,
    RepetitionControlXY = 0x51,
    RepetitionControlZ = 0x52,
    DigX1 = 0x5d,
    DigY1 = 0x5e,
    DigZ4L = 0x62,
    DigZ4H = 0x63,
    DigX2 = 0x64,
    DigY2 = 0x65,
    DigZ2L = 0x68,
    DigZ2H = 0x69,
    DigZ1L = 0x6a,
    DigZ1H = 0x6b,
    DigXYZ1L = 0x6c,
    DigXYZ1H = 0x6d,
    DigZ3L = 0x6e,
    DigZ3H = 0x6f,
    DigXY2 = 0x70,
    DigXY1 = 0x71,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum BMM150OpMode {
    Normal = 0b00,
    Forced = 0b01,
    Sleep = 0b11,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum BMM150DataRate {
    Odr2Hz = 0b001,
    Odr6Hz = 0b010,
    Odr8Hz = 0b011,
    Odr10Hz = 0b000,
    Odr15Hz = 0b100,
    Odr20Hz = 0b101,
    Odr25Hz = 0b110,
    Odr30Hz = 0b111,
}
