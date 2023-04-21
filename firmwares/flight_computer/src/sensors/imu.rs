use alloc::sync::Arc;
use nalgebra::Vector3;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI1;
use hal::prelude::*;
use hal::spi::{Master, Spi, TransferModeNormal};
use stm32f4xx_hal as hal;

use crate::prelude::*;

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
type CsPin = hal::gpio::Pin<'B', 15, Output>;

const G_TO_MS2: f32 = 9.80665;

pub struct Imu {
    spi: SharedSpi,
    cs: CsPin,
    gyro_scale: LSM6GyroscopeScale,
    accel_scale: LSM6AccelerometerScale,
    gyro: Option<Vector3<f32>>,
    accel: Option<Vector3<f32>>,
}

impl Imu {
    pub fn init(spi: SharedSpi, cs: CsPin) -> Result<Self, hal::spi::Error> {
        let gyro_scale = LSM6GyroscopeScale::Max2000Dps;
        let accel_scale = LSM6AccelerometerScale::Max16G;

        let mut imu = Self {
            spi,
            cs,
            gyro_scale,
            accel_scale,
            gyro: None,
            accel: None,
        };

        // TODO: check status register?
        //self.read_u8(LSM6RRegister::StatusReg);

        imu.configure_gyroscope(LSM6GyroscopeMode::HighPerformance1660Hz, gyro_scale)?;

        imu.configure_accelerometer(
            LSM6AccelerometerMode::HighPerformance1660Hz,
            accel_scale,
            false, // TODO
        )?;

        Ok(imu)
    }

    fn write_u8(&mut self, address: LSM6RRegister, value: u8) -> Result<(), hal::spi::Error> {
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

    fn read_sensor_data(&mut self) -> Result<(), hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [(LSM6RRegister::OutTempL as u8) | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

            self.cs.set_low();
            let response = spi.transfer(&mut payload)?;
            self.cs.set_high();

            // TODO: do we need this temp?
            let _temp = ((response[2] as i16) << 8) + (response[1] as i16);
            let gyro_x = ((response[4] as i16) << 8) + (response[3] as i16);
            let gyro_y = ((response[6] as i16) << 8) + (response[5] as i16);
            let gyro_z = ((response[8] as i16) << 8) + (response[7] as i16);
            let accel_x = ((response[10] as i16) << 8) + (response[9] as i16);
            let accel_y = ((response[12] as i16) << 8) + (response[11] as i16);
            let accel_z = ((response[14] as i16) << 8) + (response[13] as i16);

            // rotate values to match vehicle coordinate system (invert x, swap y and z)
            // and convert to m/s^2 and deg/s
            self.gyro = Some(self.gyro_scale.scale_raw_values(Vector3::new(-gyro_x, gyro_z, gyro_y)));
            self.accel = Some(self.accel_scale.scale_raw_values(Vector3::new(-accel_x, accel_z, accel_y)));

            Ok(())
        })
    }

    fn configure_accelerometer(
        &mut self,
        mode: LSM6AccelerometerMode,
        scale: LSM6AccelerometerScale,
        high_res: bool,
    ) -> Result<(), hal::spi::Error> {
        let reg = ((mode as u8) << 4) + ((scale as u8) << 2) + ((high_res as u8) << 1);
        self.write_u8(LSM6RRegister::Ctrl1Xl, reg)
    }

    fn configure_gyroscope(
        &mut self,
        mode: LSM6GyroscopeMode,
        scale: LSM6GyroscopeScale,
    ) -> Result<(), hal::spi::Error> {
        let reg = ((mode as u8) << 4) + (scale as u8);
        self.write_u8(LSM6RRegister::Ctrl2G, reg)
    }

    pub fn tick(&mut self) {
        if let Err(e) = self.read_sensor_data() {
            self.gyro = None;
            self.accel = None;
            log!(Error, "{:?}", e);
        }
    }

    pub fn accelerometer(&self) -> Option<Vector3<f32>> {
        self.accel
    }

    pub fn gyroscope(&self) -> Option<Vector3<f32>> {
        self.gyro
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LSM6AccelerometerMode {
    PowerDown = 0,
    LowPower13Hz = 0b1,
    LowPower26Hz = 0b10,
    LowPower52Hz = 0b11,
    Normal104Hz = 0b100,
    Normal208Hz = 0b101,
    HighPerformance416Hz = 0b110,
    HighPerformance833Hz = 0b111,
    HighPerformance1660Hz = 0b1000,
    HighPerformance3330Hz = 0b1001,
    HighPerformance6660Hz = 0b1010,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LSM6AccelerometerScale {
    Max2G = 0b00,
    Max4G = 0b10,
    Max8G = 0b11,
    Max16G = 0b01,
}

impl LSM6AccelerometerScale {
    fn scale_raw_values(&self, raw: Vector3<i16>) -> Vector3<f32> {
        let factor = match self {
            Self::Max2G => (i16::MAX as f32) / (2.0 * G_TO_MS2),
            Self::Max4G => (i16::MAX as f32) / (4.0 * G_TO_MS2),
            Self::Max8G => (i16::MAX as f32) / (8.0 * G_TO_MS2),
            Self::Max16G => (i16::MAX as f32) / (16.0 * G_TO_MS2),
        };

        Vector3::new(raw.x as f32, raw.y as f32, raw.z as f32) / factor
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LSM6GyroscopeMode {
    PowerDown = 0,
    LowPower13Hz = 0b1,
    LowPower26Hz = 0b10,
    LowPower52Hz = 0b11,
    Normal104Hz = 0b100,
    Normal208Hz = 0b101,
    HighPerformance416Hz = 0b1100,
    HighPerformance833Hz = 0b111,
    HighPerformance1660Hz = 0b1000,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LSM6GyroscopeScale {
    Max125Dps = 0b0010,
    Max250Dps = 0b0000,
    Max500Dps = 0b0100,
    Max1000Dps = 0b1000,
    Max2000Dps = 0b1100,
    Max4000Dps = 0b0001,
}

impl LSM6GyroscopeScale {
    fn scale_raw_values(&self, raw: Vector3<i16>) -> Vector3<f32> {
        let factor = match self {
            Self::Max125Dps => (i16::MAX as f32) / 125.0,
            Self::Max250Dps => (i16::MAX as f32) / 250.0,
            Self::Max500Dps => (i16::MAX as f32) / 500.0,
            Self::Max1000Dps => (i16::MAX as f32) / 1000.0,
            Self::Max2000Dps => (i16::MAX as f32) / 2000.0,
            Self::Max4000Dps => (i16::MAX as f32) / 4000.0,
        };

        Vector3::new(raw.x as f32, raw.y as f32, raw.z as f32) / factor
    }
}

#[derive(Clone, PartialEq, Eq)]
#[allow(dead_code)]
enum LSM6RRegister {
    FuncCfgAddress = 0x01,
    PinCtrl = 0x02,
    S4sTphL = 0x04,
    S4sTphH = 0x05,
    S4sRr = 0x06,
    FifoCtrl1 = 0x07,
    FifoCtrl2 = 0x08,
    FifoCtrl3 = 0x09,
    FifoCtrl4 = 0x0a,
    CounterBdrReg1 = 0x0b,
    CounterBdrReg2 = 0x0c,
    Int1Ctrl = 0x0d,
    Int2Ctrl = 0x0e,
    WhoAmI = 0x0f,
    Ctrl1Xl = 0x10,
    Ctrl2G = 0x11,
    Ctrl3C = 0x12,
    Ctrl4C = 0x13,
    Ctrl5C = 0x14,
    Ctrl6C = 0x15,
    Ctrl7G = 0x16,
    Ctrl8Xl = 0x17,
    Ctrl9Xl = 0x18,
    Ctrl10C = 0x19,
    AllIntSrc = 0x1a,
    WakeUpSrc = 0x1b,
    TapSrc = 0x1c,
    D6dSrc = 0x1d,
    StatusReg = 0x1e,
    OutTempL = 0x20,
    OutTempH = 0x21,
    OutXLG = 0x22,
    OutXHG = 0x23,
    OutYLG = 0x24,
    OutYHG = 0x25,
    OutZLG = 0x26,
    OutZHG = 0x27,
    OutXLA = 0x28,
    OutXHA = 0x29,
    OutYLA = 0x2a,
    OutYHA = 0x2b,
    OutZLA = 0x2c,
    OutZHA = 0x2d,
    EmbFuncStatusMainpage = 0x35,
    FsmStatusAMainpage = 0x36,
    FsmStatusBMainpage = 0x37,
    StatusMasterMainpage = 0x39,
    FifoStatus1 = 0x3a,
    FifoStatus2 = 0x3b,
    Timestamp0 = 0x40,
    Timestamp1 = 0x41,
    Timestamp2 = 0x42,
    Timestamp3 = 0x43,
    TapCfg0 = 0x56,
    TapCfg1 = 0x57,
    TapCfg2 = 0x58,
    TapThs6D = 0x59,
    IntDur2 = 0x5a,
    WakeUpThs = 0x5b,
    WakeUpDur = 0x5c,
    FreeFall = 0x5d,
    Md1Cfg = 0x5e,
    Md2Cfg = 0x5f,
    S4sStCmdCode = 0x60,
    S4sDtReg = 0x61,
    I3cBusAvb = 0x62,
    InternalFreqFine = 0x63,
    IntOis = 0x6f,
    Ctrl1Ois = 0x70,
    Ctrl2Ois = 0x71,
    Ctrl3Ois = 0x72,
    XOfsUsr = 0x73,
    YOfsUsr = 0x74,
    ZOfsUsr = 0x75,
    FifoDataOutTag = 0x78,
    FifoDataOutXL = 0x79,
    FifoDataOutXH = 0x7a,
    FifoDataOutYL = 0x7b,
    FifoDataOutYH = 0x7c,
    FifoDataOutZL = 0x7d,
    FifoDataOutZH = 0x7e, // datasheet seems to have a typo here?
}
