#[cfg(not(feature = "std"))]
use alloc::string::{String, ToString};
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

#[cfg(feature = "std")]
use std::string::{String, ToString};
#[cfg(feature = "std")]
use std::vec::Vec;

use core::iter::Extend;

use nalgebra::UnitQuaternion;
use serde::{Deserialize, Serialize};

pub use LogLevel::*;

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub struct f8 {
    raw: u8,
}

#[test]
fn test_f8() {
    let values: Vec<f32> = alloc::vec![-0.00001, 0.008, -0.125, 0.5, 1.0, -10.0, 100.0, 1000.0];
    let compressed: Vec<f8> = values.iter().map(|x| (*x).into()).collect();
    let recovered: Vec<f32> = compressed.iter().map(|x| (*x).into()).collect();
    assert_eq!(recovered, alloc::vec![-0.01953125, 0.015625, -0.125, 0.5, 1.0, -10.0, 96.0, 960.0]);
}

impl From<f32> for f8 {
    fn from(x: f32) -> Self {
        let bits = x.to_bits();
        let sign = bits >> 31;
        let exponent = (bits >> 23) & 0xff;
        let fraction = bits & 0x7fffff;

        let expo_small = (((exponent as i32) - 0x80).clamp(-7, 8) + 7) as u32;
        let fraction_small = fraction >> 20;

        let raw = ((sign << 7) | (expo_small << 3) | fraction_small) as u8;
        Self { raw }
    }
}

impl Into<f32> for f8 {
    fn into(self) -> f32 {
        let sign = self.raw >> 7;
        let exponent = (self.raw & 0x7f) >> 3;
        let fraction = self.raw & 0x7;

        let expo_large = ((exponent as i32) - 7 + 0x80) as u32;
        let fraction_large = (fraction as u32) << 20;

        let bits = ((sign as u32) << 31) | (expo_large << 23) | fraction_large;
        f32::from_bits(bits)
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum FlightMode {
    Idle = 0,
    HardwareArmed = 1,
    Armed = 2,
    Flight = 3,
    RecoveryDrogue = 4,
    RecoveryMain = 5,
    Landed = 6,
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Idle
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum GPSFixType {
    NoFix,
    AutonomousFix,
    DifferentialFix,
    RTKFix,
    RTKFloat,
    DeadReckoningFix,
}

impl Default for GPSFixType {
    fn default() -> Self {
        Self::NoFix
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMain {
    pub time: u32,
    pub mode: FlightMode,
    pub orientation: Option<UnitQuaternion<f32>>,
    pub vertical_speed: f32,
    pub altitude_baro: f32,
    pub altitude: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMainCompressed {
    pub time: u32, // TODO: combine these two?
    pub mode: FlightMode,
    pub orientation: (u8, u8, u8, u8),
    pub vertical_speed: f8,
    pub altitude_baro: u16,
    pub altitude: u16
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensors {
    pub time: u32,
    pub gyro: (f32, f32, f32),
    pub accelerometer1: (f32, f32, f32),
    pub accelerometer2: (f32, f32, f32),
    pub magnetometer: (f32, f32, f32),
    pub temperature_baro: f32,
    pub pressure_baro: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensorsCompressed {
    pub time: u32,
    pub gyro: (f8, f8, f8),
    pub accelerometer1: (f8, f8, f8),
    pub accelerometer2: (f8, f8, f8),
    pub magnetometer: (f8, f8, f8),
    pub temperature_baro: i8,
    pub pressure_baro: u16, // TODO: compress this further
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryDiagnostics {
    // TODO: compress
    pub time: u32,
    pub loop_runtime: u16,
    pub temperature_core: i16,
    pub cpu_voltage: u16,
    /// Battery voltage in mV
    pub battery_voltage: u16,
    /// Voltage behind arm switch in mV
    pub arm_voltage: u16,
    /// Current current draw (hah!) in mA
    pub current: u16,
    /// Battery capacity consumed in mAh
    pub consumed: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryGPS {
    // TODO: compress
    pub time: u32,
    pub fix: GPSFixType,
    pub hdop: u16,
    pub num_satellites: u8,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude_asl: Option<f32>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryGCS {
    pub time: u32,
    pub lora_rssi: u8,
    pub lora_rssi_signal: u8,
    pub lora_snr: u8,
}

#[derive(Debug, Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Serialize, Deserialize)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum DownlinkMessage {
    TelemetryMain(TelemetryMain),
    TelemetryMainCompressed(TelemetryMainCompressed),
    TelemetryRawSensors(TelemetryRawSensors),
    TelemetryRawSensorsCompressed(TelemetryRawSensorsCompressed),
    TelemetryDiagnostics(TelemetryDiagnostics),
    TelemetryGPS(TelemetryGPS),
    TelemetryGCS(TelemetryGCS),
    Log(u32, String, LogLevel, String),
}

impl DownlinkMessage {
    pub fn time(&self) -> u32 {
        match self {
            DownlinkMessage::TelemetryMain(tm) => tm.time,
            DownlinkMessage::TelemetryMainCompressed(tm) => tm.time,
            DownlinkMessage::TelemetryRawSensors(tm) => tm.time,
            DownlinkMessage::TelemetryRawSensorsCompressed(tm) => tm.time,
            DownlinkMessage::TelemetryDiagnostics(tm) => tm.time,
            DownlinkMessage::TelemetryGPS(tm) => tm.time,
            DownlinkMessage::TelemetryGCS(tm) => tm.time,
            DownlinkMessage::Log(t, _, _, _) => *t,
        }
    }
}

#[derive(PartialEq, Clone, Debug, Serialize, Deserialize)]
pub enum UplinkMessage {
    // TODO: attach HMAC?
    Heartbeat,
    Reboot,
    RebootToBootloader,
}

impl ToString for LogLevel {
    fn to_string(&self) -> String {
        match self {
            Debug => "DEBUG",
            Info => "INFO",
            Warning => "WARNING",
            Error => "ERROR",
            Critical => "CRITICAL",
        }
        .to_string()
    }
}

impl UplinkMessage {
    pub fn wrap(&self) -> Vec<u8> {
        let mut buf = [0u8; 256];
        let serialized = postcard::to_slice(self, &mut buf).unwrap();
        [&[0x42, serialized.len() as u8], &*serialized].concat()
    }
}

impl DownlinkMessage {
    pub fn wrap(&self) -> Vec<u8> {
        let mut buf = [0u8; 512];
        let serialized = postcard::to_slice(self, &mut buf).unwrap();
        [&[0x42, serialized.len() as u8], &*serialized].concat()
    }

    pub fn read_valid(buf: &[u8]) -> Option<Self> {
        if buf.len() == 0 {
            return None;
        }

        if buf[0] != 0x42 {
            return None;
        }

        if buf.len() < 2 {
            return None;
        }

        let len = buf[1] as usize;
        if buf.len() < 2 + len {
            return None;
        }

        postcard::from_bytes::<DownlinkMessage>(&buf[2..(len + 2)]).ok()
    }

    pub fn pop_valid(buf: &mut Vec<u8>) -> Option<Self> {
        while buf.len() > 0 {
            if buf[0] == 0x42 {
                if buf.len() < 2 {
                    return None;
                }

                let len = buf[1] as usize;
                if buf.len() < 2 + len {
                    return None;
                }

                break;
            }

            buf.remove(0);
        }

        if buf.len() < 2 {
            return None;
        }

        let len = buf[1] as usize;
        let result = postcard::from_bytes::<DownlinkMessage>(&buf[2..(len + 2)]).ok();

        for _i in 0..(len + 2) {
            buf.remove(0);
        }

        result
    }
}
