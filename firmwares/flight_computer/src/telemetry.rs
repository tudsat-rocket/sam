//! Contains data structures that the flight computer shares with the outside world,
//! namely the ground station software, as well as common (de)serialization code.

#[cfg(not(feature = "std"))]
use alloc::string::{String, ToString};
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
use siphasher::sip::SipHasher;

use core::hash::Hasher;
#[cfg(feature = "std")]
use std::string::{String, ToString};
#[cfg(feature = "std")]
use std::vec::Vec;

use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use serde::de::DeserializeOwned;

use crate::settings::*;

pub const LORA_MESSAGE_INTERVAL: u32 = 25;
pub const LORA_UPLINK_INTERVAL: u32 = 200;
pub const LORA_UPLINK_MODULO: u32 = 100;
pub const FLASH_SIZE: u32 = 32 * 1024 * 1024;
pub const FLASH_HEADER_SIZE: u32 = 4096; // needs to be multiple of 4096

pub use LogLevel::*;

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
#[allow(non_camel_case_types)]
pub struct f8(u8);

impl From<f32> for f8 {
    fn from(x: f32) -> Self {
        let bits = x.to_bits();
        let sign = bits >> 31;
        let exponent = (bits >> 23) & 0xff;
        let fraction = bits & 0x7fffff;

        let expo_small = (((exponent as i32) - 0x80).clamp(-7, 8) + 7) as u32;
        let fraction_small = fraction >> 20;

        let raw = ((sign << 7) | (expo_small << 3) | fraction_small) as u8;
        Self(raw)
    }
}

impl Into<f32> for f8 {
    fn into(self) -> f32 {
        let sign = self.0 >> 7;
        let exponent = (self.0 & 0x7f) >> 3;
        let fraction = self.0 & 0x7;

        let expo_large = ((exponent as i32) - 7 + 0x80) as u32;
        let fraction_large = (fraction as u32) << 20;

        let bits = ((sign as u32) << 31) | (expo_large << 23) | fraction_large;
        f32::from_bits(bits)
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct CompressedVector3(f8, f8, f8);

impl From<Vector3<f32>> for CompressedVector3 {
    fn from(vec: Vector3<f32>) -> Self {
        Self(vec.x.into(), vec.y.into(), vec.z.into())
    }
}

impl Into<Vector3<f32>> for CompressedVector3 {
    fn into(self) -> Vector3<f32> {
        Vector3::new(self.0.into(), self.1.into(), self.2.into())
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, PartialOrd)]
pub enum FlightMode {
    Idle = 0,
    HardwareArmed = 1,
    Armed = 2,
    Flight = 3,
    RecoveryDrogue = 4,
    RecoveryMain = 5,
    Landed = 6,
}

impl FlightMode {
    pub fn led_state(self, time: u32) -> (bool, bool, bool) {
        match self {
            FlightMode::Idle => (false, false, true),                       // ( ,  , G)
            FlightMode::HardwareArmed => (true, time % 500 < 250, false),   // (R, y,  )
            FlightMode::Armed => (true, true, false),                       // (R, Y,  )
            FlightMode::Flight => (false, true, false),                     // ( , Y,  )
            FlightMode::RecoveryDrogue => (false, true, true),              // ( , Y, G)
            FlightMode::RecoveryMain => (true, false, true),                // (R,  , G)
            FlightMode::Landed => (false, false, time % 1000 < 500),        // ( ,  , g)
        }
    }
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Idle
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum GPSFixType {
    NoFix = 0,
    AutonomousFix = 1,
    DifferentialFix = 2,
    RTKFix = 3,
    RTKFloat = 4,
    DeadReckoningFix = 5,
}

impl Default for GPSFixType {
    fn default() -> Self {
        Self::NoFix
    }
}

impl From<u8> for GPSFixType {
    fn from(x: u8) -> Self {
        match x {
            0 => Self::NoFix,
            1 => Self::AutonomousFix,
            2 => Self::DifferentialFix,
            3 => Self::RTKFix,
            4 => Self::RTKFloat,
            5 => Self::DeadReckoningFix,
            _ => Self::NoFix
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMain {
    pub time: u32,
    pub mode: FlightMode,
    pub orientation: Option<UnitQuaternion<f32>>,
    pub vertical_speed: f32,
    pub vertical_accel: f32,
    pub vertical_accel_filtered: f32,
    pub altitude_baro: f32,
    pub altitude_max: f32,
    pub altitude: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMainCompressed {
    pub time: u32, // TODO: combine these two?
    pub mode: FlightMode,
    pub orientation: (u8, u8, u8, u8),
    pub vertical_speed: f8,
    pub vertical_accel: f8,
    pub vertical_accel_filtered: f8,
    pub altitude_baro: u16,
    pub altitude_max: u16,
    pub altitude: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensors {
    pub time: u32,
    pub gyro: Vector3<f32>,
    pub accelerometer1: Vector3<f32>,
    pub accelerometer2: Vector3<f32>,
    pub magnetometer: Vector3<f32>,
    pub temperature_baro: f32,
    pub pressure_baro: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensorsCompressed {
    pub time: u32,
    pub gyro: CompressedVector3,
    pub accelerometer1: CompressedVector3,
    pub accelerometer2: CompressedVector3,
    pub magnetometer: CompressedVector3,
    pub temperature_baro: i8,
    pub pressure_baro: u16, // TODO: compress this further
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryDiagnostics {
    pub time: u32,
    pub cpu_utilization: u8,
    pub heap_utilization: u8,
    pub temperature_core: i8,
    pub cpu_voltage: u16,
    /// Battery voltage in mV
    pub battery_voltage: u16,
    /// Voltage behind arm switch in mV
    pub arm_voltage: u16,
    /// Current current draw (hah!) in mA
    pub current: u16,
    ///// Battery capacity consumed in mAh
    //pub consumed: u16,
    pub lora_rssi: u8,
    pub altitude_ground: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryGPS {
    // TODO: compress
    pub time: u32,
    pub fix_and_sats: u8,
    pub hdop: u16,
    pub latitude: [u8; 3],
    pub longitude: [u8; 3],
    pub altitude_asl: u16,
    pub flash_pointer: u16,
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
    FlashContent(u32, Vec<u8>),
    Settings(Settings)
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
            DownlinkMessage::FlashContent(_, _) => 0,
            DownlinkMessage::Settings(_) => 0,
        }
    }
}

#[derive(PartialEq, Clone, Debug, Serialize, Deserialize)]
pub enum UplinkMessage {
    /// Heartbeat command, allows the flight computer to track signal strength
    /// without sending commands
    Heartbeat,
    Command(Command),
    ReadFlash(u32, u32), // TODO: make this a command as well?
    ReadSettings,
    WriteSettings(Settings),
    ApplyLoRaSettings(LoRaSettings),
}

#[derive(PartialEq, Clone, Debug, Serialize, Deserialize)]
pub enum Command {
    Reboot,
    RebootToBootloader,
    SetFlightMode(FlightMode),
    EraseFlash,
}

impl Command {
    pub fn authenticate(&self, time: u32, key: &[u8; 16]) -> u64 {
        let mut buf: [u8; 4] = [0x00; 4];
        let serialized = postcard::to_slice_cobs(self, &mut buf).unwrap();

        let mut siphasher = SipHasher::new_with_key(key);
        siphasher.write_u32(time);
        siphasher.write(serialized);
        siphasher.finish()
    }
}

pub trait Transmit: Sized {
    fn serialize(&self) -> Result<Vec<u8>, postcard::Error>;
}

impl<M: Serialize + DeserializeOwned> Transmit for M {
    fn serialize(&self) -> Result<Vec<u8>, postcard::Error> {
        let mut buf = [0u8; 1024 + 8];
        postcard::to_slice_cobs(self, &mut buf)
            .map(|s| s.to_vec())
    }
}
