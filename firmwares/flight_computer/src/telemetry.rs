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

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Serialize, Deserialize)]
pub enum TransmitPower {
    #[default]
    P14dBm = 0x00,
    P17dBm = 0x01,
    P20dBm = 0x02,
    P22dBm = 0x03,
}

impl From<u8> for TransmitPower {
    fn from(x: u8) -> Self {
        match x {
            0x00 => TransmitPower::P14dBm,
            0x01 => TransmitPower::P17dBm,
            0x02 => TransmitPower::P20dBm,
            0x03 => TransmitPower::P22dBm,
            _ => TransmitPower::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Serialize, Deserialize)]
pub enum TelemetryDataRate {
    #[default]
    Low = 0x0,
    High = 0x1,
}

impl From<u8> for TelemetryDataRate {
    fn from(x: u8) -> Self {
        match x {
            0x00 => TelemetryDataRate::Low,
            0x01 => TelemetryDataRate::High,
            _ => TelemetryDataRate::default(),
        }
    }
}

// contains everything that might be sent via telemetry or stored
#[derive(Clone, Default)]
pub struct VehicleState {
    pub time: u32,
    pub mode: Option<FlightMode>,

    pub orientation: Option<UnitQuaternion<f32>>,
    pub acceleration_world: Option<Vector3<f32>>,
    pub vertical_speed: Option<f32>,
    pub vertical_accel: Option<f32>,
    pub vertical_accel_filtered: Option<f32>,
    pub altitude_asl: Option<f32>,
    pub altitude_ground_asl: Option<f32>,
    pub apogee: Option<f32>,

    pub gyroscope: Option<Vector3<f32>>,
    pub accelerometer1: Option<Vector3<f32>>,
    pub accelerometer2: Option<Vector3<f32>>,
    pub magnetometer: Option<Vector3<f32>>,
    pub pressure_baro: Option<f32>,
    pub altitude_baro: Option<f32>,
    pub temperature_baro: Option<f32>,

    pub charge_voltage: Option<u16>,
    pub battery_voltage: Option<u16>,
    pub current: Option<i16>,

    // TODO: rename?
    pub lora_rssi: Option<u8>,
    pub transmit_power: Option<TransmitPower>,
    // TODO: data rate?

    pub cpu_utilization: Option<f32>,
    pub flash_pointer: Option<u32>,

    //pub fix_and_sats: u8,
    //pub hdop: u16,
    //pub latitude: [u8; 3],
    //pub longitude: [u8; 3],
    //pub altitude_gps_asl: u16,

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
    pub altitude_max: f32, // TODO: rename apogee
    pub altitude: f32,
}

impl From<VehicleState> for TelemetryMain {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            mode: vs.mode.unwrap_or_default(),
            orientation: vs.orientation,
            vertical_speed: vs.vertical_speed.unwrap_or_default(),
            vertical_accel: vs.vertical_accel.unwrap_or_default(),
            vertical_accel_filtered: vs.vertical_accel_filtered.unwrap_or_default(),
            altitude_baro: vs.altitude_baro.unwrap_or_default(),
            altitude_max: vs.apogee.unwrap_or_default(),
            altitude: vs.altitude_asl.unwrap_or_default()
        }
    }
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

impl From<VehicleState> for TelemetryMainCompressed {
    fn from(vs: VehicleState) -> Self {
        let quat = vs.orientation
            .clone()
            .map(|q| q.coords)
            .map(|q| {
                (
                    (127.0 + q.x * 127.0) as u8,
                    (127.0 + q.y * 127.0) as u8,
                    (127.0 + q.z * 127.0) as u8,
                    (127.0 + q.w * 127.0) as u8,
                )
            });

        Self {
            time: vs.time,
            mode: vs.mode.unwrap_or_default(),
            orientation: quat.unwrap_or((127, 127, 127, 127)),
            vertical_speed: vs.vertical_speed.map(|x| x * 10.0).unwrap_or_default().into(),
            vertical_accel: vs.vertical_accel.map(|x| x * 10.0).unwrap_or_default().into(),
            vertical_accel_filtered: vs.vertical_accel_filtered.map(|x| x * 10.0).unwrap_or_default().into(),
            altitude_baro: (vs.altitude_baro.unwrap_or_default() * 10.0 + 1000.0) as u16, // TODO: this limits us to 6km AMSL
            altitude: (vs.altitude_asl.unwrap_or_default() * 10.0 + 1000.0) as u16,
            altitude_max: (vs.apogee.unwrap_or_default() * 10.0 + 1000.0) as u16,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensors {
    pub time: u32,
    pub gyro: Vector3<f32>,
    pub accelerometer1: Vector3<f32>,
    pub accelerometer2: Vector3<f32>,
    pub magnetometer: Vector3<f32>,
    pub pressure_baro: f32,
}

impl From<VehicleState> for TelemetryRawSensors {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            gyro: vs.gyroscope.unwrap_or_default(),
            accelerometer1: vs.accelerometer1.unwrap_or_default(),
            accelerometer2: vs.accelerometer2.unwrap_or_default(),
            magnetometer: vs.magnetometer.unwrap_or_default(),
            pressure_baro: vs.pressure_baro.unwrap_or_default(),
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensorsCompressed {
    pub time: u32,
    pub gyro: CompressedVector3,
    pub accelerometer1: CompressedVector3,
    pub accelerometer2: CompressedVector3,
    pub magnetometer: CompressedVector3,
    pub pressure_baro: u16, // TODO: compress this further
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryDiagnostics {
    pub time: u32,
    pub cpu_utilization: u8,
    pub charge_voltage: u16,
    pub battery_voltage: u16,
    pub current: i16,
    pub lora_rssi: u8,
    pub altitude_ground: u16,
    pub transmit_power_and_data_rate: u8,
    pub temperature_baro: i8,
    pub recovery_drogue: [u8; 2],
    pub recovery_main: [u8; 2],
}

impl From<VehicleState> for TelemetryDiagnostics {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            cpu_utilization: (100.0 * vs.cpu_utilization.unwrap_or_default()) as u8,
            charge_voltage: vs.charge_voltage.unwrap_or_default(),
            battery_voltage: vs.battery_voltage.unwrap_or_default() << 2, // TODO: breakwire
            current: vs.current.unwrap_or_default(),
            lora_rssi: vs.lora_rssi.unwrap_or_default(),
            altitude_ground: (vs.altitude_ground_asl.unwrap_or_default() * 10.0 + 1000.0) as u16,
            // TODO: data rate?
            transmit_power_and_data_rate: vs.transmit_power.unwrap_or_default() as u8,
            temperature_baro: (vs.temperature_baro.unwrap_or_default() * 2.0) as i8,
            // TODO: remove recovery
            ..Default::default() // TODO
        }
    }
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

impl From<VehicleState> for TelemetryGPS {
    fn from(vs: VehicleState) -> Self {
        //let latitude = self.gps.latitude
        //    .map(|lat| ((lat.clamp(-90.0, 90.0) + 90.0) * 16777215.0 / 180.0) as u32)
        //    .map(|lat| [(lat >> 16) as u8, (lat >> 8) as u8, lat as u8])
        //    .unwrap_or([0, 0, 0]);
        //let longitude = self.gps.longitude
        //    .map(|lng| ((lng.clamp(-180.0, 180.0) + 180.0) * 16777215.0 / 360.0) as u32)
        //    .map(|lng| [(lng >> 16) as u8, (lng >> 8) as u8, lng as u8])
        //    .unwrap_or([0, 0, 0]);
        //let fix_and_sats = ((self.gps.fix.clone() as u8) << 5) + ((self.gps.num_satellites as u8) & 0x1f);

        // TODO
        TelemetryGPS {
            time: vs.time,
            fix_and_sats: 0,
            hdop: 0,
//            hdop: self.gps.hdop,
            latitude: [0, 0, 0],
            longitude: [0, 0, 0],
            altitude_asl: 0,
//            altitude_asl: self.gps.altitude.map(|alt| (alt * 10.0 + 1000.0) as u16).unwrap_or(u16::MAX),
            flash_pointer: (vs.flash_pointer.unwrap_or_default() / 1024) as u16,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryGCS {
    pub time: u32,
    pub lora_rssi: u8,
    pub lora_rssi_signal: u8,
    pub lora_snr: i8,
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
    SetTransmitPower(TransmitPower),
    SetDataRate(TelemetryDataRate),
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
