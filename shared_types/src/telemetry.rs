//! Contains data structures that the flight computer shares with the outside world,
//! namely the ground station software, as well as common (de)serialization code.

use heapless::{String, Vec};

#[cfg(target_os = "none")]
use core::hash::Hasher;
#[cfg(not(target_os = "none"))]
use std::hash::Hasher;

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use siphasher::sip::SipHasher;

use crate::can::*;
pub use crate::common::*;
use crate::settings::*;

pub const LORA_MESSAGE_INTERVAL: u32 = 25;
pub const LORA_UPLINK_INTERVAL: u32 = 200;
pub const LORA_UPLINK_MODULO: u32 = 100;
pub const FLASH_SIZE: u32 = 32 * 1024 * 1024;
pub const FLASH_HEADER_SIZE: u32 = 4096; // needs to be multiple of 4096
pub const FLASH_SETTINGS_SIZE: u32 = 1024; // we don't need the full sector, so make the buffers smaller

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
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
            _ => Self::NoFix,
        }
    }
}

impl TryFrom<&str> for GPSFixType {
    type Error = ();

    fn try_from(x: &str) -> Result<Self, Self::Error> {
        match x {
            "0" => Ok(GPSFixType::NoFix),
            "1" => Ok(GPSFixType::AutonomousFix),
            "2" => Ok(GPSFixType::DifferentialFix),
            "4" => Ok(GPSFixType::RTKFix),
            "5" => Ok(GPSFixType::RTKFloat),
            "6" => Ok(GPSFixType::DeadReckoningFix),
            _ => Err(()),
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

#[allow(dead_code)]
#[derive(Clone, Default, Debug)]
pub struct GPSDatum {
    pub utc_time: Option<u64>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude: Option<f32>,
    pub fix: GPSFixType,
    pub hdop: u16,
    pub num_satellites: u8,
}

#[derive(Clone, Copy, Default, Debug, PartialEq, Serialize, Deserialize)]
pub enum AcsMode {
    #[default]
    Disabled = 0b00,
    Auto = 0b01,
    Manual = 0b10,
}

impl TryFrom<u8> for AcsMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b00 => Ok(Self::Disabled),
            0b01 => Ok(Self::Auto),
            0b10 => Ok(Self::Manual),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Default, Debug, PartialEq, Serialize, Deserialize)]
pub enum ThrusterValveState {
    #[default]
    Closed = 0b00,
    OpenAccel = 0b10,
    OpenDecel = 0b01,
    OpenBoth = 0b11,
}

impl Into<f32> for ThrusterValveState {
    fn into(self) -> f32 {
        match self {
            Self::OpenAccel => 1.0,
            Self::OpenDecel => -1.0,
            _ => 0.0,
        }
    }
}

impl TryFrom<u8> for ThrusterValveState {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b00 => Ok(Self::Closed),
            0b10 => Ok(Self::OpenAccel),
            0b01 => Ok(Self::OpenDecel),
            0b11 => Ok(Self::OpenBoth),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Serialize, Deserialize)]
pub enum BatteryChargerState {
    #[default]
    NotCharging = 0b00,
    Charging = 0b01,
    RecoverableFault = 0b10,
    NonRecoverableFault = 0b11,
}

impl TryFrom<u8> for BatteryChargerState {
    type Error = ();

    fn try_from(x: u8) -> Result<Self, Self::Error> {
        match x {
            0b00 => Ok(Self::NotCharging),
            0b01 => Ok(Self::Charging),
            0b10 => Ok(Self::RecoverableFault),
            0b11 => Ok(Self::NonRecoverableFault),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
// TODO: metrics for this
pub struct TelemetryGCS {
    pub time: u32,
    pub lora_rssi: u8,
    pub lora_rssi_signal: u8,
    pub lora_snr: i8,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum DownlinkMessage {
    Telemetry(u32, heapless::Vec<u8, 256>),
    TelemetryGCS(TelemetryGCS),
    FlashContent(u32, heapless::Vec<u8, 256>),
    Settings(Settings),
}

impl DownlinkMessage {
    pub fn time(&self) -> u32 {
        match self {
            DownlinkMessage::Telemetry(time, _) => *time,
            DownlinkMessage::TelemetryGCS(tm) => tm.time,
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
    SetAcsMode(AcsMode),
    SetAcsValveState(ThrusterValveState),
    SetIoModuleOutput(IoBoardRole, u8, bool),
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
    fn serialize(&self) -> Result<Vec<u8, 1032>, postcard::Error>;
}

impl<M: Serialize + DeserializeOwned> Transmit for M {
    fn serialize(&self) -> Result<Vec<u8, 1032>, postcard::Error> {
        let mut buf = [0u8; 1024 + 8];
        postcard::to_slice_cobs(self, &mut buf).map(|s| Vec::from_slice(s).unwrap())
    }
}
