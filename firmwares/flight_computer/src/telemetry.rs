#[cfg(not(feature = "std"))]
use alloc::string::String;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;

#[cfg(feature = "std")]
use std::string::String;
#[cfg(feature = "std")]
use std::vec::Vec;

use core::iter::Extend;

#[cfg(feature = "std")]
use bytes::BytesMut;
use crc::Crc;
use serde::{Deserialize, Serialize};

pub use LogLevel::*;

pub const X25: Crc<u16> = Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FlightMode {
    Idle,
    HardwareArmed,
    Armed,
    Flight,
    RecoveryDrogue,
    RecoveryMain,
    Landed,
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Idle
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum GPSFixType {
    NoFix,
    AutonomousFix,
    DifferentialFix,
    RTKFix,
    RTKFloat,
    DeadReckoningFix
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
    pub orientation: (f32, f32, f32, f32),
    pub vertical_speed: f32,
    pub altitude_baro: u16,
    pub altitude: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryState {
    pub time: u32,
    pub orientation: (f32, f32, f32, f32),
    pub gyroscope: (f32, f32, f32),
    pub acceleration: (f32, f32, f32),
    pub acceleration_world: (f32, f32, f32),
    pub vertical_speed: f32,
    pub altitude_baro: u16,
    pub altitude_gps: u16,
    pub altitude: u16,
    pub altitude_ground: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensors {
    pub time: u32,
    pub gyro: (f32, f32, f32),
    pub accelerometer1: (f32, f32, f32),
    pub accelerometer2: (f32, f32, f32),
    pub magnetometer: (f32, f32, f32),
    pub altitude_baro: f32,
    pub temperature_baro: f32,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryPower {
    pub time: u32,
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
pub struct TelemetryKalman {
    pub time: u32,
    // TODO
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryDiagnostics {
    pub time: u32,
    // loop runtime?
    pub loop_runtime: u16,
    pub temperature_core: u16,
    pub cpu_voltage: u16,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryGPS {
    pub time: u32,
    pub fix: GPSFixType,
    pub hdop: u16,
    pub num_satellites: u8,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude_asl: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialOrd, Ord, PartialEq, Eq, Serialize, Deserialize)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum DownlinkMessage {
    TelemetryMain(TelemetryMain),
    TelemetryState(TelemetryState),
    TelemetryRawSensors(TelemetryRawSensors),
    TelemetryPower(TelemetryPower),
    TelemetryKalman(TelemetryKalman),
    TelemetryDiagnostics(TelemetryDiagnostics),
    TelemetryGPS(TelemetryGPS),
    Log(u32, String, LogLevel, String),
}

impl DownlinkMessage {
    pub fn time(&self) -> u32 {
        match self {
            DownlinkMessage::TelemetryMain(tm) => tm.time,
            DownlinkMessage::TelemetryState(tm) => tm.time,
            DownlinkMessage::TelemetryRawSensors(tm) => tm.time,
            DownlinkMessage::TelemetryPower(tm) => tm.time,
            DownlinkMessage::TelemetryKalman(tm) => tm.time,
            DownlinkMessage::TelemetryDiagnostics(tm) => tm.time,
            DownlinkMessage::TelemetryGPS(tm) => tm.time,
            DownlinkMessage::Log(t, _, _, _) => *t
        }
    }
}

#[derive(PartialEq, Clone, Debug, Serialize, Deserialize)]
pub enum UplinkMessage { // TODO: attach HMAC?
    Heartbeat,
    Reboot,
    RebootToBootloader,
}

#[cfg(feature = "std")]
impl std::string::ToString for LogLevel {
    fn to_string(&self) -> String {
        match self {
            Debug => "DEBUG",
            Info => "INFO",
            Warning => "WARNING",
            Error => "ERROR",
            Critical => "CRITICAL",
        }.to_string()
    }
}

#[cfg(not(feature = "std"))]
impl alloc::string::ToString for LogLevel {
    fn to_string(&self) -> String {
        match self {
            Debug => "DEBUG",
            Info => "INFO",
            Warning => "WARNING",
            Error => "ERROR",
            Critical => "CRITICAL",
        }.to_string()
    }
}

pub fn wrap_msg(msg: &[u8], packet_counter: u16) -> Vec<u8> {
    // TODO: messages longer than 255?
    // TODO: packet counter size

    let mut wrapped: Vec<u8> = [
        0x42,
        (packet_counter >> 8) as u8,
        (packet_counter & 0xff) as u8,
        msg.len() as u8,
    ]
    .to_vec();
    wrapped.extend(msg);

    let mut digest = X25.digest();
    digest.update(&wrapped);
    let crc = digest.finalize();
    wrapped.push((crc >> 8) as u8);
    wrapped.push((crc & 0xff) as u8);

    wrapped
}

impl UplinkMessage {
    pub fn wrap(&self, packet_counter: u16) -> Vec<u8> {
        let mut buf = [0u8; 256];
        let serialized = postcard::to_slice(self, &mut buf).unwrap();
        wrap_msg(serialized, packet_counter)
    }
}

impl DownlinkMessage {
    pub fn wrap(&self, packet_counter: u16) -> Vec<u8> {
        let mut buf = [0u8; 512];
        let serialized = postcard::to_slice(self, &mut buf).unwrap();
        wrap_msg(serialized, packet_counter)
    }

    #[cfg(feature = "std")]
    pub fn read_valid_bytes(buf: &mut BytesMut) -> Option<(u16, Self)> {
        while buf.len() > 0 {
            if buf[0] == 0x42 {
                if buf.len() < 6 {
                    return None;
                }

                let len = buf[3] as usize;
                if buf.len() < 6 + len {
                    return None;
                }

                let crc = ((buf[len + 4] as u16) << 8) + (buf[len + 5] as u16);

                let mut digest = X25.digest();
                digest.update(&buf[0..(len + 4)]);
                let true_crc = digest.finalize();

                if crc == true_crc {
                    break;
                }
            }

            let _ = buf.split_to(1);
        }

        if buf.len() == 0 {
            return None;
        }

        let len = buf[3] as usize;
        let packet_counter = ((buf[1] as u16) << 8) + (buf[2] as u16);
        let result = postcard::from_bytes::<DownlinkMessage>(&buf[4..(len + 4)])
            .ok()
            .map(|x| (packet_counter, x));

        let _ = buf.split_to(len + 6);

        result
    }

    pub fn read_valid(buf: &mut Vec<u8>) -> Option<(u16, Self)> {
        while buf.len() > 0 {
            //#[cfg(feature = "std")]
            //println!("buf len: {:?}", buf.len());

            if buf[0] == 0x42 {
                if buf.len() < 6 {
                    return None;
                }

                let len = buf[3] as usize;
                if buf.len() < 6 + len {
                    return None;
                }

                //#[cfg(feature = "std")]
                //println!("{:02x?}", &buf[0..(len + 4)]);
                //println!("{:02x?}", &buf);

                let crc = ((buf[len + 4] as u16) << 8) + (buf[len + 5] as u16);

                let mut digest = X25.digest();
                digest.update(&buf[0..(len + 4)]);
                let true_crc = digest.finalize();

                //#[cfg(feature = "std")]
                //println!("{:?} {:?}", crc, true_crc);

                if crc == true_crc {
                    break;
                }
            }

            buf.remove(0);
        }

        if buf.len() == 0 {
            return None;
        }

        let len = buf[3] as usize;
        let packet_counter = ((buf[1] as u16) << 8) + (buf[2] as u16);
        let result = postcard::from_bytes::<DownlinkMessage>(&buf[4..(len + 4)])
            .ok()
            .map(|x| (packet_counter, x));

        for _i in 0..(len + 6) {
            buf.remove(0);
        }

        //if crc != true_crc {
        //    return None; // TODO: result
        //}

        result
    }
}
