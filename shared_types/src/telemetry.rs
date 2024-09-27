//! Contains data structures that the flight computer shares with the outside world,
//! namely the ground station software, as well as common (de)serialization code.

#[cfg(target_os = "none")]
use alloc::string::{String, ToString};
#[cfg(target_os = "none")]
use alloc::vec::Vec;

#[cfg(not(target_os = "none"))]
use std::string::{String, ToString};
#[cfg(not(target_os = "none"))]
use std::vec::Vec;

#[cfg(not(target_os = "none"))]
use std::f32::consts::PI;

#[cfg(target_os = "none")]
use core::hash::Hasher;
#[cfg(not(target_os = "none"))]
use std::hash::Hasher;

use nalgebra::*;
use serde::{Deserialize, Serialize};
use serde::de::DeserializeOwned;
use siphasher::sip::SipHasher;

pub use crate::common::FlightMode;
use crate::settings::*;
use crate::can::*;

pub const LORA_MESSAGE_INTERVAL: u32 = 25;
pub const LORA_UPLINK_INTERVAL: u32 = 200;
pub const LORA_UPLINK_MODULO: u32 = 100;
pub const FLASH_SIZE: u32 = 32 * 1024 * 1024;
pub const FLASH_HEADER_SIZE: u32 = 4096; // needs to be multiple of 4096
pub const FLASH_SETTINGS_SIZE: u32 = 1024; // we don't need the full sector, so make the buffers smaller

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
            _ => Self::NoFix
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

#[derive(Clone, Default, Debug)]
#[allow(non_snake_case)]
pub struct SimulatedState {
    pub orientation: Option<UnitQuaternion<f32>>,
    pub vertical_accel: Option<f32>,
    pub vertical_speed: Option<f32>,
    pub euler_angles: Option<Vector3<f32>>,
    pub elevation: Option<f32>,
    pub azimuth: Option<f32>,
    pub kalman_x: OVector<f32, U9>,
    pub kalman_P: OVector<f32, U9>,
    pub kalman_R: OVector<f32, U6>,
    pub mass: Option<f32>,
    pub motor_mass: Option<f32>,
    pub thruster_propellant_mass: Option<f32>,
    pub force_drag: Option<Vector3<f32>>,
    pub force_thrust: Option<Vector3<f32>>,
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
    Manual = 0b10
}

impl TryFrom<u8> for AcsMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b00 => Ok(Self::Disabled),
            0b01 => Ok(Self::Auto),
            0b10 => Ok(Self::Manual),
            _ => Err(())
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
            _ => Err(())
        }
    }
}


// contains everything that might be sent via telemetry or stored
#[derive(Clone, Default, Debug)]
#[allow(non_snake_case)]
pub struct VehicleState {
    pub time: u32,
    pub mode: Option<FlightMode>,

    pub orientation: Option<UnitQuaternion<f32>>,
    pub acceleration_world: Option<Vector3<f32>>,
    pub ground_speed: Option<f32>,
    pub vertical_speed: Option<f32>,
    pub vertical_accel: Option<f32>,
    pub altitude_asl: Option<f32>,
    pub altitude_ground_asl: Option<f32>,
    pub apogee_asl: Option<f32>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,

    pub gyroscope: Option<Vector3<f32>>,
    pub accelerometer1: Option<Vector3<f32>>,
    pub accelerometer2: Option<Vector3<f32>>,
    pub magnetometer: Option<Vector3<f32>>,
    pub pressure_baro: Option<f32>,
    pub altitude_baro: Option<f32>,
    pub temperature_baro: Option<f32>,

    pub position_variance: Option<f32>,
    pub altitude_variance: Option<f32>,
    pub vertical_speed_variance: Option<f32>,
    pub barometer_variance: Option<f32>,
    pub accelerometer_variance: Option<f32>,
    pub gps_variance: Option<f32>,

    pub charge_voltage: Option<u16>,
    pub battery_voltage: Option<u16>,
    pub arm_voltage: Option<u16>,
    pub current: Option<i32>,

    // TODO: rename?
    pub lora_rssi: Option<u8>,
    pub transmit_power: Option<TransmitPower>,
    pub data_rate: Option<TelemetryDataRate>,

    pub cpu_utilization: Option<f32>,
    pub flash_pointer: Option<u32>,

    pub gps: Option<GPSDatum>,

    pub acs_voltage: Option<Option<u16>>,
    pub acs_current: Option<Option<i16>>,
    pub acs_temperature: Option<Option<i8>>,
    pub recovery_voltage: Option<Option<u16>>,
    pub recovery_current: Option<Option<i16>>,
    pub recovery_temperature: Option<Option<i8>>,
    pub payload_voltage: Option<Option<u16>>,
    pub payload_current: Option<Option<i16>>,
    pub payload_temperature: Option<Option<i8>>,
    pub fins_present: Option<[bool; 3]>,
    pub main_release_sensor: Option<bool>,

    pub acs_mode: Option<AcsMode>,
    pub thruster_valve_state: Option<ThrusterValveState>,

    pub camera_state: Option<[bool; 3]>,

    // all in bar
    pub acs_tank_pressure: Option<f32>,
    pub acs_regulator_pressure: Option<f32>,
    pub acs_accel_valve_pressure: Option<f32>,
    pub acs_decel_valve_pressure: Option<f32>,
    pub recovery_pressure: Option<f32>,

    // ground station data, only used by sam to correlate GCS value with vehicle measurements
    // TODO: exclude for firmware?
    pub gcs_lora_rssi: Option<u8>,
    pub gcs_lora_rssi_signal: Option<u8>,
    pub gcs_lora_snr: Option<i8>,

    // TODO: refactor this
    pub io_board_sensor_data: Option<(IoBoardRole, u8, IoBoardSensorMessage)>,
    pub io_board_power_data: Option<(IoBoardRole, IoBoardPowerMessage)>,
    pub fin_board_sensor_data: Option<(u8, u8, FinBoardDataMessage)>,

    // Computed values. These are just calculated from the orientation value, but are cached for
    // performance reasons. Since these are only needed by the ground station, we don't include
    // them when compiling for the flight computer.
    #[cfg(not(target_os = "none"))]
    pub euler_angles: Option<Vector3<f32>>,
    #[cfg(not(target_os = "none"))]
    pub elevation: Option<f32>,
    #[cfg(not(target_os = "none"))]
    pub azimuth: Option<f32>,

    #[cfg(not(target_os = "none"))]
    pub sim: Option<Box<SimulatedState>>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMain {
    pub time: u32,
    pub mode: FlightMode,
    pub orientation: Option<UnitQuaternion<f32>>,
    pub vertical_speed: half::f16,
    pub vertical_accel: half::f16,
    pub altitude_baro: u16,
    pub apogee_asl: u16,
    pub altitude_asl: u16,
    pub acs_mode: AcsMode,
    pub thruster_valve_state: ThrusterValveState,
    pub battery_current: i16,
}

impl From<VehicleState> for TelemetryMain {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            mode: vs.mode.unwrap_or_default(),
            orientation: vs.orientation,
            vertical_speed: half::f16::from_f32(vs.vertical_speed.unwrap_or_default()),
            vertical_accel: half::f16::from_f32(vs.vertical_accel.unwrap_or_default()),
            altitude_baro: (vs.altitude_baro.unwrap_or_default() * 10.0) as u16,
            apogee_asl: (vs.apogee_asl.unwrap_or_default() * 10.0) as u16,
            altitude_asl: (vs.altitude_asl.unwrap_or_default() * 10.0) as u16,
            acs_mode: vs.acs_mode.unwrap_or_default(),
            thruster_valve_state: vs.thruster_valve_state.unwrap_or_default(),
            battery_current: vs.current.unwrap_or_default() as i16,
        }
    }
}

impl Into<VehicleState> for TelemetryMain {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            mode: Some(self.mode),
            orientation: self.orientation,
            altitude_asl: Some(self.altitude_asl as f32 / 10.0),
            altitude_baro: Some(self.altitude_baro as f32 / 10.0),
            apogee_asl: Some(self.apogee_asl as f32 / 10.0),
            vertical_speed: Some(self.vertical_speed.to_f32()),
            vertical_accel: Some(self.vertical_accel.to_f32()),
            acs_mode: Some(self.acs_mode),
            thruster_valve_state: Some(self.thruster_valve_state),
            current: Some(self.battery_current as i32),

            #[cfg(not(target_os = "none"))]
            euler_angles: self.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI),
            #[cfg(not(target_os = "none"))]
            elevation: self.orientation.map(|q| {
                let up = Vector3::new(0.0, 0.0, 1.0);
                let attitude = q * up;
                90.0 - up.dot(&attitude).acos().to_degrees()
            }),
            #[cfg(not(target_os = "none"))]
            azimuth: self.orientation.map(|q| {
                let attitude = q * Vector3::new(0.0, 0.0, 1.0);
                (90.0 - attitude.y.atan2(attitude.x).to_degrees()).rem_euclid(360.0)
            }),

            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryFastCompressed {
    pub time: u32,
    pub flight_mode_and_valve_state: u8,
    pub azimuth: u8,
    pub elevation: i8,
    pub vertical_speed: i16, // could be optimized
    pub vertical_accel: i8,
    pub altitude_asl: u16,
    pub altitude_baro: i8, // offset from altitude_asl
    pub apogee_asl: u16,
    pub battery_current: i16,
}

#[cfg(not(target_abi="eabi"))]
impl From<VehicleState> for TelemetryFastCompressed {
    fn from(vs: VehicleState) -> Self {
        let mode = vs.mode.unwrap_or_default() as u8;
        let acs_mode = vs.acs_mode.unwrap_or_default() as u8;
        let valve_state = vs.thruster_valve_state.unwrap_or_default() as u8;
        let flight_mode_and_valve_state = (mode << 4) | (acs_mode << 2) | (valve_state & 0b11);

        let azimuth = vs.orientation
            .map(|q| {
                let attitude = q * Vector3::new(0.0, 0.0, 1.0);
                (90.0 - attitude.y.atan2(attitude.x).to_degrees()) % 360.0
            })
            .unwrap_or_default();
        let azimuth = if azimuth < 0.0 { azimuth + 360.0 } else { azimuth };
        let azimuth = (azimuth * 256.0 / 360.0) as u8;

        let elevation = vs.orientation
            .map(|q| {
                let up = Vector3::new(0.0, 0.0, 1.0);
                let attitude = q * up;
                90.0 - up.dot(&attitude).acos().to_degrees()
            })
            .unwrap_or_default();
        let elevation = (elevation * 128.0 / 90.0) as i8;

        let vertical_speed = vs.vertical_speed.map(|x| (x * 20.0) as i16).unwrap_or_default().into();
        let vertical_accel = vs.vertical_accel.map(|x| (x * 4.0) as i8).unwrap_or_default().into();

        let altitude_asl = (vs.altitude_asl.unwrap_or_default() * 10.0 + 1000.0) as u16; // TODO: this limits us to 6km AMSL
        let altitude_baro = ((vs.altitude_baro.unwrap_or_default() - vs.altitude_asl.unwrap_or_default()) * 2.0) as i8;
        let apogee_asl = (vs.apogee_asl.unwrap_or_default() * 10.0 + 1000.0) as u16;

        let battery_current = vs.current.unwrap_or_default() as i16;

        Self {
            time: vs.time,
            flight_mode_and_valve_state,
            azimuth,
            elevation,
            vertical_speed,
            vertical_accel,
            altitude_asl,
            altitude_baro,
            apogee_asl,
            battery_current,
        }
    }
}

#[cfg(not(target_os = "none"))]
impl Into<VehicleState> for TelemetryFastCompressed {
    fn into(self) -> VehicleState {
        let altitude_asl = ((self.altitude_asl as f32) - 1000.0) / 10.0;
        let altitude_baro_asl = altitude_asl + (self.altitude_baro as f32) / 2.0;

        VehicleState {
            time: self.time,
            mode: (self.flight_mode_and_valve_state >> 4).try_into().ok(),
            acs_mode: ((self.flight_mode_and_valve_state >> 2) & 0b11).try_into().ok(),
            thruster_valve_state: (self.flight_mode_and_valve_state & 0b11).try_into().ok(),
            azimuth: Some((self.azimuth as f32) * 360.0 / 256.0),
            elevation: Some((self.elevation as f32) * 90.0 / 128.0),
            vertical_speed: Some((self.vertical_speed as f32) / 20.0),
            vertical_accel: Some((self.vertical_speed as f32) / 4.0),
            altitude_asl: Some(altitude_asl),
            altitude_baro: Some(altitude_baro_asl),
            apogee_asl: Some(((self.apogee_asl as f32) - 1000.0) / 10.0),
            current: Some(self.battery_current as i32),

            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryRawSensors {
    pub time: u32,
    pub gyro: (half::f16, half::f16, half::f16),
    pub accelerometer1: (half::f16, half::f16, half::f16),
    pub accelerometer2: (half::f16, half::f16, half::f16),
    pub magnetometer: (half::f16, half::f16, half::f16),
    pub pressure_baro: f32,
}

impl From<VehicleState> for TelemetryRawSensors {
    fn from(vs: VehicleState) -> Self {
        let gyro = vs.gyroscope.unwrap_or_default();
        let acc1 = vs.accelerometer1.unwrap_or_default();
        let acc2 = vs.accelerometer2.unwrap_or_default();
        let mag = vs.magnetometer.unwrap_or_default();

        Self {
            time: vs.time,
            gyro: (half::f16::from_f32(gyro.x), half::f16::from_f32(gyro.y), half::f16::from_f32(gyro.z)),
            accelerometer1: (half::f16::from_f32(acc1.x), half::f16::from_f32(acc1.y), half::f16::from_f32(acc1.z)),
            accelerometer2: (half::f16::from_f32(acc2.x), half::f16::from_f32(acc2.y), half::f16::from_f32(acc2.z)),
            magnetometer: (half::f16::from_f32(mag.x), half::f16::from_f32(mag.y), half::f16::from_f32(mag.z)),
            pressure_baro: vs.pressure_baro.unwrap_or_default(),
        }
    }
}

impl Into<VehicleState> for TelemetryRawSensors {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            gyroscope: Some(Vector3::new(self.gyro.0.to_f32(), self.gyro.1.to_f32(), self.gyro.2.to_f32())),
            accelerometer1: Some(Vector3::new(self.accelerometer1.0.to_f32(), self.accelerometer1.1.to_f32(), self.accelerometer1.2.to_f32())),
            accelerometer2: Some(Vector3::new(self.accelerometer2.0.to_f32(), self.accelerometer2.1.to_f32(), self.accelerometer2.2.to_f32())),
            magnetometer: Some(Vector3::new(self.magnetometer.0.to_f32(), self.magnetometer.1.to_f32(), self.magnetometer.2.to_f32())),
            pressure_baro: Some(self.pressure_baro),
            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryDiagnostics {
    pub time: u32,
    pub cpu_utilization: u8,
    pub charge_voltage: u16,
    pub battery_voltage: u16,
    pub lora_rssi: u8,
    pub altitude_ground_asl: u16,
    pub transmit_power_and_data_rate: u8,
    pub temperature_baro: i8,
}

impl From<VehicleState> for TelemetryDiagnostics {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            cpu_utilization: (100.0 * vs.cpu_utilization.unwrap_or_default()) as u8,
            charge_voltage: vs.charge_voltage.unwrap_or_default(),
            battery_voltage: vs.battery_voltage.unwrap_or_default() << 2, // TODO: breakwire
            lora_rssi: vs.lora_rssi.unwrap_or_default(),
            altitude_ground_asl: (vs.altitude_ground_asl.unwrap_or_default() * 10.0 + 1000.0) as u16,
            transmit_power_and_data_rate: ((vs.data_rate.unwrap_or_default() as u8) << 7) |
                vs.transmit_power.unwrap_or_default() as u8,
            temperature_baro: (vs.temperature_baro.unwrap_or_default() * 2.0) as i8,
            ..Default::default() // TODO
        }
    }
}

impl Into<VehicleState> for TelemetryDiagnostics {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            altitude_ground_asl: Some((self.altitude_ground_asl as f32 - 1000.0) / 10.0),
            battery_voltage: Some(self.battery_voltage >> 2),
            charge_voltage: Some(self.charge_voltage),
            cpu_utilization: Some(self.cpu_utilization as f32),
            lora_rssi: Some(self.lora_rssi),
            data_rate: Some((self.transmit_power_and_data_rate >> 7).into()),
            transmit_power: Some((self.transmit_power_and_data_rate & 0x7f).into()),
            temperature_baro: Some((self.temperature_baro as f32) / 2.0),
            ..Default::default()
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
        let gps_latitude = vs.gps.as_ref().and_then(|gps| gps.latitude);
        let gps_longitude = vs.gps.as_ref().and_then(|gps| gps.longitude);

        let fix = vs.gps.as_ref().map(|gps| gps.fix).clone().unwrap_or_default();
        let num_satellites = vs.gps.as_ref().map(|gps| gps.num_satellites).unwrap_or(0);
        let fix_and_sats = ((fix as u8) << 5) + ((num_satellites as u8) & 0x1f);

        let latitude = if num_satellites >= 6 { gps_latitude } else { vs.latitude };
        let longitude = if num_satellites >= 6 { gps_longitude } else { vs.longitude };

        let latitude = latitude
            .map(|lat| ((lat.clamp(-90.0, 90.0) + 90.0) * 16777215.0 / 180.0) as u32)
            .map(|lat| [(lat >> 16) as u8, (lat >> 8) as u8, lat as u8])
            .unwrap_or([0, 0, 0]);
        let longitude = longitude
            .map(|lng| ((lng.clamp(-180.0, 180.0) + 180.0) * 16777215.0 / 360.0) as u32)
            .map(|lng| [(lng >> 16) as u8, (lng >> 8) as u8, lng as u8])
            .unwrap_or([0, 0, 0]);

        TelemetryGPS {
            time: vs.time,
            fix_and_sats,
            hdop: vs.gps.as_ref().map(|gps| gps.hdop).unwrap_or(u16::MAX),
            latitude,
            longitude,
            altitude_asl: vs.gps.and_then(|gps| gps.altitude).map(|alt| (alt * 10.0 + 1000.0) as u16).unwrap_or(u16::MAX),
            flash_pointer: (vs.flash_pointer.unwrap_or_default() / 1024) as u16,
        }
    }
}

impl Into<VehicleState> for TelemetryGPS {
    fn into(self) -> VehicleState {
        let num_satellites = self.fix_and_sats & 0x1f;

        let lat = ((self.latitude[0] as u32) << 16) + ((self.latitude[1] as u32) << 8) + (self.latitude[2] as u32);
        let latitude = (lat > 0).then(|| lat).map(|lat| (lat as f32) * 180.0 / 16777215.0 - 90.0);

        let lng = ((self.longitude[0] as u32) << 16) + ((self.longitude[1] as u32) << 8) + (self.longitude[2] as u32);
        let longitude = (lng > 0).then(|| lng).map(|lng| (lng as f32) * 360.0 / 16777215.0 - 180.0);

        let gps = GPSDatum {
            utc_time: None,
            fix: (self.fix_and_sats >> 5).into(),
            num_satellites: self.fix_and_sats & 0x1f,
            hdop: self.hdop,
            latitude: (num_satellites >= 6).then_some(latitude).flatten(),
            longitude: (num_satellites >= 6).then_some(longitude).flatten(),
            altitude: (self.altitude_asl != u16::MAX).then(|| (self.altitude_asl as f32 - 1000.0) / 10.0),
        };

        VehicleState {
            time: self.time,
            latitude,
            longitude,
            gps: Some(gps),
            flash_pointer: Some((self.flash_pointer as u32) * 1024),
            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryPressures {
    pub time: u32,
    pub acs_tank_pressure: Option<u16>,
    pub acs_regulator_pressure: Option<u16>,
    pub acs_accel_valve_pressure: Option<u16>,
    pub acs_decel_valve_pressure: Option<u16>,
    pub recovery_pressure: Option<u16>,
}

impl From<VehicleState> for TelemetryPressures {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            acs_tank_pressure: vs.acs_tank_pressure.map(|p| (p * 100.0) as u16),
            acs_regulator_pressure: vs.acs_regulator_pressure.map(|p| (p * 1000.0) as u16),
            acs_accel_valve_pressure: vs.acs_accel_valve_pressure.map(|p| (p * 1000.0) as u16),
            acs_decel_valve_pressure: vs.acs_decel_valve_pressure.map(|p| (p * 1000.0) as u16),
            recovery_pressure: vs.recovery_pressure.map(|p| (p * 1000.0) as u16),
        }
    }
}

impl Into<VehicleState> for TelemetryPressures {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            acs_tank_pressure: self.acs_tank_pressure.map(|p| (p as f32) / 100.0),
            acs_regulator_pressure: self.acs_regulator_pressure.map(|p| (p as f32) / 1000.0),
            acs_accel_valve_pressure: self.acs_accel_valve_pressure.map(|p| (p as f32) / 1000.0),
            acs_decel_valve_pressure: self.acs_decel_valve_pressure.map(|p| (p as f32) / 1000.0),
            recovery_pressure: self.recovery_pressure.map(|p| (p as f32) / 1000.0),

            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryKalman {
    pub time: u32,
    pub ground_speed: u16,
    pub position_variance: f32,
    pub altitude_variance: u16,
    pub vertical_speed_variance: u16,
    pub barometer_variance: u16,
    pub gps_variance: f32,
}

impl From<VehicleState> for TelemetryKalman {
    fn from(vs: VehicleState) -> Self {
        Self {
            time: vs.time,
            ground_speed: (vs.ground_speed.unwrap_or_default() * 10.0) as u16,
            position_variance: vs.position_variance.unwrap_or_default(),
            altitude_variance: (vs.altitude_variance.unwrap_or_default() * 50.0) as u16,
            vertical_speed_variance: (vs.vertical_speed_variance.unwrap_or_default() * 100.0) as u16,
            barometer_variance: (vs.barometer_variance.unwrap_or_default() * 10.0) as u16,
            gps_variance: vs.gps_variance.unwrap_or_default(),
        }
    }
}

impl Into<VehicleState> for TelemetryKalman {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            ground_speed: Some((self.ground_speed as f32) / 10.0),
            position_variance: Some(self.position_variance),
            altitude_variance: Some((self.altitude_variance as f32) / 50.0),
            vertical_speed_variance: Some((self.vertical_speed_variance as f32) / 100.0),
            barometer_variance: Some((self.barometer_variance as f32) / 10.0),
            gps_variance: Some(self.gps_variance),

            ..Default::default()
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryBus {
    pub time: u32,
    pub module_presence: u8,
    pub voltages: [u16; 2],
    pub currents: [i16; 2],
    pub temperatures: [i8; 2],
    pub camera_state: u8,
}

impl From<VehicleState> for TelemetryBus {
    fn from(vs: VehicleState) -> Self {
        let acs_present = vs.acs_voltage.map(|o| o.is_some()).unwrap_or(false);
        let recovery_present = vs.recovery_voltage.map(|o| o.is_some()).unwrap_or(false);
        let payload_present = vs.payload_voltage.map(|o| o.is_some()).unwrap_or(false);
        let fins_present = vs.fins_present.unwrap_or_default();

        let module_presence = ((acs_present as u8) << 7) |
            ((recovery_present as u8) << 6) |
            ((payload_present as u8) << 5) |
            ((fins_present[0] as u8) << 2) |
            ((fins_present[1] as u8) << 1) |
            (fins_present[2] as u8);

        let camera_state = vs.camera_state.unwrap_or_default();
        let camera_state = ((camera_state[0] as u8) << 7) |
            ((camera_state[1] as u8) << 6) |
            ((camera_state[2] as u8) << 5) |
            (vs.main_release_sensor.unwrap_or_default() as u8);

        if payload_present {
            Self {
                time: vs.time,
                module_presence,
                voltages: [vs.payload_voltage.unwrap_or_default().unwrap_or_default(), 0],
                currents: [vs.payload_current.unwrap_or_default().unwrap_or_default(), 0],
                temperatures: [vs.payload_temperature.unwrap_or_default().unwrap_or_default(), 0],
                camera_state
            }
        } else {
            let voltages = [vs.acs_voltage.unwrap_or_default().unwrap_or_default(), vs.recovery_voltage.unwrap_or_default().unwrap_or_default()];
            let currents = [vs.acs_current.unwrap_or_default().unwrap_or_default(), vs.recovery_current.unwrap_or_default().unwrap_or_default()];
            let temperatures = [vs.acs_temperature.unwrap_or_default().unwrap_or_default(), vs.recovery_temperature.unwrap_or_default().unwrap_or_default()];

            Self {
                time: vs.time,
                module_presence,
                voltages,
                currents,
                temperatures,
                camera_state
            }
        }
    }
}

impl Into<VehicleState> for TelemetryBus {
    fn into(self) -> VehicleState {
        let acs_present = (self.module_presence >> 7) > 0;
        let recovery_present = ((self.module_presence >> 6) & 0b1) > 0;
        let payload_present = ((self.module_presence >> 5) & 0b1) > 0;
        let fins_present = [
            ((self.module_presence >> 2) & 0b1) > 0,
            ((self.module_presence >> 1) & 0b1) > 0,
            ((self.module_presence >> 0) & 0b1) > 0
        ];
        let camera_state = [
            ((self.camera_state >> 7) & 0b1) > 0,
            ((self.camera_state >> 6) & 0b1) > 0,
            ((self.camera_state >> 5) & 0b1) > 0
        ];
        let main_release = recovery_present.then_some((self.camera_state & 0b1) > 0);

        if payload_present {
            VehicleState {
                time: self.time,
                acs_voltage: Some(None),
                acs_current: Some(None),
                acs_temperature: Some(None),
                recovery_voltage: Some(None),
                recovery_current: Some(None),
                recovery_temperature: Some(None),
                payload_voltage: Some(Some(self.voltages[0])),
                payload_current: Some(Some(self.currents[0])),
                payload_temperature: Some(Some(self.temperatures[0])),
                fins_present: Some(fins_present),
                camera_state: Some(camera_state),
                ..Default::default()
            }
        } else {
            VehicleState {
                time: self.time,
                acs_voltage: Some(acs_present.then_some(self.voltages[0])),
                acs_current: Some(acs_present.then_some(self.currents[0])),
                acs_temperature: Some(acs_present.then_some(self.temperatures[0])),
                recovery_voltage: Some(recovery_present.then_some(self.voltages[1])),
                recovery_current: Some(recovery_present.then_some(self.currents[1])),
                recovery_temperature: Some(recovery_present.then_some(self.temperatures[1])),
                payload_voltage: Some(None),
                payload_current: Some(None),
                payload_temperature: Some(None),
                fins_present: Some(fins_present),
                camera_state: Some(camera_state),
                main_release_sensor: main_release,
                ..Default::default()
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TelemetryCanBusMessage {
    pub time: u32,
    pub msg: FcReceivedCanBusMessage,
}

impl Into<VehicleState> for TelemetryCanBusMessage {
    fn into(self) -> VehicleState {
        #[cfg(not(target_os = "none"))]
        println!("{:?}", self);

        let mut vs = VehicleState {
            time: self.time,
            ..Default::default()
        };

        match self.msg {
            FcReceivedCanBusMessage::IoBoardSensor(role, id, msg) => {
                vs.io_board_sensor_data = Some((role, id, msg));
            }
            FcReceivedCanBusMessage::IoBoardPower(role, msg) => {
                vs.io_board_power_data = Some((role, msg));
            }
            FcReceivedCanBusMessage::FinBoardData(fin, id, msg) => {
                vs.fin_board_sensor_data = Some((fin, id, msg));
            }
            FcReceivedCanBusMessage::BatteryTelemetry(_, _) => {}
        }

        vs
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TelemetryGCS {
    pub time: u32,
    pub lora_rssi: u8,
    pub lora_rssi_signal: u8,
    pub lora_snr: i8,
}

impl Into<VehicleState> for TelemetryGCS {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time,
            gcs_lora_rssi: Some(self.lora_rssi),
            gcs_lora_rssi_signal: Some(self.lora_rssi_signal),
            gcs_lora_snr: Some(self.lora_snr),
            ..Default::default()
        }
    }
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
    TelemetryFastCompressed(TelemetryFastCompressed),
    TelemetryRawSensors(TelemetryRawSensors),
    TelemetryDiagnostics(TelemetryDiagnostics),
    TelemetryGPS(TelemetryGPS),
    TelemetryPressures(TelemetryPressures),
    TelemetryKalman(TelemetryKalman),
    TelemetryBus(TelemetryBus),
    TelemetryCanBusMessage(TelemetryCanBusMessage),
    TelemetryGCS(TelemetryGCS),
    FlashContent(u32, heapless::Vec<u8, 256>),
    Settings(Settings)
}

impl DownlinkMessage {
    pub fn time(&self) -> u32 {
        match self {
            DownlinkMessage::TelemetryMain(tm) => tm.time,
            DownlinkMessage::TelemetryFastCompressed(tm) => tm.time,
            DownlinkMessage::TelemetryRawSensors(tm) => tm.time,
            //DownlinkMessage::TelemetryRawSensorsCompressed(tm) => tm.time,
            DownlinkMessage::TelemetryDiagnostics(tm) => tm.time,
            DownlinkMessage::TelemetryGPS(tm) => tm.time,
            DownlinkMessage::TelemetryPressures(tm) => tm.time,
            DownlinkMessage::TelemetryKalman(tm) => tm.time,
            DownlinkMessage::TelemetryBus(tm) => tm.time,
            DownlinkMessage::TelemetryCanBusMessage(tm) => tm.time,
            DownlinkMessage::TelemetryGCS(tm) => tm.time,
            DownlinkMessage::FlashContent(_, _) => 0,
            DownlinkMessage::Settings(_) => 0,
        }
    }
}

#[cfg(not(target_os = "none"))]
impl From<DownlinkMessage> for VehicleState {
    fn from(msg: DownlinkMessage) -> VehicleState {
        match msg {
            DownlinkMessage::TelemetryMain(tm) => tm.into(),
            DownlinkMessage::TelemetryFastCompressed(tm) => tm.into(),
            DownlinkMessage::TelemetryRawSensors(tm) => tm.into(),
            //DownlinkMessage::TelemetryRawSensorsCompressed(tm) => tm.into(),
            DownlinkMessage::TelemetryDiagnostics(tm) => tm.into(),
            DownlinkMessage::TelemetryGPS(tm) => tm.into(),
            DownlinkMessage::TelemetryPressures(tm) => tm.into(),
            DownlinkMessage::TelemetryKalman(tm) => tm.into(),
            DownlinkMessage::TelemetryBus(tm) => tm.into(),
            DownlinkMessage::TelemetryCanBusMessage(tm) => tm.into(),
            DownlinkMessage::TelemetryGCS(tm) => tm.into(),
            DownlinkMessage::FlashContent(..) | DownlinkMessage::Settings(..) => Default::default(),
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
    fn serialize(&self) -> Result<Vec<u8>, postcard::Error>;
}

impl<M: Serialize + DeserializeOwned> Transmit for M {
    fn serialize(&self) -> Result<Vec<u8>, postcard::Error> {
        let mut buf = [0u8; 1024 + 8];
        postcard::to_slice_cobs(self, &mut buf)
            .map(|s| s.to_vec())
    }
}
