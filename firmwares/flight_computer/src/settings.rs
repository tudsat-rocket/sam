//! Data structures for permanent settings stored in FC flash.

#[cfg(not(feature = "std"))]
use alloc::string::String;

use nalgebra::Vector3;
use serde::{Serialize, Deserialize};

use crate::telemetry::TelemetryDataRate;

/// Enum identifying takeoff detection source
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub enum TakeoffDetectionMode {
    #[default]
    /// use acceleration threshold only
    Acceleration,
    /// use breakwire via CAN only
    Breakwire,
    /// require both acceleration AND breakwire to trigger
    AccelerationAndBreakwire,
    /// require either acceleration OR breakwire to trigger
    AccelerationOrBreakwire,
}

/// Enum identifying main output trigger times
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub enum MainOutputMode {
    /// main output is triggered together with drogue at apogee
    AtApogee,
    /// main output is triggered after drogue, after descending below set altitude
    #[default]
    BelowAltitude,
    /// main output is disabled
    Never,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct LoRaSettings {
    /// which of the 14 500kHz LoRa channels from 863-870MHz to use
    pub channels: [bool; 14],
    /// binding phrase, used to generate LoRa FHSS sequence
    pub binding_phrase: String,
    /// key for uplink authentication
    pub authentication_key: u128
}

impl Default for LoRaSettings {
    fn default() -> Self {
        Self {
            channels: [true; 14],
            binding_phrase: "".into(),
            authentication_key: 0x00000000000000000000000000000000
        }
    }
}

/// Main Settings struct. Stored in flash using postcard (non-COBS) encoding.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Settings {
    /// unique name for this flight controller
    pub identifier: String,
    /// calibration offsets for gyroscope (deg/s)
    pub gyro_offset: Vector3<f32>,
    /// calibration offsets for main accelerometer (m/s^2)
    pub acc_offset: Vector3<f32>,
    /// calibration offsets for backup accelerometer (m/s^2)
    pub acc2_offset: Vector3<f32>,
    /// calibration offsets for magnetometer (uT)
    pub mag_offset: Vector3<f32>,
    /// proportional filter gain for Mahony attitude estimator
    pub mahony_kp: f32,
    /// integral filter gain for Mahony attitude estimator
    pub mahony_ki: f32,
    /// accelerometer standard deviation for kalman filter
    pub std_dev_accelerometer: f32,
    /// barometer standard deviation for kalman filter
    pub std_dev_barometer: f32,
    /// process standard deviation for kalman filter
    pub std_dev_process: f32,
    /// which data source to use for takeoff detection
    pub takeoff_detection_mode: TakeoffDetectionMode,
    /// minimum vertical acceleration for takeoff detection (m/s^2)
    pub min_takeoff_acc: f32,
    /// time for which min_takeoff_acc has to be exceeded (ms)
    pub min_takeoff_acc_time: u32,
    /// minimum time in flight to trigger apogee recovery (ms)
    pub min_time_to_apogee: u32,
    /// maximum time in flight to trigger apogee recovery (ms)
    pub max_time_to_apogee: u32,
    /// time vertical speed has to be negative for apogee recovery to start (ms)
    pub apogee_min_falling_time: u32,
    /// when to trigger main output
    pub main_output_mode: MainOutputMode,
    /// altitude below which main parachute is deployed (in mode BelowAltitude, m above ground)
    pub main_output_deployment_altitude: f32,
    /// duration of warning tone to play before enabling outputs (ms)
    pub outputs_warning_time: u32,
    /// time to enable recovery outputs (after warning tone, ms)
    pub outputs_high_time: u32,
    /// LoRa settings
    pub lora: LoRaSettings,
    /// Telemetry data rate
    pub default_data_rate: TelemetryDataRate,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            identifier: "Sting FC".into(),
            gyro_offset: Vector3::default(),
            acc_offset: Vector3::default(),
            acc2_offset: Vector3::default(),
            mag_offset: Vector3::default(),
            mahony_kp: 0.5,
            mahony_ki: 0.0,
            std_dev_accelerometer: 0.5,
            std_dev_barometer: 1.0,
            std_dev_process: 0.005,
            // TODO: unused
            takeoff_detection_mode: TakeoffDetectionMode::default(),
            min_takeoff_acc: 30.0,
            min_takeoff_acc_time: 50,
            min_time_to_apogee: 5000,
            max_time_to_apogee: 12000,
            apogee_min_falling_time: 1000,
            main_output_mode: MainOutputMode::default(),
            main_output_deployment_altitude: 450.0,
            // TODO: unused for actual sound duration
            outputs_warning_time: 500,
            outputs_high_time: 2000,
            lora: LoRaSettings::default(),
            default_data_rate: TelemetryDataRate::default(),
        }
    }
}
