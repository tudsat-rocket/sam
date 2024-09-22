//! Data structures for permanent settings stored in FC flash.

use nalgebra::Vector3;
use serde::{Serialize, Deserialize};

use crate::telemetry::TelemetryDataRate;

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
    pub binding_phrase: heapless::String<64>,
    /// key for uplink authentication
    pub authentication_key: u128
}

impl Default for LoRaSettings {
    fn default() -> Self {
        Self {
            channels: [true; 14],
            binding_phrase: heapless::String::new(),
            authentication_key: 0x00000000000000000000000000000000
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum Orientation {
    ZUp,
    ZDown,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RecoveryOutputSettings {
    /// time to enable recovery outputs (after warning tone, ms)
    pub output_high_time: u32,
    /// time delay between recovery output pulses (ms)
    pub output_low_time: u32,
    /// number of recovery output pulses
    pub num_pulses: u32,
    // TODO: unused for actual sound duration
    /// duration of warning tone to play before enabling outputs (ms)
    pub output_warning_time: u32,
    /// Warning sound frequency
    pub output_warning_frequency: f32,
}

impl RecoveryOutputSettings {
    pub fn total_duration(&self) -> u32 {
        self.output_warning_time +
            self.num_pulses * self.output_high_time +
            (self.num_pulses - 1) * self.output_low_time
    }

    pub fn currently_high(&self, time_in_mode: u32) -> bool {
        let phase_duration = self.output_high_time + self.output_low_time;
        time_in_mode > self.output_warning_time
            && time_in_mode < self.total_duration()
            && (time_in_mode - self.output_warning_time) % phase_duration < self.output_high_time
    }
}

impl Default for RecoveryOutputSettings {
    fn default() -> Self {
        Self {
            output_high_time: 500,
            output_low_time: 0,
            num_pulses: 1,
            output_warning_time: 500,
            output_warning_frequency: 500.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PressureSensorCalibrationSettings {
    pub intercept: f32,
    pub slope: f32,
}

impl Default for PressureSensorCalibrationSettings {
    fn default() -> Self {
        Self { intercept: 0.0, slope: 1.0 }
    }
}

impl PressureSensorCalibrationSettings {
    pub fn apply(&self, raw_adc_value: Option<(u16, bool)>) -> Option<f32> {
        let raw_adc = raw_adc_value.and_then(|(v, alert)| (!alert).then_some(v));
        let mv = raw_adc.map(|adc| (adc as f32) * 3300f32 / 1024f32);
        mv.map(|mv| (mv - self.intercept) / self.slope)
    }
}

/// Main Settings struct. Stored in flash using postcard (non-COBS) encoding.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Settings {
    /// unique name for this flight controller
    pub identifier: heapless::String<32>,
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
    /// proportional filter gain for Mahony attitude estimator
    pub mahony_kp_ascent: f32,
    /// integral filter gain for Mahony attitude estimator
    pub mahony_ki_ascent: f32,
    /// accelerometer standard deviation for kalman filter
    pub std_dev_accelerometer: f32,
    /// barometer standard deviation for kalman filter
    pub std_dev_barometer: f32,
    /// barometer standard deviation for kalman filter when in the transsonic region
    pub std_dev_barometer_transsonic: f32,
    /// process standard deviation for kalman filter
    pub std_dev_process: f32,
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
    /// output settings for drogue deployment
    pub drogue_output_settings: RecoveryOutputSettings,
    /// output settings for main deployment
    pub main_output_settings: RecoveryOutputSettings,
    /// LoRa settings
    pub lora: LoRaSettings,
    /// Telemetry data rate
    pub default_data_rate: TelemetryDataRate,
    /// Time after drogue deployment in which main deployment will not be triggered
    pub min_time_to_main: u32,
    /// Mounting orientation of the flight computer
    pub orientation: Orientation,
    ///
    pub acs_tank_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_regulator_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_accel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_decel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub recovery_pressure_sensor_settings: PressureSensorCalibrationSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            identifier: "Sting FC".try_into().unwrap(),
            gyro_offset: Vector3::default(),
            acc_offset: Vector3::default(),
            acc2_offset: Vector3::default(),
            mag_offset: Vector3::default(),
            mahony_kp: 0.1,
            mahony_ki: 0.0,
            mahony_kp_ascent: 0.1,
            mahony_ki_ascent: 0.0,
            std_dev_accelerometer: 0.5,
            std_dev_barometer: 1.0,
            std_dev_barometer_transsonic: 400.0,
            std_dev_process: 0.1,
            min_takeoff_acc: 30.0,
            min_takeoff_acc_time: 50,
            min_time_to_apogee: 5000,
            max_time_to_apogee: 40000,
            apogee_min_falling_time: 100,
            main_output_mode: MainOutputMode::default(),
            main_output_deployment_altitude: 450.0,
            drogue_output_settings: RecoveryOutputSettings::default(),
            main_output_settings: RecoveryOutputSettings::default(),
            lora: LoRaSettings::default(),
            default_data_rate: TelemetryDataRate::default(),
            min_time_to_main: 1000,
            orientation: Orientation::ZUp,
            acs_tank_pressure_sensor_settings: PressureSensorCalibrationSettings::default(),
            acs_regulator_pressure_sensor_settings: PressureSensorCalibrationSettings::default(),
            acs_accel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings::default(),
            acs_decel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings::default(),
            recovery_pressure_sensor_settings: PressureSensorCalibrationSettings::default(),
        }
    }
}
