//! Data structures for permanent settings stored in FC flash.

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

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

//
// --- Lora settings ---
//
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum LoraCodingRate {
    _4_5,
    _4_6,
    _4_7,
    _4_8,
}
impl LoraCodingRate {
    pub fn default_downlink() -> Self {
        LoraCodingRate::_4_8
    }
    pub fn default_uplink() -> Self {
        LoraCodingRate::_4_8
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum LoraSpreadingFactor {
    _5,
    _6,
    _7,
    _8,
    _9,
    _10,
    _11,
    _12,
}
impl LoraSpreadingFactor {
    pub fn default_downlink() -> Self {
        LoraSpreadingFactor::_7
    }
    pub fn default_uplink() -> Self {
        LoraSpreadingFactor::_7
    }
}
/*
impl ToString for LoraSpreadingFactor {
    fn to_string(&self) -> String {
        match self {
            LoraSpreadingFactor::_5 => String::from("5"),
            LoraSpreadingFactor::_6 => String::from("6"),
            LoraSpreadingFactor::_7 => String::from("7"),
            LoraSpreadingFactor::_8 => String::from("8"),
            LoraSpreadingFactor::_9 => String::from("9"),
            LoraSpreadingFactor::_10 => String::from("10"),
            LoraSpreadingFactor::_11 => String::from("11"),
            LoraSpreadingFactor::_12 => String::from("12"),
        }
    }
}
*/
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum LoraBandwidth {
    _7KHz,
    _10KHz,
    _15KHz,
    _20KHz,
    _31KHz,
    _41KHz,
    _62KHz,
    _125KHz,
    _250KHz,
    _500KHz,
}
impl LoraBandwidth {
    pub fn default_downlink() -> Self {
        LoraBandwidth::_500KHz
    }
    pub fn default_uplink() -> Self {
        LoraBandwidth::_250KHz
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
/// Settings for specific lora link, must be the same on both rocket and gcs to work.
pub struct LoraLinkSettings {
    // TODO: restrict output power to useful values
    pub tx_power: u16,
    pub spreading_factor: LoraSpreadingFactor,
    pub bandwidth: LoraBandwidth,
    pub coding_rate: LoraCodingRate,
}
impl LoraLinkSettings {
    pub fn default_downlink() -> Self {
        Self {
            tx_power: 10,
            spreading_factor: LoraSpreadingFactor::default_downlink(),
            bandwidth: LoraBandwidth::default_downlink(),
            coding_rate: LoraCodingRate::default_downlink(),
        }
    }
    pub fn default_uplink() -> Self {
        Self {
            tx_power: 10,
            spreading_factor: LoraSpreadingFactor::default_uplink(),
            bandwidth: LoraBandwidth::default_uplink(),
            coding_rate: LoraCodingRate::default_uplink(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct LoRaSettings {
    /// which of the 14 500kHz LoRa channels from 863-870MHz to use
    pub downlink_channels: [bool; 14],
    pub uplink_channel: usize,
    /// binding phrase, used to generate LoRa FHSS sequence
    pub binding_phrase: heapless::String<64>,
    /// key for uplink authentication
    pub authentication_key: u128,
}

impl Default for LoRaSettings {
    fn default() -> Self {
        let mut downlink_channels = [true; 14];
        downlink_channels[0] = false;
        downlink_channels[13] = false;

        Self {
            downlink_channels,
            uplink_channel: 0,
            binding_phrase: heapless::String::new(),
            authentication_key: 0x00000000000000000000000000000000,
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
    /// Duration for each pulse.
    pub pulse_high_duration: u32,
    /// Time delay between recovery output pulses (ms)
    pub pause_duration: u32,
    /// number of recovery output pulses
    pub num_pulses: u32,
    // TODO: unused for actual sound duration
    /// Duration of warning tone to play before enabling outputs (ms)
    pub forewarning_duration: u32,
    /// Warning sound frequency
    pub output_warning_frequency: f32,
}

// FIXME: fix/remove this
impl RecoveryOutputSettings {
    pub fn total_duration(&self) -> u32 {
        self.forewarning_duration
            + self.num_pulses * self.pulse_high_duration
            + (self.num_pulses - 1) * self.pause_duration
    }

    pub fn currently_high(&self, time_in_mode: u32) -> bool {
        let phase_duration = self.pulse_high_duration + self.pause_duration;
        time_in_mode > self.forewarning_duration
            && time_in_mode < self.total_duration()
            && (time_in_mode - self.forewarning_duration) % phase_duration < self.pulse_high_duration
    }
}
/*
impl Default for RecoveryOutputSettings {
    fn default() -> Self {
        Self {
            pulse_high_duration: 500,
            pause_duration: 0,
            num_pulses: 1,
            forewarning_duration: 500,
            output_warning_frequency: 500.0,
        }
    }
}

*/
impl RecoveryOutputSettings {
    fn default_main() -> Self {
        Self {
            pulse_high_duration: 200,
            pause_duration: 500,
            num_pulses: 2,
            forewarning_duration: 500,
            output_warning_frequency: 500.0,
        }
    }
    fn default_parabreaks() -> Self {
        Self {
            pulse_high_duration: 16_000,
            pause_duration: 0,
            num_pulses: 1,
            forewarning_duration: 500,
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
        Self {
            intercept: 0.0,
            slope: 1.0,
        }
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
    pub lora_uplink_settings: LoraLinkSettings,
    pub lora_downlink_settings: LoraLinkSettings,
    /// Telemetry data rate
    pub default_data_rate: TelemetryDataRate,
    /// Time after drogue deployment in which main deployment will not be triggered
    pub min_time_to_main: u32,
    /// Mounting orientation of the flight computer
    pub orientation: Orientation,
    //
    pub acs_tank_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_regulator_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_accel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_decel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub recovery_pressure_sensor_settings: PressureSensorCalibrationSettings,
    pub acs_nominal_acceleration: f32,
    pub acs_nominal_tank_pressure: f32,
    pub acs_acceleration_pressure_slope: f32,
    pub drag_reduction_factor: f32,
    pub drag_reduction_exp: f32,
    pub apogee_error_offset: f32,
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
            std_dev_barometer: 10.0,
            std_dev_barometer_transsonic: 5000.0,
            std_dev_process: 0.5,
            min_takeoff_acc: 30.0,
            min_takeoff_acc_time: 50,
            min_time_to_apogee: 8000,
            max_time_to_apogee: 32000,
            apogee_min_falling_time: 100,
            main_output_mode: MainOutputMode::default(),
            main_output_deployment_altitude: 450.0,
            drogue_output_settings: RecoveryOutputSettings::default_parabreaks(),
            main_output_settings: RecoveryOutputSettings::default_main(),
            lora: LoRaSettings::default(),
            lora_downlink_settings: LoraLinkSettings::default_downlink(),
            lora_uplink_settings: LoraLinkSettings::default_uplink(),
            default_data_rate: TelemetryDataRate::default(),
            min_time_to_main: 1000,
            orientation: Orientation::ZDown,
            acs_tank_pressure_sensor_settings: PressureSensorCalibrationSettings {
                intercept: 2293.1,
                slope: -6.24,
            },
            acs_regulator_pressure_sensor_settings: PressureSensorCalibrationSettings {
                intercept: 1495.7,
                slope: 24.43,
            },
            acs_accel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings {
                intercept: 404.5,
                slope: 25.09,
            },
            acs_decel_valve_pressure_sensor_settings: PressureSensorCalibrationSettings {
                intercept: 296.2,
                slope: 24.56,
            },
            recovery_pressure_sensor_settings: PressureSensorCalibrationSettings {
                intercept: 2425.8,
                slope: 9.11,
            },
            acs_nominal_acceleration: 15.5 / 20.0,
            acs_nominal_tank_pressure: 183.0,
            acs_acceleration_pressure_slope: 0.089 / 20.0,
            drag_reduction_factor: 0.8,
            drag_reduction_exp: 0.5,
            apogee_error_offset: 10.0,
        }
    }
}
