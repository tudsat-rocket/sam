use euroc_fc_firmware::telemetry::*;
use nalgebra::{Quaternion, UnitQuaternion};
use nalgebra::vector;

#[derive(Clone, Debug, Default)]
pub struct VehicleState {
    pub time: u32,
    pub mode: Option<FlightMode>,
    pub loop_runtime: Option<u16>,
    // raw sensor values
    pub gyroscope: Option<(f32, f32, f32)>,
    pub accelerometer1: Option<(f32, f32, f32)>,
    pub accelerometer2: Option<(f32, f32, f32)>,
    pub magnetometer: Option<(f32, f32, f32)>,
    pub pressure: Option<f32>,
    pub altitude_baro: Option<f32>,
    // GPS
    pub altitude_gps: Option<f32>,
    pub gps_fix: Option<GPSFixType>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub hdop: Option<u16>,
    pub num_satellites: Option<u8>,
    // computed/filtered values
    pub orientation: Option<UnitQuaternion<f32>>,
    pub euler_angles: Option<(f32, f32, f32)>, // calculated on GCS side from orientation
    pub acceleration: Option<(f32, f32, f32)>,
    pub acceleration_world: Option<(f32, f32, f32)>,
    pub altitude: Option<f32>,
    pub altitude_ground: Option<f32>,
    pub vertical_speed: Option<f32>,
    pub vertical_accel: Option<f32>,
    pub vertical_accel_filtered: Option<f32>,
    // temps
    pub temperature_core: Option<f32>,
    pub temperature_baro: Option<f32>,
    // power
    pub battery_voltage: Option<f32>,
    pub cpu_voltage: Option<f32>,
    pub arm_voltage: Option<f32>,
    pub current: Option<f32>,
    //pub consumed: Option<u16>,
    // signal
    pub vehicle_lora_rssi: Option<u8>,
    pub gcs_lora_rssi: Option<u8>,
    pub gcs_lora_rssi_signal: Option<u8>,
    pub gcs_lora_snr: Option<u8>,
}

impl VehicleState {
    pub fn incorporate_telemetry(&mut self, msg: &DownlinkMessage) {
        match msg {
            DownlinkMessage::TelemetryMain(tm) => {
                self.mode = Some(tm.mode);
                self.orientation = tm.orientation;
                self.vertical_speed = Some(tm.vertical_speed);
                self.vertical_accel = Some(tm.vertical_accel);
                self.vertical_accel_filtered = Some(tm.vertical_accel_filtered);
                self.altitude_baro = Some(tm.altitude_baro as f32); // TODO: units
                self.altitude = Some(tm.altitude as f32); // TODO: units
            }
            DownlinkMessage::TelemetryMainCompressed(tm) => {
                let (x,y,z,w) = tm.orientation;
                let quat_raw = Quaternion {
                    coords: vector![((x as f32) - 127.0) / 127.0, ((y as f32) - 127.0) / 127.0, ((z as f32) - 127.0) / 127.0, ((w as f32) - 127.0) / 127.0],
                };
                self.mode = Some(tm.mode);
                self.orientation = Some(UnitQuaternion::from_quaternion(quat_raw));
                self.vertical_speed = Some(<f8 as Into<f32>>::into(tm.vertical_speed) / 100.0);
                self.vertical_accel = Some(<f8 as Into<f32>>::into(tm.vertical_accel) / 100.0);
                self.vertical_accel_filtered = Some(<f8 as Into<f32>>::into(tm.vertical_accel_filtered) / 100.0);
                self.altitude_baro = Some((tm.altitude_baro as f32) / 10.0);
                self.altitude = Some((tm.altitude as f32) / 10.0);
            }
            DownlinkMessage::TelemetryRawSensors(tm) => {
                self.gyroscope = Some(tm.gyro);
                self.accelerometer1 = Some(tm.accelerometer1);
                self.accelerometer2 = Some(tm.accelerometer2);
                self.magnetometer = Some(tm.magnetometer);
                self.temperature_baro = Some(tm.temperature_baro);
                self.pressure = Some(tm.pressure_baro);
            }
            DownlinkMessage::TelemetryRawSensorsCompressed(tm) => {
                let gyro: (f32, f32, f32) = (tm.gyro.0.into(), tm.gyro.1.into(), tm.gyro.2.into());
                let acc1: (f32, f32, f32) = (tm.accelerometer1.0.into(), tm.accelerometer1.1.into(), tm.accelerometer1.2.into());
                let acc2: (f32, f32, f32) = (tm.accelerometer2.0.into(), tm.accelerometer2.1.into(), tm.accelerometer2.2.into());
                let mag: (f32, f32, f32) = (tm.magnetometer.0.into(), tm.magnetometer.1.into(), tm.magnetometer.2.into());
                self.gyroscope = Some((gyro.0 / 10.0, gyro.1 / 10.0, gyro.2 / 10.0));
                self.accelerometer1 = Some((acc1.0 / 1000.0, acc1.1 / 1000.0, acc1.2 / 1000.0));
                self.accelerometer2 = Some((acc2.0 / 100.0, acc2.1 / 100.0, acc2.2 / 100.0));
                self.magnetometer = Some((mag.0 / 10.0, mag.1 / 10.0, mag.2 / 10.0));
                self.temperature_baro = Some((tm.temperature_baro as f32) / 2.0);
                self.pressure = Some((tm.pressure_baro as f32) / 10.0);
            }
            DownlinkMessage::TelemetryDiagnostics(tm) => {
                self.loop_runtime = Some(tm.loop_runtime);
                self.temperature_core = Some(tm.temperature_core as f32 / 100.0);
                self.cpu_voltage = Some(tm.cpu_voltage as f32 / 1000.0);
                self.battery_voltage = Some(tm.battery_voltage as f32 / 1000.0);
                self.current = Some(tm.current as f32 / 1000.0);
                self.arm_voltage = Some(tm.arm_voltage as f32 / 1000.0);
                self.vehicle_lora_rssi = Some(tm.lora_rssi);
                // TODO
            }
            DownlinkMessage::TelemetryGPS(tm) => {
                self.gps_fix = Some(tm.fix);
                self.hdop = Some(tm.hdop);
                self.num_satellites = Some(tm.num_satellites);
                self.latitude = tm.latitude;
                self.longitude = tm.longitude;
                self.altitude_gps = tm.altitude_asl;
            }
            DownlinkMessage::TelemetryGCS(tm) => {
                self.gcs_lora_rssi = Some(tm.lora_rssi);
                self.gcs_lora_rssi_signal = Some(tm.lora_rssi_signal);
                self.gcs_lora_snr = Some(tm.lora_snr);
            }
            _ => {} // TODO
        }

        if self.euler_angles.is_none() && self.orientation.is_some() {
            self.euler_angles = self.orientation.map(|q| q.euler_angles());
        };
    }
}
