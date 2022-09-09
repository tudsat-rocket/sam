use euroc_fc_firmware::telemetry::*;
use nalgebra::UnitQuaternion;

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
    // temps
    pub temperature_core: Option<f32>,
    pub temperature_baro: Option<f32>,
    // power
    pub battery_voltage: Option<u16>,
    pub cpu_voltage: Option<u16>,
    pub arm_voltage: Option<u16>,
    pub current: Option<u16>,
    pub consumed: Option<u16>,
}

impl VehicleState {
    pub fn incorporate_telemetry(&mut self, msg: &DownlinkMessage) {
        match msg {
            DownlinkMessage::TelemetryMain(tm) => {
                self.mode = Some(tm.mode);
                self.orientation = tm.orientation;
                self.vertical_speed = Some(tm.vertical_speed);
                self.altitude_baro = Some(tm.altitude_baro as f32); // TODO: units
                self.altitude = Some(tm.altitude as f32); // TODO: units
            }
            DownlinkMessage::TelemetryState(tm) => {
                self.orientation = tm.orientation;
                self.gyroscope = Some(tm.gyroscope);
                self.acceleration = Some(tm.acceleration);
                self.acceleration_world = Some(tm.acceleration_world);
                self.vertical_speed = Some(tm.vertical_speed);
                self.altitude_baro = Some(tm.altitude_baro as f32); // TODO: units
                self.altitude_gps = Some(tm.altitude_gps as f32); // TODO: units
                self.altitude = Some(tm.altitude as f32); // TODO: units
                self.altitude_ground = Some(tm.altitude_ground as f32); // TODO: units
            }
            DownlinkMessage::TelemetryRawSensors(tm) => {
                self.gyroscope = Some(tm.gyro);
                self.accelerometer1 = Some(tm.accelerometer1);
                self.accelerometer2 = Some(tm.accelerometer2);
                self.magnetometer = Some(tm.magnetometer);
                self.temperature_baro = Some(tm.temperature_baro);
                self.pressure = Some(tm.pressure_baro);
                self.altitude_baro = Some(tm.temperature_baro);
            }
            DownlinkMessage::TelemetryDiagnostics(tm) => {
                self.loop_runtime = Some(tm.loop_runtime);
                self.temperature_core = Some(tm.temperature_core as f32 / 100.0);
                self.cpu_voltage = Some(tm.cpu_voltage);
                // TODO
            }
            DownlinkMessage::TelemetryPower(tm) => {
                self.battery_voltage = Some(tm.battery_voltage);
                self.current = Some(tm.current);
                self.arm_voltage = Some(tm.arm_voltage);
                // TODO
            }
            DownlinkMessage::TelemetryGPS(tm) => {
                self.gps_fix = Some(tm.fix);
                self.hdop = Some(tm.hdop);
                self.num_satellites = Some(tm.num_satellites);
                self.latitude = tm.latitude;
                self.longitude = tm.longitude;
                // TODO: altitude
            }
            _ => {} // TODO
        }

        if self.euler_angles.is_none() && self.orientation.is_some() {
            self.euler_angles = self.orientation.map(|q| q.euler_angles());
        };
    }
}
