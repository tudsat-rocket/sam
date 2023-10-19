use std::f32::consts::PI;

use mithril::telemetry::*;
use nalgebra::{vector, Vector3};
use nalgebra::{Quaternion, UnitQuaternion};

#[derive(Clone, Debug, Default)]
pub struct VehicleState {
    pub time: u32,
    pub mode: Option<FlightMode>,
    pub orientation: Option<UnitQuaternion<f32>>,
    pub altitude: Option<f32>,
    pub altitude_baro: Option<f32>,
    pub altitude_max: Option<f32>,
    pub altitude_ground: Option<f32>,
    pub vertical_speed: Option<f32>,
    pub vertical_accel: Option<f32>,
    pub vertical_accel_filtered: Option<f32>,

    pub gyroscope: Option<Vector3<f32>>,
    pub accelerometer1: Option<Vector3<f32>>,
    pub accelerometer2: Option<Vector3<f32>>,
    pub magnetometer: Option<Vector3<f32>>,
    pub pressure_baro: Option<f32>,
    pub temperature_baro: Option<f32>,

    pub gps_fix: Option<GPSFixType>,
    pub num_satellites: Option<u8>,
    pub hdop: Option<u16>,
    pub latitude: Option<f32>,
    pub longitude: Option<f32>,
    pub altitude_gps: Option<f32>,

    pub cpu_utilization: Option<u8>,
    pub heap_utilization: Option<u8>,
    pub temperature_core: Option<f32>,
    pub battery_voltage: Option<f32>,
    pub charge_voltage: Option<f32>,
    pub arm_voltage: Option<f32>,
    pub breakwire_open: Option<bool>,
    pub current: Option<f32>,
    pub flash_pointer: Option<u32>,

    pub vehicle_lora_rssi: Option<u8>,
    pub gcs_lora_rssi: Option<u8>,
    pub gcs_lora_rssi_signal: Option<u8>,
    pub gcs_lora_snr: Option<i8>,
    pub telemetry_data_rate: Option<TelemetryDataRate>,
    pub transmit_power: Option<TransmitPower>,

    pub main_cartridge_pressure: Option<f32>,
    pub main_chamber_pressure: Option<f32>,
    pub drogue_cartridge_pressure: Option<f32>,
    pub drogue_chamber_pressure: Option<f32>,

    pub true_orientation: Option<UnitQuaternion<f32>>,
    pub true_vertical_accel: Option<f32>,
    pub true_vertical_speed: Option<f32>,
}

impl VehicleState {
    pub fn euler_angles(&self) -> Option<Vector3<f32>> {
        self.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI)
    }

    pub fn true_euler_angles(&self) -> Option<Vector3<f32>> {
        self.true_orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI)
    }
}

impl From<DownlinkMessage> for VehicleState {
    fn from(msg: DownlinkMessage) -> VehicleState {
        match msg {
            DownlinkMessage::TelemetryMain(tm) => Self {
                time: tm.time,
                mode: Some(tm.mode),
                orientation: tm.orientation,
                altitude: Some(tm.altitude as f32),
                altitude_baro: Some(tm.altitude_baro),
                altitude_max: Some(tm.altitude_max as f32),
                vertical_speed: Some(tm.vertical_speed),
                vertical_accel: Some(tm.vertical_accel),
                vertical_accel_filtered: Some(tm.vertical_accel_filtered),
                ..Default::default()
            },
            DownlinkMessage::TelemetryMainCompressed(tm) => Self {
                time: tm.time,
                mode: Some(tm.mode),
                orientation: {
                    let (x, y, z, w) = tm.orientation;
                    let quat_raw = Quaternion {
                        coords: vector![
                            ((x as f32) - 127.0) / 127.0,
                            ((y as f32) - 127.0) / 127.0,
                            ((z as f32) - 127.0) / 127.0,
                            ((w as f32) - 127.0) / 127.0
                        ],
                    };
                    Some(UnitQuaternion::from_quaternion(quat_raw))
                },
                altitude: Some((tm.altitude as f32 - 1000.0) / 10.0),
                altitude_baro: Some((tm.altitude_baro as f32 - 1000.0) / 10.0),
                altitude_max: Some((tm.altitude_max as f32 - 1000.0) / 10.0),
                vertical_speed: Some(Into::<f32>::into(tm.vertical_speed) / 10.0),
                vertical_accel: Some(Into::<f32>::into(tm.vertical_accel) / 10.0),
                vertical_accel_filtered: Some(Into::<f32>::into(tm.vertical_accel_filtered) / 10.0),
                ..Default::default()
            },
            DownlinkMessage::TelemetryRawSensors(tm) => Self {
                time: tm.time,
                gyroscope: Some(tm.gyro),
                accelerometer1: Some(tm.accelerometer1),
                accelerometer2: Some(tm.accelerometer2),
                magnetometer: Some(tm.magnetometer),
                pressure_baro: Some(tm.pressure_baro),
                ..Default::default()
            },
            DownlinkMessage::TelemetryRawSensorsCompressed(tm) => Self {
                time: tm.time,
                gyroscope: Some(<_ as Into<Vector3<f32>>>::into(tm.gyro) / 10.0),
                accelerometer1: Some(<_ as Into<Vector3<f32>>>::into(tm.accelerometer1) / 100.0),
                accelerometer2: Some(<_ as Into<Vector3<f32>>>::into(tm.accelerometer2) / 10.0),
                magnetometer: Some(<_ as Into<Vector3<f32>>>::into(tm.magnetometer) / 10.0),
                pressure_baro: Some((tm.pressure_baro as f32) / 10.0),
                ..Default::default()
            },
            DownlinkMessage::TelemetryDiagnostics(tm) => Self {
                time: tm.time,
                altitude_ground: Some((tm.altitude_ground as f32 - 1000.0) / 10.0),
                battery_voltage: Some(((tm.battery_voltage >> 2) as f32) / 1000.0),
                charge_voltage: Some((tm.charge_voltage as f32) / 1000.0),
                current: Some((tm.current as f32) / 1000.0),
                breakwire_open: ((tm.battery_voltage & 0b10) == 0).then(|| (tm.battery_voltage & 0b01) > 0),
                cpu_utilization: Some(tm.cpu_utilization),
                vehicle_lora_rssi: Some(tm.lora_rssi),
                transmit_power: Some((tm.transmit_power_and_data_rate & 0x7f).into()),
                telemetry_data_rate: Some((tm.transmit_power_and_data_rate >> 7).into()),
                temperature_baro: Some((tm.temperature_baro as f32) / 2.0),
                drogue_cartridge_pressure: (tm.recovery_drogue[0] != 0x00)
                    .then(|| (tm.recovery_drogue[0] as f32 / 400.0 / 3.3 - 0.001) * 160.0 / 0.8),
                main_cartridge_pressure: (tm.recovery_main[0] != 0x00)
                    .then(|| (tm.recovery_main[0] as f32 / 400.0 / 3.3 - 0.001) * 160.0 / 0.8),
                drogue_chamber_pressure: (tm.recovery_drogue[1] != 0x00)
                    .then(|| (tm.recovery_drogue[1] as f32 / 20.0 / 3.3 + 2.0) * 6.0 / 14.85),
                main_chamber_pressure: (tm.recovery_main[1] != 0x00)
                    .then(|| (tm.recovery_main[1] as f32 / 20.0 / 3.3 + 2.0) * 6.0 / 14.85),
                ..Default::default()
            },
            DownlinkMessage::TelemetryGPS(tm) => Self {
                time: tm.time,
                gps_fix: Some((tm.fix_and_sats >> 5).into()),
                num_satellites: Some(tm.fix_and_sats & 0x1f),
                hdop: Some(tm.hdop),
                latitude: {
                    let lat =
                        ((tm.latitude[0] as u32) << 16) + ((tm.latitude[1] as u32) << 8) + (tm.latitude[2] as u32);
                    (lat > 0).then(|| lat).map(|lat| (lat as f32) * 180.0 / 16777215.0 - 90.0)
                },
                longitude: {
                    let lng =
                        ((tm.longitude[0] as u32) << 16) + ((tm.longitude[1] as u32) << 8) + (tm.longitude[2] as u32);
                    (lng > 0).then(|| lng).map(|lng| (lng as f32) * 360.0 / 16777215.0 - 180.0)
                },
                altitude_gps: (tm.altitude_asl != u16::MAX).then(|| (tm.altitude_asl as f32 - 1000.0) / 10.0),
                flash_pointer: Some((tm.flash_pointer as u32) * 1024),
                ..Default::default()
            },
            DownlinkMessage::TelemetryGCS(tm) => Self {
                time: tm.time,
                gcs_lora_rssi: Some(tm.lora_rssi),
                gcs_lora_rssi_signal: Some(tm.lora_rssi_signal),
                gcs_lora_snr: Some(tm.lora_snr),
                ..Default::default()
            },
            DownlinkMessage::Log(t, ..) => Self {
                time: t,
                ..Default::default()
            },
            DownlinkMessage::FlashContent(..) | DownlinkMessage::Settings(..) => Default::default(),
        }
    }
}
