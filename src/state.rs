use std::f32::consts::PI;

use euroc_fc_firmware::telemetry::*;
use nalgebra::{vector, Vector3};
use nalgebra::{Quaternion, UnitQuaternion};

// TODO: refactor this, maybe replace with a trait implemented by downlink messages?

pub trait VehicleState {
    fn mode(&self) -> Option<FlightMode>;
    fn gyroscope(&self) -> Option<Vector3<f32>>;
    fn accelerometer1(&self) -> Option<Vector3<f32>>;
    fn accelerometer2(&self) -> Option<Vector3<f32>>;
    fn cpu_utilization(&self) -> Option<u8>;
    fn heap_utilization(&self) -> Option<u8>;
    fn magnetometer(&self) -> Option<Vector3<f32>>;
    fn pressure(&self) -> Option<f32>;
    fn altitude_baro(&self) -> Option<f32>;
    fn altitude_gps(&self) -> Option<f32>;
    fn gps_fix(&self) -> Option<GPSFixType>;
    fn latitude(&self) -> Option<f32>;
    fn longitude(&self) -> Option<f32>;
    fn hdop(&self) -> Option<u16>;
    fn num_satellites(&self) -> Option<u8>;
    fn orientation(&self) -> Option<UnitQuaternion<f32>>;
    fn altitude(&self) -> Option<f32>;
    fn altitude_max(&self) -> Option<f32>;
    fn altitude_ground(&self) -> Option<f32>;
    fn vertical_speed(&self) -> Option<f32>;
    fn vertical_accel(&self) -> Option<f32>;
    fn vertical_accel_filtered(&self) -> Option<f32>;
    fn temperature_core(&self) -> Option<f32>;
    fn temperature_baro(&self) -> Option<f32>;
    fn battery_voltage(&self) -> Option<f32>;
    fn cpu_voltage(&self) -> Option<f32>;
    fn arm_voltage(&self) -> Option<f32>;
    fn current(&self) -> Option<f32>;
    fn flash_pointer(&self) -> Option<u32>;
    fn vehicle_lora_rssi(&self) -> Option<u8>;
    fn gcs_lora_rssi(&self) -> Option<u8>;
    fn gcs_lora_rssi_signal(&self) -> Option<u8>;
    fn gcs_lora_snr(&self) -> Option<u8>;

    fn euler_angles(&self) -> Option<Vector3<f32>> {
        self.orientation()
            .map(|q| q.euler_angles())
            .map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI)
    }
}

impl VehicleState for DownlinkMessage {
    fn mode(&self) -> Option<FlightMode> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.mode),
            Self::TelemetryMainCompressed(tm) => Some(tm.mode),
            _ => None
        }
    }

    fn gyroscope(&self) -> Option<Vector3<f32>> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.gyro),
            Self::TelemetryRawSensorsCompressed(tm) => Some(<_ as Into<Vector3<f32>>>::into(tm.gyro) / 10.0),
            _ => None
        }
    }

    fn accelerometer1(&self) -> Option<Vector3<f32>> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.accelerometer1),
            Self::TelemetryRawSensorsCompressed(tm) => Some(<_ as Into<Vector3<f32>>>::into(tm.accelerometer1) / 100.0),
            _ => None
        }
    }

    fn accelerometer2(&self) -> Option<Vector3<f32>> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.accelerometer2),
            Self::TelemetryRawSensorsCompressed(tm) => Some(<_ as Into<Vector3<f32>>>::into(tm.accelerometer2) / 10.0),
            _ => None
        }
    }

    fn magnetometer(&self) -> Option<Vector3<f32>> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.magnetometer),
            Self::TelemetryRawSensorsCompressed(tm) => Some(<_ as Into<Vector3<f32>>>::into(tm.magnetometer) / 10.0),
            _ => None
        }
    }

    fn pressure(&self) -> Option<f32> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.pressure_baro),
            Self::TelemetryRawSensorsCompressed(tm) => Some((tm.pressure_baro as f32) / 10.0),
            _ => None
        }
    }

    fn altitude_baro(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.altitude_baro),
            Self::TelemetryMainCompressed(tm) => Some((tm.altitude_baro as f32) / 10.0),
            _ => None
        }
    }

    fn altitude_gps(&self) -> Option<f32> {
        match self {
            Self::TelemetryGPS(tm) => (tm.altitude_asl != u16::MAX).then(|| (tm.altitude_asl as f32) / 10.0),
            _ => None
        }
    }

    fn gps_fix(&self) -> Option<GPSFixType> {
        match self {
            Self::TelemetryGPS(tm) => Some((tm.fix_and_sats >> 5).into()),
            _ => None
        }
    }

    fn latitude(&self) -> Option<f32> {
        match self {
            Self::TelemetryGPS(tm) => {
                let lat = ((tm.latitude[0] as u32) << 16) + ((tm.latitude[1] as u32) << 8) + (tm.latitude[2] as u32);
                (lat > 0)
                    .then(|| lat)
                    .map(|lat| (lat as f32) * 180.0 / 16777215.0 - 90.0)
            },
            _ => None
        }
    }

    fn longitude(&self) -> Option<f32> {
        match self {
            Self::TelemetryGPS(tm) => {
                let lng = ((tm.longitude[0] as u32) << 16) + ((tm.longitude[1] as u32) << 8) + (tm.longitude[2] as u32);
                (lng > 0)
                    .then(|| lng)
                    .map(|lng| (lng as f32) * 360.0 / 16777215.0 - 180.0)
            },
            _ => None
        }
    }

    fn hdop(&self) -> Option<u16> {
        match self {
            Self::TelemetryGPS(tm) => Some(tm.hdop),
            _ => None
        }
    }

    fn num_satellites(&self) -> Option<u8> {
        match self {
            Self::TelemetryGPS(tm) => Some(tm.fix_and_sats & 0x1f),
            _ => None
        }
    }

    fn orientation(&self) -> Option<UnitQuaternion<f32>> {
        match self {
            Self::TelemetryMain(tm) => tm.orientation,
            Self::TelemetryMainCompressed(tm) => {
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
            }
            _ => None
        }
    }

    fn altitude(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.altitude as f32),
            Self::TelemetryMainCompressed(tm) => Some((tm.altitude as f32) / 10.0),
            _ => None
        }
    }

    fn altitude_max(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.altitude_max as f32),
            Self::TelemetryMainCompressed(tm) => Some((tm.altitude_max as f32) / 10.0),
            _ => None
        }
    }

    fn altitude_ground(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.altitude_ground as f32) / 10.0),
            _ => None
        }
    }

    fn vertical_speed(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.vertical_speed),
            Self::TelemetryMainCompressed(tm) => Some(Into::<f32>::into(tm.vertical_speed) / 10.0),
            _ => None
        }
    }

    fn vertical_accel(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.vertical_accel),
            Self::TelemetryMainCompressed(tm) => Some(Into::<f32>::into(tm.vertical_accel) / 10.0),
            _ => None
        }
    }

    fn vertical_accel_filtered(&self) -> Option<f32> {
        match self {
            Self::TelemetryMain(tm) => Some(tm.vertical_accel_filtered),
            Self::TelemetryMainCompressed(tm) => Some(Into::<f32>::into(tm.vertical_accel_filtered) / 10.0),
            _ => None
        }
    }

    fn temperature_core(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.temperature_core as f32) / 2.0),
            _ => None
        }
    }

    fn temperature_baro(&self) -> Option<f32> {
        match self {
            Self::TelemetryRawSensors(tm) => Some(tm.temperature_baro),
            Self::TelemetryRawSensorsCompressed(tm) => Some((tm.temperature_baro as f32) / 2.0),
            _ => None
        }
    }

    fn battery_voltage(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.battery_voltage as f32) / 1000.0),
            _ => None
        }
    }

    fn cpu_voltage(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.cpu_voltage as f32) / 1000.0),
            _ => None
        }
    }

    fn arm_voltage(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.arm_voltage as f32) / 1000.0),
            _ => None
        }
    }

    fn current(&self) -> Option<f32> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some((tm.current as f32) / 1000.0),
            _ => None
        }
    }

    fn cpu_utilization(&self) -> Option<u8> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some(tm.cpu_utilization),
            _ => None
        }
    }

    fn heap_utilization(&self) -> Option<u8> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some(tm.heap_utilization),
            _ => None
        }
    }

    fn flash_pointer(&self) -> Option<u32> {
        match self {
            Self::TelemetryGPS(tm) => Some((tm.flash_pointer as u32) * 1024),
            _ => None
        }
    }

    fn vehicle_lora_rssi(&self) -> Option<u8> {
        match self {
            Self::TelemetryDiagnostics(tm) => Some(tm.lora_rssi),
            _ => None
        }
    }

    fn gcs_lora_rssi(&self) -> Option<u8> {
        match self {
            Self::TelemetryGCS(tm) => Some(tm.lora_rssi),
            _ => None
        }
    }

    fn gcs_lora_rssi_signal(&self) -> Option<u8> {
        match self {
            Self::TelemetryGCS(tm) => Some(tm.lora_rssi_signal),
            _ => None
        }
    }

    fn gcs_lora_snr(&self) -> Option<u8> {
        match self {
            Self::TelemetryGCS(tm) => Some(tm.lora_snr),
            _ => None
        }
    }

}
