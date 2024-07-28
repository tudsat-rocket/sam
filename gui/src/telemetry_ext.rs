//! Contains some extension traits of the telemetry data types.

use eframe::egui::Color32;
use nalgebra::{UnitQuaternion, Vector3};

use shared_types::telemetry::*;

pub trait ColorExt {
    fn color(&self) -> Color32;
}

impl ColorExt for FlightMode {
    fn color(&self) -> Color32 {
        match self {
            FlightMode::Idle => Color32::from_rgb(0xb8, 0xbb, 0x26),
            FlightMode::HardwareArmed => Color32::from_rgb(0xfe, 0x80, 0x19),
            FlightMode::Armed => Color32::from_rgb(0xcc, 0x24, 0x1d),
            FlightMode::ArmedLaunchImminent => Color32::from_rgb(0xdc, 0x34, 0x2f),
            FlightMode::Burn => Color32::from_rgb(0xb1, 0x62, 0x86),
            FlightMode::Coast => Color32::from_rgb(0x45, 0x85, 0x88),
            FlightMode::RecoveryDrogue => Color32::from_rgb(0xd5, 0xc4, 0xa1),
            FlightMode::RecoveryMain => Color32::from_rgb(0x68, 0x96, 0x6a),
            FlightMode::Landed => Color32::from_rgb(0x61, 0x48, 0x1c),
        }
    }
}

impl ColorExt for LogLevel {
    fn color(&self) -> Color32 {
        match self {
            LogLevel::Debug => Color32::from_rgb(0x45, 0x85, 0x88),
            LogLevel::Info => Color32::from_rgb(0x98, 0x97, 0x1a),
            LogLevel::Warning => Color32::from_rgb(0xd7, 0x99, 0x21),
            LogLevel::Error => Color32::from_rgb(0xcc, 0x24, 0x1d),
            LogLevel::Critical => Color32::from_rgb(0xb1, 0x62, 0x86),
        }
    }
}

pub trait QuaternionExt {
    fn elevation(&self) -> f32;
    fn azimuth(&self) -> f32;
}

impl QuaternionExt for UnitQuaternion<f32> {
    fn elevation(&self) -> f32 {
        let up = Vector3::new(0.0, 0.0, 1.0);
        let attitude = self * up;
        90.0 - up.dot(&attitude).acos().to_degrees()
    }

    fn azimuth(&self) -> f32 {
        let attitude = self * Vector3::new(0.0, 0.0, 1.0);
        (90.0 - attitude.y.atan2(attitude.x).to_degrees()).rem_euclid(360.0)
    }
}
