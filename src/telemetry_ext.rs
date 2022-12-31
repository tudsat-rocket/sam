//! Contains some extension traits of the telemetry data types.

use eframe::egui::Color32;

use euroc_fc_firmware::telemetry::*;

pub trait ColorExt {
    fn color(&self) -> Color32;
}

impl ColorExt for FlightMode {
    fn color(&self) -> Color32 {
        match self {
            FlightMode::Idle =>           Color32::from_rgb(0x27, 0xa3, 0x00),
            FlightMode::HardwareArmed =>  Color32::from_rgb(0xff, 0x7b, 0x00),
            FlightMode::Armed =>          Color32::from_rgb(0xae, 0x20, 0x12),
            FlightMode::Flight =>         Color32::from_rgb(0x74, 0x00, 0xc6),
            FlightMode::RecoveryDrogue => Color32::from_rgb(0x48, 0xbf, 0xe3),
            FlightMode::RecoveryMain =>   Color32::from_rgb(0x72, 0xef, 0xdd),
            FlightMode::Landed =>         Color32::from_rgb(0x7f, 0x4f, 0x24),
        }
    }
}

impl ColorExt for LogLevel {
    fn color(&self) -> Color32 {
        match self {
            LogLevel::Debug => Color32::LIGHT_BLUE,
            LogLevel::Info => Color32::GREEN,
            LogLevel::Warning => Color32::YELLOW,
            LogLevel::Error => Color32::RED,
            LogLevel::Critical => Color32::DARK_RED,
        }
    }
}
