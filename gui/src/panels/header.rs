use egui::CollapsingHeader;

use shared_types::telemetry::*;
use telemetry::{BarometerId, BatteryId, Dim, Metric, TemperatureSensorId};

use crate::Backend;
use crate::widgets::top_bar::*;

pub struct HeaderPanel {}

impl HeaderPanel {
    fn text_telemetry(ui: &mut egui::Ui, backend: &mut Backend) {
        let time = backend.fc_time().map(|time| format!("{:10.3}", time));

        let alt_ground = backend.current_value(Metric::GroundAltitudeASL).unwrap_or(0.0);
        let alt_agl = backend.current_value(Metric::PositionWorldSpace(Dim::Z)).map(|a| a - alt_ground);
        let apogee_agl = backend.current_value(Metric::ApogeeAltitudeASL).map(|a| a - alt_ground);

        let gps_fix = backend.current_enum::<GPSFixType>(Metric::GpsFix);
        let latitude = backend.current_value(Metric::Latitude);
        let longitude = backend.current_value(Metric::Latitude);
        let coords = latitude.and_then(|lat| longitude.map(|lng| format!("{:.5},{:.5}", lat, lng)));

        ui.columns_const(|[c1, c2, c3, c4, c5, c6]| {
            c1.telemetry_value("🕐", "Time [s]", time);
            c1.nominal_value(
                "🔥",
                "Baro Temp. [°C]",
                backend.current_value(Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611))),
                1,
                0.0,
                60.0,
            );

            c2.nominal_value("📡", "RSSI [dBm]", backend.current_value(Metric::DownlinkRssi), 1, -50.0, 0.0);
            c2.nominal_value("📶", "Link Quality [%]", backend.link_quality().map(|f| f.into()), 1, 90.0, 101.0);

            c3.nominal_value("📈", "Altitude (AGL) [m]", alt_agl, 1, -1.0, 10000.0);
            c3.nominal_value("📈", "Apogee (AGL) [m]", apogee_agl, 1, -1.0, 10000.0);

            c4.nominal_value(
                "⏱",
                "Vertical Speed [m/s]",
                backend.current_value(Metric::VelocityWorldSpace(Dim::Z)),
                2,
                -1.0,
                1.0,
            );
            c4.nominal_value(
                "⬆",
                "Vertical Accel. [m/s²]",
                backend.current_value(Metric::AccelerationWorldSpace(Dim::Z)),
                1,
                -1.0,
                1.0,
            );

            c5.telemetry_value("🌍", "GPS Status", gps_fix.map(|f| format!("{:?}", f)));
            c5.nominal_value("📶", "# Sats", backend.current_value(Metric::GpsSatellites), 0, 5.0, 99.0);

            c6.nominal_value("🎯", "HDOP", backend.current_value(Metric::GpsHdop), 2, 0.0, 5.0);
            c6.telemetry_value("🌐", "Coords", coords);
        });
    }

    fn header_ui(ui: &mut egui::Ui, backend: &mut Backend, vertical: bool) {
        if vertical {
            ui.horizontal(|ui| {
                Self::text_telemetry(ui, backend);
            });
        } else {
            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width() * 0.70);
                Self::text_telemetry(ui, backend);
            });
        }

        ui.separator();

        let current_transmit_power = backend.current_enum::<TransmitPower>(Metric::TransmitPower).unwrap_or_default();

        ui.vertical(|ui| {
            ui.add_space(3.0);
            ui.weak("Transmit Power [dBm]");
            ui.transmit_power_controls(current_transmit_power, backend);
        });

        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.flash_bar(
                    f32::max(0f32, ui.available_width() - 100f32),
                    backend.current_value(Metric::FlashPointer),
                );
                ui.command_button("🗑 Erase Flash", Command::EraseFlash, backend);
            });
            ui.horizontal(|ui| {
                ui.battery_bar(
                    f32::max(0f32, ui.available_width() - 100f32),
                    backend.current_value(Metric::BatteryVoltage(BatteryId::Avionics)),
                );
                ui.command_button("⟲  Reboot", Command::Reboot, backend);
            });
        });
    }

    pub fn show(ctx: &egui::Context, backend: &mut Backend, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        if ctx.screen_rect().width() > 1000.0 {
            egui::TopBottomPanel::top("topbar").min_height(60.0).show(ctx, |ui| {
                ui.add_enabled_ui(enabled, |ui| {
                    ui.horizontal_centered(|ui| {
                        Self::header_ui(ui, backend, false);
                    })
                });
            });
        } else {
            egui::TopBottomPanel::top("topbar").min_height(20.0).show(ctx, |ui| {
                ui.add_enabled_ui(enabled, |ui| {
                    CollapsingHeader::new("Status & Controls").default_open(false).show(ui, |ui| {
                        Self::header_ui(ui, backend, true);
                        ui.add_space(10.0);
                    });
                });
            });
        }
    }
}
