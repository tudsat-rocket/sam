use egui::{Align, CollapsingHeader, Layout, Vec2};

use shared_types::telemetry::*;
use telemetry::{BarometerId, BatteryId, Dim, Metric, TemperatureSensorId};

use crate::Backend;
use crate::widgets::top_bar::*;

pub struct HeaderPanel {}

impl HeaderPanel {
    fn text_telemetry(ui: &mut egui::Ui, backend: &mut Backend) {
        let spacing = 3.0; // TODO: this is ugly

        let time = backend.end().map(|time| format!("{:10.3}", time));
        let mode = backend.current_enum::<FlightMode>(Metric::FlightMode).map(|s| format!("{:?}", s));

        let alt_ground = backend.current_value(Metric::GroundAltitudeASL).unwrap_or(0.0);
        let alt_agl = backend.current_value(Metric::PositionWorldSpace(Dim::Z)).map(|a| a - alt_ground);
        let apogee_agl = backend.current_value(Metric::ApogeeAltitudeASL).map(|a| a - alt_ground);

        let gps_fix = backend.current_enum::<GPSFixType>(Metric::GpsFix);
        let latitude = backend.current_value(Metric::Latitude);
        let longitude = backend.current_value(Metric::Latitude);
        let coords = latitude.and_then(|lat| longitude.map(|lng| format!("{:.5},{:.5}", lat, lng)));

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 3.5);
            ui.add_space(spacing);
            ui.telemetry_value("🕐", "Time [s]", time);
            ui.telemetry_value("🏷", "Mode", mode);
            ui.nominal_value(
                "🔥",
                "Baro Temp. [°C]",
                backend.current_value(Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611))),
                1,
                0.0,
                60.0,
            );
            ui.nominal_value("📡", "RSSI [dBm]", backend.current_value(Metric::DownlinkRssi), 1, -50.0, 0.0);
            ui.nominal_value("📶", "Link Quality [%]", backend.link_quality().map(|f| f.into()), 1, 90.0, 101.0);
            ui.add_space(ui.spacing().item_spacing.y);
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 2.0);
            ui.add_space(spacing);
            ui.nominal_value("📈", "Altitude (AGL) [m]", alt_agl, 1, -1.0, 10000.0);
            ui.nominal_value("📈", "Apogee (AGL) [m]", apogee_agl, 1, -1.0, 10000.0);
            ui.nominal_value(
                "☁",
                "Baro. Alt. (ASL) [m]",
                backend.current_value(Metric::RawBarometricAltitude(BarometerId::MS5611)),
                1,
                -100.0,
                10000.0,
            );
            ui.nominal_value(
                "⏱",
                "Vertical Speed [m/s]",
                backend.current_value(Metric::VelocityWorldSpace(Dim::Z)),
                2,
                -1.0,
                1.0,
            );
            ui.nominal_value(
                "⬆",
                "Vertical Accel. [m/s²]",
                backend.current_value(Metric::AccelerationWorldSpace(Dim::Z)),
                1,
                -1.0,
                1.0,
            );
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width());
            ui.add_space(spacing);
            ui.telemetry_value("🌍", "GPS Status", gps_fix.map(|f| format!("{:?}", f)));
            ui.nominal_value("📶", "# Sats", backend.current_value(Metric::GpsSatellites), 0, 5.0, 99.0);
            ui.nominal_value("🎯", "HDOP", backend.current_value(Metric::GpsHdop), 2, 0.0, 5.0);
            ui.nominal_value(
                "📡",
                "GPS Alt. (ASL) [m]",
                backend.current_value(Metric::GpsAltitude),
                1,
                -100.0,
                10000.0,
            );
            ui.telemetry_value("🌐", "Coords", coords);
        });
    }

    fn header_ui(ui: &mut egui::Ui, backend: &mut Backend, vertical: bool) {
        if vertical {
            ui.horizontal(|ui| {
                Self::text_telemetry(ui, backend);
            });
        } else {
            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width() * 0.50);
                Self::text_telemetry(ui, backend);
            });
        }

        ui.separator();

        let current_transmit_power = backend.current_enum::<TransmitPower>(Metric::TransmitPower).unwrap_or_default();
        let current_acs_mode = backend.current_enum::<AcsMode>(Metric::AcsMode).unwrap_or_default();
        let current_valve_state =
            backend.current_enum::<ThrusterValveState>(Metric::ThrusterValveState).unwrap_or_default();
        let acs_present = false; // TODO
        let recovery_present = false; // TODO
        let payload_present = false; // TODO
        let fins_present = [false, false, false]; // TODO
        let current_camera_state = [false, false, false];

        if vertical {
            ui.columns(4, |uis| {
                uis[0].add_space(3.0);
                uis[0].weak("ACS Mode");
                uis[1].acs_mode_controls(current_acs_mode, backend);
                uis[2].add_space(3.0);
                uis[2].weak("ACS Valves");
                uis[3].acs_valve_controls(current_valve_state, current_acs_mode, backend);
            });

            ui.separator();

            ui.columns(4, |uis| {
                uis[0].add_space(3.0);
                uis[0].weak("Transmit Power [dBm]");
                uis[1].transmit_power_controls(current_transmit_power, backend);
                uis[2].add_space(3.0);
                uis[2].weak("Cams");
                uis[3].camera_controls(current_camera_state, backend);
            });

            ui.separator();

            ui.columns(4, |uis| {
                uis[0].add_space(3.0);
                uis[0].weak("IO Modules");
                uis[1].io_module_indicators(acs_present, recovery_present, payload_present);
                uis[2].add_space(3.0);
                uis[2].weak("Fins");
                uis[3].fin_indicators(fins_present);
            });
        } else {
            ui.vertical(|ui| {
                ui.add_space(3.0);
                ui.weak("Transmit Power [dBm]");
                ui.transmit_power_controls(current_transmit_power, backend);
                ui.add_space(3.0);
                ui.weak("IO Modules");
                ui.io_module_indicators(acs_present, recovery_present, payload_present);
                ui.horizontal(|ui| {
                    ui.weak("Fins");
                    ui.add_space(3.0);
                    ui.fin_indicators(fins_present);
                });
            });

            ui.separator();

            ui.vertical(|ui| {
                ui.add_space(3.0);
                ui.weak("ACS Mode");
                ui.acs_mode_controls(current_acs_mode, backend);
                ui.add_space(3.0);
                ui.weak("ACS Valves");
                ui.acs_valve_controls(current_valve_state, current_acs_mode, backend);
                ui.horizontal(|ui| {
                    ui.weak("Cams");
                    ui.add_space(3.0);
                    ui.camera_controls(current_camera_state, backend);
                });
            });
        }

        ui.vertical(|ui| {
            let size = Vec2::new(ui.available_width(), 30.0);
            ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                ui.command_button("⟲  Reboot", Command::Reboot, backend);
                ui.command_button("🗑 Erase Flash", Command::EraseFlash, backend);
                ui.flash_bar(ui.available_width() * 0.6, backend.current_value(Metric::FlashPointer));
                ui.battery_bar(
                    ui.available_width(),
                    backend.current_value(Metric::BatteryVoltage(BatteryId::Avionics)),
                );
            });

            ui.separator();

            ui.allocate_ui(ui.available_size(), |ui| {
                ui.flight_mode_buttons(backend.flight_mode(), backend);
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
