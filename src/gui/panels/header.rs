use egui::{CollapsingHeader, Vec2, Layout, Align};
use mithril::telemetry::*;

use crate::gui::top_bar::*;
use crate::data_source::DataSource;

pub struct HeaderPanel {}

impl HeaderPanel {
    /// Returns the "current" value for the given callback. This is the last
    /// known of the value at the current time.
    /// TODO: incorporate cursor position?
    fn current<T>(data_source: &mut dyn DataSource, callback: impl Fn(&VehicleState) -> Option<T>) -> Option<T> {
        data_source.vehicle_states().rev().find_map(|(_t, msg)| callback(msg))
    }

    fn text_telemetry(ui: &mut egui::Ui, data_source: &mut dyn DataSource) {
        let spacing = 3.0; // TODO: this is ugly

        let time = data_source
            .vehicle_states()
            .last()
            .map(|(_t, msg)| format!("{:10.3}", (msg.time as f32) / 1000.0));
        let rssi = Self::current(data_source, |vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0));
        let mode = Self::current(data_source, |vs| vs.mode).map(|s| format!("{:?}", s));

        let alt_ground = Self::current(data_source, |vs| vs.altitude_ground_asl).unwrap_or(0.0);
        let alt_agl = Self::current(data_source, |vs| vs.altitude_asl.map(|a| a - alt_ground));
        let apogee_agl = Self::current(data_source, |vs| vs.apogee_asl.map(|a| a - alt_ground));
        let vertical_accel = Self::current(data_source, |vs| vs.vertical_accel_filtered);

        // TODO: also show estimated position?
        let last_gps: Option<GPSDatum> = data_source
            .vehicle_states()
            .rev()
            .find_map(|(_t, vs)| vs.gps.clone());
        let gps_status = last_gps.as_ref().map(|gps| format!("{:?}", gps.fix));
        let hdop = last_gps.as_ref().map(|gps| gps.hdop as f32 / 100.0);
        let latitude = last_gps.as_ref().and_then(|gps| gps.latitude);
        let longitude = last_gps.as_ref().and_then(|gps| gps.longitude);
        let coords = latitude.and_then(|lat| longitude.map(|lng| format!("{:.5},{:.5}", lat, lng)));

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 3.5);
            ui.add_space(spacing);
            ui.telemetry_value("🕐", "Time [s]", time);
            ui.telemetry_value("🏷", "Mode", mode);
            ui.nominal_value("🔥", "Baro Temp. [°C]", Self::current(data_source, |vs| vs.temperature_baro), 1, 0.0, 60.0);
            ui.nominal_value("📡", "RSSI [dBm]", rssi, 1, -50.0, 0.0);
            ui.nominal_value("📶", "Link Quality [%]", data_source.link_quality(), 1, 90.0, 101.0);
            ui.add_space(ui.spacing().item_spacing.y);
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 2.0);
            ui.add_space(spacing);
            ui.nominal_value("📈", "Altitude (AGL) [m]", alt_agl, 1, -1.0, 10000.0);
            ui.nominal_value("📈", "Apogee (AGL) [m]", apogee_agl, 1, -1.0, 10000.0);
            ui.nominal_value("☁", "Baro. Alt. (ASL) [m]", Self::current(data_source, |vs| vs.altitude_baro), 1, -100.0, 10000.0);
            ui.nominal_value("⏱", "Vertical Speed [m/s]", Self::current(data_source, |vs| vs.vertical_speed), 2, -1.0, 1.0);
            ui.nominal_value("⬆", "Vertical Accel. [m/s²]", vertical_accel, 1, -1.0, 1.0);
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width());
            ui.add_space(spacing);
            ui.telemetry_value("🌍", "GPS Status", gps_status);
            ui.nominal_value("📶", "# Sats", last_gps.as_ref().map(|gps| gps.num_satellites as f32), 0, 5.0, 99.0);
            ui.nominal_value("🎯", "HDOP", hdop, 2, 0.0, 5.0);
            ui.nominal_value("📡", "GPS Alt. (ASL) [m]", last_gps.as_ref().and_then(|gps| gps.altitude), 1, -100.0, 10000.0);
            ui.telemetry_value("🌐", "Coords", coords);
        });
    }

    fn header_ui(ui: &mut egui::Ui, data_source: &mut dyn DataSource, vertical: bool) {
        if vertical {
            ui.horizontal(|ui| {
                Self::text_telemetry(ui, data_source);
            });
        } else {
            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width() * 0.50);
                Self::text_telemetry(ui, data_source);
            });
        }

        ui.separator();

        let current_data_rate = Self::current(data_source, |vs| vs.data_rate).unwrap_or_default();
        let current_transmit_power = Self::current(data_source, |vs| vs.transmit_power).unwrap_or_default();

        if vertical {
            ui.columns(4, |uis| {
                uis[0].add_space(3.0);
                uis[0].label("Data Rate [Hz]");
                uis[1].data_rate_controls(current_data_rate, data_source);
                uis[2].add_space(3.0);
                uis[2].label("Transmit Power [dBm]");
                uis[3].transmit_power_controls(current_transmit_power, data_source);
            });
        } else {
            ui.vertical(|ui| {
                ui.add_space(3.0);
                ui.label("Data Rate [Hz]");
                ui.data_rate_controls(current_data_rate, data_source);
                ui.add_space(3.0);
                ui.label("Transmit Power [dBm]");
                ui.transmit_power_controls(current_transmit_power, data_source);
            });
        }

        ui.separator();

        ui.vertical(|ui| {
            let size = Vec2::new(ui.available_width(), 30.0);
            ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                ui.command_button("⟲  Reboot", Command::Reboot, data_source);
                ui.command_button("🗑 Erase Flash", Command::EraseFlash, data_source);
                ui.flash_bar(ui.available_width() * 0.6, Self::current(data_source, |vs| vs.flash_pointer));
                ui.battery_bar(ui.available_width(), Self::current(data_source, |vs| vs.battery_voltage.map(|v| v as f32 / 1000.0)));
            });

            ui.separator();

            ui.allocate_ui(ui.available_size(), |ui| {
                ui.flight_mode_buttons(Self::current(data_source, |vs| vs.mode), data_source);
            });
        });
    }

    pub fn show(ctx: &egui::Context, data_source: &mut dyn DataSource, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        if ctx.screen_rect().width() > 1000.0 {
            egui::TopBottomPanel::top("topbar").min_height(60.0).max_height(60.0).show(ctx, |ui| {
                ui.set_enabled(enabled);
                ui.horizontal_centered(|ui| {
                    Self::header_ui(ui, data_source, false);
                });
            });
        } else {
            egui::TopBottomPanel::top("topbar").min_height(20.0).max_height(300.0).show(ctx, |ui| {
                ui.set_enabled(enabled);
                CollapsingHeader::new("Status & Controls").default_open(false).show(ui, |ui| {
                    Self::header_ui(ui, data_source, true);
                    ui.add_space(10.0);
                });
            });
        }
    }
}
