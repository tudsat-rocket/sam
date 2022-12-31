//! Contains smaller UI functions for the top bar, including text indicators
//! and flight mode buttons.

use eframe::egui;
use egui::widgets::{Button, ProgressBar};
use egui::{Color32, RichText, Stroke};

use euroc_fc_firmware::telemetry::*;

use crate::telemetry_ext::*;
use crate::data_source::*;

// TODO: move to telemetry_ext?
fn flight_mode_style(fm: FlightMode) -> (&'static str, Color32, Color32) {
    match fm {
        FlightMode::Idle => ("IDLE",             Color32::WHITE, fm.color()),
        FlightMode::HardwareArmed => ("HWARMED", Color32::BLACK, fm.color()),
        FlightMode::Armed => ("ARMED",           Color32::WHITE, fm.color()),
        FlightMode::Flight => ("FLIGHT",         Color32::BLACK, fm.color()),
        FlightMode::RecoveryDrogue => ("DROGUE", Color32::BLACK, fm.color()),
        FlightMode::RecoveryMain => ("MAIN",     Color32::BLACK, fm.color()),
        FlightMode::Landed => ("LANDED",         Color32::WHITE, fm.color()),
    }
}

pub trait TopBarUiExt {
    fn telemetry_value(&mut self, label: &str, value: Option<String>);
    fn battery_bar(&mut self, w: f32, f: f32, text: String);
    fn flash_bar(&mut self, w: f32, f: f32, text: String);
    fn command_button(
        &mut self,
        label: &'static str,
        msg: UplinkMessage,
        data_source: &mut Box<dyn DataSource>,
    );
    fn flight_mode_button(
        &mut self,
        w: f32,
        fm: FlightMode,
        current: Option<FlightMode>,
        data_source: &mut Box<dyn DataSource>,
    );
}

impl TopBarUiExt for egui::Ui {
    fn telemetry_value(&mut self, label: &str, value: Option<String>) {
        self.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(value.unwrap_or_default()).strong().monospace());
            });
        });
    }

    fn battery_bar(&mut self, w: f32, f: f32, text: String) {
        self.horizontal(|ui| {
            ui.style_mut().visuals.extreme_bg_color = Color32::from_rgb(0x10, 0x10, 0x10);
            ui.style_mut().visuals.override_text_color = Some(Color32::LIGHT_GRAY);
            ui.style_mut().visuals.selection.bg_fill = if f > 0.6 {
                Color32::DARK_GREEN
            } else if f > 0.25 {
                Color32::from_rgb(0xe6, 0x6f, 0x00)
            } else {
                Color32::from_rgb(0xff, 0x3b, 0x00)
            };

            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn flash_bar(&mut self, w: f32, f: f32, text: String) {
        self.horizontal(|ui| {
            ui.style_mut().visuals.extreme_bg_color = Color32::from_rgb(0x10, 0x10, 0x10);
            ui.style_mut().visuals.override_text_color = Some(Color32::LIGHT_GRAY);
            ui.style_mut().visuals.selection.bg_fill = if f > 0.9 {
                Color32::DARK_RED
            } else if f > 0.5 {
                Color32::from_rgb(0xe6, 0x6f, 0x00)
            } else {
                Color32::from_rgb(0x24, 0x63, 0x99)
            };

            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn command_button(
        &mut self,
        label: &'static str,
        msg: UplinkMessage,
        data_source: &mut Box<dyn DataSource>,
    ) {
        if self.button(label).clicked() {
            data_source.send(msg).unwrap();
        }
    }

    fn flight_mode_button(
        &mut self,
        w: f32,
        fm: FlightMode,
        current: Option<FlightMode>,
        data_source: &mut Box<dyn DataSource>,
    ) {
        let (label, fg, bg) = flight_mode_style(fm);

        let button = if current.map(|c| c == fm).unwrap_or(false) {
            let label = RichText::new(label).monospace().color(fg);
            Button::new(label).fill(bg)
        } else {
            let label = RichText::new(label).monospace();
            Button::new(label)
                .fill(Color32::TRANSPARENT)
                .stroke(Stroke::new(2.0, bg))
        };

        if self.add_sized([w, self.available_height() - 10.0], button).clicked() {
            data_source
                .send(UplinkMessage::SetFlightModeAuth(fm, data_source.next_mac()))
                .unwrap();
        }
    }
}
