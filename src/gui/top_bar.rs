//! Contains smaller UI functions for the top bar, including text indicators
//! and flight mode buttons.

use eframe::egui;
use egui::widgets::{Button, ProgressBar};
use egui::{Color32, RichText, Stroke, Label};

use sting_fc_firmware::telemetry::*;

use crate::telemetry_ext::*;
use crate::data_source::*;

// TODO: move to telemetry_ext?
fn flight_mode_style(fm: FlightMode) -> (&'static str, &'static str, Color32, Color32) {
    let fg = Color32::from_rgb(0x28, 0x28, 0x28);
    match fm {
        FlightMode::Idle           => ("IDLE",    "F5",  fg, fm.color()),
        FlightMode::HardwareArmed  => ("HWARMED", "F6",  fg, fm.color()),
        FlightMode::Armed          => ("ARMED",   "F7",  fg, fm.color()),
        FlightMode::Flight         => ("FLIGHT",  "F8",  fg, fm.color()),
        FlightMode::RecoveryDrogue => ("DROGUE",  "F9",  fg, fm.color()),
        FlightMode::RecoveryMain   => ("MAIN",    "F10", fg, fm.color()),
        FlightMode::Landed         => ("LANDED",  "F11", fg, fm.color()),
    }
}

pub trait TopBarUiExt {
    fn telemetry_value(&mut self, icon: &str, label: &str, value: Option<String>);
    fn battery_bar(&mut self, w: f32, f: f32, text: String);
    fn flash_bar(&mut self, w: f32, f: f32, text: String);
    fn command_button(
        &mut self,
        label: &'static str,
        cmd: Command,
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
    fn telemetry_value(&mut self, icon: &str, label: &str, value: Option<String>) {
        self.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.add_sized([15.0, ui.available_height()], Label::new(icon));
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(5.0);
                ui.label(RichText::new(value.unwrap_or_default()).strong().monospace());
            });
        });
    }

    fn battery_bar(&mut self, w: f32, f: f32, text: String) {
        self.horizontal(|ui| {
            ui.style_mut().visuals.selection.bg_fill = match f {
                _f if f > 0.6  => Color32::from_rgb(0x98, 0x97, 0x1a),
                _f if f > 0.25 => Color32::from_rgb(0xd6, 0x5d, 0x0e),
                _              => Color32::from_rgb(0xcc, 0x24, 0x1d)
            };

            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn flash_bar(&mut self, w: f32, f: f32, text: String) {
        self.horizontal(|ui| {
            ui.style_mut().visuals.selection.bg_fill = match f {
                _f if f > 0.9  => Color32::from_rgb(0xcc, 0x24, 0x1d),
                _f if f > 0.75 => Color32::from_rgb(0xd6, 0x5d, 0x0e),
                _              => Color32::from_rgb(0x45, 0x85, 0x88)
            };

            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn command_button(
        &mut self,
        label: &'static str,
        cmd: Command,
        data_source: &mut Box<dyn DataSource>,
    ) {
        if self.button(label).clicked() {
            data_source.send_command(cmd).unwrap();
        }
    }

    fn flight_mode_button(
        &mut self,
        w: f32,
        fm: FlightMode,
        current: Option<FlightMode>,
        data_source: &mut Box<dyn DataSource>,
    ) {
        let (label, shortcut, fg, bg) = flight_mode_style(fm);
        let main_text = RichText::new(format!("{}\n", label)).monospace();
        let shortcut_text = RichText::new(format!("\nSh+{}", shortcut)).monospace();

        let button = if current.map(|c| c == fm).unwrap_or(false) {
            Button::new(main_text.color(fg))
                .shortcut_text(shortcut_text.color(fg))
                .fill(bg)
        } else {
            Button::new(main_text)
                .shortcut_text(shortcut_text)
                .fill(Color32::TRANSPARENT)
                .stroke(Stroke::new(2.0, bg))
        };

        if self.add_sized([w, self.available_height() - 10.0], button).clicked() {
            data_source
                .send_command(Command::SetFlightMode(fm))
                .unwrap();
        }
    }
}
