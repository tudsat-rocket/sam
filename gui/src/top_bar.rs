//! Contains smaller UI functions for the top bar, including text indicators
//! and flight mode buttons.

use std::time::Duration;

use eframe::egui;
use egui::widgets::{Button, ProgressBar};
use egui::{Color32, Label, RichText, SelectableLabel, Stroke, Rect, Vec2};

use shared_types::{telemetry::*, IoBoardRole};

use crate::data_source::*;
use crate::telemetry_ext::*;

// TODO: move to telemetry_ext?
fn flight_mode_style(fm: FlightMode) -> (&'static str, &'static str, Color32, Color32) {
    let fg = Color32::from_rgb(0x28, 0x28, 0x28);
    match fm {
        FlightMode::Idle => ("IDLE", "F4", fg, fm.color()),
        FlightMode::HardwareArmed => ("HWARMD", "F5", fg, fm.color()),
        FlightMode::Armed => ("ARMED", "F6", fg, fm.color()),
        FlightMode::ArmedLaunchImminent => ("!ARMED!", "F6", fg, fm.color()),
        FlightMode::Burn => ("BURN", "F7", fg, fm.color()),
        FlightMode::Coast => ("COAST", "F8", fg, fm.color()),
        FlightMode::RecoveryDrogue => ("DROGUE", "F9", fg, fm.color()),
        FlightMode::RecoveryMain => ("MAIN", "F10", fg, fm.color()),
        FlightMode::Landed => ("LANDED", "F11", fg, fm.color()),
    }
}

pub trait TopBarUiExt {
    fn telemetry_value(&mut self, icon: &str, label: &str, value: Option<String>);
    fn nominal_value(
        &mut self,
        icon: &str,
        label: &str,
        value: Option<f32>,
        decimals: usize,
        nominal_min: f32,
        nominal_max: f32,
    );

    fn transmit_power_controls(&mut self, current: TransmitPower, data_source: &mut dyn DataSource);
    fn acs_mode_controls(&mut self, current: AcsMode, data_source: &mut dyn DataSource);
    fn acs_valve_controls(&mut self, current: ThrusterValveState, current_mode: AcsMode, data_source: &mut dyn DataSource);
    fn camera_controls(&mut self, current: [bool; 3], data_source: &mut dyn DataSource);
    fn io_module_indicators(&mut self, acs_present: bool, recovery_present: bool, payload_present: bool);
    fn fin_indicators(&mut self, fins_present: [bool; 3]);

    fn battery_bar(&mut self, w: f32, voltage: Option<f32>);
    fn flash_bar(&mut self, w: f32, flash_pointer: Option<u32>);
    fn command_button(&mut self, label: &'static str, cmd: Command, data_source: &mut dyn DataSource);
    fn flight_mode_button(
        &mut self,
        fm: FlightMode,
        current: Option<FlightMode>,
        data_source: &mut dyn DataSource,
    );
    fn flight_mode_buttons(&mut self, current: Option<FlightMode>, data_source: &mut dyn DataSource);
}

impl TopBarUiExt for egui::Ui {
    fn telemetry_value(&mut self, icon: &str, label: &str, value: Option<String>) {
        let value_string = value.clone().unwrap_or("N/A".to_string());
        let value_text = RichText::new(value_string).strong().monospace();

        self.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.add_sized([15.0, ui.available_height()], Label::new(icon));
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(5.0);
                if value.is_some() {
                    ui.label(value_text);
                } else {
                    ui.label(value_text.color(Color32::from_rgb(0xfa, 0xbd, 0x2f)));
                }
            });
        });
    }

    fn nominal_value(
        &mut self,
        icon: &str,
        label: &str,
        value: Option<f32>,
        decimals: usize,
        nominal_min: f32,
        nominal_max: f32,
    ) {
        let value_string = value.map(|v| format!("{0:.1$}", v, decimals)).unwrap_or("N/A".to_string());
        let value_text = RichText::new(value_string).strong().monospace();
        let nominal = value.map(|v| v > nominal_min && v < nominal_max).unwrap_or(false);

        self.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.add_sized([15.0, ui.available_height()], Label::new(icon));
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.add_space(5.0);
                if nominal {
                    ui.label(value_text.color(Color32::from_rgb(0xb8, 0xbb, 0x26)));
                } else {
                    ui.label(value_text.color(Color32::from_rgb(0xfa, 0xbd, 0x2f)));
                }
            });
        });
    }

    fn transmit_power_controls(&mut self, current: TransmitPower, data_source: &mut dyn DataSource) {
        use TransmitPower::*;

        self.horizontal(|ui| {
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P14dBm, "14")).clicked() {
                data_source.send_command(Command::SetTransmitPower(TransmitPower::P14dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P17dBm, "17")).clicked() {
                data_source.send_command(Command::SetTransmitPower(TransmitPower::P17dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P20dBm, "20")).clicked() {
                data_source.send_command(Command::SetTransmitPower(TransmitPower::P20dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P22dBm, "22")).clicked() {
                data_source.send_command(Command::SetTransmitPower(TransmitPower::P22dBm)).unwrap();
            }
        });
    }

    fn acs_mode_controls(&mut self, current: AcsMode, data_source: &mut dyn DataSource) {
        use AcsMode::*;

        self.horizontal(|ui| {
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Disabled, "OFF")).clicked() {
                data_source.send_command(Command::SetAcsMode(AcsMode::Disabled)).unwrap();
            }
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Auto, "AUTO")).clicked() {
                data_source.send_command(Command::SetAcsMode(AcsMode::Auto)).unwrap();
            }
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Manual, "MNL")).clicked() {
                data_source.send_command(Command::SetAcsMode(AcsMode::Manual)).unwrap();
            }
        });
    }

    fn acs_valve_controls(&mut self, current: ThrusterValveState, current_mode: AcsMode, data_source: &mut dyn DataSource) {
        use ThrusterValveState::*;

        let (accel, decel) = match current {
            ThrusterValveState::Closed => (false, false),
            ThrusterValveState::OpenAccel => (true, false),
            ThrusterValveState::OpenDecel => (false, true),
            ThrusterValveState::OpenBoth => (true, true),
        };

        self.horizontal(|ui| {
            ui.set_enabled(current_mode != AcsMode::Disabled);

            let response_accel = ui.add_sized([50.0, 20.0], SelectableLabel::new(accel, "ACCL"));
            let response_decel = ui.add_sized([50.0, 20.0], SelectableLabel::new(decel, "DECL"));

            if response_accel.is_pointer_button_down_on() || response_accel.dragged() {
                data_source.send_command(Command::SetAcsValveState(OpenAccel)).unwrap();
                ui.ctx().request_repaint_after(Duration::from_millis(20));
            }

            if response_decel.is_pointer_button_down_on() || response_decel.dragged() {
                data_source.send_command(Command::SetAcsValveState(OpenDecel)).unwrap();
                ui.ctx().request_repaint_after(Duration::from_millis(20));
            }

            if response_accel.drag_stopped() || response_decel.drag_stopped() {
                data_source.send_command(Command::SetAcsValveState(Closed)).unwrap();
            }
        });
    }

    // TODO: data structure
    fn camera_controls(&mut self, current: [bool; 3], data_source: &mut dyn DataSource) {
        self.horizontal(|ui| {
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[0], "R0")).clicked() {
                data_source.send_command(Command::SetIoModuleOutput(IoBoardRole::Recovery, 0, !current[0])).unwrap();
            }
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[1], "R1")).clicked() {
                data_source.send_command(Command::SetIoModuleOutput(IoBoardRole::Recovery, 1, !current[1])).unwrap();
            }
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[2], "P")).clicked() {
                data_source.send_command(Command::SetIoModuleOutput(IoBoardRole::Payload, 0, !current[2])).unwrap();
            }
        });
    }

    fn io_module_indicators(&mut self, acs_present: bool, recovery_present: bool, payload_present: bool) {
        self.horizontal(|ui| {
            let _ = ui.add_sized([40.0, 20.0], SelectableLabel::new(acs_present, "ACS"));
            let _ = ui.add_sized([40.0, 20.0], SelectableLabel::new(recovery_present, "REC"));
            let _ = ui.add_sized([40.0, 20.0], SelectableLabel::new(payload_present, "PAY"));
        });
    }

    fn fin_indicators(&mut self, fins_present: [bool; 3]) {
        self.horizontal(|ui| {
            let _ = ui.add_sized([20.0, 20.0], SelectableLabel::new(fins_present[0], "0"));
            let _ = ui.add_sized([20.0, 20.0], SelectableLabel::new(fins_present[1], "1"));
            let _ = ui.add_sized([20.0, 20.0], SelectableLabel::new(fins_present[2], "2"));
        });
    }

    fn battery_bar(&mut self, w: f32, voltage: Option<f32>) {
        let f = voltage.map(|v| (v - 6.0) / (8.4 - 6.0)).unwrap_or(0.0);
        let text = voltage.map(|v| format!("🔋 Battery: {:.2}V", v)).unwrap_or("🔋 Battery: N/A".to_string());
        let color = match voltage {
            Some(v) if v > 8.0 && v < 8.45 => Color32::from_rgb(0x98, 0x97, 0x1a),
            Some(v) if v > 7.0 => Color32::from_rgb(0xd6, 0x5d, 0x0e),
            _ => Color32::from_rgb(0xcc, 0x24, 0x1d),
        };

        self.horizontal(|ui| {
            ui.style_mut().visuals.selection.bg_fill = color;
            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn flash_bar(&mut self, w: f32, flash_pointer: Option<u32>) {
        let flash_pointer = flash_pointer.map(|fp| (fp as f32) / 1024.0 / 1024.0).unwrap_or_default();
        let flash_size = (FLASH_SIZE as f32) / 1024.0 / 1024.0;
        let f = flash_pointer / flash_size;
        let text = format!("🖴  Flash: {:.2}MiB / {:.2}MiB", flash_pointer, flash_size);

        self.horizontal(|ui| {
            ui.style_mut().visuals.selection.bg_fill = match f {
                _f if f > 0.9 => Color32::from_rgb(0xcc, 0x24, 0x1d),
                _f if f > 0.75 => Color32::from_rgb(0xd6, 0x5d, 0x0e),
                _ => Color32::from_rgb(0x45, 0x85, 0x88),
            };

            ui.add(ProgressBar::new(f).desired_width(w).text(text));
        });
    }

    fn command_button(&mut self, label: &'static str, cmd: Command, data_source: &mut dyn DataSource) {
        if self.button(label).clicked() {
            data_source.send_command(cmd).unwrap();
        }
    }

    fn flight_mode_button(
        &mut self,
        fm: FlightMode,
        current: Option<FlightMode>,
        data_source: &mut dyn DataSource,
    ) {
        let fm = match (current, fm) {
            (Some(FlightMode::ArmedLaunchImminent), FlightMode::Armed) => FlightMode::ArmedLaunchImminent,
            _ => fm,
        };

        let (label, shortcut, fg, bg) = flight_mode_style(fm);
        let is_current = current.map(|c| c == fm).unwrap_or(false);

        let main_text = RichText::new(label).monospace();
        let button = if is_current {
            Button::new(main_text.color(fg)).wrap(true).fill(bg)
        } else {
            Button::new(main_text)
                .wrap(true)
                .fill(Color32::TRANSPARENT)
                .stroke(Stroke::new(2.0, bg))
        };

        self.add_space(5.0);
        let pos = self.next_widget_position();
        let size = Vec2::new(self.available_width(), 50.0);

        let shortcut = if is_current {
            Label::new(RichText::new(format!("Shift+{}", shortcut)).size(9.0).color(fg))
        } else {
            Label::new(RichText::new(format!("Shift+{}", shortcut)).size(9.0).weak())
        };

        self.put(Rect::from_two_pos(pos + size * Vec2::new(0.0, 0.6), pos + size), shortcut);
        if self.put(Rect::from_two_pos(pos, pos + size), button).clicked() {
            data_source.send_command(Command::SetFlightMode(fm)).unwrap();
        }
    }

    fn flight_mode_buttons(&mut self, current: Option<FlightMode>, data_source: &mut dyn DataSource) {
        self.columns(8, |columns| {
            columns[0].flight_mode_button(FlightMode::Idle, current, data_source);
            columns[1].flight_mode_button(FlightMode::HardwareArmed, current, data_source);
            columns[2].flight_mode_button(FlightMode::Armed, current, data_source);
            columns[3].flight_mode_button(FlightMode::Burn, current, data_source);
            columns[4].flight_mode_button(FlightMode::Coast, current, data_source);
            columns[5].flight_mode_button(FlightMode::RecoveryDrogue, current, data_source);
            columns[6].flight_mode_button(FlightMode::RecoveryMain, current, data_source);
            columns[7].flight_mode_button(FlightMode::Landed, current, data_source);
        });
    }
}
