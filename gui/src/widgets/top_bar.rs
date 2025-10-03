//! Contains smaller UI functions for the top bar, including text indicators
//! and flight mode buttons.

use std::time::Duration;

use eframe::egui;
use egui::widgets::ProgressBar;
use egui::{Color32, Label, RichText, SelectableLabel};

use shared_types::{can::IoBoardRole, telemetry::*};

use crate::Backend;

pub trait TopBarUiExt {
    fn telemetry_value(&mut self, icon: &str, label: &str, value: Option<String>);
    fn nominal_value(
        &mut self,
        icon: &str,
        label: &str,
        value: Option<f64>,
        decimals: usize,
        nominal_min: f64,
        nominal_max: f64,
    );

    fn transmit_power_controls(&mut self, current: TransmitPower, backend: &mut Backend);
    fn acs_mode_controls(&mut self, current: AcsMode, backend: &mut Backend);
    fn acs_valve_controls(&mut self, current: ThrusterValveState, current_mode: AcsMode, backend: &mut Backend);
    fn camera_controls(&mut self, current: [bool; 3], backend: &mut Backend);
    fn io_module_indicators(&mut self, acs_present: bool, recovery_present: bool, payload_present: bool);
    fn fin_indicators(&mut self, fins_present: [bool; 3]);

    fn battery_bar(&mut self, w: f32, voltage: Option<f64>);
    fn flash_bar(&mut self, w: f32, flash_pointer: Option<f64>);
    fn command_button(&mut self, label: &'static str, cmd: Command, backend: &mut Backend);
    //fn flight_mode_button(&mut self, fm: FlightMode, current: Option<FlightMode>, backend: &mut Backend);
    //fn flight_mode_buttons(&mut self, current: Option<FlightMode>, backend: &mut Backend);
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
        value: Option<f64>,
        decimals: usize,
        nominal_min: f64,
        nominal_max: f64,
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

    fn transmit_power_controls(&mut self, current: TransmitPower, backend: &mut Backend) {
        use TransmitPower::*;

        self.horizontal(|ui| {
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P14dBm, "14")).clicked() {
                backend.send_command(Command::SetTransmitPower(TransmitPower::P14dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P17dBm, "17")).clicked() {
                backend.send_command(Command::SetTransmitPower(TransmitPower::P17dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P20dBm, "20")).clicked() {
                backend.send_command(Command::SetTransmitPower(TransmitPower::P20dBm)).unwrap();
            }
            if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == P22dBm, "22")).clicked() {
                backend.send_command(Command::SetTransmitPower(TransmitPower::P22dBm)).unwrap();
            }
        });
    }

    fn acs_mode_controls(&mut self, current: AcsMode, backend: &mut Backend) {
        use AcsMode::*;

        self.horizontal(|ui| {
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Disabled, "OFF")).clicked() {
                backend.send_command(Command::SetAcsMode(AcsMode::Disabled)).unwrap();
            }
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Auto, "AUTO")).clicked() {
                backend.send_command(Command::SetAcsMode(AcsMode::Auto)).unwrap();
            }
            if ui.add_sized([33.0, 20.0], SelectableLabel::new(current == Manual, "MNL")).clicked() {
                backend.send_command(Command::SetAcsMode(AcsMode::Manual)).unwrap();
            }
        });
    }

    fn acs_valve_controls(&mut self, current: ThrusterValveState, current_mode: AcsMode, backend: &mut Backend) {
        use ThrusterValveState::*;

        let (accel, decel) = match current {
            ThrusterValveState::Closed => (false, false),
            ThrusterValveState::OpenAccel => (true, false),
            ThrusterValveState::OpenDecel => (false, true),
            ThrusterValveState::OpenBoth => (true, true),
        };

        self.horizontal(|ui| {
            if current_mode == AcsMode::Disabled {
                ui.disable();
            }

            let response_accel = ui.add_sized([50.0, 20.0], SelectableLabel::new(accel, "ACCL"));
            let response_decel = ui.add_sized([50.0, 20.0], SelectableLabel::new(decel, "DECL"));

            if response_accel.is_pointer_button_down_on() || response_accel.dragged() {
                backend.send_command(Command::SetAcsValveState(OpenAccel)).unwrap();
                ui.ctx().request_repaint_after(Duration::from_millis(20));
            }

            if response_decel.is_pointer_button_down_on() || response_decel.dragged() {
                backend.send_command(Command::SetAcsValveState(OpenDecel)).unwrap();
                ui.ctx().request_repaint_after(Duration::from_millis(20));
            }

            if response_accel.drag_stopped() || response_decel.drag_stopped() {
                backend.send_command(Command::SetAcsValveState(Closed)).unwrap();
            }
        });
    }

    // TODO: data structure
    fn camera_controls(&mut self, current: [bool; 3], backend: &mut Backend) {
        self.horizontal(|ui| {
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[0], "R0")).clicked() {
                backend.send_command(Command::SetIoModuleOutput(IoBoardRole::Recovery, 0, !current[0])).unwrap();
            }
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[1], "R1")).clicked() {
                backend.send_command(Command::SetIoModuleOutput(IoBoardRole::Recovery, 1, !current[1])).unwrap();
            }
            if ui.add_sized([20.0, 20.0], SelectableLabel::new(current[2], "P")).clicked() {
                backend.send_command(Command::SetIoModuleOutput(IoBoardRole::Payload, 0, !current[2])).unwrap();
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

    fn battery_bar(&mut self, w: f32, voltage: Option<f64>) {
        let f = voltage.map(|v| (v - 6.0) / (8.4 - 6.0)).unwrap_or(0.0);
        let text = voltage.map(|v| format!("🔋 Battery: {:.2}V", v)).unwrap_or("🔋 Battery: N/A".to_string());
        let color = match voltage {
            Some(v) if v > 8.0 && v < 8.45 => Color32::from_rgb(0x98, 0x97, 0x1a),
            Some(v) if v > 7.0 => Color32::from_rgb(0xd6, 0x5d, 0x0e),
            _ => Color32::from_rgb(0xcc, 0x24, 0x1d),
        };

        self.horizontal(|ui| {
            ui.style_mut().visuals.selection.bg_fill = color;
            ui.add(ProgressBar::new(f as f32).desired_width(w).text(text));
        });
    }

    fn flash_bar(&mut self, w: f32, flash_pointer: Option<f64>) {
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

    fn command_button(&mut self, label: &'static str, cmd: Command, backend: &mut Backend) {
        if self.button(label).clicked() {
            if let Err(e) = backend.send_command(cmd.clone()) {
                println!("Error sending command: {:?}. {}", cmd, e);
            }
        }
    }

    // fn flight_mode_button(&mut self, fm: FlightMode, current: Option<FlightMode>, backend: &mut Backend) {
    //     let fm = match (current, fm) {
    //         (Some(FlightMode::ArmedLaunchImminent), FlightMode::Armed) => FlightMode::ArmedLaunchImminent,
    //         _ => fm,
    //     };

    //     let (label, shortcut, fg, bg) = flight_mode_style(fm);
    //     let is_current = current.map(|c| c == fm).unwrap_or(false);

    //     let main_text = RichText::new(label).monospace();
    //     let button = if is_current {
    //         Button::new(main_text.color(fg)).wrap().fill(bg)
    //     } else {
    //         Button::new(main_text).wrap().fill(Color32::TRANSPARENT).stroke(Stroke::new(2.0, bg))
    //     };

    //     self.add_space(5.0);
    //     let pos = self.next_widget_position();
    //     let size = Vec2::new(self.available_width(), 50.0);

    //     let shortcut = if is_current {
    //         Label::new(RichText::new(format!("Shift+{}", shortcut)).size(9.0).color(fg))
    //     } else {
    //         Label::new(RichText::new(format!("Shift+{}", shortcut)).size(9.0).weak())
    //     };

    //     self.put(Rect::from_two_pos(pos + size * Vec2::new(0.0, 0.6), pos + size), shortcut);
    //     if self.put(Rect::from_two_pos(pos, pos + size), button).clicked() {
    //         backend.send_command(Command::SetFlightMode(fm)).unwrap();
    //     }
    // }

    // fn flight_mode_buttons(&mut self, current: Option<FlightMode>, backend: &mut Backend) {
    //     self.columns(8, |columns| {
    //         columns[0].flight_mode_button(FlightMode::Idle, current, backend);
    //         columns[1].flight_mode_button(FlightMode::HardwareArmed, current, backend);
    //         columns[2].flight_mode_button(FlightMode::Armed, current, backend);
    //         columns[3].flight_mode_button(FlightMode::Burn, current, backend);
    //         columns[4].flight_mode_button(FlightMode::Coast, current, backend);
    //         columns[5].flight_mode_button(FlightMode::RecoveryDrogue, current, backend);
    //         columns[6].flight_mode_button(FlightMode::RecoveryMain, current, backend);
    //         columns[7].flight_mode_button(FlightMode::Landed, current, backend);
    //     });
    // }
}
