use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, Sender};
use std::time::{Duration, Instant};

use eframe::egui::{self, Ui};
use egui::widgets::plot::{Corner, Legend, Line};
use egui::{Color32, Layout, Modifiers, Key, RichText, Stroke, Vec2};

use euroc_fc_firmware::telemetry::*;

use crate::serial::*;
use crate::state::*;

#[rustfmt::skip]
const ZOOM_LEVELS: [u32; 14] = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 60000, 1200000, 300000, 600000, 1800000, 3600000];
const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;

struct Sam {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<DownlinkMessage>,
    uplink_tx: Sender<UplinkMessage>,

    serial_port: Option<String>,
    serial_status: SerialStatus,

    vehicle_states: Vec<VehicleState>,
    message_receipt_times: VecDeque<Instant>,
    log_messages: Vec<(u32, String, LogLevel, String)>,

    zoom_level: usize,

    logo: egui_extras::RetainedImage,
}

impl Sam {
    pub fn init(
        serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
        downlink_rx: Receiver<DownlinkMessage>,
        uplink_tx: Sender<UplinkMessage>,
    ) -> Self {
        Self {
            serial_status_rx,
            downlink_rx,
            uplink_tx,
            serial_port: None,
            serial_status: SerialStatus::Init,
            message_receipt_times: VecDeque::new(),
            vehicle_states: Vec::new(),
            log_messages: Vec::new(),
            zoom_level: ZOOM_LEVELS.iter().position(|zl| *zl == 10000).unwrap(),
            logo: egui_extras::RetainedImage::from_image_bytes("logo.png", include_bytes!("logo.png")).unwrap(),
        }
    }

    fn process_telemetry(&mut self, msg: DownlinkMessage) {
        match msg {
            DownlinkMessage::TelemetryGCS(_) => self.message_receipt_times.push_back(Instant::now()),
            _ => {}
        }

        let time = msg.time();

        // Clear history if this seems to be a new run
        // TODO: how wrong can this go?
        if self
            .vehicle_states
            .last()
            .map(|vs| time + 1000 < vs.time)
            .unwrap_or(false)
        {
            self.vehicle_states.truncate(0);
            //self.log_messages.truncate(0);
        }

        if let DownlinkMessage::Log(t, l, ll, m) = msg {
            self.log_messages.push((t, l, ll, m));
            return;
        }

        // TODO: also check recent history if a packet took a while
        let time = msg.time();
        if self.vehicle_states.last().map(|vs| vs.time < time).unwrap_or(true) {
            self.vehicle_states.push(VehicleState {
                time,
                ..Default::default()
            });
        }

        let i = self.vehicle_states.len() - 1;
        self.vehicle_states[i].incorporate_telemetry(&msg);
    }

    fn visible_state_indices(&self) -> usize {
        // TODO: do end as well
        let last_t = self.vehicle_states.last().map(|vs| vs.time).unwrap_or(0);
        match self
            .vehicle_states
            .binary_search_by(|vs| (vs.time + ZOOM_LEVELS[self.zoom_level]).cmp(&last_t))
        {
            Ok(i) => i,
            Err(i) => i,
        }
    }

    fn plot(&self, ui: &mut Ui, heading: &str, callbacks: Vec<(&str, Box<dyn Fn(&VehicleState) -> Option<f32>>)>) {
        let start_i = self.visible_state_indices();

        let lines = callbacks.iter().map(|(name, callback)| {
            let points: Vec<[f64; 2]> = self.vehicle_states[start_i..]
                .iter()
                .map(|vs| (vs.time as f64 / 1000.0, callback(vs)))
                .filter(|(_t, y)| y.is_some())
                .map(|(t, y)| [t, y.unwrap() as f64])
                .collect();
            Line::new(points).name(name)
        });

        let legend = Legend::default().background_alpha(0.5).position(Corner::LeftTop);

        ui.vertical_centered(|ui| {
            ui.heading(heading);
            egui::widgets::plot::Plot::new(heading)
                .allow_drag(false)
                .allow_scroll(false)
                .allow_zoom(false)
                .allow_boxed_zoom(false)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .legend(legend.clone())
                .show(ui, |plot_ui| {
                    for l in lines {
                        plot_ui.line(l);
                    }
                });
        });
    }

    fn last_value<T>(&self, callback: Box<dyn Fn(&VehicleState) -> Option<T>>) -> Option<T> {
        self.vehicle_states.iter()
            .rev()
            .find_map(|vs| callback(vs))
    }

    fn text_indicator(&self, ui: &mut Ui, label: &str, value: Option<String>) {
        ui.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::TOP), |ui| {
                ui.label(RichText::new(value.unwrap_or_default()).strong().monospace());
            });
        });
    }

    fn flight_mode_button(&mut self, ui: &mut Ui, width: f32, fm: FlightMode) {
        let (label, fg, bg) = match fm {
            FlightMode::Idle => ("IDLE", Color32::BLACK, Color32::GRAY),
            FlightMode::HardwareArmed => ("HWARMD", Color32::BLACK, Color32::KHAKI),
            FlightMode::Armed => ("ARMD", Color32::WHITE, Color32::DARK_RED),
            FlightMode::Flight => ("FLIGHT", Color32::BLACK, Color32::LIGHT_BLUE),
            FlightMode::RecoveryDrogue => ("DROGUE", Color32::BLACK, Color32::LIGHT_GREEN),
            FlightMode::RecoveryMain => ("MAIN", Color32::BLACK, Color32::GREEN),
            FlightMode::Landed => ("LANDED", Color32::BLACK, Color32::GRAY),
        };

        let button = if self.last_value(Box::new(|vs| vs.mode)).map(|m| m == fm).unwrap_or(false) {
            let label = RichText::new(label).monospace().color(fg);
            egui::Button::new(label).fill(bg)
        } else {
            let label = RichText::new(label).monospace().color(bg);
            egui::Button::new(label).fill(Color32::TRANSPARENT).stroke(Stroke::new(2.0, bg))
        };

        if ui.add_sized([width, 50.0], button).clicked() {
            self.uplink_tx.send(UplinkMessage::SetFlightMode(fm)).unwrap();
        }
    }
}

impl eframe::App for Sam {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for (status, port) in self.serial_status_rx.try_iter() {
            self.serial_status = status;
            self.serial_port = port;
        }

        let msgs: Vec<DownlinkMessage> = self.downlink_rx.try_iter().collect();
        for msg in msgs.into_iter() {
            self.process_telemetry(msg);
        }

        self.message_receipt_times.retain(|i| i.elapsed() < Duration::from_millis(1000));

        {
            let input = ctx.input();
            if input.modifiers.command_only() && input.key_down(Key::Num0) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::Idle)).unwrap();
            } else if input.modifiers.command_only() && input.key_down(Key::A) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::Armed)).unwrap();
            } else if input.modifiers.command_only() && input.key_down(Key::I) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::Flight)).unwrap();
            } else if input.modifiers.command_only() && input.key_down(Key::D) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::RecoveryDrogue)).unwrap();
            } else if input.modifiers.command_only() && input.key_down(Key::M) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::RecoveryMain)).unwrap();
            } else if input.modifiers.command_only() && input.key_down(Key::L) {
                self.uplink_tx.send(UplinkMessage::SetFlightMode(FlightMode::Landed)).unwrap();
            }
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());

            let top_bar_height = 85.0;
            //let top_bar_height = 0.0;
            let bottom_bar_height = 15.0;
            let log_height = ui.available_height() / 5.0 - 10.0;
            let plot_height = ui.available_height() - top_bar_height - log_height - bottom_bar_height - 50.0;

            ui.horizontal(|ui| {
                ui.set_height(top_bar_height);

                let max_size = ui.available_size();
                self.logo.show_max_size(ui, max_size);

                let buttons_w = f32::min(500.0, ui.available_width() * 0.45);
                let gauge_w = ui.available_width() - buttons_w - 80.0;

                ui.horizontal_centered(|ui| {
                    ui.set_width(gauge_w);
                    let w = ui.available_width() / 3.0;

                    ui.vertical(|ui| {
                        ui.set_width(w);
                        self.text_indicator(ui, "Time [s]", self.vehicle_states.last().map(|vs| format!("{:10.3}", (vs.time as f32) / 1000.0)));
                        self.text_indicator(ui, "Mode", self.last_value(Box::new(|vs| vs.mode)).map(|s| format!("{:?}", s)));
                        self.text_indicator(ui, "Bat. Voltage [V]", self.last_value(Box::new(|vs| vs.battery_voltage)).map(|v| format!("{:.2}", v)));
                        self.text_indicator(ui, "Vertical Speed [m/s]", self.last_value(Box::new(|vs| vs.vertical_speed)).map(|v| format!("{:.2}", v)));
                    });

                    ui.vertical(|ui| {
                        ui.set_width(w);
                        self.text_indicator(ui, "Altitude (AGL/ASL) [m]", self.last_value(Box::new(|vs| vs.altitude)).map(|a| format!("{:.1} ({:.1})", a, a)));
                        self.text_indicator(ui, "Max Altitude (AGL/ASL) [m]", None); // TODO
                        self.text_indicator(ui, "Altitude (Baro, ASL) [m]", self.last_value(Box::new(|vs| vs.altitude_baro)).map(|a| format!("{:.1}", a)));
                        self.text_indicator(ui, "Altitude (GPS, ASL) [m]", self.last_value(Box::new(|vs| vs.altitude_gps)).map(|a| format!("{:.1}", a)));
                    });

                    let last_gps_msg = self.vehicle_states.iter().rev().find_map(|vs| vs.gps_fix.is_some().then(|| vs));

                    ui.vertical(|ui| {
                        ui.set_width(w);
                        self.text_indicator(ui, "GPS Status (#Sats)", last_gps_msg.map(|vs| format!("{:?} ({})", vs.gps_fix.unwrap(), vs.num_satellites.unwrap_or(0))));
                        self.text_indicator(ui, "HDOP", last_gps_msg.map(|vs| format!("{:.2}", vs.hdop.unwrap_or(9999) as f32 / 100.0)));
                        self.text_indicator(ui, "Latitude", last_gps_msg.and_then(|vs| vs.latitude).map(|l| format!("{:.6}", l)));
                        self.text_indicator(ui, "Longitude", last_gps_msg.and_then(|vs| vs.longitude).map(|l| format!("{:.6}", l)));
                    });
                });

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.set_width(buttons_w);
                    let w = ui.available_width() / 7.0;
                    self.flight_mode_button(ui, w, FlightMode::Landed);
                    self.flight_mode_button(ui, w, FlightMode::RecoveryMain);
                    self.flight_mode_button(ui, w, FlightMode::RecoveryDrogue);
                    self.flight_mode_button(ui, w, FlightMode::Flight);
                    self.flight_mode_button(ui, w, FlightMode::Armed);
                    self.flight_mode_button(ui, w, FlightMode::HardwareArmed);
                    self.flight_mode_button(ui, w, FlightMode::Idle);
                });
            });

            ui.separator();

            egui::Grid::new("plot_grid")
                .min_col_width(ui.available_width() / 4.0)
                .max_col_width(ui.available_width() / 4.0)
                .min_row_height(plot_height / 3.0)
                .show(ui, |ui| {
                    self.plot(
                        ui,
                        "Orientation",
                        vec![
                            ("Roll (X) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.0 * RAD_TO_DEG))),
                            ("Roll (Y) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.1 * RAD_TO_DEG))),
                            ("Roll (Z) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.2 * RAD_TO_DEG))),
                        ],
                    );

                    self.plot(ui, "Vert. Speed & Accel.",
                        vec![
                            ("Vario [m/s]", Box::new(|vs| vs.vertical_speed)),
                            ("Vertical Accel [m/s²]", Box::new(|vs| vs.vertical_accel)),
                            ("Vertical Accel (Filt.) [m/s²]", Box::new(|vs| vs.vertical_accel_filtered)),
                        ],
                    );

                    self.plot(ui, "Altitude (ASL)",
                        vec![
                            ("Altitude [m]", Box::new(|vs| vs.altitude)),
                            ("Altitude (Baro) [m]", Box::new(|vs| vs.altitude_baro)),
                            ("Altitude (GPS) [m]", Box::new(|vs| vs.altitude_gps)),
                        ],
                    );

                    self.plot(ui, "Position (TODO)", Vec::new());

                    ui.end_row();

                    self.plot(
                        ui,
                        "Gyroscope",
                        vec![
                            ("Gyro (X) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.0))),
                            ("Gyro (Y) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.1))),
                            ("Gyro (Z) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.2))),
                        ],
                    );

                    self.plot(
                        ui,
                        "Accelerometers",
                        vec![
                            ("Accel 2 (X) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.0))),
                            ("Accel 2 (Y) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.1))),
                            ("Accel 2 (Z) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.2))),
                            ("Accel 1 (X) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.0))),
                            ("Accel 1 (Y) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.1))),
                            ("Accel 1 (Z) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.2))),
                        ],
                    );

                    self.plot(
                        ui,
                        "Magnetometer",
                        vec![
                            ("Mag (X) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.0))),
                            ("Mag (Y) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.1))),
                            ("Mag (Z) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.2))),
                        ],
                    );

                    self.plot(
                        ui,
                        "Barometer",
                        vec![
                            ("Pressure [mbar]", Box::new(|vs| vs.pressure)),
                            //("Altitude (ASL) [m]", Box::new(|vs| vs.altitude_baro)),
                        ],
                    );

                    ui.end_row();

                    self.plot(
                        ui,
                        "Temperatures",
                        vec![
                            ("Baro. Temp. [°C]", Box::new(|vs| vs.temperature_baro)),
                            ("Core Temp. [°C]", Box::new(|vs| vs.temperature_core)),
                        ],
                    );

                    #[rustfmt::skip]
                    self.plot(
                        ui,
                        "Power",
                        vec![
                            ("Battery Voltage [V]", Box::new(|vs| vs.battery_voltage)),
                            ("Arm Voltage [V]", Box::new(|vs| vs.arm_voltage)),
                            ("Current [A]", Box::new(|vs| vs.current)),
                            ("Core Voltage [V]", Box::new(|vs| vs.cpu_voltage)),
                        ],
                    );

                    self.plot(
                        ui,
                        "Runtime",
                        vec![("Loop Runtime [µs]", Box::new(|vs| vs.loop_runtime.map(|lr| lr as f32)))],
                    );

                    #[rustfmt::skip]
                    self.plot(
                        ui,
                        "Signal",
                        vec![
                            ("GCS RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))),
                            ("GCS Signal RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))),
                            ("GCS SNR [dB]", Box::new(|vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))),
                            ("Vehicle RSSI [dBm]", Box::new(|vs| vs.vehicle_lora_rssi.map(|x| x as f32 / -2.0))),
                        ],
                    );
                });

            ui.separator();

            egui::ScrollArea::vertical()
                .max_height(log_height)
                .max_width(ui.available_width())
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    ui.set_min_height(log_height);
                    ui.set_width(ui.available_width());

                    for (t, loc, ll, msg) in self.log_messages.iter() {
                        let color = match ll {
                            LogLevel::Debug => Color32::LIGHT_BLUE,
                            LogLevel::Info => Color32::GREEN,
                            LogLevel::Warning => Color32::YELLOW,
                            LogLevel::Error => Color32::RED,
                            LogLevel::Critical => Color32::DARK_RED,
                        };

                        ui.horizontal(|ui| {
                            ui.allocate_ui_with_layout(
                                Vec2::new(60.0, 10.0),
                                Layout::top_down(eframe::emath::Align::RIGHT),
                                |ui| {
                                    ui.label(
                                        RichText::new(format!("{:>8.3}", *t as f32 / 1000.0)).color(Color32::GRAY),
                                    );
                                },
                            );

                            ui.allocate_ui_with_layout(
                                Vec2::new(70.0, 10.0),
                                Layout::top_down(eframe::emath::Align::LEFT),
                                |ui| {
                                    ui.set_width(ui.available_width());
                                    ui.label(RichText::new(ll.to_string()).color(color));
                                },
                            );

                            ui.allocate_ui_with_layout(
                                Vec2::new(150.0, 10.0),
                                Layout::top_down(eframe::emath::Align::LEFT),
                                |ui| {
                                    ui.set_width(ui.available_width());
                                    ui.label(RichText::new(loc).color(Color32::GRAY));
                                },
                            );

                            ui.allocate_ui_with_layout(
                                Vec2::new(9999.0, 10.0),
                                Layout::top_down(eframe::emath::Align::LEFT),
                                |ui| {
                                    ui.set_width(ui.available_width());
                                    ui.label(RichText::new(msg).color(Color32::LIGHT_GRAY));
                                },
                            );
                        });
                    }
                });

            ui.separator();

            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width());
                ui.set_height(bottom_bar_height);

                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width()/4.0);
                    match self.serial_status {
                        SerialStatus::Connected => {
                            ui.label(RichText::new("Connected").color(Color32::GREEN));
                            ui.label(format!("to {} (1s: {})",
                                self.serial_port.as_ref().unwrap_or(&"".to_string()),
                                self.message_receipt_times.len()
                            ));
                        }
                        SerialStatus::Error => {
                            ui.label(RichText::new("Connection lost").color(Color32::RED));
                            ui.label(format!("to {}", self.serial_port.as_ref().unwrap_or(&"".to_string())));
                        }
                        _ => {}
                    }
                });
            });

            // TODO: serial status & plot buttons
        });

        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (downlink_tx, downlink_rx) = std::sync::mpsc::channel::<DownlinkMessage>();
    let (uplink_tx, uplink_rx) = std::sync::mpsc::channel::<UplinkMessage>();
    let (serial_status_tx, serial_status_rx) = std::sync::mpsc::channel::<(SerialStatus, Option<String>)>();

    let serial_thread = spawn_downlink_monitor(serial_status_tx, downlink_tx, uplink_rx);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(1000.0, 700.0)),
            ..Default::default()
        },
        Box::new(|_cc| Box::new(Sam::init(serial_status_rx, downlink_rx, uplink_tx))),
    );

    serial_thread.join().unwrap();

    Ok(())
}
