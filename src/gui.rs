use std::sync::mpsc::Receiver;

use eframe::egui::{self, Ui};
use egui::widgets::plot::{Corner, Legend, Line};
use egui::{Color32, Layout, RichText, Vec2};

use euroc_fc_firmware::telemetry::*;

use crate::serial::*;
use crate::state::*;

#[rustfmt::skip]
const ZOOM_LEVELS: [u32; 14] = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 60000, 1200000, 300000, 600000, 1800000, 3600000];
const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;

struct Sam {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<DownlinkMessage>,

    serial_port: Option<String>,
    serial_status: SerialStatus,

    vehicle_states: Vec<VehicleState>,
    log_messages: Vec<(u32, String, LogLevel, String)>,

    zoom_level: usize,

    logo: egui_extras::RetainedImage,
}

impl Sam {
    pub fn init(
        serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
        downlink_rx: Receiver<DownlinkMessage>,
    ) -> Self {
        Self {
            serial_status_rx,
            downlink_rx,
            serial_port: None,
            serial_status: SerialStatus::Init,
            vehicle_states: Vec::new(),
            log_messages: Vec::new(),
            zoom_level: ZOOM_LEVELS.iter().position(|zl| *zl == 10000).unwrap(),
            logo: egui_extras::RetainedImage::from_svg_bytes("athena.svg", include_bytes!("athena.svg")).unwrap(),
        }
    }

    fn process_telemetry(&mut self, msg: DownlinkMessage) {
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

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());

            let top_bar_height = ui.available_height() / 10.0;
            let bottom_bar_height = 20.0;
            let log_height = ui.available_height() / 5.0 - 10.0;
            let plot_height = ui.available_height() - top_bar_height - log_height - bottom_bar_height - 50.0;

            ui.horizontal(|ui| {
                ui.set_height(top_bar_height);

                let max_size = ui.available_size();
                self.logo.show_max_size(ui, max_size);
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
                            ("Roll (y) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.1 * RAD_TO_DEG))),
                            ("Roll (Z) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.2 * RAD_TO_DEG))),
                        ],
                    );

                    self.plot(ui, "Vert. Speed & Accel.", Vec::new());

                    self.plot(ui, "Altitude", Vec::new());

                    self.plot(ui, "Position (TODO)", Vec::new());

                    ui.end_row();

                    self.plot(
                        ui,
                        "Gyroscope",
                        vec![
                            ("Gyro (X) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.0))),
                            ("Gyro (y) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.1))),
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
                            ("Altitude (ASL) [m]", Box::new(|vs| vs.altitude_baro)),
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
                            ("Battery Voltage [V]", Box::new(|vs| vs.battery_voltage.map(|mv| mv as f32 / 1000.0))),
                            ("Arm Voltage [V]", Box::new(|vs| vs.arm_voltage.map(|mv| mv as f32 / 1000.0))),
                            ("Current [A]", Box::new(|vs| vs.current.map(|ma| ma as f32 / 1000.0))),
                            ("Core Voltage [V]", Box::new(|vs| vs.cpu_voltage.map(|mv| mv as f32 / 1000.0))),
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
                            ("RSSI [dBm]", Box::new(|vs| vs.lora_rssi.map(|x| x as f32 / -2.0))),
                            ("Signal RSSI [-dBm]", Box::new(|vs| vs.lora_rssi_signal.map(|x| x as f32 / -2.0))),
                            ("SNR [dB]", Box::new(|vs| vs.lora_snr.map(|x| x as f32 / 4.0))),
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
                ui.set_height(bottom_bar_height);

                match self.serial_status {
                    SerialStatus::Connected => {
                        ui.label(RichText::new("Connected").color(Color32::GREEN));
                        ui.label(format!("to {}", self.serial_port.as_ref().unwrap_or(&"".to_string())));
                    }
                    SerialStatus::Error => {
                        ui.label(RichText::new("Connection lost").color(Color32::RED));
                        ui.label(format!("to {}", self.serial_port.as_ref().unwrap_or(&"".to_string())));
                    }
                    _ => {}
                }
            });

            // TODO: serial status & plot buttons
        });

        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

pub fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (downlink_tx, downlink_rx) = std::sync::mpsc::channel::<DownlinkMessage>();
    let (serial_status_tx, serial_status_rx) = std::sync::mpsc::channel::<(SerialStatus, Option<String>)>();

    let serial_thread = spawn_downlink_monitor(serial_status_tx, downlink_tx);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(1000.0, 700.0)),
            ..Default::default()
        },
        Box::new(|_cc| Box::new(Sam::init(serial_status_rx, downlink_rx))),
    );

    serial_thread.join().unwrap();

    Ok(())
}
