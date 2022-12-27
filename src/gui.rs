use std::path::PathBuf;

use eframe::egui::{self, Ui};
use egui::widgets::plot::{LinkedAxisGroup, LinkedCursorsGroup};
use egui::widgets::{Button, ProgressBar};
use egui::FontFamily::Proportional;
use egui::FontId;
use egui::TextStyle::*;
use egui::{Color32, Key, Layout, RichText, Stroke, Vec2};
use log::*;

use euroc_fc_firmware::telemetry::*;

mod plot;
mod map;
mod log_scroller;

use crate::state::*;
use crate::data_source::*;
use crate::telemetry_ext::*;
use crate::gui::plot::*;
use crate::gui::map::*;
use crate::gui::log_scroller::*;

#[rustfmt::skip]
const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;
const ZOOM_FACTOR: f64 = 2.0;

struct Sam {
    data_source: Box<dyn DataSource>,

    vehicle_states: Vec<VehicleState>,
    log_messages: Vec<(u32, String, LogLevel, String)>,

    auto_reset: bool,

    logo: egui_extras::RetainedImage,

    xlen: f64,
    axes: LinkedAxisGroup,
    cursor: LinkedCursorsGroup,

    orientation_plot_cache: PlotCache,
    vertical_speed_plot_cache: PlotCache,
    altitude_plot_cache: PlotCache,
    gyroscope_plot_cache: PlotCache,
    accelerometer_plot_cache: PlotCache,
    magnetometer_plot_cache: PlotCache,
    barometer_plot_cache: PlotCache,
    temperature_plot_cache: PlotCache,
    power_plot_cache: PlotCache,
    runtime_plot_cache: PlotCache,
    signal_plot_cache: PlotCache,
}

impl Sam {
    pub fn init(data_source: Box<dyn DataSource>) -> Self {
        let orientation_plot_cache = PlotCache::new(
            "Orientation",
            vec![
                ("Pitch (X) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.0 * RAD_TO_DEG))),
                ("Pitch (Y) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.1 * RAD_TO_DEG))),
                ("Roll (Z) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.2 * RAD_TO_DEG))),
            ],
            (Some(-180.0), Some(180.0)),
        );

        let vertical_speed_plot_cache = PlotCache::new(
            "Vert. Speed & Accel.",
            vec![
                ("Vario [m/s]", Box::new(|vs| vs.vertical_speed)),
                ("Vertical Accel [m/s²]", Box::new(|vs| vs.vertical_accel)),
                (
                    "Vertical Accel (Filt.) [m/s²]",
                    Box::new(|vs| vs.vertical_accel_filtered),
                ),
            ],
            (Some(-1.0), Some(1.0)),
        );

        let altitude_plot_cache = PlotCache::new(
            "Altitude (ASL)",
            vec![
                ("Altitude [m]", Box::new(|vs| vs.altitude)),
                ("Altitude (Baro) [m]", Box::new(|vs| vs.altitude_baro)),
                ("Altitude (GPS) [m]", Box::new(|vs| vs.altitude_gps)),
                ("Altitude (Max) [m]", Box::new(|vs| vs.altitude_max)),
                ("Altitude (Ground) [m]", Box::new(|vs| vs.altitude_ground)),
            ],
            (Some(0.0), Some(300.0)) // TODO
        );

        let gyroscope_plot_cache = PlotCache::new(
            "Gyroscope",
            vec![
                ("Gyro (X) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.0))),
                ("Gyro (Y) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.1))),
                ("Gyro (Z) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.2))),
            ],
            (Some(-10.0), Some(10.0)),
        );

        let accelerometer_plot_cache = PlotCache::new(
            "Accelerometers",
            vec![
                ("Accel 2 (X) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.0))),
                ("Accel 2 (Y) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.1))),
                ("Accel 2 (Z) [m/s²]", Box::new(|vs| vs.accelerometer2.map(|a| a.2))),
                ("Accel 1 (X) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.0))),
                ("Accel 1 (Y) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.1))),
                ("Accel 1 (Z) [m/s²]", Box::new(|vs| vs.accelerometer1.map(|a| a.2))),
            ],
            (Some(-10.0), Some(10.0)),
        );

        let magnetometer_plot_cache = PlotCache::new(
            "Magnetometer",
            vec![
                ("Mag (X) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.0))),
                ("Mag (Y) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.1))),
                ("Mag (Z) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.2))),
            ],
            (None, None),
        );

        let barometer_plot_cache = PlotCache::new(
            "Barometer",
            vec![
                ("Pressure [mbar]", Box::new(|vs| vs.pressure)),
                //("Altitude (ASL) [m]", Box::new(|vs| vs.altitude_baro)),
            ],
            (Some(900.0), Some(1100.0)),
        );

        let temperature_plot_cache = PlotCache::new(
            "Temperatures",
            vec![
                ("Baro. Temp. [°C]", Box::new(|vs| vs.temperature_baro)),
                ("Core Temp. [°C]", Box::new(|vs| vs.temperature_core)),
            ],
            (Some(25.0), Some(35.0)),
        );

        let power_plot_cache = PlotCache::new(
            "Power",
            vec![
                ("Battery Voltage [V]", Box::new(|vs| vs.battery_voltage)),
                ("Arm Voltage [V]", Box::new(|vs| vs.arm_voltage)),
                ("Current [A]", Box::new(|vs| vs.current)),
                ("Core Voltage [V]", Box::new(|vs| vs.cpu_voltage)),
            ],
            (Some(0.0), Some(9.0)),
        );

        let runtime_plot_cache = PlotCache::new(
            "Runtime",
            vec![
                ("CPU Util. [%]", Box::new(|vs| vs.cpu_utilization.map(|u| u as f32))),
                ("Heap Util. [%]", Box::new(|vs| vs.heap_utilization.map(|u| u as f32))),
            ],
            (Some(0.0), Some(100.0)),
        );

        let signal_plot_cache = PlotCache::new(
            "Signal",
            vec![
                ("GCS RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))),
                ("GCS Signal RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))),
                ("GCS SNR [dB]", Box::new(|vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))),
                ("Vehicle RSSI [dBm]", Box::new(|vs| vs.vehicle_lora_rssi.map(|x| x as f32 / -2.0))),
            ],
            (Some(-100.0), Some(10.0)),
        );

        Self {
            data_source,
            vehicle_states: Vec::new(),
            log_messages: Vec::new(),
            auto_reset: false,
            logo: egui_extras::RetainedImage::from_image_bytes("logo.png", include_bytes!("logo.png")).unwrap(),
            xlen: 10.0,
            axes: LinkedAxisGroup::new(true, false),
            cursor: LinkedCursorsGroup::new(true, false),
            orientation_plot_cache,
            vertical_speed_plot_cache,
            altitude_plot_cache,
            gyroscope_plot_cache,
            accelerometer_plot_cache,
            magnetometer_plot_cache,
            barometer_plot_cache,
            temperature_plot_cache,
            power_plot_cache,
            runtime_plot_cache,
            signal_plot_cache,
        }
    }

    fn reset(&mut self) {
        info!("Resetting.");
        self.xlen = 10.0;
        self.vehicle_states.truncate(0);
        //self.log_messages.truncate(0); // TODO
        self.data_source.reset();
        self.orientation_plot_cache.reset();
        self.vertical_speed_plot_cache.reset();
        self.altitude_plot_cache.reset();
        self.gyroscope_plot_cache.reset();
        self.accelerometer_plot_cache.reset();
        self.magnetometer_plot_cache.reset();
        self.barometer_plot_cache.reset();
        self.temperature_plot_cache.reset();
        self.power_plot_cache.reset();
        self.runtime_plot_cache.reset();
        self.signal_plot_cache.reset();
    }

    fn process_telemetry(&mut self, msg: DownlinkMessage) {
        match msg {
            DownlinkMessage::Log(t, l, ll, m) => {
                self.log_messages.push((t, l, ll, m));
                return;
            }
            DownlinkMessage::TelemetryGCS(_) => {}
            _ => {
                let time = msg.time();
                let last_time = self.vehicle_states.last().map(|vs| vs.time).unwrap_or(0);

                // Clear history if this seems to be a new run
                if time + 1000 < last_time && self.auto_reset {
                    self.reset();
                }

                self.vehicle_states.push(VehicleState {
                    time,
                    ..Default::default()
                });
            }
        }

        if let Some(vs) = self.vehicle_states.last_mut() {
            vs.incorporate_telemetry(&msg);
            // TODO
            self.orientation_plot_cache.push(&vs);
            self.vertical_speed_plot_cache.push(&vs);
            self.altitude_plot_cache.push(&vs);
            self.gyroscope_plot_cache.push(&vs);
            self.accelerometer_plot_cache.push(&vs);
            self.magnetometer_plot_cache.push(&vs);
            self.barometer_plot_cache.push(&vs);
            self.temperature_plot_cache.push(&vs);
            self.power_plot_cache.push(&vs);
            self.runtime_plot_cache.push(&vs);
            self.signal_plot_cache.push(&vs);
        }
    }

    fn last_value<T>(&self, callback: Box<dyn Fn(&VehicleState) -> Option<T>>) -> Option<T> {
        self.vehicle_states.iter().rev().find_map(|vs| callback(vs))
    }

    fn text_indicator(&self, ui: &mut Ui, label: &str, value: Option<String>) {
        ui.horizontal(|ui| {
            ui.set_width(ui.available_width());
            ui.label(label);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(value.unwrap_or_default()).strong().monospace());
            });
        });
    }

    fn flight_mode_style(&self, fm: FlightMode) -> (&str, Color32, Color32) {
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

    fn flight_mode_button(&mut self, ui: &mut Ui, width: f32, fm: FlightMode) {
        let (label, fg, bg) = self.flight_mode_style(fm);

        let button = if self
            .last_value(Box::new(|vs| vs.mode))
            .map(|m| m == fm)
            .unwrap_or(false)
        {
            let label = RichText::new(label).monospace().color(fg);
            Button::new(label).fill(bg)
        } else {
            let label = RichText::new(label).monospace().color(Color32::LIGHT_GRAY);
            Button::new(label)
                .fill(Color32::TRANSPARENT)
                .stroke(Stroke::new(2.0, bg))
        };

        if ui.add_sized([width, 30.0], button).clicked() {
            self.data_source
                .send(UplinkMessage::SetFlightModeAuth(fm, self.data_source.next_mac()))
                .unwrap();
        }
    }

    fn utility_button(&mut self, ui: &mut Ui, width: f32, label: &str, msg: UplinkMessage) {
        let color = Color32::from_rgb(0xff, 0x7b, 0x00);
        let label = RichText::new(label).monospace().color(Color32::LIGHT_GRAY);
        let button = Button::new(label)
            .fill(Color32::TRANSPARENT)
            .stroke(Stroke::new(1.5, color));
        if ui.add_sized([width, 18.0], button).clicked() {
            self.data_source.send(msg).unwrap();
        }
    }
}

impl eframe::App for Sam {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        for msg in self.data_source.next_messages().into_iter() {
            self.process_telemetry(msg);
        }

        {
            let input = ctx.input();
            let fm = if input.modifiers.command_only() && input.key_down(Key::Num0) {
                Some(FlightMode::Idle)
            } else if input.modifiers.command_only() && input.key_down(Key::A) {
                Some(FlightMode::Armed)
            } else if input.modifiers.command_only() && input.key_down(Key::I) {
                Some(FlightMode::Flight)
            } else if input.modifiers.command_only() && input.key_down(Key::D) {
                Some(FlightMode::RecoveryDrogue)
            } else if input.modifiers.command_only() && input.key_down(Key::M) {
                Some(FlightMode::RecoveryMain)
            } else if input.modifiers.command_only() && input.key_down(Key::L) {
                Some(FlightMode::Landed)
            } else {
                None
            };
            if let Some(fm) = fm {
                self.data_source
                    .send(UplinkMessage::SetFlightModeAuth(fm, self.data_source.next_mac()))
                    .unwrap();
            }

            if input.key_released(Key::ArrowDown) {
                self.xlen /= ZOOM_FACTOR;
            }

            if input.key_released(Key::ArrowUp) {
                self.xlen *= ZOOM_FACTOR;
            }
        }

        // Redefine text_styles
        let mut style = (*ctx.style()).clone();
        style.text_styles = [
            (Heading, FontId::new(16.0, Proportional)),
            (Name("Heading2".into()), FontId::new(14.0, Proportional)),
            (Name("Context".into()), FontId::new(14.0, Proportional)),
            (Body, FontId::new(12.0, Proportional)),
            (Monospace, FontId::new(12.0, Proportional)),
            (Button, FontId::new(12.0, Proportional)),
            (Small, FontId::new(10.0, Proportional)),
        ]
        .into();

        // Mutate global style with above changes
        ctx.set_style(style);

        let alt_ground = self.last_value(Box::new(|vs| vs.altitude_ground)).unwrap_or(0.0);

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());

            let available_height = ui.available_height() - 60.0;
            let top_bar_height = 75.0;
            let bottom_bar_height = 15.0;
            let log_height = available_height / 7.0 - 10.0;
            let plot_height = available_height - top_bar_height - log_height - bottom_bar_height;

            ui.horizontal(|ui| {
                ui.set_height(top_bar_height);

                let max_size = ui.available_size();
                self.logo.show_max_size(ui, max_size);

                let buttons_w = f32::min(1000.0, f32::max(500.0, ui.available_width() * 0.40));
                let gauge_w = ui.available_width() - buttons_w - 75.0;

                ui.horizontal_centered(|ui| {
                    ui.set_width(gauge_w);
                    let w = gauge_w / 3.0;

                    ui.vertical(|ui| {
                        ui.set_width(w * 0.75);
                        self.text_indicator(
                            ui,
                            "Time [s]",
                            self.vehicle_states
                                .last()
                                .map(|vs| format!("{:10.3}", (vs.time as f32) / 1000.0)),
                        );
                        self.text_indicator(
                            ui,
                            "Mode",
                            self.last_value(Box::new(|vs| vs.mode)).map(|s| format!("{:?}", s)),
                        );
                        self.text_indicator(
                            ui,
                            "Bat. Voltage [V]",
                            self.last_value(Box::new(|vs| vs.battery_voltage))
                                .map(|v| format!("{:.2}", v)),
                        );
                        self.text_indicator(
                            ui,
                            "Vertical Speed [m/s]",
                            self.last_value(Box::new(|vs| vs.vertical_speed))
                                .map(|v| format!("{:.2}", v)),
                        );
                    });

                    ui.vertical(|ui| {
                        ui.set_width(w * 1.15);
                        self.text_indicator(
                            ui,
                            "Alt. (AGL/ASL) [m]",
                            self.last_value(Box::new(|vs| vs.altitude))
                                .map(|a| format!("{:.1} ({:.1})", a - alt_ground, a)),
                        );
                        self.text_indicator(
                            ui,
                            "Max Alt. (AGL/ASL) [m]",
                            self.last_value(Box::new(|vs| vs.altitude_max))
                                .map(|a| format!("{:.1} ({:.1})", a - alt_ground, a)),
                        );
                        self.text_indicator(
                            ui,
                            "Alt. (Baro, ASL) [m]",
                            self.last_value(Box::new(|vs| vs.altitude_baro))
                                .map(|a| format!("{:.1}", a)),
                        );
                        self.text_indicator(
                            ui,
                            "Alt. (GPS, ASL) [m]",
                            self.last_value(Box::new(|vs| vs.altitude_gps))
                                .map(|a| format!("{:.1}", a)),
                        );
                    });

                    let last_gps_msg = self
                        .vehicle_states
                        .iter()
                        .rev()
                        .find_map(|vs| vs.gps_fix.is_some().then(|| vs));

                    ui.vertical(|ui| {
                        ui.set_width(w * 1.1);
                        self.text_indicator(
                            ui,
                            "GPS Status (#Sats)",
                            last_gps_msg
                                .map(|vs| format!("{:?} ({})", vs.gps_fix.unwrap(), vs.num_satellites.unwrap_or(0))),
                        );
                        self.text_indicator(
                            ui,
                            "HDOP",
                            last_gps_msg.map(|vs| format!("{:.2}", vs.hdop.unwrap_or(9999) as f32 / 100.0)),
                        );
                        self.text_indicator(
                            ui,
                            "Latitude",
                            last_gps_msg.and_then(|vs| vs.latitude).map(|l| format!("{:.6}", l)),
                        );
                        self.text_indicator(
                            ui,
                            "Longitude",
                            last_gps_msg.and_then(|vs| vs.longitude).map(|l| format!("{:.6}", l)),
                        );
                    });
                });

                ui.vertical(|ui| {
                    ui.set_width(buttons_w);
                    ui.set_height(top_bar_height);

                    ui.horizontal(|ui| {
                        ui.set_height(top_bar_height * 0.1);
                        ui.style_mut().visuals.extreme_bg_color = Color32::from_rgb(0x10, 0x10, 0x10);
                        ui.style_mut().visuals.override_text_color = Some(Color32::LIGHT_GRAY);

                        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                            ui.set_width(buttons_w * 0.25 + 12.0);

                            let battery_voltage: f32 =
                                self.last_value(Box::new(|vs| vs.battery_voltage)).unwrap_or_default();

                            ui.style_mut().visuals.selection.bg_fill = if battery_voltage >= 7.4 {
                                Color32::DARK_GREEN
                            } else if battery_voltage >= 6.5 {
                                Color32::from_rgb(0xe6, 0x6f, 0x00)
                            } else {
                                Color32::from_rgb(0xff, 0x3b, 0x00)
                            };

                            let battery_percentage = (battery_voltage - 6.0) / (8.4 - 6.0);
                            let progress_bar_battery =
                                ProgressBar::new(battery_percentage).text(format!("Battery: {:.2}V", battery_voltage));

                            ui.add(progress_bar_battery);
                        });

                        ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                            ui.set_width(buttons_w * 0.45 + 12.0);

                            let flash_pointer: f32 = self
                                .last_value(Box::new(|vs| vs.flash_pointer))
                                .map(|fp| (fp as f32) / 1024.0 / 1024.)
                                .unwrap_or_default();
                            let flash_size = (FLASH_SIZE as f32) / 1024.0 / 1024.0;

                            ui.style_mut().visuals.selection.bg_fill = if flash_pointer > flash_size * 0.9 {
                                Color32::DARK_RED
                            } else if flash_pointer > flash_size * 0.5 {
                                Color32::from_rgb(0xe6, 0x6f, 0x00)
                            } else {
                                Color32::from_rgb(0x24, 0x63, 0x99)
                            };

                            let progress_bar_flash = ProgressBar::new(flash_pointer / flash_size)
                                .text(format!("Flash: {:.2}MiB / {:.2}MiB", flash_pointer, flash_size));

                            ui.add(progress_bar_flash);
                        });

                        // TODO: confirm these?
                        ui.horizontal_centered(|ui| {
                            let w = (buttons_w * 0.3) / 2.0;
                            self.utility_button(ui, w, "Reboot", UplinkMessage::RebootAuth(self.data_source.next_mac()));
                            self.utility_button(ui, w, "Erase Flash", UplinkMessage::EraseFlashAuth(self.data_source.next_mac()));
                        });
                    });

                    ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                        ui.set_height(top_bar_height * 0.7);
                        let w = buttons_w / 7.0;
                        self.flight_mode_button(ui, w, FlightMode::Idle);
                        self.flight_mode_button(ui, w, FlightMode::HardwareArmed);
                        self.flight_mode_button(ui, w, FlightMode::Armed);
                        self.flight_mode_button(ui, w, FlightMode::Flight);
                        self.flight_mode_button(ui, w, FlightMode::RecoveryDrogue);
                        self.flight_mode_button(ui, w, FlightMode::RecoveryMain);
                        self.flight_mode_button(ui, w, FlightMode::Landed);
                    });
                });
            });

            ui.separator();

            egui::Grid::new("plot_grid")
                .min_col_width(ui.available_width() / 4.0 - 6.0)
                .max_col_width(ui.available_width() / 4.0 - 6.0)
                .min_row_height(plot_height / 3.0)
                .show(ui, |ui| {
                    ui.plot_telemetry(&self.orientation_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.vertical_speed_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.altitude_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.map(&self.vehicle_states);

                    ui.end_row();

                    ui.plot_telemetry(&self.gyroscope_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.accelerometer_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.magnetometer_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.barometer_plot_cache, &self.axes, &self.cursor, self.xlen);

                    ui.end_row();

                    ui.plot_telemetry(&self.temperature_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.power_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.runtime_plot_cache, &self.axes, &self.cursor, self.xlen);
                    ui.plot_telemetry(&self.signal_plot_cache, &self.axes, &self.cursor, self.xlen);
                });

            ui.separator();

            ui.log_scroller(&self.log_messages, log_height);

            ui.separator();

            ui.horizontal_centered(|ui| {
                let w = ui.available_width();
                ui.set_width(w);
                ui.set_height(bottom_bar_height);

                ui.horizontal_centered(|ui| {
                    ui.set_width(w / 2.0);

                    let (status_color, status_text) = self.data_source.status();
                    ui.label(RichText::new(status_text).color(status_color));

                    ui.label(self.data_source.info_text());
                });

                ui.allocate_ui_with_layout(Vec2::new(w / 2.0 - 6.0, bottom_bar_height),
                    Layout::right_to_left(eframe::emath::Align::Center),
                    |ui| {
                        ui.checkbox(&mut self.auto_reset, "Auto-Reset");

                        if ui.button("Reset").clicked() {
                            self.reset();
                        }

                        if ui.add_sized([20.0, 20.0], egui::Button::new("+")).clicked() {
                            self.xlen /= ZOOM_FACTOR;
                        }

                        if ui.add_sized([20.0, 20.0], egui::Button::new("-")).clicked() {
                            self.xlen *= ZOOM_FACTOR;
                        }
                    },
                );
            });
        });

        ctx.request_repaint_after(std::time::Duration::from_millis(16));
    }
}

pub fn main(log_file: Option<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let data_source: Box<dyn DataSource> = match log_file {
        Some(path) => Box::new(LogFileDataSource::new(path)?),
        None => Box::new(SerialDataSource::new()),
    };

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(1000.0, 700.0)),
            ..Default::default()
        },
        Box::new(|_cc| Box::new(Sam::init(data_source))),
    )?;

    Ok(())
}
