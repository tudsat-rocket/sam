use std::path::PathBuf;

use eframe::egui;
use eframe::emath::Align;
use egui::widgets::plot::{LinkedAxisGroup, LinkedCursorsGroup};
use egui::FontFamily::Proportional;
use egui::FontId;
use egui::TextStyle::*;
use egui::{Key, Layout, RichText, Vec2, Rounding};

use log::*;

use euroc_fc_firmware::telemetry::*;

mod top_bar;
mod plot;
mod map;
mod log_scroller;

use crate::state::*;
use crate::data_source::*;

use crate::gui::top_bar::*;
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
                ("Vertical Accel (Filt.) [m/s²]", Box::new(|vs| vs.vertical_accel_filtered)),
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
            (None, Some(300.0)) // TODO
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
            (Heading, FontId::new(12.0, Proportional)),
            (Name("Heading2".into()), FontId::new(10.0, Proportional)),
            (Name("Context".into()), FontId::new(10.0, Proportional)),
            (Body, FontId::new(9.0, Proportional)),
            (Monospace, FontId::new(9.0, Proportional)),
            (Button, FontId::new(9.0, Proportional)),
            (Small, FontId::new(7.0, Proportional)),
        ].into();
        style.spacing.item_spacing = Vec2::new(6.0, 4.0);
        style.spacing.button_padding = Vec2::new(4.0, 1.0);
        style.spacing.interact_size = Vec2::new(14.0, 14.0);
        style.spacing.icon_width = 10.0;
        style.visuals.menu_rounding = Rounding::same(2.0);
        ctx.set_style(style.clone());

        let alt_ground = self.last_value(Box::new(|vs| vs.altitude_ground)).unwrap_or(0.0);

        egui::TopBottomPanel::bottom("bottombar").min_height(18.0).show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() / 2.0);
                    let (status_color, status_text) = self.data_source.status();
                    ui.label(RichText::new(status_text).color(status_color));
                    ui.label(self.data_source.info_text());
                });

                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    ui.checkbox(&mut self.auto_reset, "Auto-Reset");

                    if ui.button("Reset").clicked() {
                        self.reset();
                    }

                    //if ui.add_sized([15.0, 12.0], egui::Button::new("+")).clicked() {
                    if ui.button("+").clicked() {
                        self.xlen /= ZOOM_FACTOR;
                    }

                    if ui.button("-").clicked() {
                        self.xlen *= ZOOM_FACTOR;
                    }
                });
            });
        });

        egui::TopBottomPanel::top("topbar").min_height(50.0).max_height(50.0).show(ctx, |ui| {
            ui.horizontal(|ui| {
                self.logo.show_max_size(ui, Vec2::new(ui.available_width(), 70.0)); // TODO

                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() * 0.55);

                    let time = self.vehicle_states.last().map(|vs| format!("{:10.3}", (vs.time as f32) / 1000.0));
                    let mode = self.last_value(Box::new(|vs| vs.mode)).map(|s| format!("{:?}", s));
                    let battery_voltage = self.last_value(Box::new(|vs| vs.battery_voltage)).map(|v| format!("{:.2}", v));
                    let vertical_speed = self.last_value(Box::new(|vs| vs.vertical_speed)).map(|v| format!("{:.2}", v));

                    let alt = self.last_value(Box::new(|vs| vs.altitude)).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_max = self.last_value(Box::new(|vs| vs.altitude_max)).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_baro = self.last_value(Box::new(|vs| vs.altitude_baro)).map(|a| format!("{:.1}", a));
                    let alt_gps = self.last_value(Box::new(|vs| vs.altitude_gps)).map(|a| format!("{:.1}", a));

                    let last_gps_msg = self
                        .vehicle_states
                        .iter()
                        .rev()
                        .find_map(|vs| vs.gps_fix.is_some().then(|| vs));
                    let gps_status = last_gps_msg.map(|vs| format!("{:?} ({})", vs.gps_fix.unwrap(), vs.num_satellites.unwrap_or(0)));
                    let hdop = last_gps_msg.map(|vs| format!("{:.2}", vs.hdop.unwrap_or(9999) as f32 / 100.0));
                    let latitude = last_gps_msg.and_then(|vs| vs.latitude).map(|l| format!("{:.6}", l));
                    let longitude = last_gps_msg.and_then(|vs| vs.longitude).map(|l| format!("{:.6}", l));

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 3.0);
                        ui.telemetry_value("Time [s]", time);
                        ui.telemetry_value("Mode", mode);
                        ui.telemetry_value("Bat. Voltage [V]", battery_voltage);
                        ui.telemetry_value("Vertical Speed [m/s]", vertical_speed);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 2.0);
                        ui.telemetry_value("Alt. (AGL/ASL) [m]", alt);
                        ui.telemetry_value("Max Alt. (AGL/ASL) [m]", alt_max);
                        ui.telemetry_value("Alt. (Baro, ASL) [m]", alt_baro);
                        ui.telemetry_value("Alt. (GPS, ASL) [m]", alt_gps);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width());
                        ui.telemetry_value("GPS Status (#Sats)", gps_status);
                        ui.telemetry_value("HDOP", hdop);
                        ui.telemetry_value("Latitude", latitude);
                        ui.telemetry_value("Longitude", longitude);
                    });
                });

                ui.vertical(|ui| {
                    let size = Vec2::new(ui.available_width(), ui.available_height() * 0.4);
                    ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                        ui.command_button("Reboot", UplinkMessage::RebootAuth(self.data_source.next_mac()), &mut self.data_source);
                        ui.command_button("Erase Flash", UplinkMessage::EraseFlashAuth(self.data_source.next_mac()), &mut self.data_source);

                        let flash_pointer: f32 = self
                            .last_value(Box::new(|vs| vs.flash_pointer))
                            .map(|fp| (fp as f32) / 1024.0 / 1024.0)
                            .unwrap_or_default();
                        let flash_size = (FLASH_SIZE as f32) / 1024.0 / 1024.0;
                        let f = flash_pointer / flash_size;
                        let text = format!("Flash: {:.2}MiB / {:.2}MiB", flash_pointer, flash_size);
                        ui.flash_bar(ui.available_width() * 0.6, f, text);

                        let voltage = self.last_value(Box::new(|vs| vs.battery_voltage)).unwrap_or_default();
                        let f = (voltage - 6.0) / (8.4 - 6.0);
                        let text = format!("Battery: {:.2}V", voltage);
                        ui.battery_bar(ui.available_width(), f, text);
                    });

                    ui.horizontal_centered(|ui| {
                        ui.set_height(ui.available_height());
                        let w = ui.available_width() / 7.0 - style.spacing.item_spacing.x * (6.0 / 7.0);
                        let current = self.last_value(Box::new(|vs| vs.mode));
                        ui.flight_mode_button(w, FlightMode::Idle, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::HardwareArmed, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::Armed, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::Flight, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::RecoveryDrogue, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::RecoveryMain, current, &mut self.data_source);
                        ui.flight_mode_button(w, FlightMode::Landed, current, &mut self.data_source);
                    });
                });
            });
        });

        egui::TopBottomPanel::bottom("logbar")
            .min_height(72.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.log_scroller(&self.log_messages);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let xspacing = style.spacing.item_spacing.x * (3.0 / 4.0);
            let yspacing = style.spacing.item_spacing.y / 1.5;
            egui::Grid::new("plot_grid")
                .min_col_width(ui.available_width() / 4.0 - xspacing)
                .max_col_width(ui.available_width() / 4.0 - xspacing)
                .min_row_height(ui.available_height() / 3.0 - yspacing)
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
