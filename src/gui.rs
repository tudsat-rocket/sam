use std::path::PathBuf;

use eframe::egui::{self, Ui};
use egui::widgets::plot::{Corner, Legend, Line, VLine, LineStyle};
use egui::widgets::ProgressBar;
use egui::FontFamily::Proportional;
use egui::FontId;
use egui::TextStyle::*;
use egui::{Color32, Key, Layout, RichText, Stroke, Vec2};
use log::*;

use euroc_fc_firmware::telemetry::*;

use crate::state::*;
use crate::data_source::*;
use crate::telemetry_ext::*;

#[rustfmt::skip]
const ZOOM_LEVELS: [u32; 14] = [100, 200, 500, 1000, 2000, 5000, 10000, 20000, 60000, 120000, 300000, 600000, 1800000, 3600000];
const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;

struct Sam {
    data_source: Box<dyn DataSource>,

    vehicle_states: Vec<VehicleState>,
    log_messages: Vec<(u32, String, LogLevel, String)>,

    zoom_level: usize,
    auto_reset: bool,

    logo: egui_extras::RetainedImage,
}

impl Sam {
    pub fn init(data_source: Box<dyn DataSource>) -> Self {
        Self {
            data_source,
            vehicle_states: Vec::new(),
            log_messages: Vec::new(),
            zoom_level: ZOOM_LEVELS.iter().position(|zl| *zl == 10000).unwrap(),
            auto_reset: false,
            logo: egui_extras::RetainedImage::from_image_bytes("logo.png", include_bytes!("logo.png")).unwrap(),
        }
    }

    fn reset(&mut self) {
        info!("Resetting.");
        self.vehicle_states.truncate(0);
        //self.log_messages.truncate(0);
        self.data_source.reset();
    }

    fn adjust_zoom_level(&mut self, delta: isize) {
        let new = (self.zoom_level as isize) + delta;
        let new = isize::max(0, isize::min(new, (ZOOM_LEVELS.len() as isize) - 1));
        self.zoom_level = new as usize;
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

        self.vehicle_states.last_mut().map(|vs| vs.incorporate_telemetry(&msg));
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

    fn plot(
        &self,
        ui: &mut Ui,
        heading: &str,
        callbacks: Vec<(&str, Box<dyn Fn(&VehicleState) -> Option<f32>>)>,
        ylimits: (Option<f32>, Option<f32>),
    ) {
        let start_i = self.visible_state_indices();

        let lines = callbacks.iter().map(|(name, callback)| {
            let points: Vec<[f64; 2]> = self.vehicle_states[start_i..]
                .iter()
                .map(|vs| (vs.time as f64 / 1000.0, callback(vs)))
                .filter(|(_t, y)| y.is_some())
                .map(|(t, y)| [t, y.unwrap() as f64])
                .collect();
            Line::new(points).name(name).width(1.2)
        });

        let mut modes: Vec<(f64, FlightMode)> = self.vehicle_states[start_i..].iter()
            .map(|vs| (vs.time as f64 / 1000.0, vs.mode))
            .filter(|(_x, mode)| mode.is_some())
            .map(|(x, mode)| (x, mode.unwrap()))
            .collect();
        modes.dedup_by_key(|(_x, mode)| mode.clone());
        let mode_lines = modes.iter()
            .map(|(x, mode)| VLine::new(*x).color(mode.color()).style(LineStyle::Dashed{length: 4.0}));

        let legend = Legend::default().background_alpha(0.5).position(Corner::LeftTop);

        ui.vertical_centered(|ui| {
            ui.heading(heading);
            let mut plot = egui::widgets::plot::Plot::new(heading)
                .allow_drag(false)
                .allow_scroll(false)
                .allow_zoom(false)
                .allow_boxed_zoom(false)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .auto_bounds_y()
                .legend(legend.clone());

            if let Some(min) = ylimits.0 {
                plot = plot.include_y(min);
            }

            if let Some(max) = ylimits.1 {
                plot = plot.include_y(max);
            }

            if let Some(last) = self.vehicle_states.last() {
                plot = plot
                    .include_x((last.time as f32) / 1000.0)
                    .include_x(((last.time as f32) - (ZOOM_LEVELS[self.zoom_level] as f32)) / 1000.0);
            }

            plot.show(ui, |plot_ui| {
                for l in lines {
                    plot_ui.line(l);
                }

                for vl in mode_lines {
                    plot_ui.vline(vl);
                }
            });
        });
    }

    fn plot_position(&self, ui: &mut Ui, heading: &str) {
        let points: Vec<[f64; 3]> = self
            .vehicle_states
            .iter()
            .map(|vs| (vs.time as f64 / 1000.0, vs.latitude, vs.longitude))
            .filter(|(_t, lat, lng)| lat.is_some() && lng.is_some())
            .map(|(t, lat, lng)| [t, lng.unwrap() as f64, lat.unwrap() as f64])
            .collect();

        let last = points.last().cloned();

        let points_10m: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 600.0)
            .collect();
        let points_1m: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 60.0)
            .collect();
        let points_10s: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 10.0)
            .collect();

        let line = Line::new(points.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_10m = Line::new(points_10m.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_1m = Line::new(points_1m.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_10s = Line::new(points_10s.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);

        ui.vertical_centered(|ui| {
            ui.heading(heading);
            let mut plot = egui::widgets::plot::Plot::new(heading)
                .allow_scroll(false)
                .data_aspect(1.0)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15));

            if let Some(coords) = last {
                plot = plot.include_x(coords[1] - 0.001);
                plot = plot.include_x(coords[1] + 0.001);
                plot = plot.include_y(coords[2] - 0.001);
                plot = plot.include_y(coords[2] + 0.001);
            }

            plot.show(ui, |plot_ui| {
                plot_ui.line(line.color(Color32::DARK_RED));
                plot_ui.line(line_10m.color(Color32::RED));
                plot_ui.line(line_1m.color(Color32::YELLOW));
                plot_ui.line(line_10s.color(Color32::GREEN));
            });
        });
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
            egui::Button::new(label).fill(bg)
        } else {
            let label = RichText::new(label).monospace().color(Color32::LIGHT_GRAY);
            egui::Button::new(label)
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
        let button = egui::Button::new(label)
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
                self.adjust_zoom_level(1);
            }
            if input.key_released(Key::ArrowUp) {
                self.adjust_zoom_level(-1);
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
            //let top_bar_height = 0.0;
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
                    self.plot(
                        ui,
                        "Orientation",
                        vec![
                            (
                                "Pitch (X) [°]",
                                Box::new(|vs| vs.euler_angles.map(|a| a.0 * RAD_TO_DEG)),
                            ),
                            (
                                "Pitch (Y) [°]",
                                Box::new(|vs| vs.euler_angles.map(|a| a.1 * RAD_TO_DEG)),
                            ),
                            ("Roll (Z) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.2 * RAD_TO_DEG))),
                        ],
                        (Some(-180.0), Some(180.0)),
                    );

                    self.plot(
                        ui,
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

                    self.plot(
                        ui,
                        "Altitude (ASL)",
                        vec![
                            ("Altitude [m]", Box::new(|vs| vs.altitude)),
                            ("Altitude (Baro) [m]", Box::new(|vs| vs.altitude_baro)),
                            ("Altitude (GPS) [m]", Box::new(|vs| vs.altitude_gps)),
                            ("Altitude (Max) [m]", Box::new(|vs| vs.altitude_max)),
                            ("Altitude (Ground) [m]", Box::new(|vs| vs.altitude_ground)),
                        ],
                        (
                            Some(alt_ground),
                            Some(f32::max(
                                alt_ground + 10.0,
                                self.last_value(Box::new(|vs| vs.altitude_max)).unwrap_or(-100.0),
                            )),
                        ),
                    );

                    self.plot_position(ui, "Position");

                    ui.end_row();

                    self.plot(
                        ui,
                        "Gyroscope",
                        vec![
                            ("Gyro (X) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.0))),
                            ("Gyro (Y) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.1))),
                            ("Gyro (Z) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.2))),
                        ],
                        (Some(-10.0), Some(10.0)),
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
                        (Some(-10.0), Some(10.0)),
                    );

                    self.plot(
                        ui,
                        "Magnetometer",
                        vec![
                            ("Mag (X) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.0))),
                            ("Mag (Y) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.1))),
                            ("Mag (Z) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.2))),
                        ],
                        (None, None),
                    );

                    self.plot(
                        ui,
                        "Barometer",
                        vec![
                            ("Pressure [mbar]", Box::new(|vs| vs.pressure)),
                            //("Altitude (ASL) [m]", Box::new(|vs| vs.altitude_baro)),
                        ],
                        (Some(900.0), Some(1100.0)),
                    );

                    ui.end_row();

                    self.plot(
                        ui,
                        "Temperatures",
                        vec![
                            ("Baro. Temp. [°C]", Box::new(|vs| vs.temperature_baro)),
                            ("Core Temp. [°C]", Box::new(|vs| vs.temperature_core)),
                        ],
                        (Some(25.0), Some(35.0)),
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
                        (Some(0.0), Some(9.0))
                    );

                    self.plot(
                        ui,
                        "Runtime",
                        vec![
                            ("CPU Util. [%]", Box::new(|vs| vs.cpu_utilization.map(|u| u as f32))),
                            ("Heap Util. [%]", Box::new(|vs| vs.heap_utilization.map(|u| u as f32))),
                        ],
                        (Some(0.0), Some(100.0)),
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
                        (Some(-100.0), Some(10.0))
                    );
                });

            ui.separator();

            egui::ScrollArea::vertical()
                .max_height(log_height)
                .max_width(ui.available_width())
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    ui.set_min_height(log_height);
                    //ui.set_width(ui.available_width());

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
                                Vec2::new(ui.available_width(), 10.0),
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
                            self.adjust_zoom_level(-1);
                        }

                        if ui.add_sized([50.0, 20.0], egui::Button::new("Pause")).clicked() {}

                        if ui.add_sized([20.0, 20.0], egui::Button::new("-")).clicked() {
                            self.adjust_zoom_level(1);
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
    );

    Ok(())
}
