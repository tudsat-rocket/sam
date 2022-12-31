//! Main GUI code

use std::path::PathBuf;
use std::cell::RefCell;
use std::rc::Rc;

use eframe::egui;
use eframe::emath::Align;
use egui::widgets::plot::{LinkedAxisGroup, LinkedCursorsGroup};
use egui::FontFamily::Proportional;
use egui::TextStyle::*;
use egui::{FontId, Key, Layout, RichText, Vec2};
use egui_extras::RetainedImage;

use log::*;

use euroc_fc_firmware::telemetry::*;

mod top_bar;
mod plot;
mod map;
mod log_scroller;
mod maxi_grid;

use crate::state::*;
use crate::data_source::*;
use crate::file::*;

use crate::gui::top_bar::*;
use crate::gui::plot::*;
use crate::gui::map::*;
use crate::gui::log_scroller::*;
use crate::gui::maxi_grid::*;

const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;
const ZOOM_FACTOR: f64 = 2.0;

// Log files included with the application. These should probably be fetched
// if necessary to reduce application size.
const ARCHIVE: [(&str, &[u8], &[u8]); 2] = [
    (
        "Zülpich #1",
        include_bytes!("../archive/zuelpich_launch1_telem_filtered.json"),
        include_bytes!("../archive/zuelpich_launch1_flash_filtered.json")
    ),
    (
        "Zülpich #2",
        include_bytes!("../archive/zuelpich_launch2_telem_filtered.json"),
        include_bytes!("../archive/zuelpich_launch2_flash_filtered.json")
    ),
];

// The main state object of our GUI application
pub struct Sam {
    data_source: Box<dyn DataSource>,

    vehicle_states: Rc<RefCell<Vec<VehicleState>>>,
    log_messages: Vec<(u32, String, LogLevel, String)>,

    auto_reset: bool,

    logo: RetainedImage,
    logo_inverted: RetainedImage,

    archive_panel_open: bool,
    xlen: f64,
    maxi_grid_state: MaxiGridState,

    orientation_plot: PlotState,
    vertical_speed_plot: PlotState,
    altitude_plot: PlotState,
    gyroscope_plot: PlotState,
    accelerometer_plot: PlotState,
    magnetometer_plot: PlotState,
    barometer_plot: PlotState,
    temperature_plot: PlotState,
    power_plot: PlotState,
    runtime_plot: PlotState,
    signal_plot: PlotState,

    map: MapState,
}

impl Sam {
    /// Initialize the application, including the state objects for widgets
    /// such as plots and maps.
    pub fn init(data_source: Box<dyn DataSource>) -> Self {
        let axes = LinkedAxisGroup::new(true, false);
        let cursors = LinkedCursorsGroup::new(true, false);

        let orientation_plot = PlotState::new(
            "Orientation",
            vec![
                ("Pitch (X) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.0 * RAD_TO_DEG))),
                ("Pitch (Y) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.1 * RAD_TO_DEG))),
                ("Roll (Z) [°]", Box::new(|vs| vs.euler_angles.map(|a| a.2 * RAD_TO_DEG))),
            ],
            (Some(-180.0), Some(180.0)),
            axes.clone(),
            cursors.clone(),
        );

        let vertical_speed_plot = PlotState::new(
            "Vert. Speed & Accel.",
            vec![
                ("Vario [m/s]", Box::new(|vs| vs.vertical_speed)),
                ("Vertical Accel [m/s²]", Box::new(|vs| vs.vertical_accel)),
                ("Vertical Accel (Filt.) [m/s²]", Box::new(|vs| vs.vertical_accel_filtered)),
            ],
            (Some(-1.0), Some(1.0)),
            axes.clone(),
            cursors.clone(),
        );

        let altitude_plot = PlotState::new(
            "Altitude (ASL)",
            vec![
                ("Altitude [m]", Box::new(|vs| vs.altitude)),
                ("Altitude (Baro) [m]", Box::new(|vs| vs.altitude_baro)),
                ("Altitude (GPS) [m]", Box::new(|vs| vs.altitude_gps)),
                ("Altitude (Max) [m]", Box::new(|vs| vs.altitude_max)),
                ("Altitude (Ground) [m]", Box::new(|vs| vs.altitude_ground)),
            ],
            (None, Some(300.0)), // TODO
            axes.clone(),
            cursors.clone(),
        );

        let gyroscope_plot = PlotState::new(
            "Gyroscope",
            vec![
                ("Gyro (X) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.0))),
                ("Gyro (Y) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.1))),
                ("Gyro (Z) [°/s]", Box::new(|vs| vs.gyroscope.map(|a| a.2))),
            ],
            (Some(-10.0), Some(10.0)),
            axes.clone(),
            cursors.clone(),
        );

        let accelerometer_plot = PlotState::new(
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
            axes.clone(),
            cursors.clone(),
        );

        let magnetometer_plot = PlotState::new(
            "Magnetometer",
            vec![
                ("Mag (X) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.0))),
                ("Mag (Y) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.1))),
                ("Mag (Z) [µT]", Box::new(|vs| vs.magnetometer.map(|a| a.2))),
            ],
            (None, None),
            axes.clone(),
            cursors.clone(),
        );

        let barometer_plot = PlotState::new(
            "Barometer",
            vec![
                ("Pressure [mbar]", Box::new(|vs| vs.pressure)),
            ],
            (Some(900.0), Some(1100.0)),
            axes.clone(),
            cursors.clone(),
        );

        let temperature_plot = PlotState::new(
            "Temperatures",
            vec![
                ("Baro. Temp. [°C]", Box::new(|vs| vs.temperature_baro)),
                ("Core Temp. [°C]", Box::new(|vs| vs.temperature_core)),
            ],
            (Some(25.0), Some(35.0)),
            axes.clone(),
            cursors.clone(),
        );

        let power_plot = PlotState::new(
            "Power",
            vec![
                ("Battery Voltage [V]", Box::new(|vs| vs.battery_voltage)),
                ("Arm Voltage [V]", Box::new(|vs| vs.arm_voltage)),
                ("Current [A]", Box::new(|vs| vs.current)),
                ("Core Voltage [V]", Box::new(|vs| vs.cpu_voltage)),
            ],
            (Some(0.0), Some(9.0)),
            axes.clone(),
            cursors.clone(),
        );

        let runtime_plot = PlotState::new(
            "Runtime",
            vec![
                ("CPU Util. [%]", Box::new(|vs| vs.cpu_utilization.map(|u| u as f32))),
                ("Heap Util. [%]", Box::new(|vs| vs.heap_utilization.map(|u| u as f32))),
            ],
            (Some(0.0), Some(100.0)),
            axes.clone(),
            cursors.clone(),
        );

        let signal_plot = PlotState::new(
            "Signal",
            vec![
                ("GCS RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))),
                ("GCS Signal RSSI [dBm]", Box::new(|vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))),
                ("GCS SNR [dB]", Box::new(|vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))),
                ("Vehicle RSSI [dBm]", Box::new(|vs| vs.vehicle_lora_rssi.map(|x| x as f32 / -2.0))),
            ],
            (Some(-100.0), Some(10.0)),
            axes.clone(),
            cursors.clone(),
        );

        let bytes = include_bytes!("logo.png");
        let logo = RetainedImage::from_image_bytes("logo.png", bytes).unwrap();

        let bytes = include_bytes!("logo_inverted.png");
        let logo_inverted = RetainedImage::from_image_bytes("logo_inverted.png", bytes).unwrap();
        let vehicle_states = Rc::new(RefCell::new(Vec::new()));
        let map = MapState::new(vehicle_states.clone());

        Self {
            data_source,
            vehicle_states,
            log_messages: Vec::new(),
            auto_reset: false,
            logo,
            logo_inverted,
            archive_panel_open: cfg!(target_arch = "wasm32"),
            xlen: 10.0,
            maxi_grid_state: MaxiGridState::default(),
            orientation_plot,
            vertical_speed_plot,
            altitude_plot,
            gyroscope_plot,
            accelerometer_plot,
            magnetometer_plot,
            barometer_plot,
            temperature_plot,
            power_plot,
            runtime_plot,
            signal_plot,
            map,
        }
    }

    fn all_plots(&mut self, f: impl FnOnce(&mut PlotState) + Copy) {
        for plot in [
            &mut self.orientation_plot,
            &mut self.vertical_speed_plot,
            &mut self.altitude_plot,
            &mut self.gyroscope_plot,
            &mut self.accelerometer_plot,
            &mut self.magnetometer_plot,
            &mut self.barometer_plot,
            &mut self.temperature_plot,
            &mut self.power_plot,
            &mut self.runtime_plot,
            &mut self.signal_plot,
        ] {
            (f)(plot);
        }
    }

    /// Resets the GUI
    fn reset(&mut self) {
        info!("Resetting.");
        self.xlen = 10.0;
        self.vehicle_states.borrow_mut().truncate(0);
        self.log_messages.truncate(0);
        self.data_source.reset();
        self.all_plots(|plot| plot.reset());
        // TODO: close log file if opened?
    }

    /// Incorporates a new downlink message
    fn process_telemetry(&mut self, msg: DownlinkMessage) {
        match msg {
            DownlinkMessage::Log(t, l, ll, m) => {
                self.log_messages.push((t, l, ll, m));
                return;
            }
            DownlinkMessage::TelemetryGCS(_) => {}
            _ => {
                let time = msg.time();
                let last_time = self.vehicle_states.borrow().last().map(|vs| vs.time).unwrap_or(0);

                // Clear history if this seems to be a new run
                // TODO: refactor
                if time + 1000 < last_time && self.auto_reset {
                    self.reset();
                }

                self.vehicle_states.borrow_mut().push(VehicleState {
                    time,
                    ..Default::default()
                });
            }
        }

        let vs = self.vehicle_states.clone();
        if let Some(vs) = vs.borrow_mut().last_mut() {
            vs.incorporate_telemetry(&msg);
            self.all_plots(|plot| plot.push(&vs));
        };
    }

    /// Returns the "current" value for the given callback. This is the last
    /// known of the value at the current time.
    /// TODO: incorporate cursor position
    fn current<T>(&self, callback: impl Fn(&VehicleState) -> Option<T>) -> Option<T> {
        self.vehicle_states.borrow().iter().rev().find_map(|vs| callback(vs))
    }

    /// Opens a log file data source
    fn open_log_file(&mut self, ds: LogFileDataSource) {
        self.reset();
        self.data_source = Box::new(ds);
    }

    /// Closes the currently opened data source
    fn close_log_file(&mut self) {
        self.reset();
        self.data_source = Box::new(SerialDataSource::new());
    }
}

impl eframe::App for Sam {
    /// Main draw method of the application
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Process new messages
        for msg in self.data_source.next_messages().into_iter() {
            self.process_telemetry(msg);
        }

        // Check for keyboard inputs. TODO: clean up
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
        style.text_styles.insert(Heading, FontId::new(14.0, Proportional));
        ctx.set_style(style.clone());

        // Prevent unnecessarily large UI on non-high-DPI displays
        #[cfg(not(target_arch="wasm32"))]
        if ctx.pixels_per_point() > 1.0 && ctx.pixels_per_point() <= 1.5 {
            ctx.set_pixels_per_point(1.0);
        }

        // Top menu bar
        egui::TopBottomPanel::top("menubar").min_height(30.0).max_height(30.0).show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                egui::widgets::global_dark_light_mode_switch(ui);

                ui.separator();

                // Opening files manually is not available on web assembly
                #[cfg(not(target_arch = "wasm32"))]
                if ui.button("🗁  Open Log File").clicked() {
                    if let Some(data_source) = open_file() {
                        self.open_log_file(data_source);
                    }
                }

                // Toggle archive panel
                let text = if self.archive_panel_open {
                    "⛃ Close Archive"
                } else {
                    "⛃ Open Archive"
                };
                if ui.button(text).clicked() {
                    self.archive_panel_open = !self.archive_panel_open;
                }

                // Show a button to the right to close the current log
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    if self.data_source.is_log_file() {
                        if ui.button("❌").clicked() {
                            self.close_log_file();
                        }
                    }
                });
            });
        });

        // Bottom status bar
        egui::TopBottomPanel::bottom("bottombar").min_height(30.0).show(ctx, |ui| {
            ui.horizontal_centered(|ui| {
                // Status text for data source, such as log file path or
                // serial connection status
                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() / 2.0);
                    let (status_color, status_text) = self.data_source.status();
                    ui.label(RichText::new(status_text).color(status_color));
                    ui.label(self.data_source.info_text());
                });

                // Some buttons. TODO: refactor
                #[cfg(not(target_arch = "wasm32"))]
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    ui.checkbox(&mut self.auto_reset, "Auto-Reset");

                    if ui.button("⟲  Reset").clicked() {
                        self.reset();
                    }

                    if ui.button("➕").clicked() {
                        self.xlen /= ZOOM_FACTOR;
                    }

                    if ui.button("➖").clicked() {
                        self.xlen *= ZOOM_FACTOR;
                    }
                });
            });
        });

        // A side panel to open archived logs directly in the application
        if self.archive_panel_open {
            egui::SidePanel::left("archive").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
                ui.heading("Archived Logs");
                ui.add_space(20.0);

                for (i, (title, telem, flash)) in ARCHIVE.iter().enumerate() {
                    if i != 0 {
                        ui.separator();
                    }

                    ui.horizontal(|ui| {
                        ui.label(*title);
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            if ui.button("🖴  Flash").clicked() {
                                self.open_log_file(LogFileDataSource::from_bytes(
                                    Some(title.to_string()),
                                    flash.to_vec()
                                ));
                                self.archive_panel_open = false;
                            }

                            if ui.button("📡 Telemetry").clicked() {
                                self.open_log_file(LogFileDataSource::from_bytes(
                                    Some(title.to_string()),
                                    telem.to_vec()
                                ));
                                self.archive_panel_open = false;
                            }
                        });
                    });
                }
            });
        }

        // Top panel containing text indicators and flight mode buttons
        egui::TopBottomPanel::top("topbar").min_height(60.0).max_height(60.0).show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.style().visuals.dark_mode {
                    self.logo.show_max_size(ui, Vec2::new(ui.available_width(), 80.0));
                } else {
                    self.logo_inverted.show_max_size(ui, Vec2::new(ui.available_width(), 80.0));
                }

                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() * 0.55);

                    let time = self.vehicle_states.borrow().last().map(|vs| format!("{:10.3}", (vs.time as f32) / 1000.0));
                    let mode = self.current(|vs| vs.mode).map(|s| format!("{:?}", s));
                    let battery_voltage = self.current(|vs| vs.battery_voltage).map(|v| format!("{:.2}", v));
                    let vertical_speed = self.current(|vs| vs.vertical_speed).map(|v| format!("{:.2}", v));

                    let alt_ground = self.current(|vs| vs.altitude_ground).unwrap_or(0.0);
                    let alt = self.current(|vs| vs.altitude).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_max = self.current(|vs| vs.altitude_max).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_baro = self.current(|vs| vs.altitude_baro).map(|a| format!("{:.1}", a));
                    let alt_gps = self.current(|vs| vs.altitude_gps).map(|a| format!("{:.1}", a));

                    let last_gps_msg = self
                        .vehicle_states
                        .borrow()
                        .iter()
                        .rev()
                        .find_map(|vs| vs.gps_fix.is_some().then(|| vs))
                        .cloned();
                    let gps_status = last_gps_msg.as_ref().map(|vs| format!("{:?} ({})", vs.gps_fix.unwrap(), vs.num_satellites.unwrap_or(0)));
                    let hdop = last_gps_msg.as_ref().map(|vs| format!("{:.2}", vs.hdop.unwrap_or(9999) as f32 / 100.0));
                    let latitude = last_gps_msg.as_ref().and_then(|vs| vs.latitude).map(|l| format!("{:.6}", l));
                    let longitude = last_gps_msg.as_ref().and_then(|vs| vs.longitude).map(|l| format!("{:.6}", l));

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 3.0);
                        ui.telemetry_value("🕐 Time [s]", time);
                        ui.telemetry_value("🏷 Mode", mode);
                        ui.telemetry_value("🔋 Bat. Voltage [V]", battery_voltage);
                        ui.telemetry_value("⏱  Vertical Speed [m/s]", vertical_speed);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 2.0);
                        ui.telemetry_value("⬆  Alt. (AGL/ASL) [m]", alt);
                        ui.telemetry_value("📈 Max Alt. (AGL/ASL) [m]", alt_max);
                        ui.telemetry_value("🌁 Alt. (Baro, ASL) [m]", alt_baro);
                        ui.telemetry_value("📡 Alt. (GPS, ASL) [m]", alt_gps);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width());
                        ui.telemetry_value("📶 GPS Status (#Sats)", gps_status);
                        ui.telemetry_value("🎯 HDOP", hdop);
                        ui.telemetry_value("↕ Latitude", latitude);
                        ui.telemetry_value("↔ Longitude", longitude);
                    });
                });

                ui.vertical(|ui| {
                    let size = Vec2::new(ui.available_width(), ui.available_height() * 0.4);
                    ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                        ui.command_button("⟲  Reboot", UplinkMessage::RebootAuth(self.data_source.next_mac()), &mut self.data_source);
                        ui.command_button("🗑 Erase Flash", UplinkMessage::EraseFlashAuth(self.data_source.next_mac()), &mut self.data_source);

                        let flash_pointer: f32 = self
                            .current(|vs| vs.flash_pointer)
                            .map(|fp| (fp as f32) / 1024.0 / 1024.0)
                            .unwrap_or_default();
                        let flash_size = (FLASH_SIZE as f32) / 1024.0 / 1024.0;
                        let f = flash_pointer / flash_size;
                        let text = format!("🖴  Flash: {:.2}MiB / {:.2}MiB", flash_pointer, flash_size);
                        ui.flash_bar(ui.available_width() * 0.6, f, text);

                        let voltage = self.current(|vs| vs.battery_voltage).unwrap_or_default();
                        let f = (voltage - 6.0) / (8.4 - 6.0);
                        let text = format!("🔋 Battery: {:.2}V", voltage);
                        ui.battery_bar(ui.available_width(), f, text);
                    });

                    ui.horizontal_centered(|ui| {
                        ui.set_height(ui.available_height());
                        let w = ui.available_width() / 7.0 - style.spacing.item_spacing.x * (6.0 / 7.0);
                        let current = self.current(|vs| vs.mode);
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

        // Log scroller.
        egui::TopBottomPanel::bottom("logbar")
            .min_height(72.0)
            .resizable(true)
            .show(ctx, |ui| {
                ui.log_scroller(&self.log_messages);
            });

        // Everything else. This has to be called after all the other panels
        // are created.
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());

            let mut maxigrid = MaxiGrid::new("plot_grid", self.maxi_grid_state.clone());

            let xlen = self.xlen.clone();

            // Cloning these states is ugly. TODO: refactor
            let orientation = self.orientation_plot.clone();
            let vertical_speed = self.vertical_speed_plot.clone();
            let altitude = self.altitude_plot.clone();
            let map = self.map.clone();
            maxigrid.add_cell("Orientation",         move |ui| ui.plot_telemetry(orientation, xlen));
            maxigrid.add_cell("Vert. Speed & Accel", move |ui| ui.plot_telemetry(vertical_speed, xlen));
            maxigrid.add_cell("Altitude (ASL)",      move |ui| ui.plot_telemetry(altitude, xlen));
            maxigrid.add_cell("Position", |ui| ui.map(map));

            maxigrid.end_row();

            let gyroscope = self.gyroscope_plot.clone();
            let accelerometer = self.accelerometer_plot.clone();
            let magnetometer = self.magnetometer_plot.clone();
            let barometer = self.barometer_plot.clone();
            maxigrid.add_cell("Gyroscope",      move |ui| ui.plot_telemetry(gyroscope, xlen));
            maxigrid.add_cell("Accelerometers", move |ui| ui.plot_telemetry(accelerometer, xlen));
            maxigrid.add_cell("Magnetometer",   move |ui| ui.plot_telemetry(magnetometer, xlen));
            maxigrid.add_cell("Barometer",      move |ui| ui.plot_telemetry(barometer, xlen));

            maxigrid.end_row();

            let temperature = self.temperature_plot.clone();
            let power = self.power_plot.clone();
            let runtime = self.runtime_plot.clone();
            let signal = self.signal_plot.clone();
            maxigrid.add_cell("Temperature", move |ui| ui.plot_telemetry(temperature, xlen));
            maxigrid.add_cell("Power",       move |ui| ui.plot_telemetry(power, xlen));
            maxigrid.add_cell("Runtime",     move |ui| ui.plot_telemetry(runtime, xlen));
            maxigrid.add_cell("Signal",      move |ui| ui.plot_telemetry(signal, xlen));

            ui.add(maxigrid);
        });

        // If we have live data coming in, we need to tell egui to repaint.
        // If we don't, we shouldn't.
        if let Some(fps) = self.data_source.minimum_fps() {
            let t = std::time::Duration::from_millis(1000 / fps);
            ctx.request_repaint_after(t);
        }
    }
}

/// The main entrypoint for the egui interface.
#[cfg(not(target_arch = "wasm32"))]
pub fn main(log_file: Option<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let data_source: Box<dyn DataSource> = match log_file {
        Some(path) => Box::new(LogFileDataSource::new(path)?),
        None => Box::new(SerialDataSource::new()),
    };

    let app = Sam::init(data_source);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(1000.0, 700.0)),
            ..Default::default()
        },
        Box::new(|_cc| Box::new(app)),
    )?;

    Ok(())
}
