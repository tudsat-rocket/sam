//! Main GUI code

use std::path::PathBuf;
use std::cell::RefCell;
use std::rc::Rc;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use instant::Instant;

use eframe::egui;
use egui::FontFamily::Proportional;
use egui::{Align, TextStyle::*, Button, TextEdit, RichText, CollapsingHeader};
use egui::{FontId, Key, Layout, Vec2, Color32, FontFamily, SelectableLabel, Modifiers};
use egui_extras::RetainedImage;

use log::*;

use sting_fc_firmware::telemetry::*;

mod top_bar;
mod plot;
mod map;
mod log_scroller;
mod maxi_grid;
mod theme;
mod misc;
mod fc_settings;
mod simulation_settings;

use crate::state::*;
use crate::data_source::*;
use crate::simulation::*;
use crate::file::*;
use crate::settings::AppSettings;

use crate::gui::top_bar::*;
use crate::gui::plot::*;
use crate::gui::map::*;
use crate::gui::log_scroller::*;
use crate::gui::maxi_grid::*;
use crate::gui::theme::*;
use crate::gui::misc::*;
use crate::gui::fc_settings::*;
use crate::gui::simulation_settings::*;

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

#[derive(Debug, Copy, Clone, PartialEq)]
enum GuiTab {
    Launch,
    Plot,
    Configure
}

// The main state object of our GUI application
pub struct Sam {
    settings: AppSettings,
    data_source: Box<dyn DataSource>,
    tab: GuiTab,

    logo: RetainedImage,
    logo_inverted: RetainedImage,

    archive_panel_open: bool,
    log_scroller_open: bool,
    replay_logs: bool,
    maxi_grid_state: MaxiGridState,

    shared_plot: Rc<RefCell<SharedPlotState>>,
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
    pub fn init(cc: &eframe::CreationContext<'_>, settings: AppSettings, data_source: Box<dyn DataSource>) -> Self {
        let ctx = &cc.egui_ctx;
        let mut fonts = egui::FontDefinitions::default();
        let roboto = egui::FontData::from_static(include_bytes!("../assets/fonts/RobotoMono-Regular.ttf"));
        let lato = egui::FontData::from_static(include_bytes!("../assets/fonts/Overpass-Light.ttf"));
        fonts.font_data.insert("RobotoMono".to_owned(), roboto);
        fonts.font_data.insert("Overpass".to_owned(), lato);
        fonts.families
            .entry(FontFamily::Monospace)
            .or_default()
            .insert(0, "RobotoMono".to_owned());
        fonts.families
            .entry(FontFamily::Proportional)
            .or_default()
            .insert(0, "Overpass".to_owned());

        ctx.set_fonts(fonts);

        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));

        let r = Color32::from_rgb(0xfb, 0x49, 0x34);
        let g = Color32::from_rgb(0xb8, 0xbb, 0x26);
        let b = Color32::from_rgb(0x83, 0xa5, 0x98);
        let r1 = Color32::from_rgb(0xcc, 0x24, 0x1d);
        let g1 = Color32::from_rgb(0x98, 0x97, 0x1a);
        let b1 = Color32::from_rgb(0x45, 0x85, 0x88);
        let o = Color32::from_rgb(0xfa, 0xbd, 0x2f);
        let o1 = Color32::from_rgb(0xd6, 0x5d, 0x0e);
        let br = Color32::from_rgb(0x61, 0x48, 0x1c);
        let p = Color32::from_rgb(0xb1, 0x62, 0x86);
        let c = Color32::from_rgb(0x68, 0x9d, 0x6a);

        let orientation_plot = PlotState::new("Orientation", (Some(-180.0), Some(180.0)), shared_plot.clone())
            .line("Roll (Z) [°]", b, |vs| vs.euler_angles().map(|a| a.z))
            .line("Pitch (X) [°]", r, |vs| vs.euler_angles().map(|a| a.x))
            .line("Yaw (Y) [°]", g, |vs| vs.euler_angles().map(|a| a.y))
            .line("True Roll (Z) [°]", b1, |vs| vs.true_euler_angles().map(|a| a.z))
            .line("True Pitch (X) [°]", r1, |vs| vs.true_euler_angles().map(|a| a.x))
            .line("True Yaw (Y) [°]", g1, |vs| vs.true_euler_angles().map(|a| a.y));

        let vertical_speed_plot = PlotState::new("Vert. Speed & Accel.", (None, None), shared_plot.clone())
            .line("Vertical Accel [m/s²]", o1, |vs| vs.vertical_accel)
            .line("Vertical Accel (Filt.) [m/s²]", o, |vs| vs.vertical_accel_filtered)
            .line("Vario [m/s]", b, |vs| vs.vertical_speed)
            .line("True Vertical Accel [m/s²]", g, |vs| vs.true_vertical_accel)
            .line("True Vario [m/s]", b1, |vs| vs.true_vertical_speed);

        let altitude_plot = PlotState::new("Altitude (ASL)", (None, None), shared_plot.clone())
            .line("Altitude (Ground) [m]", br, |vs| vs.altitude_ground)
            .line("Altitude (Baro) [m]", b1, |vs| vs.altitude_baro)
            .line("Altitude [m]", b, |vs| vs.altitude)
            .line("Altitude (GPS) [m]", g, |vs| vs.altitude_gps);

        let gyroscope_plot = PlotState::new("Gyroscope", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Gyro (X) [°/s]", r, |vs| vs.gyroscope.map(|a| a.x))
            .line("Gyro (Y) [°/s]", g, |vs| vs.gyroscope.map(|a| a.y))
            .line("Gyro (Z) [°/s]", b, |vs| vs.gyroscope.map(|a| a.z));

        let accelerometer_plot = PlotState::new("Accelerometers", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Accel 2 (X) [m/s²]", r1, |vs| vs.accelerometer2.map(|a| a.x))
            .line("Accel 2 (Y) [m/s²]", g1, |vs| vs.accelerometer2.map(|a| a.y))
            .line("Accel 2 (Z) [m/s²]", b1, |vs| vs.accelerometer2.map(|a| a.z))
            .line("Accel 1 (X) [m/s²]", r, |vs| vs.accelerometer1.map(|a| a.x))
            .line("Accel 1 (Y) [m/s²]", g, |vs| vs.accelerometer1.map(|a| a.y))
            .line("Accel 1 (Z) [m/s²]", b, |vs| vs.accelerometer1.map(|a| a.z));

        let magnetometer_plot = PlotState::new("Magnetometer", (None, None), shared_plot.clone())
            .line("Mag (X) [µT]", r, |vs| vs.magnetometer.map(|a| a.x))
            .line("Mag (Y) [µT]", g, |vs| vs.magnetometer.map(|a| a.y))
            .line("Mag (Z) [µT]", b, |vs| vs.magnetometer.map(|a| a.z));

        let barometer_plot = PlotState::new("Pressures", (None, None), shared_plot.clone())
            .line("Barometer [bar]", c, |vs| vs.pressure_baro.map(|p| p / 1000.0))
            .line("Drogue Cartridge [bar]", r1, |vs| vs.drogue_cartridge_pressure)
            .line("Drogue Chamber [bar]", g1, |vs| vs.drogue_chamber_pressure)
            .line("Main Cartridge [bar]", r, |vs| vs.main_cartridge_pressure)
            .line("Main Chamber [bar]", g, |vs| vs.main_chamber_pressure);

        let temperature_plot = PlotState::new("Temperatures", (Some(25.0), Some(35.0)), shared_plot.clone())
            .line("Baro. Temp. [°C]", c, |vs| vs.temperature_baro)
            .line("Core Temp. [°C]", b, |vs| vs.temperature_core);

        let power_plot = PlotState::new("Power", (Some(0.0), Some(9.0)), shared_plot.clone())
            .line("Arm Voltage [V]", o, |vs| vs.arm_voltage)
            .line("Battery Voltage [V]", g, |vs| vs.battery_voltage)
            .line("Current [A]", o1, |vs| vs.current)
            .line("Charge Voltage [V]", b, |vs| vs.charge_voltage)
            .line("Breakwire Open?", r, |vs| vs.breakwire_open.map(|bw| bw.then(|| 1.0).unwrap_or(0.0)));

        let runtime_plot = PlotState::new("Runtime", (Some(0.0), Some(100.0)), shared_plot.clone())
            .line("CPU Util. [%]", o, |vs| vs.cpu_utilization.map(|u| u as f32))
            .line("Heap Util. [%]", g, |vs| vs.heap_utilization.map(|u| u as f32));

        let signal_plot = PlotState::new("Signal", (Some(-100.0), Some(10.0)), shared_plot.clone())
            .line("GCS RSSI [dBm]", b, |vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))
            .line("GCS Signal RSSI [dBm]", b1, |vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))
            .line("GCS SNR [dB]", c, |vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))
            .line("Vehicle RSSI [dBm]", p, |vs| vs.vehicle_lora_rssi.map(|x| x as f32 / -2.0));

        let bytes = include_bytes!("../assets/logo.png");
        let logo = RetainedImage::from_image_bytes("logo.png", bytes).unwrap();

        let bytes = include_bytes!("../assets/logo_inverted.png");
        let logo_inverted = RetainedImage::from_image_bytes("logo_inverted.png", bytes).unwrap();
        let map = MapState::new(settings.mapbox_access_token.clone());

        Self {
            settings,
            data_source,
            tab: GuiTab::Plot,
            logo,
            logo_inverted,
            archive_panel_open: cfg!(target_arch = "wasm32"),
            log_scroller_open: cfg!(target_arch = "x86_64"),
            replay_logs: false,
            maxi_grid_state: MaxiGridState::default(),
            shared_plot,
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
    fn reset(&mut self, keep_position: bool) {
        info!("Resetting.");
        self.data_source.reset();
        let now = Instant::now();
        self.all_plots(|plot| plot.reset(now, keep_position));
        self.map.reset();
    }

    fn show_all(&mut self) {
        self.all_plots(|plot| plot.show_all());
    }

    /// Returns the "current" value for the given callback. This is the last
    /// known of the value at the current time.
    /// TODO: incorporate cursor position
    fn current<T>(&mut self, callback: impl Fn(&VehicleState) -> Option<T>) -> Option<T> {
        self.data_source.vehicle_states().rev().find_map(|(_t, msg)| callback(msg))
    }

    /// Opens a log file data source
    fn open_log_file(&mut self, ds: LogFileDataSource) {
        self.reset(false);
        self.data_source = Box::new(ds);
    }

    /// Closes the currently opened data source
    fn close_data_source(&mut self) {
        self.reset(false);
        self.data_source = Box::new(SerialDataSource::new(self.settings.lora.clone()));
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();
        #[cfg(feature = "profiling")]
        puffin::GlobalProfiler::lock().new_frame();
        #[cfg(feature = "profiling")]
        puffin_egui::profiler_window(ctx);

        // Process new messages
        let new: Vec<(Instant, VehicleState)> = self.data_source.new_vehicle_states().cloned().collect();
        for (time, vs) in &new {
            self.all_plots(|plot| plot.push(*time, vs));
            self.map.push(*time, vs);
        }

        // TODO: only send this if we know it's not a ground station?
        if self.data_source.fc_settings().is_none() && self.data_source.vehicle_states().next().is_some() {
            self.data_source.send(UplinkMessage::ReadSettings).unwrap();
        }

        self.shared_plot.borrow_mut().set_end(self.data_source.end());

        if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F1)) {
            self.tab = GuiTab::Launch;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F2)) {
            self.tab = GuiTab::Plot;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F3)) {
            self.tab = GuiTab::Configure;
        }

        // Check for keyboard inputs.
        let shortcut_mode = if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F5)) {
            Some(FlightMode::Idle)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F6)) {
            Some(FlightMode::HardwareArmed)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F7)) {
            Some(FlightMode::Armed)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F8)) {
            Some(FlightMode::Flight)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F9)) {
            Some(FlightMode::RecoveryDrogue)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F10)) {
            Some(FlightMode::RecoveryMain)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F11)) {
            Some(FlightMode::Landed)
        } else {
            None
        };

        if let Some(fm) = shortcut_mode {
            self.data_source
                .send_command(Command::SetFlightMode(fm))
                .unwrap();
        }

        // Redefine text_styles
        let colors = ThemeColors::new(ctx);
        let mut style = (*ctx.style()).clone();
        colors.apply(&mut style);
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

                ui.selectable_value(&mut self.tab, GuiTab::Launch, "🚀 Launch (F1)");
                ui.selectable_value(&mut self.tab, GuiTab::Plot, "📈 Plot (F2)");
                ui.selectable_value(&mut self.tab, GuiTab::Configure, "⚙  Configure (F3)");

                ui.separator();

                // Opening files manually is not available on web assembly
                #[cfg(target_arch = "x86_64")]
                if ui.selectable_label(false, "🗁  Open Log File").clicked() {
                    if let Some(data_source) = open_log_file() {
                        self.open_log_file(data_source);
                    }
                }

                // Toggle archive panel
                let text = if self.archive_panel_open { "🗄 Close Archive" } else { "🗄 Open Archive" };
                ui.toggle_value(&mut self.archive_panel_open, text);

                // Toggle archive panel
                if ui.selectable_label(self.data_source.simulation_settings().is_some(), "💻 Simulate").clicked() {
                    self.data_source = Box::new(SimulationDataSource::default());
                    self.reset(false);
                    self.show_all();
                }

                // Show a button to the right to close the current log
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    if self.data_source.is_log_file() || self.data_source.simulation_settings().is_some() {
                        if ui.button("❌").clicked() {
                            self.close_data_source();
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
                    ui.colored_label(status_color, status_text);
                    ui.label(self.data_source.fc_settings().map(|s| s.identifier.clone()).unwrap_or_default());
                    ui.colored_label(Color32::GRAY, self.data_source.info_text());
                });

                // Some buttons
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    #[cfg(not(target_arch = "wasm32"))]
                    ui.add_enabled_ui(!self.data_source.is_log_file(), |ui| {
                        if ui.button("⏮  Reset").clicked() {
                            self.reset(false);
                        }
                    });

                    #[cfg(not(target_arch = "wasm32"))]
                    ui.separator();

                    ui.toggle_button(&mut self.log_scroller_open, "🗊  Show Logs", "🗊  Close Logs");
                    ui.toggle_button(&mut self.shared_plot.borrow_mut().show_stats, "📈 Show Stats", "📉 Hide Stats");
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
                                    Some(format!("{} (Flash)", title)),
                                    flash.to_vec(),
                                    self.replay_logs
                                ));
                                self.archive_panel_open = false;
                            }

                            if ui.button("📡 Telemetry").clicked() {
                                self.open_log_file(LogFileDataSource::from_bytes(
                                    Some(format!("{} (Telemetry)", title)),
                                    telem.to_vec(),
                                    self.replay_logs
                                ));
                                self.archive_panel_open = false;
                            }
                        });
                    });
                }

                ui.add_space(ui.available_height() - 20.0);
                ui.checkbox(&mut self.replay_logs, "Replay logs");
            });
        }

        if self.data_source.simulation_settings().is_some() {
            let old_settings = self.data_source.simulation_settings().unwrap().clone();

            egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
                ui.heading("Simulation");
                ui.add_space(20.0);

                CollapsingHeader::new("Simulation Parameters").default_open(true).show(ui, |ui| {
                    let settings = self.data_source.simulation_settings().unwrap();
                    settings.ui(ui)
                });

                CollapsingHeader::new("(Simulated) FC Settings").default_open(false).show(ui, |ui| {
                    let settings = self.data_source.simulation_settings().unwrap();
                    settings.fc_settings.ui(ui, None)
                });

                ui.add_space(20.0);

                let changed = *self.data_source.simulation_settings().unwrap() != old_settings;
                let released = false;
                ui.horizontal(|ui| {
                    if ui.button("Reset").clicked() {
                        *(self.data_source.simulation_settings().unwrap()) = SimulationSettings::default();
                    }

                    if ui.button("↻  Rerun").clicked() || (changed && released) {
                        self.reset(true);
                    }
                });
            });
        }

        // Top panel containing text indicators and flight mode buttons
        egui::TopBottomPanel::top("topbar").min_height(60.0).max_height(60.0).show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.style().visuals.dark_mode {
                    self.logo.show_max_size(ui, Vec2::new(ui.available_width(), 90.0));
                } else {
                    self.logo_inverted.show_max_size(ui, Vec2::new(ui.available_width(), 90.0));
                }

                let spacing = 3.0; // TODO: this is ugly

                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() * 0.50);

                    let time = self.data_source.vehicle_states().last().map(|(_t, msg)| format!("{:10.3}", (msg.time as f32) / 1000.0));
                    let mode = self.current(|vs| vs.mode).map(|s| format!("{:?}", s));
                    let battery_voltage = self.current(|vs| vs.battery_voltage).map(|v| format!("{:.2}", v));
                    let vertical_speed = self.current(|vs| vs.vertical_speed).map(|v| format!("{:.2}", v));

                    let alt_ground = self.current(|vs| vs.altitude_ground).unwrap_or(0.0);
                    let alt = self.current(|vs| vs.altitude).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_max = self.current(|vs| vs.altitude_max).map(|a| format!("{:.1} ({:.1})", a - alt_ground, a));
                    let alt_baro = self.current(|vs| vs.altitude_baro).map(|a| format!("{:.1}", a));
                    let alt_gps = self.current(|vs| vs.altitude_gps).map(|a| format!("{:.1}", a));

                    let last_gps = self.data_source
                        .vehicle_states()
                        .rev()
                        .find_map(|(_t, vs)| vs.gps_fix.is_some().then(|| vs))
                        .cloned();
                    let gps_status = last_gps.as_ref().map(|vs| format!("{:?} ({})", vs.gps_fix.unwrap(), vs.num_satellites.unwrap_or(0)));
                    let hdop = last_gps.as_ref().map(|vs| format!("{:.2}", vs.hdop.unwrap_or(9999) as f32 / 100.0));
                    let latitude = last_gps.as_ref().and_then(|vs| vs.latitude).map(|l| format!("{:.6}", l));
                    let longitude = last_gps.as_ref().and_then(|vs| vs.longitude).map(|l| format!("{:.6}", l));

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 4.0);
                        ui.add_space(spacing);
                        ui.telemetry_value("🕐", "Time [s]", time);
                        ui.telemetry_value("🏷", "Mode", mode);
                        ui.telemetry_value("🔋", "Bat. Voltage [V]", battery_voltage);
                        ui.telemetry_value("⏱", "Vertical Speed [m/s]", vertical_speed);
                        ui.add_space(spacing);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width() / 2.0);
                        ui.add_space(spacing);
                        ui.telemetry_value("⬆", "Alt. (AGL/ASL) [m]", alt);
                        ui.telemetry_value("📈", "Max Alt. (AGL/ASL) [m]", alt_max);
                        ui.telemetry_value("☁", "Alt. (Baro, ASL) [m]", alt_baro);
                        ui.telemetry_value("📡", "Alt. (GPS, ASL) [m]", alt_gps);
                    });

                    ui.vertical(|ui| {
                        ui.set_width(ui.available_width());
                        ui.add_space(spacing);
                        ui.telemetry_value("📶", "GPS Status (#Sats)", gps_status);
                        ui.telemetry_value("🎯", "HDOP", hdop);
                        ui.telemetry_value("↕", "Latitude", latitude);
                        ui.telemetry_value("↔", "Longitude", longitude);
                    });
                });

                ui.vertical(|ui| {
                    ui.label("Data Rate [Hz]");
                    ui.horizontal(|ui| {
                        let current = self.current(|vs| vs.telemetry_data_rate).unwrap_or_default();
                        if ui.add_sized([50.0, 20.0], SelectableLabel::new(current == TelemetryDataRate::Low, "20")).clicked() {
                            self.data_source.send_command(Command::SetDataRate(TelemetryDataRate::Low)).unwrap();
                        }
                        if ui.add_sized([50.0, 20.0], SelectableLabel::new(current == TelemetryDataRate::High, "40")).clicked() {
                            self.data_source.send_command(Command::SetDataRate(TelemetryDataRate::High)).unwrap();
                        }
                    });

                    ui.label("Transmit Power [dBm]");
                    ui.horizontal(|ui| {
                        let current = self.current(|vs| vs.transmit_power).unwrap_or_default();
                        if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == TransmitPower::P14dBm, "14")).clicked() {
                            self.data_source.send_command(Command::SetTransmitPower(TransmitPower::P14dBm)).unwrap();
                        }
                        if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == TransmitPower::P17dBm, "17")).clicked() {
                            self.data_source.send_command(Command::SetTransmitPower(TransmitPower::P17dBm)).unwrap();
                        }
                        if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == TransmitPower::P20dBm, "20")).clicked() {
                            self.data_source.send_command(Command::SetTransmitPower(TransmitPower::P20dBm)).unwrap();
                        }
                        if ui.add_sized([25.0, 20.0], SelectableLabel::new(current == TransmitPower::P22dBm, "22")).clicked() {
                            self.data_source.send_command(Command::SetTransmitPower(TransmitPower::P22dBm)).unwrap();
                        }
                    });
                });

                ui.vertical(|ui| {
                    let size = Vec2::new(ui.available_width(), ui.available_height() * 0.4);
                    ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                        ui.command_button("⟲  Reboot", Command::Reboot, &mut self.data_source);
                        ui.command_button("🗑 Erase Flash", Command::EraseFlash, &mut self.data_source);

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
                        ui.set_height(ui.available_height() - spacing);
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
        if self.log_scroller_open {
            let mut frame = egui::containers::Frame::side_top_panel(&style);
            frame.fill = colors.background_weak;
            egui::TopBottomPanel::bottom("logbar")
                .min_height(72.0)
                .resizable(true)
                .frame(frame)
                .show(ctx, |ui| {
                    ui.log_scroller(self.data_source.log_messages());
                });
        }

        // Everything else. This has to be called after all the other panels
        // are created.
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());

            match self.tab {
                GuiTab::Launch => {}, // TODO
                GuiTab::Plot => {
                    MaxiGrid::new((4, 3), ui, self.maxi_grid_state.clone())
                        .cell("Orientation",         |ui| ui.plot_telemetry(&self.orientation_plot))
                        .cell("Vert. Speed & Accel", |ui| ui.plot_telemetry(&self.vertical_speed_plot))
                        .cell("Altitude (ASL)",      |ui| ui.plot_telemetry(&self.altitude_plot))
                        .cell("Position",            |ui| ui.map(&self.map))
                        .cell("Gyroscope",           |ui| ui.plot_telemetry(&self.gyroscope_plot))
                        .cell("Accelerometers",      |ui| ui.plot_telemetry(&self.accelerometer_plot))
                        .cell("Magnetometer",        |ui| ui.plot_telemetry(&self.magnetometer_plot))
                        .cell("Pressures",           |ui| ui.plot_telemetry(&self.barometer_plot))
                        .cell("Temperature",         |ui| ui.plot_telemetry(&self.temperature_plot))
                        .cell("Power",               |ui| ui.plot_telemetry(&self.power_plot))
                        .cell("Runtime",             |ui| ui.plot_telemetry(&self.runtime_plot))
                        .cell("Signal",              |ui| ui.plot_telemetry(&self.signal_plot));
                },
                GuiTab::Configure => {
                    ui.horizontal(|ui| {
                        ui.set_width(ui.available_width());
                        ui.set_height(ui.available_height());

                        ui.vertical(|ui| {
                            ui.set_width(ui.available_width()/2.0);
                            ui.set_height(ui.available_height());

                            ui.add_space(10.0);
                            ui.heading("GCS Settings");
                            ui.add_space(10.0);

                            egui::Grid::new("app_settings_grid")
                                .num_columns(2)
                                .spacing([40.0, 4.0])
                                .striped(true)
                                .show(ui, |ui| {
                                    ui.label("MapBox Access Token");
                                    ui.add_sized(ui.available_size(), TextEdit::singleline(&mut self.settings.mapbox_access_token));
                                    ui.end_row();

                                    ui.label("LoRa channel selection (500kHz BW)");
                                    ui.vertical(|ui| {
                                        ui.horizontal(|ui| {
                                            ui.toggle_value(&mut self.settings.lora.channels[0], RichText::new("863.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[1], RichText::new("863.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[2], RichText::new("864.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[3], RichText::new("864.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[4], RichText::new("865.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[5], RichText::new("865.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[6], RichText::new("866.25").monospace().size(10.0));
                                            ui.label(RichText::new("MHz").weak().size(10.0));
                                        });
                                        ui.horizontal(|ui| {
                                            ui.toggle_value(&mut self.settings.lora.channels[7],  RichText::new("866.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[8],  RichText::new("867.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[9],  RichText::new("867.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[10], RichText::new("868.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[11], RichText::new("868.75").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[12], RichText::new("869.25").monospace().size(10.0));
                                            ui.toggle_value(&mut self.settings.lora.channels[13], RichText::new("869.75").monospace().size(10.0));
                                            ui.label(RichText::new("MHz").weak().size(10.0));
                                        });
                                    });
                                    ui.end_row();

                                    ui.label("LoRa binding phrase");
                                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                        if ui.button("⬅ Copy from FC").clicked() {
                                            self.settings.lora.binding_phrase = self.data_source
                                                .fc_settings()
                                                .map(|s| s.lora.binding_phrase.clone())
                                                .unwrap_or(self.settings.lora.binding_phrase.clone());
                                        }

                                        ui.add_sized(ui.available_size(), TextEdit::singleline(&mut self.settings.lora.binding_phrase));
                                    });
                                    ui.end_row();

                                    ui.label("LoRa uplink key");
                                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                        if ui.button("⬅ Copy from FC").clicked() {
                                            self.settings.lora.authentication_key = self.data_source
                                                .fc_settings()
                                                .map(|s| s.lora.authentication_key.clone())
                                                .unwrap_or(self.settings.lora.authentication_key.clone());
                                        }

                                        if ui.button("🔃Rekey").clicked() {
                                            self.settings.lora.authentication_key = rand::random();
                                        }

                                        ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                                            ui.monospace(format!("{:032x}", self.settings.lora.authentication_key));
                                        })
                                    });
                                    ui.end_row();
                                });

                            ui.add_space(20.0);

                            ui.horizontal_centered(|ui| {
                                ui.set_width(ui.available_width());

                                if ui.button("🔃Reload").clicked() {
                                    self.settings = AppSettings::load()
                                        .unwrap_or(AppSettings::default());
                                }

                                if ui.button("💾 Save Settings").clicked() {
                                    self.settings.save().unwrap();
                                    self.map.set_access_token(self.settings.mapbox_access_token.clone());
                                    self.data_source.apply_settings(&self.settings);
                                }
                            });
                        });

                        ui.separator();

                        ui.vertical(|ui| {
                            ui.set_width(ui.available_width());

                            ui.add_space(10.0);
                            ui.heading("FC Settings");
                            ui.add_space(10.0);

                            if let Some(settings) = self.data_source.fc_settings_mut() {
                                settings.ui(ui, Some(&self.settings));
                            } else {
                                ui.colored_label(Color32::GRAY, "Not connected.");
                            }

                            ui.add_space(20.0);

                            ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                                if ui.add_enabled(self.data_source.fc_settings().is_some(), Button::new("💾 Write Settings & Reboot")).clicked() {
                                    let settings = self.data_source.fc_settings().cloned().unwrap();
                                    self.data_source.send(UplinkMessage::WriteSettings(settings)).unwrap();
                                }

                                if ui.add_enabled(self.data_source.fc_settings().is_some(), Button::new("🖹 Save to File")).clicked() {
                                    save_fc_settings_file(&self.data_source.fc_settings().unwrap());
                                }

                                if ui.add_enabled(self.data_source.fc_settings().is_some(), Button::new("🖹 Load from File")).clicked() {
                                    if let Some(settings) = open_fc_settings_file() {
                                        info!("Loaded settings: {:?}", settings);
                                        *self.data_source.fc_settings_mut().unwrap() = settings;
                                    }
                                }

                                if ui.button("🔃Reload").clicked() {
                                    self.data_source.send(UplinkMessage::ReadSettings).unwrap();
                                }
                            });
                        });
                    });
                }
            }
        });

        // If we have live data coming in, we need to tell egui to repaint.
        // If we don't, we shouldn't.
        if let Some(fps) = self.data_source.minimum_fps() {
            let t = std::time::Duration::from_millis(1000 / fps);
            ctx.request_repaint_after(t);
        }
    }
}

impl eframe::App for Sam {
    /// Main draw method of the application
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.ui(ctx)
    }
}

/// The main entrypoint for the egui interface.
#[cfg(not(target_arch = "wasm32"))]
pub fn main(log_file: Option<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let app_settings = AppSettings::load().ok().unwrap_or(AppSettings::default());

    let data_source: Box<dyn DataSource> = match log_file {
        Some(path) => Box::new(LogFileDataSource::new(path)?),
        None => Box::new(SerialDataSource::new(app_settings.lora.clone())),
    };

    #[cfg(feature = "profiling")]
    puffin::set_scopes_on(true);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            initial_window_size: Some(egui::vec2(1000.0, 700.0)),
            ..Default::default()
        },
        Box::new(|cc| Box::new(Sam::init(cc, app_settings, data_source))),
    )?;

    Ok(())
}
