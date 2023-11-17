//! Main GUI code

use std::io::Write;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender};

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::egui;
use egui::FontFamily::Proportional;
use egui::{Align, Button, CollapsingHeader, Color32, FontFamily, FontId, Key, Layout, Modifiers, Vec2};
use egui::{Align2, ProgressBar, TextStyle::*};

use futures::StreamExt;
use log::*;

use mithril::telemetry::*;

mod components;
mod fc_settings;
mod map;
mod maxi_grid;
mod misc;
mod plot;
mod simulation_settings;
mod tabs;
mod theme;
mod top_bar;

use crate::data_source::*;
use crate::settings::AppSettings;
use crate::simulation::*;
use crate::state::*;

use crate::gui::components::top_menu_bar::*;
use crate::gui::fc_settings::*;
use crate::gui::simulation_settings::*;
use crate::gui::tabs::*;
use crate::gui::theme::*;
use crate::gui::top_bar::*;

// Log files included with the application. These should probably be fetched
// if necessary to reduce application size.
// TODO: migrate old launches
pub const ARCHIVE: [(&str, Option<&'static str>, Option<&'static str>); 5] = [
    ("Zülpich #1", None, None),
    ("Zülpich #2", None, None),
    (
        "DARE (FC A)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_flash_filtered.json"),
    ),
    (
        "DARE (FC B)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_flash_filtered.json"),
    ),
    (
        "EuRoC 2023 (ÆSIR Signý)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_flash_filtered.json"),
    ),
];

#[derive(Debug)]
enum ArchiveLoadProgress {
    Progress((u64, u64)),
    Complete(Vec<u8>),
    Error(reqwest::Error),
}

// The main state object of our GUI application
pub struct Sam {
    settings: AppSettings,
    data_source: Box<dyn DataSource>,
    tab: GuiTab,

    plot_tab: PlotTab,
    configure_tab: ConfigureTab,

    archive_window_open: bool,
    replay_logs: bool,
    archive_progress_receiver: Option<Receiver<ArchiveLoadProgress>>,
    archive_progress: Option<(u64, u64)>,
}

impl Sam {
    /// Initialize the application, including the state objects for widgets
    /// such as plots and maps.
    pub fn init(ctx: &egui::Context, settings: AppSettings, data_source: Box<dyn DataSource>) -> Self {
        let mut fonts = egui::FontDefinitions::default();
        let roboto = egui::FontData::from_static(include_bytes!("../assets/fonts/RobotoMono-Regular.ttf"));
        let lato = egui::FontData::from_static(include_bytes!("../assets/fonts/Overpass-Light.ttf"));
        fonts.font_data.insert("RobotoMono".to_owned(), roboto);
        fonts.font_data.insert("Overpass".to_owned(), lato);
        fonts.families.entry(FontFamily::Monospace).or_default().insert(0, "RobotoMono".to_owned());
        fonts.families.entry(FontFamily::Proportional).or_default().insert(0, "Overpass".to_owned());

        ctx.set_fonts(fonts);

        let plot_tab = PlotTab::init(&settings);
        let configure_tab = ConfigureTab::init();

        egui_extras::install_image_loaders(ctx);

        Self {
            settings,
            data_source,

            tab: GuiTab::Plot,
            plot_tab,
            configure_tab,

            archive_window_open: cfg!(target_arch = "wasm32"),
            replay_logs: false,
            archive_progress_receiver: None,
            archive_progress: None,
        }
    }

    /// Returns the "current" value for the given callback. This is the last
    /// known of the value at the current time.
    /// TODO: incorporate cursor position?
    fn current<T>(&mut self, callback: impl Fn(&VehicleState) -> Option<T>) -> Option<T> {
        self.data_source.vehicle_states().rev().find_map(|(_t, msg)| callback(msg))
    }

    /// Opens a log file data source
    fn open_log_file(&mut self, ds: LogFileDataSource) {
        self.data_source = Box::new(ds);
    }

    async fn load_archive_log(url: &str, progress_sender: Sender<ArchiveLoadProgress>) {
        let start = Instant::now();
        let response = match reqwest::Client::new().get(url).send().await {
            Ok(res) => res,
            Err(e) => {
                progress_sender.send(ArchiveLoadProgress::Error(e)).unwrap();
                return;
            }
        };

        let total_size = response.content_length().unwrap_or(0);
        progress_sender.send(ArchiveLoadProgress::Progress((0, total_size))).unwrap();

        let mut cursor = std::io::Cursor::new(Vec::with_capacity(total_size as usize));
        let (mut progress, mut last_progress) = (0, 0);
        let mut stream = response.bytes_stream();
        while let Some(result) = stream.next().await {
            match result {
                Ok(chunk) => {
                    cursor.write_all(&chunk).unwrap();
                    progress = u64::min(progress + chunk.len() as u64, total_size);
                    if progress == total_size || progress > last_progress + 256 * 1024 {
                        let _ = progress_sender.send(ArchiveLoadProgress::Progress((progress, total_size)));
                        last_progress = progress;
                    }
                }
                Err(e) => {
                    progress_sender.send(ArchiveLoadProgress::Error(e)).unwrap();
                    return;
                }
            }
        }

        progress_sender.send(ArchiveLoadProgress::Complete(cursor.into_inner())).unwrap();
        let duration = start.elapsed().as_secs_f32();
        let mib = (total_size as f32) / 1024.0 / 1024.0;
        info!("Downloaded {}MiB in {:.1}ms ({}MiB/s)", mib, duration * 1000.0, mib / duration);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn open_archive_log(&mut self, url: &'static str) {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.archive_progress_receiver = Some(receiver);
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
            rt.block_on(Self::load_archive_log(url, sender));
        });
    }

    #[cfg(target_arch = "wasm32")]
    fn open_archive_log(&mut self, url: &'static str) {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.archive_progress_receiver = Some(receiver);
        wasm_bindgen_futures::spawn_local(Self::load_archive_log(url, sender));
    }

    /// Closes the currently opened data source
    fn close_data_source(&mut self) {
        self.data_source = Box::new(SerialDataSource::new(self.settings.lora.clone()));
    }

    fn text_telemetry(&mut self, ui: &mut egui::Ui) {
        let spacing = 3.0; // TODO: this is ugly

        let time = self
            .data_source
            .vehicle_states()
            .last()
            .map(|(_t, msg)| format!("{:10.3}", (msg.time as f32) / 1000.0));
        let rssi = self.current(|vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0));
        let mode = self.current(|vs| vs.mode).map(|s| format!("{:?}", s));

        let alt_ground = self.current(|vs| vs.altitude_ground).unwrap_or(0.0);
        let alt_agl = self.current(|vs| vs.altitude.map(|a| a - alt_ground));
        let alt_max = self.current(|vs| vs.altitude_max.map(|a| a - alt_ground));
        let vertical_accel = self.current(|vs| vs.vertical_accel_filtered);

        let last_gps = self
            .data_source
            .vehicle_states()
            .rev()
            .find_map(|(_t, vs)| vs.gps_fix.is_some().then(|| vs))
            .cloned();
        let gps_status = last_gps.as_ref().map(|vs| format!("{:?}", vs.gps_fix.unwrap()));
        let hdop = last_gps.as_ref().map(|vs| vs.hdop.unwrap_or(9999) as f32 / 100.0);
        let latitude = last_gps.as_ref().and_then(|vs| vs.latitude);
        let longitude = last_gps.as_ref().and_then(|vs| vs.longitude);
        let coords = latitude.and_then(|lat| longitude.map(|lng| format!("{:.5},{:.5}", lat, lng)));

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 3.5);
            ui.add_space(spacing);
            ui.telemetry_value("🕐", "Time [s]", time);
            ui.telemetry_value("🏷", "Mode", mode);
            ui.nominal_value("🔥", "Baro Temp. [°C]", self.current(|vs| vs.temperature_baro), 1, 0.0, 60.0);
            ui.nominal_value("📡", "RSSI [dBm]", rssi, 1, -50.0, 0.0);
            ui.nominal_value("📶", "Link Quality [%]", self.data_source.link_quality(), 1, 90.0, 101.0);
            ui.add_space(ui.spacing().item_spacing.y);
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width() / 2.0);
            ui.add_space(spacing);
            ui.nominal_value("📈", "Altitude (AGL) [m]", alt_agl, 1, -1.0, 10000.0);
            ui.nominal_value("📈", "Max Alt. (AGL) [m]", alt_max, 1, -1.0, 10000.0);
            ui.nominal_value("☁", "Baro. Alt. (ASL) [m]", self.current(|vs| vs.altitude_baro), 1, -100.0, 10000.0);
            ui.nominal_value("⏱", "Vertical Speed [m/s]", self.current(|vs| vs.vertical_speed), 2, -1.0, 1.0);
            ui.nominal_value("⬆", "Vertical Accel. [m/s²]", vertical_accel, 1, -1.0, 1.0);
        });

        ui.vertical(|ui| {
            ui.set_width(ui.available_width());
            ui.add_space(spacing);
            ui.telemetry_value("🌍", "GPS Status", gps_status);
            ui.nominal_value("📶", "# Sats", self.current(|vs| vs.num_satellites.map(|n| n as f32)), 0, 5.0, 99.0);
            ui.nominal_value("🎯", "HDOP", hdop, 2, 0.0, 5.0);
            ui.nominal_value("📡", "GPS Alt. (ASL) [m]", self.current(|vs| vs.altitude_gps), 1, -100.0, 10000.0);
            ui.telemetry_value("🌐", "Coords", coords);
        });
    }

    pub fn top_bar(&mut self, ui: &mut egui::Ui, vertical: bool) {
        if vertical {
            ui.horizontal(|ui| {
                self.text_telemetry(ui);
            });
        } else {
            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width() * 0.50);
                self.text_telemetry(ui);
            });
        }

        ui.separator();

        let current_data_rate = self.current(|vs| vs.telemetry_data_rate).unwrap_or_default();
        let current_transmit_power = self.current(|vs| vs.transmit_power).unwrap_or_default();

        if vertical {
            ui.columns(4, |uis| {
                uis[0].add_space(3.0);
                uis[0].label("Data Rate [Hz]");
                uis[1].data_rate_controls(current_data_rate, self.data_source.as_mut());
                uis[2].add_space(3.0);
                uis[2].label("Transmit Power [dBm]");
                uis[3].transmit_power_controls(current_transmit_power, self.data_source.as_mut());
            });
        } else {
            ui.vertical(|ui| {
                ui.add_space(3.0);
                ui.label("Data Rate [Hz]");
                ui.data_rate_controls(current_data_rate, self.data_source.as_mut());
                ui.add_space(3.0);
                ui.label("Transmit Power [dBm]");
                ui.transmit_power_controls(current_transmit_power, self.data_source.as_mut());
            });
        }

        ui.separator();

        ui.vertical(|ui| {
            let size = Vec2::new(ui.available_width(), 30.0);
            ui.allocate_ui_with_layout(size, Layout::right_to_left(Align::Center), |ui| {
                ui.command_button("⟲  Reboot", Command::Reboot, self.data_source.as_mut());
                ui.command_button("🗑 Erase Flash", Command::EraseFlash, self.data_source.as_mut());
                ui.flash_bar(ui.available_width() * 0.6, self.current(|vs| vs.flash_pointer));
                ui.battery_bar(ui.available_width(), self.current(|vs| vs.battery_voltage));
            });

            ui.separator();

            ui.allocate_ui(ui.available_size(), |ui| {
                ui.flight_mode_buttons(self.current(|vs| vs.mode), self.data_source.as_mut());
            });
        });
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();
        #[cfg(feature = "profiling")]
        puffin::GlobalProfiler::lock().new_frame();
        #[cfg(feature = "profiling")]
        puffin_egui::profiler_window(ctx);

        self.data_source.update(ctx);

        // TODO: only send this if we know it's not a ground station?
        if self.data_source.fc_settings().is_none() && self.data_source.vehicle_states().next().is_some() {
            self.data_source.send(UplinkMessage::ReadSettings).unwrap();
        }

        if let Some(receiver) = self.archive_progress_receiver.as_ref() {
            match receiver.try_recv() {
                Ok(ArchiveLoadProgress::Progress(progress)) => {
                    self.archive_progress = Some(progress);
                }
                Ok(ArchiveLoadProgress::Complete(bytes)) => {
                    self.open_log_file(LogFileDataSource::from_bytes(
                        Some("".to_string()), // TODO: show title
                        bytes,
                        self.replay_logs,
                    ));
                    self.archive_window_open = false;
                    self.archive_progress_receiver = None;
                    self.archive_progress = None;
                }
                Ok(ArchiveLoadProgress::Error(e)) => {
                    error!("{:?}", e); // TODO: show this visually
                    self.archive_progress_receiver = None;
                    self.archive_progress = None;
                }
                _ => {}
            }
        }

        // Check for keyboard inputs for tab and flight mode changes
        if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F1)) {
            self.tab = GuiTab::Launch;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F2)) {
            self.tab = GuiTab::Plot;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F3)) {
            self.tab = GuiTab::Configure;
        }

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

        // Set new flight mode if keyboard shortcut was used
        if let Some(fm) = shortcut_mode {
            self.data_source.send_command(Command::SetFlightMode(fm)).unwrap();
        }

        // Redefine text_styles
        let colors = ThemeColors::new(ctx);
        let mut style = (*ctx.style()).clone();
        colors.apply(&mut style);
        style.text_styles.insert(Heading, FontId::new(14.0, Proportional));
        ctx.set_style(style.clone());

        // Prevent unnecessarily large UI on non-high-DPI displays
        #[cfg(not(target_arch = "wasm32"))]
        if ctx.pixels_per_point() > 1.0 && ctx.pixels_per_point() <= 1.5 {
            ctx.set_pixels_per_point(1.0);
        }

        // Top menu bar
        create_top_menu_bar(self, ctx);

        // A window to open archived logs directly in the application
        let mut archive_open = self.archive_window_open; // necessary to avoid mutably borrowing self
        egui::Window::new("Flight Archive")
            .open(&mut archive_open)
            .min_width(300.0)
            .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
            .resizable(false)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.add_space(10.0);

                for (i, (title, telem, flash)) in ARCHIVE.iter().enumerate() {
                    if i != 0 {
                        ui.separator();
                    }

                    ui.horizontal(|ui| {
                        ui.label(*title);
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            if ui.add_enabled(flash.is_some(), Button::new("🖴  Flash")).clicked() {
                                self.open_archive_log(flash.unwrap());
                            }

                            if ui.add_enabled(telem.is_some(), Button::new("📡 Telemetry")).clicked() {
                                self.open_archive_log(telem.unwrap());
                            }
                        });
                    });
                }

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    ui.add_visible_ui(self.archive_progress.is_some(), |ui| {
                        let (done, total) = self.archive_progress.unwrap_or((0, 0));
                        let f = (total > 0).then(|| done as f32 / total as f32).unwrap_or(0.0);
                        let text = format!(
                            "{:.2}MiB / {:.2}MiB",
                            done as f32 / (1024.0 * 1024.0),
                            total as f32 / (1024.0 * 1024.0)
                        );
                        ui.add_sized([ui.available_width(), 20.0], ProgressBar::new(f).text(text));
                    });
                });
                ui.add_space(10.0);

                ui.checkbox(&mut self.replay_logs, "Replay logs");
            });
        self.archive_window_open = archive_open;

        if self.data_source.simulation_settings().is_some() {
            let old_settings = self.data_source.simulation_settings().unwrap().clone();

            egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
                ui.set_enabled(!self.archive_window_open);
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
                        self.data_source.reset();
                    }
                });
            });
        }

        // Top panel containing text indicators and flight mode buttons
        if ctx.screen_rect().width() > 1000.0 {
            egui::TopBottomPanel::top("topbar").min_height(60.0).max_height(60.0).show(ctx, |ui| {
                ui.set_enabled(!self.archive_window_open);
                ui.horizontal_centered(|ui| {
                    self.top_bar(ui, false);
                });
            });
        } else {
            egui::TopBottomPanel::top("topbar").min_height(20.0).max_height(300.0).show(ctx, |ui| {
                ui.set_enabled(!self.archive_window_open);
                CollapsingHeader::new("Status & Controls").default_open(false).show(ui, |ui| {
                    self.top_bar(ui, true);
                    ui.add_space(10.0);
                });
            });
        }

        // Bottom status bar
        egui::TopBottomPanel::bottom("bottombar").min_height(30.0).show(ctx, |ui| {
            ui.set_enabled(!self.archive_window_open);
            ui.horizontal_centered(|ui| {
                // Give the data source some space on the left ...
                ui.horizontal_centered(|ui| {
                    ui.set_width(ui.available_width() / 2.0);
                    self.data_source.status_bar_ui(ui);
                });

                // ... and the current tab some space on the right.
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    #[cfg(not(target_arch = "wasm32"))]
                    ui.add_enabled_ui(!self.data_source.is_log_file(), |ui| {
                        if ui.button("⏮  Reset").clicked() {
                            self.data_source.reset();
                        }
                    });

                    #[cfg(not(target_arch = "wasm32"))]
                    ui.separator();

                    match self.tab {
                        GuiTab::Launch => {}
                        GuiTab::Plot => self.plot_tab.bottom_bar_ui(ui, self.data_source.as_mut()),
                        GuiTab::Configure => {}
                    }
                });
            });
        });

        // Everything else. This has to be called after all the other panels
        // are created.
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_width(ui.available_width());
            ui.set_height(ui.available_height());
            ui.set_enabled(!self.archive_window_open);

            match self.tab {
                GuiTab::Launch => {}
                GuiTab::Plot => self.plot_tab.main_ui(ui, self.data_source.as_mut()),
                GuiTab::Configure => {
                    let changed = self.configure_tab.main_ui(ui, self.data_source.as_mut(), &mut self.settings);
                    if changed {
                        self.data_source.apply_settings(&self.settings);
                        self.plot_tab.apply_settings(&self.settings);
                    }
                }
            }
        });

        // If we have live data coming in, we need to tell egui to repaint.
        // If we don't, we shouldn't.
        if let Some(fps) = self.data_source.minimum_fps().or(self.archive_window_open.then(|| 60)) {
            let t = std::time::Duration::from_millis(1000 / fps);
            ctx.request_repaint_after(t);
        }
    }
}

impl eframe::App for Sam {
    /// Main draw method of the application
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Draw UI
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
        Box::new(|cc| Box::new(Sam::init(&cc.egui_ctx, app_settings, data_source))),
    )?;

    Ok(())
}
