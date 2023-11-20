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
use egui::TextStyle::*;
use egui::{Align, Color32, FontFamily, FontId, Key, Layout, Modifiers, Vec2};

use futures::StreamExt;
use log::*;

use mithril::telemetry::*;

mod panels;
mod fc_settings;
mod map;
mod maxi_grid;
mod misc;
mod plot;
mod simulation_settings;
mod tabs;
mod theme;
mod top_bar;
pub mod windows; // TODO: make this private (it is public because it has ARCHIVE)

use crate::data_source::*;
use crate::gui::panels::*;
use crate::gui::tabs::*;
use crate::gui::theme::*;
use crate::gui::windows::archive::open_archive_window;
use crate::settings::AppSettings;

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

        // A window to open archived logs directly in the application
        let mut archive_open = self.archive_window_open; // necessary to avoid mutably borrowing self
        open_archive_window(ctx, &mut archive_open, self);
        self.archive_window_open = archive_open;

        // Top menu bar
        // TODO: avoid passing in self here
        MenuBarPanel::show(ctx, self, !self.archive_window_open);

        // If our current data source is a simulation, show a config panel to the left
        if let Some(sim) = self.data_source.as_any_mut().downcast_mut::<SimulationDataSource>() {
            SimulationPanel::show(ctx, sim, !self.archive_window_open);
        }

        // Header containing text indicators and flight mode buttons
        HeaderPanel::show(ctx, self.data_source.as_mut(), !self.archive_window_open);

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
                    match self.tab {
                        GuiTab::Launch => {}
                        GuiTab::Plot => self.plot_tab.bottom_bar_ui(ui, self.data_source.as_mut()),
                        GuiTab::Configure => {}
                    }
                });
            });
        });

        // Everything else. This has to be called after all the other panels are created.
        egui::CentralPanel::default().show(ctx, |ui| {
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
