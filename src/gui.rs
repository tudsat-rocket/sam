//! Main GUI code

use std::ops::DerefMut;
use std::path::PathBuf;

use eframe::egui;
use egui::FontFamily::Proportional;
use egui::TextStyle::*;
use egui::{Align, Color32, FontFamily, FontId, Key, Layout, Modifiers, Vec2};

use mithril::telemetry::*;

mod panels;
mod fc_settings;
mod map;
mod plot;
mod simulation_settings;
pub mod tabs;
mod theme;
mod top_bar;
pub mod windows; // TODO: make this private (it is public because it has ARCHIVE)

use crate::data_source::*;
use crate::gui::panels::*;
use crate::gui::tabs::*;
use crate::gui::theme::*;
use crate::gui::windows::*;
use crate::settings::AppSettings;

/// Main state object of our GUI application
pub struct Sam {
    settings: AppSettings,
    data_sources: Vec<Box<dyn DataSource>>,
    tab: GuiTab,
    plot_tab: PlotTab,
    configure_tab: ConfigureTab,
    archive_window: ArchiveWindow,
}

impl Sam {
    /// Initialize the application, including the state objects for widgets
    /// such as plots and maps.
    pub fn init(ctx: &egui::Context, settings: AppSettings, default_data_source: Option<Box<dyn DataSource>>) -> Self {
        let default_data_source = default_data_source
            .unwrap_or(Box::new(SerialDataSource::new(ctx, settings.lora.clone())));

        let mut fonts = egui::FontDefinitions::default();
        let roboto = egui::FontData::from_static(include_bytes!("../assets/fonts/RobotoMono-Regular.ttf"));
        let lato = egui::FontData::from_static(include_bytes!("../assets/fonts/Overpass-Light.ttf"));
        fonts.font_data.insert("RobotoMono".to_owned(), roboto);
        fonts.font_data.insert("Overpass".to_owned(), lato);
        fonts.families.entry(FontFamily::Monospace).or_default().insert(0, "RobotoMono".to_owned());
        fonts.families.entry(FontFamily::Proportional).or_default().insert(0, "Overpass".to_owned());

        ctx.set_fonts(fonts);

        let plot_tab = PlotTab::init(ctx, &settings);
        let configure_tab = ConfigureTab::init();

        egui_extras::install_image_loaders(ctx);

        Self {
            settings,
            data_sources: vec![default_data_source],

            tab: GuiTab::Plot,
            plot_tab,
            configure_tab,

            archive_window: ArchiveWindow::default(),
        }
    }

    /// Closes the currently opened data source
    fn close_data_source(&mut self) {
        // TODO: extend this to arbitrarily many data sources?
        self.data_sources.truncate(1);
    }

    fn open_data_source(&mut self, data_source: Box<dyn DataSource>) {
        self.close_data_source();
        self.data_sources.push(data_source);
    }

    fn data_source(&mut self) -> &Box<dyn DataSource> {
        self.data_sources.last().unwrap()
    }

    fn data_source_mut(&mut self) -> &mut Box<dyn DataSource> {
        self.data_sources.last_mut().unwrap()
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();
        #[cfg(feature = "profiling")]
        puffin::GlobalProfiler::lock().new_frame();
        #[cfg(feature = "profiling")]
        puffin_egui::profiler_window(ctx);

        self.data_source_mut().update(ctx);

        // TODO: only send this if we know it's not a ground station?
        if self.data_source_mut().fc_settings().is_none() && self.data_source().vehicle_states().next().is_some() {
            self.data_source_mut().send(UplinkMessage::ReadSettings).unwrap();
        }

        // Check for keyboard inputs for tab and flight mode changes
        if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F1)) {
            self.tab = GuiTab::Launch;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F2)) {
            self.tab = GuiTab::Plot;
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F3)) {
            self.tab = GuiTab::Configure;
        }

        let shortcut_mode = if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F4)) {
            Some(FlightMode::Idle)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F5)) {
            Some(FlightMode::HardwareArmed)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F6)) {
            Some(FlightMode::Armed)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F7)) {
            Some(FlightMode::Burn)
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::SHIFT, Key::F8)) {
            Some(FlightMode::Coast)
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
            self.data_source_mut().send_command(Command::SetFlightMode(fm)).unwrap();
        }

        // Redefine text_styles
        let colors = ThemeColors::new(ctx);
        let mut style = (*ctx.style()).clone();
        colors.apply(&mut style);
        style.text_styles.insert(Heading, FontId::new(14.0, Proportional));
        ctx.set_style(style.clone());

        // A window to open archived logs directly in the application
        if let Some(log) = self.archive_window.show_if_open(ctx) {
            self.open_data_source(Box::new(log));
        }

        let enabled = !self.archive_window.open;

        // Top menu bar
        // TODO: avoid passing in self here
        MenuBarPanel::show(ctx, self, enabled);

        let data_source = self.data_sources.last_mut().unwrap();

        // If our current data source is a simulation, show a config panel to the left
        if let Some(sim) = data_source.as_any_mut().downcast_mut::<SimulationDataSource>() {
            SimulationPanel::show(ctx, sim, enabled);
        }

        // Header containing text indicators and flight mode buttons
        HeaderPanel::show(ctx, data_source.deref_mut(), enabled);

        // Bottom status bar
        egui::TopBottomPanel::bottom("bottombar").min_height(30.0).show(ctx, |ui| {
            #[cfg(feature = "profiling")]
            puffin::profile_function!();

            ui.set_enabled(!self.archive_window.open);
            ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                match self.tab {
                    GuiTab::Launch => {}
                    GuiTab::Plot => self.plot_tab.bottom_bar_ui(ui, data_source.deref_mut()),
                    GuiTab::Configure => {}
                }

                ui.separator();

                ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                    data_source.status_bar_ui(ui);
                })
            });
        });

        // Everything else. This has to be called after all the other panels are created.
        match self.tab {
            GuiTab::Launch => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.set_enabled(enabled)
                }).inner
            }
            GuiTab::Plot => self.plot_tab.main_ui(ctx, data_source.deref_mut(), &mut self.settings, enabled),
            GuiTab::Configure => {
                let changed = self.configure_tab.main_ui(ctx, data_source.deref_mut(), &mut self.settings, enabled);
                if changed {
                    data_source.apply_settings(&self.settings);
                }
            }
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
    let app_settings = AppSettings::load().ok().unwrap_or_default();

    let data_source: Option<Box<dyn DataSource>> = if let Some(path) = log_file {
        Some(Box::new(LogFileDataSource::new(path)?))
    } else {
        None
    };

    #[cfg(feature = "profiling")]
    puffin::set_scopes_on(true);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions::default(),
        Box::new(|cc| Box::new(Sam::init(&cc.egui_ctx, app_settings, data_source))),
    )?;

    Ok(())
}
