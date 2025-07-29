//! Main GUI code, included by cli, wasm and android entrypoints

use std::path::PathBuf;
use std::sync::Arc;

use eframe::egui;
use egui::FontFamily::Proportional;
use egui::TextStyle::*;
use egui::{Align, Color32, FontFamily, FontId, Key, Layout, Modifiers, Vec2};

use shared_types::telemetry::*;

pub mod backend;
pub mod frontend;
pub mod system_diagram_components;
pub mod panels;
pub mod settings;
pub mod tabs;
pub mod utils;
pub mod widgets;
pub mod windows; // TODO: make this private (it is public because it has ARCHIVE)

use crate::backend::*;
use crate::frontend::Frontend;
use crate::panels::metric_status_bar::MetricStatusBar;
use crate::panels::*;
use crate::settings::AppSettings;
use crate::tabs::*;
use crate::windows::*;

/// Main state object of our GUI application
pub struct Sam {
    backends: Vec<Backend>,
    frontend: Frontend,
    settings: AppSettings,
    tab: GuiTab,
    plot_tab: PlotTab,
    configure_tab: ConfigureTab,
    archive_window: ArchiveWindow,
}

impl Sam {
    /// Initialize the application, including the state objects for widgets
    /// such as plots and maps.
    pub fn init(ctx: &egui::Context, settings: AppSettings, default_backend: Option<Backend>) -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        let backend = default_backend.unwrap_or(Backend::Udp(UdpBackend::new(ctx)));
        //let backend = default_backend.unwrap_or(Backend::Serial(SerialBackend::new(ctx, settings.lora.clone())));
        #[cfg(target_arch = "wasm32")]
        let backend = default_backend.unwrap_or(Backend::Noop(NoopBackend::default()));

        let mut fonts = egui::FontDefinitions::default();
        let roboto = egui::FontData::from_static(include_bytes!("../assets/fonts/RobotoMono-Regular.ttf"));
        let lato = egui::FontData::from_static(include_bytes!("../assets/fonts/Overpass-Light.ttf"));
        fonts.font_data.insert("RobotoMono".to_owned(), Arc::new(roboto));
        fonts.font_data.insert("Overpass".to_owned(), Arc::new(lato));
        fonts.families.entry(FontFamily::Monospace).or_default().insert(0, "RobotoMono".to_owned());
        fonts.families.entry(FontFamily::Proportional).or_default().insert(0, "Overpass".to_owned());

        ctx.set_fonts(fonts);

        let plot_tab = PlotTab::init(ctx);
        let configure_tab = ConfigureTab::init();

        egui_extras::install_image_loaders(ctx);

        Self {
            backends: vec![backend],
            frontend: Frontend::new(ctx, &settings),
            settings,

            tab: GuiTab::Plot,
            plot_tab,
            configure_tab,

            archive_window: ArchiveWindow::default(),
        }
    }

    fn close_backend(&mut self) {
        self.backends.truncate(1);
    }

    fn open_backend(&mut self, data_source: Backend) {
        self.close_backend();
        self.backends.push(data_source);
    }

    fn backend(&mut self) -> &Backend {
        self.backends.last().unwrap()
    }

    fn backend_mut(&mut self) -> &mut Backend {
        self.backends.last_mut().unwrap()
    }

    fn process_keybinding(&mut self, ctx: &egui::Context) {
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
            if self.backend().flight_mode().unwrap_or_default() == FlightMode::Armed {
                Some(FlightMode::ArmedLaunchImminent)
            } else {
                Some(FlightMode::Armed)
            }
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
            self.backend_mut().send_command(Command::SetFlightMode(fm)).unwrap();
        }

        let alt = ctx.input(|i| i.modifiers) == Modifiers::ALT;
        let key_accel = ctx.input_mut(|i| i.key_down(Key::Plus));
        let key_decel = ctx.input_mut(|i| i.key_down(Key::Minus));

        let thruster_state = if alt && key_accel && key_decel {
            Some(ThrusterValveState::OpenBoth)
        } else if alt && key_accel {
            Some(ThrusterValveState::OpenAccel)
        } else if alt && key_decel {
            Some(ThrusterValveState::OpenDecel)
        } else if (ctx.input_mut(|i| i.key_released(Key::Plus) || i.key_released(Key::Minus)))
            && !key_accel
            && !key_decel
        {
            Some(ThrusterValveState::Closed)
        } else {
            None
        };

        if let Some(s) = thruster_state {
            self.backend_mut().send_command(Command::SetAcsValveState(s)).unwrap();
        }
    }

    pub fn ui(&mut self, ctx: &egui::Context) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();
        #[cfg(feature = "profiling")]
        puffin::GlobalProfiler::lock().new_frame();
        #[cfg(feature = "profiling")]
        puffin_egui::profiler_window(ctx);

        // Redefine text_styles
        let colors = utils::theme::ThemeColors::new(ctx);
        let mut style = (*ctx.style()).clone();
        colors.apply(&mut style);
        style.text_styles.insert(Heading, FontId::new(14.0, Proportional));
        ctx.set_style(style.clone());

        // A window to open archived logs directly in the application
        if let Some(log) = self.archive_window.show_if_open(ctx) {
            self.open_backend(Backend::Log(log));
        }

        self.backend_mut().update(ctx);

        self.process_keybinding(ctx);

        let enabled = !self.archive_window.open;

        // Top menu bar
        // TODO: avoid passing in self here
        MenuBarPanel::show(ctx, self, enabled);

        // If our current backend is a simulation, show a config panel to the left
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        if let Backend::Simulation(sim) = self.backend_mut() {
            #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
            SimulationPanel::show(ctx, sim, enabled);
        }

        // Header containing text indicators and flight mode buttons
        HeaderPanel::show(ctx, self.backend_mut(), enabled);

        let tab = self.tab.clone();

        // Bottom status bar
        egui::TopBottomPanel::bottom("bottombar").min_height(30.0).show(ctx, |ui| {
            #[cfg(feature = "profiling")]
            puffin::profile_function!();

            if self.archive_window.open {
                ui.disable();
            }

            ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                match tab {
                    GuiTab::Launch => {}
                    GuiTab::Plot => self.plot_tab.bottom_bar_ui(ui),
                    GuiTab::Configure => {}
                }

                ui.separator();

                ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                    self.backend_mut().status_bar_ui(ui);
                })
            });
        });

        // Everything else. This has to be called after all the other panels are created.
        let backend = self.backends.last_mut().unwrap();
        let _ = self.frontend.metric_monitor_mut().evaluate_constraints(backend);
        let mut active_constraint_mask = self.frontend.metric_monitor().active_constraint_mask().clone();
        MetricStatusBar::show(ctx, backend, &mut self.frontend, &mut active_constraint_mask);
        *self.frontend.metric_monitor_mut().active_constraint_mask_mut() = active_constraint_mask;
        match tab {
            GuiTab::Launch => egui::CentralPanel::default().show(ctx, |_ui| {}).inner,
            GuiTab::Plot => self.plot_tab.main_ui(ctx, backend, &mut self.settings, &mut self.frontend, enabled),
            GuiTab::Configure => {
                let changed = self.configure_tab.main_ui(ctx, backend, &mut self.settings, enabled);
                if changed {
                    backend.apply_settings(&self.settings);
                }
            }
        }
    }
}

impl eframe::App for Sam {
    /// Main draw method of the application
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Draw UI
        self.ui(ctx);
        self.frontend.popup_manager_mut().update();
    }
}

/// The main entrypoint for the egui interface.
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub fn main(log_file: Option<PathBuf>, simulate: Option<Option<String>>) -> Result<(), Box<dyn std::error::Error>> {
    let app_settings = AppSettings::load().ok().unwrap_or_default();

    let data_source: Option<Backend> = if let Some(log) = simulate {
        if let Some(log) = log {
            Some(Backend::Simulation(SimulationBackend::replicate(log)))
        } else {
            Some(Backend::Simulation(SimulationBackend::simulate()))
        }
    } else if let Some(path) = log_file {
        Some(Backend::Log(LogFileBackend::new(path)?))
    } else {
        None
    };

    #[cfg(feature = "profiling")]
    puffin::set_scopes_on(true);

    eframe::run_native(
        "Sam Ground Station",
        eframe::NativeOptions {
            viewport: egui::viewport::ViewportBuilder {
                inner_size: Some(Vec2::new(1920.0, 1080.0)),
                ..Default::default()
            },
            ..Default::default()
        },
        Box::new(|cc| Ok(Box::new(Sam::init(&cc.egui_ctx, app_settings, data_source)))),
    )?;

    Ok(())
}
