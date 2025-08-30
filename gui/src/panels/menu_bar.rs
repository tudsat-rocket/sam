use eframe::egui;
use egui::{Align, Layout, Vec2};

use crate::tabs::GuiTab;
use crate::utils::file::*;
use crate::{Backend, Sam};

#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
use crate::backend::SimulationBackend;

pub struct MenuBarPanel {}

impl MenuBarPanel {
    pub fn show(ctx: &egui::Context, sam: &mut Sam, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let backend = sam.backend();

        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        let data_source_is_sim = match backend {
            Backend::Simulation(_) => true,
            _ => false,
        };
        #[cfg(any(target_arch = "wasm32", target_os = "android"))]
        let data_source_is_sim = false;

        let data_source_is_log = match backend {
            Backend::Log(_) => true,
            _ => false,
        };

        egui::TopBottomPanel::top("menubar").min_height(30.0).max_height(30.0).show(ctx, |ui| {
            if !enabled {
                ui.disable();
            }

            ui.horizontal_centered(|ui| {
                let image = if ui.style().visuals.dark_mode {
                    egui::Image::new(egui::include_image!("../../assets/logo_dark_mode.png"))
                } else {
                    egui::Image::new(egui::include_image!("../../assets/logo_light_mode.png"))
                };
                ui.add(image.max_size(Vec2::new(ui.available_width(), 20.0)));

                ui.separator();
                egui::widgets::global_theme_preference_switch(ui);
                ui.separator();

                ui.selectable_value(&mut sam.tab, GuiTab::Launch, "🚀 Launch (F1)");
                ui.selectable_value(&mut sam.tab, GuiTab::Plot, "📈 Plot (F2)");
                ui.selectable_value(&mut sam.tab, GuiTab::Configure, "⚙  Configure (F3)");

                ui.separator();

                // Opening files manually is not available on web assembly
                #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
                if ui.selectable_label(false, "🗁  Open Log File").clicked() {
                    if let Some(log) = open_log_file() {
                        println!("Opened logfile successfully");
                        sam.open_backend(Backend::Log(log));
                    }
                }

                // Toggle archive panel
                ui.toggle_value(&mut sam.archive_window.open, "🗄 Flight Archive");

                // Toggle archive panel
                #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
                if ui.selectable_label(data_source_is_sim, "💻 Simulate").clicked() {
                    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
                    sam.open_backend(Backend::Simulation(SimulationBackend::default()));
                }

                // Show a button to the right to close the current log/simulation and go back to
                // live view
                ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                    if (data_source_is_log || data_source_is_sim) && ui.button("❌").clicked() {
                        sam.close_backend();
                    }
                });
            });
        });
    }
}
