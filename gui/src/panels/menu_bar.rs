use eframe::egui;
use egui::{Ui, Vec2};

use crate::backend::{BackendDiscriminants, SerialBackend, UdpBackend};
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

                // Create new backends
                let mut new_backend: Option<BackendDiscriminants> = None;
                #[cfg(not(target_arch = "wasm32"))]
                egui::ComboBox::new(egui::Id::new("CreateNewBackend"), "").selected_text("Create Backend").show_ui(
                    ui,
                    |ui| {
                        if !sam.has_active_backend_type(BackendDiscriminants::Serial) {
                            ui.selectable_value(&mut new_backend, Some(BackendDiscriminants::Serial), "Serial");
                        }
                        if !sam.has_active_backend_type(BackendDiscriminants::Udp) {
                            ui.selectable_value(&mut new_backend, Some(BackendDiscriminants::Udp), "UDP");
                        }
                        #[cfg(not(target_os = "android"))]
                        ui.selectable_value(&mut new_backend, Some(BackendDiscriminants::Simulation), "Simulation");
                    },
                );
                if let Some(to_create) = new_backend {
                    match to_create {
                        BackendDiscriminants::Serial => {
                            sam.open_backend(Backend::Serial(SerialBackend::new(ui.ctx(), sam.settings.lora.clone())))
                        }
                        BackendDiscriminants::Udp => sam.open_backend(Backend::Udp(UdpBackend::new(ui.ctx()))),
                        BackendDiscriminants::Simulation => {
                            sam.open_backend(Backend::Simulation(SimulationBackend::default()))
                        }
                        _ => unreachable!(),
                    }
                }
            });
        });
    }
}
