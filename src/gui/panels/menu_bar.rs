use crate::data_source::SimulationDataSource;
use crate::file::open_log_file;
use crate::gui::tabs::GuiTab;
use crate::gui::Sam;

use eframe::egui;
use egui::{Align, Context, Layout, Vec2};

pub fn create_top_menu_bar(sam: &mut Sam, ctx: &Context) {
    egui::TopBottomPanel::top("menubar").min_height(30.0).max_height(30.0).show(ctx, |ui| {
        ui.set_enabled(!sam.archive_window_open);
        ui.horizontal_centered(|ui| {
            let image = if ui.style().visuals.dark_mode {
                egui::Image::new(egui::include_image!("../../../assets/logo_dark_mode.png"))
            } else {
                egui::Image::new(egui::include_image!("../../../assets/logo_light_mode.png"))
            };
            ui.add(image.max_size(Vec2::new(ui.available_width(), 20.0)));

            ui.separator();
            egui::widgets::global_dark_light_mode_switch(ui);
            ui.separator();

            ui.selectable_value(&mut sam.tab, GuiTab::Launch, "🚀 Launch (F1)");
            ui.selectable_value(&mut sam.tab, GuiTab::Plot, "📈 Plot (F2)");
            ui.selectable_value(&mut sam.tab, GuiTab::Configure, "⚙  Configure (F3)");

            ui.separator();

            // Opening files manually is not available on web assembly
            #[cfg(target_arch = "x86_64")]
            if ui.selectable_label(false, "🗁  Open Log File").clicked() {
                if let Some(data_source) = open_log_file() {
                    sam.open_log_file(data_source);
                }
            }

            // Toggle archive panel
            ui.toggle_value(&mut sam.archive_window_open, "🗄 Flight Archive");

            // Toggle archive panel
            if ui.selectable_label(sam.data_source.simulation_settings().is_some(), "💻 Simulate").clicked() {
                sam.data_source = Box::new(SimulationDataSource::default());
            }

            // Show a button to the right to close the current log/simulation and go back to
            // live view
            ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                if sam.data_source.is_log_file() || sam.data_source.simulation_settings().is_some() {
                    if ui.button("❌").clicked() {
                        sam.close_data_source();
                    }
                }
            });
        });
    });
}
