use crate::gui::tabs::GuiTab;
use crate::gui::Sam;
use egui::{Align, Context, Layout};

pub fn create_bottom_status_bar(sam: &mut Sam, ctx: &Context) {
    egui::TopBottomPanel::bottom("bottombar").min_height(30.0).show(ctx, |ui| {
        ui.set_enabled(!sam.archive_window_open);
        ui.horizontal_centered(|ui| {
            // Give the data source some space on the left ...
            ui.horizontal_centered(|ui| {
                ui.set_width(ui.available_width() / 2.0);
                sam.data_source.status_bar_ui(ui);
            });

            // ... and the current tab some space on the right.
            ui.allocate_ui_with_layout(ui.available_size(), Layout::right_to_left(Align::Center), |ui| {
                #[cfg(not(target_arch = "wasm32"))]
                ui.add_enabled_ui(!sam.data_source.is_log_file(), |ui| {
                    if ui.button("⏮  Reset").clicked() {
                        sam.data_source.reset();
                    }
                });

                #[cfg(not(target_arch = "wasm32"))]
                ui.separator();

                match sam.tab {
                    GuiTab::Launch => {}
                    GuiTab::Plot => sam.plot_tab.bottom_bar_ui(ui, sam.data_source.as_mut()),
                    GuiTab::Configure => {}
                }
            });
        });
    });
}
