use crate::gui::tabs::GuiTab;
use crate::gui::Sam;

use egui::Context;

pub fn create_body(sam: &mut Sam, ctx: &Context) {
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.set_width(ui.available_width());
        ui.set_height(ui.available_height());
        ui.set_enabled(!sam.archive_window_open);

        match sam.tab {
            GuiTab::Launch => {}
            GuiTab::Plot => sam.plot_tab.main_ui(ui, sam.data_source.as_mut()),
            GuiTab::Configure => {
                let changed = sam.configure_tab.main_ui(ui, sam.data_source.as_mut(), &mut sam.settings);
                if changed {
                    sam.data_source.apply_settings(&sam.settings);
                    sam.plot_tab.apply_settings(&sam.settings);
                }
            }
        }
    });
}
