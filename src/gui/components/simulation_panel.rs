use crate::gui::fc_settings::FcSettingsUiExt;
use crate::gui::simulation_settings::SimulationSettingsUiExt;
use crate::gui::Sam;
use crate::simulation::SimulationSettings;

use egui::{CollapsingHeader, Context};

pub fn create_simulation_panel(sam: &mut Sam, ctx: &Context) {
    let old_settings = sam.data_source.simulation_settings().unwrap().clone();

    egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
        ui.set_enabled(!sam.archive_window_open);
        ui.heading("Simulation");
        ui.add_space(20.0);

        CollapsingHeader::new("Simulation Parameters").default_open(true).show(ui, |ui| {
            let settings = sam.data_source.simulation_settings().unwrap();
            settings.ui(ui)
        });

        CollapsingHeader::new("(Simulated) FC Settings").default_open(false).show(ui, |ui| {
            let settings = sam.data_source.simulation_settings().unwrap();
            settings.fc_settings.ui(ui, None)
        });

        ui.add_space(20.0);

        let changed = *sam.data_source.simulation_settings().unwrap() != old_settings;
        let released = false;
        ui.horizontal(|ui| {
            if ui.button("Reset").clicked() {
                *(sam.data_source.simulation_settings().unwrap()) = SimulationSettings::default();
            }

            if ui.button("↻  Rerun").clicked() || (changed && released) {
                sam.data_source.reset();
            }
        });
    });
}
