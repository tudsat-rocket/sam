use egui::CollapsingHeader;

use crate::data_source::DataSource;
use crate::gui::fc_settings::FcSettingsUiExt;
use crate::gui::simulation_settings::SimulationSettingsUiExt;
use crate::simulation::SimulationSettings;

pub struct SimulationPanel {}

impl SimulationPanel {
    pub fn show(ctx: &egui::Context, data_source: &mut dyn DataSource, enabled: bool) {
        let old_settings = data_source.simulation_settings().unwrap().clone();

        egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
            ui.set_enabled(enabled);
            ui.heading("Simulation");
            ui.add_space(20.0);

            CollapsingHeader::new("Simulation Parameters").default_open(true).show(ui, |ui| {
                let settings = data_source.simulation_settings().unwrap();
                settings.ui(ui)
            });

            CollapsingHeader::new("(Simulated) FC Settings").default_open(false).show(ui, |ui| {
                let settings = data_source.simulation_settings().unwrap();
                settings.fc_settings.ui(ui, None)
            });

            ui.add_space(20.0);

            let changed = *data_source.simulation_settings().unwrap() != old_settings;
            let released = false;
            ui.horizontal(|ui| {
                if ui.button("Reset").clicked() {
                    *(data_source.simulation_settings().unwrap()) = SimulationSettings::default();
                }

                if ui.button("↻  Rerun").clicked() || (changed && released) {
                    data_source.reset();
                }
            });
        });
    }
}
