use egui::CollapsingHeader;

use crate::data_source::{DataSource, SimulationDataSource, SimulationSettings};
use crate::gui::fc_settings::FcSettingsUiExt;
use crate::gui::windows::ArchivedLog;

pub struct SimulationPanel {}

impl SimulationPanel {
    pub fn show(ctx: &egui::Context, data_source: &mut SimulationDataSource, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let old_settings = data_source.settings.clone();

        egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
            ui.set_enabled(enabled);
            ui.add_space(10.0);
            ui.heading("Simulation");
            ui.add_space(10.0);

            egui::ScrollArea::vertical().show(ui, |ui| {
                CollapsingHeader::new("Simulation Parameters").default_open(true).show(ui, |ui| {
                    ui.vertical(|ui| {
                        egui::Grid::new("simulation_settings_grid")
                            .num_columns(2)
                            .min_col_width(0.25 * ui.available_width())
                            .spacing([40.0, 4.0])
                            .striped(true)
                            .show(ui, |ui| {
                                ui.label("Replication Log");
                                egui::ComboBox::from_id_source("replication_log")
                                    .selected_text(data_source.settings.replicated_log.map(|l| l.to_string()).unwrap_or("None".into()))
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut data_source.settings.replicated_log, None, "None");
                                        for log in &ArchivedLog::all() {
                                            if log.flash_log_url().is_none() {
                                                continue;
                                            }
                                            ui.selectable_value(&mut data_source.settings.replicated_log, Some(*log), log.to_string());
                                        }
                                    });
                                ui.end_row();
                            });

                        // Disable other settings if we are using a source log
                        ui.set_enabled(data_source.settings.replicated_log.is_none());
                        ui.add_space(10.0);

                        ui.add(&mut data_source.settings.galadriel);
                    })
                });

                ui.add_space(10.0);

                CollapsingHeader::new("(Simulated) FC Settings").default_open(false).show(ui, |ui| {
                    data_source.settings.fc_settings.ui(ui, None, true)
                });

                ui.add_space(10.0);

                let changed = data_source.settings != old_settings;
                ui.horizontal(|ui| {
                    if ui.button("Reset").clicked() {
                        data_source.settings = SimulationSettings::default();
                    }

                    if ui.button("↻  Rerun").clicked() || changed {
                        data_source.reset();
                    }
                });
            });
        });
    }
}
