use egui::CollapsingHeader;

use archive::{ArchivedLog, ARCHIVED_LOGS};
use telemetry::{FLASH_SCHEMA, LORA_SCHEMA, USB_SCHEMA};

use crate::backend::{BackendVariant, SimulationBackend};
use crate::widgets::fc_settings::FcSettingsUiExt;

pub struct SimulationPanel {}

impl SimulationPanel {
    pub fn show(ctx: &egui::Context, backend: &mut SimulationBackend, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let old_settings = backend.settings.clone();
        let old_fc_settings = backend.fc_settings.clone();
        let old_schema = backend.telemetry_schema;

        egui::SidePanel::left("sim").min_width(300.0).max_width(500.0).resizable(true).show(ctx, |ui| {
            if !enabled {
                ui.disable();
            }

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
                                egui::ComboBox::from_id_salt("replication_log")
                                    .selected_text(
                                        backend
                                            .replicated_log_id
                                            .as_ref()
                                            .and_then(|id| ArchivedLog::find(&id).map(|log| log.to_string()))
                                            .unwrap_or("None".into()),
                                    )
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut backend.replicated_log_id, None, "None");
                                        for log in &ARCHIVED_LOGS {
                                            if log.flash_log_url.is_none() {
                                                continue;
                                            }
                                            ui.selectable_value(
                                                &mut backend.replicated_log_id,
                                                Some(log.id.to_string()),
                                                log.to_string(),
                                            );
                                        }
                                    });
                                ui.end_row();
                            });

                        ui.add_space(10.0);

                        // Disable other settings if we are using a source log
                        ui.add_enabled(backend.replicated_log_id.is_none(), &mut backend.settings);
                    })
                });

                ui.add_space(10.0);

                CollapsingHeader::new("(Simulated) FC Settings")
                    .default_open(false)
                    .show(ui, |ui| backend.fc_settings.ui(ui, None, true));

                ui.add_space(10.0);

                egui::Grid::new("telemetry_settings_grid")
                    .num_columns(2)
                    .min_col_width(0.25 * ui.available_width())
                    .spacing([40.0, 4.0])
                    .striped(true)
                    .show(ui, |ui| {
                        ui.label("Telemetry Schema");
                        egui::ComboBox::from_id_salt("telemetry_schema")
                            .selected_text(match backend.telemetry_schema.clone() {
                                LORA_SCHEMA => "LoRa",
                                USB_SCHEMA => "USB",
                                FLASH_SCHEMA => "Flash",
                                _ => "",
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut backend.telemetry_schema, &LORA_SCHEMA, "LoRa");
                                ui.selectable_value(&mut backend.telemetry_schema, &USB_SCHEMA, "USB");
                                ui.selectable_value(&mut backend.telemetry_schema, &FLASH_SCHEMA, "Flash");
                            });
                        ui.end_row();
                    });

                ui.add_space(10.0);

                let changed = backend.settings != old_settings
                    || backend.fc_settings != old_fc_settings
                    || backend.telemetry_schema != old_schema;
                ui.horizontal(|ui| {
                    if ui.button("Reset").clicked() {
                        backend.settings = galadriel::SimulationSettings::default();
                    }

                    if ui.button("↻  Rerun").clicked() || changed {
                        backend.reset();
                    }
                });
            });
        });
    }
}
