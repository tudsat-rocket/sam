use egui::{DragValue, InnerResponse, Ui};

use crate::simulation::SimulationSettings;

use super::ARCHIVE;

pub trait SimulationSettingsUiExt {
    fn ui(&mut self, ui: &mut Ui) -> InnerResponse<()>;
}

impl SimulationSettingsUiExt for SimulationSettings {
    fn ui(&mut self, ui: &mut Ui) -> InnerResponse<()> {
        egui::Grid::new("simulation_settings_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                #[cfg(not(target_arch = "wasm32"))]
                {
                    ui.label("Replication Log");
                    egui::ComboBox::from_id_source("replication_log")
                        .selected_text(self.replication_log_index.map(|i| ARCHIVE[i].0).unwrap_or("None"))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.replication_log_index, None, "None");
                            for (i, (name, _, f)) in ARCHIVE.iter().enumerate() {
                                if f.is_none() {
                                    continue;
                                }
                                ui.selectable_value(&mut self.replication_log_index, Some(i), *name);
                            }
                        });
                    ui.end_row();
                }

                // Disable other settings if we are using a source log
                ui.set_enabled(self.replication_log_index.is_none());

                ui.label("Launch Parameters");
                ui.horizontal(|ui| {
                    ui.weak("Altitude");
                    ui.add(
                        DragValue::new(&mut self.altitude_ground).suffix(" m").speed(1.0).clamp_range(-100.0..=2000.0),
                    );

                    ui.weak("Angle");
                    ui.add(DragValue::new(&mut self.launch_angle).suffix(" °").speed(0.5).clamp_range(0.5..=90.0));

                    ui.weak("Azimuth");
                    ui.add(
                        DragValue::new(&mut self.launch_azimuth).suffix(" °").speed(1.0).clamp_range(-360.0..=359.0),
                    );
                });
                ui.end_row();

                ui.label("Launch Location");
                ui.horizontal(|ui| {
                    ui.weak("Latitude");
                    ui.add(
                        DragValue::new(&mut self.launch_latitude)
                            .suffix(" °")
                            .speed(0.000001)
                            .clamp_range(-90.0..=90.0),
                    );

                    ui.weak("Longitude");
                    ui.add(
                        DragValue::new(&mut self.launch_longitude)
                            .suffix(" °")
                            .speed(0.000001)
                            .clamp_range(-180.0..=180.0),
                    );
                });
                ui.end_row();

                ui.label("Sim Duration");
                ui.horizontal(|ui| {
                    ui.add(
                        DragValue::new(&mut self.sim_duration).suffix(" ms").speed(100).clamp_range(10000..=1000000),
                    );

                    ui.weak("with a start delay of");
                    ui.add(
                        DragValue::new(&mut self.sim_start_delay).suffix(" ms").speed(100).clamp_range(5001..=50000),
                    );
                });
                ui.end_row();

                ui.label("Thrust");
                ui.horizontal(|ui| {
                    ui.add(DragValue::new(&mut self.thrust).suffix(" m/s²").speed(1.0).clamp_range(10.0..=1000.0));

                    ui.weak("for");
                    ui.add(DragValue::new(&mut self.thrust_duration).suffix(" ms").speed(10).clamp_range(10..=60000));
                });
                ui.end_row();

                ui.label("Drag Terms");
                ui.horizontal(|ui| {
                    ui.weak("Flight");
                    ui.add(DragValue::new(&mut self.drag_flight).suffix("").speed(0.001).clamp_range(0.0..=500.0));

                    ui.weak("Drogue");
                    ui.add(DragValue::new(&mut self.drag_drogue).suffix("").speed(0.001).clamp_range(0.0..=500.0));

                    ui.weak("Main");
                    ui.add(DragValue::new(&mut self.drag_main).suffix("").speed(0.01).clamp_range(0.0..=500.0));
                });
                ui.end_row();

                ui.label("Barometer Anomalies");
                ui.horizontal(|ui| {
                    ui.weak("Value of");
                    ui.add(
                        DragValue::new(&mut self.barometer_anomaly_value)
                            .suffix("m")
                            .speed(0.1)
                            .clamp_range(-10000.0..=20000.0),
                    );

                    ui.weak("with p=");
                    ui.add(
                        DragValue::new(&mut self.barometer_anomaly_probability)
                            .suffix("")
                            .speed(0.0001)
                            .clamp_range(0.0..=1.0),
                    );

                    ui.weak("starting after");
                    ui.add(
                        DragValue::new(&mut self.barometer_anomaly_delay)
                            .suffix("ms")
                            .speed(1)
                            .clamp_range(0.0..=1000000.0),
                    );
                });
                ui.end_row();

                //pub std_dev_gyroscope: f32,
                //pub std_dev_accelerometer1: f32,
                //pub std_dev_accelerometer2: f32,
                //pub std_dev_magnetometer: f32,
                //pub std_dev_barometer: f32,
            })
    }
}
