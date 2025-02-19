use std::str::FromStr;

use egui::{Align, DragValue, InnerResponse, Layout, RichText, TextEdit, Ui};

use shared_types::settings::*;
use shared_types::telemetry::TelemetryDataRate;

use crate::settings::AppSettings;

pub trait FcSettingsUiExt {
    fn ui(&mut self, ui: &mut Ui, app_settings: Option<&AppSettings>, sim: bool) -> InnerResponse<()>;
}

impl FcSettingsUiExt for Settings {
    fn ui(&mut self, ui: &mut Ui, app_settings: Option<&AppSettings>, sim: bool) -> InnerResponse<()> {
        egui::Grid::new("fc_settings_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                if !sim {
                    ui.label("Identifier");
                    let mut ident: String = self.identifier.to_string();
                    ui.add_sized(ui.available_size(), TextEdit::singleline(&mut ident));
                    self.identifier = heapless::String::from_str(&ident).unwrap_or_default();
                    ui.end_row();

                    ui.label("LoRa channel selection (500kHz BW)");
                    ui.vertical(|ui| {
                        ui.horizontal(|ui| {
                            ui.toggle_value(&mut self.lora.channels[0], RichText::new("863.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[1], RichText::new("863.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[2], RichText::new("864.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[3], RichText::new("864.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[4], RichText::new("865.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[5], RichText::new("865.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[6], RichText::new("866.25").monospace().size(10.0));
                            ui.label(RichText::new("MHz").weak().size(10.0));
                        });
                        ui.horizontal(|ui| {
                            ui.toggle_value(&mut self.lora.channels[7], RichText::new("866.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[8], RichText::new("867.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[9], RichText::new("867.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[10], RichText::new("868.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[11], RichText::new("868.75").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[12], RichText::new("869.25").monospace().size(10.0));
                            ui.toggle_value(&mut self.lora.channels[13], RichText::new("869.75").monospace().size(10.0));
                            ui.label(RichText::new("MHz").weak().size(10.0));
                        });
                    });
                    ui.end_row();

                    ui.label("LoRa binding phrase");
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if let Some(app_settings) = app_settings {
                            if ui.button("➡ Copy from GCS").clicked() {
                                self.lora.binding_phrase = app_settings.lora.binding_phrase.clone();
                            }
                        }

                        let mut binding_phrase: String = self.lora.binding_phrase.to_string();
                        ui.add_sized(ui.available_size(), TextEdit::singleline(&mut binding_phrase));
                        self.lora.binding_phrase = heapless::String::from_str(&binding_phrase).unwrap_or_default();
                    });
                    ui.end_row();

                    ui.label("LoRa uplink key");
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if let Some(app_settings) = app_settings {
                            if ui.button("➡ Copy from GCS").clicked() {
                                self.lora.authentication_key = app_settings.lora.authentication_key;
                            }
                        }

                        #[cfg(not(target_arch = "wasm32"))]
                        if ui.button("🔃Rekey").clicked() {
                            self.lora.authentication_key = rand::random();
                        }

                        ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                            ui.monospace(format!("{:032x}", self.lora.authentication_key));
                        });
                    });
                    ui.end_row();

                    ui.label("Default Data Rate");
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.default_data_rate, TelemetryDataRate::Low, "Low");
                        ui.selectable_value(&mut self.default_data_rate, TelemetryDataRate::High, "High");
                        ui.label(
                            RichText::new("(20Hz vs 40Hz; higher data incl. raw IMU values, may interfere w/ GPS)")
                                .weak()
                                .size(10.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Gyroscope cal. offsets");
                    ui.horizontal(|ui| {
                        ui.weak("X");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.gyro_offset.x).suffix(" °/s").speed(0.1).range(-2000.0..=2000.0),
                        );

                        ui.weak("Y");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.gyro_offset.y).suffix(" °/s").speed(0.1).range(-2000.0..=2000.0),
                        );

                        ui.weak("Z");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.gyro_offset.z).suffix(" °/s").speed(0.1).range(-2000.0..=2000.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Main accelerometer cal. offsets");
                    ui.horizontal(|ui| {
                        ui.weak("X");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc_offset.x)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-2000.0..=2000.0),
                        );

                        ui.weak("Y");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc_offset.y)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-2000.0..=2000.0),
                        );

                        ui.weak("Z");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc_offset.z)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-2000.0..=2000.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Backup accelerometer cal. offsets");
                    ui.horizontal(|ui| {
                        ui.weak("X");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc2_offset.x)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-4000.0..=4000.0),
                        );

                        ui.weak("Y");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc2_offset.y)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-4000.0..=4000.0),
                        );

                        ui.weak("Z");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.acc2_offset.z)
                                .suffix(" m/s²")
                                .speed(0.01)
                                .range(-4000.0..=4000.0),
                        );
                    });
                    ui.end_row();

                    ui.label("Magnetometer cal. offsets");
                    ui.horizontal(|ui| {
                        ui.weak("X");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.mag_offset.x).suffix(" µT").speed(0.1).range(-2000.0..=2000.0),
                        );

                        ui.weak("Y");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.mag_offset.y).suffix(" µT").speed(0.1).range(-2000.0..=2000.0),
                        );

                        ui.weak("Z");
                        ui.add_sized(
                            [100.0, ui.available_height()],
                            DragValue::new(&mut self.mag_offset.z).suffix(" µT").speed(0.1).range(-2000.0..=2000.0),
                        );
                    });
                    ui.end_row();
                }

                ui.label("Mahony gains");
                ui.horizontal(|ui| {
                    ui.weak("kP");
                    ui.add(DragValue::new(&mut self.mahony_kp).speed(0.001).range(0.0..=10.0));
                    ui.weak(" kI");
                    ui.add(DragValue::new(&mut self.mahony_ki).speed(0.001).range(0.0..=10.0));
                    ui.weak(", (kP");
                    ui.add(DragValue::new(&mut self.mahony_kp_ascent).speed(0.001).range(0.0..=10.0));
                    ui.weak(" kI");
                    ui.add(DragValue::new(&mut self.mahony_ki_ascent).speed(0.001).range(0.0..=10.0));
                    ui.weak("during ascent)")

                });
                ui.end_row();

                ui.label("Kalman std devs.");
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.weak("accelerometer");
                        ui.add(DragValue::new(&mut self.std_dev_accelerometer).speed(0.001).range(0.0..=100.0));

                        ui.weak(" barometer");
                        ui.add(DragValue::new(&mut self.std_dev_barometer).speed(0.001).range(0.0..=100.0));

                        ui.weak(" process");
                        ui.add(DragValue::new(&mut self.std_dev_process).speed(0.001).range(0.0..=100.0));
                    });
                    ui.horizontal(|ui| {
                        ui.weak("(barometer");
                        ui.add(DragValue::new(&mut self.std_dev_barometer_transsonic).speed(0.001).range(0.0..=999_999.0));
                        ui.weak("when transsonic)");
                    });
                });
                ui.end_row();

                ui.label("Takeoff detection acceleration");
                ui.horizontal(|ui| {
                    ui.weak("at least");
                    ui.add(
                        DragValue::new(&mut self.min_takeoff_acc).suffix(" m/s²").speed(0.1).range(0.0..=1000.0),
                    );
                    ui.weak("for");
                    ui.add(DragValue::new(&mut self.min_takeoff_acc_time).suffix(" ms").speed(1).range(0..=1000));
                });
                ui.end_row();

                ui.label("Apogee drogue deployment");
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.weak("between");
                        ui.add(
                            DragValue::new(&mut self.min_time_to_apogee).suffix(" ms").speed(1).range(0..=1000000),
                        );
                        ui.weak("and");
                        ui.add(
                            DragValue::new(&mut self.max_time_to_apogee).suffix(" ms").speed(1).range(0..=1000000),
                        );
                        ui.weak("post-launch,");
                    });
                    ui.horizontal(|ui| {
                        ui.weak("after falling for");
                        ui.add(
                            DragValue::new(&mut self.apogee_min_falling_time).suffix(" ms").speed(1).range(0..=10000),
                        );
                    });
                });
                ui.end_row();

                ui.label("Main deployment");
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut self.main_output_mode, MainOutputMode::Never, "never");
                        ui.selectable_value(&mut self.main_output_mode, MainOutputMode::AtApogee, "at apogee");
                        ui.selectable_value(
                            &mut self.main_output_mode,
                            MainOutputMode::BelowAltitude,
                            "below altitude of ",
                        );
                        ui.add(
                            DragValue::new(&mut self.main_output_deployment_altitude)
                                .suffix(" m")
                                .speed(0.1)
                                .range(0.0..=10000.0),
                        );
                    });
                    ui.horizontal(|ui| {
                        ui.weak("and at least");
                        ui.add(DragValue::new(&mut self.min_time_to_main).suffix(" ms").speed(1).range(0..=1000000));
                        ui.weak("after drogue");
                    });
                });
                ui.end_row();

                ui.label("Drogue output timing");
                ui.horizontal(|ui| {
                    ui.add(DragValue::new(&mut self.drogue_output_settings.num_pulses).speed(1).range(0..=20));
                    ui.weak(if self.drogue_output_settings.num_pulses == 1 { "pulse of" } else { "pulses of" });
                    ui.add(DragValue::new(&mut self.drogue_output_settings.output_high_time).suffix(" ms").speed(1).range(0..=10000));
                    if self.drogue_output_settings.num_pulses > 1 {
                        ui.weak("with");
                        ui.add(DragValue::new(&mut self.drogue_output_settings.output_low_time).suffix(" ms").speed(1).range(0..=10000));
                        ui.weak("pauses after");
                    } else {
                        ui.weak("after");
                    }
                    ui.add(
                        DragValue::new(&mut self.drogue_output_settings.output_warning_time).suffix(" ms").speed(1).range(0..=10000),
                    );
                    ui.weak("of");
                    ui.add(
                        DragValue::new(&mut self.drogue_output_settings.output_warning_frequency)
                            .suffix(" Hz")
                            .speed(1.0)
                            .range(100.0..=10000.0),
                    );
                    ui.weak("tone");
                });
                ui.end_row();

                ui.label("Main output timing");
                ui.horizontal(|ui| {
                    ui.add(DragValue::new(&mut self.main_output_settings.num_pulses).speed(1).range(0..=20));
                    ui.weak(if self.main_output_settings.num_pulses == 1 { "pulse of" } else { "pulses of" });
                    ui.add(DragValue::new(&mut self.main_output_settings.output_high_time).suffix(" ms").speed(1).range(0..=10000));
                    if self.main_output_settings.num_pulses > 1 {
                        ui.weak("with");
                        ui.add(DragValue::new(&mut self.main_output_settings.output_low_time).suffix(" ms").speed(1).range(0..=10000));
                        ui.weak("pauses after");
                    } else {
                        ui.weak("after");
                    }
                    ui.add(
                        DragValue::new(&mut self.main_output_settings.output_warning_time).suffix(" ms").speed(1).range(0..=10000),
                    );
                    ui.weak("of");
                    ui.add(
                        DragValue::new(&mut self.main_output_settings.output_warning_frequency)
                            .suffix(" Hz")
                            .speed(1.0)
                            .range(100.0..=10000.0),
                    );
                    ui.weak("tone");
                });
                ui.end_row();

                ui.label("FC Orientation");
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.orientation, Orientation::ZUp, "Z+");
                    ui.selectable_value(&mut self.orientation, Orientation::ZDown, "Z-");
                });
                ui.end_row();

                for (sensor_label, sensor_settings) in [
                    ("ACS Tank (0)", &mut self.acs_tank_pressure_sensor_settings),
                    ("ACS Regulator (1)", &mut self.acs_regulator_pressure_sensor_settings),
                    ("ACS Accel. Valve (2)", &mut self.acs_accel_valve_pressure_sensor_settings),
                    ("ACS Decel. Valve (3)", &mut self.acs_decel_valve_pressure_sensor_settings),
                    ("Recovery Section (0)", &mut self.recovery_pressure_sensor_settings),
                ] {
                    ui.label(format!("{} Pressure Sensor Calib.", sensor_label));
                    ui.horizontal(|ui| {
                        ui.add(DragValue::new(&mut sensor_settings.intercept).speed(0.1).range(0.0..=3300.0));
                        ui.weak("mV at 0 bar,");
                        ui.add(DragValue::new(&mut sensor_settings.slope).speed(0.1).range(-10000.0..=10000.0));
                        ui.weak("mV/bar");
                    });
                    ui.end_row();
                }

                ui.label("ACS Acceleration");
                ui.horizontal(|ui| {
                    ui.add(
                        DragValue::new(&mut self.acs_nominal_acceleration)
                            .suffix(" m/s²")
                            .speed(0.001)
                            .range(0.0..=1000.0),
                    );
                    ui.weak(" @ ");
                    ui.add(
                        DragValue::new(&mut self.acs_nominal_tank_pressure)
                            .suffix(" bar")
                            .speed(0.001)
                            .range(0.0..=1000.0),
                    );
                    ui.weak(" with ");
                    ui.add(
                        DragValue::new(&mut self.acs_acceleration_pressure_slope)
                            .suffix(" (m/s^3)/bar")
                            .speed(0.0001)
                            .range(0.0..=50.0),
                    );
                });
                ui.end_row();

                ui.label("Apogee prediction drag red. factor");
                ui.add(
                    DragValue::new(&mut self.drag_reduction_factor)
                        .speed(0.00001)
                        .range(0.0..=1000.0),
                );
                ui.end_row();

                ui.label("Apogee error offset");
                ui.add(
                    DragValue::new(&mut self.apogee_error_offset)
                        .speed(0.1)
                        .range(-500.0..=500.0),
                );
                ui.end_row();
            })
    }
}
