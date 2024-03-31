use egui::{Align, Button, Color32, Layout, RichText, TextEdit};

use log::*;

use mithril::telemetry::*;

use crate::data_source::DataSource;
use crate::file::*;
use crate::settings::AppSettings;

use crate::gui::fc_settings::*;

pub struct ConfigureTab {}

impl ConfigureTab {
    pub fn init() -> Self {
        Self {}
    }

    fn app_settings_ui(&mut self, ui: &mut egui::Ui, data_source: &mut dyn DataSource, settings: &mut AppSettings) -> bool {
        let mut changed = false;

        ui.add_space(10.0);
        ui.heading("GCS Settings");
        ui.add_space(10.0);

        egui::Grid::new("app_settings_grid")
            .num_columns(2)
            .spacing([40.0, 4.0])
            .striped(true)
            .show(ui, |ui| {
                ui.label("MapBox Access Token");
                ui.add_sized(ui.available_size(), TextEdit::singleline(&mut settings.mapbox_access_token));
                ui.end_row();

                ui.label("LoRa channel selection (500kHz BW)");
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        ui.toggle_value(
                            &mut settings.lora.channels[0],
                            RichText::new("863.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[1],
                            RichText::new("863.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[2],
                            RichText::new("864.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[3],
                            RichText::new("864.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[4],
                            RichText::new("865.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[5],
                            RichText::new("865.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[6],
                            RichText::new("866.25").monospace().size(10.0),
                        );
                        ui.label(RichText::new("MHz").weak().size(10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.toggle_value(
                            &mut settings.lora.channels[7],
                            RichText::new("866.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[8],
                            RichText::new("867.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[9],
                            RichText::new("867.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[10],
                            RichText::new("868.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[11],
                            RichText::new("868.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[12],
                            RichText::new("869.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.channels[13],
                            RichText::new("869.75").monospace().size(10.0),
                        );
                        ui.label(RichText::new("MHz").weak().size(10.0));
                    });
                });
                ui.end_row();

                ui.label("LoRa binding phrase");
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("⬅ Copy from FC").clicked() {
                        settings.lora.binding_phrase = data_source
                            .fc_settings()
                            .map(|s| s.lora.binding_phrase.clone())
                            .unwrap_or(settings.lora.binding_phrase.clone());
                    }

                    ui.add_sized(ui.available_size(), TextEdit::singleline(&mut settings.lora.binding_phrase));
                });
                ui.end_row();

                ui.label("LoRa uplink key");
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("⬅ Copy from FC").clicked() {
                        settings.lora.authentication_key = data_source
                            .fc_settings()
                            .map(|s| s.lora.authentication_key)
                            .unwrap_or(settings.lora.authentication_key);
                    }

                    #[cfg(not(target_arch = "wasm32"))]
                    if ui.button("🔃Rekey").clicked() {
                        settings.lora.authentication_key = rand::random();
                    }

                    ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                        ui.monospace(format!("{:032x}", settings.lora.authentication_key));
                    })
                });
                ui.end_row();
            });

        ui.add_space(20.0);

        ui.horizontal_centered(|ui| {
            ui.set_width(ui.available_width());

            if ui.button("🔃Reload").clicked() {
                *settings = AppSettings::load().unwrap_or_default();
                changed = true;
            }

            if ui.button("💾 Save Settings").clicked() {
                settings.save().unwrap();
                changed = true;
            }
        });

        changed
    }

    fn fc_settings_ui(&mut self, ui: &mut egui::Ui, data_source: &mut dyn DataSource, settings: &mut AppSettings) {
        ui.add_space(10.0);
        ui.heading("FC Settings");
        ui.add_space(10.0);

        if let Some(fc_settings) = data_source.fc_settings_mut() {
            fc_settings.ui(ui, Some(settings), false);
        } else {
            ui.colored_label(Color32::GRAY, "Not connected.");
        }

        ui.add_space(20.0);

        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            if ui
                .add_enabled(data_source.fc_settings().is_some(), Button::new("💾 Write Settings & Reboot"))
                .clicked()
            {
                let settings = data_source.fc_settings().cloned().unwrap();
                data_source.send(UplinkMessage::WriteSettings(settings)).unwrap();
            }

            #[cfg(not(any(target_arch = "wasm32", target_os="android")))]
            if ui.add_enabled(data_source.fc_settings().is_some(), Button::new("🖹 Save to File")).clicked() {
                save_fc_settings_file(data_source.fc_settings().unwrap());
            }

            #[cfg(not(any(target_arch = "wasm32", target_os="android")))]
            if ui.add_enabled(data_source.fc_settings().is_some(), Button::new("🖹 Load from File")).clicked()
            {
                if let Some(settings) = open_fc_settings_file() {
                    info!("Loaded settings: {:?}", settings);
                    *data_source.fc_settings_mut().unwrap() = settings;
                }
            }

            if ui.button("🔃Reload").clicked() {
                data_source.send(UplinkMessage::ReadSettings).unwrap();
            }
        });
    }

    pub fn main_ui(&mut self, ctx: &egui::Context, data_source: &mut dyn DataSource, settings: &mut AppSettings, enabled: bool) -> bool {
        let mut changed = false;

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.set_enabled(enabled);
            ui.horizontal(|ui| {
                ui.set_width(ui.available_width());
                ui.set_height(ui.available_height());

                ui.vertical(|ui| {
                    ui.set_width(ui.available_width() / 2.0);
                    ui.set_height(ui.available_height());

                    changed = self.app_settings_ui(ui, data_source, settings);
                });

                ui.separator();

                ui.vertical(|ui| {
                    ui.set_width(ui.available_width());

                    self.fc_settings_ui(ui, data_source, settings);
                });
            });
        });

        changed
    }
}
