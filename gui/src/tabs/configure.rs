use std::str::FromStr;

use egui::{Align, Button, DragValue, Layout, RichText, TextEdit};

use log::*;

use shared_types::telemetry::*;

use crate::Backend;
use crate::settings::AppSettings;
use crate::utils::file::*;
use crate::widgets::fc_settings::*;

pub struct ConfigureTab {}

impl ConfigureTab {
    pub fn init() -> Self {
        Self {}
    }

    fn app_settings_ui(&mut self, ui: &mut egui::Ui, backend: &mut Backend, settings: &mut AppSettings) -> bool {
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
                            &mut settings.lora.downlink_channels[0],
                            RichText::new("863.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[1],
                            RichText::new("863.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[2],
                            RichText::new("864.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[3],
                            RichText::new("864.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[4],
                            RichText::new("865.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[5],
                            RichText::new("865.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[6],
                            RichText::new("866.25").monospace().size(10.0),
                        );
                        ui.label(RichText::new("MHz").weak().size(10.0));
                    });
                    ui.horizontal(|ui| {
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[7],
                            RichText::new("866.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[8],
                            RichText::new("867.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[9],
                            RichText::new("867.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[10],
                            RichText::new("868.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[11],
                            RichText::new("868.75").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[12],
                            RichText::new("869.25").monospace().size(10.0),
                        );
                        ui.toggle_value(
                            &mut settings.lora.downlink_channels[13],
                            RichText::new("869.75").monospace().size(10.0),
                        );
                        ui.label(RichText::new("MHz").weak().size(10.0));
                    });
                });
                ui.end_row();

                ui.label("LoRa binding phrase");
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("⬅ Copy from FC").clicked() {
                        settings.lora.binding_phrase = backend
                            .fc_settings()
                            .map(|s| s.lora.binding_phrase.clone())
                            .unwrap_or(settings.lora.binding_phrase.clone());
                    }

                    let mut binding_phrase: String = settings.lora.binding_phrase.to_string();
                    ui.add_sized(ui.available_size(), TextEdit::singleline(&mut binding_phrase));
                    settings.lora.binding_phrase = heapless::String::from_str(&binding_phrase).unwrap_or_default();
                });
                ui.end_row();

                ui.label("LoRa uplink key");
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("⬅ Copy from FC").clicked() {
                        settings.lora.authentication_key = backend
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

                ui.label("GCS Pos.:");
                ui.horizontal(|ui| {
                    ui.weak("Lat");
                    ui.add_sized(
                        [100.0, ui.available_height()],
                        DragValue::new(&mut settings.ground_station_position.as_mut().unwrap().0)
                            .suffix(" °")
                            .speed(0.0001)
                            .range(-90.0..=90.0),
                    );
                    ui.weak("Lng");
                    ui.add_sized(
                        [100.0, ui.available_height()],
                        DragValue::new(&mut settings.ground_station_position.as_mut().unwrap().1)
                            .suffix(" °")
                            .speed(0.0001)
                            .range(-180.0..=180.0),
                    );
                });
                ui.end_row();
            });

        ui.add_space(20.0);

        ui.horizontal(|ui| {
            ui.label("LoRa bookmarks:");

            settings.lora_bookmarks.get_or_insert(Vec::new());
            let Some(lora_bookmarks) = settings.lora_bookmarks.as_mut() else {
                unreachable!();
            };

            for bookmark in lora_bookmarks.iter() {
                if ui.selectable_label(*bookmark == settings.lora, bookmark.binding_phrase.to_string()).clicked() {
                    settings.lora = bookmark.clone();
                }
            }

            let selected = lora_bookmarks.iter().find(|b| settings.lora == **b).cloned();

            ui.add_enabled_ui(selected.is_none(), |ui| {
                if ui.button("➕Add").clicked() {
                    lora_bookmarks.push(settings.lora.clone());
                }
            });

            ui.add_enabled_ui(selected.is_some(), |ui| {
                if ui.button("🗑 Remove").clicked() {
                    lora_bookmarks.remove(lora_bookmarks.iter().position(|b| settings.lora == *b).unwrap());
                }
            });
        });

        ui.add_space(20.0);

        ui.horizontal(|ui| {
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

    fn fc_settings_ui(&mut self, ui: &mut egui::Ui, backend: &mut Backend, settings: &mut AppSettings) {
        ui.add_space(10.0);
        ui.heading("FC Settings");
        ui.add_space(10.0);

        if let Some(fc_settings) = backend.fc_settings_mut() {
            fc_settings.ui(ui, Some(settings), false);
        } else {
            ui.weak("Not connected.");
            return;
        }

        ui.add_space(20.0);

        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            if ui.add_enabled(backend.fc_settings().is_some(), Button::new("💾 Write Settings & Reboot")).clicked() {
                let settings = backend.fc_settings().cloned().unwrap();
                backend.send(UplinkMessage::WriteSettings(settings)).unwrap();
            }

            #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
            if ui.add_enabled(backend.fc_settings().is_some(), Button::new("🖹 Save to File")).clicked() {
                save_fc_settings_file(backend.fc_settings().unwrap());
            }

            #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
            if ui.add_enabled(backend.fc_settings().is_some(), Button::new("🖹 Load from File")).clicked() {
                if let Some(settings) = open_fc_settings_file() {
                    info!("Loaded settings: {:?}", settings);

                    // TODO: remove unwrap / is correct?
                    // *backend.fc_settings_mut().unwrap() = settings;
                    match backend.fc_settings_mut() {
                        Some(fc_settings) => *fc_settings = settings,
                        None => (),
                    }
                }
            }

            if ui.button("🔃Reload from FC").clicked() {
                backend.send(UplinkMessage::ReadSettings).unwrap();
            }

            if ui.button("⏮ Reset to Defaults").clicked() {
                *backend.fc_settings_mut().unwrap() = Default::default();
            }
        });
    }

    pub fn main_ui(
        &mut self,
        ctx: &egui::Context,
        backend: &mut Backend,
        settings: &mut AppSettings,
        enabled: bool,
    ) -> bool {
        let mut changed = false;

        egui::CentralPanel::default().show(ctx, |ui| {
            if !enabled {
                ui.disable();
            }

            if ctx.screen_rect().width() > 1400.0 {
                ui.columns(2, |cols| {
                    egui::ScrollArea::vertical().id_salt("app_settings").show(&mut cols[0], |ui| {
                        changed = self.app_settings_ui(ui, backend, settings);
                    });
                    egui::ScrollArea::vertical().id_salt("fc_settings").show(&mut cols[1], |ui| {
                        self.fc_settings_ui(ui, backend, settings);
                    });
                });
            } else {
                egui::ScrollArea::vertical().id_salt("settings").show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    changed = self.app_settings_ui(ui, backend, settings);
                    ui.add_space(20.0);
                    ui.separator();
                    self.fc_settings_ui(ui, backend, settings);
                });
            }
        });

        changed
    }
}
