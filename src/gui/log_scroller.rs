use eframe::egui;
use egui::{Color32, Layout, RichText, Vec2};

use euroc_fc_firmware::telemetry::*;

use crate::telemetry_ext::*;

pub trait LogUiExt {
    fn log_scroller(
        &mut self,
        log_messages: &Vec<(u32, String, LogLevel, String)>,
        height: f32
    );
}

impl LogUiExt for egui::Ui {
    fn log_scroller(
        &mut self,
        log_messages: &Vec<(u32, String, LogLevel, String)>,
        height: f32,
    ) {
        egui::ScrollArea::vertical()
            .max_height(height)
            .max_width(self.available_width())
            .stick_to_bottom(true)
            .show(self, |ui| {
                ui.set_min_height(height);

                for (t, loc, ll, msg) in log_messages {
                    ui.horizontal(|ui| {
                        ui.allocate_ui_with_layout(
                            Vec2::new(60.0, 10.0),
                            Layout::top_down(eframe::emath::Align::RIGHT),
                            |ui| {
                                ui.label(
                                    RichText::new(format!("{:>8.3}", *t as f32 / 1000.0)).color(Color32::GRAY),
                                );
                            },
                        );

                        ui.allocate_ui_with_layout(
                            Vec2::new(70.0, 10.0),
                            Layout::top_down(eframe::emath::Align::LEFT),
                            |ui| {
                                ui.set_width(ui.available_width());
                                ui.label(RichText::new(ll.to_string()).color(ll.color()));
                            },
                        );

                        ui.allocate_ui_with_layout(
                            Vec2::new(150.0, 10.0),
                            Layout::top_down(eframe::emath::Align::LEFT),
                            |ui| {
                                ui.set_width(ui.available_width());
                                ui.label(RichText::new(loc).color(Color32::GRAY));
                            },
                        );

                        ui.allocate_ui_with_layout(
                            Vec2::new(ui.available_width(), 10.0),
                            Layout::top_down(eframe::emath::Align::LEFT),
                            |ui| {
                                ui.set_width(ui.available_width());
                                ui.label(RichText::new(msg).color(Color32::LIGHT_GRAY));
                            },
                        );
                    });
                }
            });
    }
}
