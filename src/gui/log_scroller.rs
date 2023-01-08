use eframe::egui;
use egui::{Color32, Layout, RichText, Vec2};

use euroc_fc_firmware::telemetry::*;

use crate::telemetry_ext::*;

pub trait LogUiExt {
    fn log_scroller(
        &mut self,
        log_messages: &Vec<(u32, String, LogLevel, String)>
    );
}

impl LogUiExt for egui::Ui {
    fn log_scroller(
        &mut self,
        log_messages: &Vec<(u32, String, LogLevel, String)>
    ) {
        let h = self.available_height();
        egui::ScrollArea::vertical()
            .max_height(self.available_height())
            .max_width(self.available_width())
            .stick_to_bottom(true)
            .show(self, |ui| {
                ui.set_min_height(h);

                let gray = Color32::from_rgb(0x66, 0x5c, 0x54);

                for (t, loc, ll, msg) in log_messages {
                    ui.horizontal(|ui| {
                        ui.allocate_ui_with_layout(
                            Vec2::new(60.0, 10.0),
                            Layout::top_down(eframe::emath::Align::RIGHT),
                            |ui| {
                                ui.monospace(
                                    RichText::new(format!("{:>8.3}", *t as f32 / 1000.0)).color(gray),
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
                                ui.monospace(RichText::new(loc).color(gray));
                            },
                        );

                        ui.allocate_ui_with_layout(
                            Vec2::new(ui.available_width(), 10.0),
                            Layout::top_down(eframe::emath::Align::LEFT),
                            |ui| {
                                ui.set_width(ui.available_width());
                                ui.monospace(msg);
                            },
                        );
                    });
                }
            });
    }
}
