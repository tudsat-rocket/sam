use crate::gui::Sam;

use egui::{CollapsingHeader, Context};

pub fn create_header(sam: &mut Sam, ctx: &Context) {
    if ctx.screen_rect().width() > 1000.0 {
        egui::TopBottomPanel::top("topbar").min_height(60.0).max_height(60.0).show(ctx, |ui| {
            ui.set_enabled(!sam.archive_window_open);
            ui.horizontal_centered(|ui| {
                sam.top_bar(ui, false);
            });
        });
    } else {
        egui::TopBottomPanel::top("topbar").min_height(20.0).max_height(300.0).show(ctx, |ui| {
            ui.set_enabled(!sam.archive_window_open);
            CollapsingHeader::new("Status & Controls").default_open(false).show(ui, |ui| {
                sam.top_bar(ui, true);
                ui.add_space(10.0);
            });
        });
    }
}
