use crate::gui::Sam;
use eframe::egui;
use egui::{Align, Align2, Button, Layout, ProgressBar};

// Log files included with the application.
// TODO: migrate old launches
pub const ARCHIVE: [(&str, Option<&'static str>, Option<&'static str>); 5] = [
    ("Zülpich #1", None, None),
    ("Zülpich #2", None, None),
    (
        "DARE (FC A)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_flash_filtered.json"),
    ),
    (
        "DARE (FC B)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_flash_filtered.json"),
    ),
    (
        "EuRoC 2023 (ÆSIR Signý)",
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_telem_filtered.json"),
        Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_flash_filtered.json"),
    ),
];

pub fn open_archive_window(ctx: &egui::Context, archive_open: &mut bool, sam: &mut Sam) {
    egui::Window::new("Flight Archive")
        .open(archive_open)
        .min_width(300.0)
        .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.add_space(10.0);

            for (i, (title, telem, flash)) in ARCHIVE.iter().enumerate() {
                if i != 0 {
                    ui.separator();
                }

                ui.horizontal(|ui| {
                    ui.label(*title);
                    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                        if ui.add_enabled(flash.is_some(), Button::new("🖴  Flash")).clicked() {
                            sam.open_archive_log(flash.unwrap());
                        }

                        if ui.add_enabled(telem.is_some(), Button::new("📡 Telemetry")).clicked() {
                            sam.open_archive_log(telem.unwrap());
                        }
                    });
                });
            }

            ui.add_space(10.0);
            ui.horizontal(|ui| {
                ui.add_visible_ui(sam.archive_progress.is_some(), |ui| {
                    let (done, total) = sam.archive_progress.unwrap_or((0, 0));
                    let f = (total > 0).then(|| done as f32 / total as f32).unwrap_or(0.0);
                    let text = format!(
                        "{:.2}MiB / {:.2}MiB",
                        done as f32 / (1024.0 * 1024.0),
                        total as f32 / (1024.0 * 1024.0)
                    );
                    ui.add_sized([ui.available_width(), 20.0], ProgressBar::new(f).text(text));
                });
            });
            ui.add_space(10.0);

            ui.checkbox(&mut sam.replay_logs, "Replay logs");
        });
}
