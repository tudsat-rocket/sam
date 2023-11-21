use std::io::Write;
use std::sync::mpsc::{Receiver, Sender};

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::egui;
use egui::{Align, Align2, Button, Layout, ProgressBar};

use futures::StreamExt;
use log::*;

use crate::data_source::LogFileDataSource;

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

#[derive(Debug)]
enum ArchiveLoadProgress { Progress((u64, u64)),
    Complete(Vec<u8>),
    Error(reqwest::Error),
}

#[derive(Default)]
pub struct ArchiveWindow {
    pub open: bool,
    replay_logs: bool,
    progress_receiver: Option<Receiver<ArchiveLoadProgress>>,
    progress: Option<(u64, u64)>,
}

impl ArchiveWindow {
    async fn load_log(ctx: egui::Context, url: &str, progress_sender: Sender<ArchiveLoadProgress>) {
        let start = Instant::now();
        let response = match reqwest::Client::new().get(url).send().await {
            Ok(res) => res,
            Err(e) => {
                progress_sender.send(ArchiveLoadProgress::Error(e)).unwrap();
                return;
            }
        };

        let total_size = response.content_length().unwrap_or(0);
        progress_sender.send(ArchiveLoadProgress::Progress((0, total_size))).unwrap();
        ctx.request_repaint();

        let mut cursor = std::io::Cursor::new(Vec::with_capacity(total_size as usize));
        let (mut progress, mut last_progress) = (0, 0);
        let mut stream = response.bytes_stream();
        while let Some(result) = stream.next().await {
            match result {
                Ok(chunk) => {
                    cursor.write_all(&chunk).unwrap();
                    progress = u64::min(progress + chunk.len() as u64, total_size);
                    if progress == total_size || progress > last_progress + 256 * 1024 {
                        let _ = progress_sender.send(ArchiveLoadProgress::Progress((progress, total_size)));
                        last_progress = progress;
                        ctx.request_repaint();
                    }
                }
                Err(e) => {
                    progress_sender.send(ArchiveLoadProgress::Error(e)).unwrap();
                    ctx.request_repaint();
                    return;
                }
            }
        }

        progress_sender.send(ArchiveLoadProgress::Complete(cursor.into_inner())).unwrap();
        let duration = start.elapsed().as_secs_f32();
        let mib = (total_size as f32) / 1024.0 / 1024.0;
        info!("Downloaded {}MiB in {:.1}ms ({}MiB/s)", mib, duration * 1000.0, mib / duration);
        ctx.request_repaint();
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn open_log(&mut self, ctx: &egui::Context, url: &'static str) {
        let ctx = ctx.clone();
        let (sender, receiver) = std::sync::mpsc::channel();
        self.progress_receiver = Some(receiver);
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
            rt.block_on(Self::load_log(ctx, url, sender));
        });
    }

    #[cfg(target_arch = "wasm32")]
    fn open_log(&mut self, ctx: &egui::Context, url: &'static str) {
        let ctx = ctx.clone();
        let (sender, receiver) = std::sync::mpsc::channel();
        self.progress_receiver = Some(receiver);
        wasm_bindgen_futures::spawn_local(Self::load_log(ctx, url, sender));
    }

    pub fn show_if_open(&mut self, ctx: &egui::Context) -> Option<LogFileDataSource> {
        if let Some(receiver) = self.progress_receiver.as_ref() {
            match receiver.try_recv() {
                Ok(ArchiveLoadProgress::Progress(progress)) => {
                    self.progress = Some(progress);
                }
                Ok(ArchiveLoadProgress::Complete(bytes)) => {
                    self.open = false;
                    self.progress_receiver = None;
                    self.progress = None;
                    return Some(LogFileDataSource::from_bytes(
                        Some("".to_string()), // TODO: show title
                        bytes,
                        self.replay_logs,
                    ));
                }
                Ok(ArchiveLoadProgress::Error(e)) => {
                    error!("{:?}", e); // TODO: show this visually
                    self.progress_receiver = None;
                    self.progress = None;
                }
                _ => {}
            }
        }

        // avoids mutably borrowing self
        let mut open = self.open;

        egui::Window::new("Flight Archive")
            .open(&mut open)
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
                                self.open_log(ctx, flash.unwrap());
                            }

                            if ui.add_enabled(telem.is_some(), Button::new("📡 Telemetry")).clicked() {
                                self.open_log(ctx, telem.unwrap());
                            }
                        });
                    });
                }

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    ui.add_visible_ui(self.progress.is_some(), |ui| {
                        let (done, total) = self.progress.unwrap_or((0, 0));
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

                ui.checkbox(&mut self.replay_logs, "Replay logs");
            });

        self.open = open;

        None
    }
}
