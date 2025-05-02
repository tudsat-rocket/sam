use std::io::Write;
use std::sync::mpsc::{Receiver, Sender};

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::egui;
use egui::{Align, Align2, Button, Layout, ProgressBar, RichText};

use futures::StreamExt;
use log::*;

use archive::ARCHIVED_LOGS;

use crate::backend::LogFileBackend;

#[derive(Debug)]
enum ArchiveLoadProgress {
    Progress((u64, u64)),
    Complete(Vec<u8>),
    Error(reqwest::Error),
}

pub struct ArchiveWindow {
    pub open: bool,
    progress_receiver: Option<Receiver<ArchiveLoadProgress>>,
    progress: Option<(u64, u64)>,
}

// clippy tries to remove the open difference between platforms here
#[allow(clippy::derivable_impls)]
impl Default for ArchiveWindow {
    fn default() -> Self {
        Self {
            open: cfg!(target_arch = "wasm32"),
            progress_receiver: None,
            progress: None,
        }
    }
}

impl ArchiveWindow {
    async fn load_log(ctx: egui::Context, url: &str, progress_sender: Sender<ArchiveLoadProgress>) {
        let start = Instant::now();
        let response = match reqwest::Client::new().get(url).send().await {
            Ok(res) => res,
            Err(e) => {
                let _ = progress_sender.send(ArchiveLoadProgress::Error(e));
                return;
            }
        };

        let total_size = response.content_length().unwrap_or(0);
        let _ = progress_sender.send(ArchiveLoadProgress::Progress((0, total_size)));
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
                    let _ = progress_sender.send(ArchiveLoadProgress::Error(e));
                    ctx.request_repaint();
                    return;
                }
            }
        }

        let _ = progress_sender.send(ArchiveLoadProgress::Complete(cursor.into_inner()));
        let duration = start.elapsed().as_secs_f32();
        let mib = (total_size as f32) / 1024.0 / 1024.0;
        info!("Downloaded {}MiB in {:.1}ms ({}MiB/s)", mib, duration * 1000.0, mib / duration);
        ctx.request_repaint();
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn open_log_url(&mut self, ctx: &egui::Context, url: &'static str) {
        let ctx = ctx.clone();
        let (sender, receiver) = std::sync::mpsc::channel();
        self.progress_receiver = Some(receiver);
        std::thread::spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
            rt.block_on(Self::load_log(ctx, url, sender));
        });
    }

    #[cfg(target_arch = "wasm32")]
    fn open_log_url(&mut self, ctx: &egui::Context, url: &'static str) {
        let ctx = ctx.clone();
        let (sender, receiver) = std::sync::mpsc::channel();
        self.progress_receiver = Some(receiver);
        wasm_bindgen_futures::spawn_local(Self::load_log(ctx, url, sender));
    }

    pub fn show_if_open(&mut self, ctx: &egui::Context) -> Option<LogFileBackend> {
        if let Some(receiver) = self.progress_receiver.as_ref() {
            let mut progress = None;

            while let Ok(p) = receiver.try_recv() {
                progress = Some(p);
            }

            match progress {
                Some(ArchiveLoadProgress::Progress(progress)) => {
                    self.progress = Some(progress);
                }
                Some(ArchiveLoadProgress::Complete(bytes)) => {
                    self.progress_receiver = None;
                    self.progress = None;
                    self.open = false;
                    return Some(LogFileBackend::from_bytes(bytes));
                }
                Some(ArchiveLoadProgress::Error(e)) => {
                    error!("{:?}", e); // TODO: show this visually
                    self.progress_receiver = None;
                    self.progress = None;
                }
                None => {}
            }
        }

        // avoids mutably borrowing self
        let mut open = self.open;

        egui::Window::new("Flight Archive")
            .open(&mut open)
            .min_width(400.0)
            .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
            .resizable(false)
            .collapsible(false)
            .show(ctx, |ui| {
                ui.add_space(10.0);

                for (i, log) in ARCHIVED_LOGS.iter().enumerate() {
                    if i != 0 {
                        ui.separator();
                    }

                    ui.horizontal(|ui| {
                        ui.label(log.to_string());

                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            ui.label(RichText::new(log.id).weak().monospace());
                        });
                    });

                    ui.horizontal(|ui| {
                        ui.label("🚀");
                        ui.weak(log.vehicle);
                        ui.label("🚩");
                        ui.weak(log.site.name);
                        ui.label("📆");
                        ui.weak(log.date.to_string());
                    });

                    ui.horizontal(|ui| {
                        ui.label("🏷");
                        ui.weak(log.fc_serial);
                        ui.label("⬆");
                        ui.weak(format!("{}m", log.apogee_agl));

                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            if ui.add_enabled(log.flash_log_url.is_some(), Button::new("🖴  Flash")).clicked() {
                                self.open_log_url(ctx, log.flash_log_url.unwrap());
                            }

                            if ui.add_enabled(log.telemetry_log_url.is_some(), Button::new("📡 Telemetry")).clicked()
                            {
                                self.open_log_url(ctx, log.telemetry_log_url.unwrap());
                            }
                        });
                    });

                    if !log.description.is_empty() {
                        ui.weak(log.description);
                    }
                }

                ui.add_space(10.0);
                ui.horizontal(|ui| {
                    #[allow(deprecated)] // right now, the alternatives arent't great
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
            });

        self.open = open;

        None
    }
}
