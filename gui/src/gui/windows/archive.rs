use std::io::Write;
use std::str::FromStr;
use std::sync::mpsc::{Receiver, Sender};

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::egui;
use egui::{Align, Align2, Button, Layout, ProgressBar};

use futures::StreamExt;
use log::*;

use shared_types::telemetry::{VehicleState, DownlinkMessage};

use crate::data_source::LogFileDataSource;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ArchivedLog {
    Zuelpich1,
    Zuelpich2,
    Dare23A,
    Dare23B,
    Euroc23,
}

impl FromStr for ArchivedLog {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "dare_a" => Ok(Self::Dare23A),
            "dare_b" => Ok(Self::Dare23B),
            "euroc23" => Ok(Self::Euroc23),
            _ => Err("Failed to parse log enum.")
        }
    }
}

impl ToString for ArchivedLog {
    fn to_string(&self) -> String {
        match self {
            Self::Zuelpich1 => "Zülpich #1".into(),
            Self::Zuelpich2 => "Zülpich #2".into(),
            Self::Dare23A => "Dare (FC A)".into(),
            Self::Dare23B => "Dare (FC B)".into(),
            Self::Euroc23 => "EuRoC 2023 (ÆSIR Signý)".into(),
        }
    }
}

impl ArchivedLog {
    pub fn all() -> Vec<Self> {
        vec![Self::Zuelpich1, Self::Zuelpich2, Self::Dare23A, Self::Dare23B, Self::Euroc23]
    }

    // TODO: migrate old launches
    pub fn telemetry_log_url(&self) -> Option<&'static str> {
        match self {
            Self::Dare23A => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_telem_filtered.json"),
            Self::Dare23B => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_telem_filtered.json"),
            Self::Euroc23 => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_telem_filtered.json"),
            _ => None
        }
    }

    pub fn flash_log_url(&self) -> Option<&'static str> {
        match self {
            Self::Dare23A => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_a_flash_filtered.json"),
            Self::Dare23B => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/dare_launch_b_flash_filtered.json"),
            Self::Euroc23 => Some("https://raw.githubusercontent.com/tudsat-rocket/sam/main/archive/euroc_2023_flash_filtered.json"),
            _ => None
        }
    }

    #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
    pub fn flash_log(&self) -> Option<&'static [u8]> {
        match self {
            Self::Dare23A => Some(include_bytes!("../../../../archive/dare_launch_a_flash_filtered.json").as_slice()),
            Self::Dare23B => Some(include_bytes!("../../../../archive/dare_launch_b_flash_filtered.json").as_slice()),
            Self::Euroc23 => Some(include_bytes!("../../../../archive/euroc_2023_flash_filtered.json").as_slice()),
            _ => None
        }
    }

    #[cfg(any(target_arch = "wasm32", target_os = "android"))]
    pub fn flash_log(&self) -> Option<&'static [u8]> {
        None
    }

    pub fn flash_messages(&self) -> Option<Vec<DownlinkMessage>> {
        self.flash_log().map(|bytes| serde_json::from_slice::<Vec<DownlinkMessage>>(bytes).unwrap())
    }

    pub fn flash_states(&self) -> Option<Vec<VehicleState>> {
        self.flash_messages().map(|vec| vec.into_iter().map(|msg| Into::<VehicleState>::into(msg)).collect())
    }
}

#[derive(Debug)]
enum ArchiveLoadProgress { Progress((u64, u64)),
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
            progress: None
        }
    }
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
                    return Some(LogFileDataSource::from_bytes(bytes));
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

                for (i, log) in ArchivedLog::all().iter().enumerate() {
                    if i != 0 {
                        ui.separator();
                    }

                    ui.horizontal(|ui| {
                        ui.label(log.to_string());
                        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                            if ui.add_enabled(log.flash_log_url().is_some(), Button::new("🖴  Flash")).clicked() {
                                self.open_log(ctx, log.flash_log_url().unwrap());
                            }

                            if ui.add_enabled(log.telemetry_log_url().is_some(), Button::new("📡 Telemetry")).clicked() {
                                self.open_log(ctx, log.telemetry_log_url().unwrap());
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
            });

        self.open = open;

        None
    }
}
