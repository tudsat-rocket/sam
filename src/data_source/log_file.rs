//! A data source based on a logfile, either passed as a file path, or with
//! some raw bytes.

use std::any::Any;
use std::collections::VecDeque;
use std::fs::File;
use std::io::Read;
use std::path::PathBuf;
use std::slice::Iter;
use std::sync::mpsc::SendError;
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use log::*;

use egui::{Layout, RichText, Slider};

use mithril::settings::*;
use mithril::telemetry::*;

use crate::data_source::DataSource;

#[derive(Debug)]
enum PlaybackState {
    Playing(f32),
    Paused(Instant),
}

pub struct LogFileDataSource {
    file: Option<File>,
    buffer: Vec<u8>,
    is_json: bool,
    vehicle_states: Vec<(Instant, VehicleState)>,
    playback: Option<PlaybackState>,
}

impl LogFileDataSource {
    /// Open the given file as a data source.
    pub fn new(path: PathBuf) -> Result<Self, std::io::Error> {
        let file = File::open(&path)?;
        let is_json = path.extension().map(|ext| ext == "json").unwrap_or(false);

        Ok(Self {
            file: Some(file),
            buffer: Vec::new(),
            is_json,
            vehicle_states: Vec::new(),
            playback: None,
        })
    }

    /// Create given data source using the given name and bytes. The name is
    /// only passed to the data source to allow identifying it based on the
    /// status text.
    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        let is_json = bytes[0] == b'[';

        Self {
            file: None,
            buffer: bytes,
            is_json,
            vehicle_states: Vec::new(),
            playback: None,
        }
    }

    fn playpause(&mut self) {
        match self.playback {
            Some(PlaybackState::Playing(offset)) => {
                let inst = Instant::now() + Duration::from_secs_f32(offset);
                self.playback = Some(PlaybackState::Paused(inst));
            },
            Some(PlaybackState::Paused(inst)) => {
                let offset = (inst - Instant::now()).as_secs_f32();
                self.playback = Some(PlaybackState::Playing(offset));
            },
            None => {}
        }
    }

    fn skip_to_next_flightmode(&mut self, reverse: bool) {
        let Some(playback) = &self.playback else {
            return;
        };

        // Discriminant means we're just comparing the enum variant, not the actual value.
        let playing = std::mem::discriminant(playback) == std::mem::discriminant(&PlaybackState::Playing(123.0));

        let Some(fm) = self.vehicle_states().rev().find_map(|(_t, vs)| vs.mode) else {
            return;
        };

        let Some(end) = self.end() else {
            return;
        };

        let i = self.vehicle_states.partition_point(|(t, _)| *t <= end);

        let next_flightmode = if reverse {
            self.vehicle_states[..i]
                .iter()
                .rev()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != Some(fm))
                .map(|(t, _)| t)
                .next()
        } else {
            self.vehicle_states[i..]
                .iter()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != Some(fm))
                .map(|(t, _)| t)
                .next()
        };

        if let Some(inst) = next_flightmode {
            self.playback = Some(PlaybackState::Paused(*inst));
            if playing {
                self.playpause();
            }
        }
    }
}

impl DataSource for LogFileDataSource {
    fn update(&mut self, ctx: &egui::Context) {
        if let Some(file) = self.file.as_mut() {
            if let Err(e) = file.read_to_end(&mut self.buffer) {
                error!("Failed to read log file: {:?}", e);
            }
        }

        let msgs = if self.is_json {
            if self.buffer.len() > 0 {
                serde_json::from_slice::<Vec<DownlinkMessage>>(&self.buffer).unwrap()
            } else {
                vec![]
            }
        } else {
            self.buffer.split_mut(|b| *b == 0x00).filter_map(|b| postcard::from_bytes_cobs(b).ok()).collect()
        };

        self.buffer.truncate(0);

        // We have to give an Instant to every message. We can't only use
        // the time value contained in the packet, we need to handle the
        // occasional packet with a malformed time value.
        let start = Instant::now();
        let mut last_time = start;
        let mut last_vehicle_times: VecDeque<u32> = VecDeque::new();
        for msg in msgs.into_iter() {
            let last = last_vehicle_times.front();

            // The GCS msgs are sent by the ground station, immediately after the received
            // message. The time value used in those is the runtime of the GCS, so always
            // replace with 0.
            let since_previous = if let DownlinkMessage::TelemetryGCS(..) = msg {
                0
            } else {
                last.map(|l| msg.time().saturating_sub(*l)).unwrap_or(0)
            };

            last_time += Duration::from_millis(since_previous as u64);
            last_vehicle_times.push_front(u32::max(*last.unwrap_or(&0), msg.time()));
            last_vehicle_times.truncate(5); // TODO: use these for better filtering?

            self.vehicle_states.push((last_time, msg.into()));
        }

        let last_state = self.vehicle_states.last().map(|(t, _vs)| t);

        // We default to being paused at the end of the log file.
        if self.playback.is_none() {
            self.playback = last_state.map(|l| PlaybackState::Paused(*l));
        }

        if let Some(PlaybackState::Playing(_)) = self.playback {
            ctx.request_repaint_after(Duration::from_millis(16));

            if self.end().and_then(|i| last_state.map(|l| i > *l)).unwrap_or(false) {
                self.playback = last_state.map(|l| PlaybackState::Paused(*l));
            }
        }
    }

    fn vehicle_states<'a>(&'a self) -> Iter<'_, (Instant, VehicleState)> {
        let inst = self.end().unwrap_or(Instant::now());
        let i = self.vehicle_states.partition_point(|(t, _)| t <= &inst);
        self.vehicle_states[..i].iter()
    }

    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings> {
        None // TODO: store these in flash?
    }

    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings> {
        None
    }

    fn reset(&mut self) {
        self.vehicle_states.truncate(0);
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn end(&self) -> Option<Instant> {
        match self.playback {
            Some(PlaybackState::Playing(offset)) => {
                if offset < 0.0 {
                    Some(Instant::now() - Duration::from_secs_f32(offset.abs()))
                } else {
                    Some(Instant::now() + Duration::from_secs_f32(offset.abs()))
                }
            },
            Some(PlaybackState::Paused(inst)) => Some(inst),
            None => None
        }
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        let first = self.vehicle_states.first().map(|(t, _vs)| *t).unwrap_or(Instant::now());
        let last = self.vehicle_states.last().map(|(t, _vs)| *t).unwrap_or(Instant::now());
        let total = (last - first).as_secs_f32();
        let elapsed = self.end().map(|p| (p - first).as_secs_f32()).unwrap_or(0.0);

        ui.set_enabled(self.playback.is_some());

        if ui.button("⏮").clicked() {
            self.playback = Some(PlaybackState::Paused(first));
        }
        if ui.button("⏪").clicked() {
            self.skip_to_next_flightmode(true);
        }
        let label = if let Some(PlaybackState::Playing(_)) = self.playback { "⏸" } else { "⏵" };
        if ui.button(label).clicked() {
            self.playpause();
        }
        if ui.button("⏩").clicked() {
            self.skip_to_next_flightmode(false);
        }
        if ui.button("⏭").clicked() {
            self.playback = Some(PlaybackState::Paused(last));
        }

        ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
            let t = format!("{:02.0}:{:02.0}:{:02.0}/{:02.0}:{:02.0}:{:02.0}",
                elapsed / 3600.0, (elapsed / 60.0) % 60.0, elapsed % 60.0,
                total / 3600.0, (total / 60.0) % 60.0, total % 60.0
            );
            ui.label(RichText::new(t).monospace().weak());

            ui.style_mut().spacing.slider_width = ui.available_width();

            // Progress Bar
            let current = (total > 0.0).then_some(elapsed / total).unwrap_or(0.0);
            let mut new = current;
            let slider = Slider::new(&mut new, 0.0..=1.0)
                .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.3 })
                .show_value(false);
            let response = ui.add(slider);
            if response.drag_released() {
                self.playback = match self.playback {
                    Some(PlaybackState::Playing(offset)) => {
                        Some(PlaybackState::Playing(offset + (new - current) * total))
                    },
                    Some(PlaybackState::Paused(_)) => {
                        Some(PlaybackState::Paused(first + Duration::from_secs_f32(total * new)))
                    },
                    None => None
                }
            }
        });
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
