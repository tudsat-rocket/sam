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

use shared_types::telemetry::*;

use crate::data_source::*;

pub struct LogFileDataSource {
    file: Option<File>,
    buffer: Vec<u8>,
    is_json: bool,
    vehicle_states: Vec<(Instant, VehicleState)>,
    playback: Option<PlaybackState>,
    playback_speed: usize,
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
            playback_speed: 0,
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
            playback_speed: 0,
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
            if !self.buffer.is_empty() {
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

        self.update_playback(ctx);
    }

    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)> {
        let inst = self.end().unwrap_or(Instant::now());
        let i = self.vehicle_states.partition_point(|(t, _)| t <= &inst);
        self.vehicle_states[..i].iter()
    }

    fn fc_settings(&mut self) -> Option<&Settings> {
        None // TODO: store these in flash?
    }

    fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
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
        self.playback_end()
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        self.playback_ui(ui)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

impl ReplayableDataSource for LogFileDataSource {
    fn playback_state(&self) -> Option<PlaybackState> {
        self.playback.clone()
    }

    fn playback_state_mut(&mut self) -> &mut Option<PlaybackState> {
        &mut self.playback
    }

    fn playback_speed(&self) -> usize {
        self.playback_speed
    }

    fn playback_speed_mut(&mut self) -> &mut usize {
        &mut self.playback_speed
    }

    fn all_vehicle_states(&self) -> &Vec<(Instant, VehicleState)> {
        &self.vehicle_states
    }
}
