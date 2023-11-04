//! A data source based on a logfile, either passed as a file path, or with
//! some raw bytes.

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

use eframe::epaint::Color32;
use log::*;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::data_source::DataSource;
use crate::state::*;

pub struct LogFileDataSource {
    path: Option<PathBuf>,
    name: Option<String>,
    file: Option<File>,
    buffer: Vec<u8>,
    messages: Vec<(Instant, DownlinkMessage)>,
    is_json: bool,
    vehicle_states: Vec<(Instant, VehicleState)>,
    last_time: Option<Instant>,
    replay: bool,
}

impl LogFileDataSource {
    /// Open the given file as a data source.
    pub fn new(path: PathBuf) -> Result<Self, std::io::Error> {
        let file = File::open(&path)?;
        let is_json = path.extension().map(|ext| ext == "json").unwrap_or(false);

        Ok(Self {
            path: Some(path),
            name: None,
            file: Some(file),
            buffer: Vec::new(),
            messages: Vec::new(),
            is_json,
            vehicle_states: Vec::new(),
            last_time: None,
            replay: false,
        })
    }

    /// Create given data source using the given name and bytes. The name is
    /// only passed to the data source to allow identifying it based on the
    /// status text.
    pub fn from_bytes(name: Option<String>, bytes: Vec<u8>, replay: bool) -> Self {
        let is_json = bytes[0] == b'[';

        Self {
            path: None,
            name,
            file: None,
            buffer: bytes,
            messages: Vec::new(),
            is_json,
            vehicle_states: Vec::new(),
            last_time: None,
            replay,
        }
    }
}

impl DataSource for LogFileDataSource {
    fn update(&mut self, _ctx: &egui::Context) {
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
        self.last_time.get_or_insert(start);
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

            self.last_time = self.last_time.map(|t| t + Duration::from_millis(since_previous as u64));
            last_vehicle_times.push_front(u32::max(*last.unwrap_or(&0), msg.time()));
            last_vehicle_times.truncate(5); // TODO: use these for better filtering?

            self.messages.push((self.last_time.unwrap(), msg));
        }

        let pointer = if self.replay {
            let now = Instant::now();
            self.messages.partition_point(|(t, _)| t <= &now)
        } else {
            self.messages.len()
        };

        for (t, msg) in self.messages.drain(..pointer) {
            self.vehicle_states.push((t, msg.into()));
        }
    }

    fn vehicle_states<'a>(&'a self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings> {
        None // TODO: store these in flash?
    }

    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings> {
        None
    }

    fn reset(&mut self) {
        self.messages.truncate(0);
        self.vehicle_states.truncate(0);
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn minimum_fps(&self) -> Option<u64> {
        if self.replay && self.last_time.map(|t| t > Instant::now()).unwrap_or(true) {
            Some(60)
        } else {
            None
        }
    }

    fn is_log_file(&self) -> bool {
        true
    }

    fn end(&self) -> Option<Instant> {
        if self.replay {
            let last = self.last_time.unwrap_or(Instant::now());
            Some(Instant::min(Instant::now(), last))
        } else {
            self.last_time
        }
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        ui.colored_label(Color32::from_rgb(0x45, 0x85, 0x88), "Log File");
        let name = self.path
            .as_ref()
            .map(|p| p.as_os_str().to_string_lossy().into())
            .or(self.name.clone())
            .unwrap_or_default();
        ui.weak(name);
    }
}
