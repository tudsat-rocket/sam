//! Data sources serve as an abstraction for the origin of the displayed data.
//! This data source can either be a serial port device, or an opened log file.

use core::hash::Hasher;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender, SendError};
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use instant::Instant;

use eframe::epaint::Color32;
use siphasher::sip::SipHasher;
use log::*;

use euroc_fc_firmware::telemetry::*;

use crate::serial::*;
use crate::state::*;

/// A serial port data source. The default.
pub struct SerialDataSource {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<DownlinkMessage>,
    uplink_tx: Sender<UplinkMessage>,

    serial_port: Option<String>,
    serial_status: SerialStatus,

    telemetry_log_path: PathBuf,
    telemetry_log_file: Result<File, std::io::Error>,

    message_receipt_times: VecDeque<(Instant, u32)>,
    siphasher: SipHasher,
    next_mac: (u32, u64),
    last_time: Option<Instant>,
}

impl SerialDataSource {
    /// Create a new serial port data source.
    pub fn new() -> Self {
        let (downlink_tx, downlink_rx) = std::sync::mpsc::channel::<DownlinkMessage>();
        let (uplink_tx, uplink_rx) = std::sync::mpsc::channel::<UplinkMessage>();
        let (serial_status_tx, serial_status_rx) = std::sync::mpsc::channel::<(SerialStatus, Option<String>)>();

        #[cfg(not(target_arch = "wasm32"))] // TODO: can't spawn threads on wasm
        spawn_downlink_monitor(serial_status_tx, downlink_tx, uplink_rx, true);

        let telemetry_log_path = Self::new_telemetry_log_path();
        let telemetry_log_file = File::create(&telemetry_log_path);

        Self {
            serial_status_rx,
            downlink_rx,
            uplink_tx,
            serial_port: None,
            serial_status: SerialStatus::Init,
            telemetry_log_path,
            telemetry_log_file,
            message_receipt_times: VecDeque::new(),
            siphasher: SipHasher::new_with_key(&euroc_fc_firmware::telemetry::SIPHASHER_KEY),
            next_mac: (0, 0),
            last_time: None,
        }
    }

    /// No telemetry file needed because serial port does not work on
    /// web assembly. TODO: maybe create a NoopDataSource for wasm instead?
    #[cfg(target_arch = "wasm32")]
    fn new_telemetry_log_path() -> PathBuf {
        "telem.log".into()
    }

    /// Comes up with a new, unique path for a telemetry log file.
    #[cfg(not(target_arch = "wasm32"))] // TODO: time doesn't work on wasm
    fn new_telemetry_log_path() -> PathBuf {
        let now: chrono::DateTime<chrono::Utc> = std::time::SystemTime::now().into();

        #[cfg(not(target_os = "windows"))]
        {
            // TODO: put these in XDG directories?
            let name = format!("sam_log_{}.log", now.format("%+"));
            name.into()
        }

        #[cfg(target_os = "windows")]
        {
            let name = format!("sam_log_{}.log", now.format("%Y-%m-%dT%H%M%S"));
            let mut path = home::home_dir().unwrap();
            path.push::<PathBuf>(name.into());
            path
        }
    }

    /// Stores a received message in the telemetry log.
    fn write_to_telemetry_log(&mut self, msg: &DownlinkMessage) {
        // TODO
        if let Ok(f) = self.telemetry_log_file.as_mut() {
            let serialized = msg.serialize().unwrap_or_default(); // TODO
            if let Err(e) = f.write_all(&serialized) {
                error!("Error saving msg: {:?}", e);
            }
        }

    }

    /// Update message authentication code. TODO: refactor
    fn update_mac(&mut self) {
        // predict current time on vehicle;
        let t = self
            .message_receipt_times
            .iter()
            .last()
            .map(|(i, t)| t + i.elapsed().as_millis() as u32)
            .unwrap_or(0);

        if self.next_mac.0 > (t + 500) || (self.next_mac.0 + 500) < t {
            self.next_mac = (0, 0);
            self.siphasher = SipHasher::new_with_key(&euroc_fc_firmware::telemetry::SIPHASHER_KEY);
        }

        while self.next_mac.0 < t || !self.is_uplink_window(self.next_mac.0) {
            self.next_mac.0 += LORA_MESSAGE_INTERVAL;
            self.next_mac.1 = self.siphasher.finish();
            self.siphasher.write_u64(self.next_mac.1);
        }
    }

    /// Determines if the given time is an uplink window.
    /// TODO: should be unnecessary after MAC refactor
    fn is_uplink_window(&self, time: u32) -> bool {
        (time % LORA_UPLINK_INTERVAL) == LORA_UPLINK_MODULO
    }
}

/// A data source based on a logfile, either passed as a file path, or with
/// some raw bytes.
pub struct LogFileDataSource {
    path: Option<PathBuf>,
    name: Option<String>,
    file: Option<File>,
    buffer: Vec<u8>,
    messages: Vec<(Instant, DownlinkMessage)>,
    is_json: bool,
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
            last_time: None,
            replay,
        }
    }
}

/// Trait shared by all data sources.
pub trait DataSource {
    /// New messages. TODO: replace with iterator?
    fn next_messages<'a>(&'a mut self) -> Vec<(Instant, DownlinkMessage)>;
    /// Reset data source if applicable.
    fn reset(&mut self);
    /// Calculate the next message authentication code. TODO: refactor
    fn next_mac(&self) -> u64;
    /// Send an uplink message to the connected device if applicable.
    /// TODO: maybe replace with command enum later, and handle details
    /// like MACs internally
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>>;
    /// Return the color and content of the status indicator (e.g. for
    /// connection status)
    fn status(&self) -> (Color32, String);
    /// Return the content of the gray status text (e.g. file path).
    fn info_text(&self) -> String;
    /// The minimum fps required for the data source. Occasional redraws
    /// are necessary if data source is live.
    fn minimum_fps(&self) -> Option<u64>;
    /// Is the data source a file. TODO: a bit unelegant
    fn is_log_file(&self) -> bool;
    fn end(&self) -> Option<Instant>;
}

impl DataSource for SerialDataSource {
    fn next_messages<'a>(&'a mut self) -> Vec<(Instant, DownlinkMessage)> {
        self.update_mac();

        self.message_receipt_times
            .retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        for (status, port) in self.serial_status_rx.try_iter() {
            self.serial_status = status;
            self.serial_port = port;
        }

        let msgs: Vec<_> = self.downlink_rx.try_iter().collect();
        msgs.into_iter()
            .map(|msg| {
                self.write_to_telemetry_log(&msg);

                match msg {
                    DownlinkMessage::Log(_, _, _, _) => {},
                    DownlinkMessage::TelemetryGCS(_) => {},
                    _ => {
                        self.message_receipt_times.push_back((Instant::now(), msg.time()));
                    }
                }

                let now = Instant::now();
                self.last_time = Some(now);
                (now, msg)
            })
            .collect()
    }

    fn reset(&mut self) {
        self.telemetry_log_path = Self::new_telemetry_log_path();
        self.telemetry_log_file = File::create(&self.telemetry_log_path);
        self.message_receipt_times.truncate(0);
        self.siphasher = SipHasher::new_with_key(&euroc_fc_firmware::telemetry::SIPHASHER_KEY);
        self.next_mac = (0, 0);
    }

    fn next_mac(&self) -> u64 {
        self.next_mac.1
    }

    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        self.uplink_tx.send(msg)
    }

    fn status(&self) -> (Color32, String) {
        match self.serial_status {
            SerialStatus::Init => (Color32::from_rgb(0x92, 0x83, 0x74), "Not connected".to_string()),
            SerialStatus::Connected => (Color32::from_rgb(0x98, 0x97, 0x1a), "Connected".to_string()),
            SerialStatus::Error => (Color32::from_rgb(0xcc, 0x24, 0x1d), "Error".to_string()),
        }
    }

    fn info_text(&self) -> String {
        let serial_info = if self.serial_status == SerialStatus::Connected {
            format!(
                "{} (1s: {})",
                self.serial_port.as_ref().unwrap_or(&"".to_string()),
                self.message_receipt_times.len()
            )
        } else {
            self.serial_port.clone().unwrap_or("".to_string())
        };

        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        format!("{} {}", serial_info, telemetry_log_info)
    }

    fn minimum_fps(&self) -> Option<u64> {
        if self.last_time.map(|t| t.elapsed() > Duration::from_secs_f64(10.0)).unwrap_or(false) {
            Some(1)
        } else {
            Some(u64::max(30, u64::min(self.message_receipt_times.len() as u64, 60)))
        }
    }

    fn is_log_file(&self) -> bool {
        false
    }

    fn end(&self) -> Option<Instant> {
        let postroll = Duration::from_secs_f64(10.0);

        self.last_time.map(|t| {
            if t.elapsed() < postroll {
                Instant::now()
            } else {
                t + postroll
            }
        })
    }
}

impl DataSource for LogFileDataSource {
    fn next_messages<'a>(&'a mut self) -> Vec<(Instant, DownlinkMessage)> {
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
            self.buffer.split_mut(|b| *b == 0x00)
                .filter_map(|b| postcard::from_bytes_cobs(b).ok())
                .collect()
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

            // time value contained in the msg in ms compared to the last, but always >= 0
            let mut since_previous = last.map(|l| msg.time().saturating_sub(*l)).unwrap_or(0);

            // The GCS msgs are sent by the ground station, immediately after the received
            // message. The time value used in those is the runtime of the GCS, so always
            // replace with 0.
            if msg.gcs_lora_rssi().is_some() {
                since_previous = 0;
            }

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

        self.messages.drain(..pointer).collect()
    }

    fn reset(&mut self) {
    }

    fn next_mac(&self) -> u64 {
        0xffffffff
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn status(&self) -> (Color32, String) {
        (Color32::from_rgb(0x45, 0x85, 0x88), "Log File".to_string())
    }

    fn info_text(&self) -> String {
        self.path.as_ref()
            .map(|p| p.as_os_str().to_string_lossy().into())
            .or(self.name.clone())
            .unwrap_or_default()
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
}
