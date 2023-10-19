//! Data sources serve as an abstraction for the origin of the displayed data.
//! This data source can either be a serial port device, or an opened log file.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::slice::Iter;
use std::sync::mpsc::{Receiver, SendError, Sender};
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::epaint::Color32;
use log::*;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::serial::*;
use crate::settings::AppSettings;
use crate::simulation::SimulationSettings;
use crate::state::*;

/// A serial port data source. The default.
pub struct SerialDataSource {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<DownlinkMessage>,
    uplink_tx: Sender<UplinkMessage>,

    serial_port: Option<String>,
    serial_status: SerialStatus,

    lora_settings: LoRaSettings,

    telemetry_log_path: PathBuf,
    telemetry_log_file: Result<File, std::io::Error>,

    vehicle_states: Vec<(Instant, VehicleState)>,
    log_messages: Vec<(u32, String, LogLevel, String)>,
    fc_settings: Option<Settings>,
    message_receipt_times: VecDeque<(Instant, u32)>,
    last_time: Option<Instant>,
}

impl SerialDataSource {
    /// Create a new serial port data source.
    pub fn new(lora_settings: LoRaSettings) -> Self {
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
            lora_settings,
            telemetry_log_path,
            telemetry_log_file,
            vehicle_states: Vec::new(),
            log_messages: Vec::new(),
            fc_settings: None,
            message_receipt_times: VecDeque::new(),
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
    vehicle_states: Vec<(Instant, VehicleState)>,
    log_messages: Vec<(u32, String, LogLevel, String)>,
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
            log_messages: Vec::new(),
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
            log_messages: Vec::new(),
            last_time: None,
            replay,
        }
    }
}

/// Trait shared by all data sources.
pub trait DataSource {
    /// Return an iterator over only the states that have arrived since last time.
    fn new_vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)>;
    /// Return an iterator over all known states of the vehicle.
    fn vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)>;
    /// Return an iterator over all log messages
    fn log_messages<'a>(&'a mut self) -> Iter<'_, (u32, String, LogLevel, String)>;
    /// Return the current flight computer settings, if known.
    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings>;
    /// Return the current flight computer settings, if known.
    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings>;

    /// Reset data source if applicable.
    fn reset(&mut self);
    /// Send an uplink message to the connected device if applicable.
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>>;
    /// Send an authenticated uplink command
    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>>;
    /// Return the color and content of the status indicator (e.g. for
    /// connection status)
    fn status(&self) -> (Color32, String);
    /// Return the content of the gray status text (e.g. file path).
    fn info_text(&self) -> String;
    /// The minimum fps required for the data source. Occasional redraws
    /// are necessary if data source is live.
    fn minimum_fps(&self) -> Option<u64>;
    fn end(&self) -> Option<Instant>;

    fn link_quality(&self) -> Option<f32> {
        None
    }

    fn is_log_file(&self) -> bool {
        false
    }

    fn simulation_settings(&mut self) -> Option<&mut SimulationSettings> {
        None
    }

    fn apply_settings(&mut self, _settings: &AppSettings) {}
}

impl DataSource for SerialDataSource {
    fn new_vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
        self.message_receipt_times.retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        for (status, port) in self.serial_status_rx.try_iter() {
            self.serial_status = status;
            self.serial_port = port;

            if self.serial_status == SerialStatus::Connected {
                self.uplink_tx.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        let msgs: Vec<_> = self.downlink_rx.try_iter().collect();

        let start_i = self.vehicle_states.len();
        for msg in msgs.into_iter() {
            self.write_to_telemetry_log(&msg);

            // TODO
            if let DownlinkMessage::TelemetryGCS(..) = msg {
            } else {
                self.message_receipt_times.push_back((Instant::now(), msg.time()));
            }

            match msg {
                DownlinkMessage::Log(t, loc, ll, txt) => {
                    self.log_messages.push((t, loc, ll, txt));
                }
                DownlinkMessage::Settings(settings) => {
                    self.fc_settings = Some(settings);
                }
                _ => {
                    let now = Instant::now();
                    let vs: VehicleState = msg.into();
                    self.vehicle_states.push((now, vs.clone()));
                    self.last_time = Some(now);
                }
            }
        }

        self.vehicle_states[start_i..].iter()
    }

    fn vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn log_messages<'a>(&'a mut self) -> Iter<'_, (u32, String, LogLevel, String)> {
        self.log_messages.iter()
    }

    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings> {
        self.fc_settings.as_ref()
    }

    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings> {
        self.fc_settings.as_mut()
    }

    fn reset(&mut self) {
        self.telemetry_log_path = Self::new_telemetry_log_path();
        self.telemetry_log_file = File::create(&self.telemetry_log_path);
        self.vehicle_states.truncate(0);
        self.log_messages.truncate(0);
        self.fc_settings = None;
        self.message_receipt_times.truncate(0);
    }

    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        self.uplink_tx.send(msg)
    }

    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        self.send(UplinkMessage::Command(cmd))
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

    fn apply_settings(&mut self, settings: &AppSettings) {
        self.lora_settings = settings.lora.clone();
        self.uplink_tx.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
    }

    fn link_quality(&self) -> Option<f32> {
        let telemetry_data_rate = self
            .vehicle_states
            .iter()
            .rev()
            .find_map(|(_t, msg)| msg.telemetry_data_rate)
            .unwrap_or(TelemetryDataRate::Low);
        let expected = match telemetry_data_rate {
            // TODO: don't hardcode these?
            TelemetryDataRate::Low => 15,
            TelemetryDataRate::High => 35,
        };
        let percentage = ((self.message_receipt_times.len() as f32) / (expected as f32)) * 100.0;
        Some(f32::min(percentage, 100.0))
    }
}

impl DataSource for LogFileDataSource {
    fn new_vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
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
                                            //
            self.messages.push((self.last_time.unwrap(), msg));
        }

        let start_i = self.vehicle_states.len();
        let pointer = if self.replay {
            let now = Instant::now();
            self.messages.partition_point(|(t, _)| t <= &now)
        } else {
            self.messages.len()
        };

        for (t, msg) in self.messages.drain(..pointer) {
            self.vehicle_states.push((t, msg.into()));
        }

        self.vehicle_states[start_i..].iter()
    }

    fn vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn log_messages<'a>(&'a mut self) -> Iter<'_, (u32, String, LogLevel, String)> {
        self.log_messages.iter()
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
        self.log_messages.truncate(0);
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn status(&self) -> (Color32, String) {
        (Color32::from_rgb(0x45, 0x85, 0x88), "Log File".to_string())
    }

    fn info_text(&self) -> String {
        self.path
            .as_ref()
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
