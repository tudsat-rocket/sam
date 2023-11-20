//! A serial port data source. The default.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::slice::Iter;
use std::sync::mpsc::{Receiver, SendError, Sender};
use std::thread::JoinHandle;
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
use crate::settings::AppSettings;
use crate::state::*;

pub const BAUD_RATE: u32 = 115_200;
pub const MESSAGE_TIMEOUT: Duration = Duration::from_millis(500);
pub const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(500);

// For Android, the Java wrapper has to handle the actual serial port and
// we use these questionable methods to pass the data in via JNI
#[cfg(target_os="android")]
pub static mut DOWNLINK_MESSAGE_RECEIVER: Option<Receiver<DownlinkMessage>> = None;
#[cfg(target_os="android")]
pub static mut UPLINK_MESSAGE_SENDER: Option<Sender<UplinkMessage>> = None;
#[cfg(target_os="android")]
pub static mut SERIAL_STATUS_RECEIVER: Option<Receiver<SerialStatus>> = None;

/// The current state of our downlink monitoring thread
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialStatus {
    Init,
    Connected,
    Error,
}

/// Opens the given serial port, reads downlink messages to `downlink_tx`,
/// and writes uplink messages from `uplink_rx` to the device.
///
/// If `send_heartbeats` is set, regular heartbeat messages will be sent to
/// the device. If no heartbeats are sent, the device will not send log
/// messages.
#[cfg(not(target_arch = "wasm32"))] // TODO: serial ports on wasm?
pub fn downlink_port(
    downlink_tx: &mut Sender<DownlinkMessage>,
    uplink_rx: &mut Receiver<UplinkMessage>,
    port: String,
    send_heartbeats: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    // Open the serial port
    let mut port = serialport::new(port, BAUD_RATE).timeout(std::time::Duration::from_millis(1)).open_native()?;

    // Non-exclusive access only works on Unix
    #[cfg(target_family = "unix")]
    port.set_exclusive(false)?;

    let mut downlink_buffer: Vec<u8> = Vec::new();
    let mut now = Instant::now();
    let mut last_heartbeat = Instant::now() - HEARTBEAT_INTERVAL * 2;
    let mut last_message = Instant::now();

    // Stop if no messages are sent, even if a connection exists.
    while now.duration_since(last_message) < MESSAGE_TIMEOUT {
        // Send pending uplink messages, or heartbeats if necessary.
        if let Some(msg) = uplink_rx.try_iter().next() {
            port.write(&msg.serialize().unwrap_or_default())?;
            port.flush()?;
        } else if now.duration_since(last_heartbeat) > HEARTBEAT_INTERVAL && send_heartbeats {
            port.write(&UplinkMessage::Heartbeat.serialize().unwrap())?;
            port.flush()?;
            last_heartbeat = now;
        }

        // Read all available bytes from the serial port. Our timeout is really
        // short (we don't want to block here for too long), so timeouts are
        // common, and simply ignored, and treated like an empty read.
        if let Err(e) = port.read_to_end(&mut downlink_buffer) {
            match e.kind() {
                std::io::ErrorKind::TimedOut => (),
                _ => return Err(e.into()),
            }
        }

        // If there exists a zero in our downlink_buffer, that suggests there
        // is a complete COBS-encoded message in there
        while let Some(index) = downlink_buffer.iter().position(|b| *b == 0) {
            // Split of the first message, including the zero delimiter
            let (serialized, rest) = downlink_buffer.split_at_mut(index + 1);
            let mut serialized = serialized.to_vec();

            // Store the rest in the downlink_buffer, after having removed
            // the current message
            downlink_buffer = rest.to_vec();

            // Attempt to parse the message, discarding it if unsuccessful
            let msg = match postcard::from_bytes_cobs(serialized.as_mut_slice()) {
                Ok(msg) => msg,
                Err(_e) => continue,
            };

            // If successful, send msg through channel.
            downlink_tx.send(msg)?;
            last_message = now;
        }

        now = Instant::now();
    }

    Ok(())
}

/// Finds the first connected USB serial port
pub fn find_serial_port() -> Option<String> {
    serialport::available_ports()
        .ok()
        .map(|ports| {
            ports.iter().find_map(|p| {
                if let serialport::SerialPortType::UsbPort(_) = p.port_type {
                    Some(p.port_name.clone())
                } else {
                    None
                }
            })
        })
        .flatten()
}

/// Continuously monitors for connected USB serial devices and connects to them.
/// Run in a separate thread using `spawn_downlink_monitor`.
#[cfg(not(target_arch = "wasm32"))] // TODO: serial ports on wasm?
pub fn downlink_monitor(
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    mut downlink_tx: Sender<DownlinkMessage>,
    mut uplink_rx: Receiver<UplinkMessage>,
    send_heartbeats: bool,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        // If a device was connected, start reading messages.
        if let Some(p) = find_serial_port() {
            serial_status_tx.send((SerialStatus::Connected, Some(p.clone())))?;
            if let Err(e) = downlink_port(&mut downlink_tx, &mut uplink_rx, p.clone(), send_heartbeats) {
                eprintln!("{:?}", e);
                serial_status_tx.send((SerialStatus::Error, Some(p)))?;
            }
        }

        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Spawns `downlink_monitor` in a new thread.
#[cfg(not(target_arch = "wasm32"))] // TODO: serial ports on wasm?
pub fn spawn_downlink_monitor(
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    downlink_tx: Sender<DownlinkMessage>,
    uplink_rx: Receiver<UplinkMessage>,
    send_heartbeats: bool,
) -> JoinHandle<()> {
    std::thread::spawn(move || {
        downlink_monitor(serial_status_tx, downlink_tx, uplink_rx, send_heartbeats).unwrap_or_default()
    })
}

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

impl DataSource for SerialDataSource {
    fn update(&mut self, _ctx: &egui::Context) {
        self.message_receipt_times.retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        #[cfg(target_os="android")]
        for status in unsafe { SERIAL_STATUS_RECEIVER.as_mut().unwrap().try_iter() } {
            self.serial_status = status;
            self.serial_port = Some("".to_owned());

            if self.serial_status == SerialStatus::Connected {
                self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        #[cfg(not(target_os="android"))]
        for (status, port) in self.serial_status_rx.try_iter().collect::<Vec<_>>().into_iter() {
            self.serial_status = status;
            self.serial_port = port;

            if self.serial_status == SerialStatus::Connected {
                self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        #[cfg(not(target_os = "android"))]
        let msgs: Vec<_> = self.downlink_rx.try_iter().collect();
        #[cfg(target_os = "android")]
        let msgs: Vec<_> = unsafe { DOWNLINK_MESSAGE_RECEIVER.as_mut().unwrap().try_iter().collect() };

        for msg in msgs.into_iter() {
            self.write_to_telemetry_log(&msg);

            // TODO
            if let DownlinkMessage::TelemetryGCS(..) = msg {
            } else {
                self.message_receipt_times.push_back((Instant::now(), msg.time()));
            }

            match msg {
                DownlinkMessage::Log(..) => {}
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
    }

    fn vehicle_states<'a>(&'a self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
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
        self.fc_settings = None;
        self.message_receipt_times.truncate(0);
    }

    #[cfg(not(any(target_arch = "wasm32", target_os="android")))]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        self.uplink_tx.send(msg)
    }

    #[cfg(target_os="android")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        unsafe {
            UPLINK_MESSAGE_SENDER.as_mut().unwrap().send(msg)
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        self.send(UplinkMessage::Command(cmd))
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
        self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
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

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        #[cfg(not(target_arch = "wasm32"))]
        if ui.button("⏮  Reset").clicked() {
            self.reset();
        }

        #[cfg(not(target_arch = "wasm32"))]
        ui.separator();

        let (status_color, status_text) = match self.serial_status {
            SerialStatus::Init => (Color32::from_rgb(0x92, 0x83, 0x74), "Not connected".to_string()),
            SerialStatus::Connected => (Color32::from_rgb(0x98, 0x97, 0x1a), "Connected".to_string()),
            SerialStatus::Error => (Color32::from_rgb(0xcc, 0x24, 0x1d), "Error".to_string()),
        };

        ui.colored_label(status_color, status_text);
        ui.label(self.fc_settings().map(|s| s.identifier.clone()).unwrap_or_default());

        let serial_port = self.serial_port.clone().unwrap_or("".to_string());
        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        ui.weak(format!("{} {}", serial_port, telemetry_log_info));
    }
}
