//! A serial port data source. The default.

use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::pin::{pin, Pin};
use std::thread::JoinHandle;
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use futures::{select, FutureExt};
use tokio::io::{AsyncReadExt, AsyncWriteExt, ReadHalf, WriteHalf};
use tokio::sync::mpsc::error::SendError;
use tokio::sync::mpsc::{Receiver, Sender};
use tokio_serial::SerialPortBuilderExt;
use tokio_serial::SerialStream;

use eframe::epaint::Color32;
use log::*;

use shared_types::settings::*;
use shared_types::telemetry::*;

use crate::backend::BackendVariant;
use crate::settings::AppSettings;
use crate::DataStore;

pub const BAUD_RATE: u32 = 115_200;

// For Android, the Java wrapper has to handle the actual serial port and
// we use these questionable methods to pass the data in via JNI
#[cfg(target_os = "android")]
pub static mut DOWNLINK_MESSAGE_RECEIVER: Option<Receiver<(Instant, DownlinkMessage)>> = None;
#[cfg(target_os = "android")]
pub static mut UPLINK_MESSAGE_SENDER: Option<Sender<UplinkMessage>> = None;
#[cfg(target_os = "android")]
pub static mut SERIAL_STATUS_RECEIVER: Option<Receiver<SerialStatus>> = None;

/// The current state of our downlink monitoring thread
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialStatus {
    Init,
    Connected,
    Error,
}

#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub async fn run_downlink(
    ctx: Option<egui::Context>,
    downlink_tx: &mut Sender<(Instant, DownlinkMessage)>,
    mut reader: Pin<&mut ReadHalf<SerialStream>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut downlink_buffer: Vec<u8> = Vec::new();

    loop {
        let mut buffer = [0u8; 1024];
        let n = reader.read(&mut buffer).await?;
        if n == 0 {
            return Ok(());
        }

        downlink_buffer.extend(&buffer[..n]);

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
            downlink_tx.send((Instant::now(), msg)).await?;
            if let Some(ctx) = &ctx {
                ctx.request_repaint();
            }
        }
    }
}

#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub async fn run_uplink(
    uplink_rx: &mut Receiver<UplinkMessage>,
    mut writer: Pin<&mut WriteHalf<SerialStream>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let msg = uplink_rx.recv().await.ok_or("".to_string())?;
        let serialized = msg.serialize()?;

        let mut written = 0;
        while written < serialized.len() {
            written += writer.write(&serialized[written..]).await?;
        }

        writer.flush().await?;
    }
}

/// Opens the given serial port, reads downlink messages to `downlink_tx`,
/// and writes uplink messages from `uplink_rx` to the device.
///
/// If `send_heartbeats` is set, regular heartbeat messages will be sent to
/// the device. If no heartbeats are sent, the device will not send log
/// messages.
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub async fn run_port(
    ctx: Option<egui::Context>,
    downlink_tx: &mut Sender<(Instant, DownlinkMessage)>,
    uplink_rx: &mut Receiver<UplinkMessage>,
    serial_status_rx: &mut Receiver<(SerialStatus, Option<String>)>,
    port_name: String,
) -> Result<Option<String>, Box<dyn std::error::Error>> {
    // Open the serial port

    let mut port = tokio_serial::new(&port_name, BAUD_RATE)
        .timeout(std::time::Duration::from_millis(1000))
        .open_native_async()?;

    // Non-exclusive access only works on Unix
    #[cfg(target_family = "unix")]
    port.set_exclusive(false)?;

    let (reader, writer) = tokio::io::split(port);

    let reader = std::pin::pin!(reader);
    let writer = std::pin::pin!(writer);

    let mut downlink = pin!(run_downlink(ctx, downlink_tx, reader).fuse());
    let mut uplink = pin!(run_uplink(uplink_rx, writer).fuse());
    let mut port_selection = pin!(serial_status_rx.recv().fuse());

    select!(
        _ = downlink => Ok(None),
        _ = uplink => Ok(None),
        res = port_selection => {
            if let Some((_, Some(p))) = res {
                Ok(Some(p))
            } else {
                Ok(None)
            }
        }
    )
}

/// Returns vector containing names of all available USB ports
pub fn find_serial_ports() -> Vec<String> {
    let mut ports = tokio_serial::available_ports().unwrap_or_default();
    ports.sort_by_key(|p| {
        let is_tudsat = if let tokio_serial::SerialPortType::UsbPort(info) = &p.port_type {
            info.manufacturer.as_ref().map(|m| m.to_lowercase() == "tudsat").unwrap_or(false)
        } else {
            false
        };

        (!is_tudsat) as u8
    });

    ports
        .iter()
        .filter_map(|p| {
            if let tokio_serial::SerialPortType::UsbPort(_info) = &p.port_type {
                Some(p.port_name.clone())
            } else {
                None
            }
        })
        .collect()
}

/// Continuously monitors for connected USB serial devices and connects to them.
/// Run in a separate thread using `spawn_downlink_monitor`.
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub async fn downlink_monitor(
    ctx: Option<egui::Context>,
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    mut serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    mut downlink_tx: Sender<(Instant, DownlinkMessage)>,
    mut uplink_rx: Receiver<UplinkMessage>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut selected_port = None;
    let mut ports: Vec<_> = Vec::new();

    loop {
        let new_ports: Vec<_> = find_serial_ports();
        if new_ports != ports {
            if let Some(c) = &ctx {
                c.request_repaint();
            }
        }
        ports = new_ports;

        selected_port = selected_port.clone().or(ports.first().cloned());
        while let Ok((_, Some(p))) = serial_status_rx.try_recv() {
            if ports.contains(&p) {
                selected_port = Some(p);
            }
        }

        if let Some(p) = selected_port.as_ref() {
            if !ports.contains(p) {
                selected_port = None;
                continue;
            }

            serial_status_tx.send((SerialStatus::Connected, Some(p.clone()))).await?;

            info!("Opening port {}", p);
            let res = run_port(ctx.clone(), &mut downlink_tx, &mut uplink_rx, &mut serial_status_rx, p.clone()).await;

            match res {
                Ok(None) => {
                    continue;
                }
                Ok(Some(new_port)) => selected_port = Some(new_port),
                Err(e) => {
                    eprintln!("Failed to open {}: {:?}", p, e);
                    serial_status_tx.send((SerialStatus::Error, Some(p.clone()))).await?;
                    if let Some(ctx) = &ctx {
                        ctx.request_repaint();
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Spawns `downlink_monitor` in a new thread.
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub fn spawn_downlink_monitor(
    ctx: Option<egui::Context>,
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_tx: Sender<(Instant, DownlinkMessage)>,
    uplink_rx: Receiver<UplinkMessage>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("sam-serial".to_owned())
        .spawn(move || {
            let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
            rt.block_on(downlink_monitor(ctx, serial_status_tx, serial_status_rx, downlink_tx, uplink_rx))
                .unwrap()
        })
        .unwrap()
}

pub struct SerialBackend {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    serial_status_uplink_tx: Sender<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<(Instant, DownlinkMessage)>,
    uplink_tx: Sender<UplinkMessage>,

    serial_port: Option<String>,
    serial_status: SerialStatus,

    lora_settings: LoRaSettings,

    telemetry_log_path: PathBuf,
    telemetry_log_file: Result<File, std::io::Error>,

    data_store: DataStore,
    fc_settings: Option<Settings>,

    message_receipt_times: VecDeque<(Instant, u32)>,
    last_command: Option<(Instant, Command)>,
}

impl SerialBackend {
    /// Create a new serial port data source.
    pub fn new(ctx: &egui::Context, lora_settings: LoRaSettings) -> Self {
        let (downlink_tx, downlink_rx) = tokio::sync::mpsc::channel::<(Instant, DownlinkMessage)>(10);
        let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel::<UplinkMessage>(10);
        let (serial_status_tx, serial_status_rx) = tokio::sync::mpsc::channel::<(SerialStatus, Option<String>)>(10);
        let (serial_status_uplink_tx, serial_status_uplink_rx) =
            tokio::sync::mpsc::channel::<(SerialStatus, Option<String>)>(10);

        let ctx = ctx.clone();

        let telemetry_log_path = Self::new_telemetry_log_path();
        let telemetry_log_file = File::create(&telemetry_log_path);

        // There are no serial ports on wasm, and on android the Java side handles this.
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        spawn_downlink_monitor(Some(ctx), serial_status_tx, serial_status_uplink_rx, downlink_tx, uplink_rx);

        Self {
            serial_status_rx,
            serial_status_uplink_tx,
            downlink_rx,
            uplink_tx,
            serial_port: None,
            serial_status: SerialStatus::Init,
            lora_settings,
            telemetry_log_path,
            telemetry_log_file,
            data_store: DataStore::default(),
            fc_settings: None,
            message_receipt_times: VecDeque::new(),
            last_command: None,
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

impl BackendVariant for SerialBackend {
    fn update(&mut self, _ctx: &egui::Context) {
        self.message_receipt_times.retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        #[cfg(target_os = "android")]
        for status in unsafe { SERIAL_STATUS_RECEIVER.as_mut().unwrap().try_iter() } {
            self.serial_status = status;
            self.serial_port = Some("".to_owned());

            if self.serial_status == SerialStatus::Connected {
                self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        #[cfg(not(target_os = "android"))]
        while let Ok((status, port)) = self.serial_status_rx.try_recv() {
            self.serial_status = status;
            self.serial_port = port;

            if self.serial_status == SerialStatus::Connected {
                self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        #[cfg(not(target_os = "android"))]
        let mut msgs: Vec<_> = Vec::new();
        #[cfg(not(target_os = "android"))]
        while let Ok(msg) = self.downlink_rx.try_recv() {
            msgs.push(msg);
        }

        #[cfg(target_os = "android")]
        let msgs: Vec<_> = unsafe { DOWNLINK_MESSAGE_RECEIVER.as_mut().unwrap().try_iter().collect() };

        for (t, msg) in msgs.into_iter() {
            // TODO: save instant to log as well?
            // (richer log format in general?
            self.write_to_telemetry_log(&msg);

            // TODO
            if let DownlinkMessage::TelemetryGCS(..) = msg {
            } else {
                self.message_receipt_times.push_back((t, msg.time()));
            }

            match msg {
                DownlinkMessage::Settings(settings) => {
                    self.fc_settings = Some(settings);
                }
                _ => {
                    // TODO: metrics via downlink message
                    //let vs: VehicleState = msg.into();
                    //self.vehicle_states.push((t, vs.clone()));
                }
            }
        }

        if self.fc_settings.is_none() && self.end().is_some() {
            self.send(UplinkMessage::ReadSettings).unwrap();
        }
    }

    fn data_store<'a>(&'a self) -> &'a DataStore {
        &self.data_store
    }

    fn fc_settings(&mut self) -> Option<&Settings> {
        self.fc_settings.as_ref()
    }

    fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
        self.fc_settings.as_mut()
    }

    fn reset(&mut self) {
        self.telemetry_log_path = Self::new_telemetry_log_path();
        self.telemetry_log_file = File::create(&self.telemetry_log_path);
        self.data_store = DataStore::default();
        self.fc_settings = None;
        self.message_receipt_times.truncate(0);
    }

    #[cfg(not(any(target_arch = "wasm32", target_os = "android")))]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        log::info!("Sending {:?}", msg);
        self.uplink_tx.blocking_send(msg)
    }

    #[cfg(target_os = "android")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        unsafe { UPLINK_MESSAGE_SENDER.as_mut().unwrap().send(msg) }
    }

    #[cfg(target_arch = "wasm32")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        if let Some((t, last_cmd)) = self.last_command.as_ref() {
            if last_cmd == &cmd && t.elapsed() < Duration::from_millis(100) {
                return Ok(());
            }
        }

        self.last_command = Some((Instant::now(), cmd.clone()));
        self.send(UplinkMessage::Command(cmd))
    }

    fn end(&self) -> Option<f64> {
        let postroll = Duration::from_secs_f64(10.0);
        // TODO: postroll
        let Some((time_received, time_message)) = self.message_receipt_times.front() else {
            return None;
        };

        let x = if time_received.elapsed() < postroll {
            ((*time_message as f64) / 1000.0) + time_received.elapsed().as_secs_f64()
        } else {
            ((*time_message as f64) / 1000.0) + postroll.as_secs_f64()
        };

        Some(x)
    }

    fn apply_settings(&mut self, settings: &AppSettings) {
        self.lora_settings = settings.lora.clone();
        self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
    }

    fn link_quality(&self) -> Option<f32> {
        let percentage = ((self.message_receipt_times.len() as f32) / (15 as f32)) * 100.0;
        Some(f32::min(percentage, 100.0))
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        #[cfg(not(target_arch = "wasm32"))]
        if ui.button("⏮  Reset").clicked() {
            self.reset();
        }

        #[cfg(not(target_arch = "wasm32"))]
        ui.separator();

        #[cfg(not(target_arch = "wasm32"))]
        {
            find_serial_ports().iter().for_each(|port| {
                if ui.selectable_label(self.serial_port.as_ref() == Some(port), port).clicked() {
                    self.serial_port = Some(port.clone());
                    self.serial_status_uplink_tx
                        .blocking_send((SerialStatus::Init, Some(port.clone())))
                        .unwrap_or_default();
                    self.reset();
                }
            });
            ui.separator();
        }

        let (status_color, status_text) = match self.serial_status {
            SerialStatus::Init => (Color32::from_rgb(0x92, 0x83, 0x74), "Not connected".to_string()),
            SerialStatus::Connected => (Color32::from_rgb(0x98, 0x97, 0x1a), "Connected".to_string()),
            SerialStatus::Error => (Color32::from_rgb(0xcc, 0x24, 0x1d), "Error".to_string()),
        };

        ui.colored_label(status_color, status_text);
        ui.label(self.fc_settings().map(|s| s.identifier.to_string()).unwrap_or_default());

        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        ui.weak(telemetry_log_info);
    }
}
