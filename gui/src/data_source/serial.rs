//! A serial port data source. The default.

use std::any::Any;
use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::pin::{pin, Pin};
use std::slice::Iter;
use std::thread::JoinHandle;
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use tokio_serial::SerialStream;
use futures::future::select;
use tokio::sync::mpsc::{Sender, Receiver};
use tokio::sync::mpsc::error::SendError;
use tokio_serial::SerialPortBuilderExt;
use tokio::io::{ReadHalf, AsyncReadExt, AsyncWriteExt, WriteHalf};

use eframe::epaint::Color32;
use log::*;

use shared_types::settings::*;
use shared_types::telemetry::*;

use crate::data_source::DataSource;
use crate::settings::AppSettings;

pub const BAUD_RATE: u32 = 115_200;

// For Android, the Java wrapper has to handle the actual serial port and
// we use these questionable methods to pass the data in via JNI
#[cfg(target_os="android")]
pub static mut DOWNLINK_MESSAGE_RECEIVER: Option<Receiver<(Instant, DownlinkMessage)>> = None;
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
    port_name: String,
) -> Result<(), Box<dyn std::error::Error>> {
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

    select(
        pin!(run_downlink(ctx, downlink_tx, reader)),
        pin!(run_uplink(uplink_rx, writer))
    ).await;

    info!("Closing {:?}", port_name);
    Ok(())
}

/// Returns vector containing names of all available USB ports
pub fn find_serial_ports() -> Vec<String> {
    tokio_serial::available_ports()
        .unwrap_or_default()
        .iter()
        .filter_map(|p| {
            if let tokio_serial::SerialPortType::UsbPort(info) = p.port_type.clone() {
                info.manufacturer.map(|m| m == "TUDSaT")
                    .unwrap_or(false)
                    .then_some(p.port_name.clone())
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
    mut downlink_tx: Sender<(Instant, DownlinkMessage)>,
    mut uplink_rx: Receiver<UplinkMessage>,
    serial_index: usize
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let ports: Vec<_> = find_serial_ports();
        if ports.len() > 0 {
            serial_status_tx.send((SerialStatus::Connected, Some(ports[serial_index].clone()))).await?;
            if let Err(e) = run_port(ctx.clone(), &mut downlink_tx, &mut uplink_rx, ports[serial_index].clone()).await {
                eprintln!("Failed to open {}: {:?}", ports[serial_index], e);
                serial_status_tx.send((SerialStatus::Error, Some(ports[serial_index].clone()))).await?;
                if let Some(ctx) = &ctx {
                    ctx.request_repaint();
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
    downlink_tx: Sender<(Instant, DownlinkMessage)>,
    uplink_rx: Receiver<UplinkMessage>,
    serial_index: usize,
) -> JoinHandle<()> {
    std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
        rt.block_on(downlink_monitor(ctx, serial_status_tx, downlink_tx, uplink_rx, serial_index)).unwrap()
    })
}

pub struct SerialDataSource {
    serial_status_rx: Receiver<(SerialStatus, Option<String>)>,
    downlink_rx: Receiver<(Instant, DownlinkMessage)>,
    uplink_tx: Sender<UplinkMessage>,

    serial_port: Option<String>,
    serial_index: usize,
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
    pub fn new(ctx: &egui::Context, lora_settings: LoRaSettings) -> Self {
        let (downlink_tx, downlink_rx) = tokio::sync::mpsc::channel::<(Instant, DownlinkMessage)>(10);
        let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel::<UplinkMessage>(10);
        let (serial_status_tx, serial_status_rx) = tokio::sync::mpsc::channel::<(SerialStatus, Option<String>)>(10);

        let ctx = ctx.clone();

        let telemetry_log_path = Self::new_telemetry_log_path();
        let telemetry_log_file = File::create(&telemetry_log_path);

        // There are no serial ports on wasm, and on android the Java side handles this.
        #[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
        spawn_downlink_monitor(Some(ctx), serial_status_tx, downlink_tx, uplink_rx, 0);

        Self {
            serial_status_rx,
            downlink_rx,
            uplink_tx,
            serial_port: None,
            serial_index: 0,
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
        while let Ok((status, port)) = self.serial_status_rx.try_recv() {
            self.serial_status = status;
            self.serial_port = port;

            if self.serial_status == SerialStatus::Connected {
                self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
            }
        }

        #[cfg(not(target_os="android"))]
        let mut msgs: Vec<_> = Vec::new();
        #[cfg(not(target_os="android"))]
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
                self.message_receipt_times.push_back((Instant::now(), msg.time()));
            }

            match msg {
                DownlinkMessage::Log(..) => {}
                DownlinkMessage::Settings(settings) => {
                    self.fc_settings = Some(settings);
                }
                _ => {
                    let vs: VehicleState = msg.into();
                    self.vehicle_states.push((t, vs.clone()));
                    self.last_time = Some(t);
                }
            }
        }
    }

    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
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
        self.vehicle_states.truncate(0);
        self.fc_settings = None;
        self.message_receipt_times.truncate(0);
    }

    #[cfg(not(any(target_arch = "wasm32", target_os="android")))]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        self.uplink_tx.blocking_send(msg)
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
            .find_map(|(_t, msg)| msg.data_rate)
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

        #[cfg(not(target_arch = "wasm32"))]
        {
            find_serial_ports().iter().enumerate().for_each(|arg|{
                if ui.button(arg.1).clicked(){
                    self.serial_index = arg.0;
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
        ui.label(self.fc_settings().map(|s| s.identifier.clone()).unwrap_or_default());

        let serial_port = self.serial_port.clone().unwrap_or("".to_string());
        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        ui.weak(format!("{} {}", serial_port, telemetry_log_info));
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
