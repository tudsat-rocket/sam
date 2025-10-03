use std::collections::VecDeque;
use std::error;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;

use std::time::Instant;

use serde::de::Expected;
use telemetry::MessageDefinition;
use telemetry::Metric;
use telemetry::Representation;
use telemetry::TelemetrySchema;
use tokio::sync::mpsc::error::SendError;
use tokio::sync::mpsc::{Receiver, Sender};

use log::*;

use shared_types::settings::*;
use shared_types::telemetry::*;
use telemetry::{LORA_SCHEMA, USB_SCHEMA};

use crate::backend::BackendVariant;
use crate::backend::storage::store::DataStore;
use crate::settings::AppSettings;

use super::storage::static_metrics::PressureSensorId;

pub struct UdpBackend {
    downlink_rx: Receiver<(Instant, DownlinkMessage)>,
    uplink_tx: Sender<UplinkMessage>,

    telemetry_log_path: PathBuf,
    telemetry_log_file: Result<File, std::io::Error>,

    data_store: DataStore,
    fc_settings: Option<Settings>,

    // DownlinkMessage receipt time
    // 0: Intant, when message was received it sam
    // 1: time when message was sent by FC, (for now) in [ms] since boot (inaccurate)
    message_receipt_times: VecDeque<(Instant, u32)>,
    // The last command sent to the FC, we use this the rudimentarily prevent fast command
    // repetitions
    last_command: Option<(Instant, Command)>,
    // TODO: fix
    last_read_settings: Option<Instant>,
}

impl UdpBackend {
    /// Create a new serial port data source.
    pub fn new(ctx: &egui::Context) -> Self {
        let (downlink_tx, downlink_rx) = tokio::sync::mpsc::channel::<(Instant, DownlinkMessage)>(10);
        let (uplink_tx, uplink_rx) = tokio::sync::mpsc::channel::<UplinkMessage>(10);

        let ctx = ctx.clone();

        let telemetry_log_path = Self::new_telemetry_log_path();
        let telemetry_log_file = File::create(&telemetry_log_path);

        std::thread::Builder::new()
            .name("sam-udp".to_owned())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread().enable_io().enable_time().build().unwrap();
                if let Err(e) = rt.block_on(run_socket(Some(ctx), downlink_tx, uplink_rx)) {
                    error!("UDP socket thread has ended because of a fatal error, please restart the app.");
                }
            })
            .unwrap();

        Self {
            downlink_rx,
            uplink_tx,
            telemetry_log_path,
            telemetry_log_file,
            data_store: DataStore::default(),
            fc_settings: None,
            message_receipt_times: VecDeque::new(),
            last_command: None,
            last_read_settings: None,
        }
    }

    /// Comes up with a new, unique path for a telemetry log file.
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

    fn send_ignore_error(&mut self, msg: UplinkMessage) {
        log::info!("Sending {:?}", msg);
        match self.send(msg) {
            Ok(..) => (),
            Err(e) => log::warn!("Failed to send uplink message: {}", e),
        }
    }
}

// enum SchemaInterpretError {
//     MissmatchedTimeOffsets,
//     ReprNotByteAlligned,
//     BufferToShort,
//     BufferToLong,
// }

// Schema reader
fn interpret_buffer_with_schema(schema: &TelemetrySchema, buffer: &[u8], fc_time: u32) -> Result<(), ()> {
    // let mut res = Vec::with_capacity(schema.0.iter().map(|(m, _, _)| m.0.len()).sum());
    let count: usize = 0;
    let mut bit_pointer: usize = 0;

    let mut msg_def: Option<&MessageDefinition> = None;
    for (m, period, offset) in schema.0 {
        if fc_time % *period == *offset {
            msg_def = Some(m);
        }
    }
    let Some(msg_def) = msg_def else {
        // error!("Interpretation as schema: {:?} failed because of mismatched time offsets", schema);
        return Err(());
    };
    let expected_size_bits: usize = msg_def.0.iter().map(|(_, repr)| repr.bits()).sum();
    let expected_size = expected_size_bits / 8;
    // info!("expected_size: {} bytes ({} bits), buffer size: {}", expected_size, expected_size_bits, buffer.len());
    // info!("detected MessageDefinition: {:?}", msg_def);
    if expected_size != buffer.len() {
        // error!("MessageDefinition does not match buffer lenght");
        return Err(());
    }

    for (metr, repr) in msg_def.0 {
        //Correct Bit pointer alignment
        if bit_pointer % 8 != 0 {
            warn!("BUG: Reading MessageDefinition some metric was not byte alignment, this is not allowed.");
            bit_pointer += 8 - bit_pointer % 8;
        }

        //Check if the representation is valid
        if repr.bits() % 8 != 0 || repr.bits() > 64 {
            // error!("Interpretation as schema failed because some representation was (comp time) invalid");
            return Err(());
        }

        //Check that buffer contains a complete value
        if ((bit_pointer + repr.bits()) / 8) > buffer.len() {
            // error!("Interpretation as schema failed because buffer was shorter than MessageDefinition required");
            return Err(());
        }

        //Read as u64 by applying padding if necessary
        let mut bytes = [0; 8];
        bytes[8 - repr.bits() / 8..].copy_from_slice(&buffer[(bit_pointer / 8)..((bit_pointer + repr.bits()) / 8)]);
        let value = u64::from_be_bytes(bytes);

        //Advance bit pointer
        bit_pointer += repr.bits();

        // res.push(value);
    }
    // if count != msg_def.0.len() {
    //     error!("Interpretation as schema failed because buffer was longer than schema suggested");
    //     return Err(());
    // }
    // if res.len() != schema.0.len() {
    //     error!("Interpretation as schema failed because buffer was longer than schema suggested");
    //     return Err(());
    // }
    // Ok(res)
    Ok(())
}

impl BackendVariant for UdpBackend {
    fn update(&mut self, _ctx: &egui::Context) {
        self.message_receipt_times.retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        #[cfg(not(target_os = "android"))]
        let mut msgs: Vec<_> = Vec::new();
        #[cfg(not(target_os = "android"))]
        while let Ok(msg) = self.downlink_rx.try_recv() {
            msgs.push(msg);
        }

        for (t, msg) in msgs.into_iter() {
            // log::info!("received message: {:?}", msg);
            if let (Some(last_time), Some(msg_time)) = (self.message_receipt_times.back(), msg.time_produced()) {
                if msg_time < last_time.1 {
                    log::warn!("Recieved out of order downlink message, dropping.\nMsg: {:?}", msg);
                    continue;
                }
            }

            // TODO: save instant to log as well?
            // (richer log format in general?
            self.write_to_telemetry_log(&msg);

            if let DownlinkMessage::TelemetryGCS(..) = msg {
            } else {
                if let Some(fc_time) = msg.time_produced() {
                    self.message_receipt_times.push_back((t, fc_time));
                }
            }

            match msg {
                DownlinkMessage::Settings(settings) => {
                    self.fc_settings = Some(settings);
                }
                DownlinkMessage::Telemetry(time, message) if message.len() != 0 => {
                    if interpret_buffer_with_schema(&LORA_SCHEMA, message.as_slice(), time).is_ok() {
                        self.data_store.ingest_message(&LORA_SCHEMA, time, message);
                    } else if interpret_buffer_with_schema(&USB_SCHEMA, message.as_slice(), time).is_ok() {
                        self.data_store.ingest_message(&USB_SCHEMA, time, message);
                    } else {
                        error!("Could not identify DownlinkMessage as LORA_SCHEMA nor as USB_SCHEMA, ignoring");
                    }
                }
                _ => {
                    // TODO: metrics via downlink message
                    //let vs: VehicleState = msg.into();
                    //self.vehicle_states.push((t, vs.clone()));
                }
            }
        }

        // FIXME: add rate limiting
        if self.fc_settings.is_none() && self.fc_time().is_some() {
            if let Some(last_instant) = self.last_read_settings {
                if last_instant.elapsed() < Duration::from_secs(1) {
                    // TODO: early return ugly
                    return;
                }
            }
            self.send_ignore_error(UplinkMessage::ReadSettings);
            self.last_read_settings = Some(Instant::now());
        }
    }

    fn data_store<'a>(&'a self) -> &'a DataStore {
        &self.data_store
    }

    fn data_store_mut<'a>(&'a mut self) -> &'a mut DataStore {
        &mut self.data_store
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

    /// Tries to send an UplinkMessage via UDP.
    /// An Ok() result does *not* indicate a successful send.
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        log::info!("Sending {:?}", msg);
        self.uplink_tx.blocking_send(msg)
    }

    /// Tries to send an UplinkMessage of type Command via UDP.
    /// An Ok() result does *not* indicate a successful send.
    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        println!("{:?}", cmd);
        if let Some((t, last_cmd)) = self.last_command.as_ref() {
            if last_cmd == &cmd && t.elapsed() < Duration::from_millis(100) {
                return Ok(());
            }
        }

        self.last_command = Some((Instant::now(), cmd.clone()));
        self.send(UplinkMessage::Command(cmd))
    }

    // returns approximate time of the flight computer since boot minus the signal transmission
    // time
    fn fc_time(&self) -> Option<f64> {
        let postroll = Duration::from_secs_f64(10.0);
        // TODO: (Felix) explain postroll
        let Some((time_received, time_sent)) = self.message_receipt_times.back() else {
            return None;
        };

        let current_time_fc = if time_received.elapsed() < postroll {
            ((*time_sent as f64) / 1000.0) + time_received.elapsed().as_secs_f64()
        } else {
            ((*time_sent as f64) / 1000.0) + postroll.as_secs_f64()
        };

        Some(current_time_fc)
    }

    /// TODO: this function name is highly misleading.
    fn apply_settings(&mut self, settings: &AppSettings) {
        if let Err(e) = self.send(UplinkMessage::ApplyLoRaSettings(settings.lora.clone())) {
            println!("Error trying to apply settings: {}", e);
        }
    }

    fn link_quality(&self) -> Option<f32> {
        let percentage = ((self.message_receipt_times.len() as f32) / (500 as f32)) * 100.0;
        Some(f32::min(percentage, 100.0))
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        if ui.button("⏮  Reset").clicked() {
            self.reset();
        }

        ui.separator();

        {
            ui.separator();
        }
        ui.label(self.fc_settings().map(|s| s.identifier.to_string()).unwrap_or_default());

        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        ui.weak(telemetry_log_info);
    }
}

pub async fn run_socket(
    ctx: Option<egui::Context>,
    downlink_tx: Sender<(Instant, DownlinkMessage)>,
    mut uplink_rx: Receiver<UplinkMessage>,
) -> Result<(), Box<dyn std::error::Error>> {
    let socket = tokio::net::UdpSocket::bind("0.0.0.0:18355").await?;
    println!("socket bind");

    let mut buf = [0; 1024];
    let mut peer = None;

    loop {
        tokio::select! {
            // send uplink message via udp
            Some(msg) = uplink_rx.recv() => {
                println!("Sending {:?} over UDP", msg);
                if let Some(addr) = peer {
                    match msg.serialize() {
                        Ok(s) => {let _ = socket.send_to(&s, addr).await;},
                        Err(e) => log::error!("Failed to serialize a UplinkMessage, it probably didn't  fit in the buffer."),
                    }

                } else {
                    warn!("Can't send UplinkMessage via UDP because destination ip address is unknown, dropping");
                }
            },
            // receive message via udp
            result = socket.recv_from(&mut buf) => {
                let Ok((len, addr)) = result else {
                    log::warn!("Error whilst trying to read UDP packet: {}", result.unwrap_err());
                    continue;
                };
                peer = Some(addr);

                let Ok(msg) = postcard::from_bytes_cobs(&mut buf[..(len as usize)]) else {
                    info!("Serialization of DownlinkMessage");
                    continue;
                };
                if let Err(e) = downlink_tx.send((Instant::now(), msg)).await {
                    log::warn!("Could not send message between threads: {}", e);
                }

                if let Some(ctx) = &ctx {
                    ctx.request_repaint();
                }
            }
        }
    }
}
