use std::collections::VecDeque;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::time::Duration;

use std::time::Instant;

use tokio::sync::mpsc::error::SendError;
use tokio::sync::mpsc::{Receiver, Sender};

use log::*;

use shared_types::settings::*;
use shared_types::telemetry::*;
use telemetry::{LORA_SCHEMA, USB_SCHEMA};

use crate::backend::BackendVariant;
use crate::backend::storage::store::DataStore;
use crate::settings::AppSettings;

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
                rt.block_on(run_socket(Some(ctx), downlink_tx, uplink_rx)).unwrap()
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
                    self.data_store.ingest_message(&USB_SCHEMA, time, message);
                    // FIXME: change schema
                    // self.data_store.ingest_message(&LORA_SCHEMA, time, message);

                    //self.data_store.ingest_message(&LORA_SCHEMA, time, message);
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

    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        log::info!("Sending {:?}", msg);
        self.uplink_tx.blocking_send(msg)
    }

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

    fn apply_settings(&mut self, settings: &AppSettings) {
        self.send(UplinkMessage::ApplyLoRaSettings(settings.lora.clone())).unwrap();
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
    let socket = tokio::net::UdpSocket::bind("0.0.0.0:18355").await.unwrap();
    println!("socket bind");

    let mut buf = [0; 1024];
    let mut peer = None;

    loop {
        tokio::select! {
            // send uplink message via udp
            Some(msg) = uplink_rx.recv() => {
                println!("{:?}", msg);
                if let Some(addr) = peer {
                    let serialized = msg.serialize().unwrap();
                    let _ = socket.send_to(&serialized, addr).await;
                }
            },
            // receive message via udp
            result = socket.recv_from(&mut buf) => {
                // info!("Received udp message");
                let (len, addr) = result.unwrap();
                peer = Some(addr);

                let Ok(msg) = postcard::from_bytes_cobs(&mut buf[..(len as usize)]) else {
                    // info!("Postcard serialization of message failed");
                    continue;
                };
                    // info!("Postcard serialization of message successful");

                    // TODO: remove unwrap
                // downlink_tx.send((Instant::now(), msg)).await.unwrap();
                match downlink_tx.send((Instant::now(), msg)).await {
                    Ok(..) => (),
                    Err(e) => log::warn!("Could not send message between threads: {}", e),
                }

                if let Some(ctx) = &ctx {
                    ctx.request_repaint();
                }
            }
        }
    }
}
