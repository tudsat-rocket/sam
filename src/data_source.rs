use core::hash::Hasher;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, Sender, SendError};
use std::time::{Duration, Instant};

use eframe::epaint::Color32;
use siphasher::sip::SipHasher;
use log::*;

use euroc_fc_firmware::telemetry::*;

use crate::serial::*;

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
}

impl SerialDataSource {
    pub fn new() -> Self {
        let (downlink_tx, downlink_rx) = std::sync::mpsc::channel::<DownlinkMessage>();
        let (uplink_tx, uplink_rx) = std::sync::mpsc::channel::<UplinkMessage>();
        let (serial_status_tx, serial_status_rx) = std::sync::mpsc::channel::<(SerialStatus, Option<String>)>();

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
        }
    }

    fn new_telemetry_log_path() -> PathBuf {
        let now: chrono::DateTime<chrono::Utc> = std::time::SystemTime::now().into();
        let name = format!("sam_log_{}.log", now.format("%+"));
        name.into()
    }

    fn write_to_telemetry_log(&mut self, msg: &DownlinkMessage) {
        // TODO
        if let Ok(f) = self.telemetry_log_file.as_mut() {
            if let Err(e) = f.write_all(&msg.wrap()) {
                error!("Error saving msg: {:?}", e);
            }
        }

    }

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

    fn is_uplink_window(&self, time: u32) -> bool {
        (time % LORA_UPLINK_INTERVAL) == LORA_UPLINK_MODULO
    }
}

pub struct LogFileDataSource {
    path: PathBuf,
    file: File,
    buffer: Vec<u8>
}

impl LogFileDataSource {
    pub fn new(path: PathBuf) -> Result<Self, std::io::Error> {
        let file = File::open(&path)?;
        Ok(Self {
            path,
            file,
            buffer: Vec::new()
        })
    }
}

pub trait DataSource {
    fn next_messages(&mut self) -> Vec<DownlinkMessage>;
    fn reset(&mut self);
    fn next_mac(&self) -> u64;
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>>;
    fn status(&self) -> (Color32, String);
    fn info_text(&self) -> String;
}

impl DataSource for SerialDataSource {
    fn next_messages(&mut self) -> Vec<DownlinkMessage> {
        self.update_mac();

        self.message_receipt_times
            .retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        for (status, port) in self.serial_status_rx.try_iter() {
            self.serial_status = status;
            self.serial_port = port;
        }

        let msgs: Vec<DownlinkMessage> = self.downlink_rx.try_iter().collect();
        for msg in msgs.iter() {
            self.write_to_telemetry_log(&msg);

            match msg {
                DownlinkMessage::Log(_, _, _, _) => {},
                DownlinkMessage::TelemetryGCS(_) => {},
                _ => {
                    self.message_receipt_times.push_back((Instant::now(), msg.time()));
                }
            }
        }

        msgs
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
            SerialStatus::Init => (Color32::GRAY, "Init".to_string()),
            SerialStatus::Connected => (Color32::GREEN, "Connected".to_string()),
            SerialStatus::Error => (Color32::RED, "Error".to_string()),
        }
    }

    fn info_text(&self) -> String {
        let serial_info = if self.serial_status == SerialStatus::Connected {
            format!(
                "to {} (1s: {})",
                self.serial_port.as_ref().unwrap_or(&"".to_string()),
                self.message_receipt_times.len()
            )
        } else {
            format!("to {}", self.serial_port.as_ref().unwrap_or(&"".to_string()))
        };

        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        format!("{} {}", serial_info, telemetry_log_info)
    }
}

impl DataSource for LogFileDataSource {
    fn next_messages(&mut self) -> Vec<DownlinkMessage> {
        if let Err(e) = self.file.read_to_end(&mut self.buffer) {
            error!("Failed to read log file: {:?}", e);
        }

        let mut msgs = Vec::new();

        while let Some(msg) = DownlinkMessage::pop_valid(&mut self.buffer) {
            msgs.push(msg);
        }

        msgs
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
        (Color32::LIGHT_BLUE, "Log File".to_string())
    }

    fn info_text(&self) -> String {
        self.path.as_os_str().to_string_lossy().into()
    }
}
