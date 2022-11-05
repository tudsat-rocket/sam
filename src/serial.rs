use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use euroc_fc_firmware::telemetry::*;

pub const BAUD_RATE: u32 = 115_200;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialStatus {
    Init,
    Connected,
    Error,
}

pub fn find_serial_port() -> Option<String> {
    if let Ok(ports) = serialport::available_ports() {
        for port in ports {
            if let serialport::SerialPortType::UsbPort(_) = port.port_type {
                return Some(port.port_name);
            }
        }
    }

    None
}

pub fn downlink_port(
    downlink_tx: &mut Sender<DownlinkMessage>,
    uplink_rx: &mut Receiver<UplinkMessage>,
    port: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut port = serialport::new(port, BAUD_RATE)
        .timeout(std::time::Duration::from_millis(100))
        .open_native()?;

    let mut buffer: Vec<u8> = vec![0; 1024];
    let mut downlink_buffer: Vec<u8> = Vec::new();

    let mut now = Instant::now();
    let mut last_heartbeat = Instant::now() - Duration::from_millis(1000);
    let mut last_message = Instant::now();

    while now.duration_since(last_message) < Duration::from_millis(500) {
        if let Some(msg) = uplink_rx.try_iter().next() {
            let _written = port.write(&msg.wrap())?;
            port.flush()?;
        } else if now.duration_since(last_heartbeat) > Duration::from_millis(500) {
            let _written = port.write(&UplinkMessage::Heartbeat.wrap())?;
            port.flush()?;
            last_heartbeat = now;
        }

        let read_bytes = match port.read(&mut buffer) {
            Ok(x) => Ok(x),
            Err(e) => match e.kind() {
                std::io::ErrorKind::TimedOut => Ok(0),
                _ => Err(e)
            }
        }?;

        for i in 0..read_bytes {
            if !downlink_buffer.is_empty() || buffer[i] == 0x42 {
                downlink_buffer.push(buffer[i]);
            }
        }

        while let Some(msg) = DownlinkMessage::pop_valid(&mut downlink_buffer) {
            downlink_tx.send(msg)?;
            last_message = now;
        }

        std::thread::sleep(Duration::from_micros(1000));
        now = Instant::now();
    }

    Ok(())
}

fn downlink_monitor(
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    mut downlink_tx: Sender<DownlinkMessage>,
    mut uplink_rx: Receiver<UplinkMessage>
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        if let Some(p) = find_serial_port() {
            serial_status_tx.send((SerialStatus::Connected, Some(p.clone())))?;
            if let Err(e) = downlink_port(&mut downlink_tx, &mut uplink_rx, p.clone()) {
                println!("{:?}", e);
                serial_status_tx.send((SerialStatus::Error, Some(p)))?;
            }
        }
    }
}

pub fn spawn_downlink_monitor(
    serial_status_tx: Sender<(SerialStatus, Option<String>)>,
    downlink_tx: Sender<DownlinkMessage>,
    uplink_rx: Receiver<UplinkMessage>
) -> JoinHandle<()> {
    std::thread::spawn(move || downlink_monitor(serial_status_tx, downlink_tx, uplink_rx).unwrap_or_default())
}
