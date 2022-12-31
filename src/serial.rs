//! Contains code for interacting with the flight computer via USB serial port.

use std::io::{Read, Write};
use std::sync::mpsc::{Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use euroc_fc_firmware::telemetry::*;

pub const BAUD_RATE: u32 = 115_200;
pub const MESSAGE_TIMEOUT: Duration = Duration::from_millis(500);
pub const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(500);

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
    let mut port = serialport::new(port, BAUD_RATE)
        .timeout(std::time::Duration::from_millis(1))
        .open_native()?;

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
                _ => return Err(e.into())
            }
        }

        // If there exists a zero in our downlink_buffer, that suggests there
        // is a complete COBS-encoded message in there
        while let Some(index) = downlink_buffer.iter().position(|b| *b == 0) {
            // Split of the first message, including the zero delimiter
            let (serialized, rest) = downlink_buffer.split_at_mut(index+1);
            let mut serialized = serialized.to_vec();

            // Store the rest in the downlink_buffer, after having removed
            // the current message
            downlink_buffer = rest.to_vec();

            // Attempt to parse the message, discarding it if unsuccessful
            let msg = match postcard::from_bytes_cobs(serialized.as_mut_slice()) {
                Ok(msg) => msg,
                Err(_e) => continue
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
        .map(|ports| ports.iter()
            .find_map(|p| {
                if let serialport::SerialPortType::UsbPort(_) = p.port_type {
                    Some(p.port_name.clone())
                } else {
                    None
                }
            }))
        .flatten()
}

/// Continuously monitors for connected USB serial devices and connects to them.
/// Run in a separate thread using `spawn_downlink_monitor`.
#[cfg(not(target_arch = "wasm32"))] // TODO: serial ports on wasm?
fn downlink_monitor(
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

        // While no device is connected, we spend most of our time waiting for a
        // serial port, CPU usage is mostly tied to the speed of this check.
        // This check much faster than `serialport::available_ports`, which
        // calls libudev, so this improves CPU usage and battery life somewhat.
        #[cfg(target_os = "linux")]
        while !std::path::PathBuf::from("/dev/ttyACM0").exists() {
            std::thread::sleep(Duration::from_millis(100));
        }

        // No such mechanism on other OSs at the moment.
        #[cfg(not(target_os = "linux"))]
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
