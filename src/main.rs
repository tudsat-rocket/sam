#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use std::fs::File;
use std::io::{Error, ErrorKind, Read, Write};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use colored::Colorize;
use crc::{Crc, CRC_16_IBM_SDLC};
use log::*;

use euroc_fc_firmware::telemetry::*;

mod data_source;
mod file;
mod gui;
mod serial;
mod state;
mod telemetry_ext;

use serial::*;

#[derive(Debug, Parser)]
#[clap(author, version, about, long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Option<CliCommand>,
}

#[derive(Debug, Clone, Subcommand)]
enum CliCommand {
    /// Launch the main gui [default]
    Gui { log_path: Option<PathBuf> },
    /// Attach to FC and tail logs
    Logcat {
        #[clap(short = 'v')]
        verbose: bool,
    },
    /// Dump the contents of the FC's flash to a file
    DumpFlash {
        path: PathBuf,
        #[clap(short = 'f', help = "Always overwrite existing file")]
        force: bool,
        #[clap(short = 'r', help = "Dump entire, raw flash, not just telemetry")]
        raw: bool,
    },
    /// Convert a binary flash/telem log to a JSON file
    #[clap(name = "bin2json")]
    Bin2Json {
        input: Option<PathBuf>,
        output: Option<PathBuf>
    },
    /// Convert a JSON file to a binary flash/telem log
    #[clap(name = "json2bin")]
    Json2Bin {
        input: Option<PathBuf>,
        output: Option<PathBuf>
    },
    /// Reboot the FC
    Reboot,
    /// Reboot the FC into bootloader
    Bootloader,
}

fn open_file_or_stdin(path: Option<PathBuf>) -> Result<Box<dyn Read>, std::io::Error> {
    Ok(match path {
        Some(path) => Box::new(File::open(path)?),
        None => Box::new(std::io::stdin().lock())
    })
}

fn create_file_or_stdout(path: Option<PathBuf>) -> Result<Box<dyn Write>, std::io::Error> {
    Ok(match path {
        Some(path) => Box::new(File::create(path)?),
        None => Box::new(std::io::stdout().lock())
    })
}

fn logcat(verbose: bool) -> Result<(), Box<dyn std::error::Error>> {
    let (downlink_tx, downlink_rx) = channel::<DownlinkMessage>();
    let (_uplink_tx, uplink_rx) = channel::<UplinkMessage>();
    let (serial_status_tx, serial_status_rx) = channel::<(SerialStatus, Option<String>)>();
    spawn_downlink_monitor(serial_status_tx, downlink_tx, uplink_rx, true);

    loop {
        for (status, port) in serial_status_rx.try_iter() {
            match (status, port) {
                (SerialStatus::Connected, Some(p)) => {
                    println!("{} to {}.", "Connected".bright_green().bold(), p)
                }
                (SerialStatus::Error, Some(p)) => {
                    println!("{} to {}.", "Connection lost".bright_red().bold(), p)
                }
                _ => {}
            }
        }

        for msg in downlink_rx.try_iter() {
            match msg {
                DownlinkMessage::Log(t, loc, ll, msg) => {
                    let t = (t as f32) / 1_000.0;
                    let color = match ll {
                        LogLevel::Debug => "bright blue",
                        LogLevel::Info => "bright green",
                        LogLevel::Warning => "bright yellow",
                        LogLevel::Error => "bright red",
                        LogLevel::Critical => "bright purple",
                    };

                    println!(
                        "[{:>8.3}] [{}] [{}] {}",
                        t,
                        ll.to_string().color(color).bold(),
                        loc.white(),
                        msg
                    )
                }
                msg => {
                    if verbose {
                        println!("{:?}", msg)
                    }
                }
            }
        }
    }
}

fn read_flash_chunk(
    uplink_tx: &Sender<UplinkMessage>,
    downlink_rx: &Receiver<DownlinkMessage>,
    address: u32,
    size: u32,
) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    const TIMEOUT: Duration = Duration::from_millis(100);

    let start = Instant::now();
    uplink_tx.send(UplinkMessage::ReadFlash(address, size))?;

    loop {
        if let Some(DownlinkMessage::FlashContent(adr, content)) = downlink_rx.iter().next() {
            if address != adr || size != (content.len() as u32) {
                return Err(Box::new(Error::new(ErrorKind::InvalidData, "Data mismatch.")));
            }

            return Ok(content);
        }

        if start.elapsed() > TIMEOUT {
            return Err(Box::new(Error::new(
                ErrorKind::ConnectionAborted,
                "Connection timed out.",
            )));
        }
    }
}

fn dump_flash(path: PathBuf, force: bool, raw: bool) -> Result<(), Box<dyn std::error::Error>> {
    const NUM_ATTEMPTS: u32 = 10;
    const CHUNK_SIZE: u32 = 1024;
    const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

    if path.exists() && !force {
        return Err(Box::new(Error::new(
            ErrorKind::Other,
            "File already exists. Use -f to overwrite.",
        )));
    }

    let mut f = File::create(path)?;
    let (downlink_tx, downlink_rx) = channel::<DownlinkMessage>();
    let (uplink_tx, uplink_rx) = channel::<UplinkMessage>();
    let (serial_status_tx, _serial_status_rx) = channel::<(SerialStatus, Option<String>)>();
    spawn_downlink_monitor(serial_status_tx, downlink_tx, uplink_rx, false);

    let flash_size = FLASH_SIZE;

    let pb = indicatif::ProgressBar::new(flash_size as u64);
    pb.set_style(
        indicatif::ProgressStyle::with_template(
            "{spinner:.green} [{elapsed_precise}] [{bar}] {bytes}/{total_bytes} ({bytes_per_sec}, {eta})",
        )
        .unwrap()
        .progress_chars("##-"),
    );

    // Only dump the header in raw mode
    let start_address = if raw { 0x00 } else { FLASH_HEADER_SIZE };

    for address in (start_address..flash_size).step_by(CHUNK_SIZE as usize) {
        pb.set_position(address as u64);

        let mut attempts = 0;
        loop {
            match read_flash_chunk(&uplink_tx, &downlink_rx, address, CHUNK_SIZE) {
                Ok(data) => {
                    if raw {
                        f.write_all(&data)?;
                        break;
                    }

                    for (i, p) in data.chunks(256).enumerate() {
                        if p[0] == 0xff {
                            // Flash wasn't full, and this is the end
                            return Ok(());
                        }

                        let page_data = &p[1..(256 - 2)];
                        debug!("{:02x?}", data);
                        let crc_stored = ((p[256 - 2] as u16) << 8) + (p[256 - 1] as u16);
                        let crc_calc = X25.checksum(&page_data);
                        if crc_stored != crc_calc {
                            warn!("CRC mismatch for page 0x{:08x}", address + (i as u32) * 256);
                        }

                        f.write_all(&page_data)?;
                    }
                    break;
                }
                Err(e) => {
                    warn!(
                        "Failed to fetch page 0x{:08x?} ({:?}), retrying {} more times",
                        address,
                        e,
                        NUM_ATTEMPTS - attempts - 1
                    );
                }
            }

            attempts += 1;
            if attempts >= NUM_ATTEMPTS {
                return Err(Box::new(Error::new(
                    ErrorKind::ConnectionAborted,
                    "Failed to read page.",
                )));
            }
        }
    }

    pb.finish();

    Ok(())
}

fn reboot(bootloader: bool) -> Result<(), Box<dyn std::error::Error>> {
    let path = find_serial_port().ok_or_else(|| Error::new(ErrorKind::NotFound, "Failed to find a serial port."))?;

    let mut port = serialport::new(path, serial::BAUD_RATE)
        .timeout(std::time::Duration::from_millis(10))
        .open_native()?;

    let msg = match bootloader {
        true => UplinkMessage::RebootToBootloader,
        false => UplinkMessage::Reboot,
    };

    port.write(&msg.serialize().unwrap())?;
    port.flush()?;

    Ok(())
}

fn bin2json(input: Option<PathBuf>, output: Option<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let mut input = open_file_or_stdin(input)?;
    let mut output = create_file_or_stdout(output)?;

    let mut buffer = Vec::new();
    input.read_to_end(&mut buffer)?;

    let serialized: Vec<_> = buffer.split_mut(|b| *b == 0x00)
        .filter_map(|b| postcard::from_bytes_cobs::<DownlinkMessage>(b).ok())
        .map(|msg| serde_json::to_string(&msg).unwrap())
        .collect();

    output.write(b"[\n")?;
    output.write(&serialized.join(",\n").into_bytes())?;
    output.write(b"\n]\n")?;

    Ok(())
}

fn json2bin(input: Option<PathBuf>, output: Option<PathBuf>) -> Result<(), Box<dyn std::error::Error>> {
    let input = open_file_or_stdin(input)?;
    let mut output = create_file_or_stdout(output)?;

    let msgs: Vec<DownlinkMessage> = serde_json::from_reader(input)?;
    for msg in msgs {
        output.write(&msg.serialize().unwrap())?;
    }

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter_level(LevelFilter::Info)
        .parse_default_env()
        .init();

    let args = Cli::parse();
    match args.command.unwrap_or(CliCommand::Gui { log_path: None }) {
        CliCommand::Gui { log_path } => gui::main(log_path),
        CliCommand::Logcat { verbose } => logcat(verbose),
        CliCommand::DumpFlash { path, force, raw } => dump_flash(path, force, raw),
        CliCommand::Bin2Json { input, output } => bin2json(input, output),
        CliCommand::Json2Bin { input, output } => json2bin(input, output),
        CliCommand::Reboot => reboot(false),
        CliCommand::Bootloader => reboot(true),
    }
}
