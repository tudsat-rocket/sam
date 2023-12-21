#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

#[cfg(feature = "futuresdr")]
#[macro_use]
pub extern crate async_trait;

use std::fs::File;
use std::io::{Error, ErrorKind, Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use colored::Colorize;
use crc::{Crc, CRC_16_IBM_SDLC};
use log::*;

use mithril::telemetry::*;

mod data_source;
mod file;
mod gui;
mod settings;
mod simulation;
mod telemetry_ext;

use crate::data_source::serial::{self, *};

#[derive(Debug, Parser)]
#[clap(author, version, about, long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Option<CliCommand>,
}

#[derive(Debug, Clone, Subcommand)]
enum CliCommand {
    /// Launch the main gui [default]
    Gui {
        log_path: Option<PathBuf>,
    },
    /// Attach to FC and tail logs
    /// TODO: embassy rewrite will remove USB logging, so this can be removed
    Logcat {
        #[clap(short = 'v')]
        verbose: bool,
    },
    /// Dump the contents of the FC's flash to a file
    /// TODO: rewrite flash dumping stuff after embassy rewrite
    DumpFlash {
        path: PathBuf,
        #[clap(short = 'f', help = "Always overwrite existing file")]
        force: bool,
        #[clap(short = 'r', help = "Dump entire, raw flash, not just telemetry")]
        raw: bool,
        #[clap(
            short = 's',
            help = "Start address. Default 0x00 (raw mode), 0x1000 (start of log section) otherwise"
        )]
        start: Option<u32>,
    },
    ExtractFlashLogs {
        path: PathBuf,
    },
    /// Convert a binary flash/telem log to a JSON file
    #[clap(name = "bin2json")]
    Bin2Json {
        input: Option<PathBuf>,
        output: Option<PathBuf>,
    },
    #[clap(name = "bin2kml")]
    Bin2Kml {
        input: Option<PathBuf>,
        output: Option<PathBuf>,
        #[clap(short = 'n', help = "Name of the track in the KML file. Default: Sting FC Track")]
        name: Option<String>,
    },
    /// Convert a JSON file to a binary flash/telem log
    #[clap(name = "json2bin")]
    Json2Bin {
        input: Option<PathBuf>,
        output: Option<PathBuf>,
    },
    /// Reboot the FC
    Reboot,
    /// Reboot the FC into bootloader
    Bootloader,
}

fn open_file_or_stdin(path: Option<PathBuf>) -> Result<Box<dyn Read>, std::io::Error> {
    Ok(match path {
        Some(path) => Box::new(File::open(path)?),
        None => Box::new(std::io::stdin().lock()),
    })
}

fn create_file_or_stdout(path: Option<PathBuf>) -> Result<Box<dyn Write>, std::io::Error> {
    Ok(match path {
        Some(path) => Box::new(File::create(path)?),
        None => Box::new(std::io::stdout().lock()),
    })
}

fn logcat(verbose: bool) -> Result<(), Box<dyn std::error::Error>> {
    let (downlink_tx, downlink_rx) = channel::<DownlinkMessage>();
    let (_uplink_tx, uplink_rx) = channel::<UplinkMessage>();
    let (serial_status_tx, serial_status_rx) = channel::<(SerialStatus, Option<String>)>();
    spawn_downlink_monitor(None, serial_status_tx, downlink_tx, uplink_rx, true);

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

                    println!("[{:>8.3}] [{}] [{}] {}", t, ll.to_string().color(color).bold(), loc.white(), msg)
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
            println!("{:02x?} {:?} {:02x?} {:02x?}", address, size, adr, content);
            if address != adr || size != (content.len() as u32) {
                return Err(Box::new(Error::new(ErrorKind::InvalidData, "Data mismatch.")));
            }

            return Ok(content);
        }

        if start.elapsed() > TIMEOUT {
            return Err(Box::new(Error::new(ErrorKind::ConnectionAborted, "Connection timed out.")));
        }
    }
}

fn dump_flash(path: PathBuf, force: bool, raw: bool, start: Option<u32>) -> Result<(), Box<dyn std::error::Error>> {
    const NUM_ATTEMPTS: u32 = 10;
    const CHUNK_SIZE: u32 = 256;
    const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

    if path.exists() && !force {
        return Err(Box::new(Error::new(ErrorKind::Other, "File already exists. Use -f to overwrite.")));
    }

    let mut f = File::create(path)?;
    let (downlink_tx, downlink_rx) = channel::<DownlinkMessage>();
    let (uplink_tx, uplink_rx) = channel::<UplinkMessage>();
    let (serial_status_tx, _serial_status_rx) = channel::<(SerialStatus, Option<String>)>();
    spawn_downlink_monitor(None, serial_status_tx, downlink_tx, uplink_rx, false);

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
    let start = start.unwrap_or(if raw { 0x00 } else { FLASH_HEADER_SIZE });

    for address in (start..flash_size).step_by(CHUNK_SIZE as usize) {
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
                            continue;
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
                error!("Failed to read page 0x{:08x?}, storing as 0xffs, continuing.", address);
                f.write_all(&[0xff; 256])?;
                break;
            }
        }
    }

    pb.finish();

    Ok(())
}

fn extract_flash_logs(path: PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

    let mut f = File::open(path)?;
    f.seek(SeekFrom::Start(FLASH_HEADER_SIZE as u64))?;

    let mut chunk_buffer: [u8; 256] = [0x00; 256];
    let mut address = FLASH_HEADER_SIZE;
    loop {
        if let Err(e) = f.read_exact(&mut chunk_buffer) {
            if e.kind() == ErrorKind::BrokenPipe || e.kind() == ErrorKind::UnexpectedEof {
                break;
            } else {
                return Err(Box::new(e));
            }
        }

        let contents = if chunk_buffer[0] == 0x00 {
            let data = &chunk_buffer[1..(256 - 2)];
            let crc_stored = ((chunk_buffer[256 - 2] as u16) << 8) + (chunk_buffer[256 - 1] as u16);
            Some((data, crc_stored))
        } else if chunk_buffer[256 - 1] == 0x00 {
            let data = &chunk_buffer[0..(256 - 3)];
            let crc_stored = ((chunk_buffer[256 - 3] as u16) << 8) + (chunk_buffer[256 - 2] as u16);
            Some((data, crc_stored))
        } else {
            None
        };

        if contents.map(|(data, crc)| X25.checksum(&data) == crc).unwrap_or(false) {
            std::io::stdout().write(contents.unwrap().0)?;
        } else {
            warn!("No valid data found for page {:02x?}, skipping page: {:02x?}", address, chunk_buffer);
        }

        address += 256;
    }

    Ok(())
}

fn reboot(bootloader: bool) -> Result<(), Box<dyn std::error::Error>> {
    let path = find_serial_port().ok_or_else(|| Error::new(ErrorKind::NotFound, "Failed to find a serial port."))?;

    let mut port = serialport::new(path, serial::BAUD_RATE)
        .timeout(std::time::Duration::from_millis(10))
        .open_native()?;

    let msg = match bootloader {
        true => UplinkMessage::Command(Command::RebootToBootloader),
        false => UplinkMessage::Command(Command::Reboot),
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

    let serialized: Vec<_> = buffer
        .split_mut(|b| *b == 0x00)
        .filter_map(|b| postcard::from_bytes_cobs::<DownlinkMessage>(b).ok())
        .map(|msg| serde_json::to_string(&msg).unwrap())
        .collect();

    output.write(b"[\n")?;
    output.write(&serialized.join(",\n").into_bytes())?;
    output.write(b"\n]\n")?;

    Ok(())
}

fn bin2kml(
    input: Option<PathBuf>,
    output: Option<PathBuf>,
    name: Option<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut input = open_file_or_stdin(input)?;
    let mut output = create_file_or_stdout(output)?;

    let mut buffer = Vec::new();
    input.read_to_end(&mut buffer)?;

    let coordinates: Vec<String> = buffer
        .split_mut(|b| *b == 0x00)
        .filter_map(|b| postcard::from_bytes_cobs::<DownlinkMessage>(b).ok())
        .map(|msg| msg.into())
        .filter(|vs: &VehicleState| vs.longitude.is_some() && vs.latitude.is_some() && vs.altitude_gps_asl.is_some())
        .map(|vs| (vs.longitude.unwrap(), vs.latitude.unwrap(), vs.altitude_gps_asl.unwrap()))
        .map(|(ln, lt, alt)| format!("     {},{},{}", ln, lt, alt + 100.0))
        .collect();

    let name = name.unwrap_or("Sting FC Track".into());

    let xml = format!(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
 <Document>
  <name>{name}</name>
  <description><![CDATA[{name}
{len} Trackpoints + 0 Placemarks]]></description>
  <Style id=\"TrackStyle\">
   <LineStyle>
    <color>ff00ff00</color>
    <width>3</width>
   </LineStyle>
   <PolyStyle>
    <color>7f0000ff</color>
   </PolyStyle>
   <BalloonStyle>
    <text></text>
   </BalloonStyle>
  </Style>

  <Placemark id=\"sting_fc\">
   <name>Sting FC</name>
   <description></description>
   <styleUrl>#TrackStyle</styleUrl>
   <LineString>
    <extrude>0</extrude>
    <tessellate>0</tessellate>
    <altitudeMode>clampToGround</altitudeMode>
    <coordinates>
     {coords}
    </coordinates>
   </LineString>
  </Placemark>

 </Document>
</kml>",
        name = name,
        len = coordinates.len(),
        coords = coordinates.join("\n")
    );

    output.write(xml.as_bytes())?;

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
    env_logger::Builder::new().filter_level(LevelFilter::Info).parse_default_env().init();

    let args = Cli::parse();
    match args.command.unwrap_or(CliCommand::Gui { log_path: None }) {
        CliCommand::Gui { log_path } => gui::main(log_path),
        CliCommand::Logcat { verbose } => logcat(verbose),
        CliCommand::DumpFlash {
            path,
            force,
            raw,
            start,
        } => dump_flash(path, force, raw, start),
        CliCommand::ExtractFlashLogs { path } => extract_flash_logs(path),
        CliCommand::Bin2Json { input, output } => bin2json(input, output),
        CliCommand::Bin2Kml { input, output, name } => bin2kml(input, output, name),
        CliCommand::Json2Bin { input, output } => json2bin(input, output),
        CliCommand::Reboot => reboot(false),
        CliCommand::Bootloader => reboot(true),
    }
}
