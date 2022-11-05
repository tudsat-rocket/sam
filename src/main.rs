#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use std::io::{Error, ErrorKind, Write};
use std::sync::mpsc::channel;

use clap::{Parser, Subcommand};
use colored::Colorize;

use euroc_fc_firmware::telemetry::*;

mod gui;
mod serial;
mod state;

use serial::*;

#[derive(Debug, Parser)]
#[clap(author, version, about, long_about = None)]
struct Cli {
    #[clap(subcommand)]
    command: Option<CliCommand>,
}

#[derive(Debug, Clone, Copy, Subcommand)]
enum CliCommand {
    /// Launch the main gui [default]
    Gui,
    /// Attach to FC and tail logs
    Logcat {
        #[clap(short = 'v')]
        verbose: bool,
    },
    /// Reboot the FC
    Reboot,
    /// Reboot the FC into bootloader
    Bootloader,
}

fn logcat(verbose: bool) -> Result<(), Box<dyn std::error::Error>> {
    let (downlink_tx, downlink_rx) = channel::<DownlinkMessage>();
    let (_uplink_tx, uplink_rx) = channel::<UplinkMessage>();
    let (serial_status_tx, serial_status_rx) = channel::<(SerialStatus, Option<String>)>();
    spawn_downlink_monitor(serial_status_tx, downlink_tx, uplink_rx);

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

fn reboot(bootloader: bool) -> Result<(), Box<dyn std::error::Error>> {
    let path = find_serial_port().ok_or_else(|| Error::new(ErrorKind::NotFound, "Failed to find a serial port."))?;

    let mut port = serialport::new(path, serial::BAUD_RATE)
        .timeout(std::time::Duration::from_millis(10))
        .open_native()?;

    let msg = match bootloader {
        true => UplinkMessage::RebootToBootloader,
        false => UplinkMessage::Reboot,
    };

    let _written = port.write(&msg.wrap())?;
    port.flush()?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Cli::parse();
    match args.command.unwrap_or(CliCommand::Gui) {
        CliCommand::Gui => gui::main(),
        CliCommand::Logcat { verbose } => logcat(verbose),
        CliCommand::Reboot => reboot(false),
        CliCommand::Bootloader => reboot(true),
    }
}
