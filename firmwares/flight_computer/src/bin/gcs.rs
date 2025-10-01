//! Main entrypoint for firmware. Contains mostly boilerplate stuff for
//! initializing the STM32 and peripherals. For main flight logic see `vehicle.rs`.

#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_mut)]

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::Channel as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hsi48Config, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale,
};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{Config, interrupt};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Ticker};
use shared_types::FlightMode;

use core::sync::atomic::{AtomicBool, Ordering};
use flight_computer_firmware::buzzer::run_gcs;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx126x, Sx1262};
use shared_types::{DownlinkMessage, UplinkMessage};
use static_cell::StaticCell;

use defmt::{Debug2Format, info, warn};
use {defmt_rtt as _, panic_probe as _};

use flight_computer_firmware as fw;
use shared_types::Command;

static BUZZER_ON_SIG: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static IS_ARMED_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, bool>> = StaticCell::new();
static IS_ARMED: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (mut board, _load_outputs, settings, seed) = fw::init_board().await;

    let (usb_tx, usb_rx) = ((), ());
    //let (usb_downlink, usb_uplink) = fw::usb::start(board.usb, low_priority_spawner);
    let (eth_tx, eth_rx) = fw::ethernet::start(board.ethernet, spawner, seed);
    let (lora_rx, _lora_link_settings) =
        fw::lora::start_gcs_downlink(board.lora2, settings.lora.clone(), spawner.make_send());
    let lora_tx = fw::lora::start_gcs_uplink(board.lora1, settings.lora.clone(), spawner.make_send());

    spawner.spawn(run_downlink(eth_tx, usb_tx, lora_rx)).unwrap();
    spawner.spawn(run_uplink(eth_rx, usb_rx, lora_tx)).unwrap();
    let buzzer_on_sig = BUZZER_ON_SIG.init(Signal::new());
    spawner.spawn(fw::buzzer::run_gcs(board.buzzer.0, board.buzzer.1, buzzer_on_sig)).unwrap();
    let is_armed_signal = IS_ARMED_SIGNAL.init(Signal::new());
    spawner.spawn(fw::gcs::keyswitch::run(board.other.keyswitch_in, is_armed_signal)).unwrap();

    // main loop
    loop {
        let is_armed = is_armed_signal.wait().await;
        info!("is_armed: {}", is_armed);
        buzzer_on_sig.signal(is_armed);
        IS_ARMED.store(is_armed, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn run_downlink(
    eth_tx: Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    usb_tx: (),
    lora_rx: Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
) -> ! {
    loop {
        let msg = lora_rx.receive().await;
        eth_tx.send(msg).await;
    }
}

#[embassy_executor::task]
async fn run_uplink(
    eth_rx: Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    usb_rx: (),
    lora_tx: Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    loop {
        let msg = eth_rx.receive().await;
        defmt::info!("received uplinkmessage");
        match msg {
            UplinkMessage::ReadSettings => (),
            UplinkMessage::Heartbeat => (),
            UplinkMessage::ApplyLoRaSettings(_) => (),
            UplinkMessage::ReadFlash(..) => (),
            UplinkMessage::WriteSettings(_) => (),
            UplinkMessage::Command(ref cmd) => {
                match cmd {
                    Command::Reboot
                    | Command::EraseFlash
                    | Command::RebootToBootloader
                    | Command::SetDataRate(_)
                    | Command::SetTransmitPower(_)
                    | Command::SetAcsMode(_)
                    | Command::SetAcsValveState(_) => (),
                    Command::SetIoModuleOutput(..) => {
                        if !IS_ARMED.load(Ordering::Relaxed) {
                            uplink_rejected(cmd);
                            continue;
                        }
                    }
                    Command::SetFlightMode(fm) => match fm {
                        FlightMode::Idle | FlightMode::HardwareArmed => (),
                        _ => {
                            if !IS_ARMED.load(Ordering::Relaxed) {
                                uplink_rejected(cmd);
                                continue;
                            }
                        }
                    },
                };
            }
        };
        lora_tx.send(msg).await;
        defmt::info!("sent uplinkmessage");
    }
}

fn uplink_rejected(cmd: &Command) {
    warn!("uplink command: {} was rejected because the system is not armed. Turn the key to arm!", Debug2Format(&cmd));
}
