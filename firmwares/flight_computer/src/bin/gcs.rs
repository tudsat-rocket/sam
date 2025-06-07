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
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{interrupt, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Ticker};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx1262, Sx126x};
use lora_phy::LoRa;
use shared_types::{DownlinkMessage, UplinkMessage};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use flight_computer_firmware as fw;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (mut board, settings, seed) = fw::init_board().await;

    let (usb_tx, usb_rx) = ((), ());
    //let (usb_downlink, usb_uplink) = fw::usb::start(board.usb, low_priority_spawner);
    let (eth_tx, eth_rx) = fw::ethernet::start(board.ethernet, spawner, seed);
    let lora_rx = fw::lora::start_gcs_downlink(board.lora2, settings.lora.clone(), spawner.make_send());
    let lora_tx = fw::lora::start_gcs_uplink(board.lora1, settings.lora.clone(), spawner.make_send());

    spawner.spawn(run_downlink(eth_tx, usb_tx, lora_rx)).unwrap();
    spawner.spawn(run_uplink(eth_rx, usb_rx, lora_tx)).unwrap();
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
        lora_tx.send(msg).await;
    }
}
