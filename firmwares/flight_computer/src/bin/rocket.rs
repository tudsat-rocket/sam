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
use embassy_stm32::adc::SampleTime;
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
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};

use flight_computer_firmware::board::load_outputs;
use flight_computer_firmware::lora;
use flight_computer_firmware::lora::TypedLoraLinkSettings;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx126x, Sx1262};
use shared_types::DownlinkMessage;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use flight_computer_firmware as fw;
use fw::recovery::{MAIN_RECOVERY_SIG, PARABREAKS_SIG, start_recovery_task};
use fw::storage;
use fw::vehicle::*;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) -> ! {
    let (mut board, load_outputs, settings, seed) = fw::init_board().await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    let (lora_downlink, downlink_settings) =
        fw::lora::start_rocket_downlink(board.lora1, settings.lora.clone(), medium_priority_spawner);
    let (lora_uplink, rssi_glob) =
        fw::lora::start_rocket_uplink(board.lora2, settings.lora.clone(), medium_priority_spawner);

    let (can1_tx, can1_rx) = ((), ());
    let (can2_tx, can2_rx) = ((), ());
    // let (usb_downlink, usb_uplink) = ((), ());
    let (usb_downlink, usb_uplink) = fw::usb::start(board.usb_driver, low_priority_spawner);
    let (eth_downlink, eth_uplink) = fw::ethernet::start(board.ethernet, low_priority_spawner, seed);

    fw::recovery::start_recovery_task(load_outputs, high_priority_spawner, &MAIN_RECOVERY_SIG, &PARABREAKS_SIG).await;

    let vehicle = Vehicle::init(
        board.sensors,
        board.outputs.leds,
        load_outputs,
        (can1_tx, can1_rx),
        (can2_tx, can2_rx),
        (usb_downlink, usb_uplink),
        (eth_downlink, eth_uplink),
        (lora_downlink, lora_uplink),
        &MAIN_RECOVERY_SIG,
        &PARABREAKS_SIG,
        board.flash_handle,
        settings,
        downlink_settings,
        rssi_glob,
    );

    board.iwdg.unleash();

    high_priority_spawner.spawn(fw::vehicle::run(vehicle, board.iwdg)).unwrap();

    //medium_priority_spawner.spawn(can::run_tx(can_tx)).unwrap();
    //medium_priority_spawner.spawn(can::run_rx(can_rx)).unwrap();
    //medium_priority_spawner.spawn(drivers::sensors::gps::run(gps)).unwrap(); // TODO: priority?
    medium_priority_spawner.spawn(storage::run(board.flash)).unwrap();

    low_priority_spawner.spawn(fw::buzzer::run(board.buzzer.0, board.buzzer.1)).unwrap();
    low_priority_spawner.spawn(guard_task()).unwrap();
    loop {
        Timer::after_secs(1_000_000).await;
    }
}

#[interrupt]
unsafe fn I2C3_EV() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[interrupt]
unsafe fn I2C3_ER() {
    unsafe { EXECUTOR_MEDIUM.on_interrupt() }
}

#[embassy_executor::task]
pub async fn guard_task() -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(1000));
    loop {
        let mut s: i64 = 0;
        for _i in 0..10 {
            let last = Instant::now();
            ticker.next().await;
            let millis = last.elapsed().as_millis() as i64;
            s += (1000 - millis).abs();
        }
        defmt::info!("GT: {}", s);
    }
}
