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
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{interrupt, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Ticker};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx1262, Sx126x};
use lora_phy::LoRa;
use shared_types::DownlinkMessage;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use flight_computer_firmware as fw;
use fw::vehicle::*;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let (mut board, settings, seed) = fw::init_board().await;

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    let lora_downlink =
        fw::lora::start_rocket_downlink(board.lora1, settings.lora.clone(), medium_priority_spawner);
    let lora_uplink =
        fw::lora::start_rocket_uplink(board.lora2, settings.lora.clone(), medium_priority_spawner);

    let (can1_tx, can1_rx) = ((), ());
    let (can2_tx, can2_rx) = ((), ());

    let (eth_downlink, eth_uplink) = fw::ethernet::start(board.ethernet, low_priority_spawner, seed);
    let (usb_downlink, usb_uplink) = ((), ());
    //let (usb_downlink, usb_uplink) = fw::usb::start(board.usb, low_priority_spawner);

    let vehicle = Vehicle::init(
        board.sensors,
        board.outputs,
        (can1_tx, can1_rx),
        (can2_tx, can2_rx),
        (usb_downlink, usb_uplink),
        (eth_downlink, eth_uplink),
        (lora_downlink, lora_uplink),
        settings,
    );

    board.iwdg.unleash();

    high_priority_spawner.spawn(fw::vehicle::run(vehicle, board.iwdg)).unwrap();

    //medium_priority_spawner.spawn(can::run_tx(can_tx)).unwrap();
    //medium_priority_spawner.spawn(can::run_rx(can_rx)).unwrap();
    //medium_priority_spawner.spawn(drivers::sensors::gps::run(gps)).unwrap(); // TODO: priority?
    //medium_priority_spawner.spawn(flash::run(flash)).unwrap();

    low_priority_spawner.spawn(fw::buzzer::run(board.buzzer.0, board.buzzer.1)).unwrap();
    low_priority_spawner.spawn(guard_task()).unwrap();
}

#[interrupt]
unsafe fn I2C3_EV() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn I2C3_ER() {
    EXECUTOR_MEDIUM.on_interrupt()
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
