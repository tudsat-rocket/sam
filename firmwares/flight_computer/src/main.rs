//! Main entrypoint for firmware. Contains mostly boilerplate stuff for
//! initializing the STM32 and peripherals. For main flight logic see `vehicle.rs`.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// This is why we need nightly, default_alloc_error_handler is (still) not
// stabilized. This allows using Strings and everything else that requires
// an allocator.
extern crate alloc;

//use alloc::sync::Arc;
//use core::cell::{Cell, RefCell};
//use core::ops::DerefMut;
//use cortex_m::interrupt::{free, Mutex};
//use defmt::*;

//mod bootloader;
//mod buzzer;
//#[cfg(feature = "rev2")]
//mod can;
//mod flash;
//mod logging;
//mod lora;
//mod params;
mod sensors;
mod telemetry;
mod usb;
mod vehicle;
//mod watchdog;
mod settings;
mod state_estimation;
//mod runcam;

//use mithril as _;

//use bootloader::*;
//use buzzer::*;
//#[cfg(feature = "rev2")]
//use can::*;
//use flash::*;
//use logging::Info; // TODO
//use lora::*;
//use params::*;
//use sensors::*;
use usb::*;
use vehicle::*;
//use watchdog::*;
//use runcam::*;
use sensors::*;
//
//mod prelude {
//    pub use crate::logging::*;
//    pub use crate::params::*;
//    pub use crate::{log, log_every_nth_time};
//}

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Level, Speed, Output};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::{DMA2_CH3, DMA2_CH2};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::{interrupt, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer, Delay};

use static_cell::StaticCell;

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

const HEAP_SIZE: usize = 16384;
static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] = [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
//static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn I2C3_EV() {
    EXECUTOR_HIGH.on_interrupt()
}

//#[interrupt]
//unsafe fn I2C3_ER() {
//    EXECUTOR_MED.on_interrupt()
//}

static SPI1_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<embassy_stm32::peripherals::SPI1, DMA2_CH3, DMA2_CH2>>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_low_priority_spawner: Spawner) {
//    // Set up the independent watchdog. This reboots the processor
//    // if it is not reset regularly, even if the main clock fails.
//    // This also guarantees a reboot after returning from bootloader.
//    // TODO: check if the current boot is a watchdog reset and react
//    // appropriately
//    init_watchdog();
//
//    // Reboot to bootloader if requested.
//    init_bootloader();
//
    // Basic setup, including clocks
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz::mhz(25)); // our high-speed external oscillator speed
    config.rcc.sys_ck = Some(Hertz::mhz(84)); // our desired system clock
    config.rcc.pll48 = true; // we need a 48MHz clock for USB
    let p = embassy_stm32::init(config);

    // Initialize heap
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    info!("Heap initialized.");

    let usb = UsbLink::init(p.USB_OTG_FS, p.PA12, p.PA11).await;

    let spi1_config = embassy_stm32::spi::Config::default();
    let spi1 = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi1_config);
    let spi1 = Mutex::<CriticalSectionRawMutex, _>::new(spi1);
    let spi1 = SPI1_SHARED.init(spi1);

    let spi1_cs_imu = Output::new(p.PB15, Level::High, Speed::VeryHigh);
    let spi1_cs_acc = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi1_cs_mag = Output::new(p.PB10, Level::High, Speed::VeryHigh);
    let spi1_cs_baro = Output::new(p.PC6, Level::High, Speed::VeryHigh);
    let _spi1_cs_radio = Output::new(p.PA1, Level::High, Speed::VeryHigh);
    let _spi1_cs_sd = Output::new(p.PA15, Level::High, Speed::VeryHigh);

    let imu = LSM6::init(SpiDevice::new(spi1, spi1_cs_imu)).await.map_err(|_e| ()).unwrap(); // TODO: error?
    let acc = H3LIS331DL::init(SpiDevice::new(spi1, spi1_cs_acc)).await.map_err(|_e| ()).unwrap(); // TODO: error?
    let mag = LIS3MDL::init(SpiDevice::new(spi1, spi1_cs_mag)).await.map_err(|_e| ()).unwrap(); // TODO: error?
    let baro = MS5611::init(SpiDevice::new(spi1, spi1_cs_baro)).await.map_err(|_e| ()).unwrap(); // TODO: error?
//
//    // SPI 1 peripherals
//    let radio = LoRaRadio::init(
//        spi1,
//        spi1_cs_radio,
//        gpioc.pc0.into_input(),
//        gpioc.pc1.into_input()
//    );
//
//    // SPI 2
//    let spi2 = dp.SPI2.spi(
//        (
//            gpiob.pb13.into_alternate().speed(Speed::VeryHigh),
//            gpioc.pc2.into_alternate().speed(Speed::VeryHigh),
//            gpioc.pc3.into_alternate().speed(Speed::VeryHigh),
//        ),
//        spi_mode,
//        20.MHz(),
//        &clocks
//    );
//    let spi2 = Arc::new(Mutex::new(RefCell::new(spi2)));
//
//    // SPI 2 peripherals
//    let can = MCP2517FD::init(spi2, gpiob.pb12.into_push_pull_output_in_state(PinState::High)).unwrap();
//
//    // SPI 3
//    let spi3 = dp.SPI3.spi(
//        (
//            gpioc.pc10.into_alternate().speed(Speed::VeryHigh),
//            gpioc.pc11.into_alternate().speed(Speed::VeryHigh),
//            gpioc.pc12.into_alternate().speed(Speed::VeryHigh),
//        ),
//        spi_mode,
//        if cfg!(feature = "rev2") { 20.MHz() } else { 5.MHz() },
//        &clocks
//    );
//    let spi3_dma_tx = unsafe { core::ptr::read(&spi3).use_dma().tx() };
//    let spi3 = Arc::new(Mutex::new(RefCell::new(spi3)));
//
//    // SPI 3 peripherals
//    let flash = Flash::init(spi3, gpiod.pd2.into_push_pull_output_in_state(PinState::High), spi3_dma_tx, dma1_streams.7);
//
//    // Initialize GPS
//    let gps = GPS::init(dp.USART2, gpioa.pa2, gpioa.pa3, &clocks);

    let adc = Adc::new(p.ADC1, &mut Delay);
    let power = PowerMonitor::init(adc, p.PB0, p.PC5, p.PC4).await;

    // TODO: sd

    let led_red = Output::new(p.PC13, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC14, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC15, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    let gpio_drogue = Output::new(p.PC8, Level::Low, Speed::Low);
    let gpio_main = Output::new(p.PC9, Level::Low, Speed::Low);
    let recovery = (gpio_drogue, gpio_main);
//
//    #[cfg(feature = "rev1")]
//    let pwm = dp.TIM4.pwm_hz(Channel4::new(gpiob.pb9.into_alternate()), 440.Hz(), &clocks);
//    #[cfg(feature = "rev2")]
//    let pwm = dp.TIM3.pwm_hz(Channel2::new(gpioc.pc7.into_alternate()), 440.Hz(), &clocks);
//
//    let buzzer = Buzzer::init(pwm);
//    let runcam = RuncamCamera::init(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);
//
//    // The MCO test point can either be used for Master Clock Out, e.g. for
//    // diagnosing clock-speed issues, or as general purpose debugging IO.
//    let mut mco = gpioa.pa8.into_push_pull_output_in_state(PinState::Low);

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    let vehicle = Vehicle::init(
        //high_priority_spawner,
        //low_priority_spawner,
        imu,
        acc,
        mag,
        baro,
        power,
        usb,
        leds,
        recovery
    ).await.unwrap();

    high_priority_spawner.spawn(crate::vehicle::run(vehicle)).unwrap();

    loop {
        Timer::after(Duration::from_micros(1)).await;
    }
}
