//! Main entrypoint for firmware. Contains mostly boilerplate stuff for
//! initializing the STM32 and peripherals. For main flight logic see `vehicle.rs`.

#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

// This is why we need nightly, default_alloc_error_handler is (still) not
// stabilized. This allows using Strings and everything else that requires
// an allocator.
extern crate alloc;

use alloc::sync::Arc;
use core::cell::{Cell, RefCell};
use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};
use defmt::*;

use cortex_m;
use hal::adc::config::AdcConfig;
use hal::adc::Adc;
use hal::gpio::{PinState, Speed};
use hal::gpio::alt::otg_fs::{Dm, Dp};
use hal::otg_fs::USB;
use hal::pac::{interrupt, Interrupt, TIM2};
use hal::prelude::*;
use hal::spi::{Mode, Phase, Polarity};
use hal::timer::*;
use stm32f4xx_hal as hal;

mod bootloader;
mod buzzer;
#[cfg(feature = "rev2")]
mod can;
mod flash;
mod logging;
mod lora;
mod params;
mod sensors;
mod telemetry;
mod usb;
mod vehicle;
mod watchdog;
mod settings;
mod state_estimation;
mod runcam;

use mithril as _;

use bootloader::*;
use buzzer::*;
#[cfg(feature = "rev2")]
use can::*;
use flash::*;
use logging::Info; // TODO
use lora::*;
use params::*;
use sensors::*;
use usb::*;
use vehicle::*;
use watchdog::*;
use runcam::*;

mod prelude {
    pub use crate::logging::*;
    pub use crate::params::*;
    pub use crate::{log, log_every_nth_time};
}

const HEAP_SIZE: usize = 16384;
static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] = [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static LOOP_INTERRUPT_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static LOOP_TIMER: Mutex<RefCell<Option<CounterHz<TIM2>>>> = Mutex::new(RefCell::new(None));

#[cortex_m_rt::entry]
fn main() -> ! {
    // Set up the independent watchdog. This reboots the processor
    // if it is not reset regularly, even if the main clock fails.
    // This also guarantees a reboot after returning from bootloader.
    // TODO: check if the current boot is a watchdog reset and react
    // appropriately
    init_watchdog();

    // Reboot to bootloader if requested.
    init_bootloader();

    // Initialize heap
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    info!("Heap initialized.");

    logging::Logger::init();
    log!(Info, "Logger init.");

    let dp = hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiod = dp.GPIOD.split();

    // Configure clock
    // Makes sure the processor uses the 25MHz external oscillator, sets the
    // system clock to 84MHz, and ensures the processor has an internal 48MHz
    // clock for USB
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .require_pll48clk()
        .sysclk(CLOCK_FREQ_MEGA_HERTZ.MHz())
        .freeze();
    #[allow(unused_mut, unused)] // TODO
    let mut delay = cp.SYST.delay(&clocks);

    // Configure USB serial port for debugging
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: Dm::PA11(gpioa.pa11.into_alternate()),
        pin_dp: Dp::PA12(gpioa.pa12.into_alternate()),
        hclk: clocks.hclk(),
    };
    let usb_link = UsbLink::init(usb);
    log!(Info, "USB initialized.");

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // The MCO test point can either be used for Master Clock Out, e.g. for
    // diagnosing clock-speed issues, or as general purpose debugging IO.
    let mut mco = gpioa.pa8.into_push_pull_output_in_state(PinState::Low);

    // Initialize LEDs
    let led_red = gpioc.pc13.into_push_pull_output_in_state(PinState::Low);
    let led_yellow = gpioc.pc14.into_push_pull_output_in_state(PinState::High);
    let led_green = gpioc.pc15.into_push_pull_output_in_state(PinState::High);
    let leds = (led_red, led_yellow, led_green);

    // Initialize SPI peripherals
    let dma1_streams = hal::dma::StreamsTuple::new(dp.DMA1);

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    // SPI 1
    #[cfg(feature = "rev1")]
    let spi1_miso = gpiob.pb4;
    #[cfg(feature = "rev2")]
    let spi1_miso = gpioa.pa6;

    let spi1 = dp.SPI1.spi(
        (
            gpioa.pa5.into_alternate().speed(Speed::VeryHigh),
            spi1_miso.into_alternate().speed(Speed::VeryHigh),
            gpioa.pa7.into_alternate().speed(Speed::VeryHigh),
        ),
        spi_mode,
        10.MHz(),
        &clocks,
    );
    let spi1 = Arc::new(Mutex::new(RefCell::new(spi1)));

    let spi1_cs_imu = gpiob.pb15.into_push_pull_output_in_state(PinState::High);
    let spi1_cs_baro = gpioc.pc6.into_push_pull_output_in_state(PinState::High);
    #[cfg(feature = "rev1")]
    let spi1_cs_mag = gpiob.pb14.into_push_pull_output_in_state(PinState::High);
    #[cfg(feature = "rev2")]
    let spi1_cs_mag = gpiob.pb10.into_push_pull_output_in_state(PinState::High);
    #[cfg(feature = "rev2")]
    let spi1_cs_acc = gpioa.pa4.into_push_pull_output_in_state(PinState::High);
    let spi1_cs_radio = gpioa.pa1.into_push_pull_output_in_state(PinState::High);
    let _spi1_cs_sd = gpioa.pa15.into_push_pull_output_in_state(PinState::High);

    // SPI 1 peripherals
    let imu = Imu::init(spi1.clone(), spi1_cs_imu).unwrap();
    #[cfg(feature="rev1")]
    let compass = BMM150::init(spi1.clone(), spi1_cs_mag, &mut delay).unwrap();
    #[cfg(feature="rev2")]
    let compass = LIS3MDL::init(spi1.clone(), spi1_cs_mag).unwrap();
    let barometer = Barometer::init(spi1.clone(), spi1_cs_baro).unwrap();
    #[cfg(feature = "rev2")]
    let acc = H3LIS331DL::init(spi1.clone(), spi1_cs_acc).unwrap();
    let radio = LoRaRadio::init(
        spi1,
        spi1_cs_radio,
        gpioc.pc0.into_input(),
        gpioc.pc1.into_input()
    );

    // SPI 2
    let spi2 = dp.SPI2.spi(
        (
            gpiob.pb13.into_alternate().speed(Speed::VeryHigh),
            gpioc.pc2.into_alternate().speed(Speed::VeryHigh),
            gpioc.pc3.into_alternate().speed(Speed::VeryHigh),
        ),
        spi_mode,
        20.MHz(),
        &clocks
    );
    #[cfg(feature = "rev1")]
    let spi2_dma_tx = unsafe { core::ptr::read(&spi2).use_dma().tx() };
    let spi2 = Arc::new(Mutex::new(RefCell::new(spi2)));

    // SPI 2 peripherals
    #[cfg(feature = "rev1")]
    let flash = Flash::init(spi2, gpiob.pb12.into_push_pull_output_in_state(PinState::High), spi2_dma_tx, dma1_streams.4);

    #[cfg(feature = "rev2")]
    let can = MCP2517FD::init(spi2, gpiob.pb12.into_push_pull_output_in_state(PinState::High)).unwrap();

    // SPI 3
    // For revision 1, we need a different spi mode on SPI3
    #[cfg(feature = "rev1")]
    let spi_mode = Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
    };

    let spi3 = dp.SPI3.spi(
        (
            gpioc.pc10.into_alternate().speed(Speed::VeryHigh),
            gpioc.pc11.into_alternate().speed(Speed::VeryHigh),
            gpioc.pc12.into_alternate().speed(Speed::VeryHigh),
        ),
        spi_mode,
        if cfg!(feature = "rev2") { 20.MHz() } else { 5.MHz() },
        &clocks
    );
    #[cfg(feature = "rev2")]
    let spi3_dma_tx = unsafe { core::ptr::read(&spi3).use_dma().tx() };
    let spi3 = Arc::new(Mutex::new(RefCell::new(spi3)));

    // SPI 3 peripherals
    #[cfg(feature = "rev1")]
    let acc = ADXL375::init(spi3, gpiod.pd2.into_push_pull_output_in_state(PinState::High)).unwrap();

    #[cfg(feature = "rev2")]
    let flash = Flash::init(spi3, gpiod.pd2.into_push_pull_output_in_state(PinState::High), spi3_dma_tx, dma1_streams.7);

    // Initialize GPS
    let gps = GPS::init(dp.USART2, gpioa.pa2, gpioa.pa3, &clocks);

    #[cfg(feature = "rev1")]
    let pwm = dp.TIM4.pwm_hz(Channel4::new(gpiob.pb9.into_alternate()), 440.Hz(), &clocks);
    #[cfg(feature = "rev2")]
    let pwm = dp.TIM3.pwm_hz(Channel2::new(gpioc.pc7.into_alternate()), 440.Hz(), &clocks);

    let buzzer = Buzzer::init(pwm);

    let gpio_drogue = gpioc.pc8.into_push_pull_output_in_state(PinState::Low);
    let gpio_main = gpioc.pc9.into_push_pull_output_in_state(PinState::Low);
    let recovery = (gpio_drogue, gpio_main);

    // TODO: sd

    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    adc.enable_temperature_and_vref();
    #[cfg(feature = "rev1")]
    let power = PowerMonitor::new(adc,
        gpioc.pc5.into_analog(),
        gpioc.pc4.into_analog(),
        gpioa.pa4.into_analog()
    );
    #[cfg(feature = "rev2")]
    let power = PowerMonitor::new(adc,
        gpiob.pb0.into_analog(),
        gpioc.pc5.into_analog(),
        gpioc.pc4.into_analog()
    );

    let runcam = RuncamCamera::init(dp.USART1, gpioa.pa9, gpioa.pa10, &clocks);

    // Configure main loop timer
    let mut timer = dp.TIM2.counter_hz(&clocks);
    timer.start(MAIN_LOOP_FREQ_HERTZ.Hz()).unwrap();
    timer.listen(Event::Update);

    free(|cs| {
        *LOOP_TIMER.borrow(cs).borrow_mut() = Some(timer);
    });

    // Enable main loop interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
    }

    let mut vehicle = Vehicle::init(
        clocks,
        usb_link,
        imu,
        acc,
        compass,
        barometer,
        gps,
        flash,
        radio,
        #[cfg(feature = "rev2")]
        can,
        power,
        leds,
        buzzer,
        recovery,
        runcam
    );

    log!(Info, "Starting main loop.");

    loop {
        // Read and reset loop interrupt flag
        if free(|cs| LOOP_INTERRUPT_FLAG.borrow(cs).replace(false)) {
            reset_watchdog();
            mco.set_high();

            vehicle.tick();

            mco.set_low();
        }
    }
}

#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut timer) = LOOP_TIMER.borrow(cs).borrow_mut().deref_mut() {
            timer.clear_interrupt(Event::Update);
        }

        LOOP_INTERRUPT_FLAG.borrow(cs).replace(true);
    });
}
