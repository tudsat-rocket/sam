#![no_main]
#![no_std]
#![feature(default_alloc_error_handler)]

// Halt on panic
use panic_rtt_target as _;

// This is why we need nightly, default_alloc_error_handler is (still) not
// stabilized. This allows using Strings and everything else that requires
// an allocator.
extern crate alloc;

use core::cell::{Cell, RefCell};
use core::ops::DerefMut;
use alloc::sync::Arc;
use cortex_m::interrupt::{free, Mutex};
use rtt_target::{rtt_init_print, rprintln};

use alloc_cortex_m::CortexMHeap;
use cortex_m;
use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::spi::{Mode, Phase, Polarity};
use hal::pac::{interrupt, Interrupt, TIM2};
use hal::gpio::PinState;
use hal::otg_fs::USB;
use hal::timer::{CounterHz, Event};
use hal::adc::Adc;
use hal::adc::config::AdcConfig;

mod telemetry;
mod vehicle;
mod usb;
mod bootloader;
mod sensors;
mod logging;
mod params;
mod watchdog;
mod lora;
mod flash;
mod buzzer;

use telemetry::*;
use vehicle::*;
use usb::*;
use bootloader::*;
use sensors::*;
use logging::*;
use params::*;
use watchdog::*;
use lora::*;
use flash::*;
use buzzer::*;

mod prelude {
    pub use crate::{log, log_every_nth_time};
    pub use crate::logging::*;
    pub use crate::params::*;
}

const HEAP_SIZE: usize = 8192;
static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] =
    [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

static LOOP_INTERRUPT_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static LOOP_TIMER: Mutex<RefCell<Option<CounterHz<TIM2>>>> = Mutex::new(RefCell::new(None));

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("RTT init");

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
    rprintln!("Heap initialized.");

    Logger::init();
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
    let mut delay = cp.SYST.delay(&clocks);

    // Configure USB serial port for debugging
    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
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
    // TODO
    let mut led_red = gpioc.pc13.into_push_pull_output_in_state(PinState::Low);
    let mut led_yellow = gpioc.pc14.into_push_pull_output_in_state(PinState::High);
    let mut led_green = gpioc.pc15.into_push_pull_output_in_state(PinState::High);

    // Initialize SPI peripherals
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };

    let spi1_sck = gpioa.pa5.into_alternate();
    let spi1_miso = gpiob.pb4.into_alternate();
    let spi1_mosi = gpioa.pa7.into_alternate();
    let spi1_cs_imu = gpiob.pb15.into_push_pull_output_in_state(PinState::High);
    let spi1_cs_baro = gpioc.pc6.into_push_pull_output_in_state(PinState::High);
    let spi1_cs_mag = gpiob.pb14.into_push_pull_output_in_state(PinState::High);
    let spi1_cs_radio = gpioa.pa1.into_push_pull_output_in_state(PinState::High);
    let _spi1_cs_sd = gpioa.pa15.into_push_pull_output_in_state(PinState::High);
    let spi1 = Arc::new(Mutex::new(RefCell::new(dp.SPI1.spi((spi1_sck, spi1_miso, spi1_mosi), spi_mode, 10.MHz(), &clocks))));
    let imu = Imu::init(spi1.clone(), spi1_cs_imu).unwrap();
    let compass = Compass::init(spi1.clone(), spi1_cs_mag, &mut delay).unwrap();
    let barometer = Barometer::init(spi1.clone(), spi1_cs_baro).unwrap();
    let radio_busy = gpioc.pc1.into_input();
    let radio = LoRaRadio::init(spi1, spi1_cs_radio, radio_busy);

    let spi2_sck = gpiob.pb13.into_alternate();
    let spi2_miso = gpioc.pc2.into_alternate();
    let spi2_mosi = gpioc.pc3.into_alternate();
    let spi2_cs_flash = gpiob.pb12.into_push_pull_output_in_state(PinState::High);
    let spi2 = dp.SPI2.spi((spi2_sck, spi2_miso, spi2_mosi), spi_mode, 20.MHz(), &clocks);
    let flash = Flash::init(spi2, spi2_cs_flash);

    let spi_mode = Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition
    };

    let spi3_sck = gpioc.pc10.into_alternate();
    let spi3_miso = gpioc.pc11.into_alternate();
    let spi3_mosi = gpioc.pc12.into_alternate();
    let spi3 = dp.SPI3.spi((spi3_sck, spi3_miso, spi3_mosi), spi_mode, 5.MHz(), &clocks);
    let acc = Accelerometer::init(spi3, gpiod.pd2.into_push_pull_output_in_state(PinState::High)).unwrap();

    // Initialize GPS
    let gps = GPS::init(dp.USART2, gpioa.pa2, gpioa.pa3, &clocks);

    let buzzer = Buzzer::init(dp.TIM4, gpiob.pb9.into_alternate::<2>(), &clocks);

    //let mut gpio_drogue = gpioc.pc8.into_push_pull_output_in_state(PinState::Low);
    //let mut gpio_main = gpioc.pc9.into_push_pull_output_in_state(PinState::Low);

    // TODO: radio gpio
    // TODO: sd
    // TODO: auxiliary IO

    let mut adc = Adc::adc1(dp.ADC1, true, AdcConfig::default());
    adc.enable_temperature_and_vref();
    let adc_bat_high = gpioc.pc5.into_analog();
    let adc_bat_low = gpioc.pc4.into_analog();
    let adc_arm = gpioa.pa4.into_analog();
    let power = PowerMonitor::new(adc, adc_bat_high, adc_bat_low, adc_arm);

    // Configure main loop timer
    let mut timer = dp.TIM2.counter_hz(&clocks);
    timer.start(MAIN_LOOP_FREQ_HERTZ.Hz()).unwrap();
    timer.listen(Event::Update);

    free(|cs| {
        *LOOP_TIMER.borrow(cs).borrow_mut() = Some(timer);
    });

    // Enable main loop interrupt
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2); }

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
        power,
        buzzer
    );

    log!(Info, "Starting main loop.");

    // We have successfully initialized, adjust LEDs
    led_red.set_high();
    led_yellow.set_low();
    led_green.set_low();

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
