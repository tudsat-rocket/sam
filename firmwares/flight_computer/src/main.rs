#![no_main]
#![no_std]
#![feature(default_alloc_error_handler)]

// Halt on panic
//use panic_halt as _;

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

mod telemetry;
mod vehicle;
mod usb;
mod bootloader;
mod sensors;
mod logging;
mod params;
mod watchdog;
mod lora;

use telemetry::*;
use vehicle::*;
use usb::*;
use bootloader::*;
use sensors::*;
use logging::*;
use params::*;
use watchdog::*;
use lora::*;

const HEAP_SIZE: usize = 8192;
static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] =
    [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

static LOOP_INTERRUPT_FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
static LOOP_TIMER: Mutex<RefCell<Option<CounterHz<TIM2>>>> = Mutex::new(RefCell::new(None));

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut timer) = LOOP_TIMER.borrow(cs).borrow_mut().deref_mut() {
            timer.clear_interrupt(Event::Update);
        }

        let loop_flag = LOOP_INTERRUPT_FLAG.borrow(cs);
        loop_flag.replace(true);
    });
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("RTT init");

    // Set up the independent watchdog. This reboots the processor
    // if it is not reset regularly, even if the main clock fails.
    // This also guarantees a reboot after returning from bootloader.
    init_watchdog();

    // Reboot to bootloader if requested.
    init_bootloader();

    // Initialize heap
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
    rprintln!("Heap initialized.");

    Logger::init();
    log!(Debug, "Logger init.");

    let dp = hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::peripheral::Peripherals::take().unwrap();
    cp.DWT.enable_cycle_counter();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // The MCO test point can either be used for Master Clock Out, e.g. for
    // diagnosing clock-speed issues, or as general purpose debugging IO.
    let mut mco = gpioa.pa8.into_push_pull_output_in_state(PinState::Low);

    // Initialize and turn off LEDs (TODO)
    let mut led_blue = gpioc.pc13.into_push_pull_output();
    led_blue.set_high();

    // Configure clock
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

    reset_watchdog();

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // Initialize all peripherals
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition
    };

    let spi1_sck = gpioa.pa5.into_alternate();
    let spi1_miso = gpiob.pb4.into_alternate();
    let spi1_mosi = gpioa.pa7.into_alternate();
    //let spi1_cs_imu = gpiob.pb14.into_push_pull_output();
    //let spi1_cs_baro = gpioc.pc4.into_push_pull_output();
    //let spi1_cs_mag = gpioc.pc7.into_push_pull_output();
    let spi1_cs_radio = gpioa.pa1.into_push_pull_output();
    //let spi1_cs_sd = gpioa.pa15.into_push_pull_output();
    let spi1 = Arc::new(Mutex::new(RefCell::new(dp.SPI1.spi((spi1_sck, spi1_miso, spi1_mosi), spi_mode, 1.MHz(), &clocks))));

    //let spi2_sck = gpioc.pc10.into_alternate();
    //let spi2_miso = gpioc.pc11.into_alternate();
    //let spi2_mosi = gpioc.pc12.into_alternate();
    //let spi2_cs_acc = gpiod.pd2.into_push_pull_output();
    //let spi2 = dp.SPI2.spi((spi2_sck, spi2_miso, spi2_mosi), spi_mode, 5.MHz(), &clocks);

    //let spi3_sck = gpiob.pb13.into_alternate();
    //let spi3_miso = gpioc.pc2.into_alternate();
    //let spi3_mosi = gpioc.pc3.into_alternate();
    //let spi3_cs_flash = gpiob.pb12.into_push_pull_output();
    //let spi3 = dp.SPI3.spi((spi3_sck, spi3_miso, spi3_mosi), spi_mode, 20.MHz(), &clocks);

    // TODO: gps uart
    // TODO: radio gpio
    // TODO: LEDs
    // TODO: adc
    // TODO: auxiliary IO

    //let uart2_tx = gpioa.pa2.into_alternate();
    //let uart2_rx = gpioa.pa3.into_alternate();

    let radio = LoRaRadio::init(spi1, spi1_cs_radio);

    let gps = GPS::init(dp.USART2, gpioa.pa2, gpioa.pa3, &clocks);
    if let Err(e) = gps {
        log!(Error, "Failed to initialize GPS: {:?}", e);
    }

    //let mut gpio_drogue = gpioc.pc8.into_push_pull_output_in_state(PinState::Low);
    //let mut gpio_main = gpioc.pc9.into_push_pull_output_in_state(PinState::Low);
    //let mut gpio_buzzer = gpiob.pb9.into_push_pull_output_in_state(PinState::Low);

    // Configure main loop timer
    let mut timer = dp.TIM2.counter_hz(&clocks);
    timer.start(MAIN_LOOP_FREQ_HERTZ.Hz()).unwrap();
    timer.listen(Event::Update);

    free(|cs| {
        *LOOP_TIMER.borrow(cs).borrow_mut() = Some(timer);
    });

    // Enable main loop interrupt
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2); }

    let mut vehicle = Vehicle::init(usb_link, gps, radio);

    log!(Info, "Starting main loop.");

    loop {
        // Read and reset loop interrupt flag
        if free(|cs| LOOP_INTERRUPT_FLAG.borrow(cs).replace(false)) {
            reset_watchdog();
            mco.set_high();
            //led_blue.toggle();
            vehicle.tick();
            mco.set_low();
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
