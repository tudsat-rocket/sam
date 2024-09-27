//! Main entrypoint for firmware. Contains mostly boilerplate stuff for
//! initializing the STM32 and peripherals. For main flight logic see `vehicle.rs`.

#![no_std]
#![no_main]

// TODO: some dependency still allocates
extern crate alloc;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Level, Speed, Output, Pull, Input, OutputType};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Pll, PllMul, PllPreDiv, PllPDiv, PllQDiv, PllSource, Sysclk};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::gpio::low_level::Pin;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{interrupt, Config};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Ticker};

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod buzzer;
mod can;
mod drivers;
mod flash;
mod lora;
mod usb;

#[cfg(not(feature="gcs"))]
mod vehicle;
#[cfg(feature="gcs")]
mod gcs;

use buzzer::*;
use can::*;
use flash::*;
use lora::*;
use drivers::sensors::*;
use usb::*;

#[cfg(not(feature="gcs"))]
use vehicle::*;
#[cfg(feature="gcs")]
use gcs::*;

const HEAP_SIZE: usize = 1024;
static mut HEAP: [core::mem::MaybeUninit<u8>; HEAP_SIZE] = [core::mem::MaybeUninit::uninit(); HEAP_SIZE];

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

static SPI1_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<SPI1, DMA2_CH3, DMA2_CH2>>> = StaticCell::new();
static SPI2_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<SPI2, DMA1_CH4, DMA1_CH3>>> = StaticCell::new();
static SPI3_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, Spi<SPI3, DMA1_CH7, DMA1_CH0>>> = StaticCell::new();

#[cfg_attr(feature="gcs", allow(dead_code))]
#[cfg_attr(feature="gcs", allow(unused_variables))]
#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    // Basic setup, including clocks
    // Divider values taken from STM32CubeMx
    let mut config = Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: Hertz::mhz(25), // our high-speed external oscillator speed
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV25,
        mul: PllMul::MUL336,
        divp: Some(PllPDiv::DIV4), // 25MHz / 25 * 336 / 4 = 84MHz
        divq: Some(PllQDiv::DIV7), // 25MHz / 25 * 336 / 7 = 48MHz
        divr: None,
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.sys = Sysclk::PLL1_P;
    let p = embassy_stm32::init(config);

    // Set up the independent watchdog. This reboots the processor
    // if it is not pet regularly, even if the main clock fails.
    // TODO: check if the current boot is a watchdog reset and react
    // appropriately
    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout

    // Initialize heap
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    let (usb, usb_flash) = UsbHandle::init(p.USB_OTG_FS, p.PA12, p.PA11).await;

    // Shared SPI1 bus
    let mut spi1_config = embassy_stm32::spi::Config::default();
    spi1_config.frequency = Hertz::mhz(10);
    #[cfg(feature="rev1")]
    let spi1 = Spi::new(p.SPI1, p.PA5, p.PA7, p.PB4, p.DMA2_CH3, p.DMA2_CH2, spi1_config);
    #[cfg(not(feature="rev1"))]
    let spi1 = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi1_config);
    let spi1 = Mutex::<CriticalSectionRawMutex, _>::new(spi1);
    let spi1 = SPI1_SHARED.init(spi1);

    let spi1_cs_imu = Output::new(p.PB15, Level::High, Speed::VeryHigh);
    let spi1_cs_acc = Output::new(p.PA4, Level::High, Speed::VeryHigh);
    let spi1_cs_mag = Output::new(p.PB10, Level::High, Speed::VeryHigh);
    let spi1_cs_baro = Output::new(p.PC6, Level::High, Speed::VeryHigh);
    let spi1_cs_radio = Output::new(p.PA1, Level::High, Speed::VeryHigh);
    let _spi1_cs_sd = Output::new(p.PA15, Level::High, Speed::VeryHigh);

    let imu = LSM6::init(SpiDevice::new(spi1, spi1_cs_imu)).await.unwrap();
    let acc = H3LIS331DL::init(SpiDevice::new(spi1, spi1_cs_acc)).await.unwrap();
    let mag = LIS3MDL::init(SpiDevice::new(spi1, spi1_cs_mag)).await.unwrap();
    let baro = MS5611::init(SpiDevice::new(spi1, spi1_cs_baro)).await.unwrap();
    let radio = Radio::init(
        SpiDevice::new(spi1, spi1_cs_radio),
        Input::new(p.PC0, Pull::Down),
        Input::new(p.PC1, Pull::Down),
    ).await.unwrap();

    // SPI2, only used for CAN bus
    let mut spi2_config = embassy_stm32::spi::Config::default();
    spi2_config.frequency = Hertz::mhz(20);
    let spi2 = Spi::new(p.SPI2, p.PB13, p.PC3, p.PC2, p.DMA1_CH4, p.DMA1_CH3, spi2_config);
    let spi2 = Mutex::<CriticalSectionRawMutex, _>::new(spi2);
    let spi2 = SPI2_SHARED.init(spi2);

    #[cfg(not(feature="gcs"))]
    let spi2_cs_can = Output::new(p.PB12, Level::High, Speed::VeryHigh);
    #[cfg(not(feature="gcs"))]
    let (can_tx, can_rx, can_handle) = can::init(SpiDevice::new(spi2, spi2_cs_can), CanDataRate::Kbps125).await;

    // SPI 3
    let mut spi3_config = embassy_stm32::spi::Config::default();
    spi3_config.frequency = Hertz::mhz(20);
    let spi3 = Spi::new(p.SPI3, p.PC10, p.PC12, p.PC11, p.DMA1_CH7, p.DMA1_CH0, spi3_config);
    let spi3 = Mutex::<CriticalSectionRawMutex, _>::new(spi3);
    let spi3 = SPI3_SHARED.init(spi3);

    let spi3_cs_flash = Output::new(p.PD2, Level::High, Speed::VeryHigh);
    #[cfg(not(feature="gcs"))]
    let (flash, flash_handle, settings) = Flash::init(SpiDevice::new(spi3, spi3_cs_flash), usb_flash).await.map_err(|_e| ()).unwrap();

    // Initialize GPS
    #[cfg(not(feature="gcs"))]
    let (gps, gps_handle) = GPS::init(p.USART2, p.PA3, p.PA2, p.DMA1_CH6, p.DMA1_CH5);

    #[cfg(not(feature="gcs"))]
    let adc = Adc::new(p.ADC1, &mut Delay);
    #[cfg(not(feature="gcs"))]
    let power = PowerMonitor::init(adc, p.PB0, p.PC5, p.PC4).await;

    let led_red = Output::new(p.PC13, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC14, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC15, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    #[cfg(not(feature="gcs"))]
    let gpio_drogue = Output::new(p.PC8, Level::Low, Speed::Low);
    #[cfg(not(feature="gcs"))]
    let gpio_main = Output::new(p.PC9, Level::Low, Speed::Low);
    #[cfg(not(feature="gcs"))]
    let recovery = (gpio_drogue, gpio_main);

    #[cfg(feature="rev1")]
    let buzzer = {
        let gpiob_block = p.PB9.block();
        let pwm_pin = PwmPin::new_ch4(p.PB9, OutputType::PushPull);
        let pwm = SimplePwm::new(p.TIM4, None, None, None, Some(pwm_pin), Hertz::hz(440), Default::default());
        Buzzer::init(pwm, Channel::Ch4, gpiob_block, 9)
    };

    #[cfg(not(feature="rev1"))]
    let buzzer = {
        let gpioc_block = p.PC7.block();
        let pwm_pin = PwmPin::new_ch2(p.PC7, OutputType::PushPull);
        let pwm = SimplePwm::new(p.TIM3, None, Some(pwm_pin), None, None, Hertz::hz(440), Default::default());
        Buzzer::init(pwm, Channel::Ch2, gpioc_block, 7)
    };

    iwdg.unleash();

    #[cfg(not(feature="gcs"))]
    let vehicle = Vehicle::init(
        imu,
        acc,
        mag,
        baro,
        gps_handle,
        power,
        usb,
        radio,
        flash_handle,
        can_handle,
        leds,
        buzzer,
        recovery,
        settings,
    );

    #[cfg(feature="gcs")]
    let gcs = GroundControlStation::init(usb, radio, leds, buzzer);

    // Start high priority executor
    interrupt::I2C3_EV.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::I2C3_EV);

    // Start medium priority executor
    interrupt::I2C3_ER.set_priority(Priority::P7);
    let medium_priority_spawner = EXECUTOR_MEDIUM.start(interrupt::I2C3_ER);

    #[cfg(not(feature="gcs"))]
    {
        high_priority_spawner.spawn(vehicle::run(vehicle, iwdg)).unwrap();
        medium_priority_spawner.spawn(can::run_tx(can_tx)).unwrap();
        medium_priority_spawner.spawn(can::run_rx(can_rx)).unwrap();
        medium_priority_spawner.spawn(drivers::sensors::gps::run(gps)).unwrap(); // TODO: priority?
        medium_priority_spawner.spawn(flash::run(flash)).unwrap();
    }

    #[cfg(feature="gcs")]
    high_priority_spawner.spawn(gcs::run(gcs, iwdg)).unwrap();

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
