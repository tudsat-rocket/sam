#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::Config;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Delay, Duration, Ticker, Timer};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod actors;
mod can;
mod sensors;

use actors::*;
use can::*;
use sensors::*;

type OutputStateChannel = PubSubChannel<CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStateSubscriber = Subscriber<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStatePublisher = Publisher<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
static OUTPUT_STATE: StaticCell<OutputStateChannel> = StaticCell::new();

// TODO
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MEDIUM: InterruptExecutor = InterruptExecutor::new();

embassy_stm32::bind_interrupts!(struct Irqs {


    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;
});

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(embassy_stm32::usart::Config),
    I2c(Hertz, embassy_stm32::i2c::Config),
}

#[allow(dead_code)]
enum DriveVoltage {
    Battery,
    ChargeBus,
    Regulator5V,
    HighCurrent,
}

#[embassy_executor::task]
async fn run_iwdg(mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(256));
    loop {
        iwdg.pet();
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(72));
    config.rcc.hclk = Some(Hertz::mhz(72));
    config.rcc.pclk1 = Some(Hertz::mhz(36));
    config.rcc.pclk2 = Some(Hertz::mhz(72));
    config.rcc.adcclk = Some(Hertz::mhz(14));
    let mut p = embassy_stm32::init(config);

    // TODO: check if the current boot is a watchdog reset and react
    // appropriately -> see also fc firmware
    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512000 == 512ms 

    // Run the independent watchdog
    iwdg.unleash();
    low_priority_spawner.spawn(run_iwdg(iwdg)).unwrap();

    #[cfg(feature = "guard")]
    low_priority_spawner.spawn(guard_task()).unwrap();
}

#[interrupt]
unsafe fn I2C1_EV() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn I2C1_ER() {
    EXECUTOR_MEDIUM.on_interrupt()
}

#[allow(dead_code)]
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
