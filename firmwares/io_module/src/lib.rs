#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::InterruptExecutor;
use embassy_stm32::adc::Instance;
use embassy_stm32::dma::*;
use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Duration, Ticker};
use static_cell::StaticCell;

pub mod can;
pub mod common;
pub mod heartbeat;

use can::*;
type OutputStateChannel = PubSubChannel<CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStateSubscriber = Subscriber<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStatePublisher = Publisher<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
pub static OUTPUT_STATE: StaticCell<OutputStateChannel> = StaticCell::new();

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

embassy_stm32::bind_interrupts!(pub struct Irqs {
    CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN1>;
    CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN1>;
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN1>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN1>;

    CAN2_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN2>;
    CAN2_TX => embassy_stm32::can::TxInterruptHandler<CAN2>;
    CAN2_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN2>;
    CAN2_SCE => embassy_stm32::can::SceInterruptHandler<CAN2>;


    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;

    I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;

    USART1 => embassy_stm32::usart::InterruptHandler<USART1>;
    USART2 => embassy_stm32::usart::InterruptHandler<USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
    UART5 => embassy_stm32::usart::InterruptHandler<UART5>;
});

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(embassy_stm32::usart::Config),
    I2c(Hertz, embassy_stm32::i2c::Config),
    Adc,
    Gpio(Pull),
}

#[allow(dead_code)]
pub enum DriveVoltage {
    Battery,
    ChargeBus,
    Regulator5V,
}

#[embassy_executor::task]
pub async fn run_iwdg(mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(256));
    loop {
        iwdg.pet();
        ticker.next().await;
    }
}

#[interrupt]
unsafe fn SPI2() {
    // TODO: (embassy upgrade)
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}
