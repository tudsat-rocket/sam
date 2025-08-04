#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::{Adc, Instance};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{Config, gpio};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Delay, Duration, Ticker, Timer};
use shared_types::IoBoardRole;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

pub mod can;
pub mod roles;

use can::*;
use roles::common::*;
use roles::*;

type OutputStateChannel = PubSubChannel<CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStateSubscriber = Subscriber<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStatePublisher = Publisher<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
pub static OUTPUT_STATE: StaticCell<OutputStateChannel> = StaticCell::new();

// TODO
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

    //USB_LP_CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN1>;
    //USB_HP_CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN1>;

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

// == J6
#[allow(dead_code)]
pub enum Input0 {
    Disabled,
    Uart(Uart<'static, Async>), //USART1 DMA1_CH4, DMA1_CH5
    I2c(I2c<'static, Async, Master>), //I2C1 DMA1_CH6, DMA1_CH7
}

// == J9
#[allow(dead_code)]
pub enum Input1 {
    Disabled,
    Uart(Uart<'static, Async>), //USART2, DMA1_CH7, DMA1_CH6
    I2c(I2c<'static, Async, Master>),
}

// == J7
#[allow(dead_code)]
pub enum Input2 {
    Disabled,
    Uart(Uart<'static, Async>),
    Adc,
}

// == J10
#[allow(dead_code)]
pub enum Input3 {
    Disabled,
    Uart(Uart<'static, Async>), //USART3, DMA1_CH2, DMA1_CH3
    //I2c(I2c<'static, Async, Master>), //I2C2, DMA1_CH4, DMA1_CH5
    //Gpio(Input<'static>, Input<'static>), //PB10 //PB11
}

// == J8
#[allow(dead_code)]
pub enum Input4 { //TODO add timer
    Disabled,
    Uart(Uart<'static, embassy_stm32::mode::Async>), //UART4, DMA2_CH5, DMA2_CH3
    Adc
}

// == J11
#[allow(dead_code)]
pub enum Input5 {
    Disabled,
    Adc,
}

/// Abstraction for the inputs and outputs. These are the same for all roles.
#[allow(dead_code)]
struct BoardIo {
    pub input0: Input0,
    pub input1: Input1,
    pub input2: Input2,
    pub input3: Input3,
    pub input4: Input4,
    pub input5: Input5,
    pub leds: (Output<'static>, Output<'static>, Output<'static>),
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
