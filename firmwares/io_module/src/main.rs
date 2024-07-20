#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::Config;
use embassy_stm32::gpio::{Input, Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{Priority, InterruptExt};
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use shared_types::IoBoardRole;

use {defmt_rtt as _, panic_probe as _};

mod can;
mod roles;

use can::*;
use roles::*;

// TODO
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

embassy_stm32::bind_interrupts!(struct Irqs {
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN>;
    USB_LP_CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN>;
    USB_HP_CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN>;

    I2C1_EV => embassy_stm32::i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C1>;

    I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;

    USART3 => embassy_stm32::usart::InterruptHandler<USART3>;
});

#[allow(dead_code)]
enum InputMode {
    Disabled,
    Uart(embassy_stm32::usart::Config),
    I2c(Hertz, embassy_stm32::i2c::Config),
    Adc,
}

// == J7
#[allow(dead_code)]
enum Input0 {
    Disabled,
    //Uart(), // TODO: missing embassy support for asynchronous-only?
}

// == J6
#[allow(dead_code)]
enum Input1 {
    Disabled,
    Uart(Uart<'static, USART3, DMA1_CH2, DMA1_CH3>),
    I2c(I2c<'static, I2C2, DMA1_CH1, DMA1_CH2>),
}

// == J5
#[allow(dead_code)]
enum Input2 {
    Disabled,
    Uart, // TODO
    Adc, // TODO
}

// == J4
#[allow(dead_code)]
enum Input3 {
    Disabled,
    Uart, // TODO
    I2c(I2c<'static, I2C1, DMA1_CH5, DMA1_CH4>),
}

// == J12
#[allow(dead_code)]
enum Input4 {
    Disabled,
    Adc, // TODO
}

// == J13
#[allow(dead_code)]
enum Input5 {
    Disabled,
    Adc, // TODO
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
    // TODO: for now these are just straight outputs. we'll have to come up with
    // an alternative when we want to actually use these as serve outputs.
    pub output1: (Output<'static, PB8>, Output<'static, PB9>),
    pub output2: (Output<'static, PA0>, Output<'static, PA1>),
    pub output3: (Output<'static, PC9>, Output<'static, PC8>),
    pub output4: (Output<'static, PC7>, Output<'static, PC6>),
    pub leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>)
}

#[embassy_executor::task]
async fn iwdg_task(mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(256));
    loop {
        iwdg.pet();
        ticker.next().await;
    }
}

async fn run_role<R: BoardRole>(p: embassy_stm32::Peripherals, low_priority_spawner: Spawner) {
    defmt::info!("Running as role {:?}", defmt::Debug2Format(&R::ROLE_ID));

    let led_red = Output::new(p.PB12, Level::Low, Speed::Low);
    let led_white = Output::new(p.PB13, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PB14, Level::Low, Speed::Low);
    let leds = (led_red, led_white, led_yellow);

    // TODO: label by id on the silkscreen
    let output1 = (Output::new(p.PB8, Level::Low, Speed::Low), Output::new(p.PB9, Level::Low, Speed::Low));
    let output2 = (Output::new(p.PA0, Level::Low, Speed::Low), Output::new(p.PA1, Level::Low, Speed::Low));
    let output3 = (Output::new(p.PC9, Level::Low, Speed::Low), Output::new(p.PC8, Level::Low, Speed::Low));
    let output4 = (Output::new(p.PC7, Level::Low, Speed::Low), Output::new(p.PC6, Level::Low, Speed::Low));

    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    iwdg.unleash();

    let input0 = match R::input0_mode() { // J7
        _ => Input0::Disabled,
    };

    let input1 = match R::input1_mode() { // J6
        InputMode::Uart(config) => Input1::Uart(Uart::new(p.USART3, p.PB11, p.PB10, Irqs, p.DMA1_CH2, p.DMA1_CH3, config).unwrap()),
        InputMode::I2c(freq, config) => Input1::I2c(I2c::new(p.I2C2, p.PB10, p.PB11, Irqs, p.DMA1_CH1, p.DMA1_CH2, freq, config)),
        _ => Input1::Disabled,
    };

    let input2 = match R::input2_mode() { // J5
        InputMode::Uart(_) => todo!(),
        InputMode::Adc => todo!(),
        _ => Input2::Disabled,
    };

    let input3 = match R::input3_mode() { // J4
        InputMode::Uart(_) => todo!(),
        InputMode::I2c(freq, config) => Input3::I2c(I2c::new(p.I2C1, p.PB6, p.PB7, Irqs, p.DMA1_CH5, p.DMA1_CH4, freq, config)),
        _ => Input3::Disabled,
    };

    let input4 = match R::input4_mode() { // J12
        InputMode::Adc => todo!(),
        _ => Input4::Disabled,
    };

    let input5 = match R::input5_mode() { // J13
        InputMode::Adc => todo!(),
        _ => Input5::Disabled,
    };

    let io = BoardIo {
        input0,
        input1,
        input2,
        input3,
        input4,
        input5,
        output1,
        output2,
        output3,
        output4,
        leds,
    };

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());

    let mut can = embassy_stm32::can::Can::new(p.CAN, p.PA11, p.PA12, Irqs);
    R::configure_can(&mut can);

    // Start high priority executor
    interrupt::SPI2.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::SPI2);

    R::spawn(io, high_priority_spawner, low_priority_spawner, can_in, can_out);
    can::spawn(can, low_priority_spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;
    low_priority_spawner.spawn(iwdg_task(iwdg)).unwrap();
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

    // We need to borrow the peripherals for the pins so we can later pass the
    // entire Peripherals struct to the role function.
    let dip_switch_setting = {
        let role_sw1 = Input::new(&mut p.PC4, embassy_stm32::gpio::Pull::Down);
        let role_sw2 = Input::new(&mut p.PC5, embassy_stm32::gpio::Pull::Down);
        let role_sw3 = Input::new(&mut p.PB15, embassy_stm32::gpio::Pull::Down);
        let role_sw4 = Input::new(&mut p.PA8, embassy_stm32::gpio::Pull::Down);
        let role_sw5 = Input::new(&mut p.PA9, embassy_stm32::gpio::Pull::Down);
        ((role_sw1.is_high() as u8) << 4) |
            ((role_sw2.is_high() as u8) << 3) |
            ((role_sw3.is_high() as u8) << 2) |
            ((role_sw4.is_high() as u8) << 1) |
            (role_sw5.is_high() as u8)
    };

    match IoBoardRole::try_from(dip_switch_setting) {
        Ok(IoBoardRole::Acs) => run_role::<Acs>(p, spawner).await,
        Ok(IoBoardRole::Recovery) => run_role::<Recovery>(p, spawner).await,
        Ok(IoBoardRole::Payload) => run_role::<Payload>(p, spawner).await,
        Err(id) => {
            loop {
                defmt::error!("Unknown role: {:05b}", id);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

#[interrupt]
unsafe fn SPI2() {
    EXECUTOR_HIGH.on_interrupt()
}
