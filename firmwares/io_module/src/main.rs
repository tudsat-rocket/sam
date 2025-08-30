#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::Config;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Delay, Duration, Ticker, Timer};
use shared_types::IoBoardRole;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod boost;
mod can;
mod roles;

use boost::*;
use can::*;
use roles::common::*;
use roles::*;

type OutputStateChannel = PubSubChannel<CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStateSubscriber = Subscriber<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
type OutputStatePublisher = Publisher<'static, CriticalSectionRawMutex, ([bool; 8], bool), 1, 3, 2>;
static OUTPUT_STATE: StaticCell<OutputStateChannel> = StaticCell::new();

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
    Gpio(Pull),
}

#[allow(dead_code)]
enum DriveVoltage {
    Battery,
    ChargeBus,
    BoostConverter,
    Regulator5V,
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
    I2c(I2c<'static, I2C2, DMA1_CH4, DMA1_CH5>),
}

// == J5
#[allow(dead_code)]
enum Input2 {
    Disabled,
    Uart, // TODO
    Adc,  // TODO
}

// == J4
#[allow(dead_code)]
enum Input3 {
    Disabled,
    Uart, // TODO
    I2c(I2c<'static, I2C1, DMA1_CH6, DMA1_CH7>),
    Gpio(Input<'static, PB6>, Input<'static, PB7>),
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
    pub leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
}

#[embassy_executor::task]
async fn run_iwdg(mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(256));
    loop {
        iwdg.pet();
        ticker.next().await;
    }
}

async fn run_role<R: BoardRole>(p: embassy_stm32::Peripherals, low_priority_spawner: Spawner) {
    defmt::info!("Running as role {:?}", defmt::Debug2Format(&R::ROLE_ID));

    let mut adc = Adc::new(p.ADC1, &mut Delay);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::Cycles239_5);

    let led_red = Output::new(p.PB12, Level::Low, Speed::Low);
    let led_white = Output::new(p.PB13, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PB14, Level::Low, Speed::Low);
    let leds = (led_red, led_white, led_yellow);

    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    iwdg.unleash();

    let input0 = match R::input0_mode() {
        // J7
        _ => Input0::Disabled,
    };

    let (boost_converter, input1) = match (R::boost_converter_voltage(), R::input1_mode()) {
        // J6
        (Some(v), InputMode::Disabled) => (
            Some((
                I2c::new(p.I2C2, p.PB10, p.PB11, Irqs, p.DMA1_CH4, p.DMA1_CH5, Hertz::khz(400), Default::default()),
                v,
            )),
            Input1::Disabled,
        ),
        (Some(_v), _) => unreachable!(),
        (None, InputMode::Uart(config)) => {
            (None, Input1::Uart(Uart::new(p.USART3, p.PB11, p.PB10, Irqs, p.DMA1_CH2, p.DMA1_CH3, config).unwrap()))
        }
        (None, InputMode::I2c(freq, config)) => {
            (None, Input1::I2c(I2c::new(p.I2C2, p.PB10, p.PB11, Irqs, p.DMA1_CH4, p.DMA1_CH5, freq, config)))
        }
        (None, _) => (None, Input1::Disabled),
    };

    let input2 = match R::input2_mode() {
        // J5
        InputMode::Uart(_) => todo!(),
        InputMode::Adc => todo!(),
        _ => Input2::Disabled,
    };

    let input3 = match R::input3_mode() {
        // J4
        InputMode::Uart(_) => todo!(),
        InputMode::I2c(freq, config) => {
            Input3::I2c(I2c::new(p.I2C1, p.PB6, p.PB7, Irqs, p.DMA1_CH6, p.DMA1_CH7, freq, config))
        }
        InputMode::Gpio(pull) => Input3::Gpio(Input::new(p.PB6, pull), Input::new(p.PB7, pull)),
        _ => Input3::Disabled,
    };

    let input4 = match R::input4_mode() {
        // J12
        InputMode::Adc => todo!(),
        _ => Input4::Disabled,
    };

    let input5 = match R::input5_mode() {
        // J13
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
        leds,
    };

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());
    let output_state = OUTPUT_STATE.init(PubSubChannel::new());

    let mut can = embassy_stm32::can::Can::new(p.CAN, p.PA11, p.PA12, Irqs);
    R::configure_can(&mut can);

    // Start high priority executor
    interrupt::SPI2.set_priority(Priority::P6);
    let high_priority_spawner = EXECUTOR_HIGH.start(interrupt::SPI2);

    // Control output state based on received commands, with optional
    // failsafe behaviour if connection to FC is lost.
    high_priority_spawner
        .spawn(run_output_control_via_can(
            can_in.subscriber().unwrap(),
            output_state.publisher().unwrap(),
            R::ROLE_ID,
            R::output_failsafe_duration(),
        ))
        .unwrap();

    // Run output based on published output state
    high_priority_spawner
        .spawn(run_outputs(
            (Output::new(p.PC7, Level::Low, Speed::Low), Output::new(p.PC6, Level::Low, Speed::Low)),
            (Output::new(p.PC9, Level::Low, Speed::Low), Output::new(p.PC8, Level::Low, Speed::Low)),
            (Output::new(p.PB8, Level::Low, Speed::Low), Output::new(p.PB9, Level::Low, Speed::Low)),
            (Output::new(p.PA0, Level::Low, Speed::Low), Output::new(p.PA1, Level::Low, Speed::Low)),
            output_state.subscriber().unwrap(),
        ))
        .unwrap();

    // Run CAN bus, publishing received messages on can_in and transmitting messages
    // published on can_out.
    can::spawn(can, low_priority_spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // Run the independent watchdog
    low_priority_spawner.spawn(run_iwdg(iwdg)).unwrap();

    // Every IO board occasionally reports its temperature, drive voltage and current
    low_priority_spawner
        .spawn(run_power_report(
            can_out.publisher().unwrap(),
            adc,
            p.PA7,
            p.PA6,
            p.PA5,
            p.PB1,
            p.PA4,
            R::ROLE_ID,
            R::drive_voltage(),
        ))
        .unwrap();

    // Run a listener that enables all outputs after a certain
    // flight mode if requested by the rol
    if let Some(fm) = R::outputs_on_after_flightmode() {
        high_priority_spawner
            .spawn(run_outputs_on_after_flightmode(can_in.subscriber().unwrap(), output_state.publisher().unwrap(), fm))
            .unwrap();
    }

    // Run the boost converter if requested by the role.
    if let Some((i2c, target_voltage)) = boost_converter {
        let enable_output = Output::new(p.PB0, Level::Low, Speed::Low);
        low_priority_spawner.spawn(run_boost_converter(i2c, target_voltage, enable_output)).unwrap();
    }

    // Allow role to spawn its own tasks
    R::spawn(io, high_priority_spawner, low_priority_spawner, can_in, can_out, output_state);
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
        ((role_sw1.is_high() as u8) << 4)
            | ((role_sw2.is_high() as u8) << 3)
            | ((role_sw3.is_high() as u8) << 2)
            | ((role_sw4.is_high() as u8) << 1)
            | (role_sw5.is_high() as u8)
    };

    match IoBoardRole::try_from(dip_switch_setting) {
        Ok(IoBoardRole::Acs) => run_role::<Acs>(p, spawner).await,
        Ok(IoBoardRole::Recovery) => run_role::<Recovery>(p, spawner).await,
        Ok(IoBoardRole::Payload) => run_role::<Payload>(p, spawner).await,
        Ok(IoBoardRole::Regulator) => run_role::<Regulator>(p, spawner).await,
        Err(id) => loop {
            defmt::error!("Unknown role: {:05b}", id);
            Timer::after(Duration::from_millis(100)).await;
        },
    }
}

#[interrupt]
unsafe fn SPI2() {
    EXECUTOR_HIGH.on_interrupt()
}
