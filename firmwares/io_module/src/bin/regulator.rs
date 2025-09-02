#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_pwm::{SimplePWM, PWMPin};
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{can, interrupt};
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker};
use shared_types::{CanBusMessage, CanBusMessageId, FinBoardDataMessage, IoBoardRole, IoBoardSensorMessage};

use {defmt_rtt as _, panic_probe as _};

use io_module_firmware::can::{configure, spawn, CAN_IN, CAN_OUT};
use io_module_firmware::common::{run_i2c_sensors, run_leds, run_output_control_via_can};
use io_module_firmware::OUTPUT_STATE;

const ROLE_ID: IoBoardRole = IoBoardRole::Regulator;
const ID_LED_PATTERN: [u8; 8] = [0, 0, 1, 0, 1, 0, 0, 0];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    //configure "p"
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        mode: embassy_stm32::rcc::HseMode::Oscillator,
        freq: Hertz::mhz(8), // our high-speed external oscillator speed
    });
    config.rcc.sys = embassy_stm32::rcc::Sysclk::HSE;

    //TODO maybe change?
    // 72 MHz
    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
        src: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL9,
    });

    // advanced high performace bus: 72 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV1;

    // peripheral bus 1: 36 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2;

    // peripheral bus 2: 72 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    // analog digital converter: 12 MHz (max: 14MHz)
    config.rcc.adc_pre = ADCPrescaler::DIV6;
    let mut p = embassy_stm32::init(config);

    defmt::info!("Running as role {:?}", defmt::Debug2Format(&ROLE_ID));

    //configure p.watchdog and p.pins
    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    iwdg.unleash();
    spawner.spawn(io_module_firmware::run_iwdg(iwdg)).err();

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC9, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(10);
    let i2c_freq = Hertz::khz(100);

    let input0 = I2c::new(p.I2C1, p.PB6, p.PB7, io_module_firmware::Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_freq, i2c_config);
    let input1 =
        I2c::new(p.I2C2, p.PB10, p.PB11, io_module_firmware::Irqs, p.DMA1_CH4, p.DMA1_CH5, i2c_freq, i2c_config);

    //configure CANs
    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());
    let output_state = OUTPUT_STATE.init(PubSubChannel::new());

    let mut can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, io_module_firmware::Irqs);
    io_module_firmware::can::configure(&mut can1, ROLE_ID);
    let mut can2 = embassy_stm32::can::Can::new(p.CAN2, p.PB12, p.PB13, io_module_firmware::Irqs);
    io_module_firmware::can::configure(&mut can2, ROLE_ID);

    interrupt::SPI2.set_priority(Priority::P6);
    let high_priority_spawner = io_module_firmware::EXECUTOR_HIGH.start(interrupt::SPI2);

    // Run CAN bus, publishing received messages on can_in and transmitting messages
    // published on can_out.
    io_module_firmware::can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    spawner //TODO change pins
        .spawn(io_module_firmware::common::run_power_report(
            can_out.publisher().unwrap(),
            adc,
            p.PA7.into(),
            p.PA6.into(),
            p.PC5.into(),
            p.PC4.into(),
            ROLE_ID,
            io_module_firmware::DriveVoltage::Battery,
        )).unwrap();


    // Allow role to spawn its own tasks
    // Run I2C ADC inputs on COM1 & COM3, with 2 sensors on each.
    spawner
        .spawn(run_i2c_sensors(
            Some(input0),
            Some(input1),
            None, //Some((p0, p1)),
            can_out.publisher().unwrap(),
            ROLE_ID,
            Duration::from_hz(i2c_freq.0 as u64),
        ))
        .unwrap();
    */

    // Set the LEDs based on the output state.
    // FIXME: decide for output and set correct pin
    let pwm_pin = PwmPin::new_ch3(p.PB0, OutputType::PushPull);
    let setpoint = 40.0f; // TODO set as constant
    spawner.spawn(run_control_loop(SimplePwm::new(p.TIM3, None, None, Some(pwm_pin), None, Hertz::hz(50), CountingMode::EdgeAlignedUp), adc, setpoint)).unwrap();
}

#[embassy_executor::task]
async fn run_control_loop(servo_out: SimplePWM, sensor_in_adc: Adc, setpoint: f32) -> ! {
    const P: f32 = 0.1f;
    const I: f32 = 0.1f;
    const D: f32 = 0.1f;

    // Assume control rate of 1000Hz for now
    let mut ticker = Ticker::every(Duration::from_hz(1000));

    let mut error = 0.0f;
    let mut out = 0.0f;

    loop {
        // setpoint deviation
        error = setpoint - sensor_input_subscriber.read_pressure().unwrap_or(0);

        // transfer function error -> out

        // set ouput
        servo_out.set_duty_cylce_percent(out);
    }
}
