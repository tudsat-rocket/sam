#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::Duration;
use shared_types::{CanBusMessage, CanBusMessageId, FinBoardDataMessage, IoBoardRole, IoBoardSensorMessage};

use {defmt_rtt as _, panic_probe as _};

use io_module_firmware::can::{CAN_IN, CAN_OUT, configure, spawn};
use io_module_firmware::common::{run_i2c_sensors, run_leds, run_output_control_via_can};
use io_module_firmware::{OUTPUT_STATE, heartbeat};

const ROLE_ID: IoBoardRole = IoBoardRole::Recovery;
const ID_LED_PATTERN: [u8; 8] = [0, 0, 0, 0, 1, 0, 1, 0];
// const ID_LED_PATTERN: [u8; 8] = [0, 0, 0, 0, 1, 0, 0, 1];

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
    spawner.spawn(io_module_firmware::run_iwdg(iwdg).unwrap());

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC9, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(10);
    let i2c_freq = Hertz::khz(100);

    // let input0 = I2c::new(p.I2C1, p.PB6, p.PB7, io_module_firmware::Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_freq, i2c_config);
    let input0 = I2c::new(p.I2C1, p.PB6, p.PB7, io_module_firmware::Irqs, p.DMA1_CH6, p.DMA1_CH7, i2c_config);
    let input1 = I2c::new(p.I2C2, p.PB10, p.PB11, io_module_firmware::Irqs, p.DMA1_CH4, p.DMA1_CH5, i2c_config);

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

    // Control output state based on received commands, with optional
    // failsafe behaviour if connection to FC is lost.
    high_priority_spawner.spawn(
        run_output_control_via_can(can_in.subscriber().unwrap(), output_state.publisher().unwrap(), ROLE_ID, None)
            .unwrap(),
    );

    // temporary heartbeat for thermal test
    high_priority_spawner
        .spawn(crate::heartbeat::run(can_out.publisher().unwrap(), can_in.subscriber().unwrap()).unwrap());
    spawner.spawn(crate::heartbeat::run_leds(leds).unwrap());

    // Run output based on published output state (HC_OUTS) //TODO apparently not needed
    /*high_priority_spawner
    .spawn(run_outputs(
        (Output::new(p.PC7, Level::Low, Speed::Low), Output::new(p.PC6, Level::Low, Speed::Low)),
        (Output::new(p.PC9, Level::Low, Speed::Low), Output::new(p.PC8, Level::Low, Speed::Low)),
        (Output::new(p.PB8, Level::Low, Speed::Low), Output::new(p.PB9, Level::Low, Speed::Low)),
        (Output::new(p.PA0, Level::Low, Speed::Low), Output::new(p.PA1, Level::Low, Speed::Low)),
        output_state.subscriber().unwrap(),
    ))
    .unwrap();*/

    // Run CAN bus, publishing received messages on can_in and transmitting messages
    // published on can_out.
    io_module_firmware::can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // Every IO board occasionally reports its temperature, drive voltage and current
    //TODO currently deactivated because of CAN error
    /*
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
    let led_output_state_sub = output_state.subscriber().unwrap();
    // spawner.spawn(run_leds(leds, led_output_state_sub, ID_LED_PATTERN)).unwrap();
}
