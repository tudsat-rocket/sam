#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::adc::Instance;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::Uart;
use embassy_stm32::rcc::*;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{Config, Peri, gpio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::{Delay, Duration, Ticker, Timer};
use shared_types::IoBoardRole;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use io_module_firmware::can::{configure, spawn, CAN_IN, CAN_OUT};
use io_module_firmware::OUTPUT_STATE;
use io_module_firmware::common::{run_i2c_sensors, run_leds, run_output_control_via_can, run_can_to_uart, run_uart_to_can};

const ROLE_ID: IoBoardRole = IoBoardRole::Payload; //TODO change in sam::shared_types to Pavionic
const ID_LED_PATTERN: [u8; 8] = [0, 0, 0, 0, 1, 0, 0, 0];


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

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES239_5);

    let led_red = Output::new(p.PC7, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PC8, Level::Low, Speed::Low);
    let led_green = Output::new(p.PC9, Level::Low, Speed::Low);
    let leds = (led_red, led_yellow, led_green);

    //input0 = UART
    let uart_config = embassy_stm32::usart::Config::default();
    let input0 = Uart::new(p.USART1, p.PB7, p.PB6, io_module_firmware::Irqs, p.DMA1_CH4, p.DMA1_CH5, uart_config);
    let (uart_tx, uart_rx) = input0.unwrap().split();
    //maybe e.g. input1 = i2c ...


    //configure CANs
    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());
    let output_state = OUTPUT_STATE.init(PubSubChannel::new());

    let mut can1 = embassy_stm32::can::Can::new(p.CAN1, p.PB8, p.PB9, io_module_firmware::Irqs);
    io_module_firmware::can::configure(&mut can1, ROLE_ID);
    let mut can2 = embassy_stm32::can::Can::new(p.CAN2, p.PB12, p.PB13, io_module_firmware::Irqs);
    io_module_firmware::can::configure(&mut can2, ROLE_ID);


    // Start high priority executor
    interrupt::SPI2.set_priority(Priority::P6);
    let high_priority_spawner = io_module_firmware::EXECUTOR_HIGH.start(interrupt::SPI2);

    // Receive CAN message and forward it via UART
    // failsafe behaviour if connection to FC is lost.
    high_priority_spawner.spawn(run_can_to_uart(can_in.subscriber().unwrap(), uart_tx)).unwrap();
    
    //Receive UART-data and forward it via CAN //TODO "spawner" right? and why spawner instead of high_priority_spawner?
    spawner.spawn(run_uart_to_can(can_out.publisher().unwrap(), uart_rx)).unwrap();


    // Run CAN bus, publishing received messages on can_in and transmitting messages
    // published on can_out. //TODO does pavionic need this?
    io_module_firmware::can::spawn(can1, spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;

    // Run the independent watchdog TODO what does an independent watchdog do?
    spawner.spawn(io_module_firmware::run_iwdg(iwdg)).unwrap();

    // Every IO board occasionally reports its temperature, drive voltage and current
    //TODO currently deactivated because of CAN error
    /*
    spawner //TODO change pins;
        .spawn(io_module_firmware::common::run_power_report(
            can_out.publisher().unwrap(),
            adc,
            p.PA7.into(),
            p.PA6.into(),
            p.PC5.into(),
            p.PC4.into(),
            ROLE_ID,
            io_module_firmware::DriveVoltage::Battery,
        ))
        .unwrap();
     */

    
    // Set the LEDs based on the output state.
    let led_output_state_sub = output_state.subscriber().unwrap();
    spawner.spawn(run_leds(leds, led_output_state_sub, ID_LED_PATTERN)).unwrap();
}
