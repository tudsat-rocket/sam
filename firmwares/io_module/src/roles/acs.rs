use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::peripherals::*;
use embassy_stm32::{gpio::Output, time::Hertz};
use embassy_time::Duration;
use embedded_hal::digital::OutputPin;

use shared_types::IoBoardRole;

use crate::*;
use crate::roles::common::*;

const OUTPUT_FAILSAFE_TIMEOUT_MILLIS: u64 = 500;
const SENSOR_SAMPLING_FREQUENCY_HZ: u64 = 20;

pub struct Acs {}

impl crate::roles::BoardRole for Acs {
    const ROLE_ID: IoBoardRole = IoBoardRole::Acs;

    fn drive_voltage() -> DriveVoltage {
        DriveVoltage::ChargeBus
    }

    // We want to turn off the valves if we don't hear anything from
    // the FC anymore.
    fn output_failsafe_duration() -> Option<Duration> {
        Some(Duration::from_millis(OUTPUT_FAILSAFE_TIMEOUT_MILLIS))
    }

    // Read I2C sensors at COM1
    fn input1_mode() -> InputMode {
        let mut config = embassy_stm32::i2c::Config::default();
        config.timeout = Duration::from_millis(10);
        InputMode::I2c(Hertz::khz(100), config)
    }

    // Read I2C sensors at COM3
    fn input3_mode() -> InputMode {
        let mut config = embassy_stm32::i2c::Config::default();
        config.timeout = Duration::from_millis(10);
        InputMode::I2c(Hertz::khz(100), config)
    }

    fn spawn(
        io: BoardIo,
        _high_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
        _can_in: &'static CanInChannel,
        can_out: &'static CanOutChannel,
        output_state: &'static OutputStateChannel,
    ) {
        // Run I2C ADC inputs on COM1 & COM3, with 2 sensors on each.
        match (io.input1, io.input3) {
            (Input1::I2c(i2c2), Input3::I2c(i2c1)) => {
                low_priority_spawner.spawn(run_i2c_sensors(
                    Some(i2c2),
                    Some(i2c1),
                    None,
                    can_out.publisher().unwrap(),
                    Self::ROLE_ID,
                    Duration::from_hz(SENSOR_SAMPLING_FREQUENCY_HZ),
                )).unwrap();
            },
            //_ => unreachable!(),
            _ => {},
        }

        // Set the LEDs based on the output state and whether the failsafe
        // condition was met;
        let led_output_state_sub = output_state.subscriber().unwrap();
        low_priority_spawner.spawn(run_leds(io.leds, led_output_state_sub)).unwrap();
    }
}

#[embassy_executor::task]
async fn run_leds(
    leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
    mut output_state_subscriber: OutputStateSubscriber,
) -> ! {
    let (mut led_red, mut led_white, mut led_yellow) = leds;
    led_red.set_low();
    led_white.set_high();
    led_yellow.set_high();

    loop {
        let (outputs, failsafe) = output_state_subscriber.next_message_pure().await;
        let _ = led_red.set_state((!failsafe).into());
        let _ = led_white.set_state((!outputs[0]).into());
        let _ = led_yellow.set_state((!outputs[2]).into());
    }
}
