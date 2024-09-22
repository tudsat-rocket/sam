use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::time::Hertz;

use shared_types::{FlightMode, IoBoardRole};

use crate::*;

const SENSOR_SAMPLING_FREQUENCY_HZ: u64 = 10;
const ID_LED_PATTERN: [u8; 8] = [0, 0, 0, 0, 1, 0, 1, 0];

pub struct Recovery {}

impl crate::roles::BoardRole for Recovery {
    const ROLE_ID: IoBoardRole = IoBoardRole::Recovery;

    // Read I2C sensors at COM1
    fn input1_mode() -> InputMode {
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
        if let Input1::I2c(i2c2) = io.input1 {
            low_priority_spawner.spawn(run_i2c_sensors(
                Some(i2c2),
                None,
                can_out.publisher().unwrap(),
                Self::ROLE_ID,
                Duration::from_hz(SENSOR_SAMPLING_FREQUENCY_HZ),
            )).unwrap();
        }

        // Set the LEDs based on the output state.
        let led_output_state_sub = output_state.subscriber().unwrap();
        low_priority_spawner.spawn(run_leds(
            io.leds,
            led_output_state_sub,
            ID_LED_PATTERN,
        )).unwrap();
    }
}
