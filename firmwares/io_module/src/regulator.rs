use shared_types::IoBoardRole;

use embassy_executor::Spawner;
use embassy_pwm::SimplePWM;
use embassy_time::{Duration, Ticker};

use crate::roles::common::*;
use crate::*;

pub struct Regulator {}

impl crate::roles::BoardRole for Regulator {
    const ROLE_ID: IoBoardRole = IoBoardRole::Regulator;

    fn spawn(
        servo_out: SimplePWM,
        //TODO: high priority spawner?
        sensor_input_subscriber: SensorInputSubscriber,
        low_priority_spawner: Spawner,
        setpoint: f32,
    ) {
        // associate servo outputs here ? build sensor_input_subscriber here?

        low_priority_spawner.spawn(run_control_loop(servo_out, sensor_input_subscriber, setpoint)).unwrap();
    }
}

#[embassy_executor::task]
async fn run_control_loop(servo_out: SimplePWM, sensor_input_subscriber: SensorInputSubscriber, setpoint: f32) -> ! {
    const P: f32 = 0.1f;
    const I: f32 = 0.1f;
    const D: f32 = 0.1f;

    // Assume control rate of 1000Hz for now
    let mut ticker = Ticker::every(Duration::from_hz(1000));

    let mut error = 0.0f;
    let mut out = 0.0f;

    loop {
        // gate for flight mode?

        // setpoint deviation
        error = setpoint - sensor_input_subscriber.read_pressure().unwrap_or(0);

        // transfer function

        // set ouput
        servo_out.set_duty_cylce_percent(out);
    }
}
