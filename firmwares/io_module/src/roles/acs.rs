use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Ticker};

use shared_types::{CanBusMessageId, IoBoardRole};

use crate::{BoardIo, CanInChannel, CanOutChannel, InputMode};

const OUTPUT_FAILSAFE_TIMEOUT_MILLIS: u64 = 500;

pub struct Acs {}

impl crate::roles::BoardRole for Acs {
    const ROLE_ID: IoBoardRole = IoBoardRole::Acs;

    // TODO
    fn input1_mode() -> InputMode {
        InputMode::I2c(Hertz::khz(100), embassy_stm32::i2c::Config::default())
    }

    fn spawn(
        io: BoardIo,
        high_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
        can_in: &'static CanInChannel,
        can_out: &'static CanOutChannel
    ) {
        let output_handle = crate::roles::common::run_outputs(
            io.output1,
            io.output2,
            io.output3,
            io.output4,
            can_in.subscriber().unwrap(),
            CanBusMessageId::IoBoardCommand(Self::ROLE_ID, 0).into(),
            Some(Duration::from_millis(OUTPUT_FAILSAFE_TIMEOUT_MILLIS))
        );

        let sensor_handle = run_sensors(
            can_out.publisher().unwrap(),
        );

        high_priority_spawner.spawn(output_handle).unwrap();
        low_priority_spawner.spawn(sensor_handle).unwrap();

        // TODO: report back general temperature and voltages?
    }
}

#[embassy_executor::task]
pub async fn run_sensors(
    // TODO: input
    _publisher: crate::CanOutPublisher,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    loop {
        ticker.next().await;
    }
}
