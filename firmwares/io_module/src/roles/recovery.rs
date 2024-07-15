use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::time::Hertz;

use shared_types::{CanBusMessageId, IoBoardRole};

use crate::{BoardIo, CanInChannel, CanOutChannel, InputMode};

pub struct Recovery {}

impl crate::roles::BoardRole for Recovery {
    const ROLE_ID: IoBoardRole = IoBoardRole::Recovery;

    // TODO
    fn input1_mode() -> InputMode {
        InputMode::I2c(Hertz::khz(100), embassy_stm32::i2c::Config::default())
    }

    fn spawn(
        io: BoardIo,
        high_priority_spawner: SendSpawner,
        _low_priority_spawner: Spawner,
        can_in: &'static CanInChannel,
        _can_out: &'static CanOutChannel
    ) {
        let output_handle = crate::roles::common::run_outputs(
            io.output1,
            io.output2,
            io.output3,
            io.output4,
            can_in.subscriber().unwrap(),
            CanBusMessageId::IoBoardCommand(Self::ROLE_ID, 0).into(),
            None // TODO: do we want a timeout for the cameras?
        );

        high_priority_spawner.spawn(output_handle).unwrap();

        // TODO: sensors. commonalities with ACS board?
        // TODO: report back general temperature and voltages?
    }
}
