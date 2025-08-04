use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::can::Can;
use embassy_stm32::can::{Fifo, StandardId, filter};

use embassy_time::Duration;
use shared_types::{CanBusMessageId, FlightMode, IoBoardRole};

use crate::OutputStateChannel;
use crate::{BoardIo, CanInChannel, CanOutChannel, DriveVoltage, InputMode};

pub mod common;

mod payload;
pub use payload::*;

pub trait BoardRole: Sized {
    const ROLE_ID: IoBoardRole;


    // Determines which voltage sense pin is measured for regular diagnostic messages
    fn drive_voltage() -> DriveVoltage {
        DriveVoltage::Battery
    }

    fn output_failsafe_duration() -> Option<Duration> {
        None
    }

    fn outputs_on_after_flightmode() -> Option<FlightMode> {
        None
    }

    fn input0_mode() -> InputMode {
        InputMode::Disabled
    }

    fn input1_mode() -> InputMode {
        InputMode::Disabled
    }

    fn input2_mode() -> InputMode {
        InputMode::Disabled
    }

    fn input3_mode() -> InputMode {
        InputMode::Disabled
    }

    fn input4_mode() -> InputMode {
        InputMode::Disabled
    }

    fn input5_mode() -> InputMode {
        InputMode::Disabled
    }

    fn spawn(
        io: BoardIo,
        high_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
        can_in: &'static CanInChannel,
        can_out: &'static CanOutChannel,
        output_state: &'static OutputStateChannel,
    );
}
