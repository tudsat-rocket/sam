use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::can::BxcanInstance;
use embassy_stm32::can::bxcan::Can;
use embassy_stm32::can::bxcan::{filter, Fifo, StandardId};
use embassy_stm32::peripherals::CAN;

use embassy_time::Duration;
use shared_types::{CanBusMessageId, FlightMode, IoBoardRole};

use crate::{BoardIo, CanInChannel, CanOutChannel, DriveVoltage, InputMode};
use crate::OutputStateChannel;

pub mod common;

mod acs;
pub use acs::*;

mod recovery;
pub use recovery::*;

mod payload;
pub use payload::*;

pub trait BoardRole: Sized {
    const ROLE_ID: IoBoardRole;

    fn configure_can(can: &mut Can<BxcanInstance<CAN>>) {
        let command_prefix = CanBusMessageId::IoBoardCommand(Self::ROLE_ID, 0).into();
        let command_filter = filter::Mask32::frames_with_std_id(
            StandardId::new(command_prefix).unwrap(),
            StandardId::new(0x7f0).unwrap()
        );

        let telemetry_filter = filter::Mask32::frames_with_std_id(
            StandardId::new(CanBusMessageId::TelemetryBroadcast(0).into()).unwrap(),
            StandardId::new(0x700).unwrap()
        );

        can
            .modify_filters()
            .enable_bank(0, Fifo::Fifo0, command_filter)
            .enable_bank(1, Fifo::Fifo1, telemetry_filter);
    }

    // Determines which voltage sense pin is measured for regular diagnostic messages
    fn drive_voltage() -> DriveVoltage {
        DriveVoltage::Battery
    }

    fn boost_converter_voltage() -> Option<u16> {
        None
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
