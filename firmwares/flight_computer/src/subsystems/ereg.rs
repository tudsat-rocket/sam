use defmt::info;
use embassy_time::{Duration, Instant};
use shared_types::can::{
    CanMessage, SubsystemCommand, SubsystemInfo,
    engine::LaunchCode,
    ereg::{EregInfoMsg, EregState},
};

use super::{ConnectivityStatus, Subsystem};
use crate::can::CanTxPublisher;

const CONECTIVITY_THRESHOLD: Duration = Duration::from_secs(2);
const MIN_MSG_INTERVAL: Duration = Duration::from_millis(20);

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct EregMeasurements {
    pub pressure_n2: u16,
    pub pressure_ox: u8,
    pub regulator_valve: u8,
    pub vent_valve: u8,
    pub ereg_state: EregState,
}

impl From<EregInfoMsg> for EregMeasurements {
    fn from(value: EregInfoMsg) -> Self {
        EregMeasurements {
            pressure_n2: value.pressure_n2,
            pressure_ox: value.pressure_ox,
            regulator_valve: value.regulator_valve,
            vent_valve: value.vent_valve,
            ereg_state: value.ereg_state,
        }
    }
}

#[derive(Default)]
pub struct EregSubsystem {
    target_state: EregState,
    // last state reported by the ereg
    current_state: Option<EregState>,
    // last measurements reported by the ereg
    measurements: Option<EregMeasurements>,
    // timestamp, for keeping track of connection quality
    last_received_message: Option<(CanMessage, Instant)>,
    // timestamp, for preventing successive repeats
    last_sent_state_message: Option<Instant>,
}
impl EregSubsystem {
    /// Send messages based on internal state, update internal state.
    pub fn tick(&mut self, can_publisher: &mut CanTxPublisher) {
        self.check_publish_state(can_publisher);
    }
    /// Update internal state based on received CAN messages.
    pub fn process_message(&mut self, message: CanMessage) {
        if let CanMessage::Info(SubsystemInfo::Ereg(msg)) = message {
            self.last_received_message = Some((message, Instant::now()));
            self.measurements = Some(msg.into());
            self.current_state = Some(msg.ereg_state);
        }
    }
    pub fn set_target_state(&mut self, state: EregState) {
        if self.target_state != state {
            self.last_sent_state_message = None;
        }
        // should we verify this here?
        self.target_state = state;
    }
    pub fn get_ereg_measurements(&self) -> Option<EregMeasurements> {
        self.measurements
    }

    fn check_publish_state(&mut self, can_publisher: &mut CanTxPublisher) {
        let states_differ = self.target_state != self.current_state.unwrap_or_default(); // check
        let min_time_elapsed =
            self.last_sent_state_message.map(|inst| inst.elapsed() > MIN_MSG_INTERVAL).unwrap_or(true);
        let unresponsive = self.conectivity_status() != ConnectivityStatus::Online;

        if min_time_elapsed && (states_differ | unresponsive) {
            let msg = CanMessage::Cmd(SubsystemCommand::SetEregState(self.target_state));
            // NOTE: think about publish_immediate
            can_publisher.publish_immediate(msg);
            // info!("Sending 'SetEregState' Can Message");
            self.last_sent_state_message = Some(Instant::now());
        }
    }
}

impl Subsystem for EregSubsystem {
    fn conectivity_status(&self) -> super::ConnectivityStatus {
        match self.last_received_message {
            None => ConnectivityStatus::Offline,
            Some((_, inst)) => {
                if inst.elapsed() > CONECTIVITY_THRESHOLD {
                    ConnectivityStatus::OfflineAgain
                } else {
                    ConnectivityStatus::Online
                }
            }
        }
    }
}
