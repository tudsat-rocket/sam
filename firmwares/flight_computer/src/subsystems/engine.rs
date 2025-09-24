use defmt::info;
use embassy_time::{Duration, Instant};
use shared_types::can::{
    CanMessage, SubsystemCommand, SubsystemInfo,
    engine::{EngineInfoMsg, EngineState, LaunchCode},
};

use super::{ConnectivityStatus, Subsystem};
use crate::can::CanTxPublisher;

const CONECTIVITY_THRESHOLD: Duration = Duration::from_secs(2);
const MIN_MSG_INTERVAL: Duration = Duration::from_millis(1000);

#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct EngineMeasurements {
    /// Pressure at the bottom of the oxidizer tank
    pub pressure_ox: u8,
    /// Pressure at combustion chamber
    pub pressure_combustion_chamber: u16,
    /// Temperature of the oxidizer / oxidizer tank
    pub temp_ox: u8,
    /// How open the main valve is: 1 = open to 0 = closed
    pub main_valve: u8,
    /// How open the main valve is: 1 = open to 0 = closed
    pub fill_and_dump_valve: u8,
}
impl From<EngineInfoMsg> for EngineMeasurements {
    fn from(value: EngineInfoMsg) -> Self {
        EngineMeasurements {
            pressure_ox: value.pressure_ox,
            pressure_combustion_chamber: value.pressure_combustion_chamber,
            temp_ox: value.temp_ox,
            main_valve: value.main_valve,
            fill_and_dump_valve: value.fill_and_dump_valve,
        }
    }
}

#[derive(Default)]
pub struct EngineSubsystem {
    target_state: EngineState,
    // last state reported by the engine
    current_state: Option<EngineState>,
    // last measurements reported by the engine
    measurements: Option<EngineMeasurements>,
    // timestamp, for keeping track of connection quality
    last_received_message: Option<(CanMessage, Instant)>,
    // timestamp for preventing successive repeats
    last_sent_state_message: Option<Instant>,
    // Contains launch code
    ignition: Option<u64>,
}
impl EngineSubsystem {
    /// Send messages based on internal state, update internal state.
    pub async fn tick(&mut self, can_publisher: &mut CanTxPublisher) {
        self.check_publish_state(can_publisher).await;
    }
    /// Update internal state based on received CAN messages.
    pub fn process_message(&mut self, message: CanMessage) {
        if let CanMessage::Info(SubsystemInfo::Engine(msg)) = message {
            self.last_received_message = Some((message, Instant::now()));
            self.measurements = Some(msg.into());
            self.current_state = Some(msg.engine_state);
        }
    }
    pub fn set_target_state(&mut self, state: EngineState) {
        if state == EngineState::Abort {
            self.ignition = None;
        }
        if self.target_state != state {
            // reset spam protection
            self.last_sent_state_message = None;
        }
        // should we verify this here?
        self.target_state = state;
    }
    pub fn set_ignition_true(&mut self, code: u64) {
        self.target_state = EngineState::Ignition;
        self.ignition = Some(code);
    }
    pub fn get_engine_measurements(&self) -> Option<EngineMeasurements> {
        self.measurements
    }

    async fn check_publish_state(&mut self, can_publisher: &mut CanTxPublisher) {
        let mut needs_publishing: bool = match self.target_state {
            EngineState::Disarmed
            | EngineState::Filling
            | EngineState::Ready
            | EngineState::Ignition
            | EngineState::Abort
            | EngineState::Scrub
            | EngineState::Dump => true,
            EngineState::Burn => false,
        };
        let state_automatically_changed = self.current_state == Some(EngineState::Burn);
        let states_differ =
            !state_automatically_changed && (self.target_state != self.current_state.unwrap_or_default());
        // TODO: check anomalies with automatic burn state
        let min_time_elapsed =
            self.last_sent_state_message.map(|inst| inst.elapsed() > MIN_MSG_INTERVAL).unwrap_or(true);
        let unresponsive = self.conectivity_status() != ConnectivityStatus::Online;

        if needs_publishing && min_time_elapsed && (states_differ | unresponsive) {
            let msg = match self.ignition {
                // Don't send ignition in abort state.
                Some(code) if self.target_state == EngineState::Ignition => {
                    CanMessage::Cmd(SubsystemCommand::IgniteEngine(LaunchCode(code)))
                }
                Some(_) | None => CanMessage::Cmd(SubsystemCommand::SetEngineState(self.target_state)),
            };
            // NOTE: think about publish_immediate
            can_publisher.publish_immediate(msg);
            // info!("Sending 'SetEngineState' Can Message");
            self.last_sent_state_message = Some(Instant::now());
        }
    }
}

impl Subsystem for EngineSubsystem {
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
