//! Types concerning the engine subsystem.
//! Hardware: EngineBoard, sensors, valves

use super::PayloadParseError;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct EngineInfoMsg {
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
    pub engine_state: EngineState,
}
impl TryFrom<&[u8]> for EngineInfoMsg {
    type Error = PayloadParseError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() > 7 {
            return Err(PayloadParseError::EngineInfoMsg);
        }
        Ok(Self {
            pressure_ox: value[0],
            pressure_combustion_chamber: u16::from_be_bytes([value[1], value[2]]),
            temp_ox: value[3],
            main_valve: value[4],
            fill_and_dump_valve: value[5],
            engine_state: value[6].try_into()?,
        })
    }
}

// NOTE: maybe this should be moved outside can module
/// States the engine board may be in according to technical report 2025.
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum EngineState {
    /// Idle state.
    #[default]
    Disarmed = 0,
    /// Filling oxidizer tank.
    Filling = 1,
    /// Finished filling oxidizer tank.
    Ready = 2,
    /// Ignition sequence, until liftoff.
    Ignition = 3,
    /// Motor burning.
    Burn = 4,
    /// Release oxidizer after landing.
    Dump = 5,
    /// Release oxidizer before ignition.
    Scrub = 6,
    /// Abort ignition or burning engine, release oxidizer.
    Abort = 7,
}
impl TryFrom<u8> for EngineState {
    type Error = PayloadParseError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(match value {
            0 => EngineState::Disarmed,
            1 => EngineState::Filling,
            2 => EngineState::Ready,
            3 => EngineState::Ignition,
            4 => EngineState::Burn,
            5 => EngineState::Dump,
            6 => EngineState::Scrub,
            7 => EngineState::Abort,
            _ => return Err(PayloadParseError::EngineState),
        })
    }
}
impl TryFrom<&[u8]> for EngineState {
    type Error = PayloadParseError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        let state = value.first().ok_or(PayloadParseError::EngineState)?;
        EngineState::try_from(*state)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct LaunchCode(pub u64);
impl TryFrom<&[u8]> for LaunchCode {
    type Error = PayloadParseError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != 8 {
            return Err(PayloadParseError::LaunchCode);
        }
        let bytes: [u8; 8] = value[0..8].try_into().unwrap();
        Ok(LaunchCode(u64::from_be_bytes(bytes)))
    }
}
