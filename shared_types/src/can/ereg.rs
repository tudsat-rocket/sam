use super::structure::*;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct EregInfoMsg {
    pub pressure_n2: u16,
    pub pressure_ox: u8,
    pub regulator_valve: u8,
    pub vent_valve: u8,
    pub ereg_state: EregState,
}
impl TryFrom<&[u8]> for EregInfoMsg {
    type Error = PayloadParseError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() != 6 {
            return Err(PayloadParseError::EregInfoMsg);
        }
        Ok(Self {
            pressure_n2: u16::from_be_bytes([value[0], value[1]]),
            pressure_ox: value[2],
            regulator_valve: value[3],
            vent_valve: value[4],
            ereg_state: value[5].try_into()?,
        })
    }
}

#[repr(u8)]
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum EregState {
    /// Idle state after startup.
    #[default]
    Disarmed = 0,
    /// Filling oxidizer tank.
    Fill = 1, // name change to filling?
    /// Idle state.
    Idle = 2,
    /// Regulate oxidizer tank to specified pressure.
    Regulating = 3,
    /// Release pressure after landing.
    Dump = 4,
    /// Release pressure before ignition.
    Scrub = 5,
    /// Release pressure during ignition or burn.
    Abort = 6,
}
impl TryFrom<u8> for EregState {
    type Error = PayloadParseError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(match value {
            0 => EregState::Disarmed,
            1 => EregState::Fill,
            2 => EregState::Idle,
            3 => EregState::Regulating,
            4 => EregState::Dump,
            5 => EregState::Scrub,
            6 => EregState::Abort,
            _ => return Err(PayloadParseError::EregState),
        })
    }
}
impl TryFrom<&[u8]> for EregState {
    type Error = PayloadParseError;
    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from(*value.first().ok_or(PayloadParseError::EregState)?)
    }
}
