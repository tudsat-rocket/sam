/// Froms a CAN2.0A Frame containing an 11 bit identifier and up to 8 bytes of payload.
pub struct Can2aFrame {
    pub id: CanFrameId,
    pub payload: heapless::Vec<u8, 8>,
}

/// Abstract representation of an 11 bit CAN identifier.
/// This is to loosely encode priority and make parsing by hand easier.
pub struct CanFrameId {
    pub message_kind: MessageKind,
    pub subsystem_id: SubsystemId,
    pub specific_message_id: u8,
}
/// Ignores first 5 bits because only 11 bits are used.
impl TryFrom<u16> for CanFrameId {
    type Error = IdParseError;
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        let message_kind = ((value >> 8) & 0b111) as u8;
        let subsystem_id = ((value >> 4) & 0xf) as u8;
        let specific_message_id = (value & 0xf) as u8;

        Ok(CanFrameId {
            message_kind: message_kind.try_into()?,
            subsystem_id: subsystem_id.try_into()?,
            specific_message_id,
        })
    }
}
impl From<CanFrameId> for u16 {
    fn from(value: CanFrameId) -> Self {
        let message_kind = (value.message_kind as u8 & 0b111) as u16;
        let subsystem_id = (value.subsystem_id as u8 & 0xf) as u16;
        let specific_message_id = (value.specific_message_id & 0xf) as u16;

        (message_kind << 8) | (subsystem_id << 4) | specific_message_id
    }
}

/// This is to loosely encode priority and make parsing by hand easier.
#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum SubsystemId {
    Empty = 0,
    Engine = 1,
    Ereg = 2,
    CameraIo = 7,
    PayloadIo = 8,
}
impl TryFrom<u8> for SubsystemId {
    type Error = IdParseError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use SubsystemId as Id;
        Ok(match value {
            1 => Id::Engine,
            2 => Id::Ereg,
            7 => Id::CameraIo,
            8 => Id::PayloadIo,
            _ => return Err(IdParseError::SubsystemId),
        })
    }
}

/// This is to loosely encode priority and make parsing by hand easier.
#[repr(u8)]
#[derive(PartialEq, Eq, Clone)]
pub enum MessageKind {
    /// Command sent from the FC to other boards.
    SubsystemCommand = 0,
    /// Information received by the FC from other boards.
    SubsystemInfo = 1,
    /// Information broadcasted by the FC to be interpreted by multiple boards.
    TelemetryBroadcast = 2,
}
impl TryFrom<u8> for MessageKind {
    type Error = IdParseError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Ok(match value {
            0 => MessageKind::SubsystemCommand,
            1 => MessageKind::SubsystemInfo,
            2 => MessageKind::TelemetryBroadcast,
            _ => return Err(IdParseError::MessageKind),
        })
    }
}

pub enum CanMessageParseError {
    Id(IdParseError),
    Payload(PayloadParseError),
}
impl From<IdParseError> for CanMessageParseError {
    fn from(value: IdParseError) -> Self {
        Self::Id(value)
    }
}
impl From<PayloadParseError> for CanMessageParseError {
    fn from(value: PayloadParseError) -> Self {
        Self::Payload(value)
    }
}
pub enum IdParseError {
    MessageKind,
    SubsystemId,
    SpecificMessageId,
}
pub enum PayloadParseError {
    EngineInfoMsg,
    EngineState,
    LaunchCode,
    EregInfoMsg,
    EregState,
    FlightMode,
}

/// Helper to build our 11 bit Can2.0A identifier.
pub fn build_id(msg_kind: MessageKind, subsystem_id: SubsystemId, specific_message_id: u8) -> u16 {
    let msg_kind: u8 = msg_kind as u8;
    let subsystem_id: u8 = subsystem_id as u8;

    let msg_kind = msg_kind & 0b111;
    let subsystem_id = subsystem_id & 0xf;
    let specific_message_id = specific_message_id & 0xf;

    ((msg_kind as u16) << 8) | ((subsystem_id as u16) << 4) | (specific_message_id as u16)
}
