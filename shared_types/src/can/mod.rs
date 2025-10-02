//! Data structures for messages sent via the rocket's internal CAN bus.
//!
//! Each message has a corresponding set of message IDs, which allow
//! identifying the message and also encodes priority, with lower IDs
//! being higher priority. General scheme:
//!
//! 0x000 - 0x0ff      Flight computer outputs (e.g. valve commands)
//!     0x0i0 - 0x0if  IO board i (0..f)
//!
//! 0x100 - 0x1ff      Flight computer inputs (e.g. collected sensor data)
//!     0x1i0 - 0x1af  IO board i (0..a)
//!     0x1b0 - 0x1ef  Fins 1-4
//!     0x1f0 - 0x1ff  Battery board telemetry
//!
//! 0x200 - 0x2ff      Internal telemetry (e.g. data shared with payloads)
//!
//! The bit structure is represented by types in the structure module.

pub mod engine;
pub mod ereg;
pub mod structure;

use engine::{EngineInfoMsg, EngineState, LaunchCode};
use ereg::{EregInfoMsg, EregState};
use structure::*;

use crate::common::{FlightMode, ProcedureStep};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crc::{Crc, CRC_16_IBM_SDLC};
use heapless::Vec;

const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

/// Shall represent all legal can messages in our system.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum CanMessage {
    /// Command sent from FC to a subsystem.
    Cmd(SubsystemCommand),
    /// Message sent from Subsystem containing information about themselves.
    Info(SubsystemInfo),
    // Used for example for FancyFin state display.
    /// Generic broadcast message (for now) sent from FC.
    Telem(TelemetryBroadcast),
}
impl TryFrom<Can2aFrame> for CanMessage {
    type Error = CanMessageParseError;
    fn try_from(value: Can2aFrame) -> Result<Self, Self::Error> {
        let message_kind = value.id.message_kind.clone();

        Ok(match message_kind {
            MessageKind::SubsystemCommand => CanMessage::Cmd(SubsystemCommand::try_from(value)?),
            MessageKind::SubsystemInfo => CanMessage::Info(SubsystemInfo::try_from(value)?),
            MessageKind::TelemetryBroadcast => CanMessage::Telem(TelemetryBroadcast::try_from(value)?),
        })
    }
}
impl From<CanMessage> for Can2aFrame {
    fn from(value: CanMessage) -> Self {
        match value {
            CanMessage::Cmd(msg) => msg.into(),
            CanMessage::Info(msg) => msg.into(),
            CanMessage::Telem(msg) => msg.into(),
        }
    }
}

// --- SubsystemCommand

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum SubsystemCommand {
    // Only allowed state transitions will be executed by the engine.
    // In general only transitioning forward is allowed.
    // NOTE: Maybe add a 'reset after scrub' command to override scrub state.
    SetEngineState(EngineState),
    IgniteEngine(LaunchCode),
    SetEregState(EregState),
}
impl TryFrom<Can2aFrame> for SubsystemCommand {
    type Error = CanMessageParseError;
    fn try_from(value: Can2aFrame) -> Result<Self, Self::Error> {
        let id = value.id;
        if id.message_kind != MessageKind::SubsystemCommand {
            return Err(CanMessageParseError::Id(IdParseError::SubsystemId));
        }
        use SubsystemId as Sub;
        Ok(match id.subsystem_id {
            Sub::Engine => match id.specific_message_id {
                0 => SubsystemCommand::SetEngineState(EngineState::try_from(value.payload.as_slice())?),
                1 => SubsystemCommand::IgniteEngine(value.payload.as_slice().try_into()?),
                _ => return Err(IdParseError::SpecificMessageId.into()),
            },
            Sub::Ereg => match id.specific_message_id {
                0 => SubsystemCommand::SetEregState(EregState::try_from(value.payload.as_slice())?),
                _ => return Err(IdParseError::SpecificMessageId.into()),
            },
            Sub::PayloadIo => return Err(IdParseError::SpecificMessageId.into()),
            Sub::CameraIo => return Err(IdParseError::SpecificMessageId.into()),
            Sub::Empty => return Err(IdParseError::SpecificMessageId.into()),
        })
    }
}

impl From<SubsystemCommand> for Can2aFrame {
    // type Error = CanMessageParseError;
    fn from(value: SubsystemCommand) -> Self {
        use SubsystemCommand as Cmd;
        let message_kind = MessageKind::SubsystemCommand;
        match value {
            Cmd::SetEngineState(state) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id: SubsystemId::Engine,
                    specific_message_id: 0,
                },
                payload: Vec::from_slice(&[state as u8]).unwrap(),
            },
            // NOTE: big endian was chosen arbitrarily
            Cmd::IgniteEngine(code) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id: SubsystemId::Engine,
                    specific_message_id: 1,
                },
                payload: Vec::from_slice(&code.0.to_be_bytes()).unwrap(),
            },
            Cmd::SetEregState(state) => Can2aFrame {
                // id: build_id(message_kind, SubsystemId::Ereg, 0),
                id: CanFrameId {
                    message_kind,
                    subsystem_id: SubsystemId::Ereg,
                    specific_message_id: 0,
                },
                payload: Vec::from_slice(&[state as u8]).unwrap(),
            },
        }
    }
}

// --- SubsystemInfo

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum SubsystemInfo {
    Engine(EngineInfoMsg),
    Ereg(EregInfoMsg),
}

impl TryFrom<Can2aFrame> for SubsystemInfo {
    type Error = CanMessageParseError;
    fn try_from(value: Can2aFrame) -> Result<Self, Self::Error> {
        let id: CanFrameId = value.id;
        if id.message_kind != MessageKind::SubsystemInfo {
            return Err(CanMessageParseError::Id(IdParseError::MessageKind));
        }
        use SubsystemId as Sub;
        Ok(match id.subsystem_id {
            Sub::Engine => SubsystemInfo::Engine(match id.specific_message_id {
                0 => EngineInfoMsg::try_from(value.payload.as_slice())?,
                _ => return Err(IdParseError::SpecificMessageId)?,
            }),
            Sub::Ereg => SubsystemInfo::Ereg(match id.specific_message_id {
                0 => EregInfoMsg::try_from(value.payload.as_slice())?,
                _ => return Err(IdParseError::SpecificMessageId)?,
            }),
            Sub::CameraIo => return Err(CanMessageParseError::Id(IdParseError::SubsystemId)),
            Sub::PayloadIo => return Err(CanMessageParseError::Id(IdParseError::SubsystemId)),
            Sub::Empty => return Err(IdParseError::SubsystemId.into()),
        })
    }
}

// --- TelemetryBroadcast

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum TelemetryBroadcast {
    FlightMode(FlightMode),
    ProcedureStep(ProcedureStep),
}
impl TryFrom<Can2aFrame> for TelemetryBroadcast {
    type Error = CanMessageParseError;
    fn try_from(value: Can2aFrame) -> Result<Self, Self::Error> {
        let id = value.id;
        if id.message_kind != MessageKind::TelemetryBroadcast {
            return Err(IdParseError::MessageKind.into());
        }
        if id.subsystem_id != SubsystemId::Empty {
            return Err(IdParseError::SubsystemId.into());
        }
        Ok(match id.specific_message_id {
            0 => {
                let fm = FlightMode::try_from(*value.payload.first().ok_or(PayloadParseError::FlightMode)?)
                    .map_err(|_e| PayloadParseError::FlightMode)?;
                TelemetryBroadcast::FlightMode(fm)
            }
            1 => {
                let step = ProcedureStep::try_from(*value.payload.first().ok_or(PayloadParseError::ProcedureStep)?)
                    .map_err(|_e| PayloadParseError::ProcedureStep)?;
                TelemetryBroadcast::ProcedureStep(step)
            }
            _ => return Err(IdParseError::SpecificMessageId.into()),
        })
    }
}
impl From<TelemetryBroadcast> for Can2aFrame {
    fn from(value: TelemetryBroadcast) -> Self {
        let message_kind = MessageKind::TelemetryBroadcast;
        let subsystem_id = SubsystemId::Empty;
        match value {
            TelemetryBroadcast::FlightMode(fm) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id,
                    specific_message_id: 0,
                },
                payload: { Vec::from_slice(&[fm as u8]).unwrap() },
            },
            TelemetryBroadcast::ProcedureStep(step) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id,
                    specific_message_id: 1,
                },
                payload: Vec::from_slice(&[step as u8]).unwrap(),
            },
        }
    }
}

impl From<SubsystemInfo> for Can2aFrame {
    fn from(value: SubsystemInfo) -> Self {
        use SubsystemInfo as Sub;
        let message_kind = MessageKind::SubsystemInfo;
        match value {
            Sub::Engine(msg) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id: SubsystemId::Engine,
                    specific_message_id: 0,
                },
                payload: Vec::from_slice(&[
                    msg.pressure_ox,
                    msg.pressure_combustion_chamber.to_be_bytes()[0],
                    msg.pressure_combustion_chamber.to_be_bytes()[1],
                    msg.temp_ox,
                    msg.main_valve,
                    msg.fill_and_dump_valve,
                    msg.engine_state as u8,
                ])
                .unwrap(),
            },
            Sub::Ereg(msg) => Can2aFrame {
                id: CanFrameId {
                    message_kind,
                    subsystem_id: SubsystemId::Ereg,
                    specific_message_id: 0,
                },
                payload: Vec::from_slice(&[
                    msg.pressure_n2.to_be_bytes()[0],
                    msg.pressure_n2.to_be_bytes()[1],
                    msg.pressure_ox,
                    msg.regulator_valve,
                    msg.vent_valve,
                    msg.ereg_state as u8,
                ])
                .unwrap(),
            },
        }
    }
}

// --- NOTES ---

// pub enum SubsystemId {
//     Engine = 1,
//     Ereg = 2,
//     Recovery = 3,
//     Payload = 8,
// }
//
// pub enum CanMsg {
//     SubsystemCommand(SubsystemId, u8),
//     SubsystemInput(SubsystemId, u8),
//     FinBoardInput(),
// }

// NOTE: maybe use telemetry types here?

// Overview of all collected values:
// EngineBoard:
// - PressureOxidizerTankLower (8)
// - TemperatureOxidizerTank (8)
// - PressureCombustionChamber (16)
// - ValveStateMain (8)
// - ValveStateFillAndDump (8)
//
// - EngineState (8)
//
// Ereg:
// - PressureNitrogenTank (16)
// - PressureOxidizerTankUpper (8)
// - ValveStatePressureRegulator (8)
// - ValveStateVent (8)
// -
// - EregState (8)
//
// PowerBoard
// - PowerMessage ?
//
// FancyFin
// - FancyFinMessage ?
//
// PayloadIo
// - Heartbeat?
//
// CameraIo

// Overview off all current commands sent from FC to subsystems:
//
// EngineBoard:
// - SetEngineState (except ignition)
// - IgniteEngine(64bit code)
//
// Ereg:
// - SetEregState
//
//

// --------- Old types, kept here for compatibility, should be removed in the future

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IoBoardRole {
    Acs = 1,
    Recovery = 2,
    Payload = 8,
}

#[derive(Clone, Copy, Debug)]
pub enum CanBusMessageId {
    IoBoardCommand(IoBoardRole, u8),
    IoBoardInput(IoBoardRole, u8),
    FinBoardInput(u8, u8),
    BatteryBoardInput(u8),
    TelemetryBroadcast(u8),
}

impl Into<u16> for CanBusMessageId {
    /// Returns an 11 bit CAN 2.0A identifier. First 5 bits are always 0.
    /// We try to give higher priority messages a lower overall 11 bit id.
    /// x: ignored
    /// t: message type see [`CanBusMessageId`]
    /// s: SubsystemId
    /// m: specific message identification
    /// xxxx.xttt.ssss.mmmm
    fn into(self) -> u16 {
        match self {
            // 0x000 | 0b1111_0000 | 0b1111
            Self::IoBoardCommand(role, id) => 0x000 + ((role as u16 & 0x0f) << 4) + (id as u16 & 0x0f),
            Self::IoBoardInput(role, id) => 0x100 + ((role as u16 & 0x0f) << 4) + (id as u16 & 0x0f),
            Self::FinBoardInput(fin, id) => 0x100 + (((fin + 0x0b) as u16 & 0x0f) << 4) + (id as u16 & 0x0f),
            // Self::BatteryBoardInput(id) => 0x1f0 + (id as u16 & 0x0f),
            Self::BatteryBoardInput(id) => 0x100 | 0x0f0 | (id as u16 & 0x0f),
            // Self::TelemetryBroadcast(id) => 0x200 + id as u16,
            Self::TelemetryBroadcast(id) => 0x200 | 0x0 | id as u16,
        }
    }
}

impl TryFrom<u16> for CanBusMessageId {
    type Error = u16;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0x000..=0x0ff => {
                let role_id: u8 = ((value & 0x0f0) >> 4) as u8;
                let Ok(role) = IoBoardRole::try_from(role_id) else {
                    return Err(value);
                };
                Ok(Self::IoBoardCommand(role, (value & 0x00f) as u8))
            }
            0x100..=0x1af => {
                let role_id: u8 = ((value & 0x0f0) >> 4) as u8;
                let Ok(role) = IoBoardRole::try_from(role_id) else {
                    return Err(value);
                };
                Ok(Self::IoBoardInput(role, (value & 0x00f) as u8))
            }
            0x1b0..=0x1ef => {
                let fin_id: u8 = ((value & 0x0f0) >> 4) as u8 - 0x0b;
                Ok(Self::FinBoardInput(fin_id, (value & 0x00f) as u8))
            }
            0x1f0..=0x1ff => Ok(Self::BatteryBoardInput((value & 0x00f) as u8)),
            0x200..=0x2ff => Ok(Self::TelemetryBroadcast((value & 0x0ff) as u8)),
            unknown => Err(unknown),
        }
    }
}

pub trait CanBusMessage: Sized {
    fn serialize(self) -> [u8; 6];
    fn deserialize(data: &[u8]) -> Option<Self>;

    fn serialize_with_crc(self) -> [u8; 8] {
        let mut frame = [0x00; 8];

        let data = self.serialize();
        let checksum = CRC.checksum(&data);

        frame[0..6].copy_from_slice(&data);
        frame[6..8].copy_from_slice(&checksum.to_le_bytes());

        frame
    }

    fn to_frame(self, id: CanBusMessageId) -> (u16, [u8; 8]) {
        (id.into(), self.serialize_with_crc())
    }

    fn parse(data: [u8; 8]) -> Result<Option<Self>, ()> {
        let crc_test = u16::from_le_bytes([data[6], data[7]]);
        let crc_ref = CRC.checksum(&data[..6]);
        if crc_test != crc_ref {
            return Err(());
        }

        Ok(Self::deserialize(&data[..6]))
    }
}

impl TryFrom<u8> for IoBoardRole {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(IoBoardRole::Acs),
            2 => Ok(IoBoardRole::Recovery),
            8 => Ok(IoBoardRole::Payload),
            _ => Err(value),
        }
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IoBoardOutputMessage {
    pub outputs: [bool; 8],
    // TODO: max high duration, failsafe value?
}

impl CanBusMessage for IoBoardOutputMessage {
    fn serialize(self) -> [u8; 6] {
        let byte = self
            .outputs
            .iter()
            .enumerate()
            .map(|(i, output)| (*output as u8) << (7 - i))
            .reduce(|a, b| a | b)
            .unwrap();

        [byte, byte, byte, byte, 0x00, 0x00]
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        let byte = data[0]; // TODO

        Some(Self {
            outputs: [
                byte & 0b1000_0000 > 0,
                byte & 0b0100_0000 > 0,
                byte & 0b0010_0000 > 0,
                byte & 0b0001_0000 > 0,
                byte & 0b0000_1000 > 0,
                byte & 0b0000_0100 > 0,
                byte & 0b0000_0010 > 0,
                byte & 0b0000_0001 > 0,
            ],
        })
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IoBoardSensorMessage {
    // Outputs are 4 optional 10-bit raw ADC results.
    // The additional bool is the alert flag, set if value is out of range
    pub i2c_sensors: [Option<(u16, bool)>; 4],
}

impl CanBusMessage for IoBoardSensorMessage {
    fn serialize(self) -> [u8; 6] {
        let mut sensors_packed: u64 = 0;
        for i in 0..4 {
            let sensor_value = self.i2c_sensors[i].map(|(val, _)| val).unwrap_or(0x3ff);
            let some_flag = self.i2c_sensors[i].is_some() as u16;
            let alert_flag = self.i2c_sensors[i].map(|(_, alert)| alert).unwrap_or(false) as u16;

            // each sensor gets 12 bits sacc_cccc_cccc (s = is_some, a = alert, c = value)
            let sensor = (some_flag << 11) | (alert_flag << 10) | sensor_value;
            sensors_packed |= (sensor as u64) << (52 - (i * 12));
        }

        let mut msg = [0x00; 6];
        msg[0..6].copy_from_slice(&sensors_packed.to_be_bytes()[..6]);
        msg
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        let sensors_packed = u64::from_be_bytes([data[0], data[1], data[2], data[3], data[4], data[5], 0, 0]);

        let mut i2c_sensors = [None; 4];
        for i in 0..i2c_sensors.len() {
            let sensor = (sensors_packed >> (52 - i * 12)) & 0xfff;
            let is_some = (sensor >> 11) > 0;
            let alert = ((sensor >> 10) & 0b1) > 0;
            let value = (sensor & 0x3ff) as u16;
            i2c_sensors[i] = is_some.then_some((value, alert));
        }

        Some(Self { i2c_sensors })
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IoBoardPowerMessage {
    /// Voltage applied at the outputs in mV. For IO boards outputting 5V,
    /// battery voltage is measured instead.
    pub output_voltage: u16,
    /// Current drawn by the outputs in mA.
    pub output_current: i16,
    /// Measured resistance of the NTC resistor in Ohm, B= TODO
    pub thermistor_resistance: u16,
}

impl CanBusMessage for IoBoardPowerMessage {
    fn serialize(self) -> [u8; 6] {
        let mut msg = [0x00; 6];
        msg[0..2].copy_from_slice(&self.output_voltage.to_be_bytes());
        msg[2..4].copy_from_slice(&self.output_current.to_be_bytes());
        msg[4..6].copy_from_slice(&self.thermistor_resistance.to_be_bytes());
        msg
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        let output_voltage = u16::from_be_bytes([data[0], data[1]]);
        let output_current = i16::from_be_bytes([data[2], data[3]]);
        let thermistor_resistance = u16::from_be_bytes([data[4], data[5]]);

        Some(Self {
            output_voltage,
            output_current,
            thermistor_resistance,
        })
    }
}

// Format defined in Payload Specification, do not change.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TelemetryToPayloadMessage {
    pub time: u32,
    pub mode: crate::FlightMode,
    pub altitude: u16,
}

impl CanBusMessage for TelemetryToPayloadMessage {
    fn serialize(self) -> [u8; 6] {
        let mut msg = [0x00; 6];
        msg[0..3].copy_from_slice(&self.time.to_le_bytes()[..3]);
        msg[3] = self.mode as u8;
        msg[4..6].copy_from_slice(&self.altitude.to_le_bytes());
        msg
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        let time = u32::from_le_bytes([data[0], data[1], data[2], 0x00]);
        let Ok(mode) = data[3].try_into() else { return None };
        let altitude = u16::from_le_bytes([data[4], data[5]]);

        Some(Self { time, mode, altitude })
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BatteryTelemetryMessage {
    pub voltage_battery: u16, // mV
    pub voltage_charge: u16,  // mV
    pub current: i32,         // mA
    pub stat0: bool,          // TODO: higher type
    pub stat1: bool,
}

impl CanBusMessage for BatteryTelemetryMessage {
    fn serialize(self) -> [u8; 6] {
        let mut msg = [0x00; 6];
        let vb = (self.voltage_battery << 1) | (self.stat0 as u16);
        let vc = (self.voltage_charge << 1) | (self.stat1 as u16);
        let current = (self.current + 2000) as u16;
        msg[0..2].copy_from_slice(&vb.to_be_bytes());
        msg[2..4].copy_from_slice(&vc.to_be_bytes());
        msg[4..6].copy_from_slice(&current.to_be_bytes());
        msg
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        let vb = u16::from_be_bytes([data[0], data[1]]);
        let vc = u16::from_be_bytes([data[2], data[3]]);
        let current = (u16::from_be_bytes([data[4], data[5]]) as i32) - 2000;
        Some(Self {
            voltage_battery: vb >> 1,
            voltage_charge: vc >> 1,
            current,
            stat0: (vb & 0b1) > 0,
            stat1: (vc & 0b1) > 0,
        })
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FinBoardDataMessage {
    pub data: [u8; 6],
}

impl CanBusMessage for FinBoardDataMessage {
    fn serialize(self) -> [u8; 6] {
        self.data
    }

    fn deserialize(data: &[u8]) -> Option<Self> {
        Some(Self {
            data: data.try_into().unwrap(),
        })
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FcReceivedCanBusMessage {
    IoBoardSensor(IoBoardRole, u8, IoBoardSensorMessage),
    IoBoardPower(IoBoardRole, IoBoardPowerMessage),
    FinBoardData(u8, u8, FinBoardDataMessage),
    // TODO: fin input
    BatteryTelemetry(u8, BatteryTelemetryMessage),
}

pub enum FcTransmittedCanBusMessage {
    IoBoardCommand(IoBoardRole, u8, IoBoardOutputMessage),
}
