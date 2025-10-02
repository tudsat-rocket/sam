//use crate::system_diagram_components::core::fluids::FluidType;

// use telemetry::storage_data::storeable_value::StorableValue;

// #[derive(Clone)]
// pub enum ValveState {
//     // Connected(Option<FluidType>),
//     Open,
//     Closed,
// }

// impl StorableValue for ValveState {
//     fn to_bits(&self) -> u64 {
//         match self {
//             ValveState::Open => return 1,
//             ValveState::Closed => return 0,
//         }
//     }
//     fn from_bits(bits: u64) -> Self {
//         if bits == 1 {
//             return ValveState::Open;
//         } else if bits == 0 {
//             return ValveState::Closed;
//         } else {
//             panic!("Cannot convert {bits} to valve state, expected 0 or 1")
//         }
//     }

//     fn to_string(&self) -> String {
//         match self {
//             ValveState::Open => "Open".to_string(),
//             ValveState::Closed => "Closed".to_string(),
//         }
//     }

//     fn to_float(&self) -> f64 {
//         match self {
//             ValveState::Open => return 1f64,
//             ValveState::Closed => return 0f64,
//         }
//     }
// }

// impl TryFrom<u8> for ValveState {
//     type Error = ();

//     fn try_from(value: u8) -> Result<Self, Self::Error> {
//         match value {
//             core::u8::MAX => Ok(Self::Open),
//             core::u8::MIN => Ok(Self::Closed),
//             _ => Err(()),
//         }
//     }
// }

// pub struct ValveSettings {
//     default_state: DefaultValveState,
//     metric: Metric,
// }

// impl ValveSettings{

//     pub fn toggleState(&mut ){

//     }

// }

// pub enum DefaultValveState {
//     NominallyOpen,
//     NominallyClosed,
//     Bistable,
// }
