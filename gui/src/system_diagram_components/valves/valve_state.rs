//use crate::system_diagram_components::core::fluids::FluidType;

#[derive(Clone)]
pub enum ValveState {
    // Connected(Option<FluidType>),
    Open,
    Closed,
}

impl TryFrom<u8> for ValveState {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            core::u8::MAX => Ok(Self::Open),
            core::u8::MIN => Ok(Self::Closed),
            _ => Err(()),
        }
    }
}

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