use crate::flow_components::flow_component::FluidType;

pub enum ValveState {
    Connected(Option<FluidType>),
    Disconnected,
}