use crate::system_diagram_components::core::fluids::FluidType;

#[derive(Clone)]
pub enum ValveState {
    Connected(Option<FluidType>),
    Disconnected,
}