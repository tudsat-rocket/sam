use crate::system_diagram_components::core::fluids::FluidType;

#[derive(Clone)]
pub struct StorageState{
    pub fluid: FluidType,
    pub level: f32
}

impl StorageState {

    pub fn new(fluid: FluidType, level: f32) -> Self {
        Self {fluid, level}
    }

}