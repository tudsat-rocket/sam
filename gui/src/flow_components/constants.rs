use egui::Color32;

use super::flow_component::FluidType;

pub const STROKE_WITH: f32 = 1.8;

pub const N2:       FluidType = FluidType{name: "N\u{2082}", color: Color32::LIGHT_GREEN};
pub const N2O:      FluidType = FluidType{name: "N\u{2082}O", color: Color32::LIGHT_BLUE};
pub const ETHANOL:  FluidType = FluidType{name: "Ethanol", color: Color32::RED};