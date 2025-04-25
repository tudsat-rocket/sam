use egui::Color32;

use super::flow_component::FluidType;

pub const STROKE_WITH: f32 = 1.8;


//---------- FLUIDS ------------------------
pub const N2:       FluidType = FluidType{name: "N\u{2082}", color: Color32::LIGHT_GREEN};
pub const N2O:      FluidType = FluidType{name: "N\u{2082}O", color: Color32::LIGHT_BLUE};
pub const ETHANOL:  FluidType = FluidType{name: "Ethanol", color: Color32::RED};

//---------- TANKS ------------------------
pub const TANK_BULKHEAD_HEIGHT: f32 = 15.0;
pub const TANK_BULKHEAD_STEPS: usize = 100;

//---------- BOTTLE ------------------------
pub const BOTTLE_BULKHEAD_HEIGHT: f32 = 15.0;
pub const BOTTLE_BULKHEAD_STEPS: usize = 100;
pub const BOTTLE_CAP_TOTAL_HEIGHT: f32 = 30.0;
pub const BOTTLE_CAP_WIDTH: f32 = 0.4;  //RELATIVE
pub const BOTTLE_CAP_BULKHEAD_HEIGHT: f32 = 15.0;
pub const BOTTLE_CAP_BULKHEAD_STEPS: usize = 100;