pub const STROKE_WITH: f32 = 1.8;

//-----------------------------------------
//---------- TANKS ------------------------
//-----------------------------------------

pub const TANK_BULKHEAD_HEIGHT: f32 = 15.0;
pub const TANK_BULKHEAD_STEPS: usize = 100;

//------------------------------------------
//---------- BOTTLE ------------------------
//------------------------------------------

pub const BOTTLE_BULKHEAD_HEIGHT: f32 = 15.0;
pub const BOTTLE_BULKHEAD_STEPS: usize = 100;
pub const BOTTLE_CAP_TOTAL_HEIGHT: f32 = 30.0;
pub const BOTTLE_CAP_WIDTH: f32 = 0.4; //RELATIVE
pub const BOTTLE_CAP_BULKHEAD_HEIGHT: f32 = 15.0;
pub const BOTTLE_CAP_BULKHEAD_STEPS: usize = 100;

//-------------------------------------------
//---------- LINE 2D ------------------------
//-------------------------------------------

pub const LINE_2D_WIDTH: f32 = 0.05;

//-------------------------------------------
//---------- BALL VALVE ---------------------
//-------------------------------------------

pub const BALL_VALVE_SYMBOL_DISTANCE: f32 = 0.08;
pub const BALL_VALVE_SYMBOL_RADIUS: f32 = 0.03;

//-------------------------------------------
//---------- TANK VALVE ---------------------
//-------------------------------------------

///Length of the valve handle relative to the height of the valve
pub const TANK_VALVE_HANDLE_LENGTH: f32 = 0.8;
///Width of the valve handle relative to the width of the valve
pub const TANK_VALVE_HANDLE_WIDTH: f32 = 0.5;

//-------------------------------------------
//---------- FLEX TUBE ----------------------
//-------------------------------------------

///Determines the resolution of the tube
pub const FLEX_TUBE_STEP_NUM: usize = 100;
///Maximum height of the tube relative to the component height
pub const FLEX_TUBE_HEIGHT: f32 = 0.8;
///Number of tick marks on the tube
pub const FLEX_TUBE_TICK_NUM: usize = 7;
///Length of tick marks relative to the component height
pub const FLEX_TUBE_TICK_LENGTH: f32 = 0.15;

//-------------------------------------------
//---------- CHECK VALVE --------------------
//-------------------------------------------

///Determines the padding
pub const CHECK_VALVE_PADDING: f32 = 0.1;
///Determines the number of spikes. Should be an even number
pub const CHECK_VALVE_SPIKE_NUM: usize = 6;
