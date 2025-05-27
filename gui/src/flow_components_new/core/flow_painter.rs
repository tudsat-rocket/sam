use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::{math::{parallelogram::CENTERED_UNIT_RECT, transform::Transform}, other::*, sensors_and_actuators::{motor, pressure_sensor, temperature_sensor}, storage::{storage_state::StorageState, *}, valves::{valve_state::ValveState, *}}, utils::theme::ThemeColors};

pub struct Symbol {
    painter: Painter,
    local_transform: Affine2<f32>,
}

pub struct Line1D {
    points: Vec<Point2<f32>>,
}

impl Line1D {
    
    pub fn new(points: Vec<Point2<f32>>) -> Self {
        Self { points }
    }

    pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, ctx: &egui::Context) {
        let theme = &ThemeColors::new(ctx);
        line1d::paint(&self.points, global_transform, painter, theme);
    }
}

impl Symbol {

    pub fn new(painter: Painter, local_transform: Transform) -> Self {
        Self { painter, local_transform: local_transform.to_affine2() }
    }

    pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, ctx: &egui::Context) -> egui::Rect{
        let transform = &(global_transform * self.local_transform);
        let theme = &ThemeColors::new(ctx);
        match &self.painter {
            //----------------------- Valves -----------------------
            Painter::GenericValve(state) => generic_valve::paint(transform, state, painter, theme),
            Painter::ManualValve(state) => manual_valve::paint(transform, state, painter, theme),
            Painter::MotorizedValve(state) => motorized_valve::paint(transform, state, painter, ctx),
            Painter::BurstDisc(state) => burst_disc::paint(transform, state, painter, theme),
            Painter::QuickDisconnect(state) => quick_disconnect::paint(transform, state, painter, theme),
            Painter::PressureReliefValve(state) => pressure_relief_valve::paint(transform, state, painter, theme),
            
            //----------------------- Storage ----------------------
            Painter::Tank(state) => tank::paint(transform, state, painter, theme),
            Painter::Bottle(state) => bottle::paint(transform, state, painter, theme),

            //--------------- Sensors and Actuators ----------------
            Painter::Motor => motor::paint(transform, painter, ctx),
            Painter::PressureSensor => pressure_sensor::paint(transform, painter, ctx),
            Painter::TemperatureSensor => temperature_sensor::paint(transform, painter, ctx),
        
            //----------------------- Other ------------------------
            Painter::Missing => missing::paint(transform, painter),
            Painter::FlexTube => flex_tube::paint(transform, painter, theme),
        }
        return CENTERED_UNIT_RECT.transform(transform).axis_aligned_bounding_box();
    }

}

pub enum Painter {
    //----------------------- Valves -----------------------
    GenericValve(ValveState),
    ManualValve(ValveState),
    MotorizedValve(ValveState),
    BurstDisc(ValveState),
    QuickDisconnect(ValveState),
    PressureReliefValve(ValveState),
    //----------------------- Storage ----------------------
    Tank(StorageState),
    Bottle(StorageState),
    //--------------- Sensors and Actuators ----------------
    Motor,
    PressureSensor,
    TemperatureSensor,
    //----------------------- Other ------------------------
    Missing,
    FlexTube,
}

// impl Painter {
//     pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, ctx: &egui::Context) {
//         let theme = &ThemeColors::new(ctx);
//         match self {
//             //----------------------- Valves -----------------------
//             Painter::GenericValve(local_transform, state) => generic_valve::paint(&(global_transform * local_transform.to_affine2()), state, painter, theme),
//             Painter::ManualValve(local_transform, state) => manual_valve::paint(&(global_transform * local_transform.to_affine2()), state, painter, theme),
//             Painter::MotorizedValve(local_transform, state) => motorized_valve::paint(&(global_transform * local_transform.to_affine2()), state, painter, ctx),
//             Painter::BurstDisc(local_transform, state) => burst_disc::paint(&(global_transform * local_transform.to_affine2()), state, painter, theme),
//             Painter::QuickDisconnect(local_transform, state) => quick_disconnect::paint(&(global_transform * local_transform.to_affine2()), state, painter, theme),
//             Painter::PressureReliefValve(local_transform, state) => pressure_relief_valve::paint(&(global_transform * local_transform.to_affine2()), state, painter, theme),
            
//             //----------------------- Storage ----------------------
//             Painter::Tank(local_transform, state) => tank::paint(local_transform, global_transform, state, painter, theme),
//             Painter::Bottle(local_transform, state) => bottle::paint(local_transform, global_transform, state, painter, theme),

//             //--------------- Sensors and Actuators ----------------
//             Painter::Motor(local_transform) => motor::paint(&(global_transform * local_transform.to_affine2()), painter, ctx),
//             Painter::PressureSensor(local_transform) => pressure_sensor::paint(&(global_transform * local_transform.to_affine2()), painter, ctx),
//             Painter::TemperatureSensor(local_transform) => temperature_sensor::paint(&(global_transform * local_transform.to_affine2()), painter, ctx),
        
//             //----------------------- Other ------------------------
//             Painter::Missing(local_transform) => missing::paint(&(global_transform * local_transform.to_affine2()), painter),
//             Painter::FlexTube(local_transform) => flex_tube::paint(&(global_transform * local_transform.to_affine2()), painter, theme),
//             Painter::Line1D(points) => line1d::paint(points, global_transform, painter, theme),
//         }
//     }
// }