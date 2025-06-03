use nalgebra::{Affine2, Point2};

use crate::{system_diagram_components::{core::display_value::DisplayValue, math::{parallelogram::CENTERED_UNIT_RECT, transform::Transform}, other::*, sensors_and_actuators::{motor, pressure_sensor, temperature_sensor}, storage::{storage_state::StorageState, *}, valves::{valve_state::ValveState, *}}, utils::theme::ThemeColors};

#[derive(Clone)]
pub struct Symbol {
    painter: Painter,
    local_transform: Affine2<f32>,
    attached_information: Vec<DisplayValue>,
}

#[derive(Clone)]
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

    pub fn new(painter: Painter, local_transform: Transform, attached_information: Vec<DisplayValue>) -> Self {
        Self { painter, local_transform: local_transform.to_affine2(), attached_information }
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

    pub fn attached_information(&self) -> &Vec<DisplayValue> {
        return &self.attached_information;
    }

}

#[derive(Clone)]
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