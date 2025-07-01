use nalgebra::{Affine2, Point2};

use crate::{system_diagram_components::{math::{parallelogram::CENTERED_UNIT_RECT, transform::Transform}, other::*, sensors_and_actuators::{manometer, motor, pressure_sensor, temperature_sensor}, storage::{storage_state::StorageState, *}, valves::{valve_state::ValveState, *}}, utils::theme::ThemeColors, widgets::system_diagram::Component};

#[derive(Clone)]
pub struct Symbol {
    painter: Painter,
    local_transform: Affine2<f32>,
    component: Option<Component>
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

    pub fn new(painter: Painter, local_transform: Transform, component: Option<Component>) -> Self {
        Self { painter, local_transform: local_transform.to_affine2(), component }
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
            Painter::Atmosphere => atmosphere::paint(transform, painter, ctx),

            //--------------- Sensors and Actuators ----------------
            Painter::Motor => motor::paint(transform, painter, ctx),
            Painter::PressureSensor => pressure_sensor::paint(transform, painter, ctx),
            Painter::TemperatureSensor => temperature_sensor::paint(transform, painter, ctx),
            Painter::Manometer => manometer::paint(transform, painter, ctx),
        
            //----------------------- Other ------------------------
            Painter::Missing => missing::paint(transform, painter),
            Painter::FlexTube => flex_tube::paint(transform, painter, theme),
            Painter::Thruster => thruster::paint(transform, painter, theme),
        }
        return CENTERED_UNIT_RECT.transform(transform).axis_aligned_bounding_box();
    }

    pub fn component(&self) -> &Option<Component> {
        return &self.component;
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
    Atmosphere,
    //--------------- Sensors and Actuators ----------------
    Motor,
    PressureSensor,
    TemperatureSensor,
    Manometer,
    //----------------------- Other ------------------------
    Missing,
    FlexTube,
    Thruster,
}