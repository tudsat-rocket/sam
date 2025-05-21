use nalgebra::Affine2;

use crate::{flow_components_new::{math::transform::Transform, other::*, storage::{storage_state::StorageState, *}, valves::{valve_state::ValveState, *}}, utils::theme::ThemeColors};

pub enum FlowPainter {
    //----------------------- Valves -----------------------
    GenericValve(Transform, ValveState),
    TankValve(Transform, ValveState),
    BurstDisc(Transform, ValveState),
    QuickDisconnect(Transform, ValveState),
    //----------------------- Storage ----------------------
    Tank(Transform, StorageState),
    Bottle(Transform, StorageState),
    //----------------------- Other ------------------------
    Missing(Transform),
    FlexTube(Transform),
}

impl FlowPainter {
    pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: ThemeColors) {
        match self {
            //----------------------- Valves -----------------------
            FlowPainter::GenericValve(local_transform, state) => generic_valve::paint(local_transform, global_transform, state, painter, theme),
            FlowPainter::TankValve(local_transform, state) => tank_valve::paint(local_transform, global_transform, state, painter, theme),
            FlowPainter::BurstDisc(local_transform, state) => burst_disc::paint(local_transform, global_transform, state, painter, theme),
            FlowPainter::QuickDisconnect(local_transform, state) => quick_disconnect::paint(local_transform, global_transform, state, painter, &theme),
            //----------------------- Storage ----------------------
            FlowPainter::Tank(local_transform, state) => tank::paint(local_transform, global_transform, state, painter, theme),
            FlowPainter::Bottle(local_transform, state) => bottle::paint(local_transform, global_transform, state, painter, theme),
        
            //----------------------- Other ------------------------
            FlowPainter::Missing(local_transform) => missing::paint(local_transform, global_transform, painter),
            FlowPainter::FlexTube(local_transform) => flex_tube::paint(local_transform, global_transform, painter, theme),
        }
    }
}