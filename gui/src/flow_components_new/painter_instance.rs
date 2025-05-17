use nalgebra::Affine2;

use crate::utils::theme::ThemeColors;

use super::{flow_painter::FlowPainter, math::transform::Transform, missing::paint_missing, valves::generic_valve::paint_generic_valve};

pub struct PainterInstance {
    painter_type: FlowPainter,
    local_transform: Transform,
}

impl PainterInstance {

    pub fn new(painter_type: FlowPainter, transform: Transform) -> Self {
        Self {painter_type, local_transform: transform}
    }

    pub fn paint(&self, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: ThemeColors) {
        match &self.painter_type {
            FlowPainter::Missing => paint_missing(&self.local_transform, global_transform, painter),
            FlowPainter::GenericValve(state) => paint_generic_valve(state, &self.local_transform, global_transform, painter, theme),
        }
    }

}