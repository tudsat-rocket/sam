use crate::{
    backend::storage::storeable_value::ValveState,
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::conversions::to_pos,
        valves::generic_quick_disconnect::{self, WEDGE_DIAMETER},
    },
};
use egui::Context;
use nalgebra::{Affine2, Point2};

use crate::utils::theme::ThemeColors;

const POSITIONS: [Point2<f32>; 2] = [Point2::new(-0.5, 0f32), Point2::new(-0.5 + WEDGE_DIAMETER, 0f32)];

pub fn paint(
    transform: &Affine2<f32>,
    state: &Option<ValveState>,
    painter: &egui::Painter,
    theme: &ThemeColors,
    ctx: &Context,
) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions, stroke);
    generic_quick_disconnect::paint(transform, state, painter, theme, ctx);
}
