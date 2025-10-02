use crate::backend::storage::storeable_value::ValveState;
use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, utils::gather},
    },
    utils::theme::ThemeColors,
};

use super::generic_valve;

///Length of the valve handle relative to the height of the valve
const HANDLE_LENGTH: f32 = 0.6;
///Width of the valve handle relative to the width of the valve
const HANDLE_WIDTH: f32 = 0.7;

const POSITIONS: [Point2<f32>; 4] = [
    Point2::new(0.0, 0.0),
    Point2::new(0.0, -HANDLE_LENGTH),
    Point2::new(-HANDLE_WIDTH / 2f32, -HANDLE_LENGTH),
    Point2::new(HANDLE_WIDTH / 2f32, -HANDLE_LENGTH),
];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(gather(positions.clone(), [0, 1]), stroke);
    painter.line(gather(positions, [2, 3]), stroke);
    generic_valve::paint(transform, state, painter, theme);
}
