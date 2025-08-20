use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, utils::gather},
    },
    utils::theme::ThemeColors,
};

use super::{generic_valve, valve_state::ValveState};

///TODO Requires better name
const HANDLE_LENGTH: f32 = 0.4;

const POSITIONS: [Point2<f32>; 2] = [Point2::new(0.5, 0.5), Point2::new(0.5, 0.5 - HANDLE_LENGTH)];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(gather(positions.clone(), [0, 1]), stroke);
    generic_valve::paint(transform, state, painter, theme);
}
