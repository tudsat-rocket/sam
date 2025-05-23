use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform, utils::gather}}, utils::theme::ThemeColors};

use super::{generic_valve, valve_state::ValveState};

///Length of the valve handle relative to the height of the valve
const HANDLE_LENGTH: f32 = 0.4;
///Width of the valve handle relative to the width of the valve
const HANDLE_WIDTH: f32 = 0.5;

const POSITIONS: [Point2<f32>; 4] = [Point2::new(0.5, 0.5), Point2::new(0.5, 0.5 - HANDLE_LENGTH), Point2::new(0.5 - 0.5 * HANDLE_WIDTH, 0.5 - HANDLE_LENGTH), Point2::new(0.5 + 0.5 * HANDLE_WIDTH, 0.5 - HANDLE_LENGTH)];

pub fn paint(local_transform: &Transform, global_transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions: Vec<_> = local_transform.apply_to_points(POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
    painter.line(gather(positions.clone(), [0, 1]), stroke);
    painter.line(gather(positions, [2, 3]), stroke);
    generic_valve::paint(local_transform, global_transform, state, painter, theme);
}