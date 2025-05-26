use nalgebra::{Point2, Affine2, Rotation2, Scale2, Translation2};

use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform}, utils::circle}, utils::theme::ThemeColors};

use super::valve_state::ValveState;

///Diameter of the circles to the left and right of the quick disconnect
const CIRCLE_DIAMETER: f32 = 0.2;
///The width of the gap in the middle of the quick disconnect when it is open
const GAP_WIDTH: f32 = 0.2;
///The height of the vertical lines in the middle of the quick disconnect when it is open
const OPEN_BARRIER_HEIGHT: f32 = 0.6;
///If true, sets the y-scale of the circle to 1. May look much better for some scalings TODO: Make instance dependent?
const STRETCH_CIRCLE_TO_MAX_HEIGHT: bool = true;

const LINE_SEGMENT_LENGTH: f32 = (1f32 - 2f32 * CIRCLE_DIAMETER - GAP_WIDTH) / 2f32;
const CLOSED_POSITIONS: [Point2<f32>; 4] = [Point2::new(CIRCLE_DIAMETER, 0.5), Point2::new(1f32 - CIRCLE_DIAMETER, 0.5), Point2::new(CIRCLE_DIAMETER + LINE_SEGMENT_LENGTH + 0.5 * GAP_WIDTH, 0.5 - 0.5 * OPEN_BARRIER_HEIGHT), Point2::new(CIRCLE_DIAMETER + LINE_SEGMENT_LENGTH + 0.5 * GAP_WIDTH, 0.5 + 0.5 * OPEN_BARRIER_HEIGHT)];
const OPEN_POSITIONS: [Point2<f32>; 8] = [Point2::new(CIRCLE_DIAMETER, 0.5), Point2::new(CIRCLE_DIAMETER + LINE_SEGMENT_LENGTH, 0.5), Point2::new(1f32 - CIRCLE_DIAMETER - LINE_SEGMENT_LENGTH, 0.5), Point2::new(1f32 - CIRCLE_DIAMETER, 0.5), Point2::new(CIRCLE_DIAMETER + LINE_SEGMENT_LENGTH, 0.5 - 0.5 * OPEN_BARRIER_HEIGHT), Point2::new(CIRCLE_DIAMETER + LINE_SEGMENT_LENGTH, 0.5 + 0.5 * OPEN_BARRIER_HEIGHT), Point2::new(1f32 - CIRCLE_DIAMETER - LINE_SEGMENT_LENGTH, 0.5 - 0.5 * OPEN_BARRIER_HEIGHT), Point2::new(1f32 - CIRCLE_DIAMETER - LINE_SEGMENT_LENGTH, 0.5 + 0.5 * OPEN_BARRIER_HEIGHT)];
const CIRCLE_HEIGHT: f32 = if STRETCH_CIRCLE_TO_MAX_HEIGHT {1f32} else {CIRCLE_DIAMETER};

//TODO: Implement wedges around circles
pub fn paint(local_transform: &Transform, global_transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let global_transform_for_subpainter = global_transform * local_transform.to_affine2();
    circle::paint(&Transform::new(Rotation2::new(0f32), Scale2::new(CIRCLE_DIAMETER, CIRCLE_HEIGHT), Translation2::new(0f32, 0.5 - 0.5 * CIRCLE_HEIGHT)), &global_transform_for_subpainter, painter, theme);
    circle::paint(&Transform::new(Rotation2::new(0f32), Scale2::new(CIRCLE_DIAMETER, CIRCLE_HEIGHT), Translation2::new(1f32 - CIRCLE_DIAMETER, 0.5 - 0.5 * CIRCLE_HEIGHT)), &global_transform_for_subpainter, painter, theme);
    match state {
        ValveState::Connected(_) => {
            let positions: Vec<_> = local_transform.apply_to_points(CLOSED_POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
            for i in 0..(CLOSED_POSITIONS.len() / 2) {
                painter.line(vec![positions[2 * i], positions[2 * i + 1]], stroke);
            }
        }
        ValveState::Disconnected => {
            let positions: Vec<_> = local_transform.apply_to_points(OPEN_POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
            for i in 0..(OPEN_POSITIONS.len() / 2) {
                painter.line(vec![positions[2 * i], positions[2 * i + 1]], stroke);
            }
        }
    };
}