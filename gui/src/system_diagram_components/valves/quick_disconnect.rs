use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{system_diagram_components::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform}, primitives::circle}, utils::theme::ThemeColors};

use super::valve_state::ValveState;

///Diameter of the circles to the left and right of the quick disconnect
const CIRCLE_DIAMETER: f32 = 0.2;
///The width of the gap in the middle of the quick disconnect when it is open
const GAP_WIDTH: f32 = 0.2;
///The height of the vertical lines in the middle of the quick disconnect when it is open
const OPEN_BARRIER_HEIGHT: f32 = 0.6;
///If true, sets the y-scale of the circle to 1. May look much better for some scalings TODO: Make instance dependent?
const STRETCH_CIRCLE_TO_MAX_HEIGHT: bool = true;

const CLOSED_POSITIONS: [Point2<f32>; 7] =  [
    Point2::new(-0.5 + CIRCLE_DIAMETER, 0f32), Point2::new(0.5 - CIRCLE_DIAMETER, 0f32), //Horizonal
    Point2::new(0f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(0f32, OPEN_BARRIER_HEIGHT/2f32), //Vertical
    Point2::new(-0.5, -0.5), Point2::new(-0.5 + CIRCLE_DIAMETER, 0.0), Point2::new(0.5, -0.5) // Left Spike
];
const OPEN_POSITIONS: [Point2<f32>; 8] = [
    Point2::new(-0.5 + CIRCLE_DIAMETER, 0f32), Point2::new(-GAP_WIDTH/2f32, 0f32),
    Point2::new(GAP_WIDTH/2f32, 0f32), Point2::new(0.5 - CIRCLE_DIAMETER, 0f32),
    Point2::new(-GAP_WIDTH/2f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(-GAP_WIDTH/2f32, OPEN_BARRIER_HEIGHT/2f32),
    Point2::new(GAP_WIDTH/2f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(GAP_WIDTH/2f32, OPEN_BARRIER_HEIGHT/2f32)
];
const CIRCLE_HEIGHT: f32 = if STRETCH_CIRCLE_TO_MAX_HEIGHT {1f32} else {CIRCLE_DIAMETER};

//TODO: Implement wedges around circles
pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    circle::paint(&(transform * Transform::new(Rotation2::new(0f32), Scale2::new(CIRCLE_DIAMETER, CIRCLE_HEIGHT), Translation2::new(-0.5 + CIRCLE_DIAMETER/2f32, 0f32)).to_affine2()), painter, theme);
    circle::paint(&(transform * Transform::new(Rotation2::new(0f32), Scale2::new(CIRCLE_DIAMETER, CIRCLE_HEIGHT), Translation2::new( 0.5 - CIRCLE_DIAMETER/2f32, 0f32)).to_affine2()), painter, theme);
    let positions =  match state {
        ValveState::Connected(_) => CLOSED_POSITIONS.as_slice(),
        ValveState::Disconnected => OPEN_POSITIONS.as_slice(),
    }.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    for i in 0..(positions.len() / 2) {
        painter.line(vec![positions[2 * i], positions[2 * i + 1]], stroke);
    }
}