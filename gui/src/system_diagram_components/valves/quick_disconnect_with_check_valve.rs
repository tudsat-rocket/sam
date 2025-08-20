use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{system_diagram_components::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform, triangle::Triangle}, primitives::circle}, utils::theme::ThemeColors};

use super::valve_state::ValveState;

///Diameter of the circles to the left and right of the quick disconnect
const WEDGE_DIAMETER: f32 = 0.2;
///The width of the gap in the middle of the quick disconnect when it is open
const GAP_WIDTH: f32 = 0.2;
///The height of the vertical line(s) in the middle of the quick disconnect
const OPEN_BARRIER_HEIGHT: f32 = 0.6;

const CLOSED_POSITIONS: [Point2<f32>; 10] =  [
    Point2::new(-0.5 + WEDGE_DIAMETER, 0f32), Point2::new(0.5, 0f32), //Horizonal
    Point2::new(0f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(0f32, OPEN_BARRIER_HEIGHT/2f32), //Vertical
    Point2::new(-0.5, -0.5), Point2::new(-0.5 + WEDGE_DIAMETER, 0.0), Point2::new(-0.5, 0.5), // Left Spike
    Point2::new(0.5, -0.5), Point2::new(0.5 - WEDGE_DIAMETER, 0.0), Point2::new(0.5, 0.5), // Right Spike
];
const OPEN_POSITIONS: [Point2<f32>; 14] = [
    Point2::new(-0.5 + WEDGE_DIAMETER, 0f32), Point2::new(-GAP_WIDTH/2f32, 0f32),  // Left horizontal
    Point2::new(GAP_WIDTH/2f32, 0f32), Point2::new(0.5, 0f32),    // Right horizontal
    Point2::new(-GAP_WIDTH/2f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(-GAP_WIDTH/2f32, OPEN_BARRIER_HEIGHT/2f32), //Left vertical
    Point2::new(GAP_WIDTH/2f32, -OPEN_BARRIER_HEIGHT/2f32), Point2::new(GAP_WIDTH/2f32, OPEN_BARRIER_HEIGHT/2f32), //Right vertical
    Point2::new(-0.5, -0.5), Point2::new(-0.5 + WEDGE_DIAMETER, 0.0), Point2::new(-0.5, 0.5), // Left Spike
    Point2::new(0.5, -0.5), Point2::new(0.5 - WEDGE_DIAMETER, 0.0), Point2::new(0.5, 0.5), // Right Spike
];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let incircle = Triangle::new([Point2::new(-0.5, -0.5), Point2::new(0.5, 0.0), Point2::new(-0.5, 0.5)]).incircle();
    //TODO Hans: I hardcoded this transform to look good, should be calculated correctly instead
    circle::paint(&(transform * Transform::new(Rotation2::identity(), Scale2::new(incircle.radius() * 0.55, incircle.radius()), Translation2::new(incircle.center().x - 0.225, 0f32)).to_affine2()), painter, theme);
    let mut positions =  match state {
        ValveState::Open => OPEN_POSITIONS.as_slice(),
        ValveState::Closed => CLOSED_POSITIONS.as_slice(),
    }.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions.split_off(positions.len() - 3), stroke);
    painter.line(positions.split_off(positions.len() - 3), stroke);
    for i in 0..(positions.len() / 2) {
        painter.line(vec![positions[2 * i], positions[2 * i + 1]], stroke);
    }
}