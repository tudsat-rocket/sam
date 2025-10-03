use crate::{
    backend::storage::storeable_value::ValveState,
    system_diagram_components::{math::ellipse::Circle, primitives::character},
};
use egui::{Color32, Context};
use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{core::constants::STROKE_WIDTH, math::conversions::to_pos},
    utils::theme::ThemeColors,
};

///Diameter of the circles to the left and right of the quick disconnect
pub const WEDGE_DIAMETER: f32 = 0.2;
///The width of the gap in the middle of the quick disconnect when it is open
const GAP_WIDTH: f32 = 0.2;
///The height of the vertical line(s) in the middle of the quick disconnect
const OPEN_BARRIER_HEIGHT: f32 = 0.6;

const CLOSED_POSITIONS: [Point2<f32>; 10] = [
    Point2::new(-0.5 + WEDGE_DIAMETER, 0f32),
    Point2::new(0.5, 0f32), //Horizonal
    Point2::new(0f32, -OPEN_BARRIER_HEIGHT / 2f32),
    Point2::new(0f32, OPEN_BARRIER_HEIGHT / 2f32), //Vertical
    Point2::new(-0.5, -0.5),
    Point2::new(-0.5 + WEDGE_DIAMETER, 0.0),
    Point2::new(-0.5, 0.5), // Left Spike
    Point2::new(0.5, -0.5),
    Point2::new(0.5 - WEDGE_DIAMETER, 0.0),
    Point2::new(0.5, 0.5), // Right Spike
];
const OPEN_POSITIONS: [Point2<f32>; 14] = [
    Point2::new(-0.5 + WEDGE_DIAMETER, 0f32),
    Point2::new(-GAP_WIDTH / 2f32, 0f32), // Left horizontal
    Point2::new(GAP_WIDTH / 2f32, 0f32),
    Point2::new(0.5, 0f32), // Right horizontal
    Point2::new(-GAP_WIDTH / 2f32, -OPEN_BARRIER_HEIGHT / 2f32),
    Point2::new(-GAP_WIDTH / 2f32, OPEN_BARRIER_HEIGHT / 2f32), //Left vertical
    Point2::new(GAP_WIDTH / 2f32, -OPEN_BARRIER_HEIGHT / 2f32),
    Point2::new(GAP_WIDTH / 2f32, OPEN_BARRIER_HEIGHT / 2f32), //Right vertical
    Point2::new(-0.5, -0.5),
    Point2::new(-0.5 + WEDGE_DIAMETER, 0.0),
    Point2::new(-0.5, 0.5), // Left Spike
    Point2::new(0.5, -0.5),
    Point2::new(0.5 - WEDGE_DIAMETER, 0.0),
    Point2::new(0.5, 0.5), // Right Spike
];
const UNKNOWN_POSITIONS: [Point2<f32>; 10] = [
    Point2::new(-0.5 + WEDGE_DIAMETER, 0f32),
    Point2::new(-GAP_WIDTH / 2f32, 0f32), // Left horizontal
    Point2::new(GAP_WIDTH / 2f32, 0f32),
    Point2::new(0.5, 0f32), // Right horizontal
    // Point2::new(-GAP_WIDTH / 2f32, -OPEN_BARRIER_HEIGHT / 2f32),
    // Point2::new(-GAP_WIDTH / 2f32, OPEN_BARRIER_HEIGHT / 2f32), //Left vertical
    // Point2::new(GAP_WIDTH / 2f32, -OPEN_BARRIER_HEIGHT / 2f32),
    // Point2::new(GAP_WIDTH / 2f32, OPEN_BARRIER_HEIGHT / 2f32), //Right vertical
    Point2::new(-0.5, -0.5),
    Point2::new(-0.5 + WEDGE_DIAMETER, 0.0),
    Point2::new(-0.5, 0.5), // Left Spike
    Point2::new(0.5, -0.5),
    Point2::new(0.5 - WEDGE_DIAMETER, 0.0),
    Point2::new(0.5, 0.5), // Right Spike
];

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
    let mut positions = match state {
        Some(s) => match s {
            ValveState::Open => OPEN_POSITIONS.as_slice(),
            ValveState::Closed => CLOSED_POSITIONS.as_slice(),
        },
        None => {
            // let circle_transform = transform
            //     * Transform::new(Rotation2::identity(), Scale2::new(GAP_WIDTH, 1f32), Translation2::identity())
            //         .to_affine2();
            // circle::paint(&circle_transform, painter, theme);
            character::paint(
                "?",
                transform * Point2::origin(),
                2f32 * Circle::new(Point2::origin(), 0.5).transform(transform).incircle().radius(),
                painter,
                ctx,
                Color32::ORANGE,
            );
            UNKNOWN_POSITIONS.as_slice()
        }
    }
    .iter()
    .map(|p| to_pos(transform * p))
    .collect::<Vec<_>>();
    painter.line(positions.split_off(positions.len() - 3), stroke);
    painter.line(positions.split_off(positions.len() - 3), stroke);
    for i in 0..(positions.len() / 2) {
        painter.line(vec![positions[2 * i], positions[2 * i + 1]], stroke);
    }
}
