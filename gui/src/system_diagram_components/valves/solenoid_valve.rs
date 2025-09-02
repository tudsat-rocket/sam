use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, transform::Transform},
        sensors_and_actuators::solenoid,
    },
    utils::theme::ThemeColors,
};

use super::{generic_valve, valve_state::ValveState};

///Length of the valve handle relative to the height of the valve
const HANDLE_LENGTH: f32 = 0.4;
///Diameter of the motor
const MOTOR_DIAMETER: f32 = 0.5;

const POSITIONS: [Point2<f32>; 6] = [
    Point2::new(0.5, -0.5),
    Point2::new(0.5, 0.5), //Left
    Point2::new(-0.5, -0.5),
    Point2::new(-0.5, 0.5), //Right
    Point2::new(0.0, 0.0),
    Point2::new(0.0, -HANDLE_LENGTH), //Handle
];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, ctx: &egui::Context) {
    let theme = &ThemeColors::new(ctx);
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let mut positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions.split_off(4), stroke);
    painter.line(positions.split_off(2), stroke);
    painter.line(positions, stroke);
    let solenoid_transform = transform
        * Transform::new(
            Rotation2::identity(),
            Scale2::new(MOTOR_DIAMETER, MOTOR_DIAMETER),
            Translation2::new(0f32, -(HANDLE_LENGTH + MOTOR_DIAMETER / 2f32)),
        )
        .to_affine2();
    let generic_valve_transform = transform
        * Transform::new(Rotation2::identity(), Scale2::new(0.8, 1f32), Translation2::identity()).to_affine2();
    generic_valve::paint(&generic_valve_transform, state, painter, theme);
    solenoid::paint(&solenoid_transform, painter, ctx);
}
