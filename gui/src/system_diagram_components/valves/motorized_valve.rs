use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, transform::Transform},
        sensors_and_actuators::motor,
    },
    utils::theme::ThemeColors,
};

use super::{generic_valve, valve_state::ValveState};

///Length of the valve handle relative to the height of the valve
const HANDLE_LENGTH: f32 = 0.4;
///Diameter of the motor
const MOTOR_DIAMETER: f32 = 0.5;

const POSITIONS: [Point2<f32>; 2] = [Point2::new(0.0, 0.0), Point2::new(0.0, -HANDLE_LENGTH)];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, ctx: &egui::Context) {
    let theme = &ThemeColors::new(ctx);
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions, stroke);
    generic_valve::paint(transform, state, painter, theme);
    motor::paint(
        &(transform
            * Transform::new(
                Rotation2::identity(),
                Scale2::new(MOTOR_DIAMETER, MOTOR_DIAMETER),
                Translation2::new(0f32, -(HANDLE_LENGTH + MOTOR_DIAMETER / 2f32)),
            )
            .to_affine2()),
        painter,
        ctx,
    );
}
