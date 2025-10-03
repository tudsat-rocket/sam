use crate::{
    backend::storage::storeable_value::ValveState, system_diagram_components::valves::generic_quick_disconnect,
};
use egui::Context;
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

use crate::{
    system_diagram_components::{
        math::{transform::Transform, triangle::Triangle},
        primitives::circle,
    },
    utils::theme::ThemeColors,
};

pub fn paint(
    transform: &Affine2<f32>,
    state: &Option<ValveState>,
    painter: &egui::Painter,
    theme: &ThemeColors,
    ctx: &Context,
) {
    let incircle = Triangle::new([Point2::new(-0.5, -0.5), Point2::new(0.5, 0.0), Point2::new(-0.5, 0.5)]).incircle();
    //TODO Hans: I hardcoded this transform to look good, should be calculated correctly instead
    circle::paint(
        &(transform
            * Transform::new(
                Rotation2::identity(),
                Scale2::new(incircle.radius() * 0.55, incircle.radius()),
                Translation2::new(incircle.center().x - 0.225, 0f32),
            )
            .to_affine2()),
        painter,
        theme,
    );
    generic_quick_disconnect::paint(transform, state, painter, theme, ctx);
}
