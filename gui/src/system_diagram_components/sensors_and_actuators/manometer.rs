use core::f32;

use egui::Shape;
use nalgebra::{Affine2, Rotation2, Scale2, Translation2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, transform::Transform},
        primitives::*,
    },
    utils::theme::ThemeColors,
};

pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, ctx: &egui::Context) {
    let theme = ThemeColors::new(ctx);
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let points = circle::POINTS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.add(Shape::convex_polygon(points, theme.background_weak, stroke));
    let arrow_transform =
        Transform::new(Rotation2::new(-f32::consts::FRAC_PI_4 * 3f32), Scale2::new(0.6, 1.0), Translation2::identity())
            .to_affine2();
    arrow::paint(&(transform * arrow_transform), painter, &theme);
}
