use egui::Shape;
use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, ellipse::Circle},
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
    character::paint(
        "M",
        transform * Point2::origin(),
        2f32 * Circle::new(Point2::origin(), 0.5).transform(transform).incircle().radius(),
        painter,
        ctx,
    );
}
