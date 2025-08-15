use nalgebra::{Affine2, Point2, Vector2};

use crate::{system_diagram_components::{core::constants::STROKE_WIDTH, math::conversions::{to_pos, to_vec}}, utils::theme::ThemeColors};

pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let origin = to_pos(transform * Point2::new(-0.5, 0.0));
    let vec = to_vec(transform * Vector2::new(1.0, 0.0));
    painter.arrow(origin, vec, stroke);
}