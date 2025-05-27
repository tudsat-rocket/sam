use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::conversions::to_pos}, utils::theme::ThemeColors};

pub fn paint(points: &Vec<Point2<f32>>, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    painter.line(points.into_iter().map(|p| to_pos(global_transform * p)).collect(), stroke);
    //return ResponseBounds::new(Rect::NOTHING, None);
}