use std::sync::LazyLock;

use nalgebra::{Affine2, Point2};
use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::conversions::to_pos}, utils::theme::ThemeColors};


const NUM_STEPS: usize = 100;

pub static POINTS: LazyLock<[Point2<f32>; NUM_STEPS]> = LazyLock::new(|| calculate_circle_points());

pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions = POINTS.into_iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions, stroke);
}

fn calculate_circle_points() -> [Point2<f32>; NUM_STEPS] {
    let mut points = [Point2::default(); NUM_STEPS];
    for i in 0..NUM_STEPS {
        let r = (360.0 * (i as f32) / ((NUM_STEPS - 1) as f32)).to_radians();
        points[i] = Point2::new(0.5 * f32::sin(r), 0.5 * f32::cos(r)); 
    }
    return points;
}