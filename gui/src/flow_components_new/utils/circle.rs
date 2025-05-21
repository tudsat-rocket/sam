use std::sync::LazyLock;

use nalgebra::{Affine2, Point2};
use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform}}, utils::theme::ThemeColors};


const NUM_STEPS: usize = 100;

static POSITIONS: LazyLock<[Point2<f32>; NUM_STEPS]> = LazyLock::new(|| get_positions());

pub fn paint(local_transform: &Transform, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions: Vec<_> = local_transform.apply_to_points(*POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
    painter.line(positions, stroke);
}


fn get_positions() -> [Point2<f32>; NUM_STEPS] {
    let mut positions = [Point2::default(); NUM_STEPS];
    for i in 0..NUM_STEPS {
        let r = (360.0 * (i as f32) / ((NUM_STEPS - 1) as f32)).to_radians();
        positions[i] = Point2::new(0.5 * f32::sin(r) + 0.5, 0.5 * f32::cos(r) + 0.5); 
    }
    return positions;
}