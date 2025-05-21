use std::sync::LazyLock;

use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform}}, utils::theme::ThemeColors};


///Determines the resolution of the tube
const STEP_NUM: usize = 100;
///Maximum height of the tube relative to the component height
const HEIGHT: f32 = 0.8;
///Number of tick marks on the tube
const TICK_NUM: usize = 7;
///Length of tick marks relative to the component height
const TICK_LENGTH: f32 = 0.15;

const LEFT_SIDE: [Point2<f32>; 2] = [Point2::new(0f32, 0f32), Point2::new(0f32, 1f32)];
const RIGHT_SIDE: [Point2<f32>; 2] = [Point2::new(1f32, 0f32), Point2::new(1f32, 1f32)];
static TUBE_POSITIONS: LazyLock<Vec<Point2<f32>>> = LazyLock::new(|| get_tube_positions());
static TICK_POSITIONS: LazyLock<[Point2<f32>; TICK_NUM * 2]> = LazyLock::new(|| get_tick_positions());



pub fn paint(local_transform: &Transform, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };

    let left_side: Vec<_> = local_transform.apply_to_points(LEFT_SIDE).into_iter().map(|p| to_pos(global_transform * p)).collect();
    let right_side: Vec<_> = local_transform.apply_to_points(RIGHT_SIDE).into_iter().map(|p| to_pos(global_transform * p)).collect();
    let tube_positions: Vec<_> = local_transform.apply_to_points((*TUBE_POSITIONS).clone()).into_iter().map(|p| to_pos(global_transform * p)).collect();
    let tick_positions: Vec<_> = local_transform.apply_to_points((*TICK_POSITIONS).clone()).into_iter().map(|p| to_pos(global_transform * p)).collect();

    painter.line(left_side, stroke);
    painter.line(right_side, stroke);
    painter.line(tube_positions, stroke);

    for i in 0..TICK_NUM {
        painter.line(vec![tick_positions[2 * i], tick_positions[2 * i + 1]], stroke);
    }
}

fn get_tube_positions() -> Vec<Point2<f32>> {
    let path_tube: Vec<_> = (0..=STEP_NUM)
        .map(|i| (360.0 * (i as f32) / (STEP_NUM as f32)).to_radians())
        .map(|r| {
            Point2::new(r / (2.0 * std::f32::consts::PI), -r.sin() * HEIGHT * 0.5 + 0.5)
        })
        .collect();
    return path_tube;
}

fn get_tick_positions() -> [Point2<f32>; TICK_NUM * 2] {
    let mut positions = [Point2::default(); TICK_NUM * 2];
    for i in 0..TICK_NUM {
        let r = (360.0 * ((i + 1) as f32) / (TICK_NUM + 1) as f32).to_radians();
        let mid = Point2::new(r / (2.0 * std::f32::consts::PI), -r.sin() * 0.5 * HEIGHT + 0.5);
        positions[2 * i + 0] = Point2::new(mid.x, mid.y - 0.5 * TICK_LENGTH);
        positions[2 * i + 1] = Point2::new(mid.x, mid.y + 0.5 * TICK_LENGTH);
    }
    return positions;
}