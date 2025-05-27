use std::sync::LazyLock;

use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::{core::constants::STROKE_WIDTH, math::conversions::to_pos}, utils::theme::ThemeColors};


///Determines the resolution of the tube
const STEP_NUM: usize = 100;
///Maximum height of the tube relative to the component height
const HEIGHT: f32 = 0.8;
///Number of tick marks on the tube
const TICK_NUM: usize = 7;
///Length of tick marks relative to the component height
const TICK_LENGTH: f32 = 0.15;

const LEFT_SIDE: [Point2<f32>; 2] = [Point2::new(-0.5, -0.5), Point2::new(-0.5, 0.5)];
const RIGHT_SIDE: [Point2<f32>; 2] = [Point2::new(0.5, -0.5), Point2::new(0.5, 0.5)];
static TUBE_POSITIONS: LazyLock<[Point2<f32>; STEP_NUM]> = LazyLock::new(|| get_tube_positions());
static TICK_POSITIONS: LazyLock<[Point2<f32>; TICK_NUM * 2]> = LazyLock::new(|| get_tick_positions());



pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };

    let left_side = LEFT_SIDE.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let right_side = RIGHT_SIDE.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let tube_positions = TUBE_POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let tick_positions = TICK_POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();

    painter.line(left_side, stroke);
    painter.line(right_side, stroke);
    painter.line(tube_positions, stroke);

    for i in 0..TICK_NUM {
        painter.line(vec![tick_positions[2 * i], tick_positions[2 * i + 1]], stroke);
    }
}

fn get_tube_positions() -> [Point2<f32>; STEP_NUM] {
    return core::array::from_fn(|i| i)
        .map(|i| (360.0 * (i as f32) / (STEP_NUM as f32)).to_radians())
        .map(|r| {
            Point2::new(r / (2.0 * std::f32::consts::PI) - 0.5, -r.sin() * HEIGHT/2f32)
        });
}

fn get_tick_positions() -> [Point2<f32>; TICK_NUM * 2] {
    let mut positions = [Point2::default(); TICK_NUM * 2];
    for i in 0..TICK_NUM {
        let r = (360.0 * ((i + 1) as f32) / (TICK_NUM + 1) as f32).to_radians();
        let mid = Point2::new(r / (2.0 * std::f32::consts::PI) - 0.5, -r.sin() * HEIGHT/2f32);
        positions[2 * i + 0] = Point2::new(mid.x, mid.y - TICK_LENGTH/2f32);
        positions[2 * i + 1] = Point2::new(mid.x, mid.y + TICK_LENGTH/2f32);
    }
    return positions;
}