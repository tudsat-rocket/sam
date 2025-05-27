use std::sync::LazyLock;

use egui::{epaint::PathShape, Shape, Stroke};
use nalgebra::{Affine2, Point2};

use crate::{flow_components::flow_component::get_fluid_color, flow_components_new::{core::constants::STROKE_WIDTH, math::conversions::to_pos}, utils::{mesh::{create_mesh, ColoredTexture, TextureKey}, theme::ThemeColors}};

use super::storage_state::StorageState;

const BULKHEAD_HEIGHT: f32 = 0.15;
const BULKHEAD_STEPS: usize = 100;

static POSITIONS: LazyLock<Vec<Point2<f32>>> = LazyLock::new(|| get_positions());

pub fn paint(transform: &Affine2<f32>, state: &StorageState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };

    let positions: Vec<_> = POSITIONS.iter().map(|p| to_pos(transform * p)).collect();

    //TODO: WRITE SHADER FOR FLUID INSTEAD
    let fluid_texture = ColoredTexture::new(TextureKey::PatternFull, get_fluid_color(&state.fluid));
    let fluid_mesh = create_mesh(&positions, fluid_texture);

    let full_shape = Shape::Path(PathShape::convex_polygon(positions, theme.background_weak, stroke));
    painter.add(full_shape);
    painter.add(fluid_mesh);

}

fn get_positions() -> Vec<Point2<f32>>{
    let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
        .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
        .map(|r| Point2::new(-0.5 * r.cos(), BULKHEAD_HEIGHT * (1.0 - r.sin())))
        .collect();
    path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Point2::new(-v.x, v.y)));

    let mut path_complete: Vec<_> = path_bulkhead.iter().map(|v| Point2::new(v.x, -0.5 + v.y)).collect();
    path_complete.extend(path_bulkhead.iter().rev().map(|v| Point2::new(v.x, 0.5 - v.y)));
    return path_complete;
}