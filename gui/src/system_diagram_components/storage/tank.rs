use std::sync::LazyLock;

use egui::{Shape, Stroke, epaint::PathShape};
use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{
        core::{constants::STROKE_WIDTH, fluids::get_fluid_color},
        math::conversions::to_pos,
    },
    utils::{
        mesh::{ColoredTexture, TextureKey, create_mesh},
        theme::ThemeColors,
    },
};

use super::storage_state::StorageState;

const BULKHEAD_HEIGHT: f32 = 0.15;
const BULKHEAD_STEPS: usize = 100;

static POSITIONS: LazyLock<Vec<Point2<f32>>> = LazyLock::new(|| get_positions());

pub fn paint(transform: &Affine2<f32>, state: &StorageState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };

    let positions: Vec<_> = POSITIONS.iter().map(|p| to_pos(transform * p)).collect();

    //TODO: WRITE SHADER FOR FLUID INSTEAD
    let filled_positions = get_filled_positions(state.level).iter().map(|p| to_pos(transform * p)).collect();
    let fluid_texture = ColoredTexture::new(TextureKey::PatternFull, get_fluid_color(&state.fluid));
    let fluid_mesh = create_mesh(&filled_positions, fluid_texture);

    let full_shape = Shape::Path(PathShape::convex_polygon(positions, theme.background_weak, stroke));
    painter.add(full_shape);
    painter.add(fluid_mesh);
}

fn get_positions() -> Vec<Point2<f32>> {
    let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
        .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
        .map(|r| Point2::new(-0.5 * r.cos(), BULKHEAD_HEIGHT * (1.0 - r.sin())))
        .collect();
    path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Point2::new(-v.x, v.y)));

    let mut path_complete: Vec<_> = path_bulkhead.iter().map(|v| Point2::new(v.x, -0.5 + v.y)).collect();
    path_complete.extend(path_bulkhead.iter().rev().map(|v| Point2::new(v.x, 0.5 - v.y)));
    return path_complete;
}

fn get_filled_positions(fill_percentage: f32) -> Vec<Point2<f32>> {
    let mut filled_positions =
        POSITIONS.clone().into_iter().filter(|p| p.y > (0.5 - fill_percentage)).collect::<Vec<_>>();
    if fill_percentage > BULKHEAD_HEIGHT && fill_percentage < (1.0 - BULKHEAD_HEIGHT) {
        filled_positions.extend(&[
            Point2::new(-0.5, 0.5 - fill_percentage),
            Point2::new(0.5, 0.5 - fill_percentage),
        ]);
    }
    return filled_positions;
}
