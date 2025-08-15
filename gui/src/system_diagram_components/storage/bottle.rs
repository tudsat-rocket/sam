use std::sync::LazyLock;

use egui::{epaint::PathShape, Shape, Stroke};
use nalgebra::{Affine2, Point2};

use crate::{system_diagram_components::{core::{constants::STROKE_WIDTH, fluids::get_fluid_color}, math::conversions::to_pos}, utils::{mesh::{create_mesh, ColoredTexture, TextureKey}, theme::ThemeColors}};

use super::storage_state::StorageState;

const BULKHEAD_HEIGHT: f32 = 0.15;
const BULKHEAD_STEPS: usize = 100;
const CAP_TOTAL_HEIGHT: f32 = 0.2;
const CAP_WIDTH: f32 = 0.4; 
const CAP_BULKHEAD_HEIGHT: f32 = 0.15;
const CAP_BULKHEAD_STEPS: usize = 100;

static BOTTLE_POSITIONS: LazyLock<Vec<Point2<f32>>> = LazyLock::new(|| get_bottle_positions());
static CAP_POSITIONS: LazyLock<Vec<Point2<f32>>> = LazyLock::new(|| get_cap_positions());

pub fn paint(transform: &Affine2<f32>, state: &StorageState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };

    let bottle_positions: Vec<_> = BOTTLE_POSITIONS.iter().map(|p| to_pos(transform * p)).collect();
    let cap_positions: Vec<_> = CAP_POSITIONS.iter().map(|p| to_pos(transform * p)).collect();

    let fluid_texture_cap = ColoredTexture::new(TextureKey::PatternFull, get_fluid_color(&state.fluid));
    let fluid_texture_bottle = ColoredTexture::new(TextureKey::PatternFull, get_fluid_color(&state.fluid));
    let fluid_mesh_cap = create_mesh(&cap_positions, fluid_texture_cap);
    let fluid_mesh_bottle = create_mesh(&bottle_positions, fluid_texture_bottle);

    let cap_shape = Shape::Path(PathShape::convex_polygon(cap_positions, theme.background_weak, stroke));
    let full_shape = Shape::Path(PathShape::convex_polygon(bottle_positions, theme.background_weak, stroke));

    painter.add(cap_shape);
    painter.add(fluid_mesh_cap);
    painter.add(full_shape);
    painter.add(fluid_mesh_bottle);
}

fn get_bottle_positions() -> Vec<Point2<f32>>{
    let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Point2::new(-0.5 * r.cos(), BULKHEAD_HEIGHT * (1.0 - r.sin())))
            .collect();
    path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Point2::new(-v.x, v.y)));

    let mut path_complete: Vec<_> = path_bulkhead
        .iter()
        .map(|v| Point2::new(v.x, -0.5 + v.y + CAP_TOTAL_HEIGHT))
        .collect();
    path_complete.push(Point2::new(0.5, 0.5));
    path_complete.push(Point2::new(-0.5, 0.5));
    return  path_complete;
}

fn get_cap_positions() -> Vec<Point2<f32>>{
    let mut path_cap_bulkhead: Vec<_> = (0..=CAP_BULKHEAD_STEPS)
        .map(|i| (90.0 * (i as f32) / (CAP_BULKHEAD_STEPS as f32)).to_radians())
        .map(|r| Point2::new(-CAP_WIDTH / 2.0 * r.cos(), CAP_BULKHEAD_HEIGHT * (1.0 - r.sin())))
        .collect();
    path_cap_bulkhead.extend(path_cap_bulkhead.clone().into_iter().rev().map(|v| Point2::new(-v.x, v.y)));

    let mut path_cap_complete: Vec<_> =
        path_cap_bulkhead.iter().map(|v| Point2::new(v.x, -0.5 + v.y)).collect();
    path_cap_complete
        .push(Point2::new(CAP_WIDTH / 2.0, -0.5 + CAP_TOTAL_HEIGHT + BULKHEAD_HEIGHT));
    path_cap_complete
        .push(Point2::new(-CAP_WIDTH / 2.0, -0.5 + CAP_TOTAL_HEIGHT + BULKHEAD_HEIGHT));
    return  path_cap_complete;
}