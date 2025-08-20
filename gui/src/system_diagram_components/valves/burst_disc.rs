use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::{core::constants::STROKE_WIDTH, math::conversions::to_pos},
    utils::{
        mesh::{ColoredTexture, TextureKey, create_mesh_from_indices},
        theme::ThemeColors,
    },
};

use super::valve_state::ValveState;

const POSITIONS: [Point2<f32>; 7] = [
    Point2::new(-0.5, -0.5),
    Point2::new(0.5, 0.0),
    Point2::new(-0.5, 0.5),
    Point2::new(-0.5, -0.5),
    Point2::new(0.5, -0.5),
    Point2::new(0.5, 0.5),
    Point2::new(-0.5, 0.5),
];
const MESH_INDICES: [u32; 6] = [3, 4, 5, 5, 6, 0];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let color = match state {
        ValveState::Open => theme.background,
        ValveState::Closed => theme.background_weak,
    };
    let texture = ColoredTexture::new(TextureKey::PatternFull, color);
    let mesh = create_mesh_from_indices(&positions, MESH_INDICES.to_vec(), texture);
    painter.add(mesh);
    painter.line(positions, stroke);
}
