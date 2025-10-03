use crate::backend::storage::storeable_value::ValveState;
use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, utils::gather},
    },
    utils::{
        mesh::{ColoredTexture, TextureKey, create_mesh_from_indices},
        theme::ThemeColors,
    },
};
use egui::Color32;
use nalgebra::{Affine2, Point2};

const POSITIONS: [Point2<f32>; 5] = [
    Point2::new(-0.5, -0.5),
    Point2::new(0.0, 0.0),
    Point2::new(0.5, 0.5),
    Point2::new(0.5, -0.5),
    Point2::new(-0.5, 0.5),
];
const MESH_INDICES: [u32; 6] = [0, 1, 4, 1, 2, 3];
const OUTLINE_INDICES: [usize; 5] = [0, 2, 3, 4, 0];

pub fn paint(transform: &Affine2<f32>, state: &Option<ValveState>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let texture = match state {
        Some(s) => match s {
            ValveState::Open => ColoredTexture::new(TextureKey::PatternFull, theme.background_weak),
            ValveState::Closed => ColoredTexture::new(TextureKey::PatternFull, theme.foreground_weak),
        },
        None => ColoredTexture::new(TextureKey::PatternDiagonal, Color32::ORANGE),
    };
    let mesh = create_mesh_from_indices(&positions, MESH_INDICES.to_vec(), texture);
    painter.add(mesh);
    painter.line(gather(positions, OUTLINE_INDICES), stroke);
}
