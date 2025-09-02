use egui::Color32;
use nalgebra::{Affine2, Point2};

use crate::{
    system_diagram_components::math::conversions::to_pos,
    utils::mesh::{ColoredTexture, TextureKey, create_mesh_from_indices},
};

const POSITIONS: [Point2<f32>; 4] = [
    Point2::new(-0.5, -0.5),
    Point2::new(0.5, -0.5),
    Point2::new(0.5, 0.5),
    Point2::new(-0.5, 0.5),
];
const INDICES: [u32; 6] = [0, 1, 2, 0, 2, 3];

pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter) {
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let texture = ColoredTexture::new(TextureKey::PatternMissing, Color32::WHITE);
    let mesh = create_mesh_from_indices(&positions, INDICES.to_vec(), texture);
    painter.add(mesh);
    //return ResponseBounds::new(Rect::NOTHING, None);
}
