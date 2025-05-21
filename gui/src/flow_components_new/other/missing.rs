use egui::Color32;
use nalgebra::{Affine2, Point2};

use crate::{flow_components_new::math::{conversions::to_pos, transform::Transform}, utils::mesh::{create_mesh_from_indices, ColoredTexture, TextureKey}};

const POSITIONS: [Point2<f32>; 4] = [Point2::new(0.0, 0.0), Point2::new(1.0, 0.0), Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)];
const INDICES: [u32; 6] = [0, 1, 2, 0, 2, 3];

pub fn paint(local_transform: &Transform, global_transform: &Affine2<f32>, painter: &egui::Painter) {
        let positions = local_transform.apply_to_points(POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
        let texture = ColoredTexture::new(TextureKey::PatternMissing, Color32::WHITE);
        let mesh = create_mesh_from_indices(&positions, INDICES.to_vec(), texture);
        painter.add(mesh);
        //return ResponseBounds::new(Rect::NOTHING, None);
}