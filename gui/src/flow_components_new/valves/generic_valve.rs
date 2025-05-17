use nalgebra::{Affine2, Point2};
use crate::{flow_components::flow_component::{get_fluid_color, ConnectionState}, flow_components_new::{constants::STROKE_WIDTH, math::{conversions::to_pos, transform::Transform, utils::gather}}, utils::{mesh::{create_mesh_from_indices, ColoredTexture, TextureKey}, theme::ThemeColors}};

const POSITIONS: [Point2<f32>; 5] = [Point2::new(0f32, 0f32), Point2::new(0.5, 0.5), Point2::new(1f32, 1f32), Point2::new(1f32, 0f32), Point2::new(0f32, 1f32)];
const MESH_INDICES: [u32; 6] = [0, 1, 4, 1, 2, 3];
const OUTLINE_INDICES: [usize; 5] = [0, 2, 3, 4, 0];

pub fn paint_generic_valve(state: &ConnectionState, local_transform: &Transform, global_transform: &Affine2<f32>, painter: &egui::Painter, theme: ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions: Vec<_> = local_transform.apply_to_points(POSITIONS).into_iter().map(|p| to_pos(global_transform * p)).collect();
    let color = match state {
        ConnectionState::Connected(fluid_type) => match fluid_type {
            Some(fluid) => get_fluid_color(fluid),
            None => theme.background,
        },
        ConnectionState::Disconnected => theme.foreground_weak,
    };
    let texture = ColoredTexture::new(TextureKey::PatternFull, color);
    let mesh = create_mesh_from_indices(&positions, MESH_INDICES.to_vec(), texture);
    painter.add(mesh);
    painter.line(gather(positions, OUTLINE_INDICES), stroke);
}