use nalgebra::{Affine2, Point2};
use crate::{system_diagram_components::{core::{constants::STROKE_WIDTH, fluids::get_fluid_color}, math::{conversions::to_pos, utils::gather}}, utils::{mesh::{create_mesh_from_indices, ColoredTexture, TextureKey}, theme::ThemeColors}};

use super::valve_state::ValveState;

const POSITIONS: [Point2<f32>; 5] = [Point2::new(-0.5, -0.5), Point2::new(0.0, 0.0), Point2::new(0.5, 0.5), Point2::new(0.5, -0.5), Point2::new(-0.5, 0.5)];
const MESH_INDICES: [u32; 6] = [0, 1, 4, 1, 2, 3];
const OUTLINE_INDICES: [usize; 5] = [0, 2, 3, 4, 0];

pub fn paint(transform: &Affine2<f32>, state: &ValveState, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    let color = match state {
        ValveState::Connected(contents) => match contents {
            Some(fluid) => get_fluid_color(fluid),
            None => theme.background_weak,
        },
        ValveState::Disconnected => theme.foreground_weak,
    };
    let texture = ColoredTexture::new(TextureKey::PatternFull, color);
    let mesh = create_mesh_from_indices(&positions, MESH_INDICES.to_vec(), texture);
    painter.add(mesh);
    painter.line(gather(positions, OUTLINE_INDICES), stroke);
}