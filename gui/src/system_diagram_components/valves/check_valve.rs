use crate::{
    system_diagram_components::{
        core::constants::STROKE_WIDTH,
        math::{conversions::to_pos, transform::Transform},
        primitives,
    },
    utils::theme::ThemeColors,
};
use core::f32;
use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2};

const POSITIONS: [Point2<f32>; 4] = [
    Point2::new(-0.5, -0.5),
    Point2::new(-0.5, 0.5),
    Point2::new(0.5, 0.5),
    Point2::new(0.5, -0.5),
];

pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) {
    let stroke = egui::Stroke {
        width: STROKE_WIDTH,
        color: theme.foreground_weak,
    };
    let mut positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions.split_off(2), stroke);
    painter.line(positions, stroke);
    let arrow_transform =
        Transform::new(Rotation2::new(f32::consts::FRAC_PI_4), Scale2::new(1.2, 0.2), Translation2::identity())
            .to_affine2();
    primitives::arrow::paint(&(transform * arrow_transform), painter, theme);
}
