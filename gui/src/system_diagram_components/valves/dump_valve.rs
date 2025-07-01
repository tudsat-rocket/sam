use nalgebra::{Affine2, Point2};

use crate::{system_diagram_components::{core::constants::STROKE_WIDTH, math::conversions::to_pos}, utils::theme::ThemeColors};


const DIVERGING_LENGTH: f32 = 0.3;
const COVERGING_LENGTH: f32 = 0.4;
const CHAMBER_LENGTH: f32 = 1.0 - DIVERGING_LENGTH - COVERGING_LENGTH;
const THROAT_RADIUS: f32 = 0.3;

const POSITIONS: [Point2<f32>; 8]= [Point2::new(0.5, 0.0), Point2::new(-0.5, 0.0), Point2::new(-0.5, 0.25), Point2::new(0.5, 0.25), Point2::new(0.5, 0.0), Point2::new(0.5, -0.25), Point2::new(-0.5, -0.25), Point2::new(-0.5, 0.0)];


pub fn paint(transform: &Affine2<f32>, painter: &egui::Painter, theme: &ThemeColors) { 
    let stroke = egui::Stroke { width: STROKE_WIDTH, color: theme.foreground_weak };
    let positions = POSITIONS.iter().map(|p| to_pos(transform * p)).collect::<Vec<_>>();
    painter.line(positions, stroke);
}