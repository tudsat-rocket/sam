use egui::{Painter, Pos2, Rect, Stroke};

use crate::utils::theme::ThemeColors;


pub struct Line {
    points: Vec<Pos2>
}

impl Line{

    pub fn new(points: Vec<Pos2>) -> Self{
        Self{
            points
        }
    }

    pub fn draw(&self, painter: &Painter, theme: &ThemeColors, available_space: &Rect){
        let stroke = Stroke {
            width: 1.6,
            color: theme.foreground_weak
        };
        let points_in_widget_space: Vec<Pos2>= self.points.iter()
                                                .map(|p| available_space.lerp_inside(p.to_vec2()))
                                                .collect();
        painter.line(points_in_widget_space, stroke);
    }
}