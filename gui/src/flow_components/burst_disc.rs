use egui::{epaint::RectShape, Pos2, Rect, Rounding, Vec2};

use crate::utils::theme::ThemeColors;

use super::{constants::STROKE_WITH, flow_component::{ComponentPainter, ResponseBounds}};

pub struct BurstDiscPainter {
    pos: Pos2,
    width: f32,
    height: f32
}

impl BurstDiscPainter {
    
    pub fn new(pos: Pos2, width: f32, height: f32) -> Self {
        Self { pos, width, height }
    }

}

impl ComponentPainter for BurstDiscPainter {

    fn paint(&self, ui: &mut egui::Ui) -> ResponseBounds {
        
        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let stroke = egui::Stroke {
            width: STROKE_WITH,
            color: theme.foreground_weak
        };

        let pos = available_space.lerp_inside(self.pos.to_vec2());
        let width = available_space.width() * self.width;
        let height = available_space.height() * self.height;

        let bb = Rect::from_center_size(pos, Vec2::new(width, height));

        painter.add(RectShape::stroke(bb.clone(), Rounding::ZERO, stroke));
        painter.line(vec![pos - Vec2::new(width, height) / 2.0, pos + Vec2::new(width, 0.0) / 2.0, pos + Vec2::new(-width, height) / 2.0], stroke);

        return ResponseBounds::new(bb, None);
    }

}