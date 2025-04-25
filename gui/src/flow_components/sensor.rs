use egui::{epaint::{CircleShape, TextShape}, text::LayoutJob, Align, FontFamily, FontId, Pos2, Stroke, TextFormat, Vec2};

use crate::utils::theme::ThemeColors;

use super::{constants::STROKE_WITH, flow_component::{ComponentPainter, ResponseBounds}};


pub struct SymbolPainter {
    pos: Pos2,
    radius: f32,
    symbol: &'static str
}

impl SymbolPainter {

    pub fn new(pos: Pos2, radius: f32, symbol: &'static str) -> Self {
        Self { pos, radius, symbol }
    }

}

impl ComponentPainter for SymbolPainter {

    fn paint(&self, ui: &mut egui::Ui) -> ResponseBounds {
        
        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let pos = available_space.lerp_inside(self.pos.to_vec2());
        let rad = self.radius * available_space.width().min(available_space.height());

        let stroke = Stroke {
            width: STROKE_WITH,
            color: theme.foreground_weak
        };

        let shape = CircleShape{ center: pos, radius: rad, fill: theme.background_weak, stroke };
        let bb = shape.visual_bounding_rect();
        painter.add(shape);
        let mut layout = LayoutJob::default();
        layout.halign = Align::Center;
        layout.append(
            self.symbol,
            0.0,    
            TextFormat {
                font_id: FontId::new(rad * 1.5, FontFamily::Monospace),
                color: theme.foreground_weak,
                ..Default::default()
            },
        );
        let text = TextShape::new(pos - Vec2::new(0.0, rad), ui.ctx().fonts(|fonts| fonts.layout_job(layout)), theme.background_weak);
        painter.add(text);

        return ResponseBounds::new(bb, Some(Box::new(move |pointer| (*pointer - pos).length() < rad)));
    }
}