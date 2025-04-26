use egui::{Pos2, Rect, Vec2};

use crate::utils::theme::ThemeColors;

use super::{constants::{FLEX_TUBE_HEIGHT, FLEX_TUBE_STEP_NUM, FLEX_TUBE_TICK_LENGTH, FLEX_TUBE_TICK_NUM, STROKE_WITH}, flow_component::{ComponentPainter, ResponseBounds}};

pub struct FlexTubePainter {
    pos: Pos2,
    width: f32,
    height: f32
}

impl FlexTubePainter {
    
    pub fn new(pos: Pos2, width: f32, height: f32) -> Self {
        Self { pos, width, height }
    }

}

impl ComponentPainter for FlexTubePainter {

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

        painter.line(vec![pos + Vec2::new(-width, -height) / 2.0, pos + Vec2::new(-width, height) / 2.0], stroke);
        painter.line(vec![pos + Vec2::new( width, -height) / 2.0, pos + Vec2::new( width, height) / 2.0], stroke);

        let tube_half_height = height * FLEX_TUBE_HEIGHT * 0.5;
        let path_tube: Vec<_> = (0..=FLEX_TUBE_STEP_NUM)
        .map(|i| (360.0 * (i as f32) / (FLEX_TUBE_STEP_NUM as f32)).to_radians())
        .map(|r| pos + Vec2::new(r * width / (2.0 * std::f32::consts::PI) - width / 2.0, -r.sin() * tube_half_height))
        .collect();

        painter.line(path_tube, stroke);
    
        let tick_half_length = FLEX_TUBE_TICK_LENGTH * height;
        for i in 1..(FLEX_TUBE_TICK_NUM + 1) {
            let r = (360.0 * (i as f32) / (FLEX_TUBE_TICK_NUM + 1) as f32).to_radians();
            let mid = pos + Vec2::new(r * width / (2.0 * std::f32::consts::PI) - width / 2.0, -r.sin() * tube_half_height);
            painter.line(vec![mid + Vec2::new(0.0, tick_half_length), mid - Vec2::new(0.0, tick_half_length)], stroke);
        }

        return ResponseBounds::new(Rect::from_center_size(pos, Vec2::new(width, height)), None);
    }

}