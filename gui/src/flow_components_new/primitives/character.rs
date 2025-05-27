use egui::{epaint::TextShape, text::LayoutJob, Align, FontFamily, FontId, TextFormat};
use nalgebra::{Point2, Vector2};

use crate::{flow_components_new::math::conversions::to_pos, utils::theme::ThemeColors};

pub fn paint(text: &str, target_center: Point2<f32>, target_height:f32, painter: &egui::Painter, ctx: &egui::Context) {
    let theme = ThemeColors::new(ctx);
    let mut layout = LayoutJob::default();
    layout.halign = Align::Center;
    layout.append(
        text,
        0.0,
        TextFormat {
            font_id: FontId::new(target_height, FontFamily::Monospace),
            color: theme.foreground_weak,
            ..Default::default()
        },
    );
    let text = TextShape::new(
        to_pos(target_center - Vector2::new(0f32, 0.7 * target_height)),
        ctx.fonts(|fonts| fonts.layout_job(layout)),
        theme.background_weak,
    );
    painter.add(text);
    //return ResponseBounds::new(Rect::NOTHING, None);
}