use egui::{epaint::PathShape, Rect, Shape, Stroke, Ui, Vec2};

use crate::utils::theme::ThemeColors;

use super::flow_component::{ComponentPainter, ResponseBounds};

pub struct ValvePainter {
    pos: Vec2,
    width: f32,
    //height: f32,
    //stroke_width: f32,
}

impl ValvePainter {
    
    pub fn new(pos: Vec2, width: f32) -> Self {
        Self { 
            pos, width
        }
    }

}

impl ComponentPainter for ValvePainter {
    
    fn paint(&self, ui: &mut Ui) -> ResponseBounds{

        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let stroke = Stroke {
            width: 1.6,
            color: theme.foreground_weak
        };
        // if open {
        //     let shape = Shape::Rect(
        //         RectShape::new(
        //             Rect::from_center_size(pos, Vec2::splat(w * 1.2)),
        //             Rounding::same(0.0),
        //             Color32::from_rgb(0xfe, 0x80, 0x19),
        //             Stroke::NONE
        //         )
        //     );
        //     painter.add(shape);
        // }
        let pos = available_space.lerp_inside(self.pos);
        let w = available_space.width() * self.width;

        let h = 30f32.to_radians().sin() * w;
        let (path_a, path_b) = {//if vertical {
        //     (
        //         vec![pos, pos - Vec2::new(h/2.0, w/2.0), pos - Vec2::new(-h/2.0, w/2.0)],
        //         vec![pos, pos + Vec2::new(h/2.0, w/2.0), pos + Vec2::new(-h/2.0, w/2.0)]
        //     )
        // } else {
            (
                vec![pos, pos - Vec2::new(w/2.0, h/2.0), pos - Vec2::new(w/2.0, -h/2.0)],
                vec![pos, pos + Vec2::new(w/2.0, h/2.0), pos + Vec2::new(w/2.0, -h/2.0)]
            )
        };

        let c = {//if open {
        //     Color32::TRANSPARENT
        // } else {
                stroke.color
        };

        painter.add(Shape::Path(PathShape::convex_polygon(path_a, c, stroke)));
        painter.add(Shape::Path(PathShape::convex_polygon(path_b, c, stroke)));

        return ResponseBounds::new(Rect { min: (pos - Vec2::new(w/2.0, h/2.0)), max: (pos + Vec2::new(w/2.0, h/2.0)) }, None)
    }
}

