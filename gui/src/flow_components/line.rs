use egui::{pos2, Painter, Pos2, Rect, Stroke, Ui, Vec2};
use egui_plot::Line;

use crate::{flow_components::line, utils::theme::ThemeColors};

use super::flow_component::{ComponentPainter, ResponseBounds};


pub struct LinePainter {
    points: Vec<Pos2>
}

impl LinePainter{
    pub fn new(points: Vec<Pos2>) -> Self{
        Self{
            points
        }
    }

    fn find_bounding_box(positions: Vec<Pos2>) -> Rect {
        const POS2_MAX: Pos2 = Pos2::new(f32::MAX, f32::MAX);
        const POS2_MIN: Pos2 = Pos2::new(f32::MIN, f32::MIN);

        let min = positions.clone().into_iter()
                                            .fold(POS2_MAX,|r, p| {
                                                r.min(p)
                                            });
        let max = positions.into_iter()
                                            .fold(POS2_MIN, |r, p| {
                                                    r.max(p)
                                            });
        return Rect {min, max};
    }
}

impl ComponentPainter for LinePainter{

    fn paint(&self, ui: &mut Ui) -> ResponseBounds{

        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let stroke = Stroke {
            width: 1.6,
            color: theme.foreground_weak
        };
        let line_width = 0.05;
        let mut left_side: Vec<Pos2> = vec![];
        let mut right_side: Vec<Pos2> = vec![];

        let mut o_left_prev = Vec2::ZERO;

        for i in 0..self.points.len() {
            let v =   if i < self.points.len() - 1
                                {self.points[i + 1] - self.points[i]}
                            else
                                {Vec2::ZERO};
            let o_left = Vec2::new(-v.y, v.x).normalized() * line_width / 2.0;
            left_side.push(available_space.lerp_inside((self.points[i] + (o_left + o_left_prev)).to_vec2()));
            right_side.push(available_space.lerp_inside((self.points[i] - (o_left + o_left_prev)).to_vec2()));
            o_left_prev = o_left;
        }

        let mut positions = left_side.clone();
        positions.append(&mut right_side.clone());
        let rb =  ResponseBounds::new(LinePainter::find_bounding_box(positions), None);

        painter.line(left_side, stroke);
        painter.line(right_side, stroke);

        return rb;
    }
}