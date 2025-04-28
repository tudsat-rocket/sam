use egui::{Pos2, Rect, Stroke, Ui, Vec2};

use crate::utils::theme::ThemeColors;

use super::{
    constants::{LINE_2D_WIDTH, STROKE_WITH},
    flow_component::{ComponentPainter, ResponseBounds},
};

fn find_bounding_box(positions: &Vec<Pos2>) -> Rect {
    const POS2_MAX: Pos2 = Pos2::new(f32::MAX, f32::MAX);
    const POS2_MIN: Pos2 = Pos2::new(f32::MIN, f32::MIN);

    let min = positions.iter().fold(POS2_MAX, |r, p| r.min(*p));
    let max = positions.iter().fold(POS2_MIN, |r, p| r.max(*p));
    return Rect { min, max };
}

pub struct LinePainter1D {
    points: Vec<Pos2>,
}

impl LinePainter1D {
    pub fn new(points: Vec<Pos2>) -> Self {
        Self { points }
    }
}

impl ComponentPainter for LinePainter1D {
    fn paint(&self, ui: &mut Ui) -> ResponseBounds {
        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let stroke = Stroke {
            width: STROKE_WITH,
            color: theme.foreground_weak,
        };

        let line = self.points.iter().map(|p| available_space.lerp_inside(p.to_vec2())).collect::<Vec<_>>();
        let rb = ResponseBounds::new(find_bounding_box(&line), None); //Should we have response bounds here?

        painter.line(line, stroke);

        return rb;
    }
}

pub struct LinePainter2D {
    points: Vec<Pos2>,
}

impl LinePainter2D {
    pub fn new(points: Vec<Pos2>) -> Self {
        Self { points }
    }
}

impl ComponentPainter for LinePainter2D {
    fn paint(&self, ui: &mut Ui) -> ResponseBounds {
        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let stroke = Stroke {
            width: STROKE_WITH,
            color: theme.foreground_weak,
        };
        let mut left_side: Vec<Pos2> = vec![];
        let mut right_side: Vec<Pos2> = vec![];

        let mut o_left_prev = Vec2::ZERO;

        for i in 0..self.points.len() {
            let v = if i < self.points.len() - 1 {
                self.points[i + 1] - self.points[i]
            } else {
                Vec2::ZERO
            };
            let o_left = Vec2::new(-v.y, v.x).normalized() * LINE_2D_WIDTH / 2.0;
            left_side.push(available_space.lerp_inside((self.points[i] + (o_left + o_left_prev)).to_vec2()));
            right_side.push(available_space.lerp_inside((self.points[i] - (o_left + o_left_prev)).to_vec2()));
            o_left_prev = o_left;
        }

        let mut positions = left_side.clone();
        positions.append(&mut right_side.clone());
        let rb = ResponseBounds::new(find_bounding_box(&positions), None);

        painter.line(left_side, stroke);
        painter.line(right_side, stroke);

        return rb;
    }
}
