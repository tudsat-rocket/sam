use egui::{epaint::PathShape, Pos2, Rect, Shape, Stroke, Ui, Vec2};

use crate::utils::theme::ThemeColors;

use super::{constants::{BALL_VALVE_SYMBOL_DISTANCE, BALL_VALVE_SYMBOL_RADIUS, TANK_VALVE_HANDLE_LENGTH, TANK_VALVE_HANDLE_WIDTH}, flow_component::{ComponentPainter, ResponseBounds}, line::LinePainter1D, sensor::SymbolPainter};

pub struct ValvePainter {
    pos: Pos2,
    width: f32,
    //height: f32,
    //stroke_width: f32,
}

impl ValvePainter {
    
    pub fn new(pos: Pos2, width: f32) -> Self {
        Self { 
            pos, width
        }
    }

    pub fn get_height(&self) -> f32 {
        return 30f32.to_radians().sin() * self.width;
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
        let pos = available_space.lerp_inside(self.pos.to_vec2());
        let w = available_space.width() * self.width;

        let h = available_space.height() * self.get_height();
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


pub struct BallValvePainter{
    valve_painter: ValvePainter,
    symbol_painter: SymbolPainter,
    line_painter: LinePainter1D
}

impl BallValvePainter {
    
    pub fn new(valve_painter: ValvePainter) -> Self {
        let valve_pos = valve_painter.pos;
        let symbol_pos = valve_painter.pos - Vec2::new(0.0, BALL_VALVE_SYMBOL_DISTANCE);
        Self {
            valve_painter,
            symbol_painter: SymbolPainter::new(symbol_pos, BALL_VALVE_SYMBOL_RADIUS, "M"),
            line_painter: LinePainter1D::new(vec![valve_pos, symbol_pos])
        }
    }

}

impl ComponentPainter for BallValvePainter {

    fn paint(&self, ui: &mut Ui) -> ResponseBounds {
        _ = self.line_painter.paint(ui);
        let rb_valve = self.valve_painter.paint(ui);
        let rb_symbol = self.symbol_painter.paint(ui);
        return rb_valve.combine(rb_symbol);
    }

}

pub struct TankValvePainter {
    valve_painter: ValvePainter,
    handle_width_painter: LinePainter1D,
    handle_height_painter: LinePainter1D
}

impl TankValvePainter {

    pub fn new(valve_painter: ValvePainter) -> Self {
        let valve_pos = valve_painter.pos;
        let handle_half_width = TANK_VALVE_HANDLE_WIDTH * valve_painter.width * 0.5;
        let handle_length = TANK_VALVE_HANDLE_LENGTH * valve_painter.get_height();
        Self {
            valve_painter,
            handle_height_painter: LinePainter1D::new(vec![
                valve_pos - Vec2::new(-handle_half_width, handle_length),
                valve_pos - Vec2::new(handle_half_width, handle_length)
            ]),
            handle_width_painter: LinePainter1D::new(vec![
                valve_pos,
                valve_pos - Vec2::new(0.0, handle_length)
            ])
        }
    }

}

impl ComponentPainter for TankValvePainter {

    fn paint(&self, ui: &mut Ui) -> ResponseBounds {
        _ = self.handle_height_painter.paint(ui);
        _ = self.handle_width_painter.paint(ui);
        let rb = self.valve_painter.paint(ui);
        return rb;
    }

}