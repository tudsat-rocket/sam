use egui::{Painter, Stroke, Pos2, Rect, Color32, epaint::PathShape, Vec2, Shape};

use shared_types::telemetry::ThrusterValveState;

use crate::data_source::DataSource;

use super::theme::ThemeColors;

pub struct AcsSystemDiagram {
    valve_status: Option<ThrusterValveState>,
    propellant_f: f32,
}

impl AcsSystemDiagram {
    pub fn new(data_source: &dyn DataSource) -> Self {
        Self {
            valve_status: data_source.vehicle_states()
                .rev()
                .find_map(|(_t, vs)| vs.thruster_valve_state),
            propellant_f: data_source.vehicle_states()
                .last()
                .map(|(_t, vs)| vs.sim.as_ref().and_then(|sim| sim.thruster_propellant_mass))
                .flatten()
                .unwrap_or_default() / 0.5,
        }
    }

    fn draw_tank(&self, painter: &Painter, pos: Pos2, width: f32, height: f32, stroke: Stroke, background_color: Color32, fill_color: Color32) {
        const HEIGHT_BULKHEAD: f32 = 15.0;
        const BULKHEAD_STEPS: usize = 100;

        let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Vec2::new(-width/2.0 * r.cos(), HEIGHT_BULKHEAD * (1.0 - r.sin())))
            .collect();
        path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        let mut path_complete: Vec<_> = path_bulkhead.iter().map(|v| pos + Vec2::new(v.x, -height/2.0 + v.y)).collect();
        path_complete.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)));

        let fill_height = height - 2.0*HEIGHT_BULKHEAD;
        let mut path_fill: Vec<_> = vec![
            pos + Vec2::new(-width/2.0, fill_height/2.0 - fill_height * self.propellant_f),
            pos + Vec2::new(width/2.0, fill_height/2.0 - fill_height * self.propellant_f),
        ];
        path_fill.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)));

        painter.add(Shape::Path(PathShape::convex_polygon(path_complete, background_color, stroke)));
        painter.add(Shape::Path(PathShape::convex_polygon(path_fill, fill_color, stroke)));
    }

    fn draw_regulator(&self, painter: &Painter, pos: Pos2, w: f32, stroke: Stroke) {
        // TODO: draw the rest of the fucking regulator
        painter.rect(Rect::from_center_size(pos, Vec2::splat(w)), 0.0, Color32::TRANSPARENT, stroke);
    }

    fn draw_valve(&self, painter: &Painter, pos: Pos2, w: f32, open: bool, vertical: bool, stroke: Stroke) {
        let h = 30f32.to_radians().sin() * w;
        let (path_a, path_b) = if vertical {
            (
                vec![pos, pos - Vec2::new(h/2.0, w/2.0), pos - Vec2::new(-h/2.0, w/2.0)],
                vec![pos, pos + Vec2::new(h/2.0, w/2.0), pos + Vec2::new(-h/2.0, w/2.0)]
            )
        } else {
            (
                vec![pos, pos - Vec2::new(w/2.0, h/2.0), pos - Vec2::new(w/2.0, -h/2.0)],
                vec![pos, pos + Vec2::new(w/2.0, h/2.0), pos + Vec2::new(w/2.0, -h/2.0)]
            )
        };

        let c = if open {
            //Color32::from_rgb(0xb8, 0xbb, 0x26)
            //Color32::from_rgb(0xfa, 0xbd, 0x2f)
            Color32::TRANSPARENT
        } else {
            //Color32::TRANSPARENT
            stroke.color
        };

        painter.add(Shape::Path(PathShape::convex_polygon(path_a, c, stroke)));
        painter.add(Shape::Path(PathShape::convex_polygon(path_b, c, stroke)));
    }

    fn draw_nozzle(&self, painter: &Painter, pos: Pos2, l: f32, flip: bool, stroke: Stroke) {
        let throat_w = l * 0.2;
        let mut path_local = vec![
            Vec2::new(0.0, -l/2.0),
            Vec2::new(l/5.0, -l/2.0),
            Vec2::new(l/5.0, -l/4.0),
            Vec2::new(throat_w/2.0, -throat_w/2.0),
            Vec2::new(throat_w/2.0, throat_w/2.0),
            Vec2::new(l/4.0, l/2.0),
        ];
        path_local.extend(path_local.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        if flip {
            path_local = path_local.into_iter().map(|v| Vec2::new(v.x, v.y * -1.0)).collect();
        }

        let path = path_local.into_iter().map(|v| pos + v).collect();
        painter.add(Shape::Path(PathShape::convex_polygon(path, Color32::TRANSPARENT, stroke)));
    }
}

impl egui::Widget for AcsSystemDiagram {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            let painter = ui.painter();
            let rect = ui.available_rect_before_wrap();

            let tank_w = ui.available_width() * 0.2;
            let tank_h = ui.available_height() * 0.6;
            let symbol_w = 35.0;

            let pos_tank = rect.lerp_inside(Vec2::new(0.35, 0.5));
            let pos_tank_valve = rect.lerp_inside(Vec2::new(0.15, 0.5));
            let pos_tank_port = rect.lerp_inside(Vec2::new(0.05, 0.5));
            let pos_regulator = rect.lerp_inside(Vec2::new(0.65, 0.5));
            let pos_nozzle_valve_up = rect.lerp_inside(Vec2::new(0.85, 0.35));
            let pos_nozzle_tee = rect.lerp_inside(Vec2::new(0.85, 0.5));
            let pos_nozzle_valve_down = rect.lerp_inside(Vec2::new(0.85, 0.65));
            let pos_nozzle_up = rect.lerp_inside(Vec2::new(0.85, 0.15));
            let pos_nozzle_down = rect.lerp_inside(Vec2::new(0.85, 0.85));

            let theme = ThemeColors::new(ui.ctx());
            let stroke = Stroke {
                width: 1.8,
                color: theme.foreground_weak
            };

            let (accel, decel) = match self.valve_status.unwrap_or_default() {
                ThrusterValveState::Closed => (false, false),
                ThrusterValveState::OpenAccel => (true, false),
                ThrusterValveState::OpenDecel => (false, true),
                ThrusterValveState::OpenBoth => (true, true),
            };

            self.draw_tank(painter, pos_tank, tank_w, tank_h, stroke, theme.background_weak, Color32::from_rgb(0x45, 0x85, 0x88));
            self.draw_regulator(painter, pos_regulator, symbol_w, stroke);
            self.draw_valve(painter, pos_tank_valve, symbol_w, false, false, stroke);
            self.draw_valve(painter, pos_nozzle_valve_up, symbol_w, decel, true, stroke);
            self.draw_valve(painter, pos_nozzle_valve_down, symbol_w, accel, true, stroke);
            self.draw_nozzle(painter, pos_nozzle_up, symbol_w, true, stroke);
            self.draw_nozzle(painter, pos_nozzle_down, symbol_w, false, stroke);

            painter.hline(pos_tank_port.x..=pos_tank_valve.x-symbol_w/2.0, pos_tank_port.y, stroke);
            painter.hline(pos_tank_valve.x+symbol_w/2.0..=pos_tank.x-tank_w/2.0, pos_tank.y, stroke);
            painter.hline(pos_tank.x+tank_w/2.0..=pos_regulator.x-symbol_w/2.0, pos_tank.y, stroke);
            painter.hline(pos_regulator.x+symbol_w/2.0..=pos_nozzle_tee.x, pos_tank.y, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_down.y-symbol_w/2.0..=pos_nozzle_valve_up.y+symbol_w/2.0, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_down.y+symbol_w/2.0..=pos_nozzle_down.y-symbol_w/2.0, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_up.y-symbol_w/2.0..=pos_nozzle_up.y+symbol_w/2.0, stroke);
        }).response
    }
}
