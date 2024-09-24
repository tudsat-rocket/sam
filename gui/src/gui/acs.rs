use egui::{Painter, Stroke, Pos2, Rect, Color32, epaint::{PathShape, RectShape}, Vec2, Shape, Rounding, RichText};

use shared_types::telemetry::ThrusterValveState;

use crate::data_source::DataSource;

use super::theme::ThemeColors;

const ACS_TANK_MAX_PRESSURE: f32 = 300.0;

pub struct AcsSystemDiagram {
    valve_status: Option<ThrusterValveState>,
    propellant_f: f32,
    tank_pressure: Option<f32>,
    regulator_pressure: Option<f32>,
}

impl AcsSystemDiagram {
    pub fn new(data_source: &dyn DataSource) -> Self {
        let tank_pressure = data_source.vehicle_states()
            .rev()
            .find_map(|(_t, vs)| vs.acs_tank_pressure);
        let regulator_pressure = data_source.vehicle_states()
            .rev()
            .find_map(|(_t, vs)| vs.acs_regulator_pressure);

        Self {
            valve_status: data_source.vehicle_states()
                .rev()
                .find_map(|(_t, vs)| vs.thruster_valve_state),
            propellant_f: tank_pressure.unwrap_or_default() / ACS_TANK_MAX_PRESSURE,
            tank_pressure,
            regulator_pressure,
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

        let mut path_fill: Vec<_> = path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)).collect();
        if self.propellant_f < (height - HEIGHT_BULKHEAD) / height && self.propellant_f > HEIGHT_BULKHEAD / height {
            path_fill.extend(&[
                pos + Vec2::new(-width/2.0, height/2.0 - height * self.propellant_f),
                pos + Vec2::new(width/2.0, height/2.0 - height * self.propellant_f),
            ]);
        } else {
            path_fill.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(-v.x, -height/2.0 + v.y)));
        }

        path_fill = path_fill.into_iter().filter(|v| (v.y - pos.y) > (height/2.0 - height * self.propellant_f - 1.0)).collect();

        painter.add(Shape::Path(PathShape::convex_polygon(path_complete, background_color, stroke)));
        painter.add(Shape::Path(PathShape::convex_polygon(path_fill, fill_color, stroke)));
    }

    fn draw_regulator(&self, painter: &Painter, pos: Pos2, w: f32, stroke: Stroke) {
        // TODO: draw the rest of the fucking regulator
        painter.rect(Rect::from_center_size(pos, Vec2::splat(w)), 0.0, Color32::TRANSPARENT, stroke);
    }

    fn draw_valve(&self, painter: &Painter, pos: Pos2, w: f32, open: bool, vertical: bool, stroke: Stroke) {
        if open {
            let shape = Shape::Rect(
                RectShape::new(
                    Rect::from_center_size(pos, Vec2::splat(w * 1.2)),
                    Rounding::same(0.0),
                    Color32::from_rgb(0xfe, 0x80, 0x19),
                    Stroke::NONE
                )
            );
            painter.add(shape);
        }

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
            Color32::TRANSPARENT
        } else {
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
        ui.columns(2, |columns| {
            for (i, label, value, nominal_min, nominal_max) in [
                (0, "Tank [bar]", self.tank_pressure, 250.0, 310.0),
                (1, "Regulator [bar]", self.regulator_pressure, 25.0, 40.0),
            ] {
                columns[i].horizontal(|ui| {
                    let value_string = value.map(|v| format!("{0:.1}", v)).unwrap_or("N/A".to_string());
                    let value_text = RichText::new(value_string).strong().monospace();
                    let nominal = value.map(|v| v > nominal_min && v < nominal_max).unwrap_or(false);

                    ui.set_width(ui.available_width());
                    ui.label(label);
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.add_space(5.0);
                        if nominal {
                            ui.label(value_text.color(Color32::from_rgb(0xb8, 0xbb, 0x26)));
                        } else {
                            ui.label(value_text.color(Color32::from_rgb(0xfa, 0xbd, 0x2f)));
                        }
                    });
                });
            }
        });

        ui.vertical(|ui| {
            let painter = ui.painter();
            let rect = ui.available_rect_before_wrap();
            let tank_w = ui.available_width() * 0.2;
            let tank_h = ui.available_height() * 0.6;
            let symbol_w = 30.0;

            let pos_tank = rect.lerp_inside(Vec2::new(0.15, 0.5));
            let pos_regulator = rect.lerp_inside(Vec2::new(0.5, 0.5));
            let pos_nozzle_valve_up = rect.lerp_inside(Vec2::new(0.80, 0.35));
            let pos_nozzle_tee = rect.lerp_inside(Vec2::new(0.80, 0.5));
            let pos_nozzle_valve_down = rect.lerp_inside(Vec2::new(0.80, 0.65));
            let pos_nozzle_up = rect.lerp_inside(Vec2::new(0.80, 0.15));
            let pos_nozzle_down = rect.lerp_inside(Vec2::new(0.80, 0.85));

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

            self.draw_tank(
                painter,
                pos_tank,
                tank_w,
                tank_h,
                stroke,
                theme.background_weak,
                Color32::from_rgb(0x45, 0x85, 0x88)
            );
            self.draw_regulator(painter, pos_regulator, symbol_w, stroke);
            self.draw_valve(painter, pos_nozzle_valve_up, symbol_w, decel, true, stroke);
            self.draw_valve(painter, pos_nozzle_valve_down, symbol_w, accel, true, stroke);
            self.draw_nozzle(painter, pos_nozzle_up, symbol_w, true, stroke);
            self.draw_nozzle(painter, pos_nozzle_down, symbol_w, false, stroke);

            painter.hline(pos_tank.x+tank_w/2.0..=pos_regulator.x-symbol_w/2.0, pos_tank.y, stroke);
            painter.hline(pos_regulator.x+symbol_w/2.0..=pos_nozzle_tee.x, pos_tank.y, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_down.y-symbol_w/2.0..=pos_nozzle_valve_up.y+symbol_w/2.0, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_down.y+symbol_w/2.0..=pos_nozzle_down.y-symbol_w/2.0, stroke);
            painter.vline(pos_nozzle_tee.x, pos_nozzle_valve_up.y-symbol_w/2.0..=pos_nozzle_up.y+symbol_w/2.0, stroke);
        }).response
    }
}
