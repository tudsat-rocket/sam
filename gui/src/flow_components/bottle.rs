use egui::{epaint::PathShape, Color32, Pos2, Rect, Shape, Stroke, Vec2};

use crate::{
    flow_components::{
        constants::STROKE_WITH,
        flow_component::{ResponseBounds, Value},
    },
    utils::{
        mesh::{create_mesh, ColoredTexture, TextureKey},
        theme::ThemeColors,
    },
};

use super::{
    constants::{
        BOTTLE_BULKHEAD_HEIGHT, BOTTLE_BULKHEAD_STEPS, BOTTLE_CAP_BULKHEAD_HEIGHT, BOTTLE_CAP_BULKHEAD_STEPS,
        BOTTLE_CAP_TOTAL_HEIGHT, BOTTLE_CAP_WIDTH,
    },
    flow_component::{ComponentPainter, Fluid},
};

pub struct BottlePainter {
    pos: Pos2,
    width: f32,
    height: f32,
    max_pressure: f32,
    fluid: Fluid,
}

impl BottlePainter {
    pub fn new(pos: Pos2, width: f32, height: f32, max_pressure: f32, fluid: Fluid) -> Self {
        Self {
            pos,
            width,
            height,
            max_pressure,
            fluid,
        }
    }
}

impl ComponentPainter for BottlePainter {
    fn paint(&self, ui: &mut egui::Ui) -> super::flow_component::ResponseBounds {
        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let pos = available_space.lerp_inside(self.pos.to_vec2());
        let width = available_space.width() * self.width;
        let height = available_space.height() * self.height;

        let stroke = Stroke {
            width: STROKE_WITH,
            color: theme.foreground_weak,
        };

        let fill_percentage = match self.fluid.pressure.value.clone().unwrap_or(Value::F32(self.max_pressure)) {
            Value::F32(v) => v,
            _ => -1.0,
        } / self.max_pressure;

        let mut path_bulkhead: Vec<_> = (0..=BOTTLE_BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BOTTLE_BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Vec2::new(-width / 2.0 * r.cos(), BOTTLE_BULKHEAD_HEIGHT * (1.0 - r.sin())))
            .collect();
        path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        let mut path_complete: Vec<_> = path_bulkhead
            .iter()
            .map(|v| pos + Vec2::new(v.x, -height / 2.0 + v.y + BOTTLE_CAP_TOTAL_HEIGHT))
            .collect();
        path_complete.push(pos + Vec2::new(width, height) / 2.0);
        path_complete.push(pos + Vec2::new(-width, height) / 2.0);
        //path_complete.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)));

        let mut path_fill: Vec<Pos2> = path_complete
            .iter()
            .filter(|v| (v.y - pos.y) > (height / 2.0 - height * fill_percentage - 1.0))
            .cloned()
            .collect();
        if fill_percentage < (height - BOTTLE_BULKHEAD_HEIGHT) / height
        /*&& fill_percentage > BOTTLE_BULKHEAD_HEIGHT / height*/
        {
            path_fill.extend(&[
                pos + Vec2::new(-width / 2.0, height / 2.0 - height * fill_percentage),
                pos + Vec2::new(width / 2.0, height / 2.0 - height * fill_percentage),
            ]);
        }

        let bottle_cap_width = width * BOTTLE_CAP_WIDTH;
        let mut path_cap_bulkhead: Vec<_> = (0..=BOTTLE_CAP_BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BOTTLE_CAP_BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Vec2::new(-bottle_cap_width / 2.0 * r.cos(), BOTTLE_CAP_BULKHEAD_HEIGHT * (1.0 - r.sin())))
            .collect();
        path_cap_bulkhead.extend(path_cap_bulkhead.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        let mut path_cap_complete: Vec<_> =
            path_cap_bulkhead.iter().map(|v| pos + Vec2::new(v.x, -height / 2.0 + v.y)).collect();
        path_cap_complete
            .push(pos + Vec2::new(bottle_cap_width, -height) / 2.0 + Vec2::new(0.0, BOTTLE_CAP_TOTAL_HEIGHT));
        path_cap_complete
            .push(pos + Vec2::new(-bottle_cap_width, -height) / 2.0 + Vec2::new(0.0, BOTTLE_CAP_TOTAL_HEIGHT));

        // let mut path_fill: Vec<_> = path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)).collect();
        // if fill_percentage < (height - BOTTLE_BULKHEAD_HEIGHT) / height && fill_percentage > BOTTLE_BULKHEAD_HEIGHT / height {
        //     path_fill.extend(&[
        //         pos + Vec2::new(-width/2.0, height/2.0 - height * fill_percentage),
        //         pos + Vec2::new(width/2.0, height/2.0 - height * fill_percentage),
        //     ]);
        // } else {
        //     path_fill.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(-v.x, -height/2.0 + v.y)));
        // }

        // path_fill = path_fill.into_iter().filter(|v| (v.y - pos.y) > (height/2.0 - height * fill_percentage - 1.0)).collect();

        let fluid_texture = ColoredTexture::new(TextureKey::PatternCrosshatch, self.fluid.fluid_type.color);
        let fluid_mesh = create_mesh(&path_fill, fluid_texture);

        // 3. Paint the mesh
        ui.painter().add(fluid_mesh);

        let cap_shape = Shape::Path(PathShape::convex_polygon(path_cap_complete, Color32::TRANSPARENT, stroke));
        let full_shape = Shape::Path(PathShape::convex_polygon(path_complete, Color32::TRANSPARENT, stroke));
        //let bb = full_shape.visual_bounding_rect();
        let bb = Rect::from_center_size(pos, Vec2::new(width, height));

        painter.add(cap_shape);
        painter.add(full_shape);
        //painter.add(Shape::Path(PathShape::convex_polygon(path_fill, fill_color, stroke)));

        //Currently does not use precise hit detection as this would require a second mesh or some uv trickery
        return ResponseBounds::new(
            bb, None, /*Some(Box::new(move |point| is_point_in_mesh(point, mesh.clone())))*/
        );
    }
}
