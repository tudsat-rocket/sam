use egui::{epaint::PathShape, Color32, Mesh, Pos2, Shape, Stroke, Ui, Vec2};

use crate::{flow_components::flow_component::{Justification, Value, ResponseBounds, JUSTIFICATION_MEASRURED_PATTERN, JUSTIFICATION_NONE_PATTERN, JUSTIFICATION_REASONED_PATTERN}, utils::{mesh::is_point_in_mesh, theme::ThemeColors}};

use super::flow_component::{ComponentPainter, Fluid};

pub struct TankPainter {
    pos: Vec2,
    width: f32,
    height: f32,
    stroke_width: f32,
    max_pressure: f32,
    fluid: Fluid
    // fill_color: Color32,
    // fill_percentage: f32
}

impl TankPainter {
    pub fn new(pos: Vec2, width: f32, height: f32, stroke_width: f32, max_pressure: f32, fluid: Fluid) -> Self {
        Self {
            pos,
            width,
            height,
            stroke_width,
            max_pressure,
            fluid
        }
    }
}

impl ComponentPainter for TankPainter {

    fn paint(&self, ui: &mut Ui) -> ResponseBounds{
        const HEIGHT_BULKHEAD: f32 = 15.0;
        const BULKHEAD_STEPS: usize = 100;

        let painter = ui.painter();
        let theme = ThemeColors::new(ui.ctx());
        let available_space = ui.available_rect_before_wrap();

        let pos = available_space.lerp_inside(self.pos);
        let width = available_space.width() * self.width;
        let height = available_space.height() * self.height;

        let stroke = Stroke {
            width: self.stroke_width,
            color: theme.foreground_weak
        };
        //let background_color = theme.background_weak;
        //let fluid_pressure = self.fluid.pressure;//.clone().unwrap_or(JustifiedValue { value: self.max_pressure, justification: Justification::None });
        let fill_percentage = match self.fluid.pressure.value.clone().unwrap_or(Value::F32(self.max_pressure)) {Value::F32(v) => v, _ => -1.0} / self.max_pressure;

        let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Vec2::new(-width/2.0 * r.cos(), HEIGHT_BULKHEAD * (1.0 - r.sin())))
            .collect();
        path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        let mut path_complete: Vec<_> = path_bulkhead.iter().map(|v| pos + Vec2::new(v.x, -height/2.0 + v.y)).collect();
        path_complete.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)));

        let mut path_fill: Vec<_> = path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)).collect();
        if fill_percentage < (height - HEIGHT_BULKHEAD) / height && fill_percentage > HEIGHT_BULKHEAD / height {
            path_fill.extend(&[
                pos + Vec2::new(-width/2.0, height/2.0 - height * fill_percentage),
                pos + Vec2::new(width/2.0, height/2.0 - height * fill_percentage),
            ]);
        } else {
            path_fill.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(-v.x, -height/2.0 + v.y)));
        }

        path_fill = path_fill.into_iter().filter(|v| (v.y - pos.y) > (height/2.0 - height * fill_percentage - 1.0)).collect();

        let image_data = match self.fluid.pressure.justification {
            Justification::Measured(_)  => JUSTIFICATION_MEASRURED_PATTERN,
            Justification::Reasoned     => JUSTIFICATION_REASONED_PATTERN,
            Justification::Process      => JUSTIFICATION_MEASRURED_PATTERN,
            Justification::None         => JUSTIFICATION_NONE_PATTERN,
        };

        // Decode it (e.g., using `image` crate)
        let image = image::load_from_memory(image_data).unwrap().to_rgba8();
        let (width, height) = image.dimensions();
        let pixels: Vec<Color32> = image
            .pixels()
            .map(|p| Color32::from_rgba_premultiplied(p[0], p[1], p[2], p[3]))
            .collect();
        
        // Create texture
        let texture = egui::ColorImage {
            size: [width as usize, height as usize],
            pixels,
        };

        //TODO: Where exactly should this be called?
        let tex_id = ui.ctx().load_texture("lined_pattern", texture, egui::TextureOptions::LINEAR_REPEAT);

        // 1. Triangulate the polygon (you'll need a crate like `earcutr` or `lyon`)
        let indices = earcutr::earcut(
            &path_fill.iter().map(|p| vec![p.x as f64, p.y as f64]).flatten().collect::<Vec<_>>(),
            &[],
            2,
        );

        // 2. Build a Mesh
        let mut mesh = Mesh {
            texture_id: tex_id.id(),
            ..Default::default()
        };

        // Find bounding box for UV mapping
        let min = path_fill.iter().fold(Pos2::new(f32::MAX, f32::MAX), |a, b| a.min(*b));
        let max = path_fill.iter().fold(Pos2::new(f32::MIN, f32::MIN), |a, b| a.max(*b));
        let size = max - min;

        // Add vertices with UVs
        for p in &path_fill {
        let uv = Vec2::new((p.x - min.x) / size.x, (p.y - min.y) / size.y) * size / 32.0;
        mesh.vertices.push(egui::epaint::Vertex {
            pos: *p,
            uv: uv.to_pos2(),
            color: self.fluid.fluid_type.color,
        });
        }

        // Add indices
        mesh.indices.extend(indices.unwrap().iter().map(|i| *i as u32));

        // 3. Paint the mesh
        ui.painter().add(mesh.clone());

        let full_shape = Shape::Path(PathShape::convex_polygon(path_complete, Color32::TRANSPARENT, stroke));
        let bb = full_shape.visual_bounding_rect();

        painter.add(full_shape);
        //painter.add(Shape::Path(PathShape::convex_polygon(path_fill, fill_color, stroke)));

        return ResponseBounds::new(bb, Some(Box::new(move |point| is_point_in_mesh(point, mesh.clone()))));
    }

}