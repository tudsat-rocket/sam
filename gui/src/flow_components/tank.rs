use egui::{epaint::PathShape, Color32, Painter, Rect, Shape, Stroke, Vec2};

use crate::utils::theme::ThemeColors;

pub struct PropellantTankVisData {
    pos: Vec2,
    width: f32,
    height: f32,
    stroke_width: f32,
    fill_color: Color32,
}

pub struct Tank {
    fill_percentage: f32,
    vis_data: PropellantTankVisData
}

impl PropellantTankVisData {
    pub fn new(pos: Vec2, width: f32, height: f32, stroke_width: f32, fill_color: Color32) -> Self {
        Self{
            pos, width, height, stroke_width, fill_color
        }
    }
}

impl Tank {
    pub const fn new(fill_percentage: f32, vis_data: PropellantTankVisData) -> Self {
        Self {
            fill_percentage,
            vis_data
        }
    }

    pub fn draw(&self, painter: &Painter, theme: &ThemeColors, available_space: &Rect) {
        const HEIGHT_BULKHEAD: f32 = 15.0;
        const BULKHEAD_STEPS: usize = 100;

        let pos = available_space.lerp_inside(self.vis_data.pos);
        let width = available_space.width() * self.vis_data.width;
        let height = available_space.height() * self.vis_data.height;
        let stroke = Stroke {
            width: self.vis_data.stroke_width,
            color: theme.foreground_weak
        };
        let background_color = theme.background_weak;
        let &fill_color = &self.vis_data.fill_color;

        let mut path_bulkhead: Vec<_> = (0..=BULKHEAD_STEPS)
            .map(|i| (90.0 * (i as f32) / (BULKHEAD_STEPS as f32)).to_radians())
            .map(|r| Vec2::new(-width/2.0 * r.cos(), HEIGHT_BULKHEAD * (1.0 - r.sin())))
            .collect();
        path_bulkhead.extend(path_bulkhead.clone().into_iter().rev().map(|v| Vec2::new(-v.x, v.y)));

        let mut path_complete: Vec<_> = path_bulkhead.iter().map(|v| pos + Vec2::new(v.x, -height/2.0 + v.y)).collect();
        path_complete.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)));

        let mut path_fill: Vec<_> = path_bulkhead.iter().rev().map(|v| pos + Vec2::new(v.x, height/2.0 - v.y)).collect();
        if self.fill_percentage < (height - HEIGHT_BULKHEAD) / height && self.fill_percentage > HEIGHT_BULKHEAD / height {
            path_fill.extend(&[
                pos + Vec2::new(-width/2.0, height/2.0 - height * self.fill_percentage),
                pos + Vec2::new(width/2.0, height/2.0 - height * self.fill_percentage),
            ]);
        } else {
            path_fill.extend(path_bulkhead.iter().rev().map(|v| pos + Vec2::new(-v.x, -height/2.0 + v.y)));
        }

        path_fill = path_fill.into_iter().filter(|v| (v.y - pos.y) > (height/2.0 - height * self.fill_percentage - 1.0)).collect();

        painter.add(Shape::Path(PathShape::convex_polygon(path_complete, background_color, stroke)));
        painter.add(Shape::Path(PathShape::convex_polygon(path_fill, fill_color, stroke)));
    }
}