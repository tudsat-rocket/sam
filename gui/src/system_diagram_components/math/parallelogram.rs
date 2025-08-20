use egui::{Rect, Vec2};
use nalgebra::{Affine2, Point2, Vector2};

use super::conversions::{to_point, to_pos, to_vector};

pub const CENTERED_UNIT_RECT: Parallelogram = Parallelogram {
    center: Point2::new(0f32, 0f32),
    axes: [Vector2::new(-0.5, 0.5), Vector2::new(0.5, 0.5)],
};

pub struct Parallelogram {
    ///The center of the parallelogram
    center: Point2<f32>,
    //Vectors from the center to two non-oppsite corners
    axes: [Vector2<f32>; 2],
}

impl Parallelogram {
    pub fn new(center: Point2<f32>, axes: [Vector2<f32>; 2]) -> Self {
        Self { center, axes }
    }

    pub fn transform(self, transform: &Affine2<f32>) -> Self {
        Self {
            center: transform * self.center,
            axes: [transform * self.axes[0], transform * self.axes[1]],
        }
    }

    pub fn axis_aligned_bounding_box(&self) -> Rect {
        Rect::from_center_size(
            to_pos(self.center),
            Vec2::new(
                2f32 * self.axes[0].x.abs().max(self.axes[1].x.abs()),
                2f32 * self.axes[0].y.abs().max(self.axes[1].y.abs()),
            ),
        )
    }
}

impl From<Rect> for Parallelogram {
    fn from(rect: Rect) -> Self {
        Parallelogram {
            center: to_point(rect.center()),
            axes: [
                to_vector(rect.left_top() - rect.center()),
                to_vector(rect.right_top() - rect.center()),
            ],
        }
    }
}
