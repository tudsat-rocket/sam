use nalgebra::{Affine2, Point2, Vector2};

pub struct Circle {
    center: Point2<f32>,
    radius: f32,
}

impl Circle {

    pub fn new(center: Point2<f32>, radius: f32) -> Self {
        Self{ center, radius }
    }

    pub fn transform(self, transform: &Affine2<f32>) -> Ellipse {
        Ellipse::from(self).transform(transform)
    }

    pub fn center(&self) -> Point2<f32> {
        return self.center;
    }

    pub fn radius(&self) -> f32 {
        return self.radius;
    }

}

pub struct Ellipse {
    center: Point2<f32>,
    axes: [Vector2<f32>; 2],
}

impl Ellipse {
    
    pub fn new(center: Point2<f32>, axes: [Vector2<f32>; 2]) -> Self {
        Self { center, axes }
    }

    pub fn transform(self, transform: &Affine2<f32>) -> Self {
        Self { center: transform * self.center, axes: [transform * self.axes[0], transform * self.axes[1]] }
    }

    pub fn incircle(&self) -> Circle {
        Circle { center: self.center, radius: self.axes[0].norm().min(self.axes[1].norm()) }
    }

}

impl From<Circle> for Ellipse {

    fn from(circle: Circle) -> Self {
        Ellipse { center: circle.center, axes: [Vector2::new(circle.radius, 0f32), Vector2::new(0f32, circle.radius)] }
    }

}