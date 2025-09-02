use nalgebra::Point2;

use crate::system_diagram_components::math::ellipse::Circle;

pub struct Triangle {
    points: [Point2<f32>; 3],
}

impl Triangle {
    pub fn new(points: [Point2<f32>; 3]) -> Self {
        Self { points }
    }

    // pub fn transform(self, transform: &Affine2<f32>) -> Ellipse {
    //     Ellipse::from(self).transform(transform)
    // }

    pub fn area(&self) -> f32 {
        return 0.5
            * f32::abs(
                (self.points[0].x - self.points[2].x) * (self.points[1].y - self.points[0].y)
                    - (self.points[0].x - self.points[1].x) * (self.points[2].y - self.points[0].y),
            );
    }

    ///Returns [|BC|, |CA|, |AB|] for the triangle [A, B, C]
    pub fn edge_lengths(&self) -> [f32; 3] {
        return [
            (self.points[2] - self.points[1]).norm(),
            (self.points[0] - self.points[2]).norm(),
            (self.points[1] - self.points[0]).norm(),
        ];
    }

    pub fn incircle(&self) -> Circle {
        let edges = self.edge_lengths();
        let perimeter = edges[0] + edges[1] + edges[2];
        return Circle::new(
            (edges[0] * self.points[0] + (edges[1] * self.points[1]).coords + (edges[2] * self.points[2]).coords)
                / perimeter,
            2.0 * self.area() / perimeter,
        );
    }
}
