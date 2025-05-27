use egui::{Pos2, Vec2};
use nalgebra::{Point2, Vector2};

pub fn to_vector(vec: Vec2) -> Vector2<f32> {
    return Vector2::new(vec.x, vec.y);
}

pub fn to_vec(vector: Vector2<f32>) -> Vec2 {
    return Vec2::new(vector.x, vector.y);
}

pub fn to_point(pos: Pos2) -> Point2<f32> {
    return Point2::new(pos.x, pos.y);
}

pub fn to_pos(point: Point2<f32>) -> Pos2 {
    return Pos2::new(point.x, point.y);
}

// impl From<Pos2> for Point2<f32> {

//     fn from(pos2: Pos2) -> Self {
//         Point2::new(pos2.x, pos2.y)
//     }
    
// }