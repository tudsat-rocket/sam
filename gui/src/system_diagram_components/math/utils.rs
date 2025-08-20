use core::f32;

use egui::{Pos2, Rect};
use nalgebra::Point2;

use super::conversions::to_pos;

pub fn gather<I, T, J>(values: I, indices: J) -> Vec<T>
where
    I: IntoIterator<Item = T> + Clone,
    T: Clone,
    J: IntoIterator<Item = usize>,
{
    let vec: Vec<T> = values.into_iter().collect();
    indices.into_iter().map(|i| vec[i].clone()).collect()
}

pub fn calculate_aabb(points: &Vec<Point2<f32>>) -> egui::Rect {
    points
        .iter()
        .fold(egui::Rect::from_min_max(Pos2::new(f32::MAX, f32::MAX), Pos2::new(f32::MIN, f32::MIN)), |r, p| {
            Rect::from_min_max(r.min.min(to_pos(*p)), r.max.max(to_pos(*p)))
        })
}
