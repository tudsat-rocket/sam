use nalgebra::{Affine2, Point2, Rotation2, Scale2, Translation2, Vector2};

pub struct Transform {
    rotation: Rotation2<f32>,
    scale: Scale2<f32>,
    translation: Translation2<f32>,
}

impl Transform {
    pub fn new(rotation: Rotation2<f32>, scale: Scale2<f32>, translation: Translation2<f32>) -> Self {
        Self {
            rotation,
            scale,
            translation,
        }
    }

    pub fn apply_to_point(&self, point: Point2<f32>) -> Point2<f32> {
        return self.translation
            * (self.scale * (self.rotation * (point - Vector2::new(0.5, 0.5)) + Vector2::new(0.5, 0.5)));
    }

    pub fn apply_to_points<I>(&self, points: I) -> Vec<Point2<f32>>
    where
        I: IntoIterator<Item = Point2<f32>>,
    {
        return points.into_iter().map(|p| self.apply_to_point(p)).collect();
    }

    pub fn to_affine2(&self) -> Affine2<f32> {
        let matrix = self.translation.to_homogeneous() * self.rotation.to_homogeneous() * self.scale.to_homogeneous();
        return Affine2::from_matrix_unchecked(matrix);
    }
}
