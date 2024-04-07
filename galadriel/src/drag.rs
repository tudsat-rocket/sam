#[derive(Clone, Debug, PartialEq)]
pub enum DragCoefficient {
    Constant(f32),
    // TODO: adjustable/speed-dependent drag coefficients
}

impl Default for DragCoefficient {
    fn default() -> Self {
        DragCoefficient::Constant(1.0)
    }
}

impl DragCoefficient {
    pub fn at_velocity(&self, _velocity: f32) -> f32 {
        match self {
            Self::Constant(coef) => *coef,
        }
    }

    pub fn pressure(&self, velocity: f32, density: f32) -> f32 {
        0.5 * density * velocity.powi(2) * self.at_velocity(velocity)
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut DragCoefficient {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        match self {
            DragCoefficient::Constant(coef) => {
                ui.add(
                    egui::DragValue::new(coef)
                        .speed(0.001)
                        .clamp_range(0.001..=10.0),
                )
            }
        }
    }
}
