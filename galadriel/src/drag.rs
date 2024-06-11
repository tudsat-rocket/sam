#[derive(Clone, Debug, PartialEq)]
pub enum DragCoefficient {
    Constant(f32),
    Variable(Vec<(f32, f32)>), // input is speed in m/s
}

pub fn mach_to_m_s(mach_number: f32) -> f32{
    mach_number * 343.0
}

impl Default for DragCoefficient {
    fn default() -> Self {
        DragCoefficient::Constant(1.0)
    }
}

impl DragCoefficient {
    pub fn at_velocity(&self, velocity: f32) -> f32 {
        match self {
            Self::Constant(coef) => *coef,
            Self::Variable(coefficients) => {
                // I assume velocity is given in m/s?
                self.interpolate_drag(coefficients, velocity)
            },
        }
    }

    fn interpolate_drag(&self, mach_drag_table: &[(f32, f32)], velocity: f32) -> f32{
        for i in 0..mach_drag_table.len() - 1{
            let (mach1, drag1) = mach_drag_table[i];
            let (mach2, drag2) = mach_drag_table[i+1];

            if self.mach_to_m_s(mach1) <= velocity && velocity <= self.mach_to_m_s(mach2){
                return self.interpolate(
                    self.mach_to_m_s(mach1),
                    self.mach_to_m_s(mach2),
                    drag1,
                    drag2,
                    velocity);
            }
        }

        // if here then velocity is not in the specified range
        let (min_mach, drag_min_mach) = mach_drag_table[0];
        let (max_mach, drag_max_mach) = mach_drag_table[mach_drag_table.len() - 1];

        if velocity <= self.mach_to_m_s(min_mach) {
            return drag_min_mach;
        }

        if velocity >= self.mach_to_m_s(max_mach) {
            return drag_max_mach;
        }

        // something may be wrong
        panic!("Velocity for drag interpolation is out of interpolation range or not a number");
    }

    // calculate mach to m/s by scaling with 343
    fn mach_to_m_s(&self, mach_number: f32) -> f32{
        mach_number * 343.0
    }

    // create linear function f(x) between given set of points (x1, y1) and (x2,y2) and return y=f(x)
    fn interpolate(&self, x1:f32, x2:f32, y1: f32, y2: f32, x: f32) -> f32{
        let m: f32 = (y2 - y1) / (x2 - x1);
        let b: f32 = y1 - m * x1;
        m * x + b
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
            },
            DragCoefficient::Variable(coefficients) => {
                let plot_response = egui_plot::Plot::new("drag_coefficient_plot")
                    .set_margin_fraction(egui::Vec2::new(0.1, 0.3))
                    .allow_scroll(false)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .allow_boxed_zoom(false)
                    .show_axes(false) // these are really big
                    // plot m/s über draf coefficient
                    .show(ui, |plot_ui| {
                        let points: Vec<_> = coefficients.iter()
                            .map(|(speed, coef)| [*speed as f64, *coef as f64])
                            .collect();
                        plot_ui.line(egui_plot::Line::new(points));
                    });

                // Create a dummy response if no actual response is needed
                plot_response.response
            }
        }
    }
}
