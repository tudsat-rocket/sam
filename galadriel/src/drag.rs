#[derive(Clone, Debug, PartialEq)]
pub enum DragCoefficient {
    Constant(f32),
    // TODO: adjustable/speed-dependent drag coefficients
    Variable, // input is speed in m/s
}

// points from which drag coefficient is interpolated. 
const MACH_DRAG_TABLE: [(f32, f32); 8] = [
    // (mach_number, drag_coefficient)
    (0.006, 0.622),
    (0.018, 0.535),
    (0.033, 0.497),
    (0.064, 0.46),
    (0.14, 0.43),
    (0.512, 0.461),
    (0.88, 0.526),
    (0.941, 0.545),
];

pub fn mach_to_m_s(mach_number: f32) -> f32{
    mach_number * 343.0
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
            Self::Variable => {
                // I assume velocity is given in m/s?
                self.interpolate_drag(_velocity)
            },
        }
    }

    fn interpolate_drag(&self, velocity: f32) -> f32{
        for i in 0..MACH_DRAG_TABLE.len() - 1{
            let (mach1, drag1) = MACH_DRAG_TABLE[i];
            let (mach2, drag2) = MACH_DRAG_TABLE[i+1];
            
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
        let (min_mach, drag_min_mach) = MACH_DRAG_TABLE[0];
        let (max_mach, drag_max_mach) = MACH_DRAG_TABLE[MACH_DRAG_TABLE.len() - 1];
        
        if velocity <= self.mach_to_m_s(min_mach) {
            return drag_min_mach;
        }

        if velocity >= self.mach_to_m_s(max_mach) {
            return drag_max_mach;
        }

        // something may be wrong
        panic!("Velocity out of interpolation range");
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
            DragCoefficient::Variable => { ui.label("placeholder") }
        }
    }
}
