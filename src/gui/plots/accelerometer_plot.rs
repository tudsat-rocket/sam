use std::{cell::RefCell, rc::Rc};

use crate::gui::plot::{PlotState, SharedPlotState};

use super::color_constants::*;

pub fn accelerometer_plot(shared_plot: &Rc<RefCell<SharedPlotState>>) -> PlotState {
    let accelerometer_plot = PlotState::new("Accelerometers", (Some(-10.0), Some(10.0)), shared_plot.clone())
        .line("Accel 2 (X) [m/s²]", R1, |vs| vs.accelerometer2.map(|a| a.x))
        .line("Accel 2 (Y) [m/s²]", G1, |vs| vs.accelerometer2.map(|a| a.y))
        .line("Accel 2 (Z) [m/s²]", B1, |vs| vs.accelerometer2.map(|a| a.z))
        .line("Accel 1 (X) [m/s²]", R, |vs| vs.accelerometer1.map(|a| a.x))
        .line("Accel 1 (Y) [m/s²]", G, |vs| vs.accelerometer1.map(|a| a.y))
        .line("Accel 1 (Z) [m/s²]", B, |vs| vs.accelerometer1.map(|a| a.z));
    accelerometer_plot
}
