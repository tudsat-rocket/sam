use std::{cell::RefCell, rc::Rc};

use crate::gui::plot::{PlotState, SharedPlotState};

use super::color_constants::*;

pub fn orientation_plot(shared_plot: &Rc<RefCell<SharedPlotState>>) -> PlotState {
    let orientation_plot = PlotState::new("Orientation", (Some(-180.0), Some(540.0)), shared_plot.clone())
        .line("Roll (Z) [°]", B, |vs| vs.euler_angles.map(|a| a.z))
        .line("Pitch (X) [°]", R, |vs| vs.euler_angles.map(|a| a.x))
        .line("Yaw (Y) [°]", G, |vs| vs.euler_angles.map(|a| a.y))
        .line("Angle of Attack [°]", O, |vs| vs.angle_of_attack)
        .line("Roll (True) (Z) [°]", B1, |vs| vs.true_euler_angles.map(|a| a.z))
        .line("Pitch (True) (X) [°]", R1, |vs| vs.true_euler_angles.map(|a| a.x))
        .line("Yaw (True) (Y) [°]", G1, |vs| vs.true_euler_angles.map(|a| a.y))
        .line("Angle of Attack (True) [°]", O1, |vs| vs.true_angle_of_attack);
    orientation_plot
}
