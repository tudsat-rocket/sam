use std::{cell::RefCell, rc::Rc};

use crate::gui::plot::{PlotState, SharedPlotState};

use super::color_constants::*;

pub fn vertical_speed_plot(shared_plot: &Rc<RefCell<SharedPlotState>>) -> PlotState {
    let vertical_speed_plot = PlotState::new("Vert. Speed & Accel.", (None, None), shared_plot.clone())
        .line("Vertical Accel [m/s²]", O1, |vs| vs.vertical_accel)
        .line("Vertical Accel (Filt.) [m/s²]", O, |vs| vs.vertical_accel_filtered)
        .line("Vario [m/s]", B, |vs| vs.vertical_speed)
        .line("True Vertical Accel [m/s²]", G, |vs| vs.true_vertical_accel)
        .line("True Vario [m/s]", B1, |vs| vs.true_vertical_speed);
    vertical_speed_plot
}
