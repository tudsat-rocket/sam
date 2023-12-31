use std::{cell::RefCell, rc::Rc};

use crate::gui::plot::{PlotState, SharedPlotState};

use super::color_constants::*;

pub fn altitude_plot(shared_plot: &Rc<RefCell<SharedPlotState>>) -> PlotState {
    let altitude_plot = PlotState::new("Altitude (ASL)", (None, None), shared_plot.clone())
        .line("Altitude (Ground) [m]", BR, |vs| vs.altitude_ground_asl)
        .line("Altitude (Baro) [m]", B1, |vs| vs.altitude_baro)
        .line("Altitude [m]", B, |vs| vs.altitude_asl)
        .line("Altitude (GPS) [m]", G, |vs| vs.altitude_gps_asl);
    altitude_plot
}
