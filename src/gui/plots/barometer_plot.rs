use std::{cell::RefCell, rc::Rc};

use crate::gui::plot::{PlotState, SharedPlotState};

use super::color_constants::*;

pub fn barometer_plot(shared_plot: &Rc<RefCell<SharedPlotState>>) -> PlotState {
    let barometer_plot =
        PlotState::new("Pressures", (None, None), shared_plot.clone())
            .line("Barometer [bar]", C, |vs| vs.pressure_baro.map(|p| p / 1000.0));
    barometer_plot
}
