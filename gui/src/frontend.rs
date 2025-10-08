use crate::{
    frontend::{metric_monitor::MetricMonitor, popup_manager::PopupManager},
    settings::AppSettings,
    widgets::{
        map::MapState,
        plot::SharedPlotState,
        time_line::{HyacinthAnomalousState, HyacinthNominalState},
    },
};

pub mod metric_monitor;
pub mod popup_manager;
//TODO Hans: Probably should not be here
pub mod constraints;

struct HyacinthState {
    nominal: HyacinthNominalState,
    anomalous: Option<HyacinthAnomalousState>,
}

pub struct Frontend {
    popup_manager: PopupManager,
    metric_monitor: MetricMonitor,
    map: MapState,
    shared_plot: SharedPlotState,
}

impl Frontend {
    pub fn new(ctx: &egui::Context, settings: &AppSettings) -> Self {
        let mapbox_token = (!settings.mapbox_access_token.is_empty()).then_some(settings.mapbox_access_token.clone());
        return Self {
            popup_manager: Default::default(),
            metric_monitor: Default::default(),
            map: MapState::new(ctx, mapbox_token),
            shared_plot: SharedPlotState::new(),
        };
    }

    pub fn popup_manager(&self) -> &PopupManager {
        return &self.popup_manager;
    }

    pub fn popup_manager_mut(&mut self) -> &mut PopupManager {
        return &mut self.popup_manager;
    }

    pub fn metric_monitor(&self) -> &MetricMonitor {
        return &self.metric_monitor;
    }

    pub fn metric_monitor_mut(&mut self) -> &mut MetricMonitor {
        return &mut self.metric_monitor;
    }

    pub fn map(&self) -> &MapState {
        return &self.map;
    }

    pub fn map_mut(&mut self) -> &mut MapState {
        return &mut self.map;
    }

    pub fn shared_plot(&self) -> &SharedPlotState {
        return &self.shared_plot;
    }

    pub fn shared_plot_mut(&mut self) -> &mut SharedPlotState {
        return &mut self.shared_plot;
    }
}
