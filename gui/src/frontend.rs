use telemetry::Metric;

use crate::frontend::{metric_monitor::MetricMonitor, popup_manager::PopupManager};

pub mod popup_manager;
pub mod metric_monitor;

pub struct Frontend {
    popup_manager: PopupManager,
    metric_monitor: MetricMonitor<Metric>,
}

impl Default for Frontend {
    fn default() -> Self {
        Self { popup_manager: Default::default(), metric_monitor: Default::default() }
    }
}

impl Frontend {
    
    pub fn update(&mut self) {
        self.popup_manager.update();
    }

    pub fn popup_manager_mut(&mut self) -> &mut PopupManager {
        return &mut self.popup_manager;
    }

    pub fn metric_monitor_mut(&mut self) -> &mut MetricMonitor<Metric> {
        return &mut self.metric_monitor;
    }

}