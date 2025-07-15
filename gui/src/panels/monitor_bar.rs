use telemetry::Metric;

use crate::{backend::Backend, frontend::metric_monitor::MetricMonitor};

pub struct MonitorBar {}

impl MonitorBar {
    pub fn show(ctx: &egui::Context, backend: &Backend, metric_monitor: &mut MetricMonitor<Metric>) {
        let monitored_metrics = metric_monitor.monitored_metrics();
        egui::SidePanel::left("Monitor Panel").show_animated(ctx, monitored_metrics.len() > 0, |ui| {
            ui.vertical_centered(|ui|
                ui.heading("Monitored Metrics")
            );
            ui.separator();
            egui::Grid::new("monitor_panel_metrics").striped(true).show(ui, |ui| {
                for metric in monitored_metrics {
                    let val = backend.current_value(metric).map(|v| format!("{0:.2}", v)).unwrap_or("N/A".to_string());
                    let (name, unit) = match metric {
                        Metric::Pressure(_) => ("Pressure", "bar"),
                        Metric::Temperature(_) => ("Temperature", "°C"),
                        _ => todo!(),
                    };
            
                    ui.label(name);
                    ui.label(val);
                    ui.label(unit);
                    if metric_monitor.is_pinned(&metric) && ui.button("📌").clicked() {
                        metric_monitor.unpin(metric);
                    }
                    ui.end_row();
                }
            })
        });
    }
}