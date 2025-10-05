use std::{cmp::Ordering, collections::HashMap, sync::LazyLock};

use egui::{Sense, Ui, Vec2};
use itertools::Itertools;
use strum::VariantNames;
use telemetry::{Metric, MetricDiscriminants};

use crate::{
    backend::Backend,
    frontend::{
        Frontend,
        constraints::{ConstraintResult, EvaluatedConstraint},
        metric_monitor::MetricMonitor,
    },
};

pub struct MetricStatusBar {}

fn display(metric: &Metric, backend: &Backend, metric_monitor: &MetricMonitor, ui: &mut Ui) {
    let val = backend.current_value_dynamic_as_string(&metric);
    let name = format!("{:?}", metric);

    ui.label(name);
    ui.label(val);

    let worst_constraint_result =
        metric_monitor.constraint_results().get(metric).map(|res| res.iter().max()).unwrap_or_default();

    worst_constraint_result
        .map(|constraint| {
            ui.label(constraint.result().symbol());
        })
        .unwrap_or_else(|| {
            ui.allocate_response(Vec2::ZERO, Sense::hover());
        });

    if metric_monitor.is_pinned(&metric) {
        ui.label("📌");
    }

    ui.end_row();
}

const METRIC_MONITOR_FILTER_BUTTON_ID: LazyLock<egui::Id> =
    LazyLock::new(|| egui::Id::new("MetricMonitorFilterButton"));

impl MetricStatusBar {
    fn filter_constraints<'a>(
        constraint_results: &'a HashMap<Metric, Vec<EvaluatedConstraint>>,
        active_constraint_mask: &Vec<ConstraintResult>,
    ) -> HashMap<Metric, Vec<&'a EvaluatedConstraint>> {
        return constraint_results
            .iter()
            .map(|(m, ecs)| {
                (m.clone(), ecs.iter().filter(|ec| active_constraint_mask.contains(ec.result())).collect::<Vec<_>>())
            })
            .filter(|(_, ecs)| ecs.len() > 0)
            .collect();
    }

    fn cmp_metric_for_display(m1: &Metric, m2: &Metric, metric_monitor: &MetricMonitor) -> Ordering {
        // 1. Sort by severity of constraint result
        let r1 = metric_monitor.strongest_constraint_result(m1);
        let r2 = metric_monitor.strongest_constraint_result(m2);
        if r1 != r2 {
            return r1.cmp(&r2);
        }
        // 2. Sort discriminants alphabetically TODO: Values with an identical discriminant have undefined ordering!
        let d1: MetricDiscriminants = m1.into();
        let d2: MetricDiscriminants = m2.into();
        return Metric::VARIANTS[d1 as usize].cmp(Metric::VARIANTS[d2 as usize]);
    }

    pub fn show(
        ctx: &egui::Context,
        backend: &mut Backend,
        frontend: &mut Frontend,
        active_constraint_mask: &mut Vec<ConstraintResult>,
    ) {
        let num_metrics_to_display = frontend.metric_monitor().pinned_metrics().len()
            + Self::filter_constraints(frontend.metric_monitor().constraint_results(), active_constraint_mask)
                .keys()
                .count();
        let has_open_popups = frontend.popup_manager().has_any_open_popup(&METRIC_MONITOR_FILTER_BUTTON_ID);
        egui::SidePanel::left("Monitor Panel").show_animated(
            ctx,
            num_metrics_to_display > 0 || has_open_popups,
            |ui| {
                ui.horizontal(|ui| {
                    ui.vertical_centered(|ui| {
                        ui.heading("Monitored Metrics");
                    });
                });
                ui.separator();
                egui::Grid::new("monitor_panel_metrics").striped(true).show(ui, |ui| {
                    for metric in frontend.metric_monitor().pinned_metrics() {
                        display(metric, backend, frontend.metric_monitor(), ui);
                    }
                    for metric in
                        Self::filter_constraints(frontend.metric_monitor().constraint_results(), active_constraint_mask)
                            .keys()
                            .sorted_by(|m1, m2| Self::cmp_metric_for_display(m1, m2, frontend.metric_monitor()))
                    {
                        if !frontend.metric_monitor().is_pinned(metric) {
                            display(metric, backend, frontend.metric_monitor(), ui);
                        }
                    }
                })
            },
        );
    }
}
