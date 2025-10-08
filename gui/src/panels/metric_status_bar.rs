use std::{cmp::Ordering, collections::HashMap, sync::LazyLock};

use egui::{Align, ComboBox, FontId, Layout, TextFormat, Ui, text::LayoutJob};
use itertools::Itertools;
use strum::IntoEnumIterator;
use telemetry::Metric;

use crate::{
    backend::{Backend, storage::static_metrics::debug_name_dyn},
    frontend::{
        Frontend,
        constraints::{ConstraintResult, EvaluatedConstraint},
        metric_monitor::MetricMonitor,
    },
    utils::theme::ThemeColors,
};

pub struct MetricStatusBar {}

fn display(metric: &Metric, backend: &Backend, metric_monitor: &MetricMonitor, ui: &mut Ui) {
    let theme = ThemeColors::new(ui.ctx());
    let val = backend.current_value_dynamic_as_string(&metric);
    let name = debug_name_dyn(metric);

    ui.label(name);
    ui.label(val);

    //let worst_constraint_result =
    //  metric_monitor.constraint_results().get(metric).map(|res| res.iter().max()).unwrap_or_default();
    // worst_constraint_result
    //     .map(|constraint| {
    //         ui.label(constraint.result().symbol());
    //     })
    //     .unwrap_or_else(|| {
    //         ui.allocate_response(Vec2::ZERO, Sense::hover());
    //     });

    if metric_monitor.is_pinned(&metric) {
        ui.label("📌");
    }

    ui.end_row();

    let empty_vec = vec![];
    let constraints = metric_monitor.constraint_results().get(metric).unwrap_or(&empty_vec);
    for (idx, result) in constraints
        .iter()
        .filter(|c| metric_monitor.active_constraint_mask().contains(c.result()))
        .sorted()
        .rev()
        .enumerate()
    {
        let mut reason_layout: LayoutJob = LayoutJob::default();
        result
            .result()
            .symbol()
            .append_to(&mut reason_layout, &Default::default(), Default::default(), Align::LEFT);
        reason_layout.append(
            if idx == constraints.len() - 1 {
                " └── "
            } else {
                " ├── "
            },
            0f32,
            TextFormat {
                color: theme.foreground_weak,
                font_id: FontId {
                    size: 12f32,
                    family: egui::FontFamily::Monospace,
                },
                ..Default::default()
            },
        );
        reason_layout.append(
            &result.evaluation_as_string(),
            0f32,
            TextFormat {
                color: theme.foreground_weak,
                //color: result.result().color(),
                ..Default::default()
            },
        );
        ui.label(reason_layout);
        ui.end_row();
    }
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
            return r1.cmp(&r2).reverse();
        }
        // 2. Sort discriminants alphabetically
        let name_m1 = debug_name_dyn(m1);
        let name_m2 = debug_name_dyn(m2);
        return name_m1.cmp(&name_m2);
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
                ui.scope(|ui| {
                    ui.with_layout(Layout::left_to_right(egui::Align::TOP), |ui| {
                        ui.heading("Monitored Metrics");
                        ui.add_space(50.0);
                        let mut active_constraints = vec![];
                        for result in ConstraintResult::iter() {
                            if active_constraint_mask.contains(&result) {
                                active_constraints.push(true);
                            } else {
                                active_constraints.push(false);
                            }
                        }
                        egui::ComboBox::new(*METRIC_MONITOR_FILTER_BUTTON_ID, "").selected_text("Filter").show_ui(
                            ui,
                            |ui| {
                                for (i, result) in ConstraintResult::iter().enumerate() {
                                    ui.checkbox(&mut active_constraints[i], result.string());
                                }
                            },
                        );
                        active_constraint_mask.clear();
                        for (i, result) in ConstraintResult::iter().enumerate() {
                            if active_constraints[i] {
                                active_constraint_mask.push(result);
                            }
                        }
                    });
                });

                // ui.horizontal(|ui| {
                //     ui.vertical_centered(|ui| {
                //         ui.heading("Monitored Metrics");
                //     });
                // });
                // //ui.allocate_ui_with_layout(ui.min_size(), Layout::left_to_right(Align::RIGHT), |ui| {
                // let mut active_constraints = vec![];
                // for result in ConstraintResult::iter() {
                //     if active_constraint_mask.contains(&result) {
                //         active_constraints.push(true);
                //     } else {
                //         active_constraints.push(false);
                //     }
                // }
                // egui::ComboBox::new(*METRIC_MONITOR_FILTER_BUTTON_ID, "").selected_text("Filter").show_ui(ui, |ui| {
                //     for (i, result) in ConstraintResult::iter().enumerate() {
                //         ui.checkbox(&mut active_constraints[i], result.string());
                //     }
                // });
                // active_constraint_mask.clear();
                // for (i, result) in ConstraintResult::iter().enumerate() {
                //     if active_constraints[i] {
                //         active_constraint_mask.push(result);
                //     }
                // }

                //})
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
