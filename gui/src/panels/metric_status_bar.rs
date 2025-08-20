use std::{cmp::Ordering, collections::HashMap, sync::LazyLock};

use egui::{Button, Image, Sense, Ui, Vec2};
use itertools::Itertools;
use strum::{IntoEnumIterator, VariantNames};
use telemetry::{Metric, MetricDiscriminants};

use crate::{
    backend::Backend,
    frontend::{
        Frontend,
        constraints::{ConstraintResult, EvaluatedConstraint},
        metric_monitor::MetricMonitor,
        popup_manager::{DropdownMenu, TriggerBuilder},
    },
    system_diagram_components::core::constants::{IMG_DANGER, IMG_FILTER, IMG_NOMINAL, IMG_WARNING},
    utils::theme::ThemeColors,
};

pub struct MetricStatusBar {}

trait HasImage {
    fn image(&self) -> egui::ImageSource;
}

impl HasImage for ConstraintResult {
    fn image(&self) -> egui::ImageSource {
        match self {
            ConstraintResult::NOMINAL => IMG_NOMINAL,
            ConstraintResult::WARNING => IMG_WARNING,
            ConstraintResult::DANGER => IMG_DANGER,
        }
    }
}

fn display(metric: &Metric, backend: &Backend, metric_monitor: &MetricMonitor, ui: &mut Ui, theme: &ThemeColors) {
    let val = backend.current_value(*metric).map(|v| format!("{0:.2}", v)).unwrap_or("N/A".to_string());
    let (name, unit) = match metric {
        Metric::Pressure(_) => ("Pressure", "bar"),
        Metric::Temperature(_) => ("Temperature", "°C"),
        _ => ("UNKOWN", "??"),
    };

    ui.label(name);
    ui.label(val);
    ui.label(unit);

    let worst_constraint_result =
        metric_monitor.constraint_results().get(metric).map(|res| res.iter().max()).unwrap_or_default();

    worst_constraint_result
        .map(|constraint| {
            ui.add(Image::new(constraint.result().image()).tint(theme.foreground_weak));
        })
        .unwrap_or_else(|| {
            ui.allocate_response(Vec2::ZERO, Sense::hover());
        });

    if metric_monitor.is_pinned(&metric) {
        ui.label("📌");
    } else {
        ui.allocate_response(Vec2::ZERO, Sense::hover());
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
        let theme = &ThemeColors::new(ctx);
        let num_metrics_to_display = frontend.metric_monitor().pinned_metrics().len()
            + Self::filter_constraints(frontend.metric_monitor().constraint_results(), active_constraint_mask)
                .keys()
                .count();
        let has_open_popups = frontend.popup_manager().has_any_open_popup(&METRIC_MONITOR_FILTER_BUTTON_ID);
        egui::SidePanel::left("Monitor Panel").show_animated(
            ctx,
            num_metrics_to_display > 0 || has_open_popups,
            |ui| {
                let mut dropdown_response = ui.allocate_response(Vec2::ZERO, Sense::click_and_drag());
                ui.horizontal(|ui| {
                    //ui.vertical_centered(|ui| {
                    ui.heading("Monitored Metrics");
                    //});
                    dropdown_response =
                        ui.add(Button::image(Image::new(IMG_FILTER).tint(theme.foreground_weak)).small())
                });
                TriggerBuilder::new(METRIC_MONITOR_FILTER_BUTTON_ID.clone(), dropdown_response.interact_rect)
                    .add::<DropdownMenu, _>(dropdown_response.interact_rect.right_top(), |ui, _frontend, _backend| {
                        let mut i = 0;
                        let mut response = ui.allocate_response(Vec2::ZERO, Sense::click_and_drag());
                        for constraint_result in ConstraintResult::iter() {
                            let active_position = active_constraint_mask
                                .iter()
                                .find_position(|e| **e == constraint_result)
                                .map(|(i, _c)| i);
                            let mut is_active = active_position.is_some();
                            response = response.union(ui.checkbox(&mut is_active, ConstraintResult::VARIANTS[i]));
                            if is_active && !active_position.is_some() {
                                active_constraint_mask.push(constraint_result);
                            } else if !is_active && active_position.is_some() {
                                active_position.map(|i| active_constraint_mask.swap_remove(i));
                            }
                            i += 1;
                        }
                        return response;
                    })
                    .show_active(ui, frontend, backend);
                ui.separator();
                egui::Grid::new("monitor_panel_metrics").striped(true).show(ui, |ui| {
                    for metric in frontend.metric_monitor().pinned_metrics() {
                        display(metric, backend, frontend.metric_monitor(), ui, theme);
                    }
                    for metric in
                        Self::filter_constraints(frontend.metric_monitor().constraint_results(), active_constraint_mask)
                            .keys()
                            .sorted_by(|m1, m2| Self::cmp_metric_for_display(m1, m2, frontend.metric_monitor()))
                    {
                        if !frontend.metric_monitor().is_pinned(metric) {
                            display(metric, backend, frontend.metric_monitor(), ui, theme);
                        }
                    }
                })
            },
        );
    }
}
