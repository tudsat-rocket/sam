//! A widget for plotting telemetry data and the corresponding state.

use eframe::egui;
use eframe::egui::PointerButton;
use egui::Vec2b;
use egui_plot::{Corner, Legend, Line};
use serde::{Deserialize, Serialize};

use telemetry::Metric;

use crate::*;

/// State shared by all linked plots
pub struct SharedPlotState {
    /// First x-axis value
    pub start: f64,
    /// Last x-axis value
    pub end: f64,
    /// Are we currently attached to the right edge?
    pub attached_to_edge: bool,
    /// Width of the view (in seconds)
    pub view_width: f64,
    pub box_dragging: bool,
    pub show_stats: bool,
}

impl SharedPlotState {
    pub fn new() -> Self {
        Self {
            start: 0.0,
            end: 1.0,
            attached_to_edge: true,
            view_width: 120.0,
            box_dragging: false,
            show_stats: false,
        }
    }

    pub fn set_end(&mut self, end: Option<f64>) {
        self.end = end.unwrap_or(self.start);
    }

    pub fn process_zoom(&mut self, zoom_delta: Vec2) {
        self.view_width /= zoom_delta[0] as f64;
    }

    pub fn process_box_dragging(&mut self, box_dragging: bool) {
        self.box_dragging = self.box_dragging || box_dragging;
    }

    pub fn process_drag_released(&mut self, released: bool) {
        if released && self.box_dragging {
            self.attached_to_edge = false;
            self.box_dragging = false;
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlotConfig {
    pub lines: Vec<(Metric, Color32)>,
    pub ylimits: (Option<f64>, Option<f64>),
}

pub struct Plot<'a> {
    config: &'a PlotConfig,
    shared: &'a mut SharedPlotState,
    backend: &'a Backend,
}

impl<'a> Plot<'a> {
    pub fn new(config: &'a PlotConfig, shared: &'a mut SharedPlotState, backend: &'a Backend) -> Self {
        Self {
            config,
            shared,
            backend,
        }
    }
}

impl<'a> egui::Widget for Plot<'a> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let legend = Legend::default().background_alpha(0.5).position(Corner::LeftTop);

        // Weaken the text color, used for the grid lines.
        let text_color = ui.style().visuals.text_color();
        ui.style_mut().visuals.override_text_color = Some(text_color.gamma_multiply(0.5));

        let view_end = self.backend.end().unwrap_or_default();
        #[allow(deprecated)] // the axis widths in egui suck, TODO
        let mut plot = egui_plot::Plot::new(ui.next_auto_id())
            .link_axis("plot_axis_group", [true, false])
            .link_cursor("plot_cursor_group", [true, false])
            .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
            .allow_scroll([true, false])
            .allow_drag([true, false])
            .allow_zoom([true, false])
            .auto_bounds([false, true])
            .y_axis_position(egui_plot::HPlacement::Right)
            // These two are needed to avoid egui adding a huge amount of space for the y axis ticks
            .y_axis_width(3)
            .y_axis_formatter(|gm, _range| {
                let tick = gm.value;
                let digits = (gm.step_size.log10() * -1.0) as usize;
                format!("{tick:.digits$}")
            })
            .include_x(view_end - self.shared.view_width)
            .include_x(view_end)
            .legend(legend.clone());

        if let Some(min) = self.config.ylimits.0 {
            plot = plot.include_y(min);
        }

        if let Some(max) = self.config.ylimits.1 {
            plot = plot.include_y(max);
        }

        let attached_to_edge = self.shared.attached_to_edge;
        let ir = plot.show(ui, move |plot_ui| {
            if attached_to_edge {
                plot_ui.set_auto_bounds(Vec2b::new(true, true));
            }

            for (key, color) in &self.config.lines {
                let name = format!("{key}");
                let plot_data = self.backend.plot_metric(key, plot_ui.plot_bounds());
                let line = Line::new(plot_data).name(name).color(*color).width(1.2);
                plot_ui.line(line);
            }

            // TODO: flight mode_transitions

            //for vl in cache.borrow_mut().mode_lines(backend) {
            //    plot_ui.vline(vl.style(LineStyle::Dashed { length: 4.0 }));
            //}

            //for (y, color) in &state.horizontal_lines {
            //    let hl = egui_plot::HLine::new(*y).color(*color);
            //    plot_ui.hline(hl.style(LineStyle::Dashed { length: 4.0 }));
            //}
        });

        // We have to check the interaction response to notice whether the plot
        // has been dragged or otherwise detached from the end of the data.
        if let Some(_hover_pos) = ir.response.hover_pos() {
            let zoom_delta = ui.input(|i| i.zoom_delta_2d());
            let scroll_delta = ui.input(|i| i.smooth_scroll_delta);
            if zoom_delta.x != 1.0 {
                self.shared.process_zoom(ui.input(|i| i.zoom_delta_2d()));
            } else if scroll_delta.x != 0.0 {
                self.shared.attached_to_edge = false;
            }
        };

        if ir.response.dragged_by(PointerButton::Primary) {
            self.shared.attached_to_edge = false;
        }

        if ir.response.double_clicked_by(PointerButton::Primary) {
            self.shared.attached_to_edge = true;
        }

        self.shared.process_drag_released(ir.response.drag_stopped());
        self.shared.process_box_dragging(ir.response.dragged_by(PointerButton::Secondary));

        ir.response
    }
}
