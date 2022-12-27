use eframe::egui;
use egui::widgets::plot::{Corner, Legend, Line, VLine, LineStyle, LinkedAxisGroup, LinkedCursorsGroup};

use euroc_fc_firmware::telemetry::FlightMode;

use crate::state::*;
use crate::telemetry_ext::*;

type DataCallback = Box<dyn FnMut(&VehicleState) -> Option<f32>>;

struct PlotCacheLine {
    title: String,
    pub callback: DataCallback,
    data: Vec<[f64; 2]>,
}

pub struct PlotCache {
    pub title: String,
    lines: Vec<PlotCacheLine>,
    mode_transitions: Vec<(f64, FlightMode)>,
    pub ymin: Option<f32>,
    pub ymax: Option<f32>,
}

impl PlotCache {
    pub fn new<S: ToString>(
        title: S,
        lines: Vec<(S, DataCallback)>,
        ylimits: (Option<f32>, Option<f32>)
    ) -> Self {
        let lines = lines.into_iter()
            .map(|(s, cb)| PlotCacheLine {
                title: s.to_string(),
                callback: cb,
                data: Vec::new()
            })
            .collect();
        let (ymin, ymax) = ylimits;

        Self {
            title: title.to_string(),
            lines,
            mode_transitions: Vec::new(),
            ymin,
            ymax,
        }
    }

    pub fn push(&mut self, vs: &VehicleState) {
        for l in self.lines.iter_mut() {
            if let Some(value) = (l.callback)(vs) {
                l.data.push([vs.time as f64 / 1000.0, value.into()]); // TODO: f32?
            }
        }

        if let Some(mode) = vs.mode {
            if self.mode_transitions.last().map(|l| l.1 != mode).unwrap_or(true) {
                self.mode_transitions.push((vs.time as f64 / 1000.0, mode));
            }
        }
    }

    pub fn reset(&mut self) {
        for l in self.lines.iter_mut() {
            l.data.truncate(0);
        }
        self.mode_transitions.truncate(0);
    }

    pub fn last_time(&self) -> Option<f64> {
        let maxs: Vec<f64> = self.lines.iter()
            .filter_map(|l| l.data.last().map(|d| d[0]))
            .collect();

        (maxs.len() > 0).then(|| {
            maxs.iter()
                .fold(f64::MIN, |a, b| f64::max(a, *b))
        })
    }

    pub fn plot_lines(&self) -> Box<dyn Iterator<Item = Line> + '_> {
        let iter = self.lines.iter()
            .map(|pcl| Line::new(pcl.data.clone()).name(&pcl.title));
        Box::new(iter)
    }

    pub fn mode_lines(&self) -> Box<dyn Iterator<Item = VLine> + '_> {
        let iter = self.mode_transitions.iter()
            .map(|(x, mode)| VLine::new(*x).color(mode.color()));
        Box::new(iter)
    }
}

pub trait PlotUiExt {
    fn plot_telemetry(
        &mut self,
        cache: &PlotCache,
        axis_group: &LinkedAxisGroup,
        cursors_group: &LinkedCursorsGroup,
        xlen: f64
    );
}

impl PlotUiExt for egui::Ui {
    fn plot_telemetry(
        &mut self,
        cache: &PlotCache,
        axis_group: &LinkedAxisGroup,
        cursors_group: &LinkedCursorsGroup,
        xlen: f64
    ) {
        let legend = Legend::default().background_alpha(0.5).position(Corner::LeftTop);

        self.vertical_centered(|ui| {
            ui.heading(&cache.title);
            let mut plot = egui::widgets::plot::Plot::new(&cache.title)
                .link_axis(axis_group.clone())
                .link_cursor(cursors_group.clone())
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
                .allow_scroll_y(false)
                .allow_drag_y(false)
                .allow_zoom_y(false)
                .auto_bounds_y()
                .legend(legend.clone());

            if let Some(min) = cache.ymin {
                plot = plot.include_y(min);
            }

            if let Some(max) = cache.ymax {
                plot = plot.include_y(max);
            }

            if let Some(last) = cache.last_time() {
                // TODO: unify plot dragging zoom and this
                plot = plot
                    .include_x(last)
                    .include_x(last - xlen);
            }

            plot.show(ui, |plot_ui| {
                for l in cache.plot_lines() {
                    plot_ui.line(l.width(1.2));
                }

                for vl in cache.mode_lines() {
                    plot_ui.vline(vl.style(LineStyle::Dashed{length: 4.0}));
                }
            });
        });
    }
}
