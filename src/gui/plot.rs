//! A widget for plotting telemetry data and the corresponding state.

use std::cell::RefCell;
use std::rc::Rc;

use eframe::egui::plot::PlotBounds;
use egui::plot::AxisBools;
#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use eframe::egui;
use eframe::egui::PointerButton;
use egui::widgets::plot::{Corner, Legend, Line, LineStyle, VLine};

use crate::gui::*;
use crate::state::*;
use crate::telemetry_ext::*;

/// Cache for a single line.
struct PlotCacheLine {
    name: String,
    color: Color32,
    pub callback: Box<dyn FnMut(&VehicleState) -> Option<f32>>,
    data: Vec<[f64; 2]>,
    last_bounds: Option<PlotBounds>,
    last_view: Vec<[f64; 2]>,
    stats: Option<(f64, f64, f64, f64)>,
}

impl PlotCacheLine {
    pub fn new(name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) -> Self {
        Self {
            name: name.to_string(),
            color,
            callback: Box::new(cb),
            data: Vec::new(),
            last_bounds: None,
            last_view: vec![],
            stats: None,
        }
    }

    pub fn push(&mut self, x: f64, vs: &VehicleState) {
        if let Some(value) = (self.callback)(vs) {
            self.data.push([x, value.into()]);
        }

        self.last_bounds = None; // TODO
        self.stats = None;
    }

    pub fn reset(&mut self) {
        self.data.truncate(0);
        self.last_bounds = None;
        self.last_view.truncate(0);
        self.stats = None;
    }

    pub fn data_for_bounds(&mut self, bounds: PlotBounds) -> Vec<[f64; 2]> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        if self.data.is_empty() {
            return vec![];
        }

        if self.last_bounds.map(|b| b != bounds).unwrap_or(true) {
            let (xmin, xmax) = (bounds.min()[0], bounds.max()[0]);

            let imin = self.data.partition_point(|d| d[0] < xmin);
            let imax = imin + self.data[imin..].partition_point(|d| d[0] < xmax);

            let imin = imin.saturating_sub(1);
            let imax = usize::min(imax + 1, self.data.len() - 1);

            self.last_view = self.data[imin..imax].to_vec();
            self.last_bounds = Some(bounds);
            self.stats = None;
        }

        self.last_view.clone()
    }

    pub fn stats(&mut self) -> Option<(f64, f64, f64, f64)> {
        if self.stats.is_none() && self.last_view.len() > 0 {
            let count = self.last_view.len() as f64;
            let mean = self.last_view.iter().map(|i| i[1]).sum::<f64>() / count;
            let var = self.last_view.iter().map(|i| f64::powi(i[1] - mean, 2)).sum::<f64>() / count;
            let std_dev = f64::sqrt(var);
            let min = self.last_view.iter().map(|i| i[1]).reduce(|a, b| f64::min(a, b)).unwrap();
            let max = self.last_view.iter().map(|i| i[1]).reduce(|a, b| f64::max(a, b)).unwrap();
            self.stats = Some((mean, std_dev, min, max));
        }

        self.stats
    }
}

/// Larger data structures cached for each plot, to avoid being recalculated
/// on each draw.
struct PlotCache {
    start: Instant,
    lines: Vec<PlotCacheLine>,
    mode_transitions: Vec<(f64, FlightMode)>,
    reset_on_next_draw: bool,
}

impl PlotCache {
    /// Create a new plot cache.
    fn new(start: Instant) -> Self {
        Self {
            lines: Vec::new(),
            mode_transitions: Vec::new(),
            reset_on_next_draw: false,
            start,
        }
    }

    fn add_line(&mut self, name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) {
        self.lines.push(PlotCacheLine::new(name, color, cb));
    }

    /// Incorporate some new data into the cache.
    pub fn push(&mut self, x: Instant, vs: &VehicleState) {
        let x = x.duration_since(self.start).as_secs_f64();

        // Value to be plotted.
        for l in self.lines.iter_mut() {
            l.push(x, vs);
        }

        // Vertical lines for mode transitions
        if let Some(mode) = vs.mode {
            if self.mode_transitions.last().map(|l| l.1 != mode).unwrap_or(true) {
                self.mode_transitions.push((x, mode));
            }
        }
    }

    /// Reset the cache.
    pub fn reset(&mut self, start: Instant, keep_position: bool) {
        for l in self.lines.iter_mut() {
            l.reset();
        }

        self.mode_transitions.truncate(0);
        self.start = start;

        // To reset some of the egui internals (state stored in the linked
        // axes/cursor groups), we also need to call reset on the egui object
        // during the next draw. Slightly hacky.
        self.reset_on_next_draw = !keep_position;
    }

    /// Lines to be plotted
    pub fn plot_lines(&mut self, bounds: PlotBounds, show_stats: bool) -> Vec<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.lines
            .iter_mut()
            .map(|pcl| {
                let data = pcl.data_for_bounds(bounds);
                let stats = show_stats.then(|| pcl.stats()).flatten();
                let legend = if let Some((mean, std_dev, min, max)) = stats {
                    format!(
                        "{} (mean: {:.2}, std dev.: {:.2}, min: {:.2}, max: {:.2})",
                        pcl.name, mean, std_dev, min, max
                    )
                } else {
                    pcl.name.clone()
                };
                Line::new(data).name(legend).color(pcl.color)
            })
            .collect()
    }

    /// Vertical mode transition lines to be plotted
    pub fn mode_lines(&self) -> Box<dyn Iterator<Item = VLine> + '_> {
        let iter = self.mode_transitions.iter().map(|(x, mode)| VLine::new(*x).color(mode.color()));
        Box::new(iter)
    }
}

/// State shared by all linked plots
pub struct SharedPlotState {
    /// First x-axis value
    pub start: Instant,
    /// Last x-axis value
    pub end: Instant,
    /// Are we currently attached to the right edge?
    pub attached_to_edge: bool,
    /// Width of the view (in seconds)
    pub view_width: f64,
    pub reset_on_next_draw: bool,
    pub box_dragging: bool,
    pub show_stats: bool,
}

impl SharedPlotState {
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
            end: Instant::now(),
            attached_to_edge: true,
            view_width: 10.0,
            reset_on_next_draw: false,
            box_dragging: false,
            show_stats: false,
        }
    }

    pub fn set_end(&mut self, end: Option<Instant>) {
        self.end = end.unwrap_or(self.start);
    }

    pub fn reset(&mut self, start: Instant) {
        self.start = start;
        self.end = start;
        self.view_width = 10.0;
        self.attached_to_edge = true;
        self.box_dragging = false;
    }

    pub fn show_all(&mut self) {
        self.view_width = (self.end - self.start).as_secs_f64();
        self.reset_on_next_draw = true;
    }

    pub fn process_zoom(&mut self, zoom_delta: Vec2) {
        self.view_width /= zoom_delta[0] as f64;
        // Zooming detaches the egui plot from the edge usually, so reattach
        // if we were attached previously. This allows zooming without missing
        // the end of the data.
        self.reset_on_next_draw = self.attached_to_edge;
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

    pub fn view_start(&self) -> f64 {
        self.end.duration_since(self.start).as_secs_f64() - self.view_width
    }

    pub fn view_end(&self) -> f64 {
        self.end.duration_since(self.start).as_secs_f64()
    }
}

/// State held by application for each plot, including the cached plot values.
/// This is `clone`d into the plotting callbacks on each draw, so it only holds
/// an `Rc<RefCell>` to the large `PlotCache`.
pub struct PlotState {
    // Heading of the plot
    pub title: String,
    // Cache of larger data structures
    cache: Rc<RefCell<PlotCache>>,
    // State shared among all linked plots
    shared: Rc<RefCell<SharedPlotState>>,
    // y-axis minimum (always included)
    pub ymin: Option<f32>,
    // y-axis maximum (always included)
    pub ymax: Option<f32>,
}

impl PlotState {
    // Create a new `PlotState`.
    pub fn new<S: ToString>(
        title: S,
        ylimits: (Option<f32>, Option<f32>),
        shared: Rc<RefCell<SharedPlotState>>,
    ) -> Self {
        let cache = PlotCache::new(shared.borrow().start);
        let (ymin, ymax) = ylimits;

        Self {
            title: title.to_string(),
            cache: Rc::new(RefCell::new(cache)),
            shared,
            ymin,
            ymax,
        }
    }

    pub fn line(self, name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) -> Self {
        self.cache.borrow_mut().add_line(name, color, cb);
        self
    }

    // Add some newly downlinked data to the cache.
    pub fn push(&mut self, x: Instant, vs: &VehicleState) {
        self.cache.borrow_mut().push(x, vs)
    }

    // Reset the plot, for instance when loading a different file.
    pub fn reset(&mut self, start: Instant, keep_position: bool) {
        self.cache.borrow_mut().reset(start, keep_position);
        self.shared.borrow_mut().reset(start);
    }

    pub fn show_all(&mut self) {
        self.shared.borrow_mut().show_all();
    }
}

pub trait PlotUiExt {
    fn plot_telemetry(&mut self, state: &PlotState);
}

impl PlotUiExt for egui::Ui {
    fn plot_telemetry(&mut self, state: &PlotState) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let mut shared = state.shared.borrow_mut();
        let mut cache = state.cache.borrow_mut();

        let legend =
            Legend::default().text_style(egui::TextStyle::Small).background_alpha(0.5).position(Corner::LeftTop);

        let mut plot = egui::widgets::plot::Plot::new(&state.title)
            .link_axis("plot_axis_group", true, false)
            .link_cursor("plot_cursor_gropu", true, false)
            .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
            .allow_scroll(false) // TODO: x only
            .allow_drag(AxisBools::new(true, false))
            .allow_zoom(AxisBools::new(true, false))
            .include_x(shared.view_start())
            .include_x(shared.view_end())
            .auto_bounds_y()
            .sharp_grid_lines(true)
            .legend(legend.clone());

        if let Some(min) = state.ymin {
            plot = plot.include_y(min);
        }

        if let Some(max) = state.ymax {
            plot = plot.include_y(max);
        }

        if cache.reset_on_next_draw || shared.reset_on_next_draw {
            cache.reset_on_next_draw = false;
            shared.reset_on_next_draw = false;
            shared.attached_to_edge = true;
            plot = plot.reset();
        }

        let show_stats = shared.show_stats;
        let ir = plot.show(self, move |plot_ui| {
            let lines = cache.plot_lines(plot_ui.plot_bounds(), show_stats);
            let mode_lines = cache.mode_lines();

            for l in lines.into_iter() {
                plot_ui.line(l.width(1.2));
            }

            for vl in mode_lines.into_iter() {
                plot_ui.vline(vl.style(LineStyle::Dashed { length: 4.0 }));
            }
        });

        // We have to check the interaction response to notice whether the plot
        // has been dragged or otherwise detached from the end of the data.
        if let Some(_hover_pos) = ir.response.hover_pos() {
            let zoom_delta = self.input(|i| i.zoom_delta_2d());
            let scroll_delta = self.input(|i| i.scroll_delta);
            if zoom_delta.x != 1.0 {
                shared.process_zoom(self.input(|i| i.zoom_delta_2d()));
            } else if scroll_delta.x != 0.0 {
                shared.attached_to_edge = false;
            }
        };

        if ir.response.dragged_by(PointerButton::Primary) {
            shared.attached_to_edge = false;
        }

        if ir.response.double_clicked_by(PointerButton::Primary) {
            shared.attached_to_edge = true;
        }

        shared.process_drag_released(ir.response.drag_released);
        shared.process_box_dragging(ir.response.dragged_by(PointerButton::Secondary));
    }
}
