//! A widget for plotting telemetry data and the corresponding state.

use std::cell::RefCell;
use std::rc::Rc;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
use egui_plot::HLine;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use egui::Vec2b;
use eframe::egui;
use eframe::egui::PointerButton;
use egui_plot::{Corner, Legend, Line, LineStyle, PlotBounds, VLine};

use crate::*;
use crate::utils::telemetry_ext::*;

const DOWNSAMPLING_FACTOR: usize = 4;
const MAX_DOWNSAMPLING_RUNS: usize = 2;

fn plot_time(x: &Instant, data_source: &dyn DataSource) -> f64 {
    if let Some((first_t, _first_vs)) = data_source.vehicle_states().next() {
        x.duration_since(*first_t).as_secs_f64()
    } else {
        0.0
    }
}

fn downsample_points(data: &[[f64; 2]]) -> Vec<[f64; 2]> {
    data.chunks(DOWNSAMPLING_FACTOR * 2)
        .flat_map(|chunk| {
            let (min_i, min) = chunk.iter().enumerate().min_by(|(_, x), (_, y)| x[1].total_cmp(&y[1])).unwrap();
            let (max_i, max) = chunk.iter().enumerate().max_by(|(_, x), (_, y)| x[1].total_cmp(&y[1])).unwrap();
            match (min_i, max_i) {
                (i, j) if i < j => [*min, *max],
                _ => [*max, *min],
            }
        })
        .collect()
}

type PlotCallback = dyn FnMut(&VehicleState) -> Option<f32>;

/// Cache for a single line.
struct PlotCacheLine {
    name: String,
    color: Color32,
    pub callback: Box<PlotCallback>,
    /// Increasingly downsampled plot data. The first entry is the full data, followed by
    /// smaller and smaller vectors.
    data: Vec<Vec<[f64; 2]>>,
    stats: Option<(f64, f64, f64, f64)>,
    last_bounds: Option<PlotBounds>,
    last_view: Vec<[f64; 2]>,
}

impl PlotCacheLine {
    pub fn new(name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) -> Self {
        Self {
            name: name.to_string(),
            color,
            callback: Box::new(cb),
            data: vec![Vec::new()],
            stats: None,
            last_bounds: None,
            last_view: vec![],
        }
    }

    fn update_cache(&mut self, data_source: &dyn DataSource, keep_first: usize) {
        let new_data = data_source.vehicle_states()
                .skip(keep_first)
                .map(|(t, vs)| (plot_time(t, data_source), vs))
                .filter_map(|(x, vs)| (self.callback)(vs).map(|y| [x, y as f64]));

        if keep_first > 0 {
            self.data[0].extend(new_data)
        } else {
            self.data[0] = new_data.collect();
        }

        // clear the downsample caches
        self.data.truncate(1);
        self.last_bounds = None;
        self.last_view.truncate(0);
    }

    fn clear_cache(&mut self) {
        self.data.truncate(1);
        self.data[0].truncate(0);
        self.last_bounds = None;
        self.last_view.truncate(0);
    }

    fn cached_data_downsampled(&mut self, bounds: PlotBounds, view_width: f32) -> Vec<[f64; 2]> {
        if self.data[0].is_empty() {
            return Vec::new();
        }

        let (xmin, xmax) = (bounds.min()[0], bounds.max()[0]);

        let mut imin = self.data[0].partition_point(|d| d[0] < xmin);
        let mut imax = imin + self.data[0][imin..].partition_point(|d| d[0] < xmax);

        imin = imin.saturating_sub(1);
        imax = usize::min(imax + 1, self.data[0].len() - 1);

        for level in 0..=MAX_DOWNSAMPLING_RUNS {
            if level >= self.data.len() {
                self.data.push(downsample_points(&self.data[level-1]));
            }

            if imax - imin < (2.0 * view_width) as usize {
                return self.data[level][imin..imax].to_vec();
            } else if level < MAX_DOWNSAMPLING_RUNS {
                imin /= DOWNSAMPLING_FACTOR;
                imax /= DOWNSAMPLING_FACTOR;
            }
        }

        self.data.last().unwrap()[imin..imax].to_vec()
    }

    pub fn data_for_bounds(&mut self, bounds: PlotBounds, data_source: &dyn DataSource, view_width: f32) -> Vec<[f64; 2]> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let len = data_source.vehicle_states().len();
        if len == 0 || self.data.is_empty() {
            self.last_bounds = None;
            self.stats = None;
            return vec![];
        }

        if self.last_bounds.map(|b| b != bounds).unwrap_or(true) {
            self.last_view = self.cached_data_downsampled(bounds, view_width);
            self.last_bounds = Some(bounds);
            self.stats = None;
        }

        self.last_view.clone()
    }

    pub fn stats(&mut self) -> Option<(f64, f64, f64, f64)> {
        if self.stats.is_none() && !self.last_view.is_empty() {
            let count = self.last_view.len() as f64;
            let mean = self.last_view.iter().map(|i| i[1]).sum::<f64>() / count;
            let var = self.last_view.iter().map(|i| f64::powi(i[1] - mean, 2)).sum::<f64>() / count;
            let std_dev = f64::sqrt(var);
            let min = self.last_view.iter().map(|i| i[1]).reduce(f64::min).unwrap();
            let max = self.last_view.iter().map(|i| i[1]).reduce(f64::max).unwrap();
            self.stats = Some((mean, std_dev, min, max));
        }

        self.stats
    }
}

/// Larger data structures cached for each plot, to avoid being recalculated
/// on each draw.
struct PlotCache {
    lines: Vec<PlotCacheLine>,
    mode_transitions: Vec<(f64, FlightMode)>,
    /// Identifies the origin of the current data using the last time cached and the number of
    /// states included
    cached_state: Option<(Instant, Instant, usize)> // TODO: maybe add some sort of flight identifier?
}

impl PlotCache {
    /// Create a new plot cache.
    fn new() -> Self {
        Self {
            lines: Vec::new(),
            mode_transitions: Vec::new(),
            cached_state: None,
        }
    }

    fn add_line(&mut self, name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) {
        self.lines.push(PlotCacheLine::new(name, color, cb));
    }

    fn update_mode_transition_cache(&mut self, data_source: &dyn DataSource, keep_first: usize) {
        let last_mode = if keep_first > 0 {
            self.mode_transitions.last().map(|(_,m)| *m)
        } else {
            None
        };

        let new_data = data_source.vehicle_states()
            .skip(keep_first)
            .filter(|(_t, vs)| vs.mode.is_some())
            .scan(last_mode, |state, (t, vs)| {
                if &vs.mode != state {
                    *state = vs.mode;
                    Some(Some((plot_time(t, data_source), vs.mode.unwrap())))
                } else {
                    Some(None)
                }
            })
            .flatten();

        if keep_first > 0 {
            self.mode_transitions.extend(new_data);
        } else {
            self.mode_transitions = new_data.collect();
        }
    }

    fn update_caches_if_necessary(&mut self, data_source: &dyn DataSource) {
        let new_len = data_source.vehicle_states().len();
        if new_len == 0 {
            self.mode_transitions.truncate(0);
            self.lines.iter_mut().for_each(|l| { l.clear_cache(); });
            self.cached_state = None;
            return;
        }

        let (first_t, _) = data_source.vehicle_states().next().unwrap().clone();
        let (last_t, _) = data_source.vehicle_states().next_back().unwrap().clone();
        let cached_state = Some((first_t, last_t, new_len));

        // We have already cached this exact set of vehicle states, do nothing.
        if cached_state == self.cached_state {
            return;
        }

        // Try to determine if the new data is simply a few more points appended to the previously
        // plotted data, which we have cached. If so, we keep the old and simply append the new
        // points. If not, we recalculate the cache completely.
        let old_len = self.cached_state.map(|(_, _, l)| l).unwrap_or(0);
        let mut keep_first = if new_len > old_len { old_len } else { 0 };
        if keep_first > 0 {
            // double-check that it is actually the same set of states by looking for our previous
            // last state in the new data
            let (previous_last, _) = data_source.vehicle_states()
                .rev()
                .nth(new_len - keep_first)
                .unwrap();
            if self.cached_state.map(|(_, t, _)| t != *previous_last).unwrap_or(true) {
                keep_first = 0;
            }
        }

        self.lines.iter_mut().for_each(|l| { l.update_cache(data_source, keep_first); });
        self.update_mode_transition_cache(data_source, keep_first);
        self.cached_state = cached_state;
    }

    /// Lines to be plotted
    pub fn plot_lines(
        &mut self,
        bounds: PlotBounds,
        show_stats: bool,
        data_source: &dyn DataSource,
        view_width: f32,
    ) -> Vec<Line> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.update_caches_if_necessary(data_source);
        self.lines
            .iter_mut()
            .map(|pcl| {
                let data = pcl.data_for_bounds(bounds, data_source, view_width);
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
    pub fn mode_lines(&mut self, data_source: &dyn DataSource) -> Box<dyn Iterator<Item = VLine> + '_> {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.update_caches_if_necessary(data_source);
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
            box_dragging: false,
            show_stats: false,
        }
    }

    pub fn set_end(&mut self, end: Option<Instant>) {
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
    horizontal_lines: Vec<(f64, Color32)>,
}

impl PlotState {
    // Create a new `PlotState`.
    pub fn new<S: ToString>(
        title: S,
        ylimits: (Option<f32>, Option<f32>),
        shared: Rc<RefCell<SharedPlotState>>,
    ) -> Self {
        let cache = PlotCache::new();
        let (ymin, ymax) = ylimits;

        Self {
            title: title.to_string(),
            cache: Rc::new(RefCell::new(cache)),
            shared,
            ymin,
            ymax,
            horizontal_lines: Vec::new(),
        }
    }

    pub fn line(self, name: &str, color: Color32, cb: impl FnMut(&VehicleState) -> Option<f32> + 'static) -> Self {
        self.cache.borrow_mut().add_line(name, color, cb);
        self
    }

    #[allow(dead_code)]
    pub fn horizontal_line(mut self, y: f64, color: Color32) -> Self {
        self.horizontal_lines.push((y, color));
        self
    }
}

pub trait PlotUiExt {
    fn plot_telemetry(&mut self, state: &PlotState, data_source: &dyn DataSource);
}

impl PlotUiExt for egui::Ui {
    fn plot_telemetry(&mut self, state: &PlotState, data_source: &dyn DataSource) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let mut shared = state.shared.borrow_mut();
        let mut cache = state.cache.borrow_mut();

        let legend =
            Legend::default().text_style(egui::TextStyle::Small).background_alpha(0.5).position(Corner::LeftTop);

        // Weaken the text color, used for the grid lines.
        let text_color = self.style().visuals.text_color();
        self.style_mut().visuals.override_text_color = Some(text_color.gamma_multiply(0.5));

        let view_end = plot_time(&data_source.end().unwrap_or(Instant::now()), data_source);
        #[allow(deprecated)] // the axis widths in egui suck, TODO
        let mut plot = egui_plot::Plot::new(&state.title)
            .link_axis("plot_axis_group", true, false)
            .link_cursor("plot_cursor_group", true, false)
            .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
            .allow_scroll(false) // TODO: x only
            .allow_drag([true, false])
            .allow_zoom([true, false])
            .auto_bounds([false, true].into())
            .y_axis_position(egui_plot::HPlacement::Right)
            // These two are needed to avoid egui adding a huge amount of space for the y axis ticks
            .y_axis_width(3)
            .y_axis_formatter(|gm, _range| {
                let tick = gm.value;
                let digits = (gm.step_size.log10() * -1.0) as usize;
                format!("{tick:.digits$}")
            })
            .include_x(view_end - shared.view_width)
            .include_x(view_end)
            .legend(legend.clone());

        if let Some(min) = state.ymin {
            plot = plot.include_y(min);
        }

        if let Some(max) = state.ymax {
            plot = plot.include_y(max);
        }

        let show_stats = shared.show_stats;
        let view_width = self.max_rect().width();
        let attached_to_edge = shared.attached_to_edge;
        let ir = plot.show(self, move |plot_ui| {
            if attached_to_edge {
                plot_ui.set_auto_bounds(Vec2b::new(true, true));
            }

            let lines = cache.plot_lines(plot_ui.plot_bounds(), show_stats, data_source, view_width);
            for l in lines.into_iter() {
                plot_ui.line(l.width(1.2));
            }

            for vl in cache.mode_lines(data_source) {
                plot_ui.vline(vl.style(LineStyle::Dashed { length: 4.0 }));
            }

            for (y, color) in &state.horizontal_lines {
                let hl = HLine::new(*y).color(*color);
                plot_ui.hline(hl.style(LineStyle::Dashed { length: 4.0 }));
            }
        });

        // We have to check the interaction response to notice whether the plot
        // has been dragged or otherwise detached from the end of the data.
        if let Some(_hover_pos) = ir.response.hover_pos() {
            let zoom_delta = self.input(|i| i.zoom_delta_2d());
            let scroll_delta = self.input(|i| i.smooth_scroll_delta);
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

        shared.process_drag_released(ir.response.drag_stopped());
        shared.process_box_dragging(ir.response.dragged_by(PointerButton::Secondary));
    }
}
