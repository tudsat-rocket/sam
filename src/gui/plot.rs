//! A widget for plotting telemetry data and the corresponding state.

use std::cell::RefCell;
use std::rc::Rc;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use instant::Instant;

use eframe::egui;
use egui::widgets::plot::{Corner, Legend, Line, VLine, LineStyle, LinkedAxisGroup, LinkedCursorsGroup};

use euroc_fc_firmware::telemetry::FlightMode;

use crate::gui::*;
use crate::state::*;
use crate::telemetry_ext::*;

/// Larger data structures cached for each plot, to avoid being recalculated
/// on each draw.
struct PlotCache {
    start: Instant,
    lines: Vec<PlotCacheLine>,
    mode_transitions: Vec<(f64, FlightMode)>,
    reset_on_next_draw: bool,
}

/// Cache for a single line.
struct PlotCacheLine {
    name: String,
    pub callback: Box<dyn FnMut(&DownlinkMessage) -> Option<f32>>,
    data: Vec<[f64; 2]>,
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

    fn add_line(&mut self, name: &str, cb: impl FnMut(&DownlinkMessage) -> Option<f32> + 'static) {
        self.lines.push(PlotCacheLine {
            name: name.to_string(),
            callback: Box::new(cb),
            data: Vec::new()
        });
    }

    /// Incorporate some new data into the cache.
    pub fn push(&mut self, x: Instant, vs: &DownlinkMessage) {
        let x = x.duration_since(self.start).as_secs_f64();

        // Value to be plotted.
        for l in self.lines.iter_mut() {
            if let Some(value) = (l.callback)(vs) {
                l.data.push([x, value.into()]); // TODO: f32?
            }
        }

        // Vertical lines for mode transitions
        if let Some(mode) = vs.mode() {
            if self.mode_transitions.last().map(|l| l.1 != mode).unwrap_or(true) {
                self.mode_transitions.push((x, mode));
            }
        }
    }

    /// Reset the cache.
    pub fn reset(&mut self, start: Instant) {
        for l in self.lines.iter_mut() {
            l.data.truncate(0);
        }
        self.mode_transitions.truncate(0);
        self.start = start;

        // To reset some of the egui internals (state stored in the linked
        // axes/cursor groups), we also need to call reset on the egui object
        // during the next draw. Slightly hacky.
        self.reset_on_next_draw = true;
    }

    /// The last x-axis value contained in the cache.
    pub fn last_time(&self) -> Option<f64> {
        let maxs: Vec<f64> = self.lines.iter()
            .filter_map(|l| l.data.last().map(|d| d[0]))
            .collect();

        (maxs.len() > 0).then(|| {
            maxs.iter()
                .fold(f64::MIN, |a, b| f64::max(a, *b))
        })
    }

    /// Lines to be plotted
    pub fn plot_lines(&self) -> Box<dyn Iterator<Item = Line> + '_> {
        let iter = self.lines.iter()
            .map(|pcl| Line::new(pcl.data.clone()).name(&pcl.name));
        Box::new(iter)
    }

    /// Vertical mode transition lines to be plotted
    pub fn mode_lines(&self) -> Box<dyn Iterator<Item = VLine> + '_> {
        let iter = self.mode_transitions.iter()
            .map(|(x, mode)| VLine::new(*x).color(mode.color()));
        Box::new(iter)
    }
}

/// State held by application for each plot, including the cached plot values.
/// This is `clone`d into the plotting callbacks on each draw, so it only holds
/// an `Rc<RefCell>` to the large `PlotCache`.
#[derive(Clone)]
pub struct PlotState {
    // Heading of the plot
    pub title: String,
    // Cache of larger data structures
    cache: Rc<RefCell<PlotCache>>,
    // Axis group (shared by all plots)
    pub axes: LinkedAxisGroup,
    // Cursor group (shared by all plots)
    pub cursors: LinkedCursorsGroup,
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
        axes: LinkedAxisGroup,
        cursors: LinkedCursorsGroup,
        start: Instant
    ) -> Self {
        let cache = PlotCache::new(start);
        let (ymin, ymax) = ylimits;

        Self {
            title: title.to_string(),
            cache: Rc::new(RefCell::new(cache)),
            axes,
            cursors,
            ymin,
            ymax,
        }
    }

    pub fn line(self, name: &str, cb: impl FnMut(&DownlinkMessage) -> Option<f32> + 'static) -> Self {
        self.cache.borrow_mut().add_line(name, cb);
        self
    }

    // Add some newly downlinked data to the cache.
    pub fn push(&mut self, x: Instant, msg: &DownlinkMessage) {
        self.cache.borrow_mut().push(x, msg)
    }

    // Reset the plot, for instance when loading a different file.
    pub fn reset(&mut self, start: Instant) {
        self.cache.borrow_mut().reset(start)
    }
}

pub trait PlotUiExt {
    fn plot_telemetry(
        &mut self,
        state: PlotState,
        xlen: f64
    );
}

impl PlotUiExt for egui::Ui {
    fn plot_telemetry(
        &mut self,
        state: PlotState,
        xlen: f64
    ) {
        let mut cache = state.cache.borrow_mut();

        let legend = Legend::default()
            .text_style(egui::TextStyle::Small)
            .background_alpha(0.5)
            .position(Corner::LeftTop);

        let mut plot = egui::widgets::plot::Plot::new(&state.title)
            .link_axis(state.axes.clone())
            .link_cursor(state.cursors.clone())
            .set_margin_fraction(egui::Vec2::new(0.0, 0.15))
            .allow_scroll_y(false)
            .allow_drag_y(false)
            .allow_zoom_y(false)
            .auto_bounds_y()
            .legend(legend.clone());

        if let Some(min) = state.ymin {
            plot = plot.include_y(min);
        }

        if let Some(max) = state.ymax {
            plot = plot.include_y(max);
        }

        if let Some(last) = cache.last_time() {
            // TODO: unify plot dragging zoom and this
            plot = plot
                .include_x(last)
                .include_x(last - xlen);
        }

        if cache.reset_on_next_draw {
            cache.reset_on_next_draw = false;
            plot = plot.reset();
        }

        plot.show(self, |plot_ui| {
            for l in cache.plot_lines() {
                plot_ui.line(l.width(1.2));
            }

            for vl in cache.mode_lines() {
                plot_ui.vline(vl.style(LineStyle::Dashed{length: 4.0}));
            }
        });
    }
}
