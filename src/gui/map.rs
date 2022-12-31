//! Contains code for a map widget. TODO: replace with a proper map.

use std::cell::RefCell;
use std::rc::Rc;

use eframe::egui;
use egui::widgets::plot::Line;
use egui::Color32;

use crate::state::*;

/// State of the map widget, stored by the application.
#[derive(Clone)]
pub struct MapState {
    pub vehicle_states: Rc<RefCell<Vec<VehicleState>>>
}

impl MapState {
    pub fn new(vehicle_states: Rc<RefCell<Vec<VehicleState>>>) -> Self {
        Self {
            vehicle_states
        }
    }
}

pub trait MapUiExt {
    fn map(
        &mut self,
        state: MapState
    );
}

impl MapUiExt for egui::Ui {
    fn map(
        &mut self,
        state: MapState
    ) {
        let points: Vec<[f64; 3]> = state.vehicle_states.borrow().iter()
            .map(|vs| (vs.time as f64 / 1000.0, vs.latitude, vs.longitude))
            .filter(|(_t, lat, lng)| lat.is_some() && lng.is_some())
            .map(|(t, lat, lng)| [t, lng.unwrap() as f64, lat.unwrap() as f64])
            .collect();

        let last = points.last().cloned();

        let points_10m: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 600.0)
            .collect();
        let points_1m: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 60.0)
            .collect();
        let points_10s: Vec<[f64; 3]> = points
            .iter()
            .cloned()
            .filter(|x| (last.unwrap()[0] - x[0]) < 10.0)
            .collect();

        let line = Line::new(points.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_10m = Line::new(points_10m.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_1m = Line::new(points_1m.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);
        let line_10s = Line::new(points_10s.iter().map(|x| [x[1], x[2]]).collect::<Vec<[f64; 2]>>()).width(1.2);

        self.vertical_centered(|ui| {
            let mut plot = egui::widgets::plot::Plot::new("map")
                .allow_scroll(false)
                .data_aspect(1.0)
                .set_margin_fraction(egui::Vec2::new(0.0, 0.15));

            if let Some(coords) = last {
                plot = plot.include_x(coords[1] - 0.001);
                plot = plot.include_x(coords[1] + 0.001);
                plot = plot.include_y(coords[2] - 0.001);
                plot = plot.include_y(coords[2] + 0.001);
            }

            plot.show(ui, |plot_ui| {
                plot_ui.line(line.color(Color32::DARK_RED));
                plot_ui.line(line_10m.color(Color32::RED));
                plot_ui.line(line_1m.color(Color32::YELLOW));
                plot_ui.line(line_10s.color(Color32::GREEN));
            });
        });
    }
}
