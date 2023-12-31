use std::cell::RefCell;
use std::rc::Rc;

use egui::Color32;
use egui::Rect;
use egui::Vec2;
use egui_gizmo::Gizmo;
use egui_gizmo::GizmoMode;
use egui_gizmo::GizmoVisuals;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

use crate::data_source::DataSource;
use crate::gui::plots::accelerometer_plot;
use crate::gui::plots::altitude_plot;
use crate::gui::plots::barometer_plot;
use crate::gui::plots::orientation_plot;
use crate::gui::plots::vertical_speed_plot;
use crate::settings::AppSettings;

use crate::gui::map::*;
use crate::gui::maxi_grid::*;
use crate::gui::plot::*;
use crate::gui::plots::color_constants::*;

#[derive(Debug, Clone, Copy, PartialEq)]
enum SelectedPlot {
    Orientation,
    VerticalSpeed,
    Altitude,
    Accelerometers,
    Barometer,
    Map,
}

impl std::fmt::Display for SelectedPlot {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SelectedPlot::Orientation => write!(f, "Orientation"),
            SelectedPlot::VerticalSpeed => write!(f, "Vertical Speed & Accel."),
            SelectedPlot::Altitude => write!(f, "Altitude"),
            SelectedPlot::Barometer => write!(f, "Barometer"),
            SelectedPlot::Accelerometers => write!(f, "Accelerometers"),
            SelectedPlot::Map => write!(f, "Map"),
        }
    }
}

pub struct LaunchTab {
    maxi_grid_state: MaxiGridState,
    dropdown_selected_plot: SelectedPlot,

    shared_plot: Rc<RefCell<SharedPlotState>>,
    orientation_plot: PlotState,
    vertical_speed_plot: PlotState,
    altitude_plot: PlotState,
    accelerometer_plot: PlotState,
    barometer_plot: PlotState,

    map: MapState,
}

impl LaunchTab {
    pub fn init(settings: &AppSettings) -> Self {
        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));

        let orientation_plot = orientation_plot(&shared_plot);
        let vertical_speed_plot = vertical_speed_plot(&shared_plot);
        let altitude_plot = altitude_plot(&shared_plot);
        let accelerometer_plot = accelerometer_plot(&shared_plot);
        let barometer_plot = barometer_plot(&shared_plot);

        let map = MapState::new(settings.mapbox_access_token.clone());

        Self {
            maxi_grid_state: MaxiGridState::default(),
            dropdown_selected_plot: SelectedPlot::Orientation,
            shared_plot,
            orientation_plot,
            vertical_speed_plot,
            altitude_plot,
            accelerometer_plot,
            barometer_plot,
            map,
        }
    }

    fn plot_gizmo(
        &mut self,
        ui: &mut egui::Ui,
        viewport: Rect,
        orientation: UnitQuaternion<f32>,
        colors: (Color32, Color32, Color32),
    ) {
        // We can't disable interaction with the gizmo, so we disable the entire UI
        // when the user gets too close. TODO: upstream way to disable interaction?
        let enabled = !ui.rect_contains_pointer(viewport);

        // use top right of plot for indicator, space below for plot
        let viewport = Rect::from_two_pos(viewport.lerp_inside(Vec2::new(0.4, 0.55)), viewport.right_top());

        let fade_to_color = Color32::BLACK;
        ui.visuals_mut().widgets.noninteractive.weak_bg_fill = fade_to_color;

        // square viewport
        let viewport_square_side = f32::min(viewport.width(), viewport.height());
        let viewport = viewport.shrink2((viewport.size() - Vec2::splat(viewport_square_side)) * 0.5);

        let view = UnitQuaternion::from_euler_angles(-90.0f32.to_radians(), 180f32.to_radians(), 0.0f32.to_radians());

        let visuals = GizmoVisuals {
            x_color: colors.0,
            y_color: colors.1,
            z_color: colors.2,
            inactive_alpha: 1.0,
            highlight_alpha: 1.0,
            stroke_width: 3.0,
            gizmo_size: viewport_square_side * 0.4,
            ..Default::default()
        };

        let gizmo = Gizmo::new("My gizmo")
            .mode(GizmoMode::Translate)
            .viewport(viewport)
            .orientation(egui_gizmo::GizmoOrientation::Local)
            .model_matrix(orientation.to_homogeneous())
            .view_matrix(view.to_homogeneous())
            .visuals(visuals);

        ui.add_enabled_ui(enabled, |ui| {
            gizmo.interact(ui);
        });
    }

    fn plot_orientation(&mut self, ui: &mut egui::Ui, data_source: &mut dyn DataSource) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let mut viewport = ui.cursor();
        viewport.set_width(ui.available_width());
        viewport.set_height(ui.available_height());

        let orientation = data_source
            .vehicle_states()
            .rev()
            .find_map(|(_, vs)| vs.orientation)
            .unwrap_or(UnitQuaternion::new(Vector3::new(0.0, 0.0, 0.0)));
        let true_orientation = data_source.vehicle_states().rev().find_map(|(_, vs)| vs.true_orientation);

        ui.plot_telemetry(&self.orientation_plot, data_source);

        if let Some(orientation) = true_orientation {
            self.plot_gizmo(ui, viewport, orientation, (R1, G1, B1));
        }

        self.plot_gizmo(ui, viewport, orientation, (R, G, B));
    }

    pub fn main_ui(&mut self, ui: &mut egui::Ui, data_source: &mut dyn DataSource) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        self.shared_plot.borrow_mut().set_end(data_source.end());

        if ui.available_width() > 1000.0 {
            MaxiGrid::new((3, 3), ui, self.maxi_grid_state.clone())
                .cell("Orientation", |ui| self.plot_orientation(ui, data_source))
                .cell("Vert. Speed & Accel", |ui| ui.plot_telemetry(&self.vertical_speed_plot, data_source))
                .cell("Altitude (ASL)", |ui| ui.plot_telemetry(&self.altitude_plot, data_source))
                .cell("Position", |ui| ui.map(&self.map, data_source))
                .cell("Accelerometers", |ui| ui.plot_telemetry(&self.accelerometer_plot, data_source))
                .cell("Barometer", |ui| ui.plot_telemetry(&self.barometer_plot, data_source));
        } else {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().combo_width = ui.available_width();
                    egui::ComboBox::from_id_source("plot_selector")
                        .selected_text(format!("{}", self.dropdown_selected_plot))
                        .show_ui(ui, |ui| {
                            ui.set_width(ui.available_width());
                            for p in [
                                SelectedPlot::Orientation,
                                SelectedPlot::VerticalSpeed,
                                SelectedPlot::Altitude,
                                SelectedPlot::Accelerometers,
                                SelectedPlot::Barometer,
                                SelectedPlot::Map,
                            ] {
                                ui.selectable_value(&mut self.dropdown_selected_plot, p, format!("{}", p));
                            }
                        });
                });

                match self.dropdown_selected_plot {
                    SelectedPlot::Orientation => self.plot_orientation(ui, data_source),
                    SelectedPlot::VerticalSpeed => ui.plot_telemetry(&self.vertical_speed_plot, data_source),
                    SelectedPlot::Altitude => ui.plot_telemetry(&self.altitude_plot, data_source),
                    SelectedPlot::Barometer => ui.plot_telemetry(&self.barometer_plot, data_source),
                    SelectedPlot::Accelerometers => ui.plot_telemetry(&self.accelerometer_plot, data_source),
                    SelectedPlot::Map => ui.map(&self.map, data_source),
                }
            });
        }
    }
}
