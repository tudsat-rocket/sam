use std::cell::RefCell;
use std::ops::DerefMut;
use std::rc::Rc;

use egui::Align;
use egui::CentralPanel;
use egui::Color32;
use egui::Layout;
use egui::SelectableLabel;
use egui::SidePanel;
use egui::Stroke;
use egui::TextBuffer;
use egui::TextEdit;
use egui::Vec2;
use egui_tiles::SimplificationOptions;
use egui_tiles::TabState;
use serde::{Deserialize, Serialize};

use telemetry::*;

use crate::system_diagram_components::diagrams;
use crate::settings::AppSettings;
use crate::widgets::system_diagram::*;
use crate::widgets::map::*;
use crate::widgets::plot::*;
use crate::*;

const R: Color32 = Color32::from_rgb(0xfb, 0x49, 0x34);
const G: Color32 = Color32::from_rgb(0xb8, 0xbb, 0x26);
const B: Color32 = Color32::from_rgb(0x83, 0xa5, 0x98);
const R1: Color32 = Color32::from_rgb(0xcc, 0x24, 0x1d);
const G1: Color32 = Color32::from_rgb(0x98, 0x97, 0x1a);
const B1: Color32 = Color32::from_rgb(0x45, 0x85, 0x88);
const O: Color32 = Color32::from_rgb(0xfa, 0xbd, 0x2f);
const O1: Color32 = Color32::from_rgb(0xd6, 0x5d, 0x0e);
const BR: Color32 = Color32::from_rgb(0x61, 0x48, 0x1c);
const P: Color32 = Color32::from_rgb(0xb1, 0x62, 0x86);
const C: Color32 = Color32::from_rgb(0x68, 0x9d, 0x6a);

pub struct GlobalWidgetState {
    map: MapState,
    shared_plot: Rc<RefCell<SharedPlotState>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum WidgetPane {
    Plot { title: String, config: PlotConfig },
    Map,
    Acs,
    Hybrid,
}

struct TileBehavior<'a> {
    backend: &'a mut Backend,
    settings: &'a AppSettings,
    global_widget_state: &'a mut GlobalWidgetState,
}

impl<'a> egui_tiles::Behavior<WidgetPane> for TileBehavior<'a> {
    fn tab_title_for_pane(&mut self, pane: &WidgetPane) -> egui::WidgetText {
        match pane {
            WidgetPane::Plot { title, config: _ } => title.clone(),
            WidgetPane::Map => "Map".to_string(),
            WidgetPane::Acs => "ACS".to_string(),
            WidgetPane::Hybrid => "Hybrid".to_string(),
        }
        .into()
    }

    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: egui_tiles::TileId,
        pane: &mut WidgetPane,
    ) -> egui_tiles::UiResponse {
        match pane {
            WidgetPane::Plot { title: _, config } => {
                ui.add(Plot::new(&config, self.global_widget_state.shared_plot.borrow_mut().deref_mut(), self.backend))
            }
            WidgetPane::Map => ui.add(Map::new(&mut self.global_widget_state.map, self.backend, self.settings)),
            WidgetPane::Acs => ui.add(diagrams::acs::create_diagram(self.backend)),
            WidgetPane::Hybrid => ui.add(diagrams::hyacinth::create_diagram(self.backend)),
        };

        egui_tiles::UiResponse::None
    }

    fn dragged_overlay_color(&self, _visuals: &egui::Visuals) -> Color32 {
        Color32::from_rgb(0xb8, 0xbb, 0x26).gamma_multiply(0.5)
    }

    fn drag_preview_stroke(&self, _visuals: &egui::Visuals) -> egui::Stroke {
        Stroke {
            width: 1.0,
            color: Color32::from_rgb(0xb8, 0xbb, 0x26),
        }
    }

    fn drag_preview_color(&self, _visuals: &egui::Visuals) -> Color32 {
        Color32::from_rgb(0xb8, 0xbb, 0x26).gamma_multiply(0.5)
    }

    fn tab_bar_color(&self, visuals: &egui::Visuals) -> Color32 {
        visuals.widgets.noninteractive.bg_fill
    }

    fn tab_bg_color(
        &self,
        visuals: &egui::Visuals,
        _tiles: &egui_tiles::Tiles<WidgetPane>,
        _tile_id: egui_tiles::TileId,
        tab_state: &TabState,
    ) -> Color32 {
        if tab_state.active {
            self.tab_bar_color(visuals)
        } else {
            visuals.extreme_bg_color
        }
    }

    fn tab_text_color(
        &self,
        visuals: &egui::Visuals,
        _tiles: &egui_tiles::Tiles<WidgetPane>,
        _tile_id: egui_tiles::TileId,
        tab_state: &TabState,
    ) -> Color32 {
        if tab_state.active {
            visuals.widgets.active.fg_stroke.color
        } else {
            visuals.widgets.hovered.fg_stroke.color
        }
    }

    fn simplification_options(&self) -> egui_tiles::SimplificationOptions {
        SimplificationOptions {
            prune_empty_tabs: true,
            prune_empty_containers: true,
            prune_single_child_containers: true,
            all_panes_must_have_tabs: true,
            join_nested_linear_containers: true,
            ..SimplificationOptions::OFF
        }
    }
}

pub fn default_orientation_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Orientation".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::Elevation, B),
                (Metric::Azimuth, R),
                (Metric::TrueElevation, B1),
                (Metric::TrueAzimuth, R1),
            ],
            ylimits: (Some(-180.0), Some(360.0)),
        },
    }
}

pub fn default_vertical_speed_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Vertical Speed/Accel.".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::AccelerationWorldSpace(Dim::Z), O),
                (Metric::VelocityWorldSpace(Dim::Z), B),
                (Metric::TrueAccelerationWorldSpace(Dim::Z), O1),
                (Metric::TrueVelocityWorldSpace(Dim::Z), B1),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_altitude_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Altitude (ASL)".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::GroundAltitudeASL, BR),
                (Metric::RawBarometricAltitude(BarometerId::MS5611), B1),
                (Metric::PositionWorldSpace(Dim::Z), B),
                (Metric::ApogeeAltitudeASL, O1),
                (Metric::GpsAltitude, G),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_gyroscope_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Gyroscope".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::X), R),
                (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::Y), G),
                (Metric::RawAngularVelocity(GyroscopeId::LSM6DSR, Dim::Z), B),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_accelerometer_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Accelerometers".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::X), R1),
                (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::Y), G1),
                (Metric::RawAcceleration(AccelerometerId::H3LIS331, Dim::Z), B1),
                (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::X), R),
                (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::Y), G),
                (Metric::RawAcceleration(AccelerometerId::LSM6DSR, Dim::Z), B),
            ],
            ylimits: (Some(-10.0), Some(10.0)),
        },
    }
}

pub fn default_magnetometer_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Magnetometer".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::X), R),
                (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::Y), G),
                (Metric::RawMagneticFluxDensity(MagnetometerId::LIS3MDL, Dim::Z), B),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_pressures_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Pressures".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611)), C),
                (Metric::Pressure(PressureSensorId::AcsTank), R),
                (Metric::Pressure(PressureSensorId::AcsPostRegulator), G),
                (Metric::Pressure(PressureSensorId::AcsValveAccel), O),
                (Metric::Pressure(PressureSensorId::AcsValveDecel), O1),
                (Metric::Pressure(PressureSensorId::RecoveryChamberDrogue), B),
                (Metric::Pressure(PressureSensorId::RecoveryChamberMain), B1),
                (Metric::Pressure(PressureSensorId::MainRelease), BR),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_valves_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Valves".to_string(),
        config: PlotConfig {
            lines: vec![
                // TODO
            ],
            ylimits: (Some(-1.0), Some(1.0)),
        },
    }
}

pub fn default_masses_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Masses".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::TrueVehicleMass, B),
                (Metric::TrueMotorMass, R),
                (Metric::TrueThrusterPropellantMass, O1),
            ],
            ylimits: (Some(0.0), None),
        },
    }
}

pub fn default_kalman_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Kalman".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::KalmanStateCovariance(0, 0), R),
                (Metric::KalmanStateCovariance(2, 2), B),
                (Metric::KalmanStateCovariance(5, 5), O1),
                (Metric::KalmanMeasurementCovariance(0, 0), C),
                (Metric::KalmanMeasurementCovariance(4, 4), G),
            ],
            ylimits: (None, None),
        },
    }
}

pub fn default_runtime_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Runtime".to_string(),
        config: PlotConfig {
            lines: vec![(Metric::CpuUtilization, O)],
            ylimits: (Some(0.0), Some(100.0)),
        },
    }
}

pub fn default_temperatures_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Temperatures".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611)), C),
                (Metric::Temperature(TemperatureSensorId::Acs), R),
                (Metric::Temperature(TemperatureSensorId::Recovery), B),
                (Metric::Temperature(TemperatureSensorId::Payload), O),
            ],
            ylimits: (Some(25.0), Some(35.0)),
        },
    }
}

pub fn default_power_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Power".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::BatteryVoltage(BatteryId::Avionics), G),
                (Metric::BatteryCurrent(BatteryId::Avionics), O1),
                (Metric::SupplyVoltage, B),
                (Metric::BatteryVoltage(BatteryId::Acs), G1),
                (Metric::BatteryCurrent(BatteryId::Acs), O),
                (Metric::BatteryVoltage(BatteryId::Recovery), C),
                (Metric::BatteryCurrent(BatteryId::Recovery), R),
                (Metric::BatteryVoltage(BatteryId::Payload), G1),
                (Metric::BatteryCurrent(BatteryId::Payload), O),
            ],
            ylimits: (Some(0.0), Some(15.0)),
        },
    }
}

pub fn default_signal_plot() -> WidgetPane {
    WidgetPane::Plot {
        title: "Signal".to_string(),
        config: PlotConfig {
            lines: vec![
                (Metric::DownlinkRssi, B),
                (Metric::DownlinkSnr, B1),
                (Metric::UplinkRssi, P),
                (Metric::UplinkSnr, C),
                (Metric::GpsHdop, R),
                (Metric::GpsSatellites, G),
            ],
            ylimits: (Some(-100.0), Some(10.0)),
        },
    }
}

pub struct PlotTab {
    tile_tree: egui_tiles::Tree<WidgetPane>,
    show_view_settings: bool,
    new_preset_name: String,
    global_widget_state: GlobalWidgetState,
}

impl PlotTab {
    pub fn init(ctx: &egui::Context, settings: &AppSettings) -> Self {
        SystemDiagram::init(ctx);

        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));
        let mapbox_token = (!settings.mapbox_access_token.is_empty()).then_some(settings.mapbox_access_token.clone());
        let map = MapState::new(ctx, mapbox_token);

        let global_widget_state = GlobalWidgetState { map, shared_plot };

        Self {
            tile_tree: Self::tree_default(),
            show_view_settings: false,
            new_preset_name: String::new(),
            global_widget_state,
        }
    }

    fn tree_default() -> egui_tiles::Tree<WidgetPane> {
        let mut tiles = egui_tiles::Tiles::default();

        let left_misc = vec![
            tiles.insert_pane(default_valves_plot()),
            tiles.insert_pane(default_masses_plot()),
            tiles.insert_pane(default_kalman_plot()),
        ];

        let left = vec![
            tiles.insert_pane(default_altitude_plot()),
            tiles.insert_pane(default_vertical_speed_plot()),
            tiles.insert_tab_tile(left_misc),
        ];

        let overview = vec![
            tiles.insert_pane(default_orientation_plot()),
            tiles.insert_pane(default_temperatures_plot()),
            tiles.insert_pane(default_pressures_plot()),
            tiles.insert_pane(default_power_plot()),
            tiles.insert_pane(default_runtime_plot()),
            tiles.insert_pane(default_signal_plot()),
        ];

        let raw_sensors = vec![
            tiles.insert_pane(default_gyroscope_plot()),
            tiles.insert_pane(default_accelerometer_plot()),
            tiles.insert_pane(default_magnetometer_plot()),
            tiles.insert_pane(default_pressures_plot()),
        ];

        let right_misc = vec![
            tiles.insert_grid_tile(overview),
            tiles.insert_vertical_tile(raw_sensors),
            //tiles.insert_pane(PlotCell::IoSensors),
        ];

        let top_right = vec![tiles.insert_pane(WidgetPane::Acs), /*tiles.insert_pane(WidgetPane::Hybrid),*/ tiles.insert_pane(WidgetPane::Map)];

        let right = vec![
            tiles.insert_horizontal_tile(top_right),
            tiles.insert_tab_tile(right_misc),
        ];

        let children = vec![tiles.insert_vertical_tile(left), tiles.insert_vertical_tile(right)];

        let root = tiles.insert_horizontal_tile(children);
        egui_tiles::Tree::new("plot_tree", root, tiles)
    }

    fn tree_tabs() -> egui_tiles::Tree<WidgetPane> {
        egui_tiles::Tree::new_tabs(
            "plot_tree",
            vec![
                default_altitude_plot(),
                default_vertical_speed_plot(),
                default_orientation_plot(),
                default_valves_plot(),
                default_masses_plot(),
                default_kalman_plot(),
                default_runtime_plot(),
                default_temperatures_plot(),
                default_power_plot(),
                default_signal_plot(),
                default_gyroscope_plot(),
                default_accelerometer_plot(),
                default_magnetometer_plot(),
                default_pressures_plot(),
                WidgetPane::Map,
                WidgetPane::Acs,
                WidgetPane::Hybrid,
            ],
        )
    }

    fn tree_from_tiles(tiles: egui_tiles::Tiles<WidgetPane>) -> egui_tiles::Tree<WidgetPane> {
        let root = tiles.iter().filter(|(id, _tile)| tiles.is_root(**id)).next().unwrap().0;
        egui_tiles::Tree::new("plot_tree", *root, tiles)
    }

    fn tree_presets() -> Vec<(&'static str, egui_tiles::Tree<WidgetPane>)> {
        vec![("Default", Self::tree_default()), ("Tabs", Self::tree_tabs())]
    }

    fn same_topology(&self, tiles: &egui_tiles::Tiles<WidgetPane>) -> bool {
        // remove all the additional info to allow comparison
        // TODO: find a better way
        let serialized = serde_json::to_string(&self.tile_tree.tiles).unwrap();
        let deserialized: egui_tiles::Tiles<WidgetPane> = serde_json::from_str(&serialized).unwrap();
        deserialized == *tiles
    }

    pub fn main_ui(&mut self, ctx: &egui::Context, backend: &mut Backend, settings: &mut AppSettings, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        if self.show_view_settings {
            SidePanel::right("dock_view_settings").show(ctx, |ui| {
                if !enabled {
                    ui.disable();
                }

                ui.vertical(|ui| {
                    ui.set_width(ui.available_width());
                    ui.checkbox(&mut self.global_widget_state.shared_plot.borrow_mut().show_stats, "Show Stats");
                    ui.separator();
                    ui.weak("Presets");

                    for (name, tree) in Self::tree_presets() {
                        if ui
                            .add_sized(
                                Vec2::new(ui.available_width(), 1.0),
                                SelectableLabel::new(self.same_topology(&tree.tiles), name),
                            )
                            .clicked()
                        {
                            self.tile_tree = tree;
                        }
                    }

                    ui.separator();
                    ui.weak("Custom");

                    let mut changed = false;
                    if let Some(saved) = settings.tile_presets.as_mut() {
                        for (name, tiles) in saved.clone() {
                            ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                                if ui.button("🗑").clicked() {
                                    saved.remove(&name);
                                    changed = true;
                                }

                                if ui
                                    .add_sized(
                                        Vec2::new(ui.available_width(), 1.0),
                                        SelectableLabel::new(self.same_topology(&tiles), name),
                                    )
                                    .clicked()
                                {
                                    self.tile_tree = Self::tree_from_tiles(tiles.clone());
                                }
                            });
                        }
                    }

                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        if ui.button("Save").clicked() {
                            settings
                                .tile_presets
                                .get_or_insert(std::collections::HashMap::new())
                                .insert(self.new_preset_name.take(), self.tile_tree.tiles.clone());
                            changed = true;
                        }

                        ui.add(TextEdit::singleline(&mut self.new_preset_name));
                    });

                    if changed {
                        if let Err(e) = settings.save() {
                            log::error!("Failed to save settings: {:?}", e);
                        }
                    }
                })
            });
        }

        self.global_widget_state.shared_plot.borrow_mut().set_end(backend.end());

        CentralPanel::default().show(ctx, |ui| {
            if !enabled {
                ui.disable();
            }

            let mut behavior = TileBehavior {
                backend,
                global_widget_state: &mut self.global_widget_state,
                settings,
            };

            self.tile_tree.ui(&mut behavior, ui);
        });
    }

    pub fn bottom_bar_ui(&mut self, ui: &mut egui::Ui) {
        ui.toggle_value(&mut self.show_view_settings, "⚙ View Settings");
    }
}
