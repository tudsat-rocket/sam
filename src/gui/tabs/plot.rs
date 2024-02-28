use std::cell::RefCell;
use std::rc::Rc;

use egui::Align;
use egui::Layout;
use egui::SelectableLabel;
use egui::TextBuffer;
use egui::TextEdit;
use serde::{Deserialize, Serialize};
use egui::CentralPanel;
use egui::Color32;
use egui::Rect;
use egui::SidePanel;
use egui::Stroke;
use egui::Vec2;
use egui_gizmo::Gizmo;
use egui_gizmo::GizmoMode;
use egui_gizmo::GizmoVisuals;
use egui_tiles::SimplificationOptions;
use nalgebra::UnitQuaternion;
use nalgebra::Vector3;

use crate::data_source::DataSource;
use crate::settings::AppSettings;

use crate::gui::map::*;
use crate::gui::plot::*;

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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PlotCell {
    Orientation,
    VerticalSpeed,
    Altitude,
    Gyroscope,
    Accelerometers,
    Magnetometer,
    Pressures,
    Temperatures,
    Power,
    Runtime,
    Signal,
    Map,
}

impl PlotCell {
    fn all() -> Vec<Self> {
        vec![
            PlotCell::Orientation,
            PlotCell::VerticalSpeed,
            PlotCell::Altitude,
            PlotCell::Map,
            PlotCell::Gyroscope,
            PlotCell::Accelerometers,
            PlotCell::Magnetometer,
            PlotCell::Pressures,
            PlotCell::Temperatures,
            PlotCell::Power,
            PlotCell::Runtime,
            PlotCell::Signal,
        ]
    }
}

impl std::fmt::Display for PlotCell {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PlotCell::Orientation    => write!(f, "Orientation"),
            PlotCell::VerticalSpeed  => write!(f, "Vertical Speed & Accel."),
            PlotCell::Altitude       => write!(f, "Altitude (ASL)"),
            PlotCell::Gyroscope      => write!(f, "Gyroscope"),
            PlotCell::Accelerometers => write!(f, "Accelerometers"),
            PlotCell::Magnetometer   => write!(f, "Magnetometer"),
            PlotCell::Pressures      => write!(f, "Pressures"),
            PlotCell::Temperatures   => write!(f, "Temperatures"),
            PlotCell::Power          => write!(f, "Power"),
            PlotCell::Runtime        => write!(f, "Runtime"),
            PlotCell::Signal         => write!(f, "Signal"),
            PlotCell::Map            => write!(f, "Map"),
        }
    }
}

struct TileBehavior<'a> {
    data_source: &'a mut dyn DataSource,
    orientation_plot: &'a mut PlotState,
    vertical_speed_plot: &'a mut PlotState,
    altitude_plot: &'a mut PlotState,
    gyroscope_plot: &'a mut PlotState,
    accelerometer_plot: &'a mut PlotState,
    magnetometer_plot: &'a mut PlotState,
    barometer_plot: &'a mut PlotState,
    temperature_plot: &'a mut PlotState,
    power_plot: &'a mut PlotState,
    runtime_plot: &'a mut PlotState,
    signal_plot: &'a mut PlotState,
    map: &'a mut MapState,
}

// TODO: move these somewhere else
impl<'a> TileBehavior<'a> {
    fn plot_gizmo(
        &mut self,
        ui: &mut egui::Ui,
        viewport: Rect,
        orientation: UnitQuaternion<f32>,
        colors: (Color32, Color32, Color32)
    ) {
        // We can't disable interaction with the gizmo, so we disable the entire UI
        // when the user gets too close. TODO: upstream way to disable interaction?
        let enabled = !ui.rect_contains_pointer(viewport);

        // use top right of plot for indicator, space below for plot
        let viewport = Rect::from_two_pos(viewport.lerp_inside(Vec2::new(0.55, 0.55)), viewport.right_top());

        let fade_to_color = Color32::BLACK;
        ui.visuals_mut().widgets.noninteractive.weak_bg_fill = fade_to_color;

        // square viewport
        let viewport_square_side = f32::min(viewport.width(), viewport.height());
        let viewport = viewport.shrink2((viewport.size() - Vec2::splat(viewport_square_side))*0.5);

        let view = UnitQuaternion::from_euler_angles(-90.0f32.to_radians(), 180f32.to_radians(), 0.0f32.to_radians());

        let visuals = GizmoVisuals {
            x_color: colors.0,
            y_color: colors.1,
            z_color: colors.2,
            inactive_alpha: 1.0,
            highlight_alpha: 1.0,
            gizmo_size: viewport_square_side * 0.4,
            ..Default::default()
        };

        let gizmo = Gizmo::new("My gizmo")
            .mode(GizmoMode::Translate)
            .viewport(viewport)
            .orientation(egui_gizmo::GizmoOrientation::Local)
            .model_matrix(orientation.to_homogeneous().into())
            .view_matrix(view.to_homogeneous().into())
            .visuals(visuals);

        ui.add_enabled_ui(enabled, |ui| {
            gizmo.interact(ui);
        });
    }

    fn plot_orientation(&mut self, ui: &mut egui::Ui) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        let mut viewport = ui.cursor();
        viewport.set_width(ui.available_width());
        viewport.set_height(ui.available_height());

        let orientation = self.data_source.vehicle_states()
            .rev()
            .find_map(|(_, vs)| vs.orientation)
            .unwrap_or(UnitQuaternion::new(Vector3::new(0.0, 0.0, 0.0)));
        let true_orientation = self.data_source.vehicle_states()
            .rev()
            .find_map(|(_, vs)| vs.true_orientation);

        ui.plot_telemetry(&self.orientation_plot, self.data_source);

        if let Some(orientation) = true_orientation {
            self.plot_gizmo(ui, viewport, orientation, (R1, G1, B1));
        }

        self.plot_gizmo(ui, viewport, orientation, (R, G, B));
    }
}

impl<'a> egui_tiles::Behavior<PlotCell> for TileBehavior<'a> {
    fn tab_title_for_pane(&mut self, pane: &PlotCell) -> egui::WidgetText {
        format!("{}", pane).into()
    }

    fn pane_ui(&mut self, ui: &mut egui::Ui, _tile_id: egui_tiles::TileId, pane: &mut PlotCell) -> egui_tiles::UiResponse {
        match pane {
            PlotCell::Orientation    => self.plot_orientation(ui),
            PlotCell::VerticalSpeed  => ui.plot_telemetry(&self.vertical_speed_plot, self.data_source),
            PlotCell::Altitude       => ui.plot_telemetry(&self.altitude_plot, self.data_source),
            PlotCell::Gyroscope      => ui.plot_telemetry(&self.gyroscope_plot, self.data_source),
            PlotCell::Accelerometers => ui.plot_telemetry(&self.accelerometer_plot, self.data_source),
            PlotCell::Magnetometer   => ui.plot_telemetry(&self.magnetometer_plot, self.data_source),
            PlotCell::Pressures      => ui.plot_telemetry(&self.barometer_plot, self.data_source),
            PlotCell::Temperatures   => ui.plot_telemetry(&self.temperature_plot, self.data_source),
            PlotCell::Power          => ui.plot_telemetry(&self.power_plot, self.data_source),
            PlotCell::Runtime        => ui.plot_telemetry(&self.runtime_plot, self.data_source),
            PlotCell::Signal         => ui.plot_telemetry(&self.signal_plot, self.data_source),
            PlotCell::Map            => { ui.add(Map::new(&mut self.map, self.data_source)); },
        }

        egui_tiles::UiResponse::None
    }

    fn dragged_overlay_color(&self, _visuals: &egui::Visuals) -> Color32 {
        Color32::from_rgb(0xb8, 0xbb, 0x26).gamma_multiply(0.5)
    }

    fn drag_preview_stroke(&self, _visuals: &egui::Visuals) -> egui::Stroke {
        Stroke { width: 1.0, color: Color32::from_rgb(0xb8, 0xbb, 0x26) }
    }

    fn drag_preview_color(&self, _visuals: &egui::Visuals) -> Color32 {
        Color32::from_rgb(0xb8, 0xbb, 0x26).gamma_multiply(0.5)
    }

    fn tab_bar_color(&self, visuals: &egui::Visuals) -> Color32 {
        visuals.widgets.noninteractive.bg_fill
    }

    fn tab_bg_color(&self, visuals: &egui::Visuals, _tiles: &egui_tiles::Tiles<PlotCell>, _tile_id: egui_tiles::TileId, active: bool) -> Color32 {
        if active { self.tab_bar_color(visuals) } else { visuals.extreme_bg_color }
    }

    fn tab_text_color(&self, visuals: &egui::Visuals, _tiles: &egui_tiles::Tiles<PlotCell>, _tile_id: egui_tiles::TileId, active: bool) -> Color32 {
        if active { visuals.widgets.active.fg_stroke.color } else { visuals.widgets.hovered.fg_stroke.color }
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

pub struct PlotTab {
    tile_tree: egui_tiles::Tree<PlotCell>,
    show_view_settings: bool,
    new_preset_name: String,

    shared_plot: Rc<RefCell<SharedPlotState>>,
    orientation_plot: PlotState,
    vertical_speed_plot: PlotState,
    altitude_plot: PlotState,
    gyroscope_plot: PlotState,
    accelerometer_plot: PlotState,
    magnetometer_plot: PlotState,
    barometer_plot: PlotState,
    temperature_plot: PlotState,
    power_plot: PlotState,
    runtime_plot: PlotState,
    signal_plot: PlotState,
    map: MapState,
}

impl PlotTab {
    pub fn init(ctx: &egui::Context, settings: &AppSettings) -> Self {
        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));

        let orientation_plot = PlotState::new("Orientation", (Some(-180.0), Some(540.0)), shared_plot.clone())
            .line("Roll (Z) [°]", B, |vs| vs.euler_angles.map(|a| a.z))
            .line("Pitch (X) [°]", R, |vs| vs.euler_angles.map(|a| a.x))
            .line("Yaw (Y) [°]", G, |vs| vs.euler_angles.map(|a| a.y))
            .line("Angle of Attack [°]", O, |vs| vs.angle_of_attack)
            .line("Roll (True) (Z) [°]", B1, |vs| vs.true_euler_angles.map(|a| a.z))
            .line("Pitch (True) (X) [°]", R1, |vs| vs.true_euler_angles.map(|a| a.x))
            .line("Yaw (True) (Y) [°]", G1, |vs| vs.true_euler_angles.map(|a| a.y))
            .line("Angle of Attack (True) [°]", O1, |vs| vs.true_angle_of_attack);

        let vertical_speed_plot = PlotState::new("Vert. Speed & Accel.", (None, None), shared_plot.clone())
            .line("Vertical Accel [m/s²]", O1, |vs| vs.vertical_accel)
            .line("Vertical Accel (Filt.) [m/s²]", O, |vs| vs.vertical_accel_filtered)
            .line("Vario [m/s]", B, |vs| vs.vertical_speed)
            .line("True Vertical Accel [m/s²]", G, |vs| vs.true_vertical_accel)
            .line("True Vario [m/s]", B1, |vs| vs.true_vertical_speed);

        let altitude_plot = PlotState::new("Altitude (ASL)", (None, None), shared_plot.clone())
            .line("Altitude (Ground) [m]", BR, |vs| vs.altitude_ground_asl)
            .line("Altitude (Baro) [m]", B1, |vs| vs.altitude_baro)
            .line("Altitude [m]", B, |vs| vs.altitude_asl)
            .line("Altitude (GPS) [m]", G, |vs| vs.altitude_gps_asl);

        let gyroscope_plot = PlotState::new("Gyroscope", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Gyro (X) [°/s]", R, |vs| vs.gyroscope.map(|a| a.x))
            .line("Gyro (Y) [°/s]", G, |vs| vs.gyroscope.map(|a| a.y))
            .line("Gyro (Z) [°/s]", B, |vs| vs.gyroscope.map(|a| a.z));

        let accelerometer_plot = PlotState::new("Accelerometers", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Accel 2 (X) [m/s²]", R1, |vs| vs.accelerometer2.map(|a| a.x))
            .line("Accel 2 (Y) [m/s²]", G1, |vs| vs.accelerometer2.map(|a| a.y))
            .line("Accel 2 (Z) [m/s²]", B1, |vs| vs.accelerometer2.map(|a| a.z))
            .line("Accel 1 (X) [m/s²]", R, |vs| vs.accelerometer1.map(|a| a.x))
            .line("Accel 1 (Y) [m/s²]", G, |vs| vs.accelerometer1.map(|a| a.y))
            .line("Accel 1 (Z) [m/s²]", B, |vs| vs.accelerometer1.map(|a| a.z));

        let magnetometer_plot = PlotState::new("Magnetometer", (None, None), shared_plot.clone())
            .line("Mag (X) [µT]", R, |vs| vs.magnetometer.map(|a| a.x))
            .line("Mag (Y) [µT]", G, |vs| vs.magnetometer.map(|a| a.y))
            .line("Mag (Z) [µT]", B, |vs| vs.magnetometer.map(|a| a.z));

        let barometer_plot = PlotState::new("Pressures", (None, None), shared_plot.clone())
            .line("Barometer [bar]", C, |vs| vs.pressure_baro.map(|p| p / 1000.0));

        let temperature_plot = PlotState::new("Temperatures", (Some(25.0), Some(35.0)), shared_plot.clone())
            .line("Baro. Temp. [°C]", C, |vs| vs.temperature_baro);

        let power_plot = PlotState::new("Power", (Some(0.0), Some(9.0)), shared_plot.clone())
            .line("Arm Voltage [V]", O, |vs| vs.arm_voltage.map(|v| (v as f32) / 1000.0))
            .line("Battery Voltage [V]", G, |vs| vs.battery_voltage.map(|v| (v as f32) / 1000.0))
            .line("Current [A]", O1, |vs| vs.current.map(|v| (v as f32) / 1000.0))
            .line("Charge Voltage [V]", B, |vs| vs.charge_voltage.map(|v| (v as f32) / 1000.0));

        let runtime_plot = PlotState::new("Runtime", (Some(0.0), Some(100.0)), shared_plot.clone())
            .line("CPU Util. [%]", O, |vs| vs.cpu_utilization);

        let signal_plot = PlotState::new("Signal", (Some(-100.0), Some(10.0)), shared_plot.clone())
            .line("GCS RSSI [dBm]", B, |vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))
            .line("GCS Signal RSSI [dBm]", B1, |vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))
            .line("GCS SNR [dB]", C, |vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))
            .line("Vehicle RSSI [dBm]", P, |vs| vs.lora_rssi.map(|x| x as f32 / -2.0))
            .line("HDOP", R, |vs| vs.hdop.map(|x| x as f32 / 100.0))
            .line("# Satellites", G, |vs| vs.num_satellites.map(|x| x as f32));

        let map = MapState::new(ctx, (!settings.mapbox_access_token.is_empty()).then_some(settings.mapbox_access_token.clone()));

        Self {
            tile_tree: Self::tree_grid(),
            show_view_settings: false,
            new_preset_name: String::new(),
            shared_plot,
            orientation_plot,
            vertical_speed_plot,
            altitude_plot,
            gyroscope_plot,
            accelerometer_plot,
            magnetometer_plot,
            barometer_plot,
            temperature_plot,
            power_plot,
            runtime_plot,
            signal_plot,
            map,
        }
    }

    fn tree_grid() -> egui_tiles::Tree<PlotCell> {
        egui_tiles::Tree::new_grid("plot_tree", PlotCell::all())
    }

    fn tree_from_tiles(tiles: egui_tiles::Tiles<PlotCell>) -> egui_tiles::Tree<PlotCell> {
        let root = tiles.iter()
            .filter(|(id, _tile)| tiles.is_root(**id))
            .next()
            .unwrap().0;
        egui_tiles::Tree::new("plot_tree", *root, tiles)
    }

    fn tree_presets() -> Vec<(&'static str, egui_tiles::Tree<PlotCell>)> {
        vec![
            ("Grid", egui_tiles::Tree::new_grid("plot_tree", PlotCell::all())),
            ("Tabs", egui_tiles::Tree::new_tabs("plot_tree", PlotCell::all())),
        ]
    }

    fn same_topology(&self, tiles: &egui_tiles::Tiles<PlotCell>) -> bool {
        // remove all the additional info to allow comparison
        // TODO: find a better way
        let serialized = serde_json::to_string(&self.tile_tree.tiles).unwrap();
        let deserialized: egui_tiles::Tiles<PlotCell> = serde_json::from_str(&serialized).unwrap();
        deserialized == *tiles
    }

    pub fn main_ui(&mut self, ctx: &egui::Context, data_source: &mut dyn DataSource, settings: &mut AppSettings, enabled: bool) {
        #[cfg(feature = "profiling")]
        puffin::profile_function!();

        if self.show_view_settings {
            SidePanel::right("dock_view_settings").show(ctx, |ui| {
                ui.set_enabled(enabled);

                ui.vertical(|ui| {
                    ui.set_width(ui.available_width());
                    ui.checkbox(&mut self.shared_plot.borrow_mut().show_stats, "Show Stats");
                    ui.separator();
                    ui.weak("Presets");

                    for (name, tree) in Self::tree_presets() {
                        if ui.add_sized(Vec2::new(ui.available_width(), 1.0), SelectableLabel::new(self.same_topology(&tree.tiles), name)).clicked() {
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

                                if ui.add_sized(Vec2::new(ui.available_width(), 1.0), SelectableLabel::new(self.same_topology(&tiles), name)).clicked() {
                                    self.tile_tree = Self::tree_from_tiles(tiles.clone());
                                }
                            });
                        }
                    }

                    ui.with_layout(Layout::right_to_left(Align::TOP), |ui| {
                        if ui.button("Save").clicked() {
                            settings.tile_presets
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

        self.shared_plot.borrow_mut().set_end(data_source.end());

        CentralPanel::default().show(ctx, |ui| {
            ui.set_enabled(enabled);

            let mut behavior = TileBehavior {
                data_source,
                orientation_plot: &mut self.orientation_plot,
                vertical_speed_plot: &mut self.vertical_speed_plot,
                altitude_plot: &mut self.altitude_plot,
                gyroscope_plot: &mut self.gyroscope_plot,
                accelerometer_plot: &mut self.accelerometer_plot,
                magnetometer_plot: &mut self.magnetometer_plot,
                barometer_plot: &mut self.barometer_plot,
                temperature_plot: &mut self.temperature_plot,
                power_plot: &mut self.power_plot,
                runtime_plot: &mut self.runtime_plot,
                signal_plot: &mut self.signal_plot,
                map: &mut self.map,
            };
            self.tile_tree.ui(&mut behavior, ui);
        });
    }

    pub fn bottom_bar_ui(&mut self, ui: &mut egui::Ui, _data_source: &mut dyn DataSource) {
        ui.toggle_value(&mut self.show_view_settings, "⚙ View Settings");
    }
}
