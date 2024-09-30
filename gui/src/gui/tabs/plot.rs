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
use egui::SidePanel;
use egui::Stroke;
use egui::Vec2;
use egui_tiles::SimplificationOptions;

use shared_types::telemetry::FlightMode;
use shared_types::IoBoardRole;

use crate::data_source::DataSource;
use crate::gui::acs::AcsSystemDiagram;
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
    Kalman,
    Valves,
    Masses,
    IoSensors,
    FinData,
    Acs,
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
            PlotCell::Kalman,
            PlotCell::Valves,
            PlotCell::Masses,
            PlotCell::IoSensors,
            PlotCell::FinData,
            PlotCell::Acs,
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
            PlotCell::Kalman         => write!(f, "Kalman"),
            PlotCell::Valves         => write!(f, "Valves"),
            PlotCell::Masses         => write!(f, "Masses"),
            PlotCell::IoSensors      => write!(f, "IO Board Sensors"),
            PlotCell::FinData        => write!(f, "Fin Data"),
            PlotCell::Acs            => write!(f, "ACS"),
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
    pressures_plot: &'a mut PlotState,
    temperature_plot: &'a mut PlotState,
    power_plot: &'a mut PlotState,
    runtime_plot: &'a mut PlotState,
    signal_plot: &'a mut PlotState,
    kalman_plot: &'a mut PlotState,
    valve_plot: &'a mut PlotState,
    masses_plot: &'a mut PlotState,
    raw_sensors_plot: &'a mut PlotState,
    fin_data_plot: &'a mut PlotState,
    map: &'a mut MapState,
}

impl<'a> egui_tiles::Behavior<PlotCell> for TileBehavior<'a> {
    fn tab_title_for_pane(&mut self, pane: &PlotCell) -> egui::WidgetText {
        format!("{}", pane).into()
    }

    fn pane_ui(&mut self, ui: &mut egui::Ui, _tile_id: egui_tiles::TileId, pane: &mut PlotCell) -> egui_tiles::UiResponse {
        match pane {
            PlotCell::Orientation    => ui.plot_telemetry(&self.orientation_plot, self.data_source),
            PlotCell::VerticalSpeed  => ui.plot_telemetry(&self.vertical_speed_plot, self.data_source),
            PlotCell::Altitude       => ui.plot_telemetry(&self.altitude_plot, self.data_source),
            PlotCell::Gyroscope      => ui.plot_telemetry(&self.gyroscope_plot, self.data_source),
            PlotCell::Accelerometers => ui.plot_telemetry(&self.accelerometer_plot, self.data_source),
            PlotCell::Magnetometer   => ui.plot_telemetry(&self.magnetometer_plot, self.data_source),
            PlotCell::Pressures      => ui.plot_telemetry(&self.pressures_plot, self.data_source),
            PlotCell::Temperatures   => ui.plot_telemetry(&self.temperature_plot, self.data_source),
            PlotCell::Power          => ui.plot_telemetry(&self.power_plot, self.data_source),
            PlotCell::Runtime        => ui.plot_telemetry(&self.runtime_plot, self.data_source),
            PlotCell::Signal         => ui.plot_telemetry(&self.signal_plot, self.data_source),
            PlotCell::Kalman         => ui.plot_telemetry(&self.kalman_plot, self.data_source),
            PlotCell::Valves         => ui.plot_telemetry(&self.valve_plot, self.data_source),
            PlotCell::Masses         => ui.plot_telemetry(&self.masses_plot, self.data_source),
            PlotCell::IoSensors      => ui.plot_telemetry(&self.raw_sensors_plot, self.data_source),
            PlotCell::FinData        => ui.plot_telemetry(&self.fin_data_plot, self.data_source),
            PlotCell::Map            => { ui.add(Map::new(&mut self.map, self.data_source)); },
            PlotCell::Acs            => { ui.add(AcsSystemDiagram::new(self.data_source)); },
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
    pressures_plot: PlotState,
    temperature_plot: PlotState,
    power_plot: PlotState,
    runtime_plot: PlotState,
    signal_plot: PlotState,
    kalman_plot: PlotState,
    valve_plot: PlotState,
    masses_plot: PlotState,
    raw_sensors_plot: PlotState,
    fin_data_plot: PlotState,
    map: MapState,
}

impl PlotTab {
    pub fn init(ctx: &egui::Context, settings: &AppSettings) -> Self {
        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));

        let orientation_plot = PlotState::new("Orientation", (Some(-180.0), Some(360.0)), shared_plot.clone())
            .line("Elevation [°]", B, |vs| vs.elevation)
            .line("Azimuth [°]", R, |vs| vs.azimuth)
            .line("True Elevation [°]", B1, |vs| vs.sim.as_ref().and_then(|s| s.elevation))
            .line("True Azimuth [°]", R1, |vs| vs.sim.as_ref().and_then(|s| s.azimuth));

        let vertical_speed_plot = PlotState::new("Vert. Speed & Accel.", (None, None), shared_plot.clone())
            .line("Vertical Accel [m/s²]", O, |vs| vs.vertical_accel)
            .line("Vario [m/s]", B, |vs| vs.vertical_speed)
            .line("True Vertical Accel [m/s²]", G, |vs| vs.sim.as_ref().and_then(|s| s.vertical_accel))
            .line("True Vario [m/s]", B1, |vs| vs.sim.as_ref().and_then(|s| s.vertical_speed));

        let altitude_plot = PlotState::new("Altitude (ASL)", (None, None), shared_plot.clone())
            .line("Ground [m]", BR, |vs| vs.altitude_ground_asl)
            .line("Altitude (Baro) [m]", B1, |vs| vs.altitude_baro)
            .line("Altitude [m]", B, |vs| vs.altitude_asl)
            .line("Apogee [m]", O1, |vs| (vs.mode == Some(FlightMode::Coast)).then_some(vs.apogee_asl).flatten())
            .line("Altitude (GPS) [m]", G, |vs| vs.gps.as_ref().and_then(|gps| gps.altitude));

        let gyroscope_plot = PlotState::new("Gyroscope", (None, None), shared_plot.clone())
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

        let pressures_plot = PlotState::new("Pressures", (None, None), shared_plot.clone())
            .line("Barometer [bar]", C, |vs| vs.pressure_baro.map(|p| p / 1000.0))
            .line("ACS Tank [bar]", R, |vs| vs.acs_tank_pressure)
            .line("ACS Post-Regulator [bar]", G, |vs| vs.acs_regulator_pressure)
            .line("ACS Accel Valve [bar]", O, |vs| vs.acs_accel_valve_pressure)
            .line("ACS Decel Valve [bar]", O1, |vs| vs.acs_decel_valve_pressure)
            .line("Recovery [bar]", B, |vs| vs.recovery_pressure)
            .line("Main Release Sensor", BR, |vs| vs.main_release_sensor.map(|b| if b { 10.0 } else { 0.0 }));

        let temperature_plot = PlotState::new("Temperatures", (Some(25.0), Some(35.0)), shared_plot.clone())
            .line("Barometer [°C]", C, |vs| vs.temperature_baro)
            .line("ACS [°C]", R, |vs| vs.acs_temperature.flatten().map(|t| (t as f32) / 2.0))
            .line("Recovery [°C]", B, |vs| vs.recovery_temperature.flatten().map(|t| (t as f32) / 2.0))
            .line("Payload [°C]", O, |vs| vs.payload_temperature.flatten().map(|t| (t as f32) / 2.0));

        let power_plot = PlotState::new("Power", (Some(0.0), Some(9.0)), shared_plot.clone())
            .line("Battery Voltage [V]", G, |vs| vs.battery_voltage.map(|v| (v as f32) / 1000.0))
            .line("Battery Current [A]", O1, |vs| vs.current.map(|v| (v as f32) / 1000.0))
            .line("Charge Voltage [V]", B, |vs| vs.charge_voltage.map(|v| (v as f32) / 1000.0))
            .line("ACS Voltage [V]", G1, |vs| vs.acs_voltage.flatten().map(|v| (v as f32) / 1000.0))
            .line("ACS Current [A]", O, |vs| vs.acs_current.flatten().map(|v| (v as f32) / 1000.0))
            .line("Recovery Voltage [V]", C, |vs| vs.recovery_voltage.flatten().map(|v| (v as f32) / 1000.0))
            .line("Recovery Current [A]", R, |vs| vs.recovery_current.flatten().map(|v| (v as f32) / 1000.0))
            .line("Payload Voltage [V]", G1, |vs| vs.payload_voltage.flatten().map(|v| (v as f32) / 1000.0))
            .line("Payload Current [A]", O, |vs| vs.payload_current.flatten().map(|v| (v as f32) / 1000.0));

        let runtime_plot = PlotState::new("Runtime", (Some(0.0), Some(100.0)), shared_plot.clone())
            .line("CPU Util. [%]", O, |vs| vs.cpu_utilization);

        let signal_plot = PlotState::new("Signal", (Some(-100.0), Some(10.0)), shared_plot.clone())
            .line("GCS RSSI [dBm]", B, |vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))
            .line("GCS Signal RSSI [dBm]", B1, |vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))
            .line("GCS SNR [dB]", C, |vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))
            .line("Vehicle RSSI [dBm]", P, |vs| vs.lora_rssi.map(|x| x as f32 / -2.0))
            .line("HDOP", R, |vs| vs.gps.as_ref().map(|gps| gps.hdop as f32 / 100.0))
            .line("# Satellites", G, |vs| vs.gps.as_ref().map(|gps| gps.num_satellites as f32));

        let kalman_plot = PlotState::new("Kalman", (None, None), shared_plot.clone())
            .line("x (pos. X) [m]", R, |vs| vs.sim.as_ref().map(|s| s.kalman_x[0]))
            .line("x (pos. Y) [m]", G, |vs| vs.sim.as_ref().map(|s| s.kalman_x[1]))
            .line("x (speed X) [m]", R1, |vs| vs.sim.as_ref().map(|s| s.kalman_x[3]))
            .line("x (speed Y) [m]", G1, |vs| vs.sim.as_ref().map(|s| s.kalman_x[4]))
            .line("x (accel. X) [m]", R, |vs| vs.sim.as_ref().map(|s| s.kalman_x[6]))
            .line("x (accel. Y) [m]", G, |vs| vs.sim.as_ref().map(|s| s.kalman_x[7]))
            .line("P (pos. X/Y) [m]", R, |vs| vs.sim.as_ref().map(|s| s.kalman_P[0]))
            .line("P (speed Z) [m/s]", B1, |vs| vs.sim.as_ref().map(|s| s.kalman_P[5]))
            .line("P (accel. Z) [m/s²]", B, |vs| vs.sim.as_ref().map(|s| s.kalman_P[8]))
            .line("R (baro.) [m]", O, |vs| vs.sim.as_ref().map(|s| s.kalman_R[0]))
            .line("R (pos. X/Y) [m]", O, |vs| vs.sim.as_ref().map(|s| s.kalman_R[4]))
            .line("X/Y Pos. Variance [m]", R, |vs| vs.position_variance)
            .line("Altitude Variance [m]", B, |vs| vs.altitude_variance)
            .line("Vertical Speed Variance [m/s]", O1, |vs| vs.vertical_speed_variance)
            .line("Barometer Variance [m]", C, |vs| vs.barometer_variance)
            .line("Accelerometer Variance [m/s²]", O, |vs| vs.accelerometer_variance)
            .line("GPS Variance [m]", G, |vs| vs.gps_variance);

        let valve_plot = PlotState::new("Valves", (Some(-1.0), Some(1.0)), shared_plot.clone())
            .line("Thrusters", B1, |vs| vs.thruster_valve_state.map(|v| v.into()))
            .line("Apogee Error", R, |vs| vs.sim.as_ref().and_then(|s| s.apogee_error));

        let masses_plot = PlotState::new("Masses", (Some(0.0), None), shared_plot.clone())
            .line("Vehicle [kg]", B, |vs| vs.sim.as_ref().and_then(|sim| sim.mass))
            .line("Motor [kg]", R, |vs| vs.sim.as_ref().and_then(|sim| sim.motor_mass))
            .line("Thruster Propellant [kg]", O1, |vs| vs.sim.as_ref().and_then(|sim| sim.thruster_propellant_mass));

        // TODO: refactor
        let raw_sensors_plot = PlotState::new("Raw Sensors", (Some(0f32), Some(3300f32)), shared_plot.clone())
            .line("ACS #0 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Acs && *id == 0).then_some(msg.i2c_sensors[0]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("ACS #1 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Acs && *id == 0).then_some(msg.i2c_sensors[1]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("ACS #2 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Acs && *id == 0).then_some(msg.i2c_sensors[2]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("ACS #3 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Acs && *id == 0).then_some(msg.i2c_sensors[3]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("Recovery #0 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Recovery && *id == 0).then_some(msg.i2c_sensors[0]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("Recovery #1 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Recovery && *id == 0).then_some(msg.i2c_sensors[1]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))))
            .line("Recovery #2 [mV]", B, |vs| vs.io_board_sensor_data.as_ref().and_then(|(role, id, msg)| (*role == IoBoardRole::Recovery && *id == 0).then_some(msg.i2c_sensors[2]).flatten().map(|(v, _)| 3300f32 * v as f32 / 2f32.powi(10))));

        // TODO: refactor
        let fin_data_plot = PlotState::new("Fin Data", (None, None), shared_plot.clone())
            .line("Fin Board #0 SG 0 [8-bit]", O, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 0 && *id == 0).then_some(msg.data[0] as f32)))
            .line("Fin Board #0 SG 1 [8-bit]", G, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 0 && *id == 1).then_some(msg.data[0] as f32)))
            .line("Fin Board #1 SG 0 [8-bit]", O, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 1 && *id == 0).then_some(msg.data[0] as f32)))
            .line("Fin Board #1 SG 1 [8-bit]", G, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 1 && *id == 1).then_some(msg.data[0] as f32)))
            .line("Fin Board #2 SG 0 [8-bit]", O, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 2 && *id == 0).then_some(msg.data[0] as f32)))
            .line("Fin Board #2 SG 1 [8-bit]", G, |vs| vs.fin_board_sensor_data.as_ref().and_then(|(fin, id, msg)| (*fin == 2 && *id == 1).then_some(msg.data[0] as f32)));

        let map = MapState::new(ctx, (!settings.mapbox_access_token.is_empty()).then_some(settings.mapbox_access_token.clone()));

        Self {
            tile_tree: Self::tree_launch(),
            show_view_settings: false,
            new_preset_name: String::new(),
            shared_plot,
            orientation_plot,
            vertical_speed_plot,
            altitude_plot,
            gyroscope_plot,
            accelerometer_plot,
            magnetometer_plot,
            pressures_plot,
            temperature_plot,
            power_plot,
            runtime_plot,
            signal_plot,
            kalman_plot,
            valve_plot,
            masses_plot,
            raw_sensors_plot,
            fin_data_plot,
            map,
        }
    }

    fn tree_launch() -> egui_tiles::Tree<PlotCell> {
        let mut tiles = egui_tiles::Tiles::default();

        let left_misc = vec![
            tiles.insert_pane(PlotCell::Valves),
            tiles.insert_pane(PlotCell::Masses),
            tiles.insert_pane(PlotCell::Kalman),
        ];

        let left = vec![
            tiles.insert_pane(PlotCell::Altitude),
            tiles.insert_pane(PlotCell::VerticalSpeed),
            tiles.insert_tab_tile(left_misc),
        ];

        let t = vec![
            tiles.insert_pane(PlotCell::Pressures),
            tiles.insert_pane(PlotCell::Runtime),
        ];

        let overview = vec![
            tiles.insert_pane(PlotCell::Orientation),
            tiles.insert_pane(PlotCell::Temperatures),
            tiles.insert_pane(PlotCell::Acs),
            tiles.insert_pane(PlotCell::Power),
            tiles.insert_tab_tile(t),
            tiles.insert_pane(PlotCell::Signal),
        ];

        let raw_sensors = vec![
            tiles.insert_pane(PlotCell::Gyroscope),
            tiles.insert_pane(PlotCell::Accelerometers),
            tiles.insert_pane(PlotCell::Magnetometer),
            tiles.insert_pane(PlotCell::Pressures),
        ];

        let right_misc = vec![
            tiles.insert_grid_tile(overview),
            tiles.insert_vertical_tile(raw_sensors),
            tiles.insert_pane(PlotCell::IoSensors),
            tiles.insert_pane(PlotCell::FinData),
        ];

        let right = vec![
            tiles.insert_pane(PlotCell::Map),
            tiles.insert_tab_tile(right_misc),
        ];

        let children = vec![
            tiles.insert_vertical_tile(left),
            tiles.insert_vertical_tile(right),
        ];

        let root = tiles.insert_horizontal_tile(children);
        egui_tiles::Tree::new("plot_tree", root, tiles)
    }


    fn tree_grid() -> egui_tiles::Tree<PlotCell> {
        let mut tiles = egui_tiles::Tiles::default();
        let t1 = vec![
            tiles.insert_pane(PlotCell::Acs),
            tiles.insert_pane(PlotCell::Pressures),
            tiles.insert_pane(PlotCell::Valves),
            tiles.insert_pane(PlotCell::IoSensors),
            tiles.insert_pane(PlotCell::FinData),
        ];
        let t2 = vec![
            tiles.insert_pane(PlotCell::Runtime),
            tiles.insert_pane(PlotCell::Kalman),
        ];
        let t3 = vec![
            tiles.insert_pane(PlotCell::Signal),
            tiles.insert_pane(PlotCell::Masses),
        ];
        let children = vec![
            tiles.insert_pane(PlotCell::Orientation),
            tiles.insert_pane(PlotCell::VerticalSpeed),
            tiles.insert_pane(PlotCell::Altitude),
            tiles.insert_pane(PlotCell::Map),
            tiles.insert_pane(PlotCell::Gyroscope),
            tiles.insert_pane(PlotCell::Accelerometers),
            tiles.insert_pane(PlotCell::Magnetometer),
            tiles.insert_tab_tile(t1),
            tiles.insert_pane(PlotCell::Temperatures),
            tiles.insert_pane(PlotCell::Power),
            tiles.insert_tab_tile(t2),
            tiles.insert_tab_tile(t3),
        ];
        let grid = tiles.insert_grid_tile(children);
        egui_tiles::Tree::new("plot_tree", grid, tiles)
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
            ("Launch", Self::tree_launch()),
            ("Grid", Self::tree_grid()),
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
                pressures_plot: &mut self.pressures_plot,
                temperature_plot: &mut self.temperature_plot,
                power_plot: &mut self.power_plot,
                runtime_plot: &mut self.runtime_plot,
                signal_plot: &mut self.signal_plot,
                kalman_plot: &mut self.kalman_plot,
                valve_plot: &mut self.valve_plot,
                masses_plot: &mut self.masses_plot,
                raw_sensors_plot: &mut self.raw_sensors_plot,
                fin_data_plot: &mut self.fin_data_plot,
                map: &mut self.map,
            };
            self.tile_tree.ui(&mut behavior, ui);
        });
    }

    pub fn bottom_bar_ui(&mut self, ui: &mut egui::Ui, _data_source: &mut dyn DataSource) {
        ui.toggle_value(&mut self.show_view_settings, "⚙ View Settings");
    }
}
