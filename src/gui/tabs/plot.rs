use std::cell::RefCell;
use std::rc::Rc;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use egui::Color32;

use crate::data_source::DataSource;
use crate::settings::AppSettings;
use crate::state::VehicleState;

use crate::gui::map::*;
use crate::gui::maxi_grid::*;
use crate::gui::misc::*;
use crate::gui::plot::*;

#[derive(Debug, Clone, Copy, PartialEq)]
enum SelectedPlot {
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

impl std::fmt::Display for SelectedPlot {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            SelectedPlot::Orientation    => write!(f, "Orientation"),
            SelectedPlot::VerticalSpeed  => write!(f, "Vertical Speed & Accel."),
            SelectedPlot::Altitude       => write!(f, "Altitude"),
            SelectedPlot::Gyroscope      => write!(f, "Gyroscope"),
            SelectedPlot::Accelerometers => write!(f, "Accelerometers"),
            SelectedPlot::Magnetometer   => write!(f, "Magnetometer"),
            SelectedPlot::Pressures      => write!(f, "Pressures"),
            SelectedPlot::Temperatures   => write!(f, "Temperatures"),
            SelectedPlot::Power          => write!(f, "Power"),
            SelectedPlot::Runtime        => write!(f, "Runtime"),
            SelectedPlot::Signal         => write!(f, "Signal"),
            SelectedPlot::Map            => write!(f, "Map"),
        }
    }
}

pub struct PlotTab {
    maxi_grid_state: MaxiGridState,
    dropdown_selected_plot: SelectedPlot,

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
    pub fn init(settings: &AppSettings) -> Self {
        let shared_plot = Rc::new(RefCell::new(SharedPlotState::new()));

        let r = Color32::from_rgb(0xfb, 0x49, 0x34);
        let g = Color32::from_rgb(0xb8, 0xbb, 0x26);
        let b = Color32::from_rgb(0x83, 0xa5, 0x98);
        let r1 = Color32::from_rgb(0xcc, 0x24, 0x1d);
        let g1 = Color32::from_rgb(0x98, 0x97, 0x1a);
        let b1 = Color32::from_rgb(0x45, 0x85, 0x88);
        let o = Color32::from_rgb(0xfa, 0xbd, 0x2f);
        let o1 = Color32::from_rgb(0xd6, 0x5d, 0x0e);
        let br = Color32::from_rgb(0x61, 0x48, 0x1c);
        let p = Color32::from_rgb(0xb1, 0x62, 0x86);
        let c = Color32::from_rgb(0x68, 0x9d, 0x6a);

        let orientation_plot = PlotState::new("Orientation", (Some(-180.0), Some(180.0)), shared_plot.clone())
            .line("Roll (Z) [°]", b, |vs| vs.euler_angles().map(|a| a.z))
            .line("Pitch (X) [°]", r, |vs| vs.euler_angles().map(|a| a.x))
            .line("Yaw (Y) [°]", g, |vs| vs.euler_angles().map(|a| a.y))
            .line("True Roll (Z) [°]", b1, |vs| vs.true_euler_angles().map(|a| a.z))
            .line("True Pitch (X) [°]", r1, |vs| vs.true_euler_angles().map(|a| a.x))
            .line("True Yaw (Y) [°]", g1, |vs| vs.true_euler_angles().map(|a| a.y));

        let vertical_speed_plot = PlotState::new("Vert. Speed & Accel.", (None, None), shared_plot.clone())
            .line("Vertical Accel [m/s²]", o1, |vs| vs.vertical_accel)
            .line("Vertical Accel (Filt.) [m/s²]", o, |vs| vs.vertical_accel_filtered)
            .line("Vario [m/s]", b, |vs| vs.vertical_speed)
            .line("True Vertical Accel [m/s²]", g, |vs| vs.true_vertical_accel)
            .line("True Vario [m/s]", b1, |vs| vs.true_vertical_speed);

        let altitude_plot = PlotState::new("Altitude (ASL)", (None, None), shared_plot.clone())
            .line("Altitude (Ground) [m]", br, |vs| vs.altitude_ground)
            .line("Altitude (Baro) [m]", b1, |vs| vs.altitude_baro)
            .line("Altitude [m]", b, |vs| vs.altitude)
            .line("Altitude (GPS) [m]", g, |vs| vs.altitude_gps);

        let gyroscope_plot = PlotState::new("Gyroscope", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Gyro (X) [°/s]", r, |vs| vs.gyroscope.map(|a| a.x))
            .line("Gyro (Y) [°/s]", g, |vs| vs.gyroscope.map(|a| a.y))
            .line("Gyro (Z) [°/s]", b, |vs| vs.gyroscope.map(|a| a.z));

        let accelerometer_plot = PlotState::new("Accelerometers", (Some(-10.0), Some(10.0)), shared_plot.clone())
            .line("Accel 2 (X) [m/s²]", r1, |vs| vs.accelerometer2.map(|a| a.x))
            .line("Accel 2 (Y) [m/s²]", g1, |vs| vs.accelerometer2.map(|a| a.y))
            .line("Accel 2 (Z) [m/s²]", b1, |vs| vs.accelerometer2.map(|a| a.z))
            .line("Accel 1 (X) [m/s²]", r, |vs| vs.accelerometer1.map(|a| a.x))
            .line("Accel 1 (Y) [m/s²]", g, |vs| vs.accelerometer1.map(|a| a.y))
            .line("Accel 1 (Z) [m/s²]", b, |vs| vs.accelerometer1.map(|a| a.z));

        let magnetometer_plot = PlotState::new("Magnetometer", (None, None), shared_plot.clone())
            .line("Mag (X) [µT]", r, |vs| vs.magnetometer.map(|a| a.x))
            .line("Mag (Y) [µT]", g, |vs| vs.magnetometer.map(|a| a.y))
            .line("Mag (Z) [µT]", b, |vs| vs.magnetometer.map(|a| a.z));

        let barometer_plot = PlotState::new("Pressures", (None, None), shared_plot.clone())
            .line("Barometer [bar]", c, |vs| vs.pressure_baro.map(|p| p / 1000.0))
            .line("Drogue Cartridge [bar]", r1, |vs| vs.drogue_cartridge_pressure)
            .line("Drogue Chamber [bar]", g1, |vs| vs.drogue_chamber_pressure)
            .line("Main Cartridge [bar]", r, |vs| vs.main_cartridge_pressure)
            .line("Main Chamber [bar]", g, |vs| vs.main_chamber_pressure);

        let temperature_plot = PlotState::new("Temperatures", (Some(25.0), Some(35.0)), shared_plot.clone())
            .line("Baro. Temp. [°C]", c, |vs| vs.temperature_baro)
            .line("Core Temp. [°C]", b, |vs| vs.temperature_core);

        let power_plot = PlotState::new("Power", (Some(0.0), Some(9.0)), shared_plot.clone())
            .line("Arm Voltage [V]", o, |vs| vs.arm_voltage)
            .line("Battery Voltage [V]", g, |vs| vs.battery_voltage)
            .line("Current [A]", o1, |vs| vs.current)
            .line("Charge Voltage [V]", b, |vs| vs.charge_voltage)
            .line("Breakwire Open?", r, |vs| vs.breakwire_open.map(|bw| bw.then(|| 1.0).unwrap_or(0.0)));

        let runtime_plot = PlotState::new("Runtime", (Some(0.0), Some(100.0)), shared_plot.clone())
            .line("CPU Util. [%]", o, |vs| vs.cpu_utilization.map(|u| u as f32))
            .line("Heap Util. [%]", g, |vs| vs.heap_utilization.map(|u| u as f32));

        let signal_plot = PlotState::new("Signal", (Some(-100.0), Some(10.0)), shared_plot.clone())
            .line("GCS RSSI [dBm]", b, |vs| vs.gcs_lora_rssi.map(|x| x as f32 / -2.0))
            .line("GCS Signal RSSI [dBm]", b1, |vs| vs.gcs_lora_rssi_signal.map(|x| x as f32 / -2.0))
            .line("GCS SNR [dB]", c, |vs| vs.gcs_lora_snr.map(|x| x as f32 / 4.0))
            .line("Vehicle RSSI [dBm]", p, |vs| vs.vehicle_lora_rssi.map(|x| x as f32 / -2.0))
            .line("HDOP", r, |vs| vs.hdop.map(|x| x as f32 / 100.0))
            .line("# Satellites", g, |vs| vs.num_satellites.map(|x| x as f32));

        let map = MapState::new(settings.mapbox_access_token.clone());

        Self {
            maxi_grid_state: MaxiGridState::default(),
            dropdown_selected_plot: SelectedPlot::Orientation,
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

    fn all_plots(&mut self, f: impl FnOnce(&mut PlotState) + Copy) {
        for plot in [
            &mut self.orientation_plot,
            &mut self.vertical_speed_plot,
            &mut self.altitude_plot,
            &mut self.gyroscope_plot,
            &mut self.accelerometer_plot,
            &mut self.magnetometer_plot,
            &mut self.barometer_plot,
            &mut self.temperature_plot,
            &mut self.power_plot,
            &mut self.runtime_plot,
            &mut self.signal_plot,
        ] {
            (f)(plot);
        }
    }

    pub fn reset(&mut self, keep_position: bool) {
        let now = Instant::now();
        self.all_plots(|plot| plot.reset(now, keep_position));
        self.map.reset();
    }

    pub fn show_all(&mut self) {
        self.all_plots(|plot| plot.show_all());
    }

    pub fn push_vehicle_state(&mut self, time: &Instant, vs: &VehicleState) {
        self.all_plots(|plot| plot.push(*time, vs));
        self.map.push(*time, vs);
    }

    pub fn main_ui(&mut self, ui: &mut egui::Ui, data_source: &mut dyn DataSource) {
        self.shared_plot.borrow_mut().set_end(data_source.end());

        if ui.available_width() > 1000.0 {
            MaxiGrid::new((4, 3), ui, self.maxi_grid_state.clone())
                .cell("Orientation", |ui| ui.plot_telemetry(&self.orientation_plot))
                .cell("Vert. Speed & Accel", |ui| ui.plot_telemetry(&self.vertical_speed_plot))
                .cell("Altitude (ASL)", |ui| ui.plot_telemetry(&self.altitude_plot))
                .cell("Position", |ui| ui.map(&self.map))
                .cell("Gyroscope", |ui| ui.plot_telemetry(&self.gyroscope_plot))
                .cell("Accelerometers", |ui| ui.plot_telemetry(&self.accelerometer_plot))
                .cell("Magnetometer", |ui| ui.plot_telemetry(&self.magnetometer_plot))
                .cell("Pressures", |ui| ui.plot_telemetry(&self.barometer_plot))
                .cell("Temperature", |ui| ui.plot_telemetry(&self.temperature_plot))
                .cell("Power", |ui| ui.plot_telemetry(&self.power_plot))
                .cell("Runtime", |ui| ui.plot_telemetry(&self.runtime_plot))
                .cell("Signal", |ui| ui.plot_telemetry(&self.signal_plot));
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
                                SelectedPlot::Gyroscope,
                                SelectedPlot::Accelerometers,
                                SelectedPlot::Magnetometer,
                                SelectedPlot::Pressures,
                                SelectedPlot::Temperatures,
                                SelectedPlot::Power,
                                SelectedPlot::Runtime,
                                SelectedPlot::Signal,
                                SelectedPlot::Map
                            ] {
                                ui.selectable_value(&mut self.dropdown_selected_plot, p, format!("{}", p));
                            }
                        });
                });

                match self.dropdown_selected_plot {
                    SelectedPlot::Orientation    => ui.plot_telemetry(&self.orientation_plot),
                    SelectedPlot::VerticalSpeed  => ui.plot_telemetry(&self.vertical_speed_plot),
                    SelectedPlot::Altitude       => ui.plot_telemetry(&self.altitude_plot),
                    SelectedPlot::Gyroscope      => ui.plot_telemetry(&self.gyroscope_plot),
                    SelectedPlot::Accelerometers => ui.plot_telemetry(&self.accelerometer_plot),
                    SelectedPlot::Magnetometer   => ui.plot_telemetry(&self.magnetometer_plot),
                    SelectedPlot::Pressures      => ui.plot_telemetry(&self.barometer_plot),
                    SelectedPlot::Temperatures   => ui.plot_telemetry(&self.temperature_plot),
                    SelectedPlot::Power          => ui.plot_telemetry(&self.power_plot),
                    SelectedPlot::Runtime        => ui.plot_telemetry(&self.runtime_plot),
                    SelectedPlot::Signal         => ui.plot_telemetry(&self.signal_plot),
                    SelectedPlot::Map            => ui.map(&self.map),
                }
            });
        }
    }

    pub fn bottom_bar_ui(&mut self, ui: &mut egui::Ui, _data_source: &mut dyn DataSource) {
        ui.toggle_button(&mut self.shared_plot.borrow_mut().show_stats, "📈 Show Stats", "📉 Hide Stats");
    }

    pub fn apply_settings(&mut self, settings: &AppSettings) {
        self.map.set_access_token(settings.mapbox_access_token.clone());
    }
}
