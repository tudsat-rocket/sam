use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand_distr::Distribution;

use crate::{SimulationState, SimulationSettings, GRAVITY};

#[derive(Clone, Debug, PartialEq)]
pub struct SensorSettings {
    pub std_dev_gyroscope: f32,
    pub std_dev_accelerometer1: f32,
    pub std_dev_accelerometer2: f32,
    pub std_dev_magnetometer: f32,
    pub std_dev_barometer: f32,
    // TODO: anomalies?
}

impl Default for SensorSettings {
    fn default() -> Self {
        Self {
            std_dev_gyroscope: 0.2,
            std_dev_accelerometer1: 0.05,
            std_dev_accelerometer2: 4.0,
            std_dev_magnetometer: 2.0,
            std_dev_barometer: 1.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SensorData {
    pub time: u32,
    pub gyroscope: Option<Vector3<f32>>,
    pub accelerometer1: Option<Vector3<f32>>,
    pub accelerometer2: Option<Vector3<f32>>,
    pub magnetometer: Option<Vector3<f32>>,
    pub pressure: Option<f32>,
}

impl SensorData {
    fn sample_1d(rng: &mut StdRng, std_dev: f32) -> f32 {
        rand_distr::Normal::new(0.0, std_dev).unwrap().sample(rng)
    }

    fn sample_3d(rng: &mut StdRng, std_dev: f32) -> Vector3<f32> {
        Vector3::new(
            Self::sample_1d(rng, std_dev),
            Self::sample_1d(rng, std_dev),
            Self::sample_1d(rng, std_dev)
        )
    }

    pub fn sample(rng: &mut StdRng, state: &SimulationState, settings: &SimulationSettings) -> Self {
        let alt_asl = state.position.z + settings.environment.launch_altitude + Self::sample_1d(rng, settings.sensors.std_dev_barometer);
        let pressure = 1013.25 * (1.0 - alt_asl / 44307.694).powf(1.0 / 0.190284);

        let acc2 = state.orientation.inverse_transform_vector(&(state.acceleration + &Vector3::new(0.0, 0.0, GRAVITY)));
        let acc1 = acc2.inf(&Vector3::new(150.0, 150.0, 150.0)).sup(&Vector3::new(-150.0, -150.0, -150.0));

        let mag_field = Vector3::new(0.0, 50.0, 0.0);
        let mag = state.orientation.inverse_transform_vector(&mag_field);

        SensorData {
            time: state.t,
            gyroscope: Some(state.angular_velocity + Self::sample_3d(rng, settings.sensors.std_dev_gyroscope)),
            accelerometer1: Some(acc1 + Self::sample_3d(rng, settings.sensors.std_dev_accelerometer1)),
            accelerometer2: Some(acc2 + Self::sample_3d(rng, settings.sensors.std_dev_accelerometer2)),
            magnetometer: Some(mag + Self::sample_3d(rng, settings.sensors.std_dev_magnetometer)),
            pressure: Some(pressure),
        }
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut SensorSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.set_width(ui.available_width());

            ui.label("👁 Sensors");

            egui::Grid::new("galadriel_settings")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Gyro std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_gyroscope)
                            .suffix(" °/s")
                            .speed(0.001)
                            .clamp_range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Primary accel. std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_accelerometer1)
                            .suffix(" m/s²")
                            .speed(0.001)
                            .clamp_range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Backup accel. std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_accelerometer2)
                            .suffix(" m/s²")
                            .speed(0.001)
                            .clamp_range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Magnetometer std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_magnetometer)
                            .suffix(" µT")
                            .speed(0.001)
                            .clamp_range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Barometer std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_barometer)
                            .suffix(" m")
                            .speed(0.001)
                            .clamp_range(0.001..=100.0),
                    );
                    ui.end_row();
                });
        }).response
    }
}
