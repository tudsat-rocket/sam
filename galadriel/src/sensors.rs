use std::convert::Infallible;

use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand_distr::Distribution;

use ::telemetry::{AccelerometerId, BarometerId, GyroscopeId, MagnetometerId, MetricSource, PressureSensorId};
use shared_types::{telemetry::GPSDatum, GPSFixType};

use crate::{FlightPhase, SimulationSettings, SimulationState, GRAVITY};

#[derive(Clone, Debug, PartialEq)]
pub struct SensorSettings {
    pub std_dev_gyroscope: f32,
    pub std_dev_accelerometer1: f32,
    pub std_dev_accelerometer2: f32,
    pub std_dev_magnetometer: f32,
    pub std_dev_barometer: f32,
    /// Used for low-pass filtering barometer values, simulating - for instance - an
    /// improperly vented avionics bay.
    pub barometer_iir_alpha: f32,
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
            barometer_iir_alpha: 0.995,
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
    pub lp_filtered_pressure: Option<f32>,
    pub pressure: Option<f32>,
    pub gps: Option<GPSDatum>,
}

impl SensorData {
    fn sample_1d(rng: &mut StdRng, std_dev: f32) -> f32 {
        rand_distr::Normal::new(0.0, std_dev).unwrap().sample(rng)
    }

    fn sample_3d(rng: &mut StdRng, std_dev: f32) -> Vector3<f32> {
        Vector3::new(Self::sample_1d(rng, std_dev), Self::sample_1d(rng, std_dev), Self::sample_1d(rng, std_dev))
    }

    fn altitude_asl(state: &SimulationState, settings: &SimulationSettings) -> f32 {
        state.position.z + settings.environment.launch_altitude
    }

    fn latitude(state: &SimulationState, settings: &SimulationSettings) -> f32 {
        settings.environment.launch_location.0 + state.position.y / 111_111.0
    }

    fn longitude(state: &SimulationState, settings: &SimulationSettings) -> f32 {
        settings.environment.launch_location.1
            + state.position.x / (111_111.0 * settings.environment.launch_location.0.to_radians().cos())
    }

    pub fn sample(
        rng: &mut StdRng,
        state: &SimulationState,
        settings: &SimulationSettings,
        last: Option<&Self>,
    ) -> Self {
        let alt_asl = state.position.z + settings.environment.launch_altitude;
        let sampled_pressure = 1013.25 * (1.0 - alt_asl / 44307.694).powf(1.0 / 0.190284);

        let last_filtered_pressure = last.and_then(|sd| sd.lp_filtered_pressure).unwrap_or(sampled_pressure);
        let iir_alpha = 1.0 - settings.sensors.barometer_iir_alpha;
        let new_filtered_pressure = sampled_pressure * iir_alpha + (1.0 - iir_alpha) * last_filtered_pressure;
        let pressure = new_filtered_pressure + Self::sample_1d(rng, settings.sensors.std_dev_barometer);

        let acc2 = state.orientation.inverse_transform_vector(&(state.acceleration + &Vector3::new(0.0, 0.0, GRAVITY)));
        let acc1 = acc2.inf(&Vector3::new(150.0, 150.0, 150.0)).sup(&Vector3::new(-150.0, -150.0, -150.0));

        let mag_field = Vector3::new(0.0, 50.0, 0.0);
        let mag = state.orientation.inverse_transform_vector(&mag_field);

        let gps_present = state.t % (100 - 100 % settings.delta_time) == 0
            && !(state.flight_phase == FlightPhase::Burn
                || state.flight_phase == FlightPhase::Coast
                || state.flight_phase == FlightPhase::Descent && state.seconds_in_phase() < 5.0);
        let gps = gps_present.then_some(GPSDatum {
            utc_time: None,
            // TODO: noise
            latitude: Some(Self::latitude(state, settings)),
            longitude: Some(Self::longitude(state, settings)),
            altitude: Some(Self::altitude_asl(state, settings)), // TODO: offset
            fix: GPSFixType::AutonomousFix,
            hdop: 100,
            num_satellites: 10,
        });

        SensorData {
            time: state.t,
            gyroscope: Some(state.angular_velocity + Self::sample_3d(rng, settings.sensors.std_dev_gyroscope)),
            accelerometer1: Some(acc1 + Self::sample_3d(rng, settings.sensors.std_dev_accelerometer1)),
            accelerometer2: Some(acc2 + Self::sample_3d(rng, settings.sensors.std_dev_accelerometer2)),
            magnetometer: Some(mag + Self::sample_3d(rng, settings.sensors.std_dev_magnetometer)),
            lp_filtered_pressure: Some(new_filtered_pressure),
            pressure: Some(pressure),
            gps,
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
                            .range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Primary accel. std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_accelerometer1)
                            .suffix(" m/s²")
                            .speed(0.001)
                            .range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Backup accel. std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_accelerometer2)
                            .suffix(" m/s²")
                            .speed(0.001)
                            .range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Magnetometer std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_magnetometer)
                            .suffix(" µT")
                            .speed(0.001)
                            .range(0.001..=100.0),
                    );
                    ui.end_row();

                    ui.label("Barometer std dev.");
                    ui.add(
                        egui::DragValue::new(&mut self.std_dev_barometer)
                            .suffix(" m")
                            .speed(0.001)
                            .range(0.001..=10000.0),
                    );
                    ui.end_row();

                    ui.label("Barometer IIR α");
                    ui.add(egui::DragValue::new(&mut self.barometer_iir_alpha).speed(0.0001).range(0.9..=1.0));
                    ui.end_row();
                });
        })
        .response
    }
}

impl MetricSource for SensorData {
    type Error = Infallible;

    fn write_metric<const N: usize>(
        &mut self,
        w: &mut ::telemetry::TelemetryMessageWriter<N>,
        metric: ::telemetry::Metric,
        repr: ::telemetry::Representation,
    ) -> Result<(), Self::Error> {
        use ::telemetry::Metric;
        use Metric::*;

        match metric {
            RawAngularVelocity(GyroscopeId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.gyroscope.unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.accelerometer1.unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::H3LIS331, dim) => {
                w.write_vector(repr, dim, &self.accelerometer2.unwrap_or_default())
            }
            RawMagneticFluxDensity(MagnetometerId::LIS3MDL, dim) => {
                w.write_vector(repr, dim, &self.magnetometer.unwrap_or_default())
            }
            RawBarometricAltitude(BarometerId::MS5611) => w.write_float(
                repr,
                self.pressure.map(|p| 44330.769 * (1.0 - (p / 1012.5).powf(0.190223))).unwrap_or_default(),
            ),
            Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611)) => {
                w.write_float(repr, self.pressure.unwrap_or_default())
            }
            GpsFix => w.write_enum(repr, self.gps.as_ref().and_then(|gps| Some(gps.fix)).unwrap_or_default() as u8), // TODo
            GpsLatitude => w.write_float(repr, self.gps.as_ref().and_then(|gps| gps.latitude).unwrap_or_default()),
            GpsLongitude => w.write_float(repr, self.gps.as_ref().and_then(|gps| gps.longitude).unwrap_or_default()),
            GpsAltitude => w.write_float(repr, self.gps.as_ref().and_then(|gps| gps.altitude).unwrap_or_default()),
            GpsHdop => w.write_float(repr, self.gps.as_ref().and_then(|gps| Some(gps.hdop as f32)).unwrap_or_default()),
            GpsSatellites => w.write_float(
                repr,
                self.gps.as_ref().and_then(|gps| Some(gps.num_satellites as f32)).unwrap_or_default(),
            ),
            m => todo!("{m:?}"),
        }
    }
}
