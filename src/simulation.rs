#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
use rand::SeedableRng;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use std::time::Duration;
use std::sync::mpsc::SendError;
use std::slice::Iter;

use egui::Color32;
use nalgebra::{Vector3, UnitQuaternion};
use rand::RngCore;
use rand::rngs::SmallRng;
use rand::distributions::Distribution;

use sting_fc_firmware::settings::*;
use sting_fc_firmware::telemetry::*;
use sting_fc_firmware::state_estimation::*;

use crate::state::VehicleState;
use crate::data_source::DataSource;

const GRAVITY: f32 = 9.80665;

const SIMULATION_TICK_MS: u32 = 10;
const PLOT_STEP_MS: u32 = 100;

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationSettings {
    pub altitude_ground: f32,
    pub launch_angle: f32,
    pub launch_azimuth: f32,
    pub launch_latitude: f64,
    pub launch_longitude: f64,

    pub sim_duration: u32,
    pub sim_start_delay: u32,
    pub thrust_duration: u32,
    pub thrust: f32,

    pub drag_flight: f32,
    pub drag_drogue: f32,
    pub drag_main: f32,

    pub std_dev_gyroscope: f32,
    pub std_dev_accelerometer1: f32,
    pub std_dev_accelerometer2: f32,
    pub std_dev_magnetometer: f32,
    pub std_dev_barometer: f32,

    pub barometer_anomaly_probability: f32,
    pub barometer_anomaly_value: f32,
    pub barometer_anomaly_delay: u32,

    pub fc_settings: Settings,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        Self {
            altitude_ground: 150.0,
            launch_angle: 5.0,
            launch_azimuth: 60.0,
            launch_latitude: 49.861445,
            launch_longitude: 8.68519,

            sim_duration: 240_000,
            sim_start_delay: 10_000,
            thrust_duration: 2_000,
            thrust: 120.0,

            drag_flight: 0.01,
            drag_drogue: 1.0,
            drag_main:   5.0,

            std_dev_gyroscope: 0.07,
            std_dev_accelerometer1: 0.05,
            std_dev_accelerometer2: 0.7,
            std_dev_magnetometer: 0.05,
            std_dev_barometer: 0.5,

            barometer_anomaly_probability: 0.00001,
            barometer_anomaly_value: 0.0,
            barometer_anomaly_delay: 0,

            fc_settings: Settings::default(),
        }
    }
}

#[derive(Debug)]
pub struct SimulationState {
    rng: SmallRng,
    settings: SimulationSettings,
    state_estimator: StateEstimator,

    time: u32,
    mode: FlightMode,

    latitude: f64,
    longitude: f64,
    altitude: f32,
    altitude_ground: f32,
    orientation: UnitQuaternion<f32>,
    angular_velocity: Vector3<f32>,
    acceleration: Vector3<f32>,
    velocity: Vector3<f32>,

    gyroscope: Option<Vector3<f32>>,
    accelerometer1: Option<Vector3<f32>>,
    accelerometer2: Option<Vector3<f32>>,
    magnetometer: Option<Vector3<f32>>,
    altitude_baro: Option<f32>
}

impl SimulationState {
    pub fn initialize(settings: &SimulationSettings) -> Self {
        let a = settings.launch_azimuth.to_radians();
        let b = settings.launch_angle.to_radians();
        let direction = Vector3::new(f32::sin(a) * f32::sin(b), f32::cos(a) * f32::sin(b), f32::cos(b));

        Self {
            #[cfg(not(target_arch = "wasm32"))]
            rng: rand::rngs::SmallRng::from_entropy(),
            #[cfg(target_arch = "wasm32")]
            rng: rand::rngs::SmallRng::from_seed([0x42; 16]),
            settings: settings.clone(),
            state_estimator: StateEstimator::new(1000.0 / (SIMULATION_TICK_MS as f32), settings.fc_settings.clone()),

            time: 0,
            mode: FlightMode::Idle,

            latitude: settings.launch_latitude,
            longitude: settings.launch_longitude,
            altitude: settings.altitude_ground,
            altitude_ground: settings.altitude_ground,
            orientation: UnitQuaternion::rotation_between(&Vector3::new(0.0, 0.0, 1.0), &direction).unwrap(),
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),

            gyroscope: None,
            accelerometer1: None,
            accelerometer2: None,
            magnetometer: None,
            altitude_baro: None
        }
    }

    fn sample_noise(&mut self, std_dev: f32) -> f32 {
        rand_distr::Normal::new(0.0, std_dev).unwrap().sample(&mut self.rng)
    }

    fn sample_noise_vector(&mut self, std_dev: f32) -> Vector3<f32> {
        Vector3::new(
            rand_distr::Normal::new(0.0, std_dev).unwrap().sample(&mut self.rng),
            rand_distr::Normal::new(0.0, std_dev).unwrap().sample(&mut self.rng),
            rand_distr::Normal::new(0.0, std_dev).unwrap().sample(&mut self.rng),
        )
    }

    fn sample_gyroscope(&mut self) -> Option<Vector3<f32>> {
        Some(self.angular_velocity + self.sample_noise_vector(self.settings.std_dev_gyroscope))
    }

    fn local_acceleration(&self) -> Vector3<f32> {
        self.orientation.inverse().transform_vector(&(self.acceleration + Vector3::new(0.0, 0.0, GRAVITY)))
    }

    fn sample_accelerometer1(&mut self) -> Option<Vector3<f32>> {
        let noisy = self.local_acceleration() + self.sample_noise_vector(self.settings.std_dev_accelerometer1);
        let clipped = Vector3::new(
            noisy.x.clamp(-160.0, 160.0),
            noisy.y.clamp(-160.0, 160.0),
            noisy.z.clamp(-160.0, 160.0)
        );
        Some(clipped)
    }

    fn sample_accelerometer2(&mut self) -> Option<Vector3<f32>> {
        Some(self.local_acceleration() + self.sample_noise_vector(self.settings.std_dev_accelerometer2))
    }

    fn sample_magnetometer(&mut self) -> Option<Vector3<f32>> {
        let magnetic_field = Vector3::new(00.0, 50.0, 0.0);
        let observed = self.orientation.inverse_transform_vector(&magnetic_field);
        Some(observed + self.sample_noise_vector(self.settings.std_dev_magnetometer))
    }

    fn sample_barometer(&mut self) -> Option<f32> {
        Some(self.altitude + self.sample_noise(self.settings.std_dev_barometer))
        // TODO: simulate barometer more accurately? for instance, simulate
        // noisy and rounded pressure and temp readings and run temp-comp?
    }

    pub fn tick(&mut self) {
        self.time += SIMULATION_TICK_MS;
        if self.time >= self.settings.sim_start_delay - 3000 && self.time < self.settings.sim_start_delay - 2000 {
            self.mode = FlightMode::Armed;
        }

        // advance true state of the vehicle
        self.acceleration = if self.time >= self.settings.sim_start_delay && self.time < self.settings.sim_start_delay + self.settings.thrust_duration {
            self.orientation.transform_vector(&Vector3::new(0.0, 0.0, self.settings.thrust))
        } else {
            Vector3::new(0.0, 0.0, -GRAVITY)
        };

        let velocity_dir = self.velocity.normalize();
        if !f32::is_nan(velocity_dir.x) {
            use FlightMode::*;
            let drag = match self.mode {
                Idle | HardwareArmed | Armed | Flight => self.settings.drag_flight,
                RecoveryDrogue => self.settings.drag_drogue,
                RecoveryMain | Landed => self.settings.drag_main,
            };
            self.acceleration += velocity_dir * -drag * self.velocity.magnitude().powi(2) * (SIMULATION_TICK_MS as f32) / 1000.0;
        }

        // TODO: roll

        let scaled_acceleration = self.acceleration * (SIMULATION_TICK_MS as f32) / 1000.0;
        let scaled_velocity = self.velocity * (SIMULATION_TICK_MS as f32) / 1000.0;
        let old_velocity = self.velocity.clone();

        self.velocity += scaled_acceleration;
        self.latitude += (scaled_velocity.y as f64) / 111_320.0;
        self.longitude += (scaled_velocity.x as f64) / (400_750.0 * f64::cos(self.latitude.to_radians()) / 3.6);
        self.altitude += scaled_velocity.z;

        let rotation = UnitQuaternion::rotation_between(&old_velocity, &self.velocity).unwrap();
        let (roll, pitch, yaw) = rotation.euler_angles();
        self.angular_velocity = Vector3::new(pitch, yaw, roll) * 1000.0 / (SIMULATION_TICK_MS as f32);
        self.orientation *= rotation;

        if self.velocity.z < 0.0 && self.altitude <= self.altitude_ground {
            self.acceleration = Vector3::new(0.0, 0.0, 0.0);
            self.altitude = self.altitude_ground;
            self.velocity = Vector3::new(0.0, 0.0, 0.0);
        }

        // sample our sensors
        self.gyroscope = self.sample_gyroscope();
        self.accelerometer1 = self.sample_accelerometer1();
        self.accelerometer2 = self.sample_accelerometer2();
        self.magnetometer = self.sample_magnetometer();
        self.altitude_baro = self.sample_barometer();

        if self.time > self.settings.barometer_anomaly_delay &&
            self.rng.next_u32() < (self.settings.barometer_anomaly_probability * u32::MAX as f32) as u32
        {
            self.altitude_baro = Some(self.settings.barometer_anomaly_value);
        }

        // update state estimation with sampled sensor values
        self.state_estimator.update(
            self.time,
            self.mode,
            self.gyroscope,
            self.accelerometer1,
            self.accelerometer2,
            self.magnetometer,
            self.altitude_baro
        );

        let arm_voltage = if self.time >= (self.settings.sim_start_delay - 5000) { 8400 } else { 0 };
        if let Some(mode) = self.state_estimator.new_mode(arm_voltage, None) {
            self.mode = mode;
        }
    }
}

impl From<&SimulationState> for VehicleState {
    fn from(ss: &SimulationState) -> VehicleState {
        VehicleState {
            time: ss.time,
            mode: Some(ss.mode),
            orientation: ss.state_estimator.orientation,
            altitude: Some(ss.state_estimator.altitude()), // TODO
            altitude_baro: ss.altitude_baro,
            altitude_ground: Some(ss.altitude_ground),
            altitude_max: Some(ss.state_estimator.altitude_max),
            altitude_gps: Some(ss.altitude), // TODO
            latitude: Some(ss.latitude as f32),
            longitude: Some(ss.longitude as f32),
            vertical_speed: Some(ss.state_estimator.vertical_speed()),
            vertical_accel: ss.state_estimator.acceleration_world().map(|acc| acc.z),
            vertical_accel_filtered: Some(ss.state_estimator.vertical_accel()),
            gps_fix: Some(GPSFixType::AutonomousFix),
            num_satellites: Some(6),
            hdop: Some(150),
            gyroscope: ss.gyroscope,
            accelerometer1: ss.accelerometer1,
            accelerometer2: ss.accelerometer2,
            magnetometer: ss.magnetometer,
            battery_voltage: Some(8.4),
            arm_voltage: Some(if ss.time >= (ss.settings.sim_start_delay - 5000) { 8.4 } else { 0.0 }),
            true_orientation: Some(ss.orientation),
            true_vertical_accel: Some(ss.acceleration.z),
            true_vertical_speed: Some(ss.velocity.z),
            ..Default::default()
        }
    }
}

#[derive(Default)]
pub struct SimulationDataSource {
    settings: SimulationSettings,
    state: Option<SimulationState>,
    vehicle_states: Vec<(Instant, VehicleState)>,
}

impl DataSource for SimulationDataSource {
    fn new_vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
        if self.state.is_none() {
            self.state = Some(SimulationState::initialize(&self.settings));
        }

        let start_i = self.vehicle_states.len();
        while self.vehicle_states.last().map(|(_t, vs)| vs.time < self.settings.sim_duration).unwrap_or(true) {
            self.state.as_mut().unwrap().tick();
            let sim_state = self.state.as_ref().unwrap();

            if sim_state.time % PLOT_STEP_MS != 0 {
                continue;
            }

            let vehicle_state = sim_state.into();
            let time = self.vehicle_states
                .last()
                .map(|(t, _)| *t + Duration::from_millis(PLOT_STEP_MS as u64))
                .unwrap_or(Instant::now());
            self.vehicle_states.push((time, vehicle_state));
        }

        self.vehicle_states[start_i..].iter()
    }

    fn vehicle_states<'a>(&'a mut self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn log_messages<'a>(&'a mut self) -> Iter<'_, (u32, String, LogLevel, String)> {
        Default::default()
    }

    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings> {
        None
    }

    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings> {
        None
    }

    fn reset(&mut self) {
        self.state = None;
        self.vehicle_states.truncate(0);
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn status(&self) -> (Color32, String) {
        (Color32::DARK_BLUE, "simulation".into())
    }

    fn info_text(&self) -> String {
        "".into()
    }

    fn minimum_fps(&self) -> Option<u64> {
        None
    }

    fn simulation_settings(&mut self) -> Option<&mut SimulationSettings> {
        Some(&mut self.settings)
    }

    fn end(&self) -> Option<Instant> {
        self.vehicle_states.last().map(|(t, _vs)| *t)
    }
}
