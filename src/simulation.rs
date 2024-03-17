use std::collections::VecDeque;
use std::f32::consts::PI;

use nalgebra::{UnitQuaternion, Vector3};
use rand::distributions::Distribution;
use rand::RngCore;
use rand::SeedableRng;

use mithril::settings::*;
use mithril::state_estimation::*;
use mithril::telemetry::*;

use crate::gui::windows::archive::ArchivedLog;

#[cfg(not(target_arch = "wasm32"))]
type Rng = rand::rngs::StdRng;
#[cfg(target_arch = "wasm32")]
type Rng = rand::rngs::SmallRng;

const GRAVITY: f32 = 9.80665;

pub const SIMULATION_TICK_MS: u32 = 1;
pub const PLOT_STEP_MS: u32 = 50;

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationSettings {
    pub replicated_log: Option<ArchivedLog>,

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
            replicated_log: None,

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
            drag_main: 5.0,

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
    pub(crate) rng: Rng,
    pub(crate) settings: SimulationSettings,
    remaining_replication_states: VecDeque<VehicleState>,

    pub(crate) time: u32,

    pub(crate) latitude: f64,
    pub(crate) longitude: f64,
    pub(crate) altitude: f32,
    pub(crate) altitude_ground: f32,
    pub(crate) orientation: UnitQuaternion<f32>,
    pub(crate) angular_velocity: Vector3<f32>,
    pub(crate) acceleration: Vector3<f32>,
    pub(crate) velocity: Vector3<f32>,

    pub(crate) gyroscope: Option<Vector3<f32>>,
    pub(crate) accelerometer1: Option<Vector3<f32>>,
    pub(crate) accelerometer2: Option<Vector3<f32>>,
    pub(crate) magnetometer: Option<Vector3<f32>>,
    pub(crate) pressure_baro: Option<f32>,
    pub(crate) altitude_baro: Option<f32>,

    pub(crate) state_estimator: StateEstimator,
    pub(crate) mode: FlightMode,
}

impl SimulationState {
    fn load_log_states(bytes: &[u8]) -> Result<VecDeque<VehicleState>, reqwest::Error> {
        let msgs = serde_json::from_slice::<Vec<DownlinkMessage>>(bytes).unwrap();
        Ok(msgs.into_iter().map(|x| x.into()).collect())
    }

    pub fn initialize(settings: &SimulationSettings) -> Self {
        log::info!("Initializing");
        let a = settings.launch_azimuth.to_radians();
        let b = settings.launch_angle.to_radians();
        let direction = Vector3::new(f32::sin(a) * f32::sin(b), f32::cos(a) * f32::sin(b), f32::cos(b));

        let mut settings = settings.clone();

        // Fix FC orientation for EuRoC flight (TODO: do this better)
        if let Some(ArchivedLog::Euroc23) = settings.replicated_log {
            settings.fc_settings.orientation = Orientation::ZDown;
        }

        // TODO: do this nicer (separate thread/task, progress, caching, support WASM)
        #[cfg(not(target_arch = "wasm32"))]
        let mut remaining_replication_states = settings
            .replicated_log
            .and_then(|log| log.flash_log())
            .map(|bytes| Self::load_log_states(bytes).unwrap())
            .unwrap_or_default();

        #[cfg(target_arch = "wasm32")]
        let mut remaining_replication_states: VecDeque<VehicleState> = VecDeque::new();

        // Skip the first known states without GPS data
        while remaining_replication_states.front().map(|vs| vs.latitude.is_none()).unwrap_or(false) {
            remaining_replication_states.pop_front();
        }

        let (altitude, latitude, longitude) = remaining_replication_states
            .front()
            .map(|vs| (vs.altitude_gps_asl.unwrap(), vs.latitude.unwrap() as f64, vs.longitude.unwrap() as f64))
            .unwrap_or((settings.altitude_ground, settings.launch_latitude, settings.launch_longitude));

        // Skip the first known states without raw sensor data
        while remaining_replication_states.front().map(|vs| vs.gyroscope.is_none()).unwrap_or(false) {
            remaining_replication_states.pop_front();
        }

        let mut state_estimator =
            StateEstimator::new(1000.0 / (SIMULATION_TICK_MS as f32), settings.fc_settings.clone());

        let mut time = 0;
        let mut mode = FlightMode::Idle;
        let mut gyroscope = None;
        let mut accelerometer1 = None;
        let mut accelerometer2 = None;
        let mut magnetometer = None;
        let mut pressure_baro = None;
        let mut altitude_baro = None;
        if let Some(first) = remaining_replication_states.pop_front() {
            time = first.time;
            mode = FlightMode::Armed;
            gyroscope = first.gyroscope;
            accelerometer1 = first.accelerometer1;
            accelerometer2 = first.accelerometer2;
            magnetometer = first.magnetometer;
            pressure_baro = first.pressure_baro;
            altitude_baro = pressure_baro.map(|p| 44307.694 * (1.0 - (p / 1013.25).powf(0.190284)));
            state_estimator.altitude_ground = altitude_baro.unwrap();
        }

        Self {
            #[cfg(not(target_arch = "wasm32"))]
            rng: Rng::from_entropy(),
            #[cfg(target_arch = "wasm32")]
            rng: Rng::from_seed([0x42; 16]),
            settings,
            state_estimator,

            time,
            mode,

            latitude,
            longitude,
            altitude,
            altitude_ground: altitude,
            orientation: UnitQuaternion::rotation_between(&Vector3::new(0.0, 0.0, 1.0), &direction).unwrap(),
            angular_velocity: Vector3::new(0.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),

            gyroscope,
            accelerometer1,
            accelerometer2,
            magnetometer,
            pressure_baro,
            altitude_baro,

            remaining_replication_states,
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
        let clipped =
            Vector3::new(noisy.x.clamp(-155.0, 155.0), noisy.y.clamp(-155.0, 155.0), noisy.z.clamp(-155.0, 155.0));
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

    pub fn plottable(&self) -> bool {
        if self.settings.replicated_log.is_some() {
            (self.time + 3) % 10 == 5 // match flash log raw sensor timing
        } else {
            self.time % PLOT_STEP_MS == 0
        }
    }

    fn advance_replication(&mut self) -> bool {
        loop {
            if self.remaining_replication_states.is_empty() || self.mode == FlightMode::Landed {
                return true;
            }

            let next = self.remaining_replication_states.front().unwrap();
            if next.time > self.time {
                break;
            }

            self.remaining_replication_states.pop_front();
        }

        loop {
            let next = self.remaining_replication_states.front().unwrap();
            if next.gyroscope.is_some() {
                break;
            }

            if next.altitude_gps_asl.is_some() && next.latitude.is_some() && next.longitude.is_some() {
                self.altitude = next.altitude_gps_asl.unwrap();
                self.latitude = next.latitude.unwrap() as f64;
                self.longitude = next.longitude.unwrap() as f64;
            }

            if self.remaining_replication_states.is_empty() {
                return true;
            }
            self.remaining_replication_states.pop_front();
        }

        let next = self.remaining_replication_states.front().unwrap();
        self.time += SIMULATION_TICK_MS;
        if self.time == next.time {
            self.gyroscope = next.gyroscope;
            self.accelerometer1 = next.accelerometer1;
            self.accelerometer2 = next.accelerometer2;
            self.magnetometer = next.magnetometer;
            // TODO: reuse this somehow
            self.altitude_baro = next.pressure_baro.map(|p| 44307.694 * (1.0 - (p / 1013.25).powf(0.190284)));
            self.remaining_replication_states.pop_front();
        } else {
            // TODO: linear interpolation is probably not the best choice here.
            // On the other hand, simplying applying more noise here isn't great either
            let delta_time = next.time - self.time;
            self.gyroscope =
                Some(self.gyroscope.unwrap() + (next.gyroscope.unwrap() - self.gyroscope.unwrap()) / delta_time as f32);
            self.accelerometer1 = Some(
                self.accelerometer1.unwrap()
                    + (next.accelerometer1.unwrap() - self.accelerometer1.unwrap()) / delta_time as f32,
            );
            self.accelerometer2 = Some(
                self.accelerometer2.unwrap()
                    + (next.accelerometer2.unwrap() - self.accelerometer2.unwrap()) / delta_time as f32,
            );
            self.magnetometer = Some(
                self.magnetometer.unwrap()
                    + (next.magnetometer.unwrap() - self.magnetometer.unwrap()) / delta_time as f32,
            );
            self.pressure_baro = Some(
                self.pressure_baro.unwrap()
                    + (next.pressure_baro.unwrap() - self.pressure_baro.unwrap()) / delta_time as f32,
            );
            self.altitude_baro = self.pressure_baro.map(|p| 44307.694 * (1.0 - (p / 1013.25).powf(0.190284)));
        }

        false
    }

    fn advance_simulation(&mut self) -> bool {
        if self.time > self.settings.sim_duration {
            return true;
        }

        self.time += SIMULATION_TICK_MS;
        if self.time >= self.settings.sim_start_delay - 3000 && self.time < self.settings.sim_start_delay - 2000 {
            self.mode = FlightMode::Armed;
        }

        // advance true state of the vehicle
        self.acceleration = if self.time >= self.settings.sim_start_delay
            && self.time < self.settings.sim_start_delay + self.settings.thrust_duration
        {
            self.orientation.transform_vector(&Vector3::new(0.0, 0.0, self.settings.thrust))
        } else {
            Vector3::new(0.0, 0.0, -GRAVITY)
        };

        let velocity_dir = self.velocity.normalize();
        if !f32::is_nan(velocity_dir.x) {
            use FlightMode::*;
            let drag = match self.mode {
                Idle | HardwareArmed | Armed | Burn | Coast => self.settings.drag_flight,
                RecoveryDrogue => self.settings.drag_drogue,
                RecoveryMain | Landed => self.settings.drag_main,
            };
            self.acceleration +=
                velocity_dir * -drag * self.velocity.magnitude().powi(2) * (SIMULATION_TICK_MS as f32) / 1000.0;
        }

        // TODO: roll

        let scaled_acceleration = self.acceleration * (SIMULATION_TICK_MS as f32) / 1000.0;
        let scaled_velocity = self.velocity * (SIMULATION_TICK_MS as f32) / 1000.0;
        let old_velocity = self.velocity;

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

        if self.time > self.settings.barometer_anomaly_delay
            && self.rng.next_u32() < (self.settings.barometer_anomaly_probability * u32::MAX as f32) as u32
        {
            self.altitude_baro = Some(self.settings.barometer_anomaly_value);
        }

        false
    }

    pub fn tick(&mut self) -> bool {
        let (done, arm_voltage) = if self.settings.replicated_log.is_some() {
            (self.advance_replication(), 8400)
        } else {
            let done = self.advance_simulation();
            let arm_v = if self.time >= (self.settings.sim_start_delay - 5000) {
                8400
            } else {
                0
            };
            (done, arm_v)
        };

        if done {
            return true;
        }

        // update state estimation with sampled sensor values
        self.state_estimator.update(
            std::num::Wrapping(self.time),
            self.mode,
            self.gyroscope,
            self.accelerometer1,
            self.accelerometer2,
            self.magnetometer,
            self.altitude_baro,
        );

        if let Some(mode) = self.state_estimator.new_mode(arm_voltage, None) {
            self.mode = mode;
        }

        false
    }
}

impl From<&SimulationState> for VehicleState {
    fn from(ss: &SimulationState) -> VehicleState {
        if ss.settings.replicated_log.is_some() {
            VehicleState {
                time: ss.time,
                mode: Some(ss.mode),
                orientation: ss.state_estimator.orientation,
                euler_angles: ss.state_estimator.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI),
                angle_of_attack: ss.state_estimator.orientation.map(|q| {
                    let up = Vector3::new(0.0, 0.0, 1.0);
                    let attitude = q * up;
                    90.0 - up.dot(&attitude).acos().to_degrees()
                }),
                altitude_asl: Some(ss.state_estimator.altitude_asl()),
                altitude_baro: ss.altitude_baro,
                altitude_ground_asl: Some(ss.state_estimator.altitude_ground),
                apogee_asl: Some(ss.state_estimator.altitude_max),
                altitude_gps_asl: Some(ss.altitude), // TODO
                latitude: Some(ss.latitude as f32),
                longitude: Some(ss.longitude as f32),
                vertical_speed: Some(ss.state_estimator.vertical_speed()),
                vertical_accel: ss.state_estimator.acceleration_world_raw().map(|acc| acc.z),
                vertical_accel_filtered: Some(ss.state_estimator.vertical_acceleration()),
                gps_fix: Some(GPSFixType::AutonomousFix), // TODO
                num_satellites: Some(6),
                hdop: Some(0),
                gyroscope: ss.gyroscope,
                accelerometer1: ss.accelerometer1,
                accelerometer2: ss.accelerometer2,
                magnetometer: ss.magnetometer,
                pressure_baro: ss.pressure_baro,
                ..Default::default()
            }
        } else {
            VehicleState {
                time: ss.time,
                mode: Some(ss.mode),
                orientation: ss.state_estimator.orientation,
                euler_angles: ss.state_estimator.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI),
                angle_of_attack: ss.state_estimator.orientation.map(|q| {
                    let up = Vector3::new(0.0, 0.0, 1.0);
                    let attitude = q * up;
                    90.0 - up.dot(&attitude).acos().to_degrees()
                }),
                altitude_asl: Some(ss.state_estimator.altitude_asl()), // TODO
                altitude_baro: ss.altitude_baro,
                altitude_ground_asl: Some(ss.altitude_ground),
                apogee_asl: Some(ss.state_estimator.altitude_max),
                altitude_gps_asl: Some(ss.altitude), // TODO
                latitude: Some(ss.latitude as f32),
                longitude: Some(ss.longitude as f32),
                vertical_speed: Some(ss.state_estimator.vertical_speed()),
                vertical_accel: ss.state_estimator.acceleration_world_raw().map(|acc| acc.z),
                vertical_accel_filtered: Some(ss.state_estimator.vertical_acceleration()),
                gps_fix: Some(GPSFixType::AutonomousFix),
                num_satellites: Some(6),
                hdop: Some(150),
                gyroscope: ss.gyroscope,
                accelerometer1: ss.accelerometer1,
                accelerometer2: ss.accelerometer2,
                magnetometer: ss.magnetometer,
                battery_voltage: Some(8400),
                arm_voltage: Some(if ss.time >= (ss.settings.sim_start_delay - 5000) {
                    8400
                } else {
                    0
                }),
                true_orientation: Some(ss.orientation),
                true_euler_angles: {
                    let (r, p, y) = ss.orientation.euler_angles();
                    Some(Vector3::new(r, p, y) * 180.0 / PI)
                },
                true_angle_of_attack: {
                    let up = Vector3::new(0.0, 0.0, 1.0);
                    let attitude = ss.orientation * up;
                    Some(90.0 - up.dot(&attitude).acos().to_degrees())
                },
                true_vertical_accel: Some(ss.acceleration.z),
                true_vertical_speed: Some(ss.velocity.z),
                ..Default::default()
            }
        }
    }
}
