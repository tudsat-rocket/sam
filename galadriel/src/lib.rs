use nalgebra::*;

pub mod drag;
pub mod environment;
pub mod motor;
pub mod parachute;
pub mod replication;
pub mod rocket;
pub mod sensors;
pub mod thrusters;

use rand::{rngs::StdRng, SeedableRng};
pub use replication::Replication;
pub use sensors::SensorData;

use environment::*;
use rocket::*;
use sensors::*;

const IGNITION_TIME: u32 = 10000;
const GRAVITY: f32 = 9.80665;

#[derive(Clone, Default, Debug, PartialEq, Eq)]
pub enum FlightPhase {
    #[default]
    PreLaunch,
    LaunchRail,
    Burn,
    Coast,
    Descent,
    Touchdown,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationSettings {
    delta_time: u32,
    rocket: RocketSettings,
    environment: Environment,
    sensors: SensorSettings,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        Self {
            delta_time: 1,
            rocket: RocketSettings::default(),
            environment: Environment::default(),
            sensors: SensorSettings::default(),
        }
    }
}

#[derive(Clone, Default, Debug)]
#[non_exhaustive]
pub struct SimulationState {
    /// Current time [ms]
    pub t: u32,
    /// Current flight pahse
    pub flight_phase: FlightPhase,
    /// Start of current phase [ms]
    pub t_phase_start: u32,
    /// Orientation of the rocket
    pub orientation: UnitQuaternion<f32>,
    /// Angular velocity of the rocket [rad/s]
    pub angular_velocity: Vector3<f32>,
    /// Position in global space relative to launch location [m, Y+ is north, Z+ is up]
    pub position: Vector3<f32>,
    /// Velocity in global space [m/s]
    pub velocity: Vector3<f32>,
    /// Acceleration in global space [m/s²]
    pub acceleration: Vector3<f32>,
}

pub struct Simulation {
    rng: StdRng,
    settings: SimulationSettings,
    pub rocket: Rocket,
    pub state: SimulationState,
}

impl Simulation {
    pub fn new(settings: &SimulationSettings) -> Self {
        let rocket = Rocket::new(&settings.rocket);

        let a = settings.environment.azimuth.to_radians();
        let b = settings.environment.elevation.to_radians();
        let direction = Vector3::new(-f32::sin(a) * f32::cos(b), f32::cos(a) * f32::cos(b), f32::sin(b));
        let orientation = UnitQuaternion::face_towards(&direction, &Vector3::new(0.0, 0.0, 1.0));

        let state = SimulationState {
            orientation,
            position: Vector3::new(0.0, 0.0, 0.0),
            velocity: Vector3::new(0.0, 0.0, 0.0),
            acceleration: Vector3::new(0.0, 0.0, 0.0),
            ..Default::default()
        };

        Self {
            rng: StdRng::from_entropy(),
            settings: settings.clone(),
            rocket,
            state,
        }
    }

    fn switch_phase(&mut self, phase: FlightPhase) {
        if phase == self.state.flight_phase {
            return;
        }
        self.state.flight_phase = phase;
        self.state.t_phase_start = self.state.t;
    }

    pub fn time_in_phase(&self) -> f32 {
        ((self.state.t - self.state.t_phase_start) as f32) / 1000.0
    }

    pub fn altitude_asl(&self) -> f32 {
        self.state.position.z + self.settings.environment.launch_altitude
    }

    pub fn latitude(&self) -> f32 {
        self.settings.environment.launch_location.0 + self.state.position.x / 111_111.0
    }

    pub fn longitude(&self) -> f32 {
        self.settings.environment.launch_location.1 + self.state.position.y / (111_111.0 * self.settings.environment.launch_location.0.to_radians().cos())
    }

    pub fn tick(&mut self) {
        let t = (self.state.t - self.state.t_phase_start) as f32 / 1000.0;
        let t_since_ignition = (self.state.t as f32 - IGNITION_TIME as f32) / 1000.0;

        if t > 300.0 {
            // give up. maybe zero motor thrust or something.
            self.switch_phase(FlightPhase::Touchdown);
        }

        match self.state.flight_phase {
            FlightPhase::PreLaunch if self.state.t > IGNITION_TIME => {
                self.switch_phase(FlightPhase::LaunchRail);
            }
            FlightPhase::LaunchRail
                // TODO: trigonometry
                if self.state.position.z > self.settings.environment.launch_rail_length =>
            {
                self.switch_phase(FlightPhase::Burn);
            }
            FlightPhase::Burn if self.rocket.motor.burned_out(t) => {
                self.switch_phase(FlightPhase::Coast);
            }
            FlightPhase::Coast if self.state.velocity.z < 0.0 => {
                self.switch_phase(FlightPhase::Descent);
            }
            FlightPhase::Descent if self.state.position.z <= 0.0 => {
                self.state.position.z = 0.0;
                self.state.velocity = Vector3::new(0.0, 0.0, 0.0);
                self.state.acceleration = Vector3::new(0.0, 0.0, 0.0);
                self.switch_phase(FlightPhase::Touchdown);
            }
            _ => {}
        }

        // calculate forces acting on the vehicle in vehicle space
        let vehicle_forces = self.rocket.thrust(t_since_ignition);

        // We calculate drag using global space velocity to save on quaternion operations
        // TODO: clean up air density stuff, move to environment.rs
        let alt_asl = self.state.position.z + self.settings.environment.launch_altitude;
        let pressure = 101325.0 * (1.0 - alt_asl / 44307.694).powf(1.0 / 0.190284);
        let temp = 288.15 - alt_asl * 0.0065;
        let density = pressure * 0.0289652 / (8.31446 * temp);
        let global_forces = self.rocket.drag(self.state.velocity, density);

        if let Some(thrusters) = self.rocket.thrusters.as_mut() {
            thrusters.tick(self.settings.delta_time);
        }

        // Rotate into global frame
        let forces = (self.state.orientation * vehicle_forces) + global_forces;
        let mass = self.rocket.mass(t_since_ignition);
        self.state.acceleration = forces / mass;

        if self.state.flight_phase != FlightPhase::PreLaunch &&
                self.state.flight_phase != FlightPhase::Touchdown {
            self.state.acceleration.z -= GRAVITY;
        }

        // TODO: wind

        // TODO: get some runge kutta up in here
        let delta_time = (self.settings.delta_time as f32) / 1000.0;
        self.state.velocity += self.state.acceleration * delta_time;
        self.state.position += self.state.velocity * delta_time;

        // TODO: trigger parachutes automatically

        // We assume stability, so we derive orientation from velocity.
        let last_orientation = self.state.orientation;
        let new_orientation = match self.state.flight_phase {
            FlightPhase::Burn | FlightPhase::Coast | FlightPhase::Descent =>
            Some(UnitQuaternion::face_towards(
                &self.state.velocity,
                &Vector3::new(0.0, 0.0, 1.0),
            )),
            _ => None,
        };

        if let Some(orientation) = new_orientation.and_then(|or| or.i.is_normal().then_some(or)) {
            self.state.orientation = orientation;
        }

        let (r,p,y) = last_orientation.rotation_to(&self.state.orientation).euler_angles();
        self.state.angular_velocity = Vector3::new(p,y,r) / delta_time; // TODO

        if self.state.orientation.euler_angles().0.is_nan() {
            std::process::exit(1);
        }

        self.state.t += self.settings.delta_time;
    }

    pub fn sensors(&mut self) -> SensorData {
        SensorData::sample(&mut self.rng, &self.state, &self.settings)
    }
}

impl Iterator for Simulation {
    type Item = SensorData;

    fn next(&mut self) -> Option<Self::Item> {
        self.tick();
        if self.state.flight_phase != FlightPhase::Touchdown || self.time_in_phase() < 10.0 {
            Some(self.sensors())
        } else {
            None
        }
    }
}

#[cfg(feature = "egui")]
impl egui::Widget for &mut SimulationSettings {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.vertical(|ui| {
            ui.set_width(ui.available_width());

            ui.label("💻 Simulation");

            egui::Grid::new("galadriel_settings")
                .num_columns(2)
                .min_col_width(0.25 * ui.available_width())
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Δtime");
                    ui.add(
                        egui::DragValue::new(&mut self.delta_time)
                            .suffix(" ms")
                            .speed(1)
                            .clamp_range(1..=1000),
                    );
                    ui.end_row();
                });

            ui.add_space(10.0);
            ui.add(&mut self.environment);

            ui.add_space(10.0);
            ui.add(&mut self.rocket);

            ui.add_space(10.0);
            ui.add(&mut self.sensors);
        }).response
    }
}
