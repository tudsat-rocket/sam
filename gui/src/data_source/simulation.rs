//! A simulation data source

use std::any::Any;
use std::collections::VecDeque;
use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::Duration;
use std::slice::Iter;
use std::f32::consts::PI;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use nalgebra::Vector3;

use archive::ArchivedLog;
use galadriel::FlightPhase;
use shared_types::settings::*;
use shared_types::telemetry::*;
use state_estimator::StateEstimator;

use crate::data_source::*;
use crate::telemetry_ext::QuaternionExt;

pub enum Simulation {
    Simulation(galadriel::Simulation),
    Replication(galadriel::Replication),
}

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationSettings {
    pub replicated_log_id: Option<String>,
    pub galadriel: galadriel::SimulationSettings,
    pub fc_settings: Settings,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        Self {
            replicated_log_id: None,
            galadriel: galadriel::SimulationSettings::default(),
            fc_settings: Settings::default(),
        }
    }
}

/// Trait that adds conversion methods between our vehicle state objects and the
/// measurements expected and produced by galadriel for simulation.
trait StateSimulation {
    fn measurements(&self) -> galadriel::SensorData;
    fn join_measurement(&mut self, sensors: galadriel::SensorData);
}

impl StateSimulation for VehicleState {
    fn measurements(&self) -> galadriel::SensorData {
        galadriel::SensorData {
            time: self.time,
            gyroscope: self.gyroscope,
            accelerometer1: self.accelerometer1,
            accelerometer2: self.accelerometer2,
            magnetometer: self.magnetometer,
            lp_filtered_pressure: self.pressure_baro,
            pressure: self.pressure_baro,
        }
    }

    fn join_measurement(&mut self, sensors: galadriel::SensorData) {
        self.gyroscope = sensors.gyroscope;
        self.accelerometer1 = sensors.accelerometer1;
        self.accelerometer2 = sensors.accelerometer2;
        self.magnetometer = sensors.magnetometer;
        self.pressure_baro = sensors.pressure;
        // TODO: move to state estimator?
        self.altitude_baro = self.pressure_baro.map(|p| 44307.694 * (1.0 - (p / 1013.25).powf(0.190284)));
    }
}

pub struct SimulationWorker {
    // The settings for the simulation, adjusted via GUI
    pub settings: SimulationSettings,
    // Core simulation/galadriel object and our state estimator
    sim: Simulation,
    state_estimator: StateEstimator,
    time_origin: (Instant, u32),
    done: bool,
    current_state: VehicleState,
    mode: FlightMode,
    // Remaining raw vehicle states from replicated log, if used
    remaining_states: VecDeque<VehicleState>,
    sender: Sender<(Instant, VehicleState)>,
}

impl SimulationWorker {
    pub fn run(settings: SimulationSettings, sender: Sender<(Instant, VehicleState)>) {
        let mut worker = Self::init(settings, sender);
        while !worker.done {
            worker.tick();

        }
    }

    pub fn init(mut settings: SimulationSettings, sender: Sender<(Instant, VehicleState)>) -> Self {
        let replicated_log = settings.replicated_log_id.as_ref().and_then(|id| ArchivedLog::find(id));
        if let Some(log) = replicated_log {
            if let Some(fc_settings) = log.fc_settings() {
                // TODO: do we want to copy other settings?
                settings.fc_settings.orientation = fc_settings.orientation;
            }

            let mut states: VecDeque<VehicleState> = log.flash_states().unwrap().into_iter().collect();
            let measurements = states.iter()
                .filter(|vs| vs.pressure_baro.is_some())
                .map(|state| state.measurements())
                .collect();
            let initial_orientation = states.iter().find_map(|vs| vs.orientation).unwrap();
            let repl = galadriel::Replication::new(1, measurements);

            let current_state = states.pop_front().unwrap();
            let time_origin = (Instant::now(), current_state.time);
            sender.send((time_origin.0, current_state.clone())).unwrap();

            let state_estimator = StateEstimator::new_with_quat(1000.0, settings.fc_settings.clone(), initial_orientation);

            Self {
                settings,
                sim: Simulation::Replication(repl),
                state_estimator,
                time_origin,
                done: false,
                current_state,
                mode: FlightMode::Armed,
                remaining_states: states,
                sender,
            }
        } else {
            settings.fc_settings.orientation = Orientation::ZUp;

            let sim = Simulation::Simulation(galadriel::Simulation::new(&settings.galadriel));
            let freq = 1000.0 / (settings.galadriel.delta_time as f32);
            let state_estimator = StateEstimator::new(freq, settings.fc_settings.clone());

            Self {
                settings,
                sim,
                state_estimator,
                time_origin: (Instant::now(), 0),
                done: false,
                current_state: VehicleState::default(), // TODO
                mode: FlightMode::HardwareArmed,
                remaining_states: VecDeque::new(),
                sender,
            }
        }
    }

    pub fn tick(&mut self) -> bool {
        self.current_state.time += self.settings.galadriel.delta_time;

        let plot = match &mut self.sim {
            Simulation::Replication(repl) => {
                let mut plot = false;

                // If we're replicating a log and we reached the next recorded step, use that
                if self.remaining_states.front().map(|s| self.current_state.time >= s.time).unwrap_or(false) {
                    let t = self.current_state.time;
                    self.current_state = self.remaining_states.pop_front().unwrap();
                    self.current_state.mode = Some(self.mode);
                    self.current_state.time = t;
                    plot = true;
                }

                if let Some(sensors) = repl.next() {
                    self.current_state.join_measurement(sensors);
                } else {
                    self.done = true;
                }

                if self.mode == FlightMode::Landed {
                    self.done = true;
                }

                plot
            }
            Simulation::Simulation(sim) => {
                if self.mode == FlightMode::RecoveryDrogue {
                    sim.rocket.parachutes[0].open = true;
                } else if self.mode == FlightMode::RecoveryMain {
                    sim.rocket.parachutes[0].open = false;
                    sim.rocket.parachutes[1].open = true;
                }

                if self.current_state.time == 1000 {
                    self.mode = FlightMode::Armed;
                }

                self.current_state.mode = Some(self.mode);
                if let Some(sensors) = sim.next() {
                    self.current_state.join_measurement(sensors);
                } else {
                    self.done = true;
                }

                let gps = self.current_state.time % (100 - 100 % self.settings.galadriel.delta_time) == 0 && !(
                    sim.state.flight_phase == FlightPhase::Burn ||
                    sim.state.flight_phase == FlightPhase::Coast ||
                    sim.state.flight_phase == FlightPhase::Descent && sim.time_in_phase() < 5.0
                );
                self.current_state.gps = gps.then_some(GPSDatum {
                    utc_time: None,
                    // TODO: noise
                    latitude: Some(sim.latitude()),
                    longitude: Some(sim.longitude()),
                    altitude: Some(sim.altitude_asl() + 105.0), // TODO: offset
                    fix: GPSFixType::AutonomousFix,
                    hdop: 100,
                    num_satellites: 10,
                });

                if let Some(thrusters) = sim.rocket.thrusters.as_mut() {
                    self.current_state.acs_tank_pressure = Some(thrusters.tank_pressure());
                    thrusters.set_valve(self.state_estimator.thruster_valve(thrusters.tank_pressure()));
                }

                // We don't really want to show every single tick in the plot. For simulations,
                // we plot 100Hz, so every 50th simulated state.
                self.settings.galadriel.delta_time >= 50 ||
                    self.current_state.time % (50 - 50 % self.settings.galadriel.delta_time) == 0
            }
        };

        // update state estimation with sampled sensor values
        self.state_estimator.update(
            std::num::Wrapping(self.current_state.time),
            self.mode,
            self.current_state.gyroscope,
            self.current_state.accelerometer1,
            self.current_state.accelerometer2,
            self.current_state.magnetometer,
            self.current_state.altitude_baro,
            self.current_state.gps.clone(),
        );

        if let Some(mode) = self.state_estimator.new_mode(8400) {
            self.mode = mode;
            self.current_state.mode = Some(mode);
        }

        if plot {
            self.current_state.orientation = self.state_estimator.orientation;
            self.current_state.euler_angles = self.state_estimator.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI);
            self.current_state.elevation = self.state_estimator.orientation.map(|q| q.elevation());
            self.current_state.azimuth = self.state_estimator.orientation.map(|q| q.azimuth());
            self.current_state.vertical_speed = Some(self.state_estimator.vertical_speed());
            self.current_state.vertical_accel = Some(self.state_estimator.vertical_acceleration());
            self.current_state.altitude_asl = Some(self.state_estimator.altitude_asl());
            self.current_state.altitude_ground_asl = Some(self.state_estimator.altitude_ground);
            self.current_state.apogee_asl = self.state_estimator.apogee_asl(self.current_state.acs_tank_pressure.unwrap_or_default());
            self.current_state.latitude = self.state_estimator.latitude();
            self.current_state.longitude = self.state_estimator.longitude();
            self.current_state.thruster_valve_state = Some(self.state_estimator.last_valve_state);

            let mut sim_state = SimulatedState {
                orientation: None,
                euler_angles: None,
                elevation: None,
                azimuth: None,
                vertical_accel: None,
                vertical_speed: None,
                kalman_x: self.state_estimator.kalman.x,
                kalman_P: self.state_estimator.kalman.P.diagonal(),
                kalman_R: self.state_estimator.kalman.R.diagonal(),
                apogee_error: Some(self.state_estimator.last_apogee_error),
                ..Default::default()
            };

            if let Simulation::Simulation(sim) = &self.sim {
                let (r, p, y) = sim.state.orientation.euler_angles();
                sim_state.orientation = Some(sim.state.orientation);
                sim_state.euler_angles = Some(Vector3::new(r, p, y) * 180.0 / PI);
                sim_state.elevation = Some(sim.state.orientation.elevation());
                sim_state.azimuth = Some(sim.state.orientation.azimuth());
                sim_state.vertical_speed = Some(sim.state.velocity.z);
                sim_state.vertical_accel = Some(sim.state.acceleration.z);
                sim_state.mass = Some(sim.rocket.last_mass);
                sim_state.motor_mass = Some(sim.rocket.motor.last_mass);
                sim_state.thruster_propellant_mass = sim.rocket.thrusters.as_ref().map(|t| t.propellant_mass);
                sim_state.force_drag = Some(sim.rocket.last_drag);
                sim_state.force_thrust = Some(sim.rocket.last_thrust);
            }

            self.current_state.sim = Some(Box::new(sim_state));

            let t = self.time_origin.0 + Duration::from_millis((self.current_state.time.wrapping_sub(self.time_origin.1)) as u64);
            if let Err(_) = self.sender.send((t, self.current_state.clone())) {
                self.done = true;
            }
        }

        self.done
    }
}

#[derive(Default)]
pub struct SimulationDataSource {
    // The settings for the simulation, adjusted via GUI
    pub settings: SimulationSettings,
    // Produced vehicle states that are plotted by GUI
    stored_vehicle_states: Vec<(Instant, VehicleState)>,
    playback: Option<PlaybackState>,
    playback_speed: usize,
    // Handles for interacting with worker thread
    worker_thread: Option<JoinHandle<()>>,
    state_receiver: Option<Receiver<(Instant, VehicleState)>>,
}

impl SimulationDataSource {
    pub fn replicate(replicated_log_id: String) -> Self {
        let settings = SimulationSettings {
            replicated_log_id: Some(replicated_log_id),
            ..Default::default()
        };
        Self {
            settings,
            ..Default::default()
        }
    }
}

impl DataSource for SimulationDataSource {
    fn update(&mut self, ctx: &egui::Context) {
        if self.worker_thread.is_none() || self.state_receiver.is_none() {
            self.reset();
        }

        if let Some(receiver) = &self.state_receiver {
            let new: Vec<_> = receiver.try_iter().collect();
            if new.len() > 0 {
                // We're still getting states in, keep skipping to end
                *self.playback_state_mut() = new.last().map(|(t, _vs)| PlaybackState::Paused(*t));
            }
            self.stored_vehicle_states.extend(new);
        }

        if self.worker_thread.as_ref().map(|h| !h.is_finished()).unwrap_or(false) {
            ctx.request_repaint();
        }

        self.update_playback(ctx);
    }

    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)> {
        let inst = self.end().unwrap_or(Instant::now());
        let i = self.stored_vehicle_states.partition_point(|(t, _)| t <= &inst);
        self.stored_vehicle_states[..i].iter()
    }

    fn reset(&mut self) {
        let (sender, receiver) = channel::<(Instant, VehicleState)>();
        let settings = self.settings.clone();
        self.worker_thread = Some(std::thread::spawn(move || {
            SimulationWorker::run(settings, sender);
        }));
        self.state_receiver = Some(receiver);
        self.stored_vehicle_states.truncate(0);
        self.playback = None;
    }

    fn end(&self) -> Option<Instant> {
        self.playback_end()
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        self.playback_ui(ui)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

impl ReplayableDataSource for SimulationDataSource {
    fn playback_state(&self) -> Option<PlaybackState> {
        self.playback.clone()
    }

    fn playback_state_mut(&mut self) -> &mut Option<PlaybackState> {
        &mut self.playback
    }

    fn playback_speed(&self) -> usize {
        self.playback_speed
    }

    fn playback_speed_mut(&mut self) -> &mut usize {
        &mut self.playback_speed
    }

    fn all_vehicle_states(&self) -> &Vec<(Instant, VehicleState)> {
        &self.stored_vehicle_states
    }
}
