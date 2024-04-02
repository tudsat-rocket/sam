//! A simulation data source

use std::any::Any;
use std::collections::VecDeque;
use std::sync::mpsc::SendError;
use std::time::Duration;
use std::slice::Iter;
use std::f32::consts::PI;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
use galadriel::FlightPhase;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use nalgebra::Vector3;

use mithril::settings::*;
use mithril::telemetry::*;
use mithril::state_estimation::StateEstimator;

use crate::data_source::*;
use crate::gui::windows::ArchivedLog;
use crate::telemetry_ext::QuaternionExt;

pub enum Simulation {
    Simulation(galadriel::Simulation),
    Replication(galadriel::Replication),
}

#[derive(Clone, Debug, PartialEq)]
pub struct SimulationSettings {
    pub replicated_log: Option<ArchivedLog>,
    pub galadriel: galadriel::SimulationSettings,
    pub fc_settings: Settings,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        Self {
            replicated_log: None,
            galadriel: galadriel::SimulationSettings::default(),
            fc_settings: Settings::default(),
        }
    }
}

/// Trait that adds conversion methods between our vehicle state objects and the
/// measurements expected and produced by galadriel for simulation.
trait StateSimulation {
    // TODO: names
    fn measurements(&self) -> galadriel::SensorData;
    fn from_measurement(sensors: galadriel::SensorData) -> Self;
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
            pressure: self.pressure_baro,
        }
    }

    fn from_measurement(sensors: galadriel::SensorData) -> Self {
        let mut state = Self::default();
        state.join_measurement(sensors);
        state
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

#[derive(Default)]
pub struct SimulationDataSource {
    // The settings for the simulation, adjusted via GUI
    pub settings: SimulationSettings,
    // Core simulation/galadriel object and our state estimator
    sim: Option<Simulation>,
    state_estimator: Option<StateEstimator>,
    done: bool,
    current_state: VehicleState,
    mode: FlightMode,
    // Produced vehicle states that are plotted by GUI
    stored_vehicle_states: Vec<(Instant, VehicleState)>,
    // Remaining raw vehicle states from replicated log, if used
    remaining_states: VecDeque<VehicleState>,
    playback: Option<PlaybackState>,
}

impl SimulationDataSource {
    pub fn replicate(replicated_log: ArchivedLog) -> Self {
        let settings = SimulationSettings {
            replicated_log: Some(replicated_log),
            ..Default::default()
        };
        Self {
            settings,
            ..Default::default()
        }
    }

    pub fn init_simulation(&mut self) -> Simulation {
        if let Some(log) = self.settings.replicated_log {
            if log == ArchivedLog::Euroc23 {
                self.settings.fc_settings.orientation = Orientation::ZDown;
            } else {
                self.settings.fc_settings.orientation = Orientation::ZUp;
            }

            // TODO: wasm, hardcoded tick rate
            let mut states: VecDeque<VehicleState> = log.flash_states().unwrap().into_iter().collect();
            let measurements = states.iter()
                .filter(|vs| vs.pressure_baro.is_some())
                .map(|state| state.measurements())
                .collect();
            let repl = galadriel::Replication::new(1, measurements);
            self.current_state = states.pop_front().unwrap();
            self.stored_vehicle_states.push((Instant::now(), self.current_state.clone()));
            self.remaining_states = states;
            self.mode = FlightMode::Armed;
            Simulation::Replication(repl)
        } else {
            self.settings.fc_settings.orientation = Orientation::ZUp;
            Simulation::Simulation(galadriel::Simulation::new(&self.settings.galadriel))
        }
    }

    pub fn tick(&mut self) -> bool {
        if self.done {
            return true;
        }

        let Some(sim) = self.sim.as_mut() else {
            return true;
        };

        let Some(state_estimator) = self.state_estimator.as_mut() else {
            return true;
        };

        self.current_state.time += 1;

        let plot = match sim {
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

                let gps = self.current_state.time % 100 == 0 && !(
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
                    // TODO: this is slightly awkward
                    thrusters.set_valve(match state_estimator.thruster_valve() {
                        ThrusterValveState::Closed => galadriel::thrusters::ValveState::Closed,
                        ThrusterValveState::OpenPrograde => galadriel::thrusters::ValveState::OpenPrograde,
                        ThrusterValveState::OpenRetrograde => galadriel::thrusters::ValveState::OpenRetrograde,
                    });
                }

                // We don't really want to show every single tick in the plot. For simulations,
                // we plot 100Hz, so every 10th simulated state.
                self.current_state.time % 10 == 0
            }
        };

        // update state estimation with sampled sensor values
        state_estimator.update(
            std::num::Wrapping(self.current_state.time),
            self.mode,
            self.current_state.gyroscope,
            self.current_state.accelerometer1,
            self.current_state.accelerometer2,
            self.current_state.magnetometer,
            self.current_state.altitude_baro,
            self.current_state.gps.clone(),
        );

        if let Some(mode) = state_estimator.new_mode(8400, None) {
            self.mode = mode;
            self.current_state.mode = Some(mode);
        }

        if plot {
            self.current_state.orientation = state_estimator.orientation;
            self.current_state.euler_angles = state_estimator.orientation.map(|q| q.euler_angles()).map(|(r, p, y)| Vector3::new(r, p, y) * 180.0 / PI);
            self.current_state.elevation = state_estimator.orientation.map(|q| q.elevation());
            self.current_state.azimuth = state_estimator.orientation.map(|q| q.azimuth());
            self.current_state.vertical_speed = Some(state_estimator.vertical_speed());
            self.current_state.vertical_accel = state_estimator.acceleration_world_raw().map(|v| v.z);
            self.current_state.vertical_accel_filtered = Some(state_estimator.vertical_acceleration());
            self.current_state.altitude_asl = Some(state_estimator.altitude_asl());
            self.current_state.altitude_ground_asl = Some(state_estimator.altitude_ground);
            self.current_state.apogee_asl = state_estimator.apogee_asl();
            self.current_state.latitude = state_estimator.latitude();
            self.current_state.longitude = state_estimator.longitude();
            self.current_state.thruster_valve = Some(state_estimator.thruster_valve());

            let mut sim_state = SimulatedState {
                orientation: None,
                euler_angles: None,
                elevation: None,
                azimuth: None,
                vertical_accel: None,
                vertical_speed: None,
                kalman_x: state_estimator.kalman.x,
                kalman_P: state_estimator.kalman.P.diagonal(),
                kalman_R: state_estimator.kalman.R.diagonal(),
                ..Default::default()
            };

            if let Some(Simulation::Simulation(sim)) = &self.sim {
                let (r, p, y) = sim.state.orientation.euler_angles();
                sim_state.orientation = Some(sim.state.orientation);
                sim_state.euler_angles = Some(Vector3::new(r, p, y) * 180.0 / PI);
                sim_state.elevation = Some(sim.state.orientation.elevation());
                sim_state.azimuth = Some(sim.state.orientation.azimuth());
                sim_state.vertical_speed = Some(sim.state.velocity.z);
                sim_state.vertical_accel = Some(sim.state.acceleration.z);
                sim_state.mass = Some(0.0); // TODO
                sim_state.motor_mass = Some(0.0); // TODO
                sim_state.thruster_propellant_mass = sim.rocket.thrusters.as_ref().map(|t| t.propellant_mass);
            }

            self.current_state.sim = Some(Box::new(sim_state));

            let t = self.stored_vehicle_states.first()
                .map(|(t, vs)| *t + Duration::from_millis((self.current_state.time.wrapping_sub(vs.time)) as u64))
                .unwrap_or(Instant::now());
            self.stored_vehicle_states.push((t, self.current_state.clone()));
        }

        self.done
    }
}

impl DataSource for SimulationDataSource {
    fn update(&mut self, ctx: &egui::Context) {
        if self.sim.is_none() {
            self.sim = Some(self.init_simulation());
            self.state_estimator = Some(StateEstimator::new(1000.0, self.settings.fc_settings.clone()));
        }

        // TODO: move to another thread
        while !self.tick() {}

        self.update_playback(ctx);
    }

    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)> {
        let inst = self.end().unwrap_or(Instant::now());
        let i = self.stored_vehicle_states.partition_point(|(t, _)| t <= &inst);
        self.stored_vehicle_states[..i].iter()
    }

    fn reset(&mut self) {
        self.sim = None;
        self.state_estimator = None;
        self.current_state = VehicleState::default();
        self.stored_vehicle_states.truncate(0);
        self.remaining_states.truncate(0);
        self.done = false;
        self.mode = FlightMode::HardwareArmed;
        self.playback = None;
    }

    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn end(&self) -> Option<Instant> {
        self.playback_end()
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        // TODO: maybe computation times or something?
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

    fn all_vehicle_states(&self) -> &Vec<(Instant, VehicleState)> {
        &self.stored_vehicle_states
    }
}
