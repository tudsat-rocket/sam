//! A simulation data source

use std::convert::Infallible;
use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;

use archive::ArchivedLog;
use galadriel::SensorData;
use shared_types::settings::*;
use shared_types::telemetry::*;
use state_estimator::StateEstimator;
use telemetry::*;

use crate::backend::*;

const TELEMETRY_BUFFER_SIZE: usize = 512;

pub enum Simulation {
    Simulation(galadriel::Simulation),
    Replication(galadriel::Replication),
}

pub struct SimulationWorker {
    time: u32,
    // The settings for the simulation, adjusted via GUI
    pub settings: galadriel::SimulationSettings,
    pub fc_settings: Settings,
    telemetry_schema: &'static TelemetrySchema,
    // Core simulation/galadriel object and our state estimator
    sim: Simulation,
    state_estimator: StateEstimator,
    sensors: Option<SensorData>,
    mode: FlightMode,
    sender: Sender<(u32, heapless::Vec<u8, TELEMETRY_BUFFER_SIZE>)>,
}

// TODO: include simulation metrics (TrueXYZ)
impl MetricSource for SimulationWorker {
    type Error = Infallible;

    fn write_metric<const N: usize>(
        &mut self, // TODO: try to reduce this to non-mutable reference
        w: &mut telemetry::TelemetryMessageWriter<N>,
        metric: Metric,
        repr: telemetry::Representation,
    ) -> Result<(), Self::Error> {
        use Metric::*;

        match metric {
            FlightMode => w.write_enum(repr, self.mode as u8),
            // State estimator
            Orientation(_)
            | Elevation
            | Azimuth
            | AccelerationWorldSpace(_)
            | VelocityWorldSpace(_)
            | PositionWorldSpace(_)
            | Latitude
            | Longitude
            | GroundAltitudeASL
            | ApogeeAltitudeASL
            | GroundSpeed
            | KalmanStateCovariance(_, _) => self.state_estimator.write_metric(w, metric, repr),
            // Raw sensor values
            RawAngularVelocity(_, _)
            | RawAcceleration(_, _)
            | RawMagneticFluxDensity(_, _)
            | RawBarometricAltitude(_)
            | Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611))
            | GpsFix
            | GpsLatitude
            | GpsLongitude
            | GpsAltitude
            | GpsHdop
            | GpsSatellites => self.sensors.as_mut().unwrap().write_metric(w, metric, repr),
            // Pressures
            Pressure(_) => w.write_float(repr, 0.0),
            Temperature(_) => w.write_float(repr, 30.0),
            BatteryVoltage(_) => w.write_float(repr, 0),
            BatteryCurrent(_) => w.write_float(repr, 0),
            SupplyVoltage => w.write_float(repr, 0),
            // Misc.
            CpuUtilization => w.write_float(repr, 0.0),
            FlashPointer => w.write_float(repr, 0.0),
            UplinkRssi => w.write_float(repr, 72.0),
            TransmitPower => w.write_enum(repr, 0),
            m => todo!("{m:?}"),
        }
    }
}

impl SimulationWorker {
    pub fn run(
        settings: galadriel::SimulationSettings,
        fc_settings: Settings,
        replicated_log_id: Option<String>,
        telemetry_schema: &'static TelemetrySchema,
        sender: Sender<(u32, heapless::Vec<u8, TELEMETRY_BUFFER_SIZE>)>,
    ) {
        let mut worker = Self::init(settings, fc_settings, replicated_log_id, telemetry_schema, sender);
        while !worker.tick() {}
    }

    pub fn init(
        settings: galadriel::SimulationSettings,
        mut fc_settings: Settings,
        replicated_log_id: Option<String>,
        telemetry_schema: &'static TelemetrySchema,
        sender: Sender<(u32, heapless::Vec<u8, TELEMETRY_BUFFER_SIZE>)>,
    ) -> Self {
        let replicated_log = replicated_log_id.as_ref().and_then(|id| ArchivedLog::find(id));
        if let Some(_log) = replicated_log {
            todo!();
        } else {
            fc_settings.orientation = Orientation::ZUp;

            let sim = Simulation::Simulation(galadriel::Simulation::new(&settings));
            let freq = 1000.0 / (settings.delta_time as f32);
            let state_estimator = StateEstimator::new(freq, fc_settings.clone());

            Self {
                time: 0,
                settings,
                fc_settings,
                telemetry_schema,
                sim,
                state_estimator,
                sensors: None,
                mode: FlightMode::HardwareArmed,
                sender,
            }
        }
    }

    pub fn tick(&mut self) -> bool {
        self.time += self.settings.delta_time;

        // REFACTOR: sensor handling for new sensor id stuff and more sensors in general

        let sensors = match &mut self.sim {
            Simulation::Replication(_repl) => {
                todo!();
            }
            Simulation::Simulation(sim) => {
                if self.mode == FlightMode::RecoveryDrogue {
                    sim.rocket.parachutes[0].open = true;
                } else if self.mode == FlightMode::RecoveryMain {
                    sim.rocket.parachutes[0].open = false;
                    sim.rocket.parachutes[1].open = true;
                }

                if self.time == 1000 {
                    self.mode = FlightMode::Armed;
                }

                let Some(sensors) = sim.next() else {
                    return true;
                };

                sensors
            }
        };

        // update state estimation with sampled sensor values
        self.state_estimator.update(
            std::num::Wrapping(self.time),
            self.mode,
            sensors.gyroscope,
            sensors.accelerometer1,
            sensors.accelerometer2,
            sensors.magnetometer,
            sensors.pressure.map(|p| 44330.769 * (1.0 - (p / 1012.5).powf(0.190223))),
            sensors.gps.clone(),
        );

        self.sensors = Some(sensors);

        if let Some(mode) = self.state_estimator.new_mode(8400) {
            self.mode = mode;
        }

        if let Some(message) = self.telemetry_schema.message(self, self.time) {
            if let Err(_) = self.sender.send((self.time, message)) {
                return true;
            }
        }

        // TODO

        false
    }
}

#[derive(Default)]
pub struct SimulationBackend {
    // The settings for the simulation, adjusted via GUI
    pub settings: galadriel::SimulationSettings,
    pub fc_settings: Settings,
    pub replicated_log_id: Option<String>,
    pub telemetry_schema: &'static TelemetrySchema,
    // Produced vehicle states that are plotted by GUI
    data_store: DataStore,
    playback: Option<PlaybackState>,
    playback_speed: usize,
    // Handles for interacting with worker thread
    worker_thread: Option<JoinHandle<()>>,
    state_receiver: Option<Receiver<(u32, heapless::Vec<u8, TELEMETRY_BUFFER_SIZE>)>>,
}

impl SimulationBackend {
    pub fn simulate() -> Self {
        Self::default()
    }

    pub fn replicate(replicated_log_id: String) -> Self {
        Self {
            replicated_log_id: Some(replicated_log_id),
            ..Default::default()
        }
    }
}

impl BackendVariant for SimulationBackend {
    fn update(&mut self, ctx: &egui::Context) {
        if self.worker_thread.is_none() || self.state_receiver.is_none() {
            self.reset();
        }

        if let Some(receiver) = &self.state_receiver {
            let new: Vec<_> = receiver.try_iter().collect();
            for (time, message) in new {
                self.data_store.ingest_message(self.telemetry_schema, time, message);
                *self.playback_state_mut() = self.data_store.last_time().map(|t| PlaybackState::Paused(t));
            }
        }

        if self.worker_thread.as_ref().map(|h| !h.is_finished()).unwrap_or(false) {
            ctx.request_repaint();
        }

        self.update_playback(ctx);
    }

    fn data_store<'a>(&'a self) -> &'a DataStore {
        &self.data_store
    }

    fn reset(&mut self) {
        let (sender, receiver) = channel::<(u32, heapless::Vec<u8, TELEMETRY_BUFFER_SIZE>)>();
        let settings = self.settings.clone();
        let fc_settings = self.fc_settings.clone();
        let log_id = self.replicated_log_id.clone();
        let schema = self.telemetry_schema;
        self.worker_thread = Some(
            std::thread::Builder::new()
                .name("sam-simulation".to_owned())
                .spawn(move || {
                    SimulationWorker::run(settings, fc_settings, log_id, schema, sender);
                })
                .unwrap(),
        );
        self.state_receiver = Some(receiver);
        self.playback = None;
        self.data_store = DataStore::default();
    }

    fn end(&self) -> Option<f64> {
        self.playback_end()
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        self.playback_ui(ui)
    }
}

impl ReplayableBackendVariant for SimulationBackend {
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
}
