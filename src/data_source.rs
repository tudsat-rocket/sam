//! Data sources serve as an abstraction for the origin of the displayed data.

use std::slice::Iter;
use std::sync::mpsc::SendError;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::settings::AppSettings;
use crate::simulation::SimulationSettings;
use crate::state::*;

pub mod log_file;
pub mod serial;
pub mod simulation;

pub use log_file::LogFileDataSource;
pub use serial::*;
pub use simulation::SimulationDataSource;

/// Trait shared by all data sources.
pub trait DataSource {
    /// Called every frame.
    fn update(&mut self, ctx: &egui::Context);
    /// Return an iterator over all known states of the vehicle.
    fn vehicle_states<'a>(&'a self) -> Iter<'_, (Instant, VehicleState)>;

    /// Return the current flight computer settings, if known.
    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings>;
    /// Return the current flight computer settings, if known.
    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings>;

    /// Reset data source if applicable.
    fn reset(&mut self);

    /// Send an uplink message to the connected device if applicable.
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>>;
    /// Send an authenticated uplink command
    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>>;

    /// The minimum fps required for the data source. Occasional redraws
    /// are necessary if data source is live.
    fn minimum_fps(&self) -> Option<u64>;
    fn end(&self) -> Option<Instant>;

    fn status_bar_ui(&mut self, _ui: &mut egui::Ui) {
    }

    fn link_quality(&self) -> Option<f32> {
        None
    }

    fn is_log_file(&self) -> bool {
        false
    }

    fn simulation_settings(&mut self) -> Option<&mut SimulationSettings> {
        None
    }

    fn apply_settings(&mut self, _settings: &AppSettings) {}
}
