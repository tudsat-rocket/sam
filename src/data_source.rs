//! Data sources serve as an abstraction for the origin of the displayed data.

use std::any::Any;
use std::slice::Iter;
use std::sync::mpsc::SendError;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::settings::AppSettings;

pub mod log_file;
pub mod serial;
pub mod simulation;

pub use log_file::LogFileDataSource;
pub use serial::*;
pub use simulation::{SimulationDataSource, SimulationSettings};

/// Trait shared by all data sources.
#[allow(clippy::result_large_err)]
pub trait DataSource {
    /// Called every frame.
    fn update(&mut self, ctx: &egui::Context);
    /// Return an iterator over all known states of the vehicle.
    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)>;

    /// Return the current flight computer settings, if known.
    fn fc_settings(&mut self) -> Option<&Settings> {
        None
    }
    /// Return the current flight computer settings, if known.
    fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
        None
    }

    /// Reset data source if applicable.
    fn reset(&mut self);

    /// Send an uplink message to the connected device if applicable.
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>>;
    /// Send an authenticated uplink command
    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>>;

    fn end(&self) -> Option<Instant>;

    fn status_bar_ui(&mut self, _ui: &mut egui::Ui) {
    }

    fn link_quality(&self) -> Option<f32> {
        None
    }

    fn apply_settings(&mut self, _settings: &AppSettings) {}

    /// Helper methods to allow us to downcast from a boxed DataSource trait to a specific
    /// implementation type.
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}
