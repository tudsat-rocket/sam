//! A simulation data source

use std::sync::mpsc::SendError;
use std::time::Duration;
use std::slice::Iter;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use egui::Color32;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::data_source::DataSource;
use crate::simulation::*;
use crate::state::VehicleState;

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
        while !self.state.as_mut().unwrap().tick() {
            let sim_state = self.state.as_ref().unwrap();
            if !sim_state.plottable() {
                continue;
            }

            let vehicle_state = sim_state.into();
            let time = self
                .vehicle_states
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

    fn minimum_fps(&self) -> Option<u64> {
        None
    }

    fn simulation_settings(&mut self) -> Option<&mut SimulationSettings> {
        Some(&mut self.settings)
    }

    fn end(&self) -> Option<Instant> {
        self.vehicle_states.last().map(|(t, _vs)| *t)
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        ui.colored_label(Color32::KHAKI, "Simulation");
        // TODO: maybe computation times or something?
    }
}
