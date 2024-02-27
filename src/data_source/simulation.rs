//! A simulation data source

use std::any::Any;
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

#[derive(Default)]
pub struct SimulationDataSource {
    pub settings: SimulationSettings,
    state: Option<SimulationState>,
    vehicle_states: Vec<(Instant, VehicleState)>,
}

impl DataSource for SimulationDataSource {
    fn update(&mut self, _ctx: &egui::Context) {
        if self.state.is_none() {
            self.state = Some(SimulationState::initialize(&self.settings));
        }

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
    }

    fn vehicle_states(&self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn fc_settings(&mut self) -> Option<&Settings> {
        None
    }

    fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
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

    fn end(&self) -> Option<Instant> {
        self.vehicle_states.last().map(|(t, _vs)| *t)
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        ui.colored_label(Color32::KHAKI, "Simulation");
        // TODO: maybe computation times or something?
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
