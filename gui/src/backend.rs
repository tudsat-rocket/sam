//! Data sources serve as an abstraction for the origin of the displayed data.

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

#[cfg(not(target_arch = "wasm32"))]
use tokio::sync::mpsc::error::SendError;
#[cfg(target_arch = "wasm32")]
#[derive(Debug)]
pub struct SendError<T>(T);

use egui::{Layout, RichText, Slider};

use shared_types::settings::*;
use shared_types::telemetry::*;
use telemetry::{DataStore, Metric};

use crate::settings::AppSettings;

pub mod log_file;
pub mod noop;
#[cfg(not(target_arch = "wasm32"))]
pub mod serial;
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub mod simulation;
#[cfg(not(target_arch = "wasm32"))]
pub mod udp;

pub use log_file::*;
pub use noop::*;
#[cfg(not(target_arch = "wasm32"))]
pub use serial::*;
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub use simulation::*;
#[cfg(not(target_arch = "wasm32"))]
pub use udp::*;

const PLAYBACK_SPEEDS: [(&'static str, f64); 7] = [
    ("1x", 1.0),
    ("2x", 2.0),
    ("5x", 5.0),
    ("10x", 10.0),
    ("⅒ x", 0.1),
    ("⅕ x", 0.2),
    ("½ x", 0.5),
];

#[allow(clippy::result_large_err)]
pub trait BackendVariant {
    /// Called every frame.
    fn update(&mut self, ctx: &egui::Context);

    fn data_store<'a>(&'a self) -> &'a DataStore;

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
    fn send(&mut self, _msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }
    /// Send an authenticated uplink command
    fn send_command(&mut self, _cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn end(&self) -> Option<f64>;

    fn status_bar_ui(&mut self, _ui: &mut egui::Ui) {}

    fn link_quality(&self) -> Option<f32> {
        None
    }

    fn apply_settings(&mut self, _settings: &AppSettings) {}
}

#[derive(Debug, Clone)]
pub enum PlaybackState {
    Playing(Instant, f64),
    Paused(f64),
}

pub trait ReplayableBackendVariant: BackendVariant {
    // TODO: do this better?
    fn playback_state(&self) -> Option<PlaybackState>;
    fn playback_state_mut(&mut self) -> &mut Option<PlaybackState>;
    fn playback_speed(&self) -> usize;
    fn playback_speed_mut(&mut self) -> &mut usize;

    fn playing(&self) -> bool {
        // Discriminant means we're just comparing the enum variant, not the actual value.
        self.playback_state()
            .map(|pb| {
                std::mem::discriminant(&pb) == std::mem::discriminant(&PlaybackState::Playing(Instant::now(), 123.0))
            })
            .unwrap_or(false)
    }

    fn reached_end(&self) -> bool {
        self.playback_end().unwrap_or_default() > self.data_store().last_time().unwrap_or_default()
    }

    fn current_speed(&self) -> f64 {
        PLAYBACK_SPEEDS[self.playback_speed()].1
    }

    fn update_playback(&mut self, ctx: &egui::Context) {
        let last_time = self.data_store().last_time();

        // We default to being paused at the end of the log file.
        if self.playback_state().is_none() {
            *self.playback_state_mut() = last_time.map(|l| PlaybackState::Paused(l));
        }

        if let Some(PlaybackState::Playing(_, _)) = self.playback_state() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));

            if self.reached_end() {
                *self.playback_state_mut() = last_time.map(|l| PlaybackState::Paused(l));
            }
        }
    }

    fn playback_end(&self) -> Option<f64> {
        match self.playback_state().as_ref() {
            Some(PlaybackState::Playing(playback_start, offset)) => {
                Some(offset + playback_start.elapsed().as_secs_f64() * self.current_speed())
            }
            Some(PlaybackState::Paused(inst)) => Some(*inst),
            None => None,
        }
    }

    fn playpause(&mut self) {
        match self.playback_state() {
            Some(PlaybackState::Playing(_, _)) => {
                *self.playback_state_mut() = self.playback_end().map(|inst| PlaybackState::Paused(inst));
            }
            Some(PlaybackState::Paused(mut inst)) => {
                // we're at the end, restart when pressing play again
                if self.reached_end() {
                    inst = self.data_store().first_time().unwrap_or_default();
                }

                *self.playback_state_mut() = Some(PlaybackState::Playing(Instant::now(), inst));
            }
            None => {}
        }
    }

    fn skip_to_next_flightmode(&mut self, _reverse: bool) {
        //let fm: Option<FlightMode> = self.data_store().current_enum_value(Metric::FlightMode, self.end());

        //let Some(end) = self.end() else {
        //    return;
        //};

        //let i = self.all_vehicle_states().partition_point(|(t, _)| *t <= end);

        //let next_flightmode = if reverse {
        //    self.all_vehicle_states()[..i]
        //        .iter()
        //        .rev()
        //        .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != fm)
        //        .map(|(t, _)| t)
        //        .next()
        //} else {
        //    self.all_vehicle_states()[i..]
        //        .iter()
        //        .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != fm)
        //        .map(|(t, _)| t)
        //        .next()
        //};

        //if let Some(inst) = next_flightmode {
        //    let playing = self.playing();
        //    *self.playback_state_mut() = Some(PlaybackState::Paused(*inst));
        //    if playing {
        //        self.playpause();
        //    }
        //}
    }

    fn playback_ui(&mut self, ui: &mut egui::Ui) {
        let first = self.data_store().first_time().unwrap_or_default();
        let last = self.data_store().last_time().unwrap_or_default();
        let total = last - first;
        let elapsed = self.end().map(|p| p - first).unwrap_or(0.0);

        if self.playback_state().is_none() {
            ui.disable();
        }

        if ui.button("⏮").clicked() {
            *self.playback_state_mut() = Some(PlaybackState::Paused(first));
        }
        if ui.button("⏪").clicked() {
            self.skip_to_next_flightmode(true);
        }

        let label = if self.playing() { "⏸" } else { "⏵" };
        if ui.button(label).clicked() {
            self.playpause();
        }

        let current_speed = PLAYBACK_SPEEDS[self.playback_speed()];
        let speed_response = ui.button(current_speed.0);
        if speed_response.clicked() || speed_response.secondary_clicked() {
            // we reset the playback origin so we only modify the speed for the
            // stuff that hasn't been played yet.
            self.playpause();
            self.playpause();
            if speed_response.clicked() {
                *self.playback_speed_mut() = (self.playback_speed() + 1) % PLAYBACK_SPEEDS.len();
            } else {
                *self.playback_speed_mut() =
                    (self.playback_speed() + PLAYBACK_SPEEDS.len() - 1) % PLAYBACK_SPEEDS.len();
            }
        }

        if ui.button("⏩").clicked() {
            self.skip_to_next_flightmode(false);
        }
        if ui.button("⏭").clicked() {
            *self.playback_state_mut() = Some(PlaybackState::Paused(last));
        }

        ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
            let t = format!(
                "{:02.0}:{:02.0}:{:02.0}/{:02.0}:{:02.0}:{:02.0}",
                elapsed / 3600.0,
                (elapsed / 60.0) % 60.0,
                elapsed % 60.0,
                total / 3600.0,
                (total / 60.0) % 60.0,
                total % 60.0
            );
            ui.label(RichText::new(t).monospace().weak());

            ui.style_mut().spacing.slider_width = ui.available_width();

            // Progress Bar
            let current = if total > 0.0 { elapsed / total } else { 0.0 };
            let mut new = current;
            let slider = Slider::new(&mut new, 0.0..=1.0)
                .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.3 })
                .show_value(false);
            let response = ui.add(slider);
            if response.drag_stopped() {
                *self.playback_state_mut() = match self.playback_state() {
                    Some(PlaybackState::Playing(origin, offset)) => {
                        Some(PlaybackState::Playing(origin, offset + (new - current) * total / self.current_speed()))
                    }
                    Some(PlaybackState::Paused(_)) => Some(PlaybackState::Paused(first + total * new)),
                    None => None,
                }
            }
        });
    }
}

pub enum Backend {
    #[cfg(not(target_arch = "wasm32"))]
    Serial(SerialBackend),
    #[cfg(not(target_arch = "wasm32"))]
    Udp(UdpBackend),
    Noop(NoopBackend),
    Log(LogFileBackend),
    #[cfg(not(target_arch = "wasm32"))]
    Simulation(SimulationBackend),
}

impl Backend {
    pub fn update(&mut self, ctx: &egui::Context) {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.update(ctx),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.update(ctx),
            Self::Noop(b) => b.update(ctx),
            Self::Log(b) => b.update(ctx),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.update(ctx),
        }
    }

    pub fn data_store<'a>(&'a self) -> &'a DataStore {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.data_store(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.data_store(),
            Self::Noop(b) => b.data_store(),
            Self::Log(b) => b.data_store(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.data_store(),
        }
    }

    pub fn end(&self) -> Option<f64> {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.end(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.end(),
            Self::Noop(b) => b.end(),
            Self::Log(b) => b.end(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.end(),
        }
    }

    pub fn current_value(&self, metric: Metric) -> Option<f64> {
        self.data_store().current_float_value(metric, self.end())
    }

    pub fn current_enum<E>(&self, metric: Metric) -> Option<E>
    where
        E: TryFrom<u8>,
        <E as TryFrom<u8>>::Error: std::fmt::Debug,
    {
        self.data_store().current_enum_value(metric, self.end())
    }

    pub fn plot_metric<'a>(&'a self, key: &Metric, bounds: egui_plot::PlotBounds) -> egui_plot::PlotPoints<'a> {
        self.data_store().plot_metric(key, bounds, self.end())
    }

    pub fn timeseries<'a>(
        &'a self,
        metric: &Metric,
    ) -> Option<impl Iterator<Item = (f64, f64)> + ExactSizeIterator + std::fmt::Debug + use<'a>> {
        self.data_store().timeseries(metric, self.end())
    }

    pub fn zip_timeseries<'a, const N: usize>(
        &'a self,
        metrics: [Metric; N],
        carry_forward: f64,
    ) -> impl Iterator<Item = (f64, [Option<f64>; N])> + std::fmt::Debug + use<'a, N> {
        self.data_store().zip_timeseries(metrics, carry_forward, self.end())
    }

    pub fn link_quality(&mut self) -> Option<f64> {
        Some(0.0) // TODO
    }

    pub fn fc_settings(&mut self) -> Option<&Settings> {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.fc_settings(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.fc_settings(),
            Self::Noop(b) => b.fc_settings(),
            Self::Log(b) => b.fc_settings(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.fc_settings(),
        }
    }

    pub fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.fc_settings_mut(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.fc_settings_mut(),
            Self::Noop(b) => b.fc_settings_mut(),
            Self::Log(_b) => None,
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.fc_settings_mut(),
        }
    }

    pub fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.send(msg),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.send(msg),
            _ => Ok(()),
        }
    }

    pub fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.send_command(cmd),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.send_command(cmd),
            _ => Ok(()),
        }
    }

    pub fn flight_mode(&self) -> Option<FlightMode> {
        self.current_enum::<FlightMode>(Metric::FlightMode)
    }

    pub fn reset(&mut self) {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.reset(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.reset(),
            Self::Noop(b) => b.reset(),
            Self::Log(b) => b.reset(),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.reset(),
        }
    }

    pub fn apply_settings(&mut self, settings: &AppSettings) {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.apply_settings(settings),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.apply_settings(settings),
            Self::Noop(b) => b.apply_settings(settings),
            Self::Log(b) => b.apply_settings(settings),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.apply_settings(settings),
        }
    }

    pub fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        match self {
            #[cfg(not(target_arch = "wasm32"))]
            Self::Serial(b) => b.status_bar_ui(ui),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Udp(b) => b.status_bar_ui(ui),
            Self::Noop(b) => b.status_bar_ui(ui),
            Self::Log(b) => b.status_bar_ui(ui),
            #[cfg(not(target_arch = "wasm32"))]
            Self::Simulation(b) => b.status_bar_ui(ui),
        }
    }
}
