//! Data sources serve as an abstraction for the origin of the displayed data.

use std::any::Any;
use std::slice::Iter;
use std::sync::mpsc::SendError;
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use egui::{Layout, RichText, Slider};

use shared_types::settings::*;
use shared_types::telemetry::*;

use crate::settings::AppSettings;

pub mod log_file;
pub mod serial;
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
pub mod simulation;

pub use log_file::LogFileDataSource;
pub use serial::*;
#[cfg(all(not(target_arch = "wasm32"), not(target_os = "android")))]
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

#[derive(Debug, Clone)]
pub enum PlaybackState {
    Playing(f32),
    Paused(Instant),
}

pub trait ReplayableDataSource: DataSource {
    fn playback_state(&self) -> Option<PlaybackState>;
    fn playback_state_mut(&mut self) -> &mut Option<PlaybackState>;
    fn all_vehicle_states(&self) -> &Vec<(Instant, VehicleState)>;

    fn update_playback(&mut self, ctx: &egui::Context) {
        let last_state = self.all_vehicle_states().last().map(|(t, _vs)| t).copied();

        // We default to being paused at the end of the log file.
        if self.playback_state().is_none() {
            *self.playback_state_mut() = last_state.map(|l| PlaybackState::Paused(l));
        }

        if let Some(PlaybackState::Playing(_)) = self.playback_state() {
            ctx.request_repaint_after(Duration::from_millis(16));

            if self.end().and_then(|i| last_state.map(|l| i > l)).unwrap_or(false) {
                *self.playback_state_mut() = last_state.map(|l| PlaybackState::Paused(l));
            }
        }
    }

    fn playback_end(&self) -> Option<Instant> {
        match self.playback_state().as_ref() {
            Some(PlaybackState::Playing(offset)) => {
                if *offset < 0.0 {
                    Some(Instant::now() - Duration::from_secs_f32(offset.abs()))
                } else {
                    Some(Instant::now() + Duration::from_secs_f32(offset.abs()))
                }
            },
            Some(PlaybackState::Paused(inst)) => Some(*inst),
            None => None
        }
    }

    fn playpause(&mut self) {
        match self.playback_state() {
            Some(PlaybackState::Playing(offset)) => {
                let inst = if offset < 0.0 {
                    Instant::now() - Duration::from_secs_f32(offset.abs())
                } else {
                    Instant::now() + Duration::from_secs_f32(offset.abs())
                };
                *self.playback_state_mut() = Some(PlaybackState::Paused(inst));
            },
            Some(PlaybackState::Paused(inst)) => {
                let offset = if inst > Instant::now() {
                    (inst - Instant::now()).as_secs_f32()
                } else {
                    -1.0 * (Instant::now() - inst).as_secs_f32()
                };
                *self.playback_state_mut() = Some(PlaybackState::Playing(offset));
            },
            None => {}
        }
    }

    fn skip_to_next_flightmode(&mut self, reverse: bool) {
        let Some(playback) = &self.playback_state() else {
            return;
        };

        // Discriminant means we're just comparing the enum variant, not the actual value.
        let playing = std::mem::discriminant(playback) == std::mem::discriminant(&PlaybackState::Playing(123.0));

        let Some(fm) = self.vehicle_states().rev().find_map(|(_t, vs)| vs.mode) else {
            return;
        };

        let Some(end) = self.end() else {
            return;
        };

        let i = self.all_vehicle_states().partition_point(|(t, _)| *t <= end);

        let next_flightmode = if reverse {
            self.all_vehicle_states()[..i]
                .iter()
                .rev()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != Some(fm))
                .map(|(t, _)| t)
                .next()
        } else {
            self.all_vehicle_states()[i..]
                .iter()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != Some(fm))
                .map(|(t, _)| t)
                .next()
        };

        if let Some(inst) = next_flightmode {
            *self.playback_state_mut() = Some(PlaybackState::Paused(*inst));
            if playing {
                self.playpause();
            }
        }
    }

    fn playback_ui(&mut self, ui: &mut egui::Ui) {
        let first = self.all_vehicle_states().first().map(|(t, _vs)| *t).unwrap_or(Instant::now());
        let last = self.all_vehicle_states().last().map(|(t, _vs)| *t).unwrap_or(Instant::now());
        let total = (last - first).as_secs_f32();
        let elapsed = self.end().map(|p| (p - first).as_secs_f32()).unwrap_or(0.0);

        ui.set_enabled(self.playback_state().is_some());

        if ui.button("⏮").clicked() {
            *self.playback_state_mut() = Some(PlaybackState::Paused(first));
        }
        if ui.button("⏪").clicked() {
            self.skip_to_next_flightmode(true);
        }
        let label = if let Some(PlaybackState::Playing(_)) = self.playback_state() { "⏸" } else { "⏵" };
        if ui.button(label).clicked() {
            self.playpause();
        }
        if ui.button("⏩").clicked() {
            self.skip_to_next_flightmode(false);
        }
        if ui.button("⏭").clicked() {
            *self.playback_state_mut() = Some(PlaybackState::Paused(last));
        }

        ui.with_layout(Layout::right_to_left(egui::Align::Center), |ui| {
            let t = format!("{:02.0}:{:02.0}:{:02.0}/{:02.0}:{:02.0}:{:02.0}",
                elapsed / 3600.0, (elapsed / 60.0) % 60.0, elapsed % 60.0,
                total / 3600.0, (total / 60.0) % 60.0, total % 60.0
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
            if response.drag_released() {
                *self.playback_state_mut() = match self.playback_state() {
                    Some(PlaybackState::Playing(offset)) => {
                        Some(PlaybackState::Playing(offset + (new - current) * total))
                    },
                    Some(PlaybackState::Paused(_)) => {
                        Some(PlaybackState::Paused(first + Duration::from_secs_f32(total * new)))
                    },
                    None => None
                }
            }
        });
    }
}
