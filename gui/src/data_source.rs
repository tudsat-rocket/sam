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

const PLAYBACK_SPEEDS: [(&'static str, f32); 7] = [("1x", 1.0), ("2x", 2.0), ("5x", 5.0), ("10x", 10.0), ("⅒ x", 0.1), ("⅕ x", 0.2), ("½ x", 0.5)];

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
    Playing(Instant, f32),
    Paused(Instant),
}

pub trait ReplayableDataSource: DataSource {
    // TODO: do this better?
    fn playback_state(&self) -> Option<PlaybackState>;
    fn playback_state_mut(&mut self) -> &mut Option<PlaybackState>;
    fn playback_speed(&self) -> usize;
    fn playback_speed_mut(&mut self) -> &mut usize;
    fn all_vehicle_states(&self) -> &Vec<(Instant, VehicleState)>;

    fn playing(&self) -> bool {
        // Discriminant means we're just comparing the enum variant, not the actual value.
        self.playback_state()
            .map(|pb| std::mem::discriminant(&pb) == std::mem::discriminant(&PlaybackState::Playing(Instant::now(), 123.0)))
            .unwrap_or(false)
    }

    fn reached_end(&self) -> bool {
        let last_state = self.all_vehicle_states().last().map(|(t, _vs)| t).copied();
        self.end().and_then(|i| last_state.map(|l| i >= l)).unwrap_or(false)
    }

    fn current_speed(&self) -> f32 {
        PLAYBACK_SPEEDS[self.playback_speed()].1
    }

    fn update_playback(&mut self, ctx: &egui::Context) {
        let last_state = self.all_vehicle_states().last().map(|(t, _vs)| t).copied();

        // We default to being paused at the end of the log file.
        if self.playback_state().is_none() {
            *self.playback_state_mut() = last_state.map(|l| PlaybackState::Paused(l));
        }

        if let Some(PlaybackState::Playing(_, _)) = self.playback_state() {
            ctx.request_repaint_after(Duration::from_millis(16));

            if self.reached_end() {
                *self.playback_state_mut() = last_state.map(|l| PlaybackState::Paused(l));
            }
        }
    }

    fn playback_end(&self) -> Option<Instant> {
        match self.playback_state().as_ref() {
            Some(PlaybackState::Playing(playback_start, offset)) => {
                // in real time
                let playback_end = if *offset < 0.0 {
                    Instant::now() - Duration::from_secs_f32(offset.abs())
                } else {
                    Instant::now() + Duration::from_secs_f32(offset.abs())
                };
                let rt_duration = playback_end - *playback_start;
                let played_duration = Duration::from_secs_f32(rt_duration.as_secs_f32() * self.current_speed());

                Some(*playback_start + played_duration)
            },
            Some(PlaybackState::Paused(inst)) => Some(*inst),
            None => None
        }
    }

    fn playpause(&mut self) {
        match self.playback_state() {
            Some(PlaybackState::Playing(_, _)) => {
                *self.playback_state_mut() = self.playback_end()
                    .map(|inst| PlaybackState::Paused(inst));
            },
            Some(PlaybackState::Paused(mut inst)) => {
                // we're at the end, restart when pressing play again
                if self.reached_end() {
                    inst = self.all_vehicle_states().first().map(|(t, _vs)| *t).unwrap_or(inst);
                }

                let offset = if inst > Instant::now() {
                    (inst - Instant::now()).as_secs_f32()
                } else {
                    -1.0 * (Instant::now() - inst).as_secs_f32()
                };
                *self.playback_state_mut() = Some(PlaybackState::Playing(inst, offset));
            },
            None => {}
        }
    }

    fn skip_to_next_flightmode(&mut self, reverse: bool) {
        let fm = self.vehicle_states().rev().find_map(|(_t, vs)| vs.mode);

        let Some(end) = self.end() else {
            return;
        };

        let i = self.all_vehicle_states().partition_point(|(t, _)| *t <= end);

        let next_flightmode = if reverse {
            self.all_vehicle_states()[..i]
                .iter()
                .rev()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != fm)
                .map(|(t, _)| t)
                .next()
        } else {
            self.all_vehicle_states()[i..]
                .iter()
                .filter(|(_t, vs)| vs.mode.is_some() && vs.mode != fm)
                .map(|(t, _)| t)
                .next()
        };

        if let Some(inst) = next_flightmode {
            let playing = self.playing();
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
                *self.playback_speed_mut() = (self.playback_speed() + PLAYBACK_SPEEDS.len() - 1) % PLAYBACK_SPEEDS.len();
            }
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
            if response.drag_stopped() {
                *self.playback_state_mut() = match self.playback_state() {
                    Some(PlaybackState::Playing(origin, offset)) => {
                        Some(PlaybackState::Playing(origin, offset + (new - current) * total / self.current_speed()))
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
