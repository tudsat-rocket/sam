//! A data source based on a logfile, either passed as a file path, or with
//! some raw bytes.

use std::path::PathBuf;

use telemetry::DataStore;

use crate::backend::*;

pub struct LogFileBackend {
    playback: Option<PlaybackState>,
    playback_speed: usize,
    data_store: DataStore,
}

// TODO: right now, we can only load serialized DataStores here. Restore the ability to load raw
// logs of messages

impl LogFileBackend {
    /// Open the given file as a data source.
    pub fn new(path: PathBuf) -> Result<Self, std::io::Error> {
        let data_store = DataStore::from_bytes(&std::fs::read(path)?);

        Ok(Self {
            playback: None,
            playback_speed: 0,
            data_store,
        })
    }

    pub fn from_bytes(bytes: Vec<u8>) -> Self {
        let data_store = DataStore::from_bytes(&bytes);

        Self {
            playback: None,
            playback_speed: 0,
            data_store,
        }
    }
}

impl BackendVariant for LogFileBackend {
    fn update(&mut self, ctx: &egui::Context) {
        self.update_playback(ctx);
    }

    fn data_store<'a>(&'a self) -> &'a DataStore {
        &self.data_store
    }

    fn data_store_mut<'a>(&'a mut self) -> &'a mut DataStore {
        &mut self.data_store
    }

    fn fc_settings(&mut self) -> Option<&Settings> {
        None // TODO: store these in flash?
    }

    fn fc_settings_mut(&mut self) -> Option<&mut Settings> {
        None
    }

    fn reset(&mut self) {
        self.data_store = DataStore::default();
    }

    fn fc_time(&self) -> Option<f64> {
        self.playback_end()
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        self.playback_ui(ui)
    }
}

impl ReplayableBackendVariant for LogFileBackend {
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
