//! Code for opening/uploading files

use std::fs::File;

use shared_types::settings::Settings;

use crate::data_source::*;

// TODO: file support for wasm

#[cfg(target_arch = "x86_64")]
pub fn open_log_file() -> Option<LogFileDataSource> {
    rfd::FileDialog::new().pick_file().and_then(|p| LogFileDataSource::new(p).ok())
}

#[cfg(target_arch = "x86_64")]
pub fn open_fc_settings_file() -> Option<Settings> {
    rfd::FileDialog::new()
        .pick_file()
        .and_then(|p| File::open(p).ok())
        .and_then(|f| serde_json::from_reader(f).ok())
}

#[cfg(target_arch = "x86_64")]
pub fn save_fc_settings_file(settings: &Settings) {
    let path = rfd::FileDialog::new().save_file();
    if let Some(f) = path.and_then(|p| File::create(p).ok()) {
        serde_json::to_writer_pretty(f, settings).unwrap();
    }
}
