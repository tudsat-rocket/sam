//! Code for opening/uploading files

use crate::data_source::*;

// TODO: file support for wasm

#[cfg(not(target_arch = "wasm32"))]
pub fn open_file() -> Option<LogFileDataSource> {
    rfd::FileDialog::new().pick_file()
        .map(|p| LogFileDataSource::new(p).ok())
        .flatten()
}
