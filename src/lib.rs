//! This "library" only exists to provide the Web Assembly entry points.

// Since this "library" doesn't execute all methods used in the application,
// allow dead code here to avoid having to place lots of conditions throughout
// the application to silence warnings.
#[allow(dead_code)]
#[allow(unused_imports)]
mod gui;
mod state;
#[allow(dead_code)]
#[allow(unused_variables)]
mod data_source;
#[allow(unused_imports)]
mod file;
mod telemetry_ext;
#[allow(dead_code)]
#[allow(unused_variables)]
#[allow(unused_imports)]
mod serial;

use crate::gui::*;

#[cfg(target_arch = "wasm32")]
use crate::data_source::*;

// Entry point for wasm
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(start)]
pub async fn start() -> Result<(), JsValue> {
    wasm_logger::init(wasm_logger::Config::default());

    std::panic::set_hook(Box::new(console_error_panic_hook::hook));

    let data_source = Box::new(SerialDataSource::new());

    eframe::start_web(
        "sam_gcs", // needs to match the canvas id in index.html
        eframe::WebOptions::default(),
        Box::new(|_cc| Box::new(Sam::init(data_source))),
    ).await?;
    Ok(())
}
