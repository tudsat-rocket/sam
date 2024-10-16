//! This "library" only exists to provide the Web Assembly entry points.

// Since this "library" doesn't execute all methods used in the application,
// allow dead code here to avoid having to place lots of conditions throughout
// the application to silence warnings.
#[allow(dead_code)]
#[allow(unused_variables)]
pub mod data_source;
#[allow(unused_imports)]
mod file;
#[allow(dead_code)]
#[allow(unused_imports)]
mod gui;
pub mod settings;
mod telemetry_ext;

pub use crate::gui::*;

#[cfg(target_arch = "wasm32")]
use eframe::WebRunner;

#[cfg(target_arch = "wasm32")]
use settings::AppSettings;

// Entry point for wasm
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
#[derive(Clone)]
#[wasm_bindgen]
pub struct WebHandle {
    runner: WebRunner,
}

#[cfg(target_arch = "wasm32")]
#[wasm_bindgen]
impl WebHandle {
    /// Installs a panic hook, then returns.
    #[allow(clippy::new_without_default)]
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        // Redirect [`log`] message to `console.log` and friends:
        eframe::WebLogger::init(log::LevelFilter::Debug).ok();

        Self {
            runner: WebRunner::new(),
        }
    }

    /// Call this once from JavaScript to start your app.
    #[wasm_bindgen]
    pub async fn start(&self, canvas_id: &str) -> Result<(), wasm_bindgen::JsValue> {
        self.runner
            .start(
                canvas_id,
                eframe::WebOptions::default(),
                Box::new(|cc| Box::new(Sam::init(&cc.egui_ctx, AppSettings::default(), None))),
            )
            .await
    }

    /// The JavaScript can check whether or not your app has crashed:
    #[wasm_bindgen]
    pub fn has_panicked(&self) -> bool {
        self.runner.has_panicked()
    }

    #[wasm_bindgen]
    pub fn panic_message(&self) -> Option<String> {
        self.runner.panic_summary().map(|s| s.message())
    }

    #[wasm_bindgen]
    pub fn panic_callstack(&self) -> Option<String> {
        self.runner.panic_summary().map(|s| s.callstack())
    }
}
