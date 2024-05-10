#![cfg_attr(target_os="none", no_std)] // this is imported by the firmware, so no standard library

#[cfg(target_os = "none")]
extern crate alloc;

pub mod settings;
pub mod telemetry;

pub use settings::*;
pub use telemetry::*;
