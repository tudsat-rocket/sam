#![cfg_attr(target_os = "none", no_std)] // this is imported by the firmware, so no standard library

pub mod common;
pub use common::*;

pub mod can;

#[cfg(feature = "serde")]
pub mod settings;
#[cfg(feature = "serde")]
pub use settings::*;

#[cfg(feature = "serde")]
pub mod telemetry;
#[cfg(feature = "serde")]
pub use telemetry::*;
