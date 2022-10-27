//#![no_std]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

mod params;
pub mod telemetry;
pub use params::*;
