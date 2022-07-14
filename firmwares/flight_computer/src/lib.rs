//#![no_std]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod telemetry;
mod params;
pub use params::*;
