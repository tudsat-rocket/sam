#![cfg_attr(not(feature = "std"), no_std)]
#![no_main]

#[cfg(not(feature = "std"))]
extern crate alloc;

mod params;
pub mod telemetry;
pub use params::*;

#[cfg(not(feature = "std"))]
use defmt_rtt as _; // global logger (TODO)

#[cfg(not(feature = "std"))]
use panic_probe as _;

/// same panicking *behavior* as `panic-probe` but doesn't print a panic message
/// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
#[cfg(not(feature = "std"))]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
#[cfg(not(feature = "std"))]
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
