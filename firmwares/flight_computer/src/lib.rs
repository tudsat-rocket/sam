//! Export of selected modules for use in external programs, such as ground station.

#![cfg_attr(not(feature = "std"), no_std)]
#![no_main]

#[cfg(not(feature = "std"))]
extern crate alloc;

pub mod telemetry;
pub mod settings;
pub mod state_estimation;

#[cfg(feature = "no_std")]
use defmt_rtt as _; // global logger (TODO)

#[cfg(feature = "no_std")]
use panic_probe as _;

/// same panicking *behavior* as `panic-probe` but doesn't print a panic message
/// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
#[cfg(feature = "no_std")]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
#[cfg(feature = "no_std")]
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
