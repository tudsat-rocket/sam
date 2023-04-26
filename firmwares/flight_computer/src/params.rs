//! Some global parameters, some of which should be replaced by in-flash storage.

pub const CLOCK_FREQ_MEGA_HERTZ: u32 = 84;
pub const MAIN_LOOP_FREQ_HERTZ: u32 = 1000;

pub const USB_POLL_FREQ_HERTZ: u32 = 100; // needs to be at least 100Hz
pub const USB_LOG_FREQ_HERTZ: u32 = 100;

pub const MAHONY_KP: f32 = 200.0;
pub const MAHONY_KI: f32 = 50000.0;
