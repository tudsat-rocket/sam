use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use rtt_target::rprintln;

pub use crate::telemetry::LogLevel;
pub use crate::telemetry::LogLevel::*;

const QUEUE_SIZE: usize = 32;
static LOGGER: Mutex<RefCell<Option<Logger>>> = Mutex::new(RefCell::new(None));

pub struct Logger {
    pub time: u32,
    usb_msg_queue: VecDeque<(u32, String, LogLevel, String)>,
}

impl Logger {
    pub fn new() -> Self {
        let usb_msg_queue = VecDeque::with_capacity(QUEUE_SIZE);
        Self { time: 0, usb_msg_queue }
    }

    pub fn push_usb_message(&mut self, location: &str, log_level: LogLevel, msg: String) {
        if msg.len() > 256 {
            return;
        }

        // If we run out of space, get rid of less important usb_msg_queue first.
        if self.usb_msg_queue.len() == QUEUE_SIZE {
            // TODO: do this nicer
            let mut log_levels: Vec<LogLevel> = self.usb_msg_queue.iter().map(|(_, _, ll, _)| ll).cloned().collect();
            log_levels.sort();
            let lowest_log_level = log_levels[0];

            if lowest_log_level > log_level {
                return;
            }

            let i = log_levels.iter().position(|ll| *ll == lowest_log_level).unwrap();
            self.usb_msg_queue.remove(i);
        }

        self.usb_msg_queue
            .push_back((self.time, location.to_string(), log_level, msg));
    }

    pub fn pop_usb_message(&mut self) -> Option<(u32, String, LogLevel, String)> {
        self.usb_msg_queue.pop_front()
    }

    pub fn init() {
        let logger = Self::new();
        free(|cs| {
            *LOGGER.borrow(cs).borrow_mut() = Some(logger);
        });
    }

    pub fn log(location: &str, log_level: LogLevel, msg: String) {
        let mut segments: Vec<&str> = location.split("::").collect();
        if segments.len() > 1 {
            segments.remove(0);
        }
        let location_trimmed = segments.join("::");

        rprintln!("[{}] [{}] {}", log_level.to_string(), location_trimmed, msg);
        free(|cs| {
            if let Some(ref mut logger) = LOGGER.borrow(cs).borrow_mut().deref_mut() {
                logger.push_usb_message(&location_trimmed, log_level, msg);
            }
        })
    }

    pub fn update_time(time: u32) {
        free(|cs| {
            if let Some(ref mut logger) = LOGGER.borrow(cs).borrow_mut().deref_mut() {
                logger.time = time;
            }
        })
    }

    pub fn next_usb() -> Option<(u32, String, LogLevel, String)> {
        free(|cs| {
            if let Some(ref mut logger) = LOGGER.borrow(cs).borrow_mut().deref_mut() {
                logger.pop_usb_message()
            } else {
                None
            }
        })
    }
}

/// Macro for logging something both to USB and ST-Link
#[macro_export]
macro_rules! log {
    ($ll: expr, $($fmt_args:expr),*) => {{
        crate::logging::Logger::log(module_path!(), $ll, alloc::format!($($fmt_args),*))
    }};
}

/// Macro for logging something occasionally, such as data in the main loop
#[macro_export]
macro_rules! log_every_nth_time {
    ($mod: expr, $ll: expr, $($fmt_args:expr),*) => {{
        static mut LOG_COUNTER: usize = 0;
        if unsafe { LOG_COUNTER } == 0 {
            crate::logging::Logger::log(module_path!(), $ll, alloc::format!($($fmt_args),*))
        }
        unsafe {
            LOG_COUNTER = (LOG_COUNTER + 1) % $mod;
        }
    }};
}
