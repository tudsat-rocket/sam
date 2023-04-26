//! Driver for the on-board buzzer, responsible for playing mode change beeps and
//! warning tones using the STM32's timers for PWM generation.

use alloc::vec::Vec;

use hal::prelude::*;
use hal::rcc::Clocks;
use stm32f4xx_hal as hal;

use num_traits::Float;

use crate::telemetry::FlightMode;
use Semitone::*;

// TODO: make this more generic, get rid of these feature flags.

#[cfg(feature = "rev1")]
type Timer = hal::pac::TIM4;
#[cfg(feature = "rev2")]
type Timer = hal::pac::TIM3;

#[cfg(feature = "rev1")]
type Pin = hal::gpio::Pin<'B', 9, hal::gpio::Alternate<2>>;
#[cfg(feature = "rev2")]
type Pin = hal::gpio::Pin<'C', 7, hal::gpio::Alternate<2>>;

#[cfg(feature = "rev1")]
type Pwm = hal::timer::PwmHz<Timer, hal::timer::Ch<3>, Pin>;
#[cfg(feature = "rev2")]
type Pwm = hal::timer::PwmHz<Timer, hal::timer::Ch<1>, Pin>;

#[cfg(feature = "rev1")]
const CHANNEL: hal::timer::Channel = hal::timer::Channel::C4;
#[cfg(feature = "rev2")]
const CHANNEL: hal::timer::Channel = hal::timer::Channel::C2;

const STARTUP: [Note; 6] = [
    Note::note(C, 4, 150), Note::pause(10),
    Note::note(E, 4, 150), Note::pause(10),
    Note::note(G, 4, 150), Note::pause(10),
];

const HWARMED: [Note; 6] = [
    Note::note(A, 3, 150), Note::pause(10),
    Note::note(A, 3, 150), Note::pause(10),
    Note::note(A, 3, 150), Note::pause(10),
];

const ARMED: [Note; 6] = [
    Note::note(G, 4, 150), Note::pause(10),
    Note::note(G, 4, 150), Note::pause(10),
    Note::note(G, 4, 150), Note::pause(10),
];

const LANDED: [Note; 57] = [
    Note::note(C, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),
    Note::note(F, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),

    Note::note(As, 4, 450 - 50), Note::pause(50),
    Note::note(As, 4, 450 - 50), Note::pause(50),
    Note::note(G, 4, 600 - 50), Note::pause(50),
    Note::pause(300),
    Note::note(C, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),
    Note::note(F, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),

    Note::note(G, 4, 450 - 50), Note::pause(50),
    Note::note(G, 4, 450 - 50), Note::pause(50),
    Note::note(F, 4, 450 - 50), Note::pause(50),
    Note::note(E, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 300 - 10), Note::pause(10),
    Note::note(C, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),
    Note::note(F, 4, 150 - 10), Note::pause(10),
    Note::note(D, 4, 150 - 10), Note::pause(10),

    Note::note(F, 4, 600 - 50), Note::pause(50),
    Note::note(G, 4, 300 - 50), Note::pause(50),
    Note::note(E, 4, 450 - 50), Note::pause(50),
    Note::note(D, 4, 150 - 50), Note::pause(50),
    Note::note(C, 4, 600 - 50), Note::pause(50),
    Note::note(C, 4, 300 - 50), Note::pause(50),

    Note::note(G, 4, 600 - 50), Note::pause(50),
    Note::note(F, 4, 600 - 50), Note::pause(50),
];

pub const RECOVERY_WARNING_TIME: u32 = 750;
const RECOVERY: [Note; 1] = [Note::note(C, 5, RECOVERY_WARNING_TIME)];

pub struct Buzzer {
    pwm: Pwm,
    current_melody: Vec<Note>,
    current_index: usize,
    time_note_change: u32,
    repeat: bool
}

impl Buzzer {
    pub fn init(timer: Timer, pin: Pin, clocks: &Clocks) -> Self {
        let mut pwm = timer.pwm_hz(pin, 440.Hz(), clocks);
        pwm.set_duty(CHANNEL, pwm.get_max_duty() / 2);

        let current_melody = STARTUP.to_vec();

        let buzzer = Self {
            pwm,
            current_melody,
            current_index: 0,
            time_note_change: 0,
            repeat: false
        };

        buzzer
    }

    pub fn tick(&mut self, time: u32) {
        if self.current_index >= self.current_melody.len() {
            if self.repeat {
                self.current_index = 0;
            } else {
                self.pwm.disable(CHANNEL);
                return;
            }
        }

        let current_note = &self.current_melody[self.current_index];
        if let Some(freq) = current_note.freq() {
            self.pwm.set_period((freq as u32).Hz());
            self.pwm.enable(CHANNEL);
        } else {
            self.pwm.disable(CHANNEL);
        }

        if time - self.time_note_change > current_note.duration {
            self.current_index += 1;
            self.time_note_change = time;
        }
    }

    pub fn switch_mode(&mut self, time: u32, mode: FlightMode) {
        self.current_melody = match mode {
            FlightMode::HardwareArmed => HWARMED.to_vec(),
            FlightMode::Armed => ARMED.to_vec(),
            FlightMode::RecoveryDrogue | FlightMode::RecoveryMain => RECOVERY.to_vec(),
            FlightMode::Landed => LANDED.to_vec(),
            _ => Vec::new()
        };
        self.current_index = 0;
        self.time_note_change = time;
        self.repeat = mode == FlightMode::Landed;
    }
}

#[derive(Clone)]
struct Note {
    pitch: Option<Pitch>,
    duration: u32,
}

impl Note {
    const fn note(semitone: Semitone, octave: u8, duration: u32) -> Self {
        Self {
            pitch: Some(Pitch { semitone, octave }),
            duration,
        }
    }

    const fn pause(duration: u32) -> Self {
        Self { pitch: None, duration }
    }

    fn freq(&self) -> Option<f32> {
        self.pitch.as_ref().map(|p| p.freq())
    }
}

#[derive(Clone)]
struct Pitch {
    semitone: Semitone,
    octave: u8,
}

impl Pitch {
    fn freq(&self) -> f32 {
        let middle_c_i = 3 * 12 + Semitone::C as u8;
        let i = self.octave * 12 + self.semitone as u8;
        let n = (i as f32) - (middle_c_i as f32);
        440.0 * 2.0_f32.powf((n - 9.0) / 12.0)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum Semitone {
    C = 0,
    Cs = 1,
    D = 2,
    Ds = 3,
    E = 4,
    F = 5,
    Fs = 6,
    G = 7,
    Gs = 8,
    A = 9,
    As = 10,
    B = 11,
}
