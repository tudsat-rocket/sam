//! Driver for the on-board buzzer, responsible for playing mode change beeps and
//! warning tones using the STM32's timers for PWM generation.

use hal::pac::*;
use hal::prelude::*;
use hal::timer::pwm::PwmHz;
use stm32f4xx_hal as hal;

use num_traits::Float;

use crate::telemetry::FlightMode;
use Semitone::*;

// TODO: make this more generic, get rid of these feature flags.
// To do this, some of the required traits need to be exposed by stm32f4xx_hal

#[cfg(feature = "rev1")]
type Timer = hal::pac::TIM4;
#[cfg(feature = "rev2")]
type Timer = hal::pac::TIM3;

#[cfg(feature = "rev1")]
type Pins = hal::timer::Channel4<Timer, false>;
#[cfg(feature = "rev2")]
type Pins = hal::timer::Channel2<Timer, false>;

type Pwm = hal::timer::PwmHz<Timer, Pins>;

#[cfg(feature = "rev1")]
const CHANNEL: hal::timer::Channel = hal::timer::Channel::C4;
#[cfg(feature = "rev2")]
const CHANNEL: hal::timer::Channel = hal::timer::Channel::C2;

#[cfg(feature = "rev1")]
const GPIO_REG: *const hal::pac::gpiob::RegisterBlock = GPIOB::ptr();
#[cfg(feature = "rev2")]
const GPIO_REG: *const hal::pac::gpioh::RegisterBlock = GPIOC::ptr();

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

pub struct Buzzer {
    pwm: PwmHz<Timer, Pins>,
    warning_note: Note,
    current_tone: Option<Note>,
    current_melody: Option<&'static [Note]>,
    current_index: usize,
    time_note_change: u32,
    repeat: bool
}

impl Buzzer {
    pub fn init(mut pwm: Pwm) -> Self {
        pwm.set_duty(CHANNEL, pwm.get_max_duty() / 2);

        let buzzer = Self {
            pwm,
            warning_note: Note::note(C, 5, 500),
            current_tone: None,
            current_melody: Some(&STARTUP),
            current_index: 0,
            time_note_change: 0,
            repeat: false
        };

        buzzer
    }

    fn current_frequency(&self) -> Option<f32> {
        let melody_note = self.current_melody
            .map(|m| m.get(self.current_index))
            .flatten();

        self.current_tone.as_ref()
            .or(melody_note)
            .map(|n| n.freq())
            .flatten()
    }

    pub fn set_warning_tone(&mut self, frequency: f32, duration: u32) {
        self.warning_note = Note::frequency(frequency, duration);
    }

    pub fn tick(&mut self, time: u32) {
        if let Some(melody) = self.current_melody {
            let note = melody.get(self.current_index);
            if note.map(|n| time.wrapping_sub(self.time_note_change) > n.duration).unwrap_or(false) {
                self.current_index += 1;
                self.time_note_change = time;

                if self.current_index >= melody.len() && self.repeat {
                    self.current_index = 0;
                }
            } else if time != self.time_note_change {
                return;
            }
        }

        if let Some(note) = &self.current_tone {
            if time.wrapping_sub(self.time_note_change) > note.duration {
                self.current_tone = None;
            } else if time != self.time_note_change {
                return;
            }
        }

        // We set the buzzer output pin into an open-drain state when not using it to
        // reduce leakage current. Since the HAL doesn't provide a straight-forward way
        // to do that, we do it manually.
        let gpio_pin = if cfg!(feature = "rev2") { 7 } else { 9 };
        if let Some(freq) = self.current_frequency() {
            unsafe {
                (*GPIO_REG).moder.modify(|r, w| w.bits(r.bits() & !(0b11 << 2*gpio_pin) | (0b10 << 2*gpio_pin)));
                (*GPIO_REG).otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << gpio_pin)));
            }
            self.pwm.set_period((freq as u32).Hz());
            self.pwm.enable(CHANNEL);
        } else {
            unsafe {
                (*GPIO_REG).moder.modify(|r, w| w.bits(r.bits() & !(0b11 << 2*gpio_pin) | (0b01 << 2*gpio_pin)));
                (*GPIO_REG).otyper.modify(|r, w| w.bits(r.bits() | (0b1 << gpio_pin)));
            }
            self.pwm.disable(CHANNEL);
        }
    }

    pub fn switch_mode(&mut self, time: u32, mode: FlightMode) {
        self.current_tone = if mode == FlightMode::RecoveryDrogue || mode == FlightMode::RecoveryMain {
            Some(self.warning_note.clone())
        } else {
            None
        };

        self.current_melody = match mode {
            FlightMode::HardwareArmed => Some(&HWARMED),
            FlightMode::Armed => Some(&ARMED),
            FlightMode::Landed => Some(&LANDED),
            _ => None
        };

        self.current_index = 0;
        self.time_note_change = time;
        self.repeat = mode == FlightMode::Landed;
    }
}

#[derive(Clone)]
struct Note {
    pitch: Option<Pitch>,
    frequency: Option<f32>,
    duration: u32,
}

impl Note {
    const fn note(semitone: Semitone, octave: u8, duration: u32) -> Self {
        Self {
            pitch: Some(Pitch { semitone, octave }),
            frequency: None,
            duration,
        }
    }

    const fn frequency(frequency: f32, duration: u32) -> Self {
        Self {
            pitch: None,
            frequency: Some(frequency),
            duration,
        }
    }

    const fn pause(duration: u32) -> Self {
        Self { pitch: None, frequency: None, duration }
    }

    fn freq(&self) -> Option<f32> {
        self.frequency.or(self.pitch.as_ref().map(|p| p.freq()))
    }
}

#[derive(Clone)]
struct Pitch {
    semitone: Semitone,
    octave: u8,
}

impl Pitch {
    fn freq(&self) -> f32 {
        let a_i = 3 * 12 + (Semitone::A as i32);
        let note_i = (self.octave as i32) * 12 + (self.semitone as i32);
        440.0 * 2.0_f32.powf((note_i - a_i) as f32 / 12.0)
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
