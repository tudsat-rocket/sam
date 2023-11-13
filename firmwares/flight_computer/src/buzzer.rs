//! Driver for the on-board buzzer, responsible for playing mode change beeps and
//! warning tones using the STM32's timers for PWM generation.
//! TODO: maybe run melodies in a separate embassy task

use embassy_stm32::timer::{simple_pwm::SimplePwm, Channel, CaptureCompare16bitInstance};
use embassy_stm32::time::Hertz;
use embassy_stm32::pac::gpio::Gpio;
use embassy_stm32::pac::gpio::vals;

use num_traits::Float;

use crate::telemetry::FlightMode;
use Semitone::*;

#[allow(dead_code)]
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

#[allow(dead_code)]
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

#[allow(dead_code)]
const REMNANTS: [Note; 40] = [
    Note::note(E, 3, 200), Note::pause(10),
    Note::note(D, 3, 200), Note::pause(10),
    Note::note(A, 4, 400), Note::pause(10),
    Note::note(D, 5, 400), Note::pause(10),
    Note::note(D, 5, 400), Note::pause(10),

    Note::note(A, 4, 400), Note::pause(10),
    Note::note(D, 5, 200), Note::pause(10),
    Note::note(A, 4, 100), Note::pause(10),
    Note::note(D, 5, 100), Note::pause(10),
    Note::note(F, 5, 800), Note::pause(10),

    Note::note(E, 3, 200), Note::pause(10),
    Note::note(D, 3, 200), Note::pause(10),
    Note::note(F, 3, 400), Note::pause(10),
    Note::note(D, 5, 400), Note::pause(10),
    Note::note(D, 5, 400), Note::pause(10),

    Note::note(F, 3, 400), Note::pause(10),
    Note::note(D, 5, 200), Note::pause(10),
    Note::note(D, 5, 100), Note::pause(10),
    Note::note(D, 5, 100), Note::pause(10),
    Note::note(As, 4, 800), Note::pause(10),
];

#[allow(dead_code)]
const THUNDERSTRUCK: [Note; 64] = [
    Note::note(B, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(A, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Gs, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(A, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Gs, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Fs, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Gs, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(E, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),

    Note::note(Fs, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Ds, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(E, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Ds, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(E, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Ds, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(E, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
    Note::note(Ds, 4, 100), Note::pause(10),
    Note::note(B, 3, 100), Note::pause(10),
];

#[allow(dead_code)]
const E1M1: [Note; 56] = [
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(D, 4, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(C, 4, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(As, 3, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(Gs, 3, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(A, 3, 100), Note::pause(10),
    Note::note(As, 3, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(D, 4, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(C, 4, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(As, 3, 100), Note::pause(10),

    Note::note(E, 3, 100), Note::pause(10),
    Note::note(E, 3, 100), Note::pause(10),
    Note::note(Gs, 3, 500), Note::pause(10),
];

pub struct Buzzer<TIM: 'static> {
    pwm: SimplePwm<'static, TIM>,
    channel: Channel,
    block: Gpio,
    pin: usize,
    warning_note: Note,
    current_tone: Option<Note>,
    current_melody: Option<&'static [Note]>,
    current_index: usize,
    time_note_change: u32,
    repeat: bool
}

impl<TIM: CaptureCompare16bitInstance> Buzzer<TIM> {
    pub fn init(mut pwm: SimplePwm<'static, TIM>, channel: Channel, block: Gpio, pin: usize) -> Self {
        pwm.set_duty(Channel::Ch2, pwm.get_max_duty() / 2);

        let buzzer = Self {
            pwm,
            channel,
            block,
            pin,
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
        if let Some(freq) = self.current_frequency() {
            self.block.moder().modify(|w| w.set_moder(self.pin, vals::Moder::ALTERNATE));
            self.block.otyper().modify(|w| w.set_ot(self.pin, vals::Ot::PUSHPULL));
            self.pwm.set_freq(Hertz::hz(freq as u32));
            self.pwm.enable(self.channel);
        } else {
            self.block.moder().modify(|w| w.set_moder(self.pin, vals::Moder::OUTPUT));
            self.block.otyper().modify(|w| w.set_ot(self.pin, vals::Ot::OPENDRAIN));
            self.pwm.disable(self.channel);
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
