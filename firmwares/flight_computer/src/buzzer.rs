use alloc::vec::Vec;

use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::rcc::Clocks;

use num_traits::Float;

use Semitone::*;

type Timer = hal::pac::TIM4;
type Pin = hal::gpio::Pin<'B', 9, hal::gpio::Alternate<2>>;
type Pwm = hal::timer::PwmHz<Timer, hal::timer::Ch<3>, Pin>;

const CHANNEL: hal::timer::Channel = hal::timer::Channel::C4;

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
const STAR_TREK: [Note; 20] = [
    Note::note(D, 4, 600), Note::pause(10),
    Note::note(E, 4, 200), Note::pause(10),
    Note::note(F, 4, 400), Note::pause(10),
    Note::note(E, 4, 400), Note::pause(10),
    Note::note(C, 4, 400), Note::pause(10),

    Note::note(D, 4, 600), Note::pause(10),
    Note::note(E, 4, 200), Note::pause(10),
    Note::note(F, 4, 400), Note::pause(10),
    Note::note(E, 4, 400), Note::pause(10),
    Note::note(G, 4, 400), Note::pause(10),
];

#[allow(dead_code)]
const SHIRE: [Note; 14] = [
    Note::note(A, 3, 200), Note::pause(20),
    Note::note(B, 3, 300), Note::pause(20),
    Note::note(Cs, 4, 800), Note::pause(20),
    Note::note(E, 4, 800), Note::pause(20),
    Note::note(Cs, 4, 800), Note::pause(20),
    Note::note(B, 3, 750), Note::pause(70),
    Note::note(A, 3, 1500), Note::pause(20),
];

#[allow(dead_code)]
const E1M1: [Note; 56] = [
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(D, 3, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(C, 3, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(As, 2, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(Gs, 2, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(A, 2, 100), Note::pause(10),
    Note::note(As, 2, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(D, 3, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(C, 3, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(As, 2, 100), Note::pause(10),

    Note::note(E, 2, 100), Note::pause(10),
    Note::note(E, 2, 100), Note::pause(10),
    Note::note(Gs, 2, 500), Note::pause(10),
];

pub struct Buzzer {
    pwm: Pwm,
    current_melody: Vec<Note>,
    current_index: usize,
    time_note_change: u32
}

impl Buzzer {
    pub fn init(timer: Timer, pin: Pin, clocks: &Clocks) -> Self {
        let mut pwm = timer.pwm_hz(pin, 440.Hz(), clocks);
        pwm.set_duty(CHANNEL, pwm.get_max_duty() / 2);

        //let current_melody = E1M1.to_vec();
        let current_melody = Vec::new();

        let buzzer = Self {
            pwm,
            current_melody,
            current_index: 0,
            time_note_change: 0,
        };

        buzzer
    }

    pub fn tick(&mut self, time: u32) {
        if self.current_index >= self.current_melody.len() {
            self.pwm.disable(CHANNEL);
            return;
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
}

#[derive(Clone)]
struct Note {
    pitch: Option<Pitch>,
    duration: u32
}

impl Note {
    const fn note(semitone: Semitone, octave: u8, duration: u32) -> Self {
        Self {
            pitch: Some(Pitch { semitone, octave }),
            duration
        }
    }

    const fn pause(duration: u32) -> Self {
        Self {
            pitch: None,
            duration
        }
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
    B = 11
}
