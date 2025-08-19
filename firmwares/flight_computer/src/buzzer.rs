//! Driver for the on-board buzzer, responsible for playing mode change beeps and
//! warning tones using the STM32's timers for PWM generation.

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::pac::gpio::Gpio;
use embassy_stm32::pac::gpio::vals;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::{Channel, simple_pwm::SimplePwm};
use embassy_time::{Duration, Timer};

use num_traits::Float;

use shared_types::*;

//use crate::drivers::sensors::BatteryStatus;
use Semitone::*;

#[allow(dead_code)]
const STARTUP: [Note; 6] = [
    Note::note(C, 4, 150),
    Note::pause(10),
    Note::note(E, 4, 150),
    Note::pause(10),
    Note::note(G, 4, 150),
    Note::pause(10),
];

const HWARMED: [Note; 6] = [
    Note::note(A, 3, 150),
    Note::pause(10),
    Note::note(A, 3, 150),
    Note::pause(10),
    Note::note(A, 3, 150),
    Note::pause(10),
];

const ARMED: [Note; 6] = [
    Note::note(G, 4, 150),
    Note::pause(10),
    Note::note(G, 4, 150),
    Note::pause(10),
    Note::note(G, 4, 150),
    Note::pause(10),
];

#[allow(dead_code)]
const LANDED: [Note; 57] = [
    Note::note(C, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(F, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(As, 4, 450 - 50),
    Note::pause(50),
    Note::note(As, 4, 450 - 50),
    Note::pause(50),
    Note::note(G, 4, 600 - 50),
    Note::pause(50),
    Note::pause(300),
    Note::note(C, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(F, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(G, 4, 450 - 50),
    Note::pause(50),
    Note::note(G, 4, 450 - 50),
    Note::pause(50),
    Note::note(F, 4, 450 - 50),
    Note::pause(50),
    Note::note(E, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 300 - 10),
    Note::pause(10),
    Note::note(C, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(F, 4, 150 - 10),
    Note::pause(10),
    Note::note(D, 4, 150 - 10),
    Note::pause(10),
    Note::note(F, 4, 600 - 50),
    Note::pause(50),
    Note::note(G, 4, 300 - 50),
    Note::pause(50),
    Note::note(E, 4, 450 - 50),
    Note::pause(50),
    Note::note(D, 4, 150 - 50),
    Note::pause(50),
    Note::note(C, 4, 600 - 50),
    Note::pause(50),
    Note::note(C, 4, 300 - 50),
    Note::pause(50),
    Note::note(G, 4, 600 - 50),
    Note::pause(50),
    Note::note(F, 4, 600 - 50),
    Note::pause(50),
];

#[allow(dead_code)]
const REMNANTS: [Note; 40] = [
    Note::note(E, 3, 200),
    Note::pause(10),
    Note::note(D, 3, 200),
    Note::pause(10),
    Note::note(A, 4, 400),
    Note::pause(10),
    Note::note(D, 5, 400),
    Note::pause(10),
    Note::note(D, 5, 400),
    Note::pause(10),
    Note::note(A, 4, 400),
    Note::pause(10),
    Note::note(D, 5, 200),
    Note::pause(10),
    Note::note(A, 4, 100),
    Note::pause(10),
    Note::note(D, 5, 100),
    Note::pause(10),
    Note::note(F, 5, 800),
    Note::pause(10),
    Note::note(E, 3, 200),
    Note::pause(10),
    Note::note(D, 3, 200),
    Note::pause(10),
    Note::note(F, 3, 400),
    Note::pause(10),
    Note::note(D, 5, 400),
    Note::pause(10),
    Note::note(D, 5, 400),
    Note::pause(10),
    Note::note(F, 3, 400),
    Note::pause(10),
    Note::note(D, 5, 200),
    Note::pause(10),
    Note::note(D, 5, 100),
    Note::pause(10),
    Note::note(D, 5, 100),
    Note::pause(10),
    Note::note(As, 4, 800),
    Note::pause(10),
];

#[allow(dead_code)]
const THUNDERSTRUCK: [Note; 64] = [
    Note::note(B, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(A, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Gs, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(A, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Gs, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Fs, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Gs, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(E, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Fs, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Ds, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(E, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Ds, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(E, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Ds, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(E, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
    Note::note(Ds, 4, 100),
    Note::pause(10),
    Note::note(B, 3, 100),
    Note::pause(10),
];

#[allow(dead_code)]
const E1M1: [Note; 56] = [
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(D, 4, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(C, 4, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(As, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(Gs, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(A, 3, 100),
    Note::pause(10),
    Note::note(As, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(D, 4, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(C, 4, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(As, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(E, 3, 100),
    Note::pause(10),
    Note::note(Gs, 3, 500),
    Note::pause(10),
];
#[allow(dead_code)]
const WARNING_MELODY: [Note; 4] = [
    Note::note(C, 5, 500),
    Note::pause(200),
    Note::note(C, 5, 500),
    Note::pause(9000),
];

#[allow(dead_code)]
const SHORT_WARNING_MELODY: [Note; 2] = [Note::note(C, 5, 500), Note::pause(500)];
#[allow(dead_code)]
const NO_BATTERY_ATTACHED_MELODY: [Note; 4] = [
    Note::note(F, 5, 400),
    Note::pause(10),
    Note::note(D, 5, 100),
    Note::pause(10),
];

#[embassy_executor::task]
pub async fn run(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM2>, channel: Channel) -> ! {
    let max_duty = pwm.max_duty_cycle();

    pwm.channel(channel).enable();
    pwm.channel(channel).set_duty_cycle(max_duty / 2);

    let mut flight_mode = FlightMode::default();
    let mut melody = Some(STARTUP.as_slice());

    loop {
        // let flight_mode_fut = crate::FLIGHT_MODE_SIGNAL.wait();

        let pwm_ref = &mut pwm;
        let play_fut = async move {
            if let Some(notes) = melody {
                for note in notes {
                    if let Some(freq) = note.freq() {
                        pwm_ref.set_frequency(Hertz::hz(freq as u32));
                        pwm_ref.channel(channel).enable();
                    } else {
                        pwm_ref.channel(channel).disable();
                    }

                    Timer::after(Duration::from_millis(note.duration.into())).await;
                }
            } else {
                // TODO
                Timer::after(Duration::from_millis(1000)).await
            }
        };

        match select(crate::FLIGHT_MODE_SIGNAL.wait(), play_fut).await {
            Either::First(fm) => {
                flight_mode = fm;
                melody = match flight_mode {
                    FlightMode::RecoveryDrogue | FlightMode::RecoveryMain => Some(&SHORT_WARNING_MELODY),
                    FlightMode::HardwareArmed => Some(&HWARMED),
                    FlightMode::Armed | FlightMode::ArmedLaunchImminent => Some(&ARMED),
                    FlightMode::Landed => Some(&LANDED),
                    _ => None,
                };
            }
            Either::Second(()) => {
                if flight_mode != FlightMode::Landed {
                    melody = None;
                }
            }
        }

        pwm.channel(channel).disable();
    }
}

#[embassy_executor::task]
pub async fn run_b(mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM2>, channel: Channel) -> ! {
    let max_duty = pwm.max_duty_cycle();

    pwm.channel(channel).enable();
    pwm.channel(channel).set_duty_cycle(max_duty / 2);

    let mut flight_mode = FlightMode::default();
    let mut melody = Some(STARTUP.as_slice());

    loop {
        // let flight_mode_fut = crate::FLIGHT_MODE_SIGNAL.wait();

        let pwm_ref = &mut pwm;
        let play_fut = async move {
            if let Some(notes) = melody {
                for note in notes {
                    if let Some(freq) = note.freq() {
                        pwm_ref.set_frequency(Hertz::hz(freq as u32));
                        pwm_ref.channel(channel).enable();
                    } else {
                        pwm_ref.channel(channel).disable();
                    }

                    Timer::after(Duration::from_millis(note.duration.into())).await;
                }
            } else {
                // TODO
                Timer::after(Duration::from_millis(1000)).await
            }
        };

        // let play_fut = async {
        //     future::pending::<()>()
        // }

        match select(crate::FLIGHT_MODE_SIGNAL.wait(), play_fut).await {
            Either::First(fm) => {
                flight_mode = fm;
                melody = match flight_mode {
                    FlightMode::RecoveryDrogue | FlightMode::RecoveryMain => Some(&SHORT_WARNING_MELODY),
                    FlightMode::HardwareArmed => Some(&HWARMED),
                    FlightMode::Armed | FlightMode::ArmedLaunchImminent => Some(&ARMED),
                    FlightMode::Landed => Some(&LANDED),
                    _ => None,
                };
            }
            Either::Second(()) => {
                if flight_mode != FlightMode::Landed {
                    melody = None;
                }
            }
        }

        pwm.channel(channel).disable();
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
        Self {
            pitch: None,
            frequency: None,
            duration,
        }
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
