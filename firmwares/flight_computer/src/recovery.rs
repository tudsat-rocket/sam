use crate::board::load_outputs::{LoadOutput, LoadOutputs};
use core::cell::RefCell;
use embassy_executor::SendSpawner;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use shared_types::{FlightMode, settings::RecoveryOutputSettings};

pub static MAIN_RECOVERY_SIG: Signal<CriticalSectionRawMutex, bool> = Signal::new();
pub static PARABREAKS_SIG: Signal<CriticalSectionRawMutex, bool> = Signal::new();

pub async fn start_recovery_task(
    main_output_settings: RecoveryOutputSettings,
    load_outputs: &'static Mutex<CriticalSectionRawMutex, LoadOutputs>,
    spawner: SendSpawner,
    main_recovery_sig: &'static Signal<CriticalSectionRawMutex, bool>,
    parabreaks_sig: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    spawner.spawn(run_main_recovery(main_output_settings, load_outputs, main_recovery_sig)).unwrap();
    spawner.spawn(run_parabreaks(load_outputs, parabreaks_sig)).unwrap();
}
// TODO: add settings

// TODO: check timings, add timings in settings
#[embassy_executor::task]
async fn run_main_recovery(
    settings: RecoveryOutputSettings,
    load_outputs: &'static Mutex<CriticalSectionRawMutex, LoadOutputs>,
    main_recovery_sig: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    loop {
        let sig = main_recovery_sig.wait().await;
        if !sig {
            continue;
        }

        Timer::after(Duration::from_millis(settings.forewarning_duration.into())).await;

        // Check armed status
        for _ in 0..settings.num_pulses {
            unsafe {
                load_outputs.lock_mut(|o| o.activate(LoadOutput::Out1));
            }
            defmt::info!("Main recovery motor active.");

            Timer::after(Duration::from_millis(settings.pulse_high_duration.into())).await;
            unsafe {
                load_outputs.lock_mut(|o| o.deactivate(LoadOutput::Out1));
            }
            defmt::info!("Main recovery motor stop.");

            Timer::after(Duration::from_millis(settings.pause_duration.into())).await;
        }
    }
}

#[embassy_executor::task]
async fn run_parabreaks(
    load_outputs: &'static Mutex<CriticalSectionRawMutex, LoadOutputs>,
    parabreaks_sig: &'static Signal<CriticalSectionRawMutex, bool>,
) {
    loop {
        let sig = parabreaks_sig.wait().await;
        if !sig {
            continue;
        }
        // TODO: probably remove arm
        unsafe {
            load_outputs.lock_mut(|o| o.arm());
        }
        unsafe {
            load_outputs.lock_mut(|o| o.activate(LoadOutput::Out3));
        }
        defmt::info!("Parabreaks motor active.");

        Timer::after(Duration::from_secs(12)).await;

        unsafe {
            load_outputs.lock_mut(|o| o.deactivate(LoadOutput::Out3));
        }
        defmt::info!("Parabreaks motor stop.");
    }
}
