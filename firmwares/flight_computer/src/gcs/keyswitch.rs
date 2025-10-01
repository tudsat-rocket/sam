use embassy_stm32::{
    adc::Adc,
    dac::{Dac, DacChannel},
    gpio::{Input, Output},
    mode::Blocking,
    peripherals::{ADC3, DAC1},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn run(read_pin: Input<'static>, is_armed_signal: &'static Signal<CriticalSectionRawMutex, bool>) {
    let mut key_turned = false;
    loop {
        if read_pin.is_high() != key_turned {
            key_turned = read_pin.is_high();
            is_armed_signal.signal(key_turned);
        }
        // NOTE: idiomatically implement with interrupt
        Timer::after(Duration::from_millis(50)).await;
    }
}
