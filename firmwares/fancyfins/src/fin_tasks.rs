use embassy_executor;
use embassy_stm32::adc::{Adc, ADC_MAX};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Ticker, Timer};
use w25q::prelude;
use w25q::series25::Flash;

#[embassy_executor::task]
pub async fn flight_state_led() -> ! {
    
    // set led outputs
    
    loop {
        // obtain flight state and update leds
    }
}

#[embassy_executor::task]
pub async fn strain_gauge() -> ! {

    // 2000 Hz 
    let mut ticker = Ticker::every(Duration::from_micros(500));
    loop {
        // TODO read strain gauge value and write to flash
        ticker.next().await;
    }
}