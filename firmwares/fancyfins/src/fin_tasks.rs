use embassy_executor;
use embassy_stm32::adc::{Adc, ADC_MAX};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Ticker, Timer};

use flash::*;

#[embassy_executor::task]
async fn flight_state_led() -> ! {
    
    // set led outputs
    
    loop {
        // obtain flight state and update leds
    }
}

#[embassy_executor::task]
async fn strain_gauge() -> ! {

    // 2000 Hz 
    let mut ticker = Ticker::every(Duration::from_millis(0.5));
    loop {
        // TODO read strain gauge value and write to flash
        ticker.next().await;
    }
}