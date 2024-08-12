use embassy_executor;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::low_level::Pin;
use embassy_stm32::peripherals::*;
use embassy_time::{Duration, Ticker};
use shared_types::{CanBusMessage, CanBusMessageId, FlightMode, TelemetryToPayloadMessage};
use w25q::prelude;
use w25q::series25::Flash;


#[embassy_executor::task]
pub async fn flight_state_led(
    mut can_subscriber: crate::can::CanInSubscriper
) -> ! {
    
    let mut current_fm = FlightMode::default();

    //TODO set led outputs
    
    loop {

        let (sid , data) = can_subscriber.next_message_pure().await; 
        if sid == CanBusMessageId::TelemetryBroadcast(0).into() {
            if let Ok(Some(msg)) = TelemetryToPayloadMessage::parse(data) {
                if msg.mode != current_fm {
                    // Fligh mode changed update leds
                    let led_state = msg.mode.led_state(0);
                    
                    // TODO set led state

                    current_fm = msg.mode;
                }
            }
        }
    }
}


#[embassy_executor::task]
pub async fn strain_gauge(
    mut adc: Adc<'static, ADC1>,
    mut sg_en_pin: PC0,
    mut sg_vout0_pin: PC1,
    mut sg_ref_pin: PC2,
    mut sg_vout1_pin: PC3,
    mut can_subscriber: crate::can::CanInSubscriper
) -> ! {

    let mut launch_not_imminent = true;

    while launch_not_imminent {
        let (sid , data) = can_subscriber.next_message_pure().await; 
        if sid == CanBusMessageId::TelemetryBroadcast(0).into() {
            if let Ok(Some(msg)) = TelemetryToPayloadMessage::parse(data) {
                if msg.mode >= FlightMode::ArmedLaunchImminent {
                    launch_not_imminent = false;
                }
            }
        }
    }
    
    // 2000 Hz 
    let mut ticker = Ticker::every(Duration::from_micros(500));

    // let spi = SPI2;
    //let cs = embassy_stm32::peripherals::PC6;

    //let flash = w25q::series25::Flash::init(spi, cs).unwrap();
    loop {
        // TODO read strain gauge value and write to flash
        ticker.next().await;
    }
}