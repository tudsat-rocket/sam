use embassy_executor;

use defmt::*;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Ticker, Timer};
use shared_types::{CanBusMessage, IoBoardOutputMessage, Transmit};

#[embassy_executor::task]
pub async fn run(can_tx_pub: crate::can::CanOutPublisher, can_rx_sub: crate::can::CanInSubscriber) {
    let const_hartbeat_msg = IoBoardOutputMessage {
        outputs: [false, false, false, false, true, true, true, true],
    };
    let serialized = const_hartbeat_msg.serialize_with_crc();
    loop {
        info!("Published Can heartbeat message");
        can_tx_pub.publish((0x1af, serialized)).await;
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
pub async fn run_leds(
    // leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
    leds: (Output<'static>, Output<'static>, Output<'static>),
) -> ! {
    // info!("run heartbeat leds start");
    let (mut led_red, mut led_yellow, mut led_white) = leds;
    led_red.set_high();
    led_yellow.set_high();
    led_white.set_high();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        // info!("heartbeat led on");
        led_white.set_low();
        ticker.next().await;
        led_white.set_high();
        ticker.next().await;
    }
}
