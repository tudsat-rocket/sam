use defmt::*;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::{Config, Phase, Polarity};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};
use shared_types::FlightMode;
use shared_types::can::{CanMessage, TelemetryBroadcast};
use smart_leds::RGB8;
use smart_leds::SmartLedsWrite;
use ws2812_spi::Ws2812;

use animations::{BLACK, BLUE, GREEN, RED, WHITE};

use crate::animations::{self, mk_fill_percentage};

const NAVIGATION_LIGHT_PATTERN: [RGB8; 16] = [
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
    RGB8 {
        r: 0xff,
        g: 0x00,
        b: 0x00,
    },
    RGB8 {
        r: 0x00,
        g: 0xff,
        b: 0x00,
    },
];
const NAVIGATION_LIGHT_BLINK_DURATION_MILLIS: u64 = 50;

async fn boot_animation(
    buffer: &mut [RGB8],
    led_lightup_time: Duration,
    color: RGB8,
    mut update_cb: impl FnMut(&[RGB8]),
) {
    for i in 0..buffer.len() {
        mk_fill_percentage(((100 * i) / buffer.len()) as u8, color, buffer, Some(BLACK));
        update_cb(buffer);
        Timer::after(led_lightup_time).await;
    }
    Timer::after(Duration::from_millis(200)).await;
    mk_fill_percentage(100, BLACK, buffer, Some(BLACK));
    update_cb(buffer);
    Timer::after(Duration::from_millis(200)).await;
}

#[embassy_executor::task]
pub async fn run(
    mut can_rx_sub: crate::can::CanRxSub,
    spi: SPI3,
    led_signal_pin: PB5,
    dma_out: DMA1_CH1,
    dma_in: DMA1_CH2,
) -> ! {
    // spi settings
    let mut config = Config::default();
    config.frequency = Hertz::khz(2400);
    config.mode.polarity = Polarity::IdleLow;
    config.mode.phase = Phase::CaptureOnSecondTransition;
    let spi_bus = embassy_stm32::spi::Spi::new_txonly_nosck(spi, led_signal_pin, dma_out, dma_in, config);

    let mut leds = Ws2812::new(spi_bus);
    let mut flight_mode = FlightMode::default();
    let mut buffer: [RGB8; 20] = [BLACK; 20];

    boot_animation(&mut buffer, Duration::from_millis(20), GREEN, |b: &[RGB8]| {
        // TODO: get rid of ocpied
        let _ = leds.write(b.iter().copied());
    })
    .await;

    loop {
        while let Some(msg) = can_rx_sub.try_next_message_pure() {
            if let CanMessage::Telem(TelemetryBroadcast::FlightMode(new_fm)) = msg {
                flight_mode = new_fm;
            }
        }

        if flight_mode >= FlightMode::Burn {
            if let Err(_e) = leds.write(NAVIGATION_LIGHT_PATTERN) {
                error!("Failed to write LED pattern");
            }

            Timer::after(Duration::from_millis(NAVIGATION_LIGHT_BLINK_DURATION_MILLIS)).await;

            if let Err(_e) = leds.write([RGB8::default(); 16]) {
                error!("Failed to write LED pattern");
            }

            Timer::after(Duration::from_millis(1000 - NAVIGATION_LIGHT_BLINK_DURATION_MILLIS)).await;
        } else {
            let color = match flight_mode {
                FlightMode::Idle => RGB8 {
                    r: 0x00,
                    g: 0xff,
                    b: 0x00,
                },
                FlightMode::HardwareArmed => RGB8 {
                    r: 0xff,
                    g: 0x90,
                    b: 0x00,
                },
                FlightMode::Armed => RGB8 {
                    r: 0xff,
                    g: 0x00,
                    b: 0x00,
                },
                FlightMode::ArmedLaunchImminent => RGB8 {
                    r: 0xff,
                    g: 0x00,
                    b: 0x00,
                }, // TODO
                _ => RGB8::default(),
            };

            if let Err(_e) = leds.write([color; 16]) {
                error!("Failed to write LED pattern");
            }

            Timer::after(Duration::from_millis(100)).await;
        }
    }
}
