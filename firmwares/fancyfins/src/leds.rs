use defmt::*;
use embassy_executor;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::{Config, Phase, Polarity, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::spi::SpiBus;
use shared_types::FlightMode;
use shared_types::can::{CanMessage, TelemetryBroadcast};
use smart_leds::RGB8;
use smart_leds::SmartLedsWrite;
use ws2812_spi::Ws2812;

const GREEN: RGB8 = RGB8::new(255, 0, 0);
const RED: RGB8 = RGB8::new(0, 255, 0);
const BLUE: RGB8 = RGB8::new(0, 0, 255);

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

// async fn boot_animation(leds: &mut Ws2812<Spi<'static, SPI3, DMA1_CH1, DMA1_CH2>>) {
async fn boot_animation(leds: &mut Ws2812<Spi<'static, embassy_stm32::mode::Blocking>>) {
    let mut colors = [RGB8::default(); 16];

    for i in 0..16 {
        Timer::after(Duration::from_millis(20)).await;
        colors[i] = RGB8 {
            r: 0x00,
            g: 0xff,
            b: 0x00,
        };
        let _ = leds.write(colors);
    }

    Timer::after(Duration::from_millis(200)).await;
    let _ = leds.write([RGB8::default(); 16]);
    Timer::after(Duration::from_millis(200)).await;
}

#[embassy_executor::task]
pub async fn run(mut can_message_sub: crate::can::CanRxSubscriber, spi_bus: Spi<'static, Blocking>) -> ! {
    info!("led task start");
    let mut flight_mode = FlightMode::default();

    let mut config = Config::default();
    // config.frequency = Hertz::khz(3500);
    // config.frequency = Hertz::khz(3226);
    config.frequency = Hertz::khz(2400);
    // config.frequency = Hertz::khz(7200);
    config.mode.polarity = Polarity::IdleLow;
    config.mode.phase = Phase::CaptureOnSecondTransition;
    // let spi_bus = embassy_stm32::spi::Spi::new_txonly_nosck(spi, led_signal_pin, dma_out, dma_in, config);

    let mut leds = Ws2812::new(spi_bus);

    boot_animation(&mut leds).await;

    // Hyacinth FancyFins use GRB: green, red, blue

    // let mut test_ticker = Ticker::every(Duration::from_millis(500));
    // loop {
    //     leds.write([GREEN; 20]).unwrap();
    //     test_ticker.next().await;
    //     leds.write([BLUE; 20]).unwrap();
    //     test_ticker.next().await;
    // }

    loop {
        while let Some(msg) = can_message_sub.try_next_message_pure() {
            if let CanMessage::Telem(TelemetryBroadcast::FlightMode(mode)) = msg {
                flight_mode = mode;
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
