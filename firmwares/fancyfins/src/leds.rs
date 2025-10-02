use embassy_futures::select::{Either, select};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::{Config, Phase, Polarity};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use shared_types::can::{CanMessage, TelemetryBroadcast};
use shared_types::{FlightMode, ProcedureStep};
use smart_leds::RGB8;
use smart_leds::SmartLedsWrite;
use ws2812_spi::Ws2812;

use animations::{BLACK, BLUE, GREEN, LILA, RED, WHITE};

use crate::animations::{self, ORANGE, mk_every_nth, mk_fill_percentage};

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

async fn all_animations(time_ms: u64, step: ProcedureStep, buffer: &mut [RGB8], mut update_cb: impl FnMut(&[RGB8])) {
    use ProcedureStep as S;
    let n2_filling = GREEN;
    let n2o_filling = BLUE;
    match step {
        S::Verification => {
            if time_ms % 1000 < 500 {
                mk_fill_percentage(100, GREEN, buffer, None);
                let primary = RED;
                mk_every_nth(8, 0, primary, buffer, None);
                mk_every_nth(8, 1, primary, buffer, None);
                mk_every_nth(8, 2, primary, buffer, None);
                mk_every_nth(8, 3, primary, buffer, None);
            } else {
                mk_fill_percentage(100, RED, buffer, None);
                let primary = GREEN;
                mk_every_nth(8, 0, primary, buffer, None);
                mk_every_nth(8, 1, primary, buffer, None);
                mk_every_nth(8, 2, primary, buffer, None);
                mk_every_nth(8, 3, primary, buffer, None);
            }
        }
        S::IdlePassivated | S::LandedPassivated => {
            mk_fill_percentage(100, GREEN, buffer, None);
        }
        S::N2Filling => {
            mk_fill_percentage(100, BLACK, buffer, None);
            match time_ms % 1000 {
                0..250 => mk_every_nth(8, 0, n2_filling, buffer, Some(BLACK)),
                250..500 => mk_every_nth(8, 1, n2_filling, buffer, Some(BLACK)),
                500..750 => mk_every_nth(8, 1, n2_filling, buffer, Some(BLACK)),
                750..1000 => mk_every_nth(8, 1, n2_filling, buffer, Some(BLACK)),

                _ => mk_every_nth(3, 1, GREEN, buffer, Some(BLACK)),
            }
        }
        S::IdleActive => mk_fill_percentage(100, GREEN, buffer, None),
        S::HardwareArmed => mk_fill_percentage(100, RED, buffer, None),
        S::N2OFilling => {
            mk_fill_percentage(100, BLACK, buffer, None);
            match time_ms % 1000 {
                0..250 => mk_every_nth(8, 0, n2o_filling, buffer, Some(BLACK)),
                250..500 => mk_every_nth(8, 1, n2o_filling, buffer, Some(BLACK)),
                500..750 => mk_every_nth(8, 1, n2o_filling, buffer, Some(BLACK)),
                750..1000 => mk_every_nth(8, 1, n2o_filling, buffer, Some(BLACK)),

                _ => (),
            }
        }
        S::SoftwareArmed => match time_ms % 1000 {
            0..500 => mk_fill_percentage(100, RED, buffer, None),
            500..1000 => mk_fill_percentage(0, RED, buffer, None),
            _ => (),
        },
        S::Ignition
        | S::BurnPhase
        | S::CoastPhase
        | S::DroguePhase
        | S::MainPhase
        | S::LandedActive
        | S::Passivation => mk_fill_percentage(100, LILA, buffer, None),
    }
    update_cb(&buffer);
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
        let _ = leds.write(b.iter().copied());
    })
    .await;

    let mut step: ProcedureStep = ProcedureStep::default();

    loop {
        // while let Some(msg) = can_rx_sub.try_next_message_pure() {
        //     if let CanMessage::Telem(TelemetryBroadcast::FlightMode(new_fm)) = msg {
        //         flight_mode = new_fm;
        //     }
        //     if let CanMessage::Telem(TelemetryBroadcast::ProcedureStep(new_step)) = msg {
        //         new_step = step;
        //     }
        // }
        match select(can_rx_sub.next_message_pure(), Timer::after(Duration::from_millis(10))).await {
            Either::First(msg) => match msg {
                CanMessage::Telem(TelemetryBroadcast::ProcedureStep(new_step)) => {
                    step = new_step;
                }
                _ => (),
            },
            Either::Second(..) => {
                all_animations(Instant::now().as_millis(), step, &mut buffer, |b: &[RGB8]| {
                    let _ = leds.write(b.iter().copied());
                })
                .await;
            }
        }

        // if flight_mode >= FlightMode::Burn {
        //     if let Err(_e) = leds.write(NAVIGATION_LIGHT_PATTERN) {
        //         error!("Failed to write LED pattern");
        //     }
        //
        //     Timer::after(Duration::from_millis(NAVIGATION_LIGHT_BLINK_DURATION_MILLIS)).await;
        //
        //     if let Err(_e) = leds.write([RGB8::default(); 16]) {
        //         error!("Failed to write LED pattern");
        //     }
        //
        //     Timer::after(Duration::from_millis(1000 - NAVIGATION_LIGHT_BLINK_DURATION_MILLIS)).await;
        // } else {
        //     let color = match flight_mode {
        //         FlightMode::Idle => RGB8 {
        //             r: 0x00,
        //             g: 0xff,
        //             b: 0x00,
        //         },
        //         FlightMode::HardwareArmed => RGB8 {
        //             r: 0xff,
        //             g: 0x90,
        //             b: 0x00,
        //         },
        //         FlightMode::Armed => RGB8 {
        //             r: 0xff,
        //             g: 0x00,
        //             b: 0x00,
        //         },
        //         FlightMode::ArmedLaunchImminent => RGB8 {
        //             r: 0xff,
        //             g: 0x00,
        //             b: 0x00,
        //         }, // TODO
        //         _ => RGB8::default(),
        //     };
        //
        //     if let Err(_e) = leds.write([color; 16]) {
        //         error!("Failed to write LED pattern");
        //     }
        //
        //     Timer::after(Duration::from_millis(100)).await;
        // }
    }
}
