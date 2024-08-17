use embassy_executor;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Output, Level, Speed};
use embassy_stm32::spi::{Spi, Config};
use embassy_stm32::Peripherals;
use embassy_stm32::peripherals::*;
use embassy_time::{Duration, Ticker};
use shared_types::{CanBusMessage, CanBusMessageId, FlightMode, TelemetryToPayloadMessage};
use w25q::prelude;
use w25q::series25::Flash;
use ws2812_spi::Ws2812;


#[embassy_executor::task] 
pub async fn flight_state_led(
    mut can_subscriber: crate::can::CanInSubscriper,
    spi: SPI3,
    led_signal_pin: PB5,
    dma_out: DMA1_CH1,
    dma_in: DMA1_CH2,
) -> ! {
    
    let mut current_fm = FlightMode::default();

    let config = Config::default();
    let spi_bus = embassy_stm32::spi::Spi::new_txonly_nosck(spi, led_signal_pin, dma_out, dma_in, config);

    let led_driver = Ws2812::new(spi_bus);

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
    mut can_subscriber: crate::can::CanInSubscriper,
    spi: SPI2,
    cs: PC6,
    sck: PB13,
    miso: PB14,
    mosi: PB15,
    dma_out: DMA2_CH1,
    dma_in: DMA2_CH2
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

    let config = Config::default();
    let spi_bus = embassy_stm32::spi::Spi::new(spi, sck, mosi, miso, dma_out, dma_in,  config);
    
    let out = Output::new(cs, Level::Low, Speed::Low);
    let flash = Flash::init(spi_bus, out).unwrap();
    
    loop {
        // TODO read strain gauge value and write to flash
        ticker.next().await;
    }
}