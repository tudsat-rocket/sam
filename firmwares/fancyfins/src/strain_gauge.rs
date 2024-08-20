use embassy_executor;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Output, Level, Speed};
use embassy_stm32::spi::{Config, Phase, Polarity, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::peripherals::*;
use embassy_time::{with_timeout, Delay, Duration, Ticker, Timer};
use smart_leds::RGB8;
use w25q::prelude;
use w25q::series25::Flash;
use ws2812_spi::Ws2812;
use smart_leds::SmartLedsWrite;

fn to_millivolts(vref_sample: u16, sample: u16) -> u16 {
    const VREFINT_MV: u32 = 1200; // mV
    (u32::from(sample) * VREFINT_MV / u32::from(vref_sample)) as u16
}

#[embassy_executor::task]
pub async fn run(
    mut adc: Adc<'static, ADC1>,
    sg_en_pin: PC0,
    mut sg_vout0_pin: PC1,
    _sg_ref_pin: PC2,
    mut sg_vout1_pin: PC3,
    flight_mode_subscriber: crate::can::FlightModeSubscriber,
    spi: SPI2,
    cs: PC6,
    sck: PB13,
    miso: PB14,
    mosi: PB15,
    dma_out: DMA1_CH5,
    dma_in: DMA1_CH4
) -> ! {
    const TIMEOUT: Duration = Duration::from_micros(100);

    let mut enable = Output::new(sg_en_pin, Level::Low, Speed::Low);

    // 2000 Hz
    let mut ticker = Ticker::every(Duration::from_micros(50000));

    let mut config = Config::default();
    config.frequency = Hertz::mhz(2);
    let mut spi_bus = embassy_stm32::spi::Spi::new(spi, sck, mosi, miso, dma_out, dma_in, config);
    let mut cs = Output::new(cs, Level::High, Speed::Low);
    //let mut flash = Flash::init(spi_bus, out).unwrap();

    let mut i = 0;
    loop {
        enable.set_high();
        //let sg_ref_sample = with_timeout(TIMEOUT, adc.read(&mut sg_ref_pin)).await.unwrap_or_default();
        let sg_vout0_sample = with_timeout(TIMEOUT, adc.read(&mut sg_vout0_pin)).await.unwrap_or_default();
        let sg_vout1_sample = with_timeout(TIMEOUT, adc.read(&mut sg_vout1_pin)).await.unwrap_or_default();
        enable.set_low();

        i += 1;
        if i > 100 {
            defmt::println!("[SG] {:?} {:?}", sg_vout0_sample, sg_vout1_sample);
            i = 0;
        }

        //let sg_ref_voltage = to_millivolts(vref_sample, sg_ref_sample);
        //let sg_vout0_voltage = to_millivolts(vref_sample, sg_vout0_sample);
        //let sg_vout1_voltage = to_millivolts(vref_sample, sg_vout1_sample);

        //defmt::println!("{:?} {:?} {:?}", sg_ref_voltage, sg_vout0_voltage, sg_vout1_voltage);

        cs.set_low();
        Timer::after(Duration::from_micros(100)).await;
        let mut read: [u8; 4] = [0x00, 0x00, 0x00, 0x00];
        let mut write: [u8; 4] = [0x9f, 0x00, 0x00, 0x00];
        let res = spi_bus.transfer(&mut read, &write).await.unwrap();
        Timer::after(Duration::from_micros(100)).await;
        cs.set_high();
        defmt::println!("{:?} {:?}", read, write);

        // TODO read strain gauge value and write to flash
        ticker.next().await;
    }
}
