use embassy_executor;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::*;
use embassy_time::{with_timeout, Duration, Ticker};
use shared_types::{CanBusMessage, CanBusMessageId, FinBoardDataMessage, FlightMode};

const PRIMARY_STRAIN_GAUGE_SAMPLE_RATE_HZ: u64 = 1000;
const SECONDARY_STRAIN_GAUGE_SAMPLE_RATE_HZ: u64 = 100;

const DIVISOR: i16 = 4; // 1 => no loss in quality, clipping after sample > +/- 128
                        // 16 => no clipping inside ADC range

pub async fn sample_strain_gauges(
    adc: &mut Adc<'static, ADC1>,
    sg_vout0_pin: &mut PC1,
    _sg_ref_pin: &mut PC2,
    sg_vout1_pin: &mut PC3,
) -> (u8, u8) {
    const TIMEOUT: Duration = Duration::from_micros(100);

    let sg0_sample = with_timeout(TIMEOUT, adc.read(sg_vout0_pin)).await.unwrap_or_default();
    let sg1_sample = with_timeout(TIMEOUT, adc.read(sg_vout1_pin)).await.unwrap_or_default();

    let sg0 = ((sg0_sample as i16 - 2048) / DIVISOR + 128) as u8;
    let sg1 = ((sg1_sample as i16 - 2048) / DIVISOR + 128) as u8;

    (sg0, sg1)
}

#[embassy_executor::task]
pub async fn run(
    address: u8,
    mut adc: Adc<'static, ADC1>,
    sg_en_pin: PC0,
    mut sg_vout0_pin: PC1,
    mut sg_ref_pin: PC2,
    mut sg_vout1_pin: PC3,
    mut flight_mode_subscriber: crate::can::FlightModeSubscriber,
    can_out: crate::can::CanOutPublisher,
) -> ! {
    let mut enable = Output::new(sg_en_pin, Level::Low, Speed::Low);
    let mut fm = FlightMode::default();

    let sg0_id = CanBusMessageId::FinBoardInput(address, 0); // TODO: fin id
    let sg1_id = CanBusMessageId::FinBoardInput(address, 1); // TODO: fin id

    let mut data0 = [0; 6];
    let mut data1 = [0; 6];
    let mut i = 0;

    let mut ticker = if address == 0 {
        Ticker::every(Duration::from_micros(1_000_000 / PRIMARY_STRAIN_GAUGE_SAMPLE_RATE_HZ))
    } else {
        Ticker::every(Duration::from_micros(1_000_000 / SECONDARY_STRAIN_GAUGE_SAMPLE_RATE_HZ))
    };

    loop {
        while let Some(new_fm) = flight_mode_subscriber.try_next_message_pure() {
            fm = new_fm;
        }

        if fm >= FlightMode::ArmedLaunchImminent && fm < FlightMode::Landed {
            // TODO
            enable.set_high();

            let (sg0, sg1) =
                sample_strain_gauges(&mut adc, &mut sg_vout0_pin, &mut sg_ref_pin, &mut sg_vout1_pin).await;
            data0[i % 6] = sg0;
            data1[i % 6] = sg1;
            i += 1;

            if i >= 6 {
                let sg0_message = FinBoardDataMessage {
                    data: data0.try_into().unwrap_or_default(),
                };
                can_out.publish((sg0_id.into(), sg0_message.serialize_with_crc())).await;
                let sg1_message = FinBoardDataMessage {
                    data: data1.try_into().unwrap_or_default(),
                };
                can_out.publish((sg1_id.into(), sg1_message.serialize_with_crc())).await;
                i = 0;
            }
        } else {
            i = 0;
            enable.set_low();
        }

        ticker.next().await;
    }
}
