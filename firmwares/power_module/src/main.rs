#![no_std]
#![no_main]

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::can::bxcan::{Frame, StandardId};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::pac::SYSCFG;
use embassy_stm32::peripherals::CAN;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, Config};
use embassy_time::{Delay, Duration, Ticker};

use shared_types::can::*;

const MEASUREMENT_FREQUENCY_HZ: u64 = 5;

bind_interrupts!(struct Irqs {
    ADC1 => embassy_stm32::adc::InterruptHandler<embassy_stm32::peripherals::ADC>;
    CEC_CAN => Rx0InterruptHandler<CAN>, Rx1InterruptHandler<CAN>, SceInterruptHandler<CAN>, TxInterruptHandler<CAN>;
});

fn to_millivolts(vref_sample: u16, sample: u16) -> u32 {
    // From https://www.st.com/resource/en/datasheet/stm32f031c6.pdf
    // 6.3.4 Embedded reference voltage
    const VREFINT_MV: u32 = 1230; // mV
    u32::from(sample) * VREFINT_MV / u32::from(vref_sample)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config: Config = Default::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(48));

    let p = embassy_stm32::init(config);

    SYSCFG.cfgr1().modify(|w| w.set_pa11_pa12_rmp(true));

    let mut adc = Adc::new(p.ADC, Irqs, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles239_5);

    let mut vref = adc.enable_vref(&mut Delay);
    let vref_sample = adc.read(&mut vref).await;

    let stat1 = Input::new(p.PA1, Pull::None);
    let stat2 = Input::new(p.PA0, Pull::None);
    let mut vsense_charge = p.PA6;
    let mut vsense_batt = p.PA7;
    let mut isense = p.PB1;

    let mut can = Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    // We don't want to receive anything.
    can.as_mut().modify_filters().clear();

    can.as_mut()
        .modify_config()
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .set_automatic_retransmit(false)
        .leave_disabled();

    can.set_bitrate(125_000);

    can.enable().await;

    let can_message_id = CanBusMessageId::BatteryBoardInput(0);
    let standard_id = StandardId::new(can_message_id.into()).unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(MEASUREMENT_FREQUENCY_HZ));

    loop {
        let voltage_charge =
            (to_millivolts(vref_sample, adc.read(&mut vsense_charge).await) as f32 * (6.1 / 1.0)) as u16;
        let voltage_battery =
            (to_millivolts(vref_sample, adc.read(&mut vsense_batt).await) as f32 * (2.8 / 1.0)) as u16;
        let current_raw = (to_millivolts(vref_sample, adc.read(&mut isense).await) as i16) - 1650;
        let current = ((current_raw as f32) / (50.0 * 0.005)) as i32;

        info!("{}, {}, {}", voltage_charge, voltage_battery, current);

        let msg = BatteryTelemetryMessage {
            voltage_battery,
            voltage_charge,
            current,
            stat0: stat1.is_high(),
            stat1: stat2.is_high(), // TODO
        }
        .serialize_with_crc();

        let tx_frame = Frame::new_data(standard_id, msg);

        can.write(&tx_frame).await;
        can.flush_all().await;

        ticker.next().await;
    }
}
