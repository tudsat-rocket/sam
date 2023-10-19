#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use crc::{Crc, CRC_16_IBM_SDLC};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::{bind_interrupts, Config};
use embassy_stm32::can::bxcan::filter::Mask32;
use embassy_stm32::can::bxcan::{Fifo, Frame, StandardId};
use embassy_stm32::can::{Can, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::pac::SYSCFG;
use embassy_stm32::peripherals::CAN;
use embassy_stm32::pac::syscfg::vals::Pa11Pa12Rmp;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Timer, Ticker};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    CEC_CAN => Rx0InterruptHandler<CAN>, Rx1InterruptHandler<CAN>, SceInterruptHandler<CAN>, TxInterruptHandler<CAN>;
});

const CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config: Config = Default::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(48));

    let p = embassy_stm32::init(config);

    SYSCFG.cfgr1().modify(|w| w.set_pa11_pa12_rmp(Pa11Pa12Rmp::REMAPPED));

    let mut adc = Adc::new(p.ADC, &mut Delay);
    adc.set_sample_time(SampleTime::Cycles239_5);

    let mut vrefint = adc.enable_vref(&mut Delay);
    let vrefint_sample = adc.read_internal(&mut vrefint);
    let convert_to_millivolts = |sample: u16| {
        // From https://www.st.com/resource/en/datasheet/stm32f031c6.pdf
        // 6.3.4 Embedded reference voltage
        const VREFINT_MV: u32 = 1230; // mV

        (u32::from(sample) * VREFINT_MV / u32::from(vrefint_sample)) as u16
    };

    let stat1 = Input::new(p.PA1, Pull::None);
    let stat2 = Input::new(p.PA0, Pull::None);
    //let ntc = p.PA2;
    //let charge_enable = p.PA3;
    //let balancer_out = p.PA4;
    let mut breakwire = p.PA5;
    let mut vsense_charge = p.PA6;
    let mut vsense_batt = p.PA7;
    let mut isense = p.PB1;

    let mut can = Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    // We don't want to receive anything.
    can.as_mut()
        .modify_filters()
        .clear();

    can.as_mut()
        .modify_config()
        .set_loopback(false) // Receive own frames
        .set_silent(false)
        .set_automatic_retransmit(false)
        .leave_disabled();

    can.set_bitrate(125_000);

    can.enable().await;

    const INTERVAL: u64 = 10;
    let mut ticker = Ticker::every(Duration::from_millis(INTERVAL));
    loop {
        let voltage_charge = (convert_to_millivolts(adc.read(&mut vsense_charge)) as f32 * (6.1 / 1.0)) as u16;
        let voltage_battery = (convert_to_millivolts(adc.read(&mut vsense_batt)) as f32 * (2.8 / 1.0)) as u16;
        let current_raw = (convert_to_millivolts(adc.read(&mut isense)) as i16) - 1650;
        let current = ((((current_raw as f32) / (50.0 * 0.005)) as i16) + 2000) as u16;

        let voltage_breakwire = convert_to_millivolts(adc.read(&mut breakwire)) as i16;
        let breakwire_open = match voltage_breakwire {
            3150..=3500 => Some(true), // no connection, we're going
            1500..=1800 => Some(false), // voltage divider with breakwire, we're on the rail
            _ => None, // something weird has happened (shorted wires, etc.), we have no idea
        };

        let breakwire_serialized = match breakwire_open {
            None => 0b10,
            Some(x) => x as u8
        };

        info!("{}, {:?}, {}, {}, {}", voltage_breakwire, breakwire_open, voltage_charge, voltage_battery, current);

        let mut msg = [
            voltage_charge.to_le_bytes()[0],
            (voltage_charge.to_le_bytes()[1] << 1) | (stat1.is_high() as u8),
            current.to_le_bytes()[0],
            (current.to_le_bytes()[1] << 1) | (stat1.is_high() as u8),
            voltage_battery.to_le_bytes()[0],
            (voltage_battery.to_le_bytes()[1] << 2) | breakwire_serialized,
            0,
            0
        ];

        let crc = CRC.checksum(&msg);
        msg[6..].copy_from_slice(&crc.to_le_bytes());

        let tx_frame = Frame::new_data(unwrap!(StandardId::new(0x100)), msg);
        can.write(&tx_frame).await;
        can.flush_all().await;

        ticker.next().await;
    }
}
