use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Output, Input};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use embassy_time::{Ticker, Duration};

use defmt::*;

use shared_types::*;

use crate::buzzer::Buzzer as BuzzerDriver;
use crate::lora::*;
use crate::usb::*;

// TODO
type RadioHandle = Radio<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PA1>>, Input<'static, PC0>,Input<'static, PC1>>;

type LEDs = (Output<'static, PC13>, Output<'static, PC14>, Output<'static, PC15>);
// TODO
type Buzzer = BuzzerDriver<TIM3>;

const MAIN_LOOP_FREQUENCY: Hertz = Hertz::hz(1000);

pub struct GroundControlStation {
    pub time: core::num::Wrapping<u32>,
    usb: UsbHandle,
    radio: RadioHandle,
    leds: LEDs,
    buzzer: Buzzer,
    last_msg_received: core::num::Wrapping<u32>,
}

// TODO: move to main?
#[embassy_executor::task]
pub async fn run(mut gcs: GroundControlStation, mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
    loop {
        gcs.tick().await;
        iwdg.pet();
        ticker.next().await;
    }
}

impl GroundControlStation {
    pub fn init(
        usb: UsbHandle,
        radio: RadioHandle,
        leds: LEDs,
        buzzer: Buzzer,
    ) -> Self {
        Self {
            time: core::num::Wrapping(0),
            usb,
            radio,
            leds,
            buzzer,
            last_msg_received: core::num::Wrapping(0),
        }
    }

    pub async fn tick(&mut self) {
        let downlink_msg = self.radio.tick(self.time.0).await;
        let uplink_msg = self.usb.next_uplink_message().and_then(|msg| {
            match msg {
                UplinkMessage::Heartbeat => None,
                // TODO: remove?
                UplinkMessage::Command(Command::RebootToBootloader) => {
                    None
                },
                UplinkMessage::ApplyLoRaSettings(lora_settings) => {
                    self.radio.apply_settings(&lora_settings);
                    None
                },
                UplinkMessage::ReadSettings => None,
                msg => Some(msg)
            }
        });

        let rssi_led = (self.time - self.last_msg_received).0 < 50;
        self.leds.0.set_level((!(self.radio.transmit_power >= TransmitPower::P20dBm)).into());
        self.leds.1.set_level((!rssi_led).into());
        self.leds.2.set_level((!true).into());
        self.buzzer.tick(self.time.0, None);

        if let Some(msg) = uplink_msg {
            self.radio.queue_uplink_message(msg);
        }

        if let Some(msg) = downlink_msg {
            self.last_msg_received = self.time;
            let gcs_message = DownlinkMessage::TelemetryGCS(TelemetryGCS {
                time: msg.time(),
                lora_rssi: self.radio.trx.rssi,
                lora_rssi_signal: self.radio.trx.rssi_signal,
                lora_snr: self.radio.trx.snr,
            });

            self.usb.send_message(msg);
            self.usb.send_message(gcs_message);
        }

        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;
    }
}
