use core::hash::Hasher;

use alloc::string::String;
use alloc::vec::Vec;

use embedded_hal::digital::InputPin;
use embedded_hal_async::spi::SpiDevice;

use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use serde::de::DeserializeOwned;
use siphasher::sip::SipHasher;

use defmt::*;

use shared_types::*;

use crate::drivers::lora::*;

// The available channels for telemetry, assuming a 500kHz band width.
const CHANNELS: [u32; 14] = [
    863_250_000,
    863_750_000,
    864_250_000,
    864_750_000,
    865_250_000,
    865_750_000,
    866_250_000,
    866_750_000,
    867_250_000,
    867_750_000,
    868_250_000,
    868_750_000,
    869_250_000,
    869_750_000,
];

#[cfg(feature = "gcs")]
type TxHmac = u64;
#[cfg(not(feature = "gcs"))]
type TxHmac = u16;

#[cfg(feature = "gcs")]
type RxHmac = u16;
#[cfg(not(feature = "gcs"))]
type RxHmac = u64;

#[derive(Debug, PartialEq, Eq)]
enum RadioState {
    Idle,
    Transmitting,
}

#[derive(Debug)]
pub enum RadioError<E> {
    Spi(E),
    Crc,
    Busy,
    Timeout,
}

impl<E> From<E> for RadioError<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

pub struct Radio<SPI, IRQ, BUSY> {
    pub trx: LLCC68<SPI, IRQ, BUSY>,
    time: u32,
    state: RadioState,
    state_time: u32,
    pub transmit_power: TransmitPower,
    transmit_power_setpoint: TransmitPower,
    #[cfg(feature="gcs")]
    uplink_message: Option<UplinkMessage>,
    last_message_received: u32,
    #[cfg(feature="gcs")]
    fc_time_offset: i64,
    authentication_key: [u8; 16],
    channels: [bool; CHANNELS.len()],
    binding_phrase: String,
    sequence: Option<[usize; CHANNELS.len()]>,
}

impl<SPI: SpiDevice<u8>, IRQ: InputPin, BUSY: InputPin> Radio<SPI, IRQ, BUSY> {
    pub async fn init(spi: SPI, irq: IRQ, busy: BUSY) -> Result<Self, RadioError<SPI::Error>> {
        let llcc68 = LLCC68::init(spi, irq, busy, CHANNELS[CHANNELS.len() / 2]).await?;

        Ok(Self {
            trx: llcc68,
            time: 0,
            state: RadioState::Idle,
            state_time: 0,
            transmit_power: TransmitPower::P14dBm,
            transmit_power_setpoint: TransmitPower::P14dBm,
            #[cfg(feature="gcs")]
            uplink_message: None,
            last_message_received: 0,
            #[cfg(feature="gcs")]
            fc_time_offset: 0,
            authentication_key: [0x00; 16],
            channels: [true; CHANNELS.len()],
            binding_phrase: "".into(),
            sequence: None,
        })
    }

    pub fn set_transmit_power(&mut self, tx_power: TransmitPower) {
        self.transmit_power_setpoint = tx_power;
    }

    pub fn set_max_transmit_power(&mut self) {
        self.transmit_power_setpoint = TransmitPower::P22dBm;
    }

    fn set_state(&mut self, state: RadioState) {
        self.state = state;
        self.state_time = self.time;
    }

    fn generate_sequence(&mut self, channels: [bool; CHANNELS.len()], binding_phrase: &String) -> Option<[usize; CHANNELS.len()]> {
        let mut available: Vec<_> = channels.iter()
            .enumerate()
            .filter_map(|(i, x)| x.then(|| i))
            .collect();

        if available.len() == 0 {
            return None;
        }

        // Generate a u64 from binding phrase to seed rng
        let mut siphasher = SipHasher::new_with_key(&[0x00; 16]);
        siphasher.write(binding_phrase.as_bytes());
        let seed = siphasher.finish();
        let mut rng = ChaCha20Rng::seed_from_u64(seed);

        // Fill sequence from available channels. If we disable some channels,
        // we repeat the process until we have enough.
        let mut sequence = Vec::new();
        while sequence.len() < CHANNELS.len() {
            available.shuffle(&mut rng);
            sequence.extend(available.clone());
        }

        Some(sequence[..CHANNELS.len()].try_into().unwrap())
    }

    pub fn apply_settings(&mut self, settings: &LoRaSettings) {
        self.authentication_key = settings.authentication_key.to_be_bytes();
        if settings.channels == self.channels && settings.binding_phrase == self.binding_phrase {
            return;
        }

        self.channels = settings.channels;
        self.binding_phrase = settings.binding_phrase.clone();
        self.sequence = self.generate_sequence(settings.channels, &settings.binding_phrase);
        //info!("Generated sequence {:?} using phrase {:?}", self.sequence, Debug2Format(&self.binding_phrase));
    }

    async fn switch_to_next_frequency(&mut self) -> Result<(), RadioError<SPI::Error>> {
        // Switch to the correct frequency for the current message interval.
        // On the FC, this is pretty straight forward.

        #[cfg(not(feature="gcs"))]
        let t = self.time;
        #[cfg(feature="gcs")]
        let t = (self.time as i64).wrapping_add(self.fc_time_offset) as u32;

        let message_i = (t / LORA_MESSAGE_INTERVAL) as usize % CHANNELS.len();
        self.trx.set_frequency(CHANNELS[self.sequence.map(|s| s[message_i]).unwrap_or(0)]).await
    }

    fn start_of_current_interval(&self) -> u32 {
        // Returns the start of the current message interval. The result is
        // always in FC time, and is used for message authentication.
        #[cfg(not(feature="gcs"))]
        let t = self.time;
        #[cfg(feature="gcs")]
        let t = (self.time as i64).wrapping_add(self.fc_time_offset) as u32;

        t.wrapping_sub(t % LORA_MESSAGE_INTERVAL)
    }

    pub async fn send<M: Transmit>(&mut self, msg: M) -> Result<(), RadioError<SPI::Error>> {
        let serialized = msg.serialize().unwrap_or_default();

        if self.sequence.is_none() {
            return Ok(());
        }

        if self.state != RadioState::Idle {
            error!("skipping");
            return Ok(()); // TODO
        }

        // Prepend message authentication
        let mut siphasher = SipHasher::new_with_key(&self.authentication_key);
        #[cfg(feature="gcs")] // only include time for uplink messages, prevents replay attacks
        siphasher.write(&self.start_of_current_interval().to_be_bytes());
        siphasher.write(&serialized);
        let hash = (siphasher.finish() as TxHmac).to_be_bytes();
        let msg = [&hash, serialized.as_slice()].concat();

        self.trx.send(&msg).await?;
        self.set_state(RadioState::Transmitting);
        Ok(())
    }

    #[cfg(feature="gcs")]
    pub fn queue_uplink_message(&mut self, msg: UplinkMessage) {
        self.uplink_message = Some(msg);
    }

    async fn receive<M: Transmit + DeserializeOwned>(&mut self) -> Result<Option<M>, RadioError<SPI::Error>> {
        let mut buffer = match self.trx.receive().await? {
            Some(buffer) => buffer,
            None => return Ok(None),
        };

        let (hmac, serialized) = buffer[1..].split_at_mut(core::mem::size_of::<RxHmac>());
        let serialized_end = serialized.iter()
            .position(|b| *b == 0)
            .map(|i| i + 1)
            .unwrap_or(serialized.len());

        let mut siphasher = SipHasher::new_with_key(&self.authentication_key);
        #[cfg(not(feature="gcs"))] // only include time for uplink messages, prevents replay attacks
        siphasher.write(&self.start_of_current_interval().to_be_bytes());
        siphasher.write(&serialized[..serialized_end]);
        let correct = (siphasher.finish() as RxHmac).to_be_bytes();

        if correct != hmac {
            warn!("HMAC mismatch.");
            return Ok(None);
        }

        let deserialized = postcard::from_bytes_cobs(serialized).ok();
        if deserialized.is_none() {
            error!("Failed to decode message: {}", &buffer[1..]);
        }

        Ok(deserialized)
    }

    fn is_uplink_window(&self, time: u32, first_only: bool) -> bool {
        let mut t = time % 1000;

        if !first_only {
            t -= t % LORA_MESSAGE_INTERVAL;
        }
        (t % LORA_UPLINK_INTERVAL) == LORA_UPLINK_MODULO
    }

    async fn tick_common(&mut self, time: u32) {
        self.time = time;

        // Return to rx mode after transmission. A delay is necessary in order
        // to allow the LLCC68 to actually finish the transmission
        if self.state == RadioState::Transmitting && time >= self.state_time.wrapping_add(TRANSMISSION_TIMEOUT_MS + 2) {
            if let Err(e) = self.trx.switch_to_rx().await {
                error!("Failed to return to RX mode: {:?}", Debug2Format(&e));
            } else {
                self.set_state(RadioState::Idle);
            }
        }

        if self.transmit_power != self.transmit_power_setpoint {
            if let Err(e) = self.trx.set_output_power(self.transmit_power_setpoint).await {
                error!("Error setting power level: {:?}", Debug2Format(&e));
            } else {
                self.transmit_power = self.transmit_power_setpoint;
            }
        }
    }

    #[cfg(not(feature = "gcs"))]
    pub async fn tick(&mut self, time: u32) -> Option<Command> {
        self.tick_common(time).await;

        if self.state != RadioState::Idle {
            return None;
        }

        if self.time % LORA_MESSAGE_INTERVAL == 0 {
            if let Err(e) = self.switch_to_next_frequency().await {
                error!("Failed to switch frequencies: {:?}", Debug2Format(&e));
            }
        }

        if self.is_uplink_window(self.time, false) {
            match self.receive().await {
                Ok(Some(msg)) => {
                    self.last_message_received = self.time;

                    match msg {
                        UplinkMessage::Command(cmd) => Some(cmd),
                        _ => None
                    }
                },
                Ok(None) => None,
                Err(e) => {
                    error!("Error receiving message: {:?}", Debug2Format(&e));
                    None
                }
            }
        } else {
            None
        }
    }

    #[cfg(feature = "gcs")]
    pub async fn tick(&mut self, time: u32) -> Option<DownlinkMessage> {
        if self.sequence.is_none() {
            return None;
        }

        self.tick_common(time).await;

        if self.state != RadioState::Idle {
            return None;
        }

        let in_contact = self.last_message_received > 0 && self.time.wrapping_sub(self.last_message_received) < 5000;
        let fc_time = (self.time as i64).wrapping_add(self.fc_time_offset as i64) as u32;

        // When not in contact with the FC we do a slow sweep across channels.
        if !in_contact && self.time % 1000 == 0 {
            let i = (self.time as usize / 1000) % CHANNELS.len();
            info!("Sweeping, switching to {}kHz.", CHANNELS[i] / 1_000);
            if let Err(e) = self.trx.set_frequency(CHANNELS[i]).await {
                error!("Failed to switch frequencies: {:?}", Debug2Format(&e));
            }

            if let Err(e) = self.trx.switch_to_rx().await {
                error!("Failed to switch frequencies: {:?}", Debug2Format(&e));
            }
        }

        if in_contact && fc_time % LORA_MESSAGE_INTERVAL == 0 {
            if let Err(e) = self.switch_to_next_frequency().await {
                error!("Failed to switch frequencies: {:?}", Debug2Format(&e));
            }

            if let Err(e) = self.trx.switch_to_rx().await {
                error!("Failed to switch frequencies: {:?}", Debug2Format(&e));
            }
        }

        if in_contact && self.is_uplink_window(fc_time.wrapping_sub(2), true) {
            let msg = self.uplink_message.take().unwrap_or(UplinkMessage::Heartbeat);
            if let Err(e) = self.send(msg).await {
                error!("Failed to send uplink message: {:?}", Debug2Format(&e));
            }

            None
        } else {
            let result: Result<Option<DownlinkMessage>, _> = self.receive().await;
            match &result {
                Ok(Some(msg)) => {
                    self.last_message_received = self.time;
                    self.fc_time_offset = (msg.time() as i64)
                        .wrapping_sub(self.time as i64)
                        .wrapping_add(FC_GCS_TIME_OFFSET_MS); // compensate for message delay

                    if let DownlinkMessage::TelemetryDiagnostics(tm) = msg {
                        self.transmit_power_setpoint = (tm.transmit_power_and_data_rate & 0x7f).into();
                    }
                }
                Ok(None) => {},
                Err(e) => {
                    error!("Error receiving message: {:?}", Debug2Format(&e));
                }
            }

            result.unwrap_or(None)
        }
    }
}
