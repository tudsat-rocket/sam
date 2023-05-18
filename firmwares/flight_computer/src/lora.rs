//! Driver for the LLCC68 LoRa transceiver and our telemetry implementation. Just
//! like `flash.rs`, this could probably be separated into driver and protocol
//! implementations.
//!
//! Datasheet: https://www.mouser.com/pdfDocs/DS_LLCC68_V10-2.pdf

use core::cell::RefCell;
use core::ops::DerefMut;
use core::hash::Hasher;

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_one::digital::blocking::InputPin;
use embedded_hal_one::spi::blocking::SpiBus;

use cortex_m::interrupt::{free, Mutex};

use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use serde::de::DeserializeOwned;
use siphasher::sip::SipHasher;
use stm32f4xx_hal::gpio::PinPull;

use crate::prelude::*;
use crate::settings::LoRaSettings;
use crate::telemetry::*;

// both RX and TX get half of the available 256 bytes
const TX_BASE_ADDRESS: u8 = 0;
const RX_BASE_ADDRESS: u8 = 64;

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

const TRANSMISSION_TIMEOUT_MS: u32 = 17;

const DOWNLINK_PACKET_SIZE: u8 = 26;
const UPLINK_PACKET_SIZE: u8 = 16;

const TX_PACKET_SIZE: u8 = if cfg!(feature = "gcs") {
    UPLINK_PACKET_SIZE
} else {
    DOWNLINK_PACKET_SIZE
};

const RX_PACKET_SIZE: u8 = if cfg!(feature = "gcs") {
    DOWNLINK_PACKET_SIZE
} else {
    UPLINK_PACKET_SIZE
};

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
    Init,
    Idle,
    Transmitting,
}

#[derive(Debug)]
pub enum LoRaError<E> {
    Spi(E),
    Crc,
    Busy,
    Timeout,
}

impl<E> From<E> for LoRaError<E> {
    fn from(e: E) -> Self {
        Self::Spi(e)
    }
}

pub struct LoRaRadio<SPI, CS, IRQ, BUSY> {
    time: u32,
    state: RadioState,
    state_time: u32,
    spi: Arc<Mutex<RefCell<SPI>>>,
    cs: CS,
    irq: IRQ,
    busy: BUSY,
    pub high_power: bool,
    high_power_configured: bool,
    frequency: u32,
    pub rssi: u8,
    pub rssi_signal: u8,
    pub snr: u8,
    #[cfg(feature="gcs")]
    uplink_message: Option<UplinkMessage>,
    last_message_received: u32,
    #[cfg(feature="gcs")]
    fc_time_offset: i64,
    ignore_busy: bool,
    authentication_key: [u8; 16],
    channels: [bool; CHANNELS.len()],
    binding_phrase: String,
    sequence: Option<[usize; CHANNELS.len()]>,
}

impl<
    SPI: SpiBus,
    CS: OutputPin,
    IRQ: InputPin,
    BUSY: InputPin + PinPull
> LoRaRadio<SPI, CS, IRQ, BUSY> {
    pub fn init(
        spi: Arc<Mutex<RefCell<SPI>>>,
        cs: CS,
        irq: IRQ,
        busy: BUSY
    ) -> Self {
        Self {
            time: 0,
            state: RadioState::Init,
            state_time: 0,
            spi,
            cs,
            irq,
            busy,
            high_power: false,
            high_power_configured: false,
            frequency: CHANNELS[CHANNELS.len() / 2],
            rssi: 255,
            rssi_signal: 255,
            snr: 0,
            #[cfg(feature="gcs")]
            uplink_message: None,
            last_message_received: 0,
            #[cfg(feature="gcs")]
            fc_time_offset: 0,
            ignore_busy: true,
            authentication_key: [0x00; 16],
            channels: [true; CHANNELS.len()],
            binding_phrase: "".into(),
            sequence: None,
        }
    }

    fn command(
        &mut self,
        opcode: LLCC68OpCode,
        params: &[u8],
        response_len: usize,
    ) -> Result<Vec<u8>, LoRaError<SPI::Error>> {
        if self.busy.is_high().unwrap_or(false) && !self.ignore_busy {
            return Err(LoRaError::Busy);
        }

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

            self.cs.set_low().ok();
            let res = spi.transfer_in_place(&mut payload);
            self.cs.set_high().ok();
            res?;

            Ok(payload[(1 + params.len())..].to_vec())
        })
    }

    fn read_register(&mut self, address: u16) -> Result<u8, LoRaError<SPI::Error>> {
        Ok(self.command(LLCC68OpCode::ReadRegister, &address.to_be_bytes(), 2)?[1])
    }

    fn write_register(&mut self, address: u16, value: u8) -> Result<(), LoRaError<SPI::Error>> {
        let buffer = [(address >> 8) as u8, address as u8, value];
        self.command(LLCC68OpCode::WriteRegister, &buffer, 0)?;
        Ok(())
    }

    fn set_packet_type(&mut self, packet_type: LLCC68PacketType) -> Result<(), LoRaError<SPI::Error>> {
        self.command(LLCC68OpCode::SetPacketType, &[packet_type as u8], 0)?;
        Ok(())
    }

    fn set_rf_frequency(&mut self, frequency: u32) -> Result<(), LoRaError<SPI::Error>> {
        const XTAL_FREQ: u32 = 32_000_000;
        const PLL_STEP_SHIFT_AMOUNT: u32 = 14;
        const PLL_STEP_SCALED: u32 = XTAL_FREQ >> (25 - PLL_STEP_SHIFT_AMOUNT);

        let int = frequency / PLL_STEP_SCALED;
        let frac = frequency / (int * PLL_STEP_SCALED);

        let pll = (int << PLL_STEP_SHIFT_AMOUNT) + ((frac << PLL_STEP_SHIFT_AMOUNT) + (PLL_STEP_SCALED >> 1)) / PLL_STEP_SCALED;

        let params = [(pll >> 24) as u8, (pll >> 16) as u8, (pll >> 8) as u8, pll as u8];
        self.command(LLCC68OpCode::SetRfFrequency, &params, 0)?;
        self.frequency = frequency;
        Ok(())
    }

    fn set_output_power(
        &mut self,
        output_power: LLCC68OutputPower,
        ramp_time: LLCC68RampTime,
    ) -> Result<(), LoRaError<SPI::Error>> {
        let (duty_cycle, hp_max) = match output_power {

            LLCC68OutputPower::P14dBm => (0x02, 0x02),
            LLCC68OutputPower::P17dBm => (0x02, 0x03),
            LLCC68OutputPower::P20dBm => (0x03, 0x05),
            LLCC68OutputPower::P22dBm => (0x04, 0x07),
        };
        self.command(LLCC68OpCode::SetPaConfig, &[duty_cycle, hp_max, 0x00, 0x01], 0)?;
        self.command(LLCC68OpCode::SetTxParams, &[22, ramp_time as u8], 0)?;
        //self.command(LLCC68OpCode::SetTxParams, &[0, ramp_time as u8], 0)?;

        // workaround to prevent overly protective power clamping (chapter 15.2, p. 97)
        let tx_clamp_config = self.read_register(0x08d8)?;
        self.write_register(0x08d8, tx_clamp_config | 0x1e)?;

        Ok(())
    }

    fn set_lora_mod_params(
        &mut self,
        bandwidth: LLCC68LoRaModulationBandwidth,
        mut spreading_factor: LLCC68LoRaSpreadingFactor,
        coding_rate: LLCC68LoRaCodingRate,
        low_data_rate_optimization: bool,
    ) -> Result<(), LoRaError<SPI::Error>> {
        if bandwidth == LLCC68LoRaModulationBandwidth::Bw125
            && (spreading_factor == LLCC68LoRaSpreadingFactor::SF10
                || spreading_factor == LLCC68LoRaSpreadingFactor::SF11)
        {
            spreading_factor = LLCC68LoRaSpreadingFactor::SF9;
        }

        if bandwidth == LLCC68LoRaModulationBandwidth::Bw250 && spreading_factor == LLCC68LoRaSpreadingFactor::SF11 {
            spreading_factor = LLCC68LoRaSpreadingFactor::SF10;
        }

        self.command(
            LLCC68OpCode::SetModulationParams,
            &[spreading_factor as u8, bandwidth as u8, coding_rate as u8, low_data_rate_optimization as u8],
            0,
        )?;
        Ok(())
    }

    fn set_lora_packet_params(
        &mut self,
        preamble_length: u16,
        fixed_length_header: bool,
        payload_length: u8,
        crc: bool,
        invert_iq: bool,
    ) -> Result<(), LoRaError<SPI::Error>> {
        let preamble_length = u16::max(1, preamble_length);
        self.command(
            LLCC68OpCode::SetPacketParams,
            &[
                (preamble_length >> 8) as u8,
                preamble_length as u8,
                fixed_length_header as u8,
                payload_length,
                crc as u8,
                invert_iq as u8,
            ],
            0,
        )?;
        Ok(())
    }

    fn set_buffer_base_addresses(&mut self, tx_address: u8, rx_address: u8) -> Result<(), LoRaError<SPI::Error>> {
        self.command(LLCC68OpCode::SetBufferBaseAddress, &[tx_address, rx_address], 0)?;
        Ok(())
    }

    fn set_dio1_interrupt(&mut self, irq_mask: u16, dio1_mask: u16) -> Result<(), LoRaError<SPI::Error>> {
        self.command(
            LLCC68OpCode::SetDioIrqParams,
            &[(irq_mask >> 8) as u8, irq_mask as u8, (dio1_mask >> 8) as u8, dio1_mask as u8, 0, 0, 0, 0],
            0,
        )?;
        Ok(())
    }

    fn switch_to_rx(&mut self) -> Result<(), LoRaError<SPI::Error>> {
        self.set_lora_packet_params(12, true, RX_PACKET_SIZE, true, false)?;
        self.set_rx_mode(0)?;
        Ok(())
    }

    fn configure(&mut self) -> Result<(), LoRaError<SPI::Error>> {
        self.ignore_busy = true;
        self.busy.set_internal_resistor(stm32f4xx_hal::gpio::Pull::Down);

        // Force the device to sleep to reset configuration, then wait for it
        // to finish setting up.
        self.command(LLCC68OpCode::SetSleep, &[0x00], 0)?;

        let mut done = false;
        for _i in 0..50 {
            done = self.command(LLCC68OpCode::GetStatus, &[], 1)
                .map(|x| (x[0] >> 4) == 0x2)
                .unwrap_or(false);
            if done {
                break;
            }
        }

        if !done {
            return Err(LoRaError::Timeout);
        }

        // The busy line can either indicate a running command or that the LLCC68
        // is in sleep mode (for instance if a previous setmode command failed).
        // Successfully running a command means it should be back now, so we can
        // go back to treating the busy line more seriously, since we want to make
        // sure the following commands are executed.
        self.ignore_busy = false;
        self.busy.set_internal_resistor(stm32f4xx_hal::gpio::Pull::None);

        self.command(LLCC68OpCode::SetDIO2AsRfSwitchCtrl, &[1], 0)?;
        //self.command(LLCC68OpCode::CalibrateImage, &[0xd7, 0xdb], 0)?;
        self.write_register(0x08ac, 0x96)?; // boost rx gain (9.6, p. 53)
        self.set_packet_type(LLCC68PacketType::LoRa)?;
        self.set_lora_mod_params(
            LLCC68LoRaModulationBandwidth::Bw500,
            LLCC68LoRaSpreadingFactor::SF7,
            LLCC68LoRaCodingRate::CR4of6,
            false,
        )?;
        self.set_rf_frequency(self.frequency)?;
        self.set_buffer_base_addresses(TX_BASE_ADDRESS, RX_BASE_ADDRESS)?;
        self.set_output_power(LLCC68OutputPower::P14dBm, LLCC68RampTime::R20U)?;
        self.set_dio1_interrupt(
            (LLCC68Interrupt::RxDone as u16) | (LLCC68Interrupt::CrcErr as u16),
            LLCC68Interrupt::RxDone as u16,
        )?;
        self.switch_to_rx()?;

        // After the configuration we can treat the busy line a bit more relaxed,
        // it sometimes can go high seemingly incorrectly (or if mode change
        // fails), but during normal operation this should solve itself.
        self.ignore_busy = true;

        Ok(())
    }

    fn set_tx_mode(&mut self, timeout_us: u32) -> Result<(), LoRaError<SPI::Error>> {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command(
            LLCC68OpCode::SetTx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0
        )?;
        Ok(())
    }

    fn set_rx_mode(&mut self, _timeout_us: u32) -> Result<(), LoRaError<SPI::Error>> {
        let timeout = 0; // TODO
        self.command(
            LLCC68OpCode::SetRx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0,
        )?;
        Ok(())
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
        log!(Info, "Generated sequence {:?} using phrase {:?}", self.sequence, settings.binding_phrase);
    }

    fn switch_to_next_frequency(&mut self) -> Result<(), LoRaError<SPI::Error>> {
        // Switch to the correct frequency for the current message interval.
        // On the FC, this is pretty straight forward.

        #[cfg(not(feature="gcs"))]
        let t = self.time;
        #[cfg(feature="gcs")]
        let t = (self.time as i64).wrapping_add(self.fc_time_offset) as u32;

        let message_i = (t / LORA_MESSAGE_INTERVAL) as usize % CHANNELS.len();
        self.set_rf_frequency(CHANNELS[self.sequence.map(|s| s[message_i]).unwrap_or(0)])
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

    fn send_packet(&mut self, msg: &[u8]) -> Result<(), LoRaError<SPI::Error>> {
        if self.sequence.is_none() {
            return Ok(());
        }

        if self.state != RadioState::Idle {
            log!(Error, "skipping");
            return Ok(()); // TODO
        }

        // Prepend message authentication
        let mut siphasher = SipHasher::new_with_key(&self.authentication_key);
        #[cfg(feature="gcs")] // only include time for uplink messages, prevents replay attacks
        siphasher.write(&self.start_of_current_interval().to_be_bytes());
        siphasher.write(&msg);
        let hash = (siphasher.finish() as TxHmac).to_be_bytes();
        let msg = [&hash, msg].concat();

        if msg.len() > TX_PACKET_SIZE as usize {
            log!(Error, "message exceeds PACKET_SIZE");
            return Ok(());
        }

        // The LLCC68 datasheet mentions this workaround to prevent modulation quality
        // issues with 500khz bandwidth. (chapter 15.1, p. 97)
        // This should be changed if we change bandwidths.
        let reg = self.read_register(0x0889)?;
        if reg & 0xfb != reg {
            log!(Info, "Applying LLCC68 mod quality workaround.");
            self.write_register(0x0889, reg & 0xfb)?;
        }

        self.set_lora_packet_params(12, true, TX_PACKET_SIZE, true, false)?;
        const CMD_SIZE: usize = (TX_PACKET_SIZE as usize) + 1;
        let mut params: [u8; CMD_SIZE] = [0x00; CMD_SIZE];
        params[0] = TX_BASE_ADDRESS;
        params[1..(msg.len()+1)].copy_from_slice(&msg);
        self.command(LLCC68OpCode::WriteBuffer, &params, 0)?;
        self.set_tx_mode(TRANSMISSION_TIMEOUT_MS * 1000)?;
        self.set_state(RadioState::Transmitting);
        Ok(())
    }

    #[cfg(not(feature="gcs"))]
    pub fn send_downlink_message(&mut self, msg: DownlinkMessage) {
        let serialized = match msg.serialize() {
            Ok(b) => b,
            Err(e) => {
                log!(Error, "Failed to serialize message: {:?}", e);
                return;
            }
        };

        if let Err(e) = self.send_packet(&serialized) {
            log!(Error, "Error sending LoRa packet: {:?}", e);
        }
    }

    #[cfg(feature="gcs")]
    fn send_uplink_message(&mut self, msg: UplinkMessage) -> Result<(), LoRaError<SPI::Error>> {
        self.send_packet(&msg.serialize().unwrap_or_default())
    }

    #[cfg(feature="gcs")]
    pub fn queue_uplink_message(&mut self, msg: UplinkMessage) {
        self.uplink_message = Some(msg);
    }

    fn receive_data(&mut self) -> Result<Option<Vec<u8>>, LoRaError<SPI::Error>> {
        // No RxDone interrupt, do nothing
        if !self.irq.is_high().unwrap() {
            return Ok(None);
        }

        // Get IRQ status to allow checking for CrcErr
        let irq_status = self
            .command(LLCC68OpCode::GetIrqStatus, &[], 3)
            .map(|r| ((r[1] as u16) << 8) + (r[2] as u16))
            .unwrap_or(0);

        self.command(LLCC68OpCode::ClearIrqStatus, &[0xff, 0xff], 0)?;

        // Get the packet stats before the data, since this is useful even if the data is corrupted
        let packet_status = self.command(LLCC68OpCode::GetPacketStatus, &[], 4)?;
        self.rssi = packet_status[1];
        self.rssi_signal = packet_status[3];
        self.snr = packet_status[2];

        // Abort in case of a CRC mismatch
        if irq_status & (LLCC68Interrupt::CrcErr as u16) > 0 {
            return Err(LoRaError::Crc);
        }

        // Get RX buffer status (this contains the length of the received data)
        let rx_buffer_status = self.command(LLCC68OpCode::GetRxBufferStatus, &[], 3)?;
        let len = u8::min(rx_buffer_status[1], RX_PACKET_SIZE);

        // Read received data
        let buffer = self.command(
            LLCC68OpCode::ReadBuffer,
            &[rx_buffer_status[2]],
            len as usize + 1,
        )?;

        self.set_rx_mode(0)?;

        Ok(Some(buffer))
    }

    fn receive_message<M: Transmit + DeserializeOwned>(&mut self) -> Result<Option<M>, LoRaError<SPI::Error>> {
        let mut buffer = self.receive_data()?.unwrap_or_default();
        if buffer.len() < UPLINK_PACKET_SIZE as usize {
            return Ok(None);
        }

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
            log!(Warning, "HMAC mismatch.");
            return Ok(None);
        }

        let deserialized = postcard::from_bytes_cobs(serialized).ok();
        if deserialized.is_none() {
            log!(Error, "{}: Failed to decode message: {:02x?}", self.time, &buffer[1..]);
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

    fn tick_common(&mut self, time: u32) {
        self.time = time;

        if self.state == RadioState::Init {
            if let Err(e) = self.configure() {
                log!(Error, "Error configuring LoRa transceiver: {:?}", e);
            } else {
                self.set_state(RadioState::Idle);
            }
        }

        // Return to rx mode after transmission. A delay is necessary in order
        // to allow the LLCC68 to actually finish the transmission
        if self.state == RadioState::Transmitting && time >= self.state_time.wrapping_add(TRANSMISSION_TIMEOUT_MS + 2) {
            if let Err(e) = self.switch_to_rx() {
                log!(Error, "Failed to return to RX mode: {:?}", e);
            } else {
                self.set_state(RadioState::Idle);
            }
        }

        if self.high_power != self.high_power_configured {
            let power = if self.high_power {
                LLCC68OutputPower::P22dBm
            } else {
                LLCC68OutputPower::P14dBm
            };

            if let Err(e) = self.set_output_power(power, LLCC68RampTime::R20U) {
                log!(Error, "Error setting power level: {:?}", e);
            } else {
                self.high_power_configured = self.high_power;
            }
        }
    }

    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self, time: u32, mode: FlightMode) -> Option<Command> {
        self.tick_common(time);
        self.high_power = mode >= FlightMode::Armed;

        if self.state != RadioState::Idle {
            return None;
        }

        if self.time % LORA_MESSAGE_INTERVAL == 0 {
            if let Err(e) = self.switch_to_next_frequency() {
                log!(Error, "Failed to switch frequencies: {:?}", e);
            }
        }

        if self.is_uplink_window(self.time, false) {
            match self.receive_message() {
                Ok(Some(msg)) => {
                    self.last_message_received = self.time;

                    match msg {
                        UplinkMessage::Heartbeat => None,
                        UplinkMessage::Command(cmd) => {
                            Some(cmd)
                        },
                        msg => {
                            log!(Warning, "Unsupported message type via LoRa: {:?}", msg);
                            None
                        }
                    }
                },
                Ok(None) => None,
                Err(e) => {
                    log!(Error, "Error receiving message: {:?}", e);
                    None
                }
            }
        } else {
            None
        }
    }

    #[cfg(feature = "gcs")]
    pub fn tick(&mut self, time: u32) -> Option<DownlinkMessage> {
        self.tick_common(time);

        if self.state != RadioState::Idle {
            return None;
        }

        let in_contact = self.last_message_received > 0 && self.time.wrapping_sub(self.last_message_received) < 5000;
        let fc_time = (self.time as i64).wrapping_add(self.fc_time_offset as i64) as u32;

        // When not in contact with the FC we do a slow sweep across channels.
        if !in_contact && self.time % 2000 == 0 {
            let i = (self.time as usize / 2000) % CHANNELS.len();
            log!(Info, "Sweeping, switching to {}MHz.", (CHANNELS[i] as f32) / 1_000_000.0);
            if let Err(e) = self.set_rf_frequency(CHANNELS[i]).and_then(|()| self.switch_to_rx()) {
                log!(Error, "Failed to switch frequencies: {:?}", e);
            }
        }

        if in_contact && fc_time % LORA_MESSAGE_INTERVAL == 0 {
            if let Err(e) = self.switch_to_next_frequency().and_then(|()| self.switch_to_rx()) {
                log!(Error, "Failed to switch frequencies: {:?}", e);
            }
        }

        if in_contact && self.is_uplink_window(fc_time.wrapping_sub(2), true) {
            let msg = self.uplink_message.take().unwrap_or(UplinkMessage::Heartbeat);
            if let Err(e) = self.send_uplink_message(msg) {
                log!(Error, "Failed to send uplink message: {:?}", e);
            }

            None
        } else {
            let result: Result<Option<DownlinkMessage>, _> = self.receive_message();
            match &result {
                Ok(Some(msg)) => {
                    self.last_message_received = self.time;
                    self.fc_time_offset = (msg.time() as i64)
                        .wrapping_sub(self.time as i64)
                        .wrapping_add(TRANSMISSION_TIMEOUT_MS as i64); // compensate for message delay

                    if let DownlinkMessage::TelemetryMainCompressed(tm) = msg {
                        self.high_power = tm.mode >= FlightMode::Armed;
                    }
                }
                Ok(None) => {},
                Err(e) => {
                    log!(Error, "Error receiving message: {:?}", e);
                }
            }

            result.unwrap_or(None)
        }
    }
}

#[derive(Clone, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68Interrupt {
    TxDone = 0x01,
    RxDone = 0x02,
    PreambleDetected = 0x04,
    SyncWordValid = 0x08,
    HeaderValid = 0x10,
    HeaderErr = 0x20,
    CrcErr = 0x40,
    CadDone = 0x80,
    CadDetected = 0x100,
    Timeout = 0x200,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68OpCode {
    // Operational Modes (11.1, page 57)
    SetSleep = 0x84,
    SetStandby = 0x80,
    SetFs = 0xc1,
    SetTx = 0x83,
    SetRx = 0x82,
    StopTimerOnPreamble = 0x9f,
    SetRxDutyCycle = 0x94,
    SetCad = 0xc5,
    SetTxContinuousWave = 0xd1,
    SetTxInfinitePreamble = 0xd2,
    SetRegulatorMode = 0x96,
    Calibrate = 0x89,
    CalibrateImage = 0x98,
    SetPaConfig = 0x95,
    SetRxTxFallbackMode = 0x93,
    // Register & Buffer Access (11.2, page 58)
    WriteRegister = 0x0d,
    ReadRegister = 0x1d,
    WriteBuffer = 0x0e,
    ReadBuffer = 0x1e,
    // DIO & IRQ Control (11.3, page 58)
    SetDioIrqParams = 0x08,
    GetIrqStatus = 0x12,
    ClearIrqStatus = 0x02,
    SetDIO2AsRfSwitchCtrl = 0x9d,
    SetDIO3AsTcxoCtrl = 0x97,
    // RF, Modulation & Packet (11.4, page 58)
    SetRfFrequency = 0x86,
    SetPacketType = 0x8a,
    GetPacketType = 0x11,
    SetTxParams = 0x8e,
    SetModulationParams = 0x8b,
    SetPacketParams = 0x8c,
    SetCadParams = 0x88,
    SetBufferBaseAddress = 0x8f,
    SetLoRaSymbNumTimeout = 0xa0,
    // Status (11.5, page 59)
    GetStatus = 0xc0,
    GetRssiInst = 0x15,
    GetRxBufferStatus = 0x13,
    GetPacketStatus = 0x14,
    GetDeviceErrors = 0x17,
    ClearDeviceErrors = 0x07,
    GetStats = 0x10,
    ResetStats = 0x00,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68PacketType {
    GFSK = 0x00,
    LoRa = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68OutputPower {
    P14dBm = 14,
    P17dBm = 17,
    P20dBm = 20,
    P22dBm = 22,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68RampTime {
    R10U = 0x00,
    R20U = 0x01,
    R40U = 0x02,
    R80U = 0x03,
    R200U = 0x04,
    R800U = 0x05,
    R1700U = 0x06,
    R3400U = 0x07,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68LoRaModulationBandwidth {
    Bw125 = 0x04,
    Bw250 = 0x05,
    Bw500 = 0x06,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68LoRaSpreadingFactor {
    SF5 = 0x05,
    SF6 = 0x06,
    SF7 = 0x07,
    SF8 = 0x08,
    SF9 = 0x09,
    SF10 = 0x0a,
    SF11 = 0x0b,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68LoRaCodingRate {
    CR4of5 = 0x01,
    CR4of6 = 0x02,
    CR4of7 = 0x03,
    CR4of8 = 0x04,
}
