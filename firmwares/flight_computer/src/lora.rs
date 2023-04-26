//! Driver for the LLCC68 LoRa transceiver and our telemetry implementation. Just
//! like `flash.rs`, this could probably be separated into driver and protocol
//! implementations.
//!
//! Datasheet: https://www.mouser.com/pdfDocs/DS_LLCC68_V10-2.pdf

use core::cell::RefCell;
use core::ops::DerefMut;
use core::hash::Hasher;

use alloc::sync::Arc;
use alloc::vec::Vec;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal_one::digital::blocking::InputPin;
use embedded_hal_one::spi::blocking::TransferInplace;

use cortex_m::interrupt::{free, Mutex};
use hal::dma::config::DmaConfig;
use hal::dma::{MemoryToPeripheral, Transfer, StreamX};
use hal::pac::{interrupt, Interrupt, DMA2, NVIC, SPI1};
use hal::spi::Tx;
use stm32f4xx_hal as hal;

use siphasher::sip::SipHasher;
use serde::de::DeserializeOwned;

use crate::prelude::*;
use crate::telemetry::*;

// both RX and TX get half of the available 256 bytes
const TX_BASE_ADDRESS: u8 = 0;
const RX_BASE_ADDRESS: u8 = 64;

const DMA_BUFFER_SIZE: usize = 32;

const FREQUENCY: u32 = 868_000_000;

const RX_RETURN_DELAY: u32 = 25;
const TRANSMISSION_TIMEOUT_MS: u32 = 25;

const DOWNLINK_PACKET_SIZE: u8 = 24;
const UPLINK_PACKET_SIZE: u8 = 16;

type SpiDmaTransfer = Transfer<
    StreamX<DMA2, 3>, // TODO
    3,
    Tx<SPI1>,
    MemoryToPeripheral,
    &'static mut [u8; DMA_BUFFER_SIZE],
>;

static DMA_TRANSFER: Mutex<RefCell<Option<SpiDmaTransfer>>> = Mutex::new(RefCell::new(None));
static DMA_RESULT: Mutex<RefCell<Option<Result<(), ()>>>> = Mutex::new(RefCell::new(None));
static mut DMA_BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];

#[derive(Debug, PartialEq, Eq)]
#[allow(dead_code)] // TODO
enum RadioState {
    Init,
    Idle,
    Writing,
    Transmitting,
    Reading,
}

#[derive(Debug)]
pub enum LoRaError<E> {
    Spi(E),
    Crc,
    #[allow(dead_code)]
    Busy,
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
    last_sync: u32,
    spi: Arc<Mutex<RefCell<SPI>>>,
    cs: CS,
    #[allow(dead_code)] // TODO
    irq: IRQ,
    #[allow(dead_code)] // TODO
    busy: BUSY,
    pub high_power: bool,
    high_power_configured: bool,
    pub rssi: u8,
    pub rssi_signal: u8,
    pub snr: u8,
    #[cfg(feature="gcs")]
    uplink_message: Option<UplinkMessage>,
    #[cfg(not(feature="gcs"))]
    siphasher: SipHasher,
    #[cfg(not(feature="gcs"))]
    last_hash: u64,
}

#[interrupt]
fn DMA2_STREAM3() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::DMA2_STREAM3);

    free(|cs| {
        let transfer = &mut *DMA_TRANSFER.borrow(cs).borrow_mut();
        if let Some(ref mut transfer) = transfer.as_mut() {
            transfer.clear_fifo_error_interrupt();
            transfer.clear_transfer_complete_interrupt();
        }

        let result = Some(Ok(())); // TODO: properly track errors
        *DMA_RESULT.borrow(cs).borrow_mut() = result;
    })
}

impl<
    SPI: TransferInplace,
    CS: OutputPin,
    IRQ: InputPin,
    BUSY: InputPin
> LoRaRadio<SPI, CS, IRQ, BUSY> {
    pub fn init(
        spi: Arc<Mutex<RefCell<SPI>>>,
        cs: CS,
        dma_tx: Tx<SPI1>,
        dma_stream: StreamX<DMA2, 3>,
        irq: IRQ,
        busy: BUSY
    ) -> Self {
        let mut transfer = unsafe {
            Transfer::init_memory_to_peripheral(
                dma_stream,
                dma_tx,
                &mut DMA_BUFFER,
                None,
                DmaConfig::default()
                    .memory_increment(true)
                    .fifo_enable(true)
                    .fifo_error_interrupt(true)
                    .transfer_complete_interrupt(true),
            )
        };

        transfer.start(|_tx| {});

        // Hand off transfer to interrupt handler
        free(|cs| *DMA_TRANSFER.borrow(cs).borrow_mut() = Some(transfer));

        Self {
            time: 0,
            state: RadioState::Init,
            state_time: 0,
            last_sync: 0,
            spi,
            cs,
            irq,
            busy,
            high_power: false,
            high_power_configured: false,
            rssi: 255,
            rssi_signal: 255,
            snr: 0,
            #[cfg(feature="gcs")]
            uplink_message: None,
            #[cfg(not(feature="gcs"))]
            siphasher: SipHasher::new_with_key(&SIPHASHER_KEY),
            #[cfg(not(feature="gcs"))]
            last_hash: 0,
        }
    }

    fn command(
        &mut self,
        opcode: LLCC68OpCode,
        params: &[u8],
        response_len: usize,
    ) -> Result<Vec<u8>, LoRaError<SPI::Error>> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut payload = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

            self.cs.set_low().ok();

            // At 10MHz SPI freq, the setup commands seem to require some
            // extra delay between cs going low and the SCK going high
            for _i in 0..300 {
                core::hint::spin_loop()
            }

            let res = spi.transfer_inplace(&mut payload);
            self.cs.set_high().ok();
            res?;

            Ok(payload[(1 + params.len())..].to_vec())
        })
    }

    fn command_dma(&mut self, opcode: LLCC68OpCode, params: &[u8]) -> () {
        if params.len() > DMA_BUFFER_SIZE - 1 {
            return;
        }

        self.cs.set_low().ok();

        // At 10MHz SPI freq, the setup commands seem to require some
        // extra delay between cs going low and the SCK going high
        for _i in 0..=200 {
            core::hint::spin_loop()
        }

        free(|cs| {
            let msg = [&[opcode as u8], params, &[0].repeat(DMA_BUFFER_SIZE - params.len() - 1)].concat();

            let mut transfer = DMA_TRANSFER.borrow(cs).borrow_mut();
            if let Some(ref mut transfer) = transfer.as_mut() {
                unsafe {
                    DMA_BUFFER = msg.try_into().unwrap();
                    transfer.next_transfer(&mut DMA_BUFFER).unwrap();
                    transfer.start(|_tx| {});
                }
            }

            *DMA_RESULT.borrow(cs).borrow_mut() = None;

            unsafe {
                NVIC::unmask(hal::pac::Interrupt::DMA2_STREAM3);
            }
        })
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

        // TODO: also set txclampconfig
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
        self.set_lora_packet_params(12, false, UPLINK_MAX_LEN, true, false)?; // TODO
        self.set_rx_mode(0)?;
        Ok(())
    }

    fn configure(&mut self) -> Result<(), LoRaError<SPI::Error>> {
        let mut result = self.command(LLCC68OpCode::GetStatus, &[], 1);
        for _i in 1..5 {
            if result.is_ok() {
                break;
            }
            result = self.command(LLCC68OpCode::GetStatus, &[], 1);
        }

        result?;

        self.command(LLCC68OpCode::SetDIO2AsRfSwitchCtrl, &[1], 0)?;
        //self.command(LLCC68OpCode::CalibrateImage, &[0xd7, 0xdb], 0)?;
        self.command(LLCC68OpCode::WriteRegister, &[0x08, 0xac, 0x96], 0)?; // boost rx gain
        self.set_packet_type(LLCC68PacketType::LoRa)?;
        self.set_rf_frequency(FREQUENCY)?;
        self.set_lora_mod_params(
            LLCC68LoRaModulationBandwidth::Bw500,
            LLCC68LoRaSpreadingFactor::SF6,
            LLCC68LoRaCodingRate::CR4of6,
            false,
        )?;
        self.set_buffer_base_addresses(TX_BASE_ADDRESS, RX_BASE_ADDRESS)?;
        self.set_output_power(LLCC68OutputPower::P14dBm, LLCC68RampTime::R20U)?;
        self.set_dio1_interrupt(
            (LLCC68Interrupt::RxDone as u16) | (LLCC68Interrupt::CrcErr as u16),
            LLCC68Interrupt::RxDone as u16,
        )?;
        self.switch_to_rx()?;
        Ok(())
    }

    fn set_tx_mode_dma(&mut self, timeout_us: u32) {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command_dma(
            LLCC68OpCode::SetTx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
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

    fn send_packet(&mut self, msg: &[u8]) -> Result<(), LoRaError<SPI::Error>> {
        if self.state != RadioState::Idle {
            log!(Error, "skipping");
            return Ok(()); // TODO
        }

        self.set_lora_packet_params(12, false, msg.len() as u8, true, false)?;
        let mut params = Vec::with_capacity(msg.len() + 1);
        params.push(TX_BASE_ADDRESS);
        params.append(&mut msg.to_vec());
        self.command_dma(LLCC68OpCode::WriteBuffer, &params);
        self.set_state(RadioState::Writing);
        Ok(())
    }

    #[cfg(not(feature="gcs"))]
    pub fn send_downlink_message(&mut self, msg: DownlinkMessage) {
        if let DownlinkMessage::TelemetryGPS(_) = msg {
            self.last_sync = self.time;
        }

        // TODO
        let serialized = match msg.serialize() {
            Ok(b) => b[..b.len()-1].to_vec(), // last byte is always 0, not needed
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
        let serialized = msg.serialize()
            .map(|b| b[..(b.len()-1)].to_vec()) // the last byte is always 0, not needed
            .unwrap_or_default(); // TODO
        self.send_packet(&serialized)
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

        // Avoid unnecessarily long reads for corrupted uplink commands
        #[cfg(feature="gcs")]
        let len = rx_buffer_status[1];
        #[cfg(not(feature="gcs"))]
        let len = u8::min(rx_buffer_status[1], UPLINK_MAX_LEN);

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
        if buffer.len() == 0 {
            return Ok(None);
        }

        // TODO: move deserialization code to src/telemetry
        let deserialized = postcard::from_bytes_cobs(&mut buffer[1..]).ok();
        if deserialized.is_none() {
            log!(Error, "Failed to decode message: {:02x?}", buffer);
        }

        Ok(deserialized)
    }

    fn is_uplink_window(&self, time: u32, first_only: bool) -> bool {
        let mut t = time.saturating_sub(self.last_sync) % 1000;
        if !first_only {
            t -= t % LORA_MESSAGE_INTERVAL;
        }
        t != 0 && (t % LORA_UPLINK_INTERVAL) == LORA_UPLINK_MODULO
    }

    fn check_dma_result(&mut self) -> Option<Result<(), ()>> {
        let result = free(|cs| DMA_RESULT.borrow(cs).replace(None));
        if result.is_some() {
            self.cs.set_high().ok();
        }
        result
    }

    fn tick_common(&mut self, time: u32) {
        self.time = time;

        if self.state == RadioState::Init {
            if let Err(e) = self.configure() {
                log!(Error, "Error configuring LoRa transceiver: {:?}", e);
            } else {
                self.set_state(RadioState::Idle);
            }
        } else if self.state == RadioState::Writing {
            if let Some(_result) = self.check_dma_result() {
                self.set_tx_mode_dma(TRANSMISSION_TIMEOUT_MS * 1000);
                self.set_state(RadioState::Transmitting);
            }
        } else if self.state == RadioState::Transmitting {
            if let Some(_result) = self.check_dma_result() {
                self.set_state(RadioState::Idle);
            }
        }

        // Return to rx mode after transmission. A delay is necessary in order
        // to allow the LLCC68 to actually finish the transmission
        if self.state == RadioState::Idle && time == self.state_time + RX_RETURN_DELAY {
            // The ground station only sends single messages, so always go
            // back to receiving
            #[cfg(feature="gcs")]
            {
                for _i in 0..3 { // TODO: why is this necessary?
                    if let Err(_e) = self.switch_to_rx() {
                        //log!(Error, "Failed to return to RX mode: {:?}", e);
                    } else {
                        break
                    }
                }
            }

            // The vehicle switches to RX mode if the next radio interval
            // is an uplink window
            #[cfg(not(feature="gcs"))]
            {
                let next_msg = self.time + LORA_MESSAGE_INTERVAL - (self.time % LORA_MESSAGE_INTERVAL);
                if self.is_uplink_window(next_msg, false) {
                    if let Err(_e) = self.switch_to_rx() {
                        //log!(Error, "Failed to return to RX mode: {:?}", e);
                    }
                }
            }
        }

        if (self.state == RadioState::Writing || self.state == RadioState::Transmitting)
            && ((self.state_time + TRANSMISSION_TIMEOUT_MS) < self.time)
        {
            log!(Error, "LoRa DMA timeout");
            self.set_state(RadioState::Idle);
        }

        if self.high_power != self.high_power_configured {
            let power = if self.high_power {
                LLCC68OutputPower::P22dBm
            } else {
                LLCC68OutputPower::P17dBm
            };

            if let Err(e) = self.set_output_power(power, LLCC68RampTime::R20U) {
                log!(Error, "Error setting power level: {:?}", e);
            } else {
                self.high_power_configured = self.high_power;
            }
        }
    }

    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self, time: u32, mode: FlightMode) -> Option<UplinkMessage> {
        self.tick_common(time);
        self.high_power = mode >= FlightMode::Armed;

        if time % LORA_MESSAGE_INTERVAL == 0 {
            self.last_hash = self.siphasher.finish();
            self.siphasher.write_u64(self.last_hash);
        }

        if self.is_uplink_window(time, false) {
            match self.receive_message() {
                Ok(opt) => {
                    if let Some(UplinkMessage::RebootAuth(mac)) |
                            Some(UplinkMessage::SetFlightModeAuth(_, mac)) |
                            Some(UplinkMessage::EraseFlashAuth(mac)) = opt {
                        let current = self.siphasher.finish();
                        if mac != self.last_hash && mac != current {
                            log!(Error, "MAC mismatch: {:02x?} vs ({:02x?}, {:02x?})", mac, self.last_hash, current);
                            return None;
                        }
                    }
                    opt
                },
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

        if self.last_sync > 0 && (time - self.last_sync) < 5000 && self.is_uplink_window(time + 5, true) {
            let msg = self.uplink_message.take().unwrap_or(UplinkMessage::Heartbeat);
            if let Err(e) = self.send_uplink_message(msg) {
                log!(Error, "Failed to send uplink message: {:?}", e);
            }

            None
        } else {
            let result: Result<Option<DownlinkMessage>, _> = self.receive_message();
            match &result {
                Ok(msg) => {
                    if let Some(DownlinkMessage::TelemetryGPS(_)) = msg {
                        self.last_sync = time;
                    } else if let Some(DownlinkMessage::TelemetryMainCompressed(tm)) = msg {
                        self.high_power = tm.mode >= FlightMode::Armed;
                    }
                }
                Err(e) => log!(Error, "Error receiving message: {:?}", e),
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
