//! Driver for the LLCC68 LoRa transceiver and our telemetry implementation. Just
//! like `flash.rs`, this could probably be separated into driver and protocol
//! implementations.
//!
//! Datasheet: https://www.mouser.com/pdfDocs/DS_LLCC68_V10-2.pdf

use heapless::Vec;

use embedded_hal::digital::InputPin;
use embedded_hal_async::spi::SpiDevice;

use embassy_time::{Duration, Timer};

use shared_types::*;

use crate::lora::RadioError;

// both RX and TX get half of the available 256 bytes
const TX_BASE_ADDRESS: u8 = 0;
const RX_BASE_ADDRESS: u8 = 64;

pub const TRANSMISSION_TIMEOUT_MS: u32 = 18;
#[cfg(feature="gcs")]
pub const FC_GCS_TIME_OFFSET_MS: i64 = 16;

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

const RAMP_TIME: LLCC68RampTime = LLCC68RampTime::R800U;

pub struct LLCC68<SPI, IRQ, BUSY> {
    spi: SPI,
    irq: IRQ,
    busy: BUSY,
    ignore_busy: bool,
    frequency: u32,
    pub rssi: u8,
    pub rssi_signal: u8,
    pub snr: i8,
}

impl<SPI: SpiDevice<u8>, IRQ: InputPin, BUSY: InputPin> LLCC68<SPI, IRQ, BUSY> {
    pub async fn init(spi: SPI, irq: IRQ, busy: BUSY, frequency: u32) -> Result<Self, RadioError<SPI::Error>> {
        let mut llcc68 = LLCC68 {
            spi,
            irq,
            busy,
            frequency,
            ignore_busy: true,
            // TODO
            rssi: 255,
            rssi_signal: 255,
            snr: 0,
        };

        llcc68.configure().await?;

        Ok(llcc68)
    }

    async fn configure(&mut self) -> Result<(), RadioError<SPI::Error>> {
        self.ignore_busy = true;

        // Wait for LLCC68 to enter standby mode
        let mut done = false;
        for _i in 0..20 {
            done = self.command(LLCC68OpCode::GetStatus, &[], 1).await
                .map(|x| (x[0] >> 4) == 0x2)
                .unwrap_or(false);
            if done {
                break;
            }

            Timer::after(Duration::from_millis(1)).await;
        }

        if !done {
            // Force the device to sleep to reset configuration
            //self.command(LLCC68OpCode::SetSleep, &[0x00], 0).await?;
            //return Err(RadioError::Timeout);
        }

        // The busy line can either indicate a running command or that the LLCC68
        // is in sleep mode (for instance if a previous setmode command failed).
        // Successfully running a command means it should be back now, so we can
        // go back to treating the busy line more seriously, since we want to make
        // sure the following commands are executed.
        self.ignore_busy = false;

        self.command(LLCC68OpCode::SetDIO2AsRfSwitchCtrl, &[1], 0).await?;
        //self.command(LLCC68OpCode::CalibrateImage, &[0xd7, 0xdb], 0)?;
        self.write_register(0x08ac, 0x96).await?; // boost rx gain (9.6, p. 53)
        self.set_packet_type(LLCC68PacketType::LoRa).await?;
        self.set_lora_mod_params(
            LLCC68LoRaModulationBandwidth::Bw500,
            LLCC68LoRaSpreadingFactor::SF7,
            LLCC68LoRaCodingRate::CR4of6,
            false,
        ).await?;
        self.set_frequency(self.frequency).await?;
        self.set_buffer_base_addresses(TX_BASE_ADDRESS, RX_BASE_ADDRESS).await?;
        self.set_output_power(TransmitPower::P14dBm).await?;
        self.set_dio1_interrupt(
            (LLCC68Interrupt::RxDone as u16) | (LLCC68Interrupt::CrcErr as u16),
            LLCC68Interrupt::RxDone as u16,
        ).await?;
        self.switch_to_rx().await?;

        // After the configuration we can treat the busy line a bit more relaxed,
        // it sometimes can go high seemingly incorrectly (or if mode change
        // fails), but during normal operation this should solve itself.
        self.ignore_busy = true;

        Ok(())
    }

    async fn command(
        &mut self,
        opcode: LLCC68OpCode,
        params: &[u8],
        response_len: usize,
    ) -> Result<Vec<u8, 64>, RadioError<SPI::Error>> {
        if self.busy.is_high().unwrap_or(false) && !self.ignore_busy {
            return Err(RadioError::Busy);
        }

        let mut payload = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();
        self.spi.transfer_in_place(&mut payload).await?;

        Ok(Vec::from_slice(&payload[(1 + params.len())..]).unwrap_or_default())
    }

    async fn read_register(&mut self, address: u16) -> Result<u8, RadioError<SPI::Error>> {
        Ok(self.command(LLCC68OpCode::ReadRegister, &address.to_be_bytes(), 2).await?[1])
    }

    async fn write_register(&mut self, address: u16, value: u8) -> Result<(), RadioError<SPI::Error>> {
        let buffer = [(address >> 8) as u8, address as u8, value];
        self.command(LLCC68OpCode::WriteRegister, &buffer, 0).await?;
        Ok(())
    }

    async fn set_packet_type(&mut self, packet_type: LLCC68PacketType) -> Result<(), RadioError<SPI::Error>> {
        self.command(LLCC68OpCode::SetPacketType, &[packet_type as u8], 0).await?;
        Ok(())
    }

    async fn set_lora_mod_params(
        &mut self,
        bandwidth: LLCC68LoRaModulationBandwidth,
        mut spreading_factor: LLCC68LoRaSpreadingFactor,
        coding_rate: LLCC68LoRaCodingRate,
        low_data_rate_optimization: bool,
    ) -> Result<(), RadioError<SPI::Error>> {
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
        ).await?;
        Ok(())
    }

    async fn set_lora_packet_params(
        &mut self,
        preamble_length: u16,
        fixed_length_header: bool,
        payload_length: u8,
        crc: bool,
        invert_iq: bool,
    ) -> Result<(), RadioError<SPI::Error>> {
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
        ).await?;
        Ok(())
    }

    async fn set_buffer_base_addresses(&mut self, tx_address: u8, rx_address: u8) -> Result<(), RadioError<SPI::Error>> {
        self.command(LLCC68OpCode::SetBufferBaseAddress, &[tx_address, rx_address], 0).await?;
        Ok(())
    }

    async fn set_dio1_interrupt(&mut self, irq_mask: u16, dio1_mask: u16) -> Result<(), RadioError<SPI::Error>> {
        self.command(
            LLCC68OpCode::SetDioIrqParams,
            &[(irq_mask >> 8) as u8, irq_mask as u8, (dio1_mask >> 8) as u8, dio1_mask as u8, 0, 0, 0, 0],
            0,
        ).await?;
        Ok(())
    }

    async fn set_tx_mode(&mut self, timeout_us: u32) -> Result<(), RadioError<SPI::Error>> {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command(
            LLCC68OpCode::SetTx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0
        ).await?;
        Ok(())
    }

    async fn set_rx_mode(&mut self, _timeout_us: u32) -> Result<(), RadioError<SPI::Error>> {
        let timeout = 0; // TODO
        self.command(
            LLCC68OpCode::SetRx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0,
        ).await?;
        Ok(())
    }

    pub async fn set_output_power(
        &mut self,
        output_power: TransmitPower,
        //ramp_time: LLCC68RampTime,
    ) -> Result<(), RadioError<SPI::Error>> {
        let (duty_cycle, hp_max) = match output_power {
            TransmitPower::P14dBm => (0x02, 0x02),
            TransmitPower::P17dBm => (0x02, 0x03),
            TransmitPower::P20dBm => (0x03, 0x05),
            TransmitPower::P22dBm => (0x04, 0x07),
        };
        self.command(LLCC68OpCode::SetPaConfig, &[duty_cycle, hp_max, 0x00, 0x01], 0).await?;
        self.command(LLCC68OpCode::SetTxParams, &[22, RAMP_TIME as u8], 0).await?;

        // workaround to prevent overly protective power clamping (chapter 15.2, p. 97)
        let tx_clamp_config = self.read_register(0x08d8).await?;
        self.write_register(0x08d8, tx_clamp_config | 0x1e).await?;

        Ok(())
    }

    pub async fn switch_to_rx(&mut self) -> Result<(), RadioError<SPI::Error>> {
        self.set_lora_packet_params(12, true, RX_PACKET_SIZE, true, false).await?;
        self.set_rx_mode(0).await?;
        Ok(())
    }

    pub async fn set_frequency(&mut self, frequency: u32) -> Result<(), RadioError<SPI::Error>> {
        const XTAL_FREQ: u32 = 32_000_000;
        const PLL_STEP_SHIFT_AMOUNT: u32 = 14;
        const PLL_STEP_SCALED: u32 = XTAL_FREQ >> (25 - PLL_STEP_SHIFT_AMOUNT);

        let int = frequency / PLL_STEP_SCALED;
        let frac = frequency / (int * PLL_STEP_SCALED);

        let pll = (int << PLL_STEP_SHIFT_AMOUNT) + ((frac << PLL_STEP_SHIFT_AMOUNT) + (PLL_STEP_SCALED >> 1)) / PLL_STEP_SCALED;

        let params = [(pll >> 24) as u8, (pll >> 16) as u8, (pll >> 8) as u8, pll as u8];
        self.command(LLCC68OpCode::SetRfFrequency, &params, 0).await?;
        self.frequency = frequency;
        Ok(())
    }

    pub async fn send(&mut self, msg: &[u8]) -> Result<(), RadioError<SPI::Error>> {
        if msg.len() > TX_PACKET_SIZE as usize {
            //error!("message exceeds PACKET_SIZE");
            return Ok(());
        }

        // The LLCC68 datasheet mentions this workaround to prevent modulation quality
        // issues with 500khz bandwidth. (chapter 15.1, p. 97)
        // This should be changed if we change bandwidths.
        let reg = self.read_register(0x0889).await?;
        if reg & 0xfb != reg {
            //info!("Applying LLCC68 mod quality workaround.");
            self.write_register(0x0889, reg & 0xfb).await?;
        }

        self.set_lora_packet_params(12, true, TX_PACKET_SIZE, true, false).await?;
        const CMD_SIZE: usize = (TX_PACKET_SIZE as usize) + 1;
        let mut params: [u8; CMD_SIZE] = [0x00; CMD_SIZE];
        params[0] = TX_BASE_ADDRESS;
        params[1..(msg.len()+1)].copy_from_slice(&msg);
        self.command(LLCC68OpCode::WriteBuffer, &params, 0).await?;
        self.set_tx_mode(TRANSMISSION_TIMEOUT_MS * 1000).await?;

        Ok(())
    }

    pub async fn receive(&mut self) -> Result<Option<Vec<u8, 64>>, RadioError<SPI::Error>> {
        // No RxDone interrupt, do nothing
        if !self.irq.is_high().unwrap() {
            return Ok(None);
        }

        // Get IRQ status to allow checking for CrcErr
        #[cfg(feature = "gcs")]
        let irq_status = self
            .command(LLCC68OpCode::GetIrqStatus, &[], 3).await
            .map(|r| ((r[1] as u16) << 8) + (r[2] as u16))
            .unwrap_or(0);

        self.command(LLCC68OpCode::ClearIrqStatus, &[0xff, 0xff], 0).await?;

        // Get the packet stats before the data, since this is useful even if the data is corrupted.
        // Sometimes the response data is shifted to the right for some reason, which is why we read
        // 5 bytes instead of the 4 we'd actually need.
        // e.g. [164, 0, 78, 51, 80] instead of [164, 78, 51, 80]
        let packet_status = self.command(LLCC68OpCode::GetPacketStatus, &[], 5).await?;
        let offset = if packet_status[1] == 0 { 1 } else { 0 };
        self.rssi = packet_status[1+offset];
        self.rssi_signal = packet_status[3+offset];
        self.snr = packet_status[2+offset] as i8;

        // Abort in case of a CRC mismatch
        // Sometimes this seems to trigger unnecessarily, especially for uplink
        // messages. Since those are HMACed pretty heavily anyways, we ignore
        // CRC mismatches for uplink messages.
        #[cfg(feature = "gcs")]
        if irq_status & (LLCC68Interrupt::CrcErr as u16) > 0 {
            return Err(RadioError::Crc);
        }

        // Get RX buffer status (this contains the length of the received data)
        let rx_buffer_status = self.command(LLCC68OpCode::GetRxBufferStatus, &[], 3).await?;
        let len = u8::min(rx_buffer_status[1], RX_PACKET_SIZE);

        // Read received data
        let buffer = self.command(
            LLCC68OpCode::ReadBuffer,
            &[rx_buffer_status[2]],
            len as usize + 1,
        ).await?;

        self.set_rx_mode(0).await?;

        if buffer.len() < UPLINK_PACKET_SIZE as usize {
            return Ok(None);
        }

        Ok(Some(buffer))
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
