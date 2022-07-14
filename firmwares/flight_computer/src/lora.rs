use core::ops::DerefMut;
use core::cell::RefCell;
use alloc::vec::Vec;
use alloc::sync::Arc;

use stm32f4xx_hal as hal;
use hal::spi::{Spi, Master, TransferModeNormal};
use hal::pac::SPI1;
use hal::gpio::{Pin, Output, Alternate};
use cortex_m::interrupt::{free, Mutex};
use embedded_hal::prelude::*;

use crate::telemetry::*;
use crate::log;

type RawSpi = Spi<SPI1, (
    Pin<'A', 5, Alternate<5>>,
    Pin<'B', 4, Alternate<5>>,
    Pin<'A', 7, Alternate<5>>,
), TransferModeNormal, Master>;
type SharedSpi = Arc<Mutex<RefCell<RawSpi>>>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RadioState {
    Init,
    Idle,
    Reading,
    Writing
}

pub struct LoRaRadio {
    state: RadioState,
    spi: SharedSpi,
    cs: Pin<'A', 1, Output>
}

// both RX and TX get half of the available 128 bytes
const TX_BASE_ADDRESS: u8 = 0;
const RX_BASE_ADDRESS: u8 = 128;

impl LoRaRadio {
    pub fn init(spi: SharedSpi, mut cs: Pin<'A', 1, Output>) -> Self {
        cs.set_high();
        Self {
            state: RadioState::Init,
            spi,
            cs
        }
    }

    fn command(&mut self, opcode: LLCC68OpCode, params: &[u8], response_len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut msg = Vec::with_capacity(response_len + params.len() + 1);
            msg.push(opcode as u8);
            msg.append(&mut params.to_vec());
            msg.append(&mut [0x00].repeat(response_len));

            self.cs.set_low();
            let response = spi.transfer(&mut msg)?;
            self.cs.set_high();

            let response = response[(1 + params.len())..].to_vec();
            if response.len() > 0 {
                log!(Info, "{:02x?}", response);
            }

            Ok(response)
        })
    }

    fn set_packet_type(&mut self, packet_type: LLCC68PacketType) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::SetPacketType, &[packet_type as u8], 0)?;
        Ok(())
    }

    fn freq_to_pll_steps(&self, freq: u32) -> u32 {
        const XTAL_FREQ: u32 = 32_000_000;
        const PLL_STEP_SHIFT_AMOUNT: u32 = 14;
        const PLL_STEP_SCALED: u32 = XTAL_FREQ >> (25 - PLL_STEP_SHIFT_AMOUNT);

        let int = freq / PLL_STEP_SCALED;
        let frac = freq / (int * PLL_STEP_SCALED);

        (int << PLL_STEP_SHIFT_AMOUNT) + ((frac << PLL_STEP_SHIFT_AMOUNT) + (PLL_STEP_SCALED >> 1)) / PLL_STEP_SCALED
    }

    fn set_rf_frequency(&mut self, frequency: u32) -> Result<(), hal::spi::Error> {
        let pll = self.freq_to_pll_steps(frequency);
        let params = [(pll >> 24) as u8, (pll >> 16) as u8, (pll >> 8) as u8, pll as u8];
        self.command(LLCC68OpCode::SetRfFrequency, &params, 0);
        Ok(())
    }

    fn set_output_power(&mut self, output_power: LLCC68OutputPower, ramp_time: LLCC68RampTime) -> Result<(), hal::spi::Error> {
        let (duty_cycle, hp_max) = match output_power {
            LLCC68OutputPower::P14dBm => (0x02, 0x02),
            LLCC68OutputPower::P17dBm => (0x02, 0x03),
            LLCC68OutputPower::P20dBm => (0x03, 0x05),
            LLCC68OutputPower::P22dBm => (0x04, 0x07)
        };
        self.command(LLCC68OpCode::SetPaConfig, &[duty_cycle, hp_max, 0x00, 0x01], 0)?;
        self.command(LLCC68OpCode::SetTxParams, &[output_power as u8, ramp_time as u8], 0)?;
        Ok(())
    }

    fn set_lora_mod_params(
        &mut self,
        bandwidth: LLCC68LoRaModulationBandwidth,
        spreading_factor: LLCC68LoRaSpreadingFactor,
        coding_rate: LLCC68LoRaCodingRate,
        low_data_rate_optimization: bool
    ) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::SetModulationParams, &[
            spreading_factor as u8,
            bandwidth as u8,
            coding_rate as u8,
            low_data_rate_optimization as u8
        ], 0)?;
        Ok(())
    }

    fn set_lora_packet_params(
        &mut self,
        preamble_length: u16,
        fixed_length_header: bool,
        payload_length: u8,
        crc: bool,
        invert_iq: bool
    ) -> Result<(), hal::spi::Error> {
        let preamble_length = u16::max(1, preamble_length);
        self.command(LLCC68OpCode::SetPacketParams, &[
            (preamble_length >> 8) as u8,
            preamble_length as u8,
            fixed_length_header as u8,
            payload_length,
            crc as u8,
            invert_iq as u8
        ], 0)?;
        Ok(())
    }

    fn set_buffer_base_addresses(&mut self, tx_address: u8, rx_address: u8) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::SetBufferBaseAddress, &[TX_BASE_ADDRESS, RX_BASE_ADDRESS], 0)?;
        Ok(())
    }

    fn set_sync_word(&mut self, sync_word: u64) -> Result<(), hal::spi::Error> {
        Ok(())
    }

    fn configure_tx(&mut self) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::GetStatus, &[], 1)?;
        self.set_packet_type(LLCC68PacketType::LoRa)?;
        self.set_rf_frequency(434_000_000)?;
        self.set_output_power(LLCC68OutputPower::P14dBm, LLCC68RampTime::R200U)?;
        self.set_lora_mod_params(
            LLCC68LoRaModulationBandwidth::Bw125,
            LLCC68LoRaSpreadingFactor::SF6,
            LLCC68LoRaCodingRate::CR46,
            false
        )?;
        self.set_buffer_base_addresses(TX_BASE_ADDRESS, RX_BASE_ADDRESS)?;

        // write data to buffer
        // set dio irq params
            // define sync word (write reg) (lora?)
        // settx
        // wait for irq txdone 
        // clear irq txdone
        Ok(())
    }

    fn set_tx_mode(&mut self, timeout_us: u32) -> Result<(), hal::spi::Error> {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command(LLCC68OpCode::SetTx, &[
            (timeout >> 16) as u8,
            (timeout >> 8) as u8,
            timeout as u8
        ], 0)?;
        Ok(())
    }

    fn send_packet(&mut self, msg: &[u8]) -> Result<(), hal::spi::Error> {
        self.set_lora_packet_params(16, false, msg.len() as u8, true, false)?;
        let mut params = Vec::with_capacity(msg.len() + 1);
        params.push(TX_BASE_ADDRESS);
        params.append(&mut msg.to_vec());
        self.command(LLCC68OpCode::WriteBuffer, &params, 0)?;
        self.set_tx_mode(5000);
        Ok(())
    }

    pub fn tick(&mut self, msg: Option<DownlinkMessage>) -> Option<UplinkMessage> {
        if self.state == RadioState::Init {
            self.configure_tx();
            self.state = RadioState::Idle;
        }

        if let Some(m) = msg {
            let serialized = m.wrap(0);
            self.send_packet(&serialized);
        }

        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    ResetStats = 0x00
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68PacketType {
    GFSK = 0x00,
    LoRa = 0x01,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68OutputPower {
    P14dBm = 14,
    P17dBm = 17,
    P20dBm = 20,
    P22dBm = 22
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68RampTime {
    R10u = 0x00,
    R20u = 0x01,
    R40u = 0x02,
    R80U = 0x03,
    R200U = 0x04,
    R800U = 0x05,
    R1700U = 0x06,
    R3400U = 0x07
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68LoRaModulationBandwidth {
    Bw125 = 0x04,
    Bw250 = 0x05,
    Bw500 = 0x06
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68LoRaSpreadingFactor {
    SF5 = 0x05,
    SF6 = 0x06,
    SF7 = 0x07,
    SF8 = 0x08,
    SF9 = 0x09,
    SF10 = 0x0a,
    SF11 = 0x0B
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LLCC68LoRaCodingRate {
    CR45 = 0x01,
    CR46 = 0x02,
    CR47 = 0x03,
    CR48 = 0x04
}
