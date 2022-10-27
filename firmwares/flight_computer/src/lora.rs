use alloc::sync::Arc;
use alloc::vec::Vec;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use embedded_hal::prelude::*;
use hal::dma::config::DmaConfig;
use hal::dma::traits::{Stream, StreamISR};
use hal::dma::{MemoryToPeripheral, Stream3, StreamX, StreamsTuple, Transfer};
use hal::gpio::{Alternate, Input, Output, Pin};
use hal::interrupt::{DMA2_STREAM3, DMA2_STREAM4};
use hal::pac::{interrupt, Interrupt, DMA1, DMA2, NVIC, SPI1};
use hal::spi::{Master, Spi, TransferModeNormal, Tx};
use stm32f4xx_hal as hal;

use crate::prelude::*;
use crate::telemetry::*;

// both RX and TX get half of the available 256 bytes
const TX_BASE_ADDRESS: u8 = 3;
const RX_BASE_ADDRESS: u8 = 64;
const DMA_BUFFER_SIZE: usize = 32;

type RawSpi = Spi<
    SPI1,
    (
        Pin<'A', 5, Alternate<5>>,
        Pin<'B', 4, Alternate<5>>,
        Pin<'A', 7, Alternate<5>>,
    ),
    TransferModeNormal,
    Master,
>;
type SharedSpi = Arc<Mutex<RefCell<RawSpi>>>;
type CsPin = Pin<'A', 1, Output>;
type IrqPin = Pin<'C', 0, Input>;
type BusyPin = Pin<'C', 1, Input>;

type SpiDmaTransfer = Transfer<
    Stream3<DMA2>, // TODO
    3,
    Tx<SPI1>,
    MemoryToPeripheral,
    &'static mut [u8; DMA_BUFFER_SIZE],
>;

static DMA_TRANSFER: Mutex<RefCell<Option<SpiDmaTransfer>>> = Mutex::new(RefCell::new(None));

static DMA_RESULT: Mutex<RefCell<Option<Result<(), ()>>>> = Mutex::new(RefCell::new(None));

static mut DMA_BUFFER: [u8; DMA_BUFFER_SIZE] = [0; DMA_BUFFER_SIZE];

#[derive(PartialEq, Eq)]
#[allow(dead_code)] // TODO
enum RadioState {
    Init,
    Idle,
    Writing,
    Transmitting,
    Reading,
}

pub struct LoRaRadio {
    state: RadioState,
    spi: SharedSpi,
    cs: CsPin,
    irq: IrqPin,
    busy: BusyPin,
}

#[interrupt]
fn DMA2_STREAM3() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::DMA2_STREAM3);

    free(|cs| {
        let mut transfer = &mut *DMA_TRANSFER.borrow(cs).borrow_mut();

        if let Some(ref mut transfer) = transfer.as_mut() {
            transfer.clear_fifo_error_interrupt();
            transfer.clear_transfer_complete_interrupt();
        }

        let result = Some(Ok(())); // TODO: properly track errors
        *DMA_RESULT.borrow(cs).borrow_mut() = result;
    })
}

impl LoRaRadio {
    pub fn init(spi: SharedSpi, cs: CsPin, irq: IrqPin, busy: BusyPin, dma_streams: StreamsTuple<DMA2>) -> Self {
        free(|cs| {
            let mut ref_mut = spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let stream = dma_streams.3;
            let tx = unsafe {
                let spi_copy = core::ptr::read(spi);
                let tx = spi_copy.use_dma().tx();
                tx
            };

            let mut transfer = unsafe {
                Transfer::init_memory_to_peripheral(
                    stream,
                    tx,
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
        });

        Self {
            state: RadioState::Init,
            spi,
            cs,
            irq,
            busy,
        }
    }

    fn command(
        &mut self,
        opcode: LLCC68OpCode,
        params: &[u8],
        response_len: usize,
    ) -> Result<Vec<u8>, hal::spi::Error> {
        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            let mut msg = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

            self.cs.set_low();

            // At 10MHz SPI freq, the setup commands seem to require some
            // extra delay between cs going low and the SCK going high
            // TODO: make this prettier
            for _i in 0..400 {
                core::hint::spin_loop()
            }

            let response = spi.transfer(&mut msg);
            self.cs.set_high();

            Ok(response?[(1 + params.len())..].to_vec())
        })
    }

    fn command_dma(&mut self, opcode: LLCC68OpCode, params: &[u8]) -> () {
        if params.len() > DMA_BUFFER_SIZE - 1 {
            return;
        }

        self.cs.set_low();

        // At 10MHz SPI freq, the setup commands seem to require some
        // extra delay between cs going low and the SCK going high
        // TODO: make this prettier
        for _i in 0..=200 {
            core::hint::spin_loop()
        }

        free(|cs| {
            let mut msg = [&[opcode as u8], params, &[0].repeat(DMA_BUFFER_SIZE - params.len() - 1)].concat();

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
        self.command(LLCC68OpCode::SetRfFrequency, &params, 0)?;
        Ok(())
    }

    fn set_output_power(
        &mut self,
        output_power: LLCC68OutputPower,
        ramp_time: LLCC68RampTime,
    ) -> Result<(), hal::spi::Error> {
        let (duty_cycle, hp_max) = match output_power {
            LLCC68OutputPower::P14dBm => (0x02, 0x02),
            LLCC68OutputPower::P17dBm => (0x02, 0x03),
            LLCC68OutputPower::P20dBm => (0x03, 0x05),
            LLCC68OutputPower::P22dBm => (0x04, 0x07),
        };
        self.command(LLCC68OpCode::SetPaConfig, &[duty_cycle, hp_max, 0x00, 0x01], 0)?;
        self.command(LLCC68OpCode::SetTxParams, &[output_power as u8, ramp_time as u8], 0)?;
        //self.command(LLCC68OpCode::SetTxParams, &[0, ramp_time as u8], 0)?;
        Ok(())
    }

    fn set_lora_mod_params(
        &mut self,
        bandwidth: LLCC68LoRaModulationBandwidth,
        mut spreading_factor: LLCC68LoRaSpreadingFactor,
        coding_rate: LLCC68LoRaCodingRate,
        low_data_rate_optimization: bool,
    ) -> Result<(), hal::spi::Error> {
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
    ) -> Result<(), hal::spi::Error> {
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

    fn set_buffer_base_addresses(&mut self, tx_address: u8, rx_address: u8) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::SetBufferBaseAddress, &[tx_address, rx_address], 0)?;
        Ok(())
    }

    fn configure_common(&mut self) -> Result<(), hal::spi::Error> {
        self.command(LLCC68OpCode::GetStatus, &[], 1)?;
        self.set_packet_type(LLCC68PacketType::LoRa)?;
        self.set_rf_frequency(868_000_000)?;
        self.set_lora_mod_params(
            LLCC68LoRaModulationBandwidth::Bw500,
            LLCC68LoRaSpreadingFactor::SF5,
            //LLCC68LoRaSpreadingFactor::SF8,
            LLCC68LoRaCodingRate::CR4of6,
            //LLCC68LoRaCodingRate::CR4of8,
            //true,
            false,
        )?;
        // TODO: use larger TX buffer on vehicle, since downlink msgs > uplink msgs ?
        self.set_buffer_base_addresses(TX_BASE_ADDRESS, RX_BASE_ADDRESS)?;
        Ok(())
    }

    fn set_dio1_interrupt(&mut self, irq_mask: u16, dio1_mask: u16) -> Result<(), hal::spi::Error> {
        self.command(
            LLCC68OpCode::SetDioIrqParams,
            &[(irq_mask >> 8) as u8, irq_mask as u8, (dio1_mask >> 8) as u8, dio1_mask as u8, 0, 0, 0, 0],
            0,
        )?;
        Ok(())
    }

    fn configure_tx(&mut self) -> Result<(), hal::spi::Error> {
        self.configure_common()?;
        self.set_output_power(LLCC68OutputPower::P14dBm, LLCC68RampTime::R800U)?;
        Ok(())
    }

    fn configure_rx(&mut self) -> Result<(), hal::spi::Error> {
        self.configure_common()?;
        self.set_dio1_interrupt(
            (LLCC68Interrupt::RxDone as u16) | (LLCC68Interrupt::CrcErr as u16),
            LLCC68Interrupt::RxDone as u16,
        )?;

        self.set_lora_packet_params(10, false, 0xff, true, false)?; // TODO
        self.set_rx_mode(0);

        Ok(())
    }

    fn set_tx_mode(&mut self, timeout_us: u32) -> Result<(), hal::spi::Error> {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command(
            LLCC68OpCode::SetTx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0,
        )?;
        Ok(())
    }

    fn set_tx_mode_dma(&mut self, timeout_us: u32) -> Result<(), hal::spi::Error> {
        let timeout = ((timeout_us as f32) / 15.625) as u32;
        self.command_dma(
            LLCC68OpCode::SetTx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
        );
        Ok(())
    }

    fn set_rx_mode(&mut self, timeout_us: u32) -> Result<(), hal::spi::Error> {
        let timeout = 0;
        self.command(
            LLCC68OpCode::SetRx,
            &[(timeout >> 16) as u8, (timeout >> 8) as u8, timeout as u8],
            0,
        )?;
        Ok(())
    }

    fn send_packet(&mut self, msg: &[u8]) -> Result<(), hal::spi::Error> {
        if self.state != RadioState::Idle {
            return Ok(()); // TODO
        }

        log!(Debug, "{:02x?}", msg);
        self.set_lora_packet_params(10, false, msg.len() as u8, true, false)?;
        let mut params = Vec::with_capacity(msg.len() + 1);
        params.push(TX_BASE_ADDRESS);
        params.append(&mut msg.to_vec());
        self.command_dma(LLCC68OpCode::WriteBuffer, &params);
        self.state = RadioState::Writing;
        Ok(())
    }

    pub fn send_message(&mut self, msg: DownlinkMessage) {
        let serialized = msg.wrap(0); // TODO
        if let Err(e) = self.send_packet(&serialized) {
            log!(Error, "Error sending LoRa packet: {:?}", e);
        }
    }

    pub fn receive_message(&mut self) -> Option<(u16, DownlinkMessage)> {
        // No RxDone interrupt, do nothing
        if !self.irq.is_high() {
            return None;
        }

        // Get IRQ status to allow checking for CrcErr
        let irq_status = self
            .command(LLCC68OpCode::GetIrqStatus, &[], 3)
            .map(|r| ((r[1] as u16) << 8) + (r[2] as u16))
            .unwrap_or(0);

        self.command(LLCC68OpCode::ClearIrqStatus, &[0xff, 0xff], 0);

        log!(Debug, "{:?}", irq_status);

        if irq_status & (LLCC68Interrupt::CrcErr as u16) > 0 {
            log!(Error, "CRC Error.");
        }

        // get rx buffer status
        let rx_buffer_status = self.command(LLCC68OpCode::GetRxBufferStatus, &[], 3).unwrap(); // TODO

        let buffer = self
            .command(
                LLCC68OpCode::ReadBuffer,
                &[rx_buffer_status[2]],
                rx_buffer_status[1] as usize + 1,
            )
            .unwrap();

        log!(Debug, "{:02x?}", buffer);

        self.set_rx_mode(0);

        DownlinkMessage::read_valid(&buffer[1..])
    }

    fn check_dma_result(&mut self) -> Option<Result<(), ()>> {
        let result = free(|cs| DMA_RESULT.borrow(cs).replace(None));
        if result.is_some() {
            self.cs.set_high();
        }
        result
    }

    fn tick_common(&mut self) {
        if self.state == RadioState::Writing {
            if let Some(_result) = self.check_dma_result() {
                self.set_tx_mode_dma(5000);
                self.state = RadioState::Transmitting;
            }
        } else if self.state == RadioState::Transmitting {
            if let Some(_result) = self.check_dma_result() {
                self.state = RadioState::Idle;
            }
        }
    }

    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self) -> Option<UplinkMessage> {
        self.tick_common();

        if self.state == RadioState::Init {
            if let Err(e) = self.configure_tx() {
                log!(Error, "Error configuring LoRa transceiver: {:?}", e);
            } else {
                self.state = RadioState::Idle;
            }
        }

        None // TODO
    }

    #[cfg(feature = "gcs")]
    pub fn tick(&mut self) -> Option<(u16, DownlinkMessage)> {
        self.tick_common();

        if self.state == RadioState::Init {
            if let Err(e) = self.configure_rx() {
                log!(Error, "Error configuring LoRa transceiver: {:?}", e);
            } else {
                self.state = RadioState::Idle;
            }
        }

        self.receive_message()
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
    R10u = 0x00,
    R20u = 0x01,
    R40u = 0x02,
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
    SF11 = 0x0B,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LLCC68LoRaCodingRate {
    CR4of5 = 0x01,
    CR4of6 = 0x02,
    CR4of7 = 0x03,
    CR4of8 = 0x04,
}
