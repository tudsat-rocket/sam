//! Driver for the MCP2517FD CAN controller.
//!
//! Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2517FD-External-CAN-FD-Controller-with-SPI-Interface-20005688B.pdf

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SpiDeviceImpl;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use static_cell::StaticCell;

use defmt::*;

use shared_types::can::*;

use crate::drivers::can::*;

pub struct CanTx<SPI> {
    driver: MCP2517FDTx<SPI>,
    receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
}

pub struct CanRx<SPI> {
    driver: MCP2517FDRx<SPI>,
    sender: Sender<'static, CriticalSectionRawMutex, FcReceivedCanBusMessage, 5>,
}

pub struct CanHandle {
    receiver: Receiver<'static, CriticalSectionRawMutex, FcReceivedCanBusMessage, 5>,
    sender: Sender<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
}

#[allow(dead_code)]
pub enum CanDataRate {
    Kbps125,
    Kbps250,
    Kbps500,
    Kbps1000,
}

type SpiInst = Spi<'static, SPI2, DMA1_CH4, DMA1_CH3>;
type SpiDeviceImplInst = SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB12>>;
type CanTxInst = CanTx<SpiDeviceImplInst>;
type CanRxInst = CanRx<SpiDeviceImplInst>;

static INCOMING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, FcReceivedCanBusMessage, 5>> = StaticCell::new();
static OUTGOING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 5>> = StaticCell::new();

impl CanHandle {
    pub fn receive(&mut self) -> Option<FcReceivedCanBusMessage> {
        self.receiver.try_receive().ok()
    }

    pub fn transmit(&mut self, id: u16, msg: [u8; 8]) {
        if let Err(_e) = self.sender.try_send((id, msg)) {
            error!("Failed to enqueue CAN msg.");
        }
    }
}

impl<SPI: SpiDevice<u8>> CanTx<SPI> {
    async fn run(&mut self) -> ! {
        loop {
            let (id, msg) = self.receiver.receive().await;
            if let Err(e) = self.driver.transmit(id, msg).await {
                error!("Failed to transmit CAN msg: {:?}", Debug2Format(&e));
            }
        }
    }
}

impl<SPI: SpiDevice<u8>> CanRx<SPI> {
    async fn run(&mut self) -> ! {
        loop {
            let (id, msg) = match self.driver.try_receive().await {
                Ok(Some((id, msg))) => (id, msg),
                Ok(None) => {
                    Timer::after(Duration::from_micros(100)).await;
                    continue;
                }
                Err(e) => {
                    error!("Failed to receive CAN msg: {:?}", Debug2Format(&e));
                    continue;
                }
            };

            let message_id = match CanBusMessageId::try_from(id) {
                Ok(id) => id,
                Err(e) => {
                    error!("Unknown CAN bus message id: {}", e);
                    continue;
                }
            };

            let received_message = match message_id {
                //CanBusMessageId::IoBoardInput(_role, _id) => {
                //}
                //CanBusMessageId::FinBoardInput(_fin, _id) => {
                //}
                CanBusMessageId::BatteryBoardInput(id) => {
                    let Ok(Some(parsed)) = BatteryTelemetryMessage::parse(msg) else {
                        error!("Malformed battery telemetry message");
                        continue;
                    };

                    FcReceivedCanBusMessage::BatteryTelemetry(id, parsed)
                }
                m_id => {
                    error!("Unsupported CAN bus message id: {}", Debug2Format(&m_id));
                    continue;
                }
            };

            self.sender.send(received_message).await;
        }
    }
}

pub async fn init<SPI: SpiDevice<u8>>(spi: SPI, data_rate: CanDataRate) -> Result<(CanTx<SPI>, CanRx<SPI>, CanHandle), MCP2517Error<SPI::Error>> {
    let incoming_channel = INCOMING_CHANNEL.init(Channel::new());
    let outgoing_channel = OUTGOING_CHANNEL.init(Channel::new());

    let (mcp_tx, mcp_rx) = MCP2517FD::init(spi, data_rate).await?;

    let can_tx = CanTx {
        driver: mcp_tx,
        receiver: outgoing_channel.receiver(),
    };

    let can_rx = CanRx {
        driver: mcp_rx,
        sender: incoming_channel.sender(),
    };

    let handle = CanHandle {
        receiver: incoming_channel.receiver(),
        sender: outgoing_channel.sender(),
    };

    Ok((can_tx, can_rx, handle))
}

#[embassy_executor::task]
pub async fn run_tx(mut can_tx: CanTxInst) -> ! {
    can_tx.run().await
}

#[embassy_executor::task]
pub async fn run_rx(mut can_rx: CanRxInst) -> ! {
    can_rx.run().await
}
