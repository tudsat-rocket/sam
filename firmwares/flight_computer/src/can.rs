//! Driver for the MCP2517FD CAN controller.
//!
//! Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2517FD-External-CAN-FD-Controller-with-SPI-Interface-20005688B.pdf

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SpiDeviceImpl;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use static_cell::StaticCell;

use defmt::*;

use shared_types::can::*;

use crate::drivers::can::*;

pub struct CanTx<SPI: 'static> {
    driver: &'static Mutex<CriticalSectionRawMutex, MCP2517FD<SPI>>,
    receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 3>,
}

pub struct CanRx<SPI: 'static> {
    driver: &'static Mutex<CriticalSectionRawMutex, MCP2517FD<SPI>>,
    sender: Sender<'static, CriticalSectionRawMutex, FcReceivedCanBusMessage, 3>,
}

pub struct CanHandle {
    receiver: Receiver<'static, CriticalSectionRawMutex, FcReceivedCanBusMessage, 3>,
    sender: Sender<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 3>,
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
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

static DRIVER_SHARED: StaticCell<Mutex<CriticalSectionRawMutex, MCP2517FD<SpiDeviceImplInst>>> = StaticCell::new();

static INCOMING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, FcReceivedCanBusMessage, 3>> = StaticCell::new();
static OUTGOING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 3>> = StaticCell::new();

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

#[cfg(not(feature="gcs"))]
impl<SPI: SpiDevice<u8>> CanTx<SPI> {
    async fn run(&mut self) -> ! {
        loop {
            let (id, msg) = self.receiver.receive().await;
            if let Err(e) = self.driver.lock().await.transmit(id, msg).await {
                error!("Failed to transmit CAN msg: {:?}", Debug2Format(&e));
            }
        }
    }
}

#[cfg(not(feature="gcs"))]
impl<SPI: SpiDevice<u8>> CanRx<SPI> {
    async fn run(&mut self) -> ! {
        loop {
            let res = {
                let mut driver = self.driver.lock().await;
                driver.try_receive().await
            };

            let (id, msg) = match res {
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
                CanBusMessageId::IoBoardInput(role, 0xf) => {
                    let Ok(Some(parsed)) = IoBoardPowerMessage::parse(msg) else {
                        error!("Malformed message");
                        continue;
                    };

                    FcReceivedCanBusMessage::IoBoardPower(role, parsed)
                }
                CanBusMessageId::IoBoardInput(role, id) => {
                    let Ok(Some(parsed)) = IoBoardSensorMessage::parse(msg) else {
                        error!("Malformed message");
                        continue;
                    };

                    FcReceivedCanBusMessage::IoBoardSensor(role, id, parsed)
                }
                CanBusMessageId::FinBoardInput(fin, id) => {
                    let Ok(Some(parsed)) = FinBoardDataMessage::parse(msg) else {
                        error!("Malformed message");
                        continue;
                    };

                    FcReceivedCanBusMessage::FinBoardData(fin, id, parsed)
                }
                CanBusMessageId::BatteryBoardInput(id) => {
                    let Ok(Some(parsed)) = BatteryTelemetryMessage::parse(msg) else {
                        error!("Malformed message");
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

pub async fn init(
    spi: SpiDeviceImplInst,
    data_rate: CanDataRate
) -> (CanTx<SpiDeviceImplInst>, CanRx<SpiDeviceImplInst>, CanHandle) {
    let incoming_channel = INCOMING_CHANNEL.init(Channel::new());
    let outgoing_channel = OUTGOING_CHANNEL.init(Channel::new());

    let mcp = MCP2517FD::init(spi, data_rate).await.unwrap();
    let mcp = Mutex::new(mcp);
    let mcp = DRIVER_SHARED.init(mcp);

    let can_tx = CanTx {
        driver: mcp,
        receiver: outgoing_channel.receiver(),
    };

    let can_rx = CanRx {
        driver: mcp,
        sender: incoming_channel.sender(),
    };

    let handle = CanHandle {
        receiver: incoming_channel.receiver(),
        sender: outgoing_channel.sender(),
    };

    (can_tx, can_rx, handle)
}

#[cfg(not(feature="gcs"))]
#[embassy_executor::task]
pub async fn run_tx(mut can_tx: CanTxInst) -> ! {
    can_tx.run().await
}

#[cfg(not(feature="gcs"))]
#[embassy_executor::task]
pub async fn run_rx(mut can_rx: CanRxInst) -> ! {
    can_rx.run().await
}
