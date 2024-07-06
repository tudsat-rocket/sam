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

use crate::drivers::can::{MCP2517FD, MCP2517Error};

pub struct Can<SPI> {
    driver: MCP2517FD<SPI>,
    sender: Sender<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
    receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
}

pub struct CanHandle {
    receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
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
type CanInst = Can<SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB12>>>;

static INCOMING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 5>> = StaticCell::new();
static OUTGOING_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 5>> = StaticCell::new();

#[embassy_executor::task]
pub async fn run(mut flash: CanInst) -> ! {
    flash.run().await
}

impl CanHandle {
    pub fn receive(&mut self) -> Option<(u16, [u8; 8])> {
        self.receiver.try_receive().ok()
    }

    pub fn transmit(&mut self, id: u16, msg: [u8; 8]) {
        if let Err(_e) = self.sender.try_send((id, msg)) {
            error!("Failed to enqueue CAN msg.");
        }
    }
}

impl<SPI: SpiDevice<u8>> Can<SPI> {
    pub async fn init(spi: SPI, data_rate: CanDataRate) -> Result<(Can<SPI>, CanHandle), MCP2517Error<SPI::Error>> {
        let incoming_channel = INCOMING_CHANNEL.init(Channel::new());
        let outgoing_channel = OUTGOING_CHANNEL.init(Channel::new());

        let mcp = MCP2517FD::init(spi, data_rate).await?;

        let can = Self {
            driver: mcp,
            sender: incoming_channel.sender(),
            receiver: outgoing_channel.receiver(),
        };

        let handle = CanHandle {
            receiver: incoming_channel.receiver(),
            sender: outgoing_channel.sender(),
        };

        Ok((can, handle))
    }

    async fn run(&mut self) -> ! {
        loop {
            while let  Ok((id, msg)) = self.receiver.try_receive() {
                if let Err(e) = self.driver.transmit(id, msg).await {
                    error!("Failed to transmit CAN msg: {:?}", Debug2Format(&e));
                }
            }

            let (id, msg) = match self.driver.receive().await {
                Ok(Some((id, msg))) => (id, msg),
                Ok(None) => {
                    Timer::after(Duration::from_micros(1000)).await;
                    continue;
                }
                Err(e) => {
                    error!("Failed to receive CAN msg: {:?}", Debug2Format(&e));
                    Timer::after(Duration::from_micros(1000)).await;
                    continue;
                }
            };

            if let Err(e) = self.sender.try_send((id, msg)) {
                error!("Failed to pass received CAN msg along: {:?}", Debug2Format(&e));
            }
        }
    }
}
