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

use static_cell::make_static;

use defmt::*;

use crate::drivers::can::MCP2517FD;

pub struct Can<SPI> {
    driver: MCP2517FD<SPI>,
    sender: Sender<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
    //receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
}

pub struct CanHandle {
    receiver: Receiver<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
    //sender: Sender<'static, CriticalSectionRawMutex, (u16, [u8; 8]), 5>,
}

type SpiInst = Spi<'static, SPI2, DMA1_CH4, DMA1_CH3>;
type CanInst = Can<SpiDeviceImpl<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB12>>>;

#[embassy_executor::task]
pub async fn run(mut flash: CanInst) -> ! {
    flash.run().await
}

impl CanHandle {
    pub fn receive(&mut self) -> Option<(u16, [u8; 8])> {
        self.receiver.try_receive().ok()
    }
}

impl<SPI: SpiDevice<u8>> Can<SPI> {
    pub async fn init(spi: SPI) -> Result<(Can<SPI>, CanHandle), SPI::Error> {
        //let outgoing_channel = make_static!(Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 5>::new());
        let incoming_channel = make_static!(Channel::<CriticalSectionRawMutex, (u16, [u8; 8]), 5>::new());

        let mcp = MCP2517FD::init(spi).await?;

        let can = Self {
            driver: mcp,
            sender: incoming_channel.sender(),
            //receiver: outgoing_channel.receiver(),
        };

        let handle = CanHandle {
            receiver: incoming_channel.receiver(),
            //sender: outgoing_channel.sender(),
        };

        info!("MCP2517FD initalized.");

        Ok((can, handle))
    }

    async fn run(&mut self) -> ! {
        loop {
            let (id, msg) = match self.driver.receive().await {
                Ok(Some((id, msg))) => (id, msg),
                Ok(None) => {
                    Timer::after(Duration::from_micros(1000)).await;
                    continue;
                }
                Err(e) => {
                    error!("Failed to read from CAN controller: {:?}", Debug2Format(&e));
                    Timer::after(Duration::from_micros(1000)).await;
                    continue;
                }
            };

            if let Err(e) = self.sender.try_send((id, msg)) {
                error!("Failed to send CAN message along: {:?}", Debug2Format(&e));
            }
        }
    }
}
