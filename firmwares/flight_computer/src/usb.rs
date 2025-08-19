//! USB serial link implementation, handling regular telemetry, log messsages
//! and uplink commands.

use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::bind_interrupts;
use embassy_stm32::usb::Driver;
use embassy_stm32::{
    Peri,
    peripherals::{PA11, PA12, USB_OTG_FS},
    usb,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Channel, Receiver, Sender};
use embassy_time::{Duration, TimeoutError, Timer, with_timeout};
use embassy_usb::class::cdc_acm;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::{Builder, UsbDevice, driver::EndpointError};
use heapless::Vec;
use static_cell::StaticCell;

use shared_types::*;

static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 128]> = StaticCell::new();

static CDC_ACM_STATE: StaticCell<cdc_acm::State> = StaticCell::new();

static UPLINK_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, UplinkMessage, 3>> = StaticCell::new();
static DOWNLINK_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();
// static FLASH_DOWNLINK_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();

// already defined in lib.rs
// bind_interrupts!(struct Irqs {
//     OTG_FS => usb::InterruptHandler<USB_OTG_FS>;
// });

pub fn start(
    driver: Driver<'static, USB_OTG_FS>,
    spawner: Spawner,
) -> (
    Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) {
    let uplink = UPLINK_CHANNEL.init(Channel::new());
    let downlink = DOWNLINK_CHANNEL.init(Channel::new());

    let mut config = embassy_usb::Config::new(0x0483, 0x5740);
    config.manufacturer = Some("TUDSaT");
    config.product = Some("Sting FC"); // TODO
    config.serial_number = Some("12345678"); // TODO

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR_BUFFER.init([0; 256]),
        BOS_DESCRIPTOR_BUFFER.init([0; 256]),
        MSOS_DESCRIPTOR_BUFFER.init([0; 256]),
        CONTROL_BUFFER.init([0; 128]),
    );

    let class = CdcAcmClass::new(&mut builder, CDC_ACM_STATE.init(cdc_acm::State::new()), 64);
    // let (usb_sender, usb_receiver) = class.split();

    let usb = builder.build();

    spawner.spawn(run_usb(usb)).unwrap();

    spawner.spawn(usb_daemon(class, downlink.receiver(), uplink.sender())).unwrap();

    defmt::info!("Usb configuration completed");

    (downlink.sender(), uplink.receiver())
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, usb::Driver<'static, USB_OTG_FS>>) -> ! {
    defmt::info!("Started running usb driver.");
    usb.run().await
}

async fn write_message(
    class: &mut CdcAcmClass<'static, Driver<'static, USB_OTG_FS>>,
    serialized: &[u8],
) -> Result<(), EndpointError> {
    for chunk in serialized.chunks(64) {
        class.write_packet(chunk).await?;
    }

    if serialized.len() % 64 == 0 {
        class.write_packet(&[]).await?;
    }

    Ok(())
}

#[embassy_executor::task]
async fn usb_daemon(
    // mut class: Sender<'static, usb::Driver<'static, USB_OTG_FS>>,
    mut class: CdcAcmClass<'static, Driver<'static, USB_OTG_FS>>,
    downlink_receiver: Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    uplink_sender: Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    const UPLINK_BUFFER_SIZE: usize = 512;

    // NOTE: change to [] at some point
    let mut uplink_buffer: Vec<u8, UPLINK_BUFFER_SIZE> = Vec::new();
    // NOTE: enforce packet size
    let mut packet_buffer: [u8; 64] = [0; 64];

    loop {
        defmt::info!("Waiting for usb connection.");
        class.wait_connection().await;
        defmt::info!("Usb connection established");
        match select(downlink_receiver.receive(), class.read_packet(&mut packet_buffer)).await {
            Either::First(msg) => {
                defmt::info!("usb daemon: processing message");
                let serialized = msg.serialize().unwrap_or_default();

                match with_timeout(Duration::from_millis(10), write_message(&mut class, &serialized)).await {
                    Ok(Ok(())) => {}
                    Ok(Err(EndpointError::BufferOverflow)) => {
                        defmt::error!("buffer overflow");
                    }
                    Ok(Err(EndpointError::Disabled)) => {
                        defmt::error!("disabled");
                    }
                    Err(TimeoutError) => {
                        defmt::error!("timeout");
                        Timer::after(Duration::from_millis(1000)).await;
                        // Clear the queue
                        while downlink_receiver.try_receive().is_ok() {}
                    }
                }
            }

            Either::Second(res) => {
                // let n = if let Some(n) = res
                let Ok(n) = res else {
                    continue;
                };

                let packet = &packet_buffer[..usize::min(n, UPLINK_BUFFER_SIZE - uplink_buffer.len())];
                let _ = uplink_buffer.extend_from_slice(packet);

                if uplink_buffer.len() >= UPLINK_BUFFER_SIZE {
                    uplink_buffer.truncate(0);
                }

                match postcard::take_from_bytes_cobs::<UplinkMessage>(&mut uplink_buffer.clone()) {
                    Ok((msg, rest)) => {
                        uplink_buffer = Vec::from_slice(rest).unwrap_or_default();
                        uplink_sender.send(msg).await;
                    }
                    Err(_) => {
                        if uplink_buffer.contains(&0) {
                            uplink_buffer.truncate(0);
                        }
                    }
                }
            }
        }
    }
}
