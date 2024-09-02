//! USB serial link implementation, handling regular telemetry, log messsages
//! and uplink commands.

use alloc::vec::Vec;
use embassy_executor::Spawner;
use embassy_stm32::{peripherals::{PA12, PA11, USB_OTG_FS}, usb_otg::Driver};
use embassy_stm32::bind_interrupts;
use embassy_sync::channel::{self, Channel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Timer, Duration, TimeoutError, with_timeout};
use embassy_usb::{driver::EndpointError, Builder, UsbDevice};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State, Sender, Receiver};
use static_cell::StaticCell;

use shared_types::*;

static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUFFER: StaticCell<[u8; 128]> = StaticCell::new();

static CDC_ACM_STATE: StaticCell<State> = StaticCell::new();

static UPLINK_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, UplinkMessage, 3>> = StaticCell::new();
static DOWNLINK_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();
static FLASH_DOWNLINK_CHANNEL: StaticCell<Channel::<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();

bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb_otg::InterruptHandler<USB_OTG_FS>;
});

/// Main USB handle used by vehicle to send telemetry and receive commands
pub struct UsbHandle {
    downlink_sender: channel::Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    uplink_receiver: channel::Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
}

/// Handle given to flash implementation to allow faster flash download
pub struct FlashUsbHandle {
    downlink_sender: channel::Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
}

impl UsbHandle {
    pub async fn init(peripheral: USB_OTG_FS, pin_dm: PA12, pin_dp: PA11) -> (UsbHandle, FlashUsbHandle) {
        let uplink_channel = UPLINK_CHANNEL.init(Channel::new());
        let downlink_channel = DOWNLINK_CHANNEL.init(Channel::new());
        let flash_downlink_channel = FLASH_DOWNLINK_CHANNEL.init(Channel::new());

        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = false;

        let driver = Driver::new_fs(peripheral, Irqs, pin_dm, pin_dp, EP_OUT_BUFFER.init([0; 256]), config);

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

        let class = CdcAcmClass::new(&mut builder, CDC_ACM_STATE.init(State::new()), 64);
        let (usb_sender, usb_receiver) = class.split();

        let usb = builder.build();

        let spawner = Spawner::for_current_executor().await;
        spawner.spawn(run_usb(usb)).unwrap();
        spawner.spawn(handle_usb_downlink(usb_sender, downlink_channel.receiver(), flash_downlink_channel.receiver())).unwrap();
        spawner.spawn(handle_usb_uplink(usb_receiver, uplink_channel.sender())).unwrap();

        let usb_handle = Self {
            downlink_sender: downlink_channel.sender(),
            uplink_receiver: uplink_channel.receiver(),
        };

        let flash_usb_handle = FlashUsbHandle {
            downlink_sender: downlink_channel.sender(),
        };

        (usb_handle, flash_usb_handle)
    }

    pub fn send_message(&mut self, msg: DownlinkMessage) {
        if let Err(_e) = self.downlink_sender.try_send(msg) {
            //defmt::error!("Failed to send USB message.");
        }
    }

    pub fn next_uplink_message(&mut self) -> Option<UplinkMessage> {
        self.uplink_receiver.try_receive().ok()
    }
}

impl FlashUsbHandle {
    pub async fn send_message(&mut self, msg: DownlinkMessage) {
        let _ = self.downlink_sender.try_send(msg);
    }
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>) -> ! {
    usb.run().await
}

async fn write_message(class: &mut Sender<'static, Driver<'static, USB_OTG_FS>>, serialized: &[u8]) -> Result<(), EndpointError> {
    for chunk in serialized.chunks(64) {
        class.write_packet(chunk).await?;
    }

    if serialized.len() % 64 == 0 {
        class.write_packet(&[]).await?;
    }

    Ok(())
}

#[embassy_executor::task]
async fn handle_usb_downlink(
    mut class: Sender<'static, Driver<'static, USB_OTG_FS>>,
    downlink_receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    flash_downlink_receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>
) -> ! {
    loop {
        class.wait_connection().await;

        let msg = if let Ok(msg) = downlink_receiver.try_receive() {
            msg
        } else if let Ok(msg) = flash_downlink_receiver.try_receive() {
            msg
        } else {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        };

        let serialized = msg.serialize().unwrap_or_default();

        match with_timeout(Duration::from_millis(10), write_message(&mut class, &serialized)).await {
            Ok(Ok(())) => {}
            Ok(Err(EndpointError::BufferOverflow)) => {
                defmt::error!("buffer overflow");
            },
            Ok(Err(EndpointError::Disabled)) => {
                defmt::error!("disabled");
            }
            Err(TimeoutError) => {
                defmt::error!("timeout");
                Timer::after(Duration::from_millis(1000)).await;
                // Clear the queue
                while let Ok(_) = downlink_receiver.try_receive() {}
            }
        }
    }
}

#[embassy_executor::task]
async fn handle_usb_uplink(
    mut class: Receiver<'static, Driver<'static, USB_OTG_FS>>,
    uplink_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>
) -> ! {
    const UPLINK_BUFFER_SIZE: usize = 512;

    let mut uplink_buffer: Vec<u8> = Vec::with_capacity(UPLINK_BUFFER_SIZE);
    let mut packet_buffer: [u8; 64] = [0; 64];

    loop {
        class.wait_connection().await;

        match class.read_packet(&mut packet_buffer).await {
            Ok(n) => {
                uplink_buffer.extend(&packet_buffer[..usize::min(n, UPLINK_BUFFER_SIZE - uplink_buffer.len())]);

                if uplink_buffer.len() >= UPLINK_BUFFER_SIZE {
                    uplink_buffer.truncate(0);
                }

                match postcard::take_from_bytes_cobs::<UplinkMessage>(&mut uplink_buffer.clone()) {
                    Ok((msg, rest)) => {
                        uplink_buffer = rest.to_vec();
                        uplink_sender.send(msg).await;
                    },
                    Err(_) => {
                        if uplink_buffer.iter().position(|b| *b == 0).is_some() {
                            uplink_buffer.truncate(0);
                        }
                    }
                }
            },
            Err(_e) => {
                continue;
            }
        }
    }
}
