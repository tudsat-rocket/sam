//! USB serial link implementation, handling regular telemetry, log messsages
//! and uplink commands.

//use alloc::vec::Vec;
//
//use hal::otg_fs::{UsbBus, USB};
//use stm32f4xx_hal as hal;
//
//use usb_device::class_prelude::*;
//use usb_device::prelude::*;
//use usbd_serial::{SerialPort, USB_CLASS_CDC};
//
//use crate::prelude::*;
//use crate::telemetry::*;
//
//static mut USB_BUFFER: [u32; 4096] = [0; 4096];
//static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;
//
//type ReadBufferStore = [u8; 128];
//type WriteBufferStore = [u8; 2048];

use embassy_executor::Spawner;
use embassy_stm32::{peripherals::{PA12, PA11, USB_OTG_FS}, usb_otg::{Driver, Instance}};
use embassy_stm32::bind_interrupts;
use embassy_sync::{channel::Channel, blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_time::{Timer, Duration, with_timeout, TimeoutError};
use embassy_usb::{Builder, class::cdc_acm::{CdcAcmClass, State, Sender, Receiver}, UsbDevice, driver::EndpointError};

use static_cell::{StaticCell, make_static};

use defmt::*;

use crate::telemetry::{DownlinkMessage, TelemetryMain, Transmit, UplinkMessage};

bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb_otg::InterruptHandler<USB_OTG_FS>;
});

pub struct UsbLink {
    //device: UsbDevice<'static, UsbBus<USB>>,
    //serial: SerialPort<'static, UsbBus<USB>, ReadBufferStore, WriteBufferStore>,
    //poll_counter: u32,
    //log_counter: u32,
    //uplink_buffer: Vec<u8>,
    //time: u32,
    //last_heartbeat: Option<u32>,
    downlink_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 10>,
}

impl UsbLink {
    pub async fn init(peripheral: USB_OTG_FS, pin_dm: PA12, pin_dp: PA11) -> Self {
        let mut config = embassy_stm32::usb_otg::Config::default();
        config.vbus_detection = false;

        let ep_out_buffer = &mut make_static!([0; 1024])[..];
        let driver = Driver::new_fs(peripheral, Irqs, pin_dm, pin_dp, ep_out_buffer, config);

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
            &mut make_static!([0; 256])[..],
            &mut make_static!([0; 256])[..],
            &mut make_static!([0; 256])[..],
            &mut make_static!([0; 128])[..]
        );

        let class = CdcAcmClass::new(&mut builder, make_static!(State::new()), 64);
        let (usb_sender, usb_receiver) = class.split();

        let usb = builder.build();

        let uplink_channel = make_static!(Channel::<CriticalSectionRawMutex, UplinkMessage, 10>::new());
        let downlink_channel = make_static!(Channel::<CriticalSectionRawMutex, DownlinkMessage, 10>::new());

        let spawner = Spawner::for_current_executor().await;
        spawner.spawn(run_usb(usb)).unwrap();
        spawner.spawn(handle_usb_downlink(usb_sender, downlink_channel.receiver())).unwrap();
        spawner.spawn(handle_usb_uplink(usb_receiver, uplink_channel.sender())).unwrap();

        Self {
            downlink_sender: downlink_channel.sender(),
        }
    }

    pub async fn send_downlink_message(&mut self, msg: DownlinkMessage) {
        let _ = self.downlink_sender.try_send(msg);
    }
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, Driver<'static, USB_OTG_FS>>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn handle_usb_downlink(
    mut class: Sender<'static, Driver<'static, USB_OTG_FS>>,
    downlink_receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 10>
) -> ! {
    loop {
        class.wait_connection().await;
        //info!("USB connected.");

        loop {
            //info!("USB disconnected.");
            //let msg = DownlinkMessage::TelemetryMain(TelemetryMain::default());
            let msg = downlink_receiver.receive().await; // TODO: replace with signal?

            let serialized = msg.serialize().unwrap_or_default();
            //debug!("sending {:?}", Debug2Format(&serialized));

            let res = with_timeout(Duration::from_millis(5), async {
                for chunk in serialized.chunks(64) {
                    let res = class.write_packet(chunk).await;
                    //info!("result: {:?}", Debug2Format(&res));
                }

                if serialized.len() % 64 == 0 {
                    let res = class.write_packet(&[]).await;
                    //info!("result: {:?}", Debug2Format(&res));
                }
            }).await;
            //info!("result: {:?}", Debug2Format(&res));

            if let Err(TimeoutError) = res {
                Timer::after(Duration::from_millis(10)).await;
                break;
            }

            //Timer::after(Duration::from_micros(10)).await;
        }
    }
}

#[embassy_executor::task]
async fn handle_usb_uplink(
    mut class: Receiver<'static, Driver<'static, USB_OTG_FS>>,
    uplink_sender: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, UplinkMessage, 10>
) -> ! {
    loop {
        class.wait_connection().await;

        let mut buffer: [u8; 64] = [0; 64];
        match class.read_packet(&mut buffer).await {
            Ok(n) => {
                //info!("received: {:?}", buffer[..n]);
            },
            Err(_e) => {
                continue;
            }
        }
    }
}

//impl UsbLink {
//    pub fn init(usb: USB) -> Self {
//        unsafe {
//            USB_BUS = Some(UsbBus::new(usb, &mut USB_BUFFER));
//        }
//
//        let mut serial = unsafe {
//            SerialPort::new_with_store(
//                USB_BUS.as_ref().unwrap(),
//                [0; 128],
//                [0; 2048]
//            )
//        };
//
//        let product = match (cfg!(feature = "gcs"), cfg!(feature = "rev2")) {
//            (false, false) => "Sting FC (rev. 1)",
//            (false, true)  => "Sting FC (rev. 2)",
//            (true, false)  => "Sting FC GCS (rev. 1)",
//            (true, true)   => "Sting FC GCS (rev. 2)"
//        };
//
//        let mut device = unsafe {
//            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x0483, 0x5740))
//                .manufacturer("TUDSat")
//                .product(product)
//                .device_class(USB_CLASS_CDC)
//                .max_packet_size_0(64)
//                .build()
//        };
//
//        device.poll(&mut [&mut serial]);
//
//        Self {
//            device,
//            serial,
//            poll_counter: 0,
//            log_counter: 0,
//            uplink_buffer: Vec::with_capacity(1024),
//            time: 0,
//            last_heartbeat: None,
//        }
//    }
//
//    pub fn poll(&mut self) -> bool {
//        self.device.poll(&mut [&mut self.serial])
//    }
//
//    fn send_data(&mut self, data: &[u8]) -> Result<(), UsbError> {
//        self.serial.write(data)?;
//        self.serial.flush()?;
//        Ok(())
//    }
//
//    pub fn send_message(&mut self, msg: DownlinkMessage) {
//        let serialized = msg.serialize().unwrap_or_default();
//        if let Err(_e) = self.send_data(&serialized) {
//            // Since USB doesn't seem to work, we only try to send this via SWD
//            //rtt_target::rprintln!("Failed to send data via USB: {:?}", e);
//        }
//    }
//
//    pub fn tick(&mut self, time: u32) -> Option<UplinkMessage> {
//        self.time = time;
//        self.poll_counter += 1;
//        self.log_counter += 1;
//
//        if self.poll_counter >= (MAIN_LOOP_FREQ_HERTZ / USB_POLL_FREQ_HERTZ) {
//            self.poll();
//            self.poll_counter = 0;
//        }
//
//        if self.log_counter >= (MAIN_LOOP_FREQ_HERTZ / USB_LOG_FREQ_HERTZ) {
//            if self.last_heartbeat.map(|t| self.time - t < 750).unwrap_or(false) {
//                if let Some((t, l, ll, m)) = Logger::next_usb() {
//                    self.send_message(DownlinkMessage::Log(t, l, ll, m));
//                }
//            }
//            self.log_counter = 0;
//        }
//
//        let mut buf: [u8; 128] = [0; 128];
//        let read = self.serial.read(&mut buf).unwrap_or(0);
//        self.uplink_buffer.extend(&buf[..read]);
//
//        if self.uplink_buffer.len() > 512 {
//            self.uplink_buffer.truncate(0);
//        }
//
//        match postcard::take_from_bytes_cobs(&mut self.uplink_buffer.clone()) {
//            Ok((msg, rest)) => {
//                self.uplink_buffer = rest.to_vec();
//
//                if msg == UplinkMessage::Heartbeat {
//                    self.last_heartbeat = Some(self.time);
//                }
//
//                Some(msg)
//            },
//            Err(_) => {
//                if self.uplink_buffer.iter().position(|b| *b == 0).is_some() {
//                    self.uplink_buffer.truncate(0);
//                }
//                None
//            }
//        }
//    }
//}
