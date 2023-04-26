//! USB serial link implementation, handling regular telemetry, log messsages
//! and uplink commands.

use alloc::vec::Vec;

use hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal as hal;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::prelude::*;
use crate::telemetry::*;

static mut USB_BUFFER: [u32; 4096] = [0; 4096];
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

type ReadBufferStore = [u8; 128];
type WriteBufferStore = [u8; 2048];

pub struct UsbLink {
    device: UsbDevice<'static, UsbBus<USB>>,
    serial: SerialPort<'static, UsbBus<USB>, ReadBufferStore, WriteBufferStore>,
    poll_counter: u32,
    log_counter: u32,
    uplink_buffer: Vec<u8>,
    time: u32,
    last_heartbeat: Option<u32>,
}

impl UsbLink {
    pub fn init(usb: USB) -> Self {
        unsafe {
            USB_BUS = Some(UsbBus::new(usb, &mut USB_BUFFER));
        }

        let mut serial = unsafe {
            SerialPort::new_with_store(
                USB_BUS.as_ref().unwrap(),
                [0; 128],
                [0; 2048]
            )
        };

        let product = match (cfg!(feature = "gcs"), cfg!(feature = "rev2")) {
            (false, false) => "Sting FC (rev. 1)",
            (false, true)  => "Sting FC (rev. 2)",
            (true, false)  => "Sting FC GCS (rev. 1)",
            (true, true)   => "Sting FC GCS (rev. 2)"
        };

        let mut device = unsafe {
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x0483, 0x5740))
                .manufacturer("TUDSat")
                .product(product)
                .device_class(USB_CLASS_CDC)
                .max_packet_size_0(64)
                .build()
        };

        device.poll(&mut [&mut serial]);

        Self {
            device,
            serial,
            poll_counter: 0,
            log_counter: 0,
            uplink_buffer: Vec::with_capacity(128),
            time: 0,
            last_heartbeat: None,
        }
    }

    pub fn poll(&mut self) -> bool {
        self.device.poll(&mut [&mut self.serial])
    }

    fn send_data(&mut self, data: &[u8]) -> Result<(), UsbError> {
        self.serial.write(data)?;
        self.serial.flush()?;
        Ok(())
    }

    pub fn send_message(&mut self, msg: DownlinkMessage) {
        let serialized = msg.serialize().unwrap_or_default();
        if let Err(_e) = self.send_data(&serialized) {
            // Since USB doesn't seem to work, we only try to send this via SWD
            //rtt_target::rprintln!("Failed to send data via USB: {:?}", e);
        }
    }

    pub fn tick(&mut self, time: u32) -> Option<UplinkMessage> {
        self.time = time;
        self.poll_counter += 1;
        self.log_counter += 1;

        if self.poll_counter >= (MAIN_LOOP_FREQ_HERTZ / USB_POLL_FREQ_HERTZ) {
            self.poll();
            self.poll_counter = 0;
        }

        if self.log_counter >= (MAIN_LOOP_FREQ_HERTZ / USB_LOG_FREQ_HERTZ) {
            if self.last_heartbeat.map(|t| self.time - t < 750).unwrap_or(false) {
                if let Some((t, l, ll, m)) = Logger::next_usb() {
                    self.send_message(DownlinkMessage::Log(t, l, ll, m));
                }
            }
            self.log_counter = 0;
        }

        let mut buf: [u8; 128] = [0; 128];
        let read = self.serial.read(&mut buf).unwrap_or(0);
        self.uplink_buffer.extend(&buf[..read]);

        if self.uplink_buffer.len() > UPLINK_MAX_LEN as usize {
            self.uplink_buffer.truncate(0);
        }

        match postcard::take_from_bytes_cobs(&mut self.uplink_buffer) {
            Ok((msg, rest)) => {
                self.uplink_buffer = rest.to_vec();

                if msg == UplinkMessage::Heartbeat {
                    self.last_heartbeat = Some(self.time);
                }

                Some(msg)
            },
            Err(_) => {
                if self.uplink_buffer.iter().position(|b| *b == 0).is_some() {
                    self.uplink_buffer.truncate(0);
                }
                None
            }
        }
    }
}
