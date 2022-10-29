use alloc::vec::Vec;
use core::num::Wrapping;

use hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal as hal;
//use hal::pac::{interrupt, Interrupt, TIM2};
//use cortex_m::interrupt::{free, Mutex};

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::prelude::*;
use crate::telemetry::*;

static mut USB_BUFFER: [u32; 1024] = [0; 1024];
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

pub struct UsbLink {
    device: UsbDevice<'static, UsbBus<USB>>,
    serial: SerialPort<'static, UsbBus<USB>>,
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

        let mut serial = unsafe { SerialPort::new(USB_BUS.as_ref().unwrap()) };
        let mut device = unsafe {
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x0483, 0x5740))
                .product("AthenaFC")
                .device_class(USB_CLASS_CDC)
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

    pub fn send_message(&mut self, msg: DownlinkMessage) {
        let wrapped = msg.wrap();

        // USB only allows packet sizes up to 64 bytes
        for chunk in wrapped.chunks(64) {
            self.serial.write(&chunk);
            self.serial.flush();
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
        for i in 0..self.serial.read(&mut buf).unwrap_or(0) {
            if self.uplink_buffer.len() > 0 || buf[0] == 0x42 {
                self.uplink_buffer.push(buf[i]);
            }
        }

        if self.uplink_buffer.len() > 2 {
            if let Ok(msg) = postcard::from_bytes::<UplinkMessage>(&self.uplink_buffer[2..]) {
                self.uplink_buffer.truncate(0);

                if msg == UplinkMessage::Heartbeat {
                    self.last_heartbeat = Some(self.time);
                }

                return Some(msg);
            } else if self.uplink_buffer[2] > 10 || self.uplink_buffer.len() > (self.uplink_buffer[2] as usize + 2) {
                self.uplink_buffer.truncate(0);
            }
        }

        None
    }
}

//#[interrupt]
//fn OTG_FS() {
//    free(|cs| {
//        cortex_m::peripheral::NVIC::unpend(Interrupt::OTG_FS);
//
//        if let Some(ref mut logger) = USB_LOGGER.borrow(cs).borrow_mut().deref_mut() {
//            logger.handle_interrupt();
//        }
//    });
//}
