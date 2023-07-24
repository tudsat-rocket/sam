use alloc::vec::Vec;
use alloc::vec;
use core::cell::RefCell;
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use embedded_hal::serial::Read;
use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::gpio::Pin;
use hal::pac::{interrupt, Interrupt, USART1};
use hal::rcc::Clocks;
use hal::serial::Serial;

use crc::{Crc, CRC_8_DVB_S2};

use crate::prelude::*;
use crate::telemetry::FlightMode;

pub enum RuncamCameraState {
    Standby,
    Recording,
    QrCodeReader,
    Unknown
}

pub enum RuncamCommand {
    GetDeviceInformation = 0x00,
    CameraControl = 0x01,
}

const FEATURE_SIMULATE_POWER_BUTTON: u16    = 1 << 0;
const FEATURE_SIMULATE_WIFI_BUTTON: u16     = 1 << 1;
const FEATURE_CHANGE_MODE: u16              = 1 << 2;
const FEATURE_SIMULATE_5_KEY_OSD_CABLE: u16 = 1 << 3;
const FEATURE_DEVICE_SETTINGS_ACCESS: u16   = 1 << 4;
const FEATURE_DISPLAYP_PORT: u16            = 1 << 5;
const FEATURE_START_RECORDING: u16          = 1 << 6;
const FEATURE_STOP_RECORDING: u16           = 1 << 7;

pub struct RuncamCameraInformation {
    protocol_version: u8,
    features: u16
}

pub enum RuncamCommandState {
    Idle,
    Running {
        command: RuncamCommand,
        param: Option<u8>,
        time: u32
    }
}

pub struct RuncamCamera {
    state: RuncamCameraState,
    command_state: RuncamCommandState,
    last_mode: FlightMode,
}

const BUFFER_SIZE: usize = 16;
const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

static UART: Mutex<RefCell<Option<Serial<USART1>>>> = Mutex::new(RefCell::new(None));
static mut BUFFER: Option<Vec<u8>> = None;

impl RuncamCamera {
    pub fn init(dp_uart: USART1, tx: Pin<'A', 9>, rx: Pin<'A', 10>, clocks: &Clocks) -> Self {
        let config = hal::serial::config::Config::default()
            .baudrate(115_200.bps())
            .parity_none();

        let mut uart: Serial<USART1, u8> = dp_uart.serial(
            (tx.into_alternate(), rx.into_alternate()),
            config,
            clocks
        ).unwrap();
        uart.listen(hal::serial::Event::Rxne);

        free(|cs| {
            *UART.borrow(cs).borrow_mut() = Some(uart);
        });

        unsafe {
            BUFFER.replace(Vec::with_capacity(BUFFER_SIZE));
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART1);
        }

        RuncamCamera {
            state: RuncamCameraState::Unknown,
            command_state: RuncamCommandState::Idle,
            last_mode: FlightMode::Idle,
        }
    }

    fn send_command(&mut self, command: RuncamCommand, param: Option<u8>) -> Result<(), hal::serial::Error> {
        let mut msg = if let Some(p) = param {
            vec![0xcc, command as u8, p]
        } else {
            vec![0xcc, command as u8]
        };

        msg.push(CRC8.checksum(&msg));
        log!(Debug, "{:02x?}", msg);

        let mut uart = free(|cs| UART.borrow(cs).replace(None)).unwrap();

        uart.bwrite_all(&msg)?;
        uart.bflush()?;

        free(|cs| {
            *UART.borrow(cs).borrow_mut() = Some(uart);
        });

        Ok(())
    }

    fn toggle_recording(&mut self) -> Result<(), hal::serial::Error> {
        self.send_command(RuncamCommand::CameraControl, Some(0x01))
    }

    pub fn tick(&mut self, time: u32, mode: FlightMode) {
        let response = free(|_cs| unsafe {
            let buffer = BUFFER.as_ref().unwrap();
            if buffer.len() >= 5 {
                let response = buffer[..5].to_vec();
                BUFFER.as_mut().unwrap().truncate(0);
                Some(response)
            } else {
                None
            }
        });

        if let Some(response) = response {
            log!(Debug, "{:02x?}", &response);
        }

        // Annoyingly, there doesn't seem to be a way to ask the camera about
        // its current state, so that button better not have been presssd...
        if (mode >= FlightMode::Armed && self.last_mode < FlightMode::Armed) ||
            (mode < FlightMode::Armed && self.last_mode >= FlightMode::Armed) {
            self.toggle_recording();
        }

        self.last_mode = mode;
    }
}

#[interrupt]
fn USART1() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::USART1);

    free(|cs| {
        if let Some(ref mut uart) = UART.borrow(cs).borrow_mut().deref_mut() {
            let buffer = unsafe { BUFFER.as_mut().unwrap() };

            while let Ok(b) = uart.read() {
                if buffer.len() > 0 || b == 0xcc as u8 {
                    buffer.push(b);
                }

                if buffer.len() >= BUFFER_SIZE {
                    buffer.truncate(0);
                }
            }
        }
    });
}
