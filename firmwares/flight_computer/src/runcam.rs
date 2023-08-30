use alloc::vec;

use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::gpio::Pin;
use hal::pac::USART1;
use hal::rcc::Clocks;
use hal::serial::Serial;

use crc::{Crc, CRC_8_DVB_S2};

use crate::prelude::*;
use crate::telemetry::FlightMode;

#[allow(unused)]
pub enum RuncamCommand {
    GetDeviceInformation = 0x00,
    CameraControl = 0x01,
}

//const FEATURE_SIMULATE_POWER_BUTTON: u16    = 1 << 0;
//const FEATURE_SIMULATE_WIFI_BUTTON: u16     = 1 << 1;
//const FEATURE_CHANGE_MODE: u16              = 1 << 2;
//const FEATURE_SIMULATE_5_KEY_OSD_CABLE: u16 = 1 << 3;
//const FEATURE_DEVICE_SETTINGS_ACCESS: u16   = 1 << 4;
//const FEATURE_DISPLAYP_PORT: u16            = 1 << 5;
//const FEATURE_START_RECORDING: u16          = 1 << 6;
//const FEATURE_STOP_RECORDING: u16           = 1 << 7;

#[allow(unused)]
pub struct RuncamCameraInformation {
    protocol_version: u8,
    features: u16
}

pub struct RuncamCamera {
    uart: Serial<USART1, u8>,
    last_mode: FlightMode,
}

const CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

impl RuncamCamera {
    pub fn init(dp_uart: USART1, tx: Pin<'A', 9>, rx: Pin<'A', 10>, clocks: &Clocks) -> Self {
        let config = hal::serial::config::Config::default()
            .baudrate(115_200.bps())
            .parity_none();

        let uart: Serial<USART1, u8> = dp_uart.serial(
            (tx.into_alternate(), rx.into_alternate()),
            config,
            clocks
        ).unwrap();

        RuncamCamera {
            uart,
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

        self.uart.bwrite_all(&msg)?;
        self.uart.bflush()?;

        Ok(())
    }

    fn toggle_recording(&mut self) -> Result<(), hal::serial::Error> {
        self.send_command(RuncamCommand::CameraControl, Some(0x01))
    }

    pub fn tick(&mut self, mode: FlightMode) {
        // Annoyingly, there doesn't seem to be a way to ask the camera about
        // its current state, so that button better not have been presssd...
        if (mode >= FlightMode::Armed && self.last_mode < FlightMode::Armed) ||
            (mode < FlightMode::Armed && self.last_mode >= FlightMode::Armed) {
            if let Err(e) = self.toggle_recording() {
                log!(Error, "Failed to toggle recording: {:?}", e);
            }
        }

        self.last_mode = mode;
    }
}
