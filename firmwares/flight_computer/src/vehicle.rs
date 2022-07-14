use stm32f4xx_hal as hal;

use crate::telemetry::*;
use crate::usb::*;
use crate::bootloader::*;
use crate::sensors::*;
use crate::logging::*;
use crate::params::*;
use crate::lora::*;
use crate::{log, log_every_nth_time};

pub struct Vehicle {
    usb_link: UsbLink,
    radio: LoRaRadio,
    gps: Result<GPS, hal::serial::Error>,
    pub time: u32,
    mode: FlightMode,
    loop_runtime: u16,
    usb_telem_counter: u16,
    lora_telem_counter: u16,
}

impl<'a> Vehicle {
    pub fn init(
        usb_link: UsbLink,
        gps: Result<GPS, hal::serial::Error>,
        radio: LoRaRadio
    ) -> Self {
        Self {
            usb_link,
            radio,
            mode: FlightMode::Idle,
            gps,
            time: 0,
            loop_runtime: 0,
            usb_telem_counter: 0,
            lora_telem_counter: 0,
        }
    }

    pub fn tick_mode(&mut self) -> Option<FlightMode> {
        match self.mode {
            FlightMode::Idle => {
                // TODO: Detect hardware arm
            },
            FlightMode::HardwareArmed => {
                // TODO: Detect software arm (however that looks like)
            },
            FlightMode::Armed => {
                // TODO: Detect liftoff
            },
            FlightMode::Flight => {
                // TODO: Detect apogee
            },
            FlightMode::RecoveryDrogue => {
                // TODO: Detect alt < MAIN_THRESHOLE
            },
            FlightMode::RecoveryMain => {
                // TODO: Detect landing
            },
            FlightMode::Landed => {
                // TODO: Buzz
            }
        }

        None
    }

    /// Called every MAIN_LOOP_FREQ_HERTZ Hz.
    pub fn tick(&mut self) {
        let cycles_before = hal::pac::DWT::cycle_count();

        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        // TODO: sensor readings
        if let Ok(gps) = self.gps.as_mut() {
            gps.tick();
        }

        if let Some(new_mode) = self.tick_mode() {
            self.mode = new_mode;
        }

        // TODO: write to flash, sd card
        // TODO: write to radio

        //if let Some(msg) = self.radio.tick(self.next_lora_telem()) {
        //    // TODO
        //}

        // Handle incoming USB messages
        if let Some(msg) = self.usb_link.tick(self.time, self.next_usb_telem()) {
            match msg {
                UplinkMessage::Heartbeat => {},
                UplinkMessage::Reboot => reboot(),
                UplinkMessage::RebootToBootloader => reboot_to_bootloader()
            }
        }

        let cycles_elapsed = hal::pac::DWT::cycle_count().wrapping_sub(cycles_before);
        self.loop_runtime = (cycles_elapsed / CLOCK_FREQ_MEGA_HERTZ) as u16;
        self.usb_telem_counter = (self.usb_telem_counter + 1) % 1000;
        self.lora_telem_counter = (self.lora_telem_counter + 1) % 1000; // TODO: do we need only one?
    }

    fn next_usb_telem(&self) -> Option<DownlinkMessage> {
        if self.usb_telem_counter % 20 == 0 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else if self.usb_telem_counter % 20 == 5 {
            Some(DownlinkMessage::TelemetryState(self.into()))
        } else if self.usb_telem_counter % 20 == 10 {
            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
        } else if self.usb_telem_counter % 200 == 2 {
            Some(DownlinkMessage::TelemetryPower(self.into()))
        } else if self.usb_telem_counter % 20 == 15 {
            Some(DownlinkMessage::TelemetryKalman(self.into()))
        } else if self.usb_telem_counter % 100 == 1 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if self.usb_telem_counter % 100 == 2 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else {
            None
        }
    }

    fn next_lora_telem(&self) -> Option<DownlinkMessage> {
        if self.lora_telem_counter % 100 == 0 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else if self.lora_telem_counter % 200 == 5 {
            Some(DownlinkMessage::TelemetryState(self.into()))
        } else if self.lora_telem_counter == 550 {
            Some(DownlinkMessage::TelemetryPower(self.into()))
        } else if self.lora_telem_counter == 50 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else {
            None
        }
    }
}

impl Into<TelemetryMain> for &Vehicle {
    fn into(self) -> TelemetryMain {
        TelemetryMain {
            time: self.time,
            mode: self.mode.clone(),
            orientation: (1.0, 0.0, 0.0, 0.0),
            ..Default::default()
        }
    }
}

impl Into<TelemetryState> for &Vehicle {
    fn into(self) -> TelemetryState {
        TelemetryState {
            time: self.time,
            orientation: (1.0, 0.0, 0.0, 0.0),
            ..Default::default()
        }
    }
}

impl Into<TelemetryRawSensors> for &Vehicle {
    fn into(self) -> TelemetryRawSensors {
        TelemetryRawSensors {
            time: self.time,
            ..Default::default()
        }
    }
}

impl Into<TelemetryPower> for &Vehicle {
    fn into(self) -> TelemetryPower {
        TelemetryPower {
            time: self.time,
            ..Default::default()
        }
    }
}

impl Into<TelemetryKalman> for &Vehicle {
    fn into(self) -> TelemetryKalman {
        TelemetryKalman {
            time: self.time,
            ..Default::default()
        }
    }
}

impl Into<TelemetryDiagnostics> for &Vehicle {
    fn into(self) -> TelemetryDiagnostics {
        TelemetryDiagnostics {
            time: self.time,
            loop_runtime: self.loop_runtime,
            ..Default::default()
        }
    }
}

impl Into<TelemetryGPS> for &Vehicle {
    fn into(self) -> TelemetryGPS {
        if let Ok(gps) = &self.gps {
            TelemetryGPS {
                time: self.time,
                fix: gps.fix.clone(),
                hdop: gps.hdop,
                num_satellites: gps.num_satellites,
                latitude: gps.latitude,
                longitude: gps.longitude,
                altitude_asl: gps.altitude
            }
        } else {
            TelemetryGPS {
                time: self.time,
                ..Default::default()
            }
        }
    }
}
