use stm32f4xx_hal as hal;
use hal::rcc::Clocks;

use nalgebra::{Vector3, UnitQuaternion};
use ahrs::Ahrs;

use crate::telemetry::*;
use crate::usb::*;
use crate::bootloader::*;
use crate::sensors::*;
use crate::logging::*;
use crate::params::*;
use crate::lora::*;
use crate::flash::*;
use crate::buzzer::*;

pub struct Vehicle {
    clocks: Clocks,

    imu: Imu,
    acc: Accelerometer,
    compass: Compass,
    barometer: Barometer,
    gps: GPS,
    power: PowerMonitor,

    usb_link: UsbLink,
    radio: LoRaRadio,
    flash: Flash,
    buzzer: Buzzer,

    ahrs: ahrs::Madgwick<f32>,
    orientation: Option<UnitQuaternion<f32>>,

    pub time: u32,
    mode: FlightMode,
    loop_runtime: u16,
    usb_telem_counter: u16,
    lora_telem_counter: u16,
}

impl<'a> Vehicle {
    pub fn init(
        clocks: Clocks,
        usb_link: UsbLink,
        imu: Imu,
        acc: Accelerometer,
        compass: Compass,
        barometer: Barometer,
        gps: GPS,
        flash: Flash,
        radio: LoRaRadio,
        power: PowerMonitor,
        buzzer: Buzzer,
    ) -> Self {
        Self {
            clocks,

            imu,
            acc,
            compass,
            barometer,
            gps,
            power,

            usb_link,
            flash,
            radio,
            buzzer,

            ahrs: ahrs::Madgwick::new(1.0 / (MAIN_LOOP_FREQ_HERTZ as f32), MADGEWICK_BETA),
            orientation: None,

            time: 0,
            mode: FlightMode::Idle,
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
    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self) {
        let cycles_before = hal::pac::DWT::cycle_count();

        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        // TODO: sensor readings
        self.acc.tick();
        self.barometer.tick();
        self.gps.tick(self.time, &self.clocks);
        self.power.tick();

        self.imu.tick();
        self.compass.tick();

        let gyro_vector = self.imu.gyroscope().map(|v| Vector3::new(v.0, v.1, v.2));
        let acc_vector = self.imu.accelerometer().map(|v| Vector3::new(v.0, v.1, v.2));
        let mag_vector = self.compass.magnetometer().map(|v| Vector3::new(v.0, v.1, v.2));
        if gyro_vector.is_some() && acc_vector.is_some() && mag_vector.is_some() {
            self.orientation = self.ahrs.update(&gyro_vector.unwrap(), &acc_vector.unwrap(), &mag_vector.unwrap()).ok().map(|q| *q);
        } else {
            self.orientation = None;
        }

        if let Some(new_mode) = self.tick_mode() {
            self.mode = new_mode;
        }

        self.buzzer.tick(self.time);

        // TODO: write to flash, sd card
        // TODO: write to radio

        if let Some(msg) = self.radio.tick() {
            match msg {
                UplinkMessage::Heartbeat => {}
                UplinkMessage::Reboot => reboot(),
                UplinkMessage::RebootToBootloader => reboot_to_bootloader(),
            }
        }

        // Handle incoming USB messages
        if let Some(msg) = self.usb_link.tick(self.time) {
            match msg {
                UplinkMessage::Heartbeat => {}
                UplinkMessage::Reboot => reboot(),
                UplinkMessage::RebootToBootloader => reboot_to_bootloader(),
            }
        }

        self.flash.tick(self.time, None);

        if let Some(msg) = self.next_usb_telem() {
            self.usb_link.send_message(msg);
        }

        if let Some(msg) = self.next_lora_telem() {
            self.radio.send_message(msg);
        }

        self.usb_telem_counter = (self.usb_telem_counter + 1) % 1000;
        self.lora_telem_counter = (self.lora_telem_counter + 1) % 1000; // TODO: do we need only one?

        let cycles_elapsed = hal::pac::DWT::cycle_count().wrapping_sub(cycles_before);
        // TODO: maybe take the max over the last N iterations so we catch spikes?
        self.loop_runtime = (cycles_elapsed / CLOCK_FREQ_MEGA_HERTZ) as u16;
    }

    #[cfg(feature = "gcs")]
    pub fn tick(&mut self) {
        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        self.buzzer.tick(self.time);

        let downlink_msg = self.radio.tick();

        let uplink_msg = self.usb_link.tick(self.time).map(|msg| {
            match msg {
                UplinkMessage::Heartbeat => Some(UplinkMessage::Heartbeat),
                UplinkMessage::Reboot => { reboot(); None }, // TODO: passthrough reboot?
                UplinkMessage::RebootToBootloader => { reboot_to_bootloader(); None },
            }
        });

        if let Some((pc, msg)) = downlink_msg {
            log!(Debug, "{:?}", msg);
            self.usb_link.send_message(msg); // TODO: pass through pc
        }

        // TODO: uplink messages
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
        if self.lora_telem_counter % 1000 == 0 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else {
            None
        }

        //if self.lora_telem_counter % 100 == 0 {
        //    Some(DownlinkMessage::TelemetryMain(self.into()))
        //} else if self.lora_telem_counter % 200 == 5 {
        //    Some(DownlinkMessage::TelemetryState(self.into()))
        //} else if self.lora_telem_counter == 550 {
        //    Some(DownlinkMessage::TelemetryPower(self.into()))
        //} else if self.lora_telem_counter == 50 {
        //    Some(DownlinkMessage::TelemetryGPS(self.into()))
        //} else {
        //    None
        //}
    }
}

impl Into<TelemetryMain> for &Vehicle {
    fn into(self) -> TelemetryMain {
        TelemetryMain {
            time: self.time,
            mode: self.mode.clone(),
            orientation: self.orientation.clone(),
            ..Default::default()
        }
    }
}

impl Into<TelemetryState> for &Vehicle {
    fn into(self) -> TelemetryState {
        // TODO: send options?
        TelemetryState {
            time: self.time,
            orientation: self.orientation.clone(),
            gyroscope: self.imu.gyroscope().unwrap_or((0.0, 0.0, 0.0)),
            acceleration: self.imu.accelerometer().unwrap_or((0.0, 0.0, 0.0)),
            ..Default::default()
        }
    }
}

impl Into<TelemetryRawSensors> for &Vehicle {
    fn into(self) -> TelemetryRawSensors {
        TelemetryRawSensors {
            time: self.time,
            gyro: self.imu.gyroscope().unwrap_or((0.0, 0.0, 0.0)),
            accelerometer1: self.imu.accelerometer().unwrap_or((0.0, 0.0, 0.0)),
            accelerometer2: self.acc.accelerometer().unwrap_or((0.0, 0.0, 0.0)),
            magnetometer: self.compass.magnetometer().unwrap_or((0.0, 0.0, 0.0)),
            temperature_baro: self.barometer.temperature().unwrap_or(0.0),
            pressure_baro: self.barometer.pressure().unwrap_or(0.0),
            altitude_baro: self.barometer.altitude().unwrap_or(0.0),
        }
    }
}

impl Into<TelemetryPower> for &Vehicle {
    fn into(self) -> TelemetryPower {
        TelemetryPower {
            time: self.time,
            battery_voltage: self.power.battery_voltage().unwrap_or(0),
            arm_voltage: self.power.arm_voltage().unwrap_or(0),
            current: self.power.battery_current().unwrap_or(0),
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
            temperature_core: self.power.temperature().unwrap_or(0),
            cpu_voltage: self.power.cpu_voltage().unwrap_or(0)
        }
    }
}

impl Into<TelemetryGPS> for &Vehicle {
    fn into(self) -> TelemetryGPS {
        TelemetryGPS {
            time: self.time,
            fix: self.gps.fix.clone(),
            hdop: self.gps.hdop,
            num_satellites: self.gps.num_satellites,
            latitude: self.gps.latitude,
            longitude: self.gps.longitude,
            altitude_asl: self.gps.altitude
        }
    }
}
