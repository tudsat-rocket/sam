use hal::rcc::Clocks;
use stm32f4xx_hal as hal;

#[cfg(not(feature = "gcs"))]
use ahrs::Ahrs;
use nalgebra::UnitQuaternion;
#[cfg(not(feature = "gcs"))]
use nalgebra::Vector3;

#[allow(unused_imports)]
use crate::prelude::*;

use crate::bootloader::*;
use crate::buzzer::*;
use crate::flash::*;
use crate::logging::*;
use crate::lora::*;
use crate::params::*;
use crate::sensors::*;
use crate::telemetry::*;
use crate::usb::*;

#[cfg_attr(feature = "gcs", allow(dead_code))]
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

    ahrs: ahrs::Mahony<f32>,
    orientation: Option<UnitQuaternion<f32>>,

    pub time: u32,
    mode: FlightMode,
    loop_runtime: u16,
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

            ahrs: ahrs::Mahony::new(1.0 / (MAIN_LOOP_FREQ_HERTZ as f32), MAHONY_KP, MAHONY_KI),
            orientation: None,

            time: 0,
            mode: FlightMode::Idle,
            loop_runtime: 0,
        }
    }

    pub fn tick_mode(&mut self) -> Option<FlightMode> {
        match self.mode {
            FlightMode::Idle => {
                // TODO: Detect hardware arm
            }
            FlightMode::HardwareArmed => {
                // TODO: Detect software arm (however that looks like)
            }
            FlightMode::Armed => {
                // TODO: Detect liftoff
            }
            FlightMode::Flight => {
                // TODO: Detect apogee
            }
            FlightMode::RecoveryDrogue => {
                // TODO: Detect alt < MAIN_THRESHOLE
            }
            FlightMode::RecoveryMain => {
                // TODO: Detect landing
            }
            FlightMode::Landed => {
                // TODO: Buzz
            }
        }

        None
    }

    fn handle_uplink_message(&mut self, msg: UplinkMessage) {
        match msg {
            UplinkMessage::Heartbeat => {}
            UplinkMessage::Reboot => reboot(),
            UplinkMessage::RebootToBootloader => reboot_to_bootloader(),
        }
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

        self.flash.tick(self.time, None);

        let gyro_vector = self.imu.gyroscope().map(|v| Vector3::new(v.0, v.1, v.2));
        let acc_vector = self.imu.accelerometer().map(|v| Vector3::new(v.0, v.1, v.2));
        let mag_vector = self.compass.magnetometer().map(|v| Vector3::new(v.0, v.1, v.2));
        if gyro_vector.is_some() && acc_vector.is_some() && mag_vector.is_some() {
            self.orientation = self
                .ahrs
                .update(&gyro_vector.unwrap(), &acc_vector.unwrap(), &mag_vector.unwrap())
                .ok()
                .map(|q| *q);
        } else {
            self.orientation = None;
        }

        // TODO: write to flash, sd card

        // Handle incoming messages
        if let Some(msg) = self.radio.tick(self.time) {
            self.handle_uplink_message(msg);
        }
        if let Some(msg) = self.usb_link.tick(self.time) {
            self.handle_uplink_message(msg);
        }

        if let Some(new_mode) = self.tick_mode() {
            self.mode = new_mode;
        }

        self.buzzer.tick(self.time);

        if let Some(msg) = self.next_usb_telem() {
            self.usb_link.send_message(msg);
        }
        if let Some(msg) = self.next_lora_telem() {
            self.radio.send_downlink_message(msg);
        }

        let cycles_elapsed = hal::pac::DWT::cycle_count().wrapping_sub(cycles_before);
        // TODO: maybe take the max over the last N iterations so we catch spikes?
        self.loop_runtime = (cycles_elapsed / CLOCK_FREQ_MEGA_HERTZ) as u16;
    }

    #[cfg(feature = "gcs")]
    pub fn tick(&mut self) {
        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        let downlink_msg = self.radio.tick(self.time);
        let uplink_msg = self.usb_link.tick(self.time).and_then(|msg| {
            match msg {
                UplinkMessage::Heartbeat => None,
                UplinkMessage::Reboot => Some(UplinkMessage::Reboot),
                UplinkMessage::RebootToBootloader => {
                    reboot_to_bootloader();
                    None
                }
            }
        });

        self.buzzer.tick(self.time);

        if let Some(msg) = uplink_msg {
            self.radio.queue_uplink_message(msg);
        }

        if let Some(msg) = downlink_msg {
            let gcs_message = DownlinkMessage::TelemetryGCS(TelemetryGCS {
                time: msg.time(),
                lora_rssi: self.radio.rssi,
                lora_rssi_signal: self.radio.rssi_signal,
                lora_snr: self.radio.snr,
            });

            self.usb_link.send_message(msg);
            self.usb_link.send_message(gcs_message);
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_usb_telem(&self) -> Option<DownlinkMessage> {
        if self.time % 20 == 0 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else if self.time % 20 == 10 {
            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
        } else if self.time % 100 == 1 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if self.time % 100 == 2 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else {
            None
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_lora_telem(&self) -> Option<DownlinkMessage> {
        if self.time % 1000 == 0 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else if (self.time % 1000) % 160 == 80 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if (self.time % 1000) % 80 == 40 {
            Some(DownlinkMessage::TelemetryMainCompressed(self.into()))
        } else if (self.time % 1000) % 40 == 20 {
            Some(DownlinkMessage::TelemetryRawSensorsCompressed(self.into()))
        } else {
            None
        }
    }
}

// TODO: move these trait implementations to telemetry.rs?

impl Into<TelemetryMain> for &Vehicle {
    fn into(self) -> TelemetryMain {
        TelemetryMain {
            time: self.time,
            mode: self.mode.clone(),
            orientation: self.orientation.clone(),
            altitude_baro: self.barometer.altitude().unwrap_or(0.0),
            ..Default::default()
        }
    }
}

impl Into<TelemetryMainCompressed> for &Vehicle {
    fn into(self) -> TelemetryMainCompressed {
        let quat = self.orientation.clone().map(|q| q.coords).map(|q| {
            (
                (127.0 + q.x * 127.0) as u8,
                (127.0 + q.y * 127.0) as u8,
                (127.0 + q.z * 127.0) as u8,
                (127.0 + q.w * 127.0) as u8,
            )
        });
        TelemetryMainCompressed {
            time: self.time,
            mode: self.mode.clone(),
            orientation: quat.unwrap_or((127, 127, 127, 127)),
            altitude_baro: (self.barometer.altitude().unwrap_or(0.0) * 10.0) as u16, // TODO: this limits us to 6km AMSL
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
        }
    }
}

impl Into<TelemetryRawSensorsCompressed> for &Vehicle {
    fn into(self) -> TelemetryRawSensorsCompressed {
        let gyro = self.imu.gyroscope().unwrap_or((0.0, 0.0, 0.0));
        let acc1 = self.imu.accelerometer().unwrap_or((0.0, 0.0, 0.0));
        let acc2 = self.acc.accelerometer().unwrap_or((0.0, 0.0, 0.0));
        let mag = self.compass.magnetometer().unwrap_or((0.0, 0.0, 0.0));
        TelemetryRawSensorsCompressed {
            time: self.time,
            gyro: ((gyro.0 * 10.0).into(), (gyro.1 * 10.0).into(), (gyro.2 * 10.0).into()),
            accelerometer1: (
                (acc1.0 * 100.0).into(),
                (acc1.1 * 100.0).into(),
                (acc1.2 * 100.0).into(),
            ),
            accelerometer2: ((acc2.0 * 10.0).into(), (acc2.1 * 10.0).into(), (acc2.2 * 10.0).into()),
            magnetometer: ((mag.0 * 10.0).into(), (mag.1 * 10.0).into(), (mag.2 * 10.0).into()),
            temperature_baro: (self.barometer.temperature().unwrap_or(0.0) * 2.0) as i8,
            pressure_baro: (self.barometer.pressure().unwrap_or(0.0) * 10.0) as u16,
        }
    }
}

impl Into<TelemetryDiagnostics> for &Vehicle {
    fn into(self) -> TelemetryDiagnostics {
        TelemetryDiagnostics {
            time: self.time,
            loop_runtime: self.loop_runtime,
            temperature_core: self.power.temperature().unwrap_or(0),
            cpu_voltage: self.power.cpu_voltage().unwrap_or(0),
            battery_voltage: self.power.battery_voltage().unwrap_or(0),
            arm_voltage: self.power.arm_voltage().unwrap_or(0),
            current: self.power.battery_current().unwrap_or(0),
            lora_rssi: self.radio.rssi,
            ..Default::default()
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
            altitude_asl: self.gps.altitude,
        }
    }
}

impl Into<TelemetryGCS> for &Vehicle {
    fn into(self) -> TelemetryGCS {
        TelemetryGCS {
            time: self.time,
            lora_rssi: self.radio.rssi,
            lora_rssi_signal: self.radio.rssi_signal,
            lora_snr: self.radio.snr,
        }
    }
}
