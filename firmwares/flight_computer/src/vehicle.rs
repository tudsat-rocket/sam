//! Main flight logic for flight computer. State estimation and ground control station
//! stuff could maybe be moved out of here.

use alloc::collections::VecDeque;

use stm32f4xx_hal as hal;
use hal::gpio::Analog;
use hal::gpio::Input;
use hal::pac::{SPI1, SPI2, SPI3};
use hal::rcc::Clocks;
use hal::spi::Spi;
use hal::gpio::{Pin, Output};

#[allow(unused_imports)]
use crate::prelude::*;

use crate::bootloader::*;
#[cfg(feature = "rev2")]
use crate::can::MCP2517FD;
use crate::buzzer::*;
use crate::logging::*;
use crate::lora::*;
use crate::params::*;
use crate::sensors::*;
use crate::settings::*;
use crate::state_estimation::StateEstimator;
use crate::telemetry::*;
use crate::usb::*;
use crate::runcam::*;

const RUNTIME_HISTORY_LEN: usize = 200;

type LEDs = (Pin<'C',13,Output>, Pin<'C',14,Output>, Pin<'C',15,Output>);
type Recovery = (Pin<'C', 8, Output>, Pin<'C', 9, Output>);

type Spi1 = Spi<SPI1, false, u8>;
type Spi2 = Spi<SPI2, false, u8>;
type Spi3 = Spi<SPI3, false, u8>;

#[cfg(feature = "rev1")]
type Accelerometer = ADXL375<Spi3, Pin<'D', 2, Output>>;
#[cfg(feature = "rev2")]
type Accelerometer = H3LIS331DL<Spi1, Pin<'A', 4, Output>>;

#[cfg(feature = "rev1")]
type Compass = BMM150<Spi1, Pin<'B', 14, Output>>;
#[cfg(feature = "rev2")]
type Compass = LIS3MDL<Spi1, Pin<'B', 10, Output>>;

#[cfg(feature = "rev1")]
type Power = PowerMonitor<Pin<'C', 5, Analog>, Pin<'C', 4, Analog>, Pin<'A', 4, Analog>>;
#[cfg(feature = "rev2")]
type Power = PowerMonitor<Pin<'B', 0, Analog>, Pin<'C', 5, Analog>, Pin<'C', 4, Analog>>;

#[cfg(feature = "rev1")]
type Flash = crate::flash::Flash<Spi2, Pin<'B', 12, Output>>;
#[cfg(feature = "rev2")]
type Flash = crate::flash::Flash<Spi3, Pin<'D', 2, Output>>;

#[derive(Default, Clone)]
struct RecoveryState {
    pressure_cartridge: u16,
    pressure_chamber: u16,
    temperature: f32
}

impl From<[u8; 8]> for RecoveryState {
    fn from(msg: [u8; 8]) -> Self {
        Self {
            pressure_cartridge: u16::from_le_bytes([msg[0], msg[1]]),
            pressure_chamber: u16::from_le_bytes([msg[2], msg[3]]),
            temperature: u16::from_le_bytes([msg[4], msg[5]]) as f32,
        }
    }
}

impl Into<[u8; 2]> for RecoveryState {
    fn into(self) -> [u8; 2] {
        let pressure_cartridge = self.pressure_cartridge as u8;
        let pressure_chamber = self.pressure_chamber as u8;
        [pressure_cartridge, pressure_chamber]
    }
}

#[cfg_attr(feature = "gcs", allow(dead_code))]
pub struct Vehicle {
    clocks: Clocks,
    // sensors
    imu: Imu<Spi1, Pin<'B', 15, Output>>,
    acc: Accelerometer,
    compass: Compass,
    barometer: Barometer<Spi1, Pin<'C', 6, Output>>,
    gps: GPS,
    power: Power,
    // other peripherals
    usb_link: UsbLink,
    radio: LoRaRadio<Spi1, Pin<'A', 1, Output>, Pin<'C', 0, Input>, Pin<'C', 1, Input>>,
    flash: Flash,
    #[cfg(feature = "rev2")]
    can: MCP2517FD<Spi2, Pin<'B', 12, Output>>,
    // outputs
    leds: LEDs,
    buzzer: Buzzer,
    recovery: Recovery,
    runcam: RuncamCamera,
    // vehicle state
    pub time: u32,
    state_estimator: StateEstimator,
    mode: FlightMode,
    loop_runtime_history: VecDeque<u16>,
    settings: Settings,
    data_rate: TelemetryDataRate,
    recovery_drogue: Option<RecoveryState>,
    recovery_main: Option<RecoveryState>,
}

impl Vehicle {
    #[rustfmt::skip]
    pub fn init(
        clocks: Clocks,
        usb_link: UsbLink,
        mut imu: Imu<Spi1, Pin<'B', 15, Output>>,
        mut acc: Accelerometer,
        mut compass: Compass,
        barometer: Barometer<Spi1, Pin<'C', 6, Output>>,
        gps: GPS,
        mut flash: Flash,
        mut radio: LoRaRadio<Spi1, Pin<'A', 1, Output>, Pin<'C', 0, Input>, Pin<'C', 1, Input>>,
        #[cfg(feature = "rev2")]
        can: MCP2517FD<Spi2, Pin<'B', 12, Output>>,
        power: Power,
        leds: LEDs,
        buzzer: Buzzer,
        recovery: Recovery,
        runcam: RuncamCamera,
    ) -> Self {
        // Use this to reset settings without ground station during development.
        //flash.write_settings(&Settings::default()).unwrap();

        #[cfg(not(feature="gcs"))]
        let settings = {
            match flash.read_settings() {
                Ok(settings) => settings,
                Err(e) => {
                    log!(Error, "Failed to read settings: {:?}, reverting to defaults.", e);
                    Settings::default()
                }
            }
        };

        #[cfg(not(feature="gcs"))]
        log!(Info, "Loaded Settings: {:#?}", settings);

        #[cfg(feature="gcs")]
        let settings = Settings::default();

        radio.apply_settings(&settings.lora);
        imu.set_offsets(settings.gyro_offset, settings.acc_offset);
        acc.set_offset(settings.acc2_offset);
        compass.set_offset(settings.mag_offset);
        let data_rate = settings.default_data_rate;

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
            #[cfg(feature = "rev2")]
            can,
            leds,
            buzzer,
            recovery,
            runcam,

            time: 0,
            state_estimator: StateEstimator::new(MAIN_LOOP_FREQ_HERTZ as f32, settings.clone()),
            mode: FlightMode::Idle,
            loop_runtime_history: VecDeque::with_capacity(RUNTIME_HISTORY_LEN),
            settings,
            data_rate,

            recovery_drogue: None,
            recovery_main: None,
        }
    }

    fn switch_mode(&mut self, new_mode: FlightMode) {
        if new_mode == self.mode {
            return;
        }

        // We are going to or beyond Armed, switch to max tx power
        if new_mode >= FlightMode::Armed && self.mode < FlightMode::Armed {
            self.radio.set_max_transmit_power();
        }

        self.mode = new_mode;
        self.buzzer.switch_mode(self.time, new_mode);
    }

    fn handle_command(&mut self, cmd: Command) {
        log!(Info, "Received command: {:?}", cmd);
        match cmd {
            Command::Reboot => reboot(),
            Command::SetFlightMode(fm) => self.switch_mode(fm),
            Command::SetTransmitPower(txp) => self.radio.set_transmit_power(txp),
            Command::SetDataRate(dr) => self.data_rate = dr,
            Command::EraseFlash => self.flash.erase(),
            _ => {},
        }
    }

    /// Called every MAIN_LOOP_FREQ_HERTZ Hz.
    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self) {
        let cycles_before = hal::pac::DWT::cycle_count();

        // Read sensors
        self.power.tick(self.time);
        self.barometer.tick();
        self.imu.tick();
        self.acc.tick();
        self.gps.tick(self.time, &self.clocks);
        self.compass.tick();

        // Update state estimator
        self.state_estimator.update(
            self.time,
            self.mode,
            self.imu.gyroscope(),
            self.imu.accelerometer(),
            self.acc.accelerometer(),
            self.compass.magnetometer(),
            self.barometer.altitude()
        );

        // Switch to new mode if necessary
        if let Some(fm) = self.state_estimator.new_mode(
            self.power.arm_voltage().unwrap_or(0),
            self.power.breakwire_open()
        ) {
            self.switch_mode(fm);
        }

        // Handle incoming messages
        #[cfg(feature = "rev2")]
        if let Some((id, msg)) = self.can.tick() {
            match id {
                0x100 => self.power.handle_battery_can_msg(msg),
                0x110 => self.recovery_drogue = Some(msg.into()),
                0x111 => self.recovery_main = Some(msg.into()),
                _id => {
                    //log!(Debug, "Message from unknown CAN id (0x{:03x}): {:02x?}", id, msg)
                }
            }
        }

        if let Some(cmd) = self.radio.tick(self.time) {
            self.handle_command(cmd);
        }

        if let Some(msg) = self.usb_link.tick(self.time) {
            match msg {
                UplinkMessage::Heartbeat => {},
                UplinkMessage::Command(cmd) => if let Command::RebootToBootloader = cmd {
                    reboot_to_bootloader()
                } else {
                    self.handle_command(cmd)
                },
                UplinkMessage::ReadFlash(adr, size) => self.flash.downlink(&mut self.usb_link, adr, size),
                UplinkMessage::ReadSettings => self.usb_link.send_message(DownlinkMessage::Settings(self.settings.clone())),
                UplinkMessage::WriteSettings(settings) => {
                    if let Err(e) = self.flash.write_settings(&settings) {
                        log!(Error, "Failed to save settings: {:?}", e);
                    } else {
                        log!(Info, "Successfully saved settings, rebooting...");
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                },
                UplinkMessage::ApplyLoRaSettings(_) => {}
            }
        }

        // Set outputs
        let elapsed = self.state_estimator.time_in_mode();
        let recovery_duration = self.settings.outputs_warning_time + self.settings.outputs_high_time;
        let recovery_high = elapsed > self.settings.outputs_warning_time && elapsed < recovery_duration;
        self.recovery.0.set_state(((self.mode == FlightMode::RecoveryDrogue) && recovery_high).into());
        self.recovery.1.set_state(((self.mode == FlightMode::RecoveryMain) && recovery_high).into());

        let (r,y,g) = self.mode.led_state(self.time);
        self.leds.0.set_state((!r).into());
        self.leds.1.set_state((!y).into());
        self.leds.2.set_state((!g).into());
        self.buzzer.tick(self.time);

        // Send telemetry
        if let Some(msg) = self.next_usb_telem() {
            self.usb_link.send_message(msg);
        }

        if let Some(msg) = self.next_lora_telem() {
            self.radio.send_downlink_message(msg);
        }

        // Write data to flash
        let flash_message = (self.mode >= FlightMode::Armed)
            .then(|| self.next_flash_telem())
            .flatten();
        self.flash.tick(self.time, flash_message);

        self.runcam.tick(self.mode);

        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        // get CPU usage (max of RUNTIME_HISTORY_LEN last iterations)
        self.loop_runtime_history.truncate(RUNTIME_HISTORY_LEN - 1);
        let cycles_elapsed = hal::pac::DWT::cycle_count().wrapping_sub(cycles_before);
        self.loop_runtime_history.push_front((cycles_elapsed / CLOCK_FREQ_MEGA_HERTZ) as u16);
    }

    #[cfg(feature = "gcs")]
    pub fn tick(&mut self) {
        let downlink_msg = self.radio.tick(self.time);
        let uplink_msg = self.usb_link.tick(self.time).and_then(|msg| {
            match msg {
                UplinkMessage::Heartbeat => None,
                UplinkMessage::Command(Command::RebootToBootloader) => {
                    reboot_to_bootloader();
                    None
                },
                UplinkMessage::ApplyLoRaSettings(lora_settings) => {
                    self.radio.apply_settings(&lora_settings);
                    None
                },
                UplinkMessage::ReadSettings => None,
                msg => Some(msg)
            }
        });

        let rssi_led = (self.time % 100) > (self.radio.rssi as u32);
        self.leds.0.set_state((!(self.radio.transmit_power >= TransmitPower::P20dBm)).into());
        self.leds.1.set_state((!rssi_led).into());
        self.leds.2.set_state((!true).into());
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

        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);
    }

    #[cfg(not(feature = "gcs"))]
    fn next_usb_telem(&self) -> Option<DownlinkMessage> {
        if self.time % 100 == 0 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else if self.time % 50 == 0 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if self.time % 50 == 20 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else if self.time % 10 == 5 {
            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
        } else {
            None
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_lora_telem(&self) -> Option<DownlinkMessage> {
        if self.time % 1000 == 0 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else if self.time % 200 == 0 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if self.time % 100 == 50 {
            Some(DownlinkMessage::TelemetryMainCompressed(self.into()))
        } else if self.time % 50 == 25 && self.data_rate == TelemetryDataRate::High {
            Some(DownlinkMessage::TelemetryRawSensorsCompressed(self.into()))
        } else {
            None
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_flash_telem(&self) -> Option<DownlinkMessage> {
        // Offset everything a little so that flash message writes don't coincide
        // with lora message writes.
        let t = self.time + 3;
        if t % 100 == 0 {
            Some(DownlinkMessage::TelemetryGPS(self.into()))
        } else if t % 50 == 0 {
            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
        } else if t % 50 == 20 {
            Some(DownlinkMessage::TelemetryMain(self.into()))
        } else if t % 10 == 5 {
            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
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
            orientation: self.state_estimator.orientation.clone(),
            vertical_speed: self.state_estimator.vertical_speed(),
            vertical_accel: self.state_estimator.acceleration_world().map(|a| a.z).unwrap_or(0.0),
            vertical_accel_filtered: self.state_estimator.vertical_accel(),
            altitude_baro: self.barometer.altitude().unwrap_or(0.0),
            altitude: self.state_estimator.altitude(),
            altitude_max: self.state_estimator.altitude_max,
        }
    }
}

impl Into<TelemetryMainCompressed> for &Vehicle {
    fn into(self) -> TelemetryMainCompressed {
        let quat = self.state_estimator.orientation.clone().map(|q| q.coords).map(|q| {
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
            vertical_speed: (self.state_estimator.vertical_speed() * 10.0).into(),
            vertical_accel: self.state_estimator.acceleration_world().map(|a| a.z * 10.0).unwrap_or(0.0).into(),
            vertical_accel_filtered: (self.state_estimator.vertical_accel() * 10.0).into(),
            altitude_baro: (self.barometer.altitude().unwrap_or(0.0) * 10.0) as u16, // TODO: this limits us to 6km AMSL
            altitude: (self.state_estimator.altitude() * 10.0) as u16,
            altitude_max: (self.state_estimator.altitude_max * 10.0) as u16,
        }
    }
}

impl Into<TelemetryRawSensors> for &Vehicle {
    fn into(self) -> TelemetryRawSensors {
        TelemetryRawSensors {
            time: self.time,
            gyro: self.imu.gyroscope().unwrap_or_default(),
            accelerometer1: self.imu.accelerometer().unwrap_or_default(),
            accelerometer2: self.acc.accelerometer().unwrap_or_default(),
            magnetometer: self.compass.magnetometer().unwrap_or_default(),
            temperature_baro: self.barometer.temperature().unwrap_or_default(),
            pressure_baro: self.barometer.pressure().unwrap_or_default(),
        }
    }
}

impl Into<TelemetryRawSensorsCompressed> for &Vehicle {
    fn into(self) -> TelemetryRawSensorsCompressed {
        let gyro = self.imu.gyroscope().unwrap_or_default();
        let acc1 = self.imu.accelerometer().unwrap_or_default();
        let acc2 = self.acc.accelerometer().unwrap_or_default();
        let mag = self.compass.magnetometer().unwrap_or_default();
        TelemetryRawSensorsCompressed {
            time: self.time,
            gyro: (gyro * 10.0).into(),
            accelerometer1: (acc1 * 100.0).into(),
            accelerometer2: (acc2 * 10.0).into(),
            magnetometer: (mag * 10.0).into(),
            temperature_baro: (self.barometer.temperature().unwrap_or(0.0) * 2.0) as i8,
            pressure_baro: (self.barometer.pressure().unwrap_or(0.0) * 10.0) as u16,
        }
    }
}

impl Into<TelemetryDiagnostics> for &Vehicle {
    fn into(self) -> TelemetryDiagnostics {
        let loop_runtime = self.loop_runtime_history.iter()
            .fold(0, |a, b| u16::max(a, *b));
        let cpu_util = 100.0 * (loop_runtime as f32) / (1_000_000.0 / MAIN_LOOP_FREQ_HERTZ as f32);
        let heap_util = 100.0 * (crate::ALLOCATOR.used() as f32) / (crate::HEAP_SIZE as f32);
        let power_and_dr = ((self.data_rate as u8) << 7) | (self.radio.transmit_power as u8);

        TelemetryDiagnostics {
            time: self.time,
            cpu_utilization: cpu_util as u8,
            heap_utilization: heap_util as u8,
            temperature_core: (self.power.temperature().unwrap_or(0.0) * 2.0) as i8,
            cpu_voltage: self.power.cpu_voltage().unwrap_or(0),
            battery_voltage: self.power.battery_voltage().unwrap_or(0),
            arm_voltage: self.power.arm_voltage().unwrap_or(0),
            current: self.power.battery_current().unwrap_or(0),
            lora_rssi: self.radio.rssi,
            altitude_ground: (self.state_estimator.altitude_ground * 10.0) as u16,
            transmit_power_and_data_rate: power_and_dr,
        }
    }
}

impl Into<TelemetryGPS> for &Vehicle {
    fn into(self) -> TelemetryGPS {
        let latitude = self.gps.latitude
            .map(|lat| ((lat.clamp(-90.0, 90.0) + 90.0) * 16777215.0 / 180.0) as u32)
            .map(|lat| [(lat >> 16) as u8, (lat >> 8) as u8, lat as u8])
            .unwrap_or([0, 0, 0]);
        let longitude = self.gps.longitude
            .map(|lng| ((lng.clamp(-180.0, 180.0) + 180.0) * 16777215.0 / 360.0) as u32)
            .map(|lng| [(lng >> 16) as u8, (lng >> 8) as u8, lng as u8])
            .unwrap_or([0, 0, 0]);
        let fix_and_sats = ((self.gps.fix.clone() as u8) << 5) + ((self.gps.num_satellites as u8) & 0x1f);

        TelemetryGPS {
            time: self.time,
            fix_and_sats,
            hdop: self.gps.hdop,
            latitude,
            longitude,
            altitude_asl: self.gps.altitude.map(|alt| (alt * 10.0) as u16).unwrap_or(u16::MAX),
            flash_pointer: (self.flash.pointer / 1024) as u16,
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
