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

#[cfg(not(feature = "gcs"))]
use ahrs::Ahrs;
use filter::kalman::kalman_filter::KalmanFilter;
use nalgebra::*;
use num_traits::Pow;

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
use crate::telemetry::*;
use crate::usb::*;

const RUNTIME_HISTORY_LEN: usize = 200;

const G: f32 = 9.80665;

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
    // vehicle state
    ahrs: ahrs::Mahony<f32>,
    kalman: KalmanFilter<f32, U3, U2, U0>,
    orientation: Option<Unit<Quaternion<f32>>>,
    acceleration_world: Option<Vector3<f32>>,
    altitude_ground: f32,
    altitude_max: f32,
    pub time: u32,
    mode: FlightMode,
    mode_time: u32,
    condition_true_since: Option<u32>,
    loop_runtime_history: VecDeque<u16>,
    settings: Settings,
    data_rate: TelemetryDataRate,
}

impl Vehicle {
    #[rustfmt::skip]
    pub fn init(
        clocks: Clocks,
        usb_link: UsbLink,
        imu: Imu<Spi1, Pin<'B', 15, Output>>,
        acc: Accelerometer,
        compass: Compass,
        barometer: Barometer<Spi1, Pin<'C', 6, Output>>,
        gps: GPS,
        mut flash: Flash,
        mut radio: LoRaRadio<Spi1, Pin<'A', 1, Output>, Pin<'C', 0, Input>, Pin<'C', 1, Input>>,
        #[cfg(feature = "rev2")]
        can: MCP2517FD<Spi2, Pin<'B', 12, Output>>,
        power: Power,
        leds: LEDs,
        buzzer: Buzzer,
        recovery: Recovery
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
        let data_rate = settings.default_data_rate;

        let dt = 1.0 / (MAIN_LOOP_FREQ_HERTZ as f32);
        let ahrs = ahrs::Mahony::new(dt, settings.mahony_kp, settings.mahony_ki);
        let kalman = Self::init_kalman(dt, &settings);

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

            ahrs,
            kalman,
            orientation: None,
            acceleration_world: None,
            altitude_ground: 0.0,
            altitude_max: -10_000.0,

            time: 0,
            mode: FlightMode::Idle,
            mode_time: 0,
            condition_true_since: None,
            loop_runtime_history: VecDeque::with_capacity(RUNTIME_HISTORY_LEN),
            settings,
            data_rate,
        }
    }

    fn init_kalman(dt: f32, settings: &Settings) -> KalmanFilter<f32, U3, U2, U0> {
        let mut kalman = KalmanFilter::default();
        kalman.x = Vector3::new(0.0, 0.0, 0.0);

        kalman.F = Matrix3::new(
            1.0, dt, dt * dt * 0.5,
            0.0, 1.0, dt,
            0.0, 0.0, 1.0
        );

        kalman.H = Matrix2x3::new(
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0
        );

        kalman.P = Matrix3::new(
            settings.std_dev_barometer.pow(2), 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, settings.std_dev_accelerometer.pow(2),
        );

        kalman.Q = Matrix3::new(
            0.25f32 * dt.pow(4), 0.5f32 * dt.pow(3), 0.5f32 * dt.pow(2),
            0.5f32 * dt.pow(3), dt.pow(2), dt,
            0.5f32 * dt.pow(2), dt, 1.0f32,
        ) * settings.std_dev_process.pow(2);

        kalman.R *= Matrix2::new(
            settings.std_dev_barometer.pow(2), 0.0,
            0.0, settings.std_dev_accelerometer.pow(2)
        );

        kalman
    }

    fn true_since(&mut self, cond: bool, duration: u32) -> bool {
        let dur = self.condition_true_since
            .map(|d| d + 1000 / crate::MAIN_LOOP_FREQ_HERTZ)
            .unwrap_or(0);
        self.condition_true_since = cond.then(|| dur);
        self.condition_true_since.map(|dur| dur >= duration).unwrap_or(false)
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
        self.mode_time = self.time;
        self.condition_true_since = None;
        self.buzzer.switch_mode(self.time, new_mode);
    }

    pub fn tick_mode(&mut self) {
        if self.mode < FlightMode::Armed {
            self.altitude_ground = self.altitude();
        }

        if self.mode <= FlightMode::Armed {
            self.altitude_max = self.altitude();
        }

        if self.mode == FlightMode::Flight {
            self.altitude_max = f32::max(self.altitude_max, self.altitude());
        }

        let elapsed = self.time.checked_sub(self.mode_time).unwrap_or(0);
        let armv = self.power.arm_voltage().unwrap_or(0);
        let vacc = self.acceleration().map(|acc| acc.z).unwrap_or(0.0);
        let rec_min_dur = self.settings.outputs_warning_time + self.settings.outputs_high_time;

        let new_mode = match self.mode {
            FlightMode::Idle => self.true_since(armv >= 100, 100).then(|| FlightMode::HardwareArmed),
            FlightMode::HardwareArmed => self.true_since(armv < 10, 100).then(|| FlightMode::Idle),
            FlightMode::Armed => self.true_since(vacc > self.settings.min_takeoff_acc, self.settings.min_takeoff_acc_time).then(|| FlightMode::Flight),
            FlightMode::Flight => {
                let falling = self.true_since(self.vertical_speed() < 0.0, self.settings.apogee_min_falling_time);
                let min_exceeded = elapsed > self.settings.min_time_to_apogee;
                let max_exceeded = elapsed > self.settings.max_time_to_apogee;
                ((min_exceeded && falling) || max_exceeded).then(|| FlightMode::RecoveryDrogue)
            }
            FlightMode::RecoveryDrogue => {
                match self.settings.main_output_mode {
                    MainOutputMode::AtApogee => (elapsed > rec_min_dur).then(|| FlightMode::RecoveryMain),
                    MainOutputMode::BelowAltitude => {
                        let below_alt = self.altitude() < self.settings.main_output_deployment_altitude;
                        (elapsed > rec_min_dur && below_alt).then(|| FlightMode::RecoveryMain)
                    },
                    MainOutputMode::Never => {
                        let landed = self.true_since(self.acceleration().map(|acc| (9.5..10.0).contains(&acc.magnitude())).unwrap_or(true), 1000);
                        (elapsed > rec_min_dur && landed).then(|| FlightMode::Landed)
                    },
                }
            },
            FlightMode::RecoveryMain => {
                let landed = self.true_since(self.acceleration().map(|acc| (9.5..10.0).contains(&acc.magnitude())).unwrap_or(true), 1000);
                (elapsed > rec_min_dur && landed).then(|| FlightMode::Landed)
            },
            FlightMode::Landed => None
        };

        if let Some(fm) = new_mode {
            self.switch_mode(fm);
        }

        let recovery_high = elapsed > self.settings.outputs_warning_time && elapsed < rec_min_dur;
        self.recovery.0.set_state(((self.mode == FlightMode::RecoveryDrogue) && recovery_high).into());
        self.recovery.1.set_state(((self.mode == FlightMode::RecoveryMain) && recovery_high).into());
    }

    fn acceleration(&self) -> Option<Vector3<f32>> {
        // TODO: use backup acc if necessary
        self.imu.accelerometer()
    }

    fn altitude(&self) -> f32 {
        self.kalman.x.x
    }

    fn vertical_speed(&self) -> f32 {
        self.kalman.x.y
    }

    fn vertical_accel(&self) -> f32 {
        self.kalman.x.z
    }

    #[cfg(not(feature = "gcs"))]
    fn update_state_estimator(&mut self) {
        let gyro = self.imu.gyroscope();
        let acc = self.acceleration();
        let mag = self.compass.magnetometer();
        if let (Some(gyro), Some(acc), Some(mag)) = (&gyro, &acc, &mag) {
            self.orientation = self
                .ahrs
                .update(&(gyro * 3.14159 / 180.0), acc, mag)
                .ok()
                .map(|q| *q);
            self.acceleration_world = self.orientation
                .map(|quat| quat.transform_vector(acc) - Vector3::new(0.0, 0.0, G));
        } else {
            self.orientation = None;
            self.acceleration_world = None;
        }

        let altitude_baro = self.barometer.altitude()
            .and_then(|a| (!a.is_nan()).then(|| a));
        let accel_z = self.acceleration_world.map(|a| a.z)
            .and_then(|a| (!a.is_nan()).then(|| a));

        match (accel_z, altitude_baro) {
            (Some(accel_z), Some(altitude_baro)) => {
                let z = Vector2::new(altitude_baro, accel_z);
                self.kalman.update(&z, None, None);
                self.kalman.predict(None, None, None, None);
            },
            // TODO: handle error cases
            _ => {}
        }
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
        self.power.tick();
        self.barometer.tick();
        self.imu.tick();
        self.acc.tick(self.time, self.imu.accelerometer());
        self.gps.tick(self.time, &self.clocks);
        self.compass.tick();

        self.update_state_estimator();

        #[cfg(feature = "rev2")]
        self.can.tick();

        // Handle incoming messages
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

        self.tick_mode();

        let (r,y,g) = self.mode.led_state(self.time);
        self.leds.0.set_state((!r).into());
        self.leds.1.set_state((!y).into());
        self.leds.2.set_state((!g).into());
        self.buzzer.tick(self.time);

        if let Some(msg) = self.next_usb_telem() {
            self.usb_link.send_message(msg);
        }

        if let Some(msg) = self.next_lora_telem() {
            self.radio.send_downlink_message(msg);
        }

        let flash_message = (self.mode >= FlightMode::Armed)
            .then(|| self.next_flash_telem())
            .flatten();
        self.flash.tick(self.time, flash_message);

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
            orientation: self.orientation.clone(),
            vertical_speed: self.vertical_speed(),
            vertical_accel: self.acceleration_world.map(|a| a.z).unwrap_or(0.0),
            vertical_accel_filtered: self.vertical_accel(),
            altitude_baro: self.barometer.altitude().unwrap_or(0.0),
            altitude: self.altitude(),
            altitude_max: self.altitude_max,
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
            vertical_speed: (self.vertical_speed() * 10.0).into(),
            vertical_accel: self.acceleration_world.map(|a| a.z * 10.0).unwrap_or(0.0).into(),
            vertical_accel_filtered: (self.vertical_accel() * 10.0).into(),
            altitude_baro: (self.barometer.altitude().unwrap_or(0.0) * 10.0) as u16, // TODO: this limits us to 6km AMSL
            altitude: (self.altitude() * 10.0) as u16,
            altitude_max: (self.altitude_max * 10.0) as u16,
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
            altitude_ground: (self.altitude_ground * 10.0) as u16,
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
