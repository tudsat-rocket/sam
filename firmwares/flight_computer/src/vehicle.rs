use hal::rcc::Clocks;
use stm32f4xx_hal as hal;
use hal::gpio::{Pin, Output};

#[cfg(not(feature = "gcs"))]
use ahrs::Ahrs;
use filter::kalman::kalman_filter::KalmanFilter;
use nalgebra::*;
use num_traits::Pow;

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

const STD_DEV_ACCELEROMETER: f32 = 0.5;
const STD_DEV_BAROMETER: f32 = 1.0;
const STD_DEV_PROCESS: f32 = 0.005;
const G: f32 = 9.80665;

const MIN_TAKEOFF_ACC: f32 = 20.; // minimum vertical acceleration for takeoff detection (m/s^2)
const APOGEE_VERTICAL_DISTANCE: f32 = 1.0; // minimum vertical distance to max altitude for apogee
                                           // detection (m)
const APOGEE_FALLING_TIME: u32 = 1000; // time APOGEE_VERTICAL_DISTANCE has to be exceeded for
                                       // recovery to be started (ms)

type LEDS = (Pin<'C',13,Output>, Pin<'C',14,Output>, Pin<'C',15,Output>);
type RECOVERY = (Pin<'C', 8, Output>, Pin<'C', 9, Output>);

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
    leds: LEDS,
    buzzer: Buzzer,
    recovery: RECOVERY,

    ahrs: ahrs::Mahony<f32>,
    kalman: KalmanFilter<f32, U3, U2, U0>,
    orientation: Option<Unit<Quaternion<f32>>>,
    acceleration_world: Option<Vector3<f32>>,

    pub time: u32,
    mode: FlightMode,
    mode_time: u32,
    loop_runtime: u16,
}

impl<'a> Vehicle {
    #[rustfmt::skip]
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
        leds: LEDS,
        buzzer: Buzzer,
        recovery: RECOVERY
    ) -> Self {
        let dt = 1.0 / (MAIN_LOOP_FREQ_HERTZ as f32);
        let ahrs = ahrs::Mahony::new(dt, MAHONY_KP, MAHONY_KI);
        let kalman = Self::init_kalman(dt);

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
            leds,
            buzzer,
            recovery,

            ahrs,
            kalman,
            orientation: None,
            acceleration_world: None,

            time: 0,
            mode: FlightMode::Idle,
            mode_time: 0,
            loop_runtime: 0,
        }
    }

    fn init_kalman(dt: f32) -> KalmanFilter<f32, U3, U2, U0> {
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
            STD_DEV_BAROMETER * STD_DEV_BAROMETER, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, STD_DEV_ACCELEROMETER * STD_DEV_ACCELEROMETER,
        );

        kalman.Q = Matrix3::new(
            0.25f32 * dt.pow(4), 0.5f32 * dt.pow(3), 0.5f32 * dt.pow(2),
            0.5f32 * dt.pow(3), dt.pow(2), dt,
            0.5f32 * dt.pow(2), dt, 1.0f32,
        ) * STD_DEV_PROCESS * STD_DEV_PROCESS;

        kalman.R *= Matrix2::new(
            STD_DEV_BAROMETER * STD_DEV_BAROMETER, 0.0,
            0.0, STD_DEV_ACCELEROMETER * STD_DEV_ACCELEROMETER
        );

        kalman
    }

    pub fn tick_mode(&mut self) {
        let new_mode = match self.mode {
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
        };

        //if let Some(fm) = new_mode {
        //    self.switch_mode(fm);
        //}

        self.recovery.0.set_state((self.mode == FlightMode::RecoveryDrogue).into()); // TODO
        self.recovery.1.set_state((self.mode == FlightMode::RecoveryMain).into()); // TODO
    }

    fn switch_mode(&mut self, mode: FlightMode) {
        self.mode = mode;
        self.mode_time = self.time;
    }

    fn acceleration(&self) -> Option<Vector3<f32>> {
        // TODO: use backup acc if necessary
        self.imu.accelerometer().map(|v| Vector3::new(v.0, v.1, v.2))
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
        let gyro = self.imu.gyroscope().map(|v| Vector3::new(v.0, v.1, v.2));
        let acc = self.acceleration();
        let mag = self.compass.magnetometer().map(|v| Vector3::new(v.0, v.1, v.2));
        if let (Some(gyro), Some(acc), Some(mag)) = (&gyro, &acc, &mag) {
            self.orientation = self
                .ahrs
                .update(gyro, acc, mag)
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

    /// Called every MAIN_LOOP_FREQ_HERTZ Hz.
    #[cfg(not(feature = "gcs"))]
    pub fn tick(&mut self) {
        let cycles_before = hal::pac::DWT::cycle_count();

        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
        Logger::update_time(self.time);

        // Read sensors
        self.acc.tick();
        self.barometer.tick();
        self.gps.tick(self.time, &self.clocks);
        self.power.tick();
        self.imu.tick();
        self.compass.tick();

        self.update_state_estimator();

        // TODO: write to flash, sd card

        // Handle incoming messages
        if let Some(msg) = self.radio.tick(self.time) {
            match msg {
                UplinkMessage::RebootAuth(_) => reboot(),
                UplinkMessage::SetFlightModeAuth(fm, _) => self.switch_mode(fm),
                _ => {},
            }
        }

        if let Some(msg) = self.usb_link.tick(self.time) {
            match msg {
                UplinkMessage::Heartbeat => {}
                UplinkMessage::Reboot | UplinkMessage::RebootAuth(_) => reboot(),
                UplinkMessage::RebootToBootloader => reboot_to_bootloader(),
                UplinkMessage::SetFlightMode(fm) | UplinkMessage::SetFlightModeAuth(fm, _) => self.switch_mode(fm)
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

        // TODO
        self.flash.tick(self.time, None);

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
                UplinkMessage::RebootToBootloader => {
                    reboot_to_bootloader();
                    None
                },
                msg => Some(msg)
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
            vertical_speed: self.vertical_speed(),
            vertical_accel: self.acceleration_world.map(|a| a.z).unwrap_or(0.0),
            vertical_accel_filtered: self.vertical_accel(),
            altitude_baro: self.barometer.altitude().unwrap_or(0.0),
            altitude: self.altitude(),
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
            vertical_speed: (self.vertical_speed() * 100.0).into(),
            vertical_accel: self.acceleration_world.map(|a| a.z * 100.0).unwrap_or(0.0).into(),
            vertical_accel_filtered: (self.vertical_accel() * 100.0).into(),
            altitude_baro: (self.barometer.altitude().unwrap_or(0.0) * 10.0) as u16, // TODO: this limits us to 6km AMSL
            altitude: (self.altitude() * 10.0) as u16,
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
