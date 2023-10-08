//! Main flight logic for flight computer. State estimation and ground control station
//! stuff could maybe be moved out of here.

use alloc::collections::VecDeque;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use embassy_time::{Ticker, Duration};

use defmt::*;

//use crate::bootloader::*;
//#[cfg(feature = "rev2")]
//use crate::can::MCP2517FD;
//use crate::buzzer::*;
//use crate::logging::*;
//use crate::lora::*;
//use crate::params::*;
use crate::sensors::*;
use crate::settings::*;
use crate::state_estimation::StateEstimator;
use crate::telemetry::*;
use crate::usb::UsbLink;
//use crate::usb::*;
//use crate::runcam::*;

const RUNTIME_HISTORY_LEN: usize = 200;

//#[cfg(feature = "rev1")]
//type Flash = crate::flash::Flash<Spi2, Pin<'B', 12, Output>>;
//#[cfg(feature = "rev2")]
//type Flash = crate::flash::Flash<Spi3, Pin<'D', 2, Output>>;
//
//#[derive(Default, Clone)]
//struct RecoveryState {
//    pressure_cartridge: u16,
//    pressure_chamber: u16,
//    temperature: f32
//}
//
//impl From<[u8; 8]> for RecoveryState {
//    fn from(msg: [u8; 8]) -> Self {
//        Self {
//            pressure_cartridge: u16::from_le_bytes([msg[0], msg[1]]),
//            pressure_chamber: u16::from_le_bytes([msg[2], msg[3]]),
//            temperature: u16::from_le_bytes([msg[4], msg[5]]) as f32,
//        }
//    }
//}
//
//impl Into<[u8; 2]> for RecoveryState {
//    fn into(self) -> [u8; 2] {
//        let pressure_cartridge = self.pressure_cartridge as u8;
//        let pressure_chamber = self.pressure_chamber as u8;
//        [pressure_cartridge, pressure_chamber]
//    }
//}

type Imu = LSM6<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PB15>>>;
type Accelerometer = H3LIS331DL<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PA4>>>;
type Magnetometer = LIS3MDL<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PB10>>>;
type Barometer = MS5611<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PC6>>>;
type Power = PowerMonitor<ADC1, PB0, PC5, PC4>;

type LEDs = (Output<'static, PC13>, Output<'static, PC14>, Output<'static, PC15>);
type Recovery = (Output<'static, PC8>, Output<'static, PC9>);

const MAIN_LOOP_FREQUENCY: Hertz = Hertz::hz(1000);

#[cfg_attr(feature = "gcs", allow(dead_code))]
pub struct Vehicle {
    pub time: core::num::Wrapping<u32>,
    // sensors
    imu: Imu,
    acc: Accelerometer,
    mag: Magnetometer,
    baro: Barometer,
//    gps: GPS,
    power: Power,
    // other peripherals
    usb: UsbLink,
//    radio: LoRaRadio<Spi1, Pin<'A', 1, Output>, Pin<'C', 0, Input>, Pin<'C', 1, Input>>,
//    flash: Flash,
//    can: MCP2517FD<Spi2, Pin<'B', 12, Output>>,
    // outputs
    leds: LEDs,
//    buzzer: Buzzer,
    recovery: Recovery,
//    runcam: RuncamCamera,
    // vehicle state
    state_estimator: StateEstimator,
    mode: FlightMode,
    loop_runtime_history: VecDeque<f32>,
    settings: Settings,
//    recovery_drogue: Option<RecoveryState>,
//    recovery_main: Option<RecoveryState>,
}

impl Into<VehicleState> for &mut Vehicle {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time.0,
            mode: Some(self.mode),
            orientation: self.state_estimator.orientation,
            vertical_speed: Some(self.state_estimator.vertical_speed()),
            vertical_accel: self.state_estimator.acceleration_world().map(|a| a.z * 10.0),
            vertical_accel_filtered: Some(self.state_estimator.vertical_accel()),
            altitude_max: Some(self.state_estimator.altitude_max),
            altitude: Some(self.state_estimator.altitude()),

            gyroscope: self.imu.gyroscope(),
            accelerometer1: self.imu.accelerometer(),
            accelerometer2: self.acc.accelerometer(),
            magnetometer: self.mag.magnetometer(),
            pressure_baro: self.baro.pressure(),
            altitude_baro: self.baro.altitude(),

            // MAX
            //cpu_utilization: Some(self.loop_runtime_history.iter().fold(0.0, |a, b| f32::max(a, *b))),
            cpu_utilization: Some(self.loop_runtime_history.iter().sum::<f32>() / self.loop_runtime_history.len() as f32),
            charge_voltage: self.power.charge_voltage(),
            battery_voltage: self.power.battery_voltage(),
            current: self.power.battery_current(),
            //pub lora_rssi: u8,
            altitude_ground: Some(self.state_estimator.altitude_ground),
            //pub transmit_power_and_data_rate: u8,
            temperature_baro: self.baro.temperature(),
            //pub recovery_drogue: [u8; 2],
            //pub recovery_main: [u8; 2],

            ..Default::default()

        }
    }
}

// TODO: move to main?
#[embassy_executor::task]
pub async fn run(mut vehicle: Vehicle) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
    loop {
        vehicle.tick().await;
        ticker.next().await;
    }
}

impl Vehicle {
    pub async fn init(
        //high_priority_spawner: SendSpawner,
        //low_priority_spawner: Spawner,
        mut imu: Imu,
        mut acc: Accelerometer,
        mut mag: Magnetometer,
        baro: Barometer,
        power: Power,
        usb: UsbLink,
        leds: LEDs,
        recovery: Recovery,
    ) -> Result<Self, ()> { // TODO: error type
//        // Use this to reset settings without ground station during development.
//        //flash.write_settings(&Settings::default()).unwrap();
//
        let settings = Settings::default(); // TODO

//        #[cfg(not(feature="gcs"))]
//        let settings = {
//            match flash.read_settings() {
//                Ok(settings) => settings,
//                Err(e) => {
//                    log!(Error, "Failed to read settings: {:?}, reverting to defaults.", e);
//                    Settings::default()
//                }
//            }
//        };
//
        #[cfg(not(feature="gcs"))]
        info!("Loaded Settings: {:#?}", Debug2Format(&settings));

        #[cfg(feature="gcs")]
        let settings = Settings::default();

//        buzzer.set_warning_tone(settings.outputs_warning_frequency, settings.outputs_warning_time);
//        radio.apply_settings(&settings.lora);
        imu.set_offsets(settings.gyro_offset, settings.acc_offset);
        acc.set_offset(settings.acc2_offset);
        mag.set_offset(settings.mag_offset);

        Ok(Self {
            time: core::num::Wrapping(0),

            imu,
            acc,
            mag,
            baro,
            power,

            usb,

            leds,
            recovery,

            state_estimator: StateEstimator::new(MAIN_LOOP_FREQUENCY.0 as f32, settings.clone()),
            mode: FlightMode::Idle,

            loop_runtime_history: VecDeque::with_capacity(RUNTIME_HISTORY_LEN),
            settings
        })
    }

    async fn tick(&mut self) {
        let start = Instant::now();

        // TODO: should we separate these into separate tasks?
        self.imu.tick().await;
        self.acc.tick().await;
        self.mag.tick().await;
        self.baro.tick().await;
        self.power.tick();

        // Update state estimator
        self.state_estimator.update(
            self.time,
            self.mode,
            self.imu.gyroscope(),
            self.imu.accelerometer(),
            self.acc.accelerometer(),
            self.mag.magnetometer(),
            self.baro.altitude()
        );

        // Switch to new mode if necessary
        if let Some(fm) = self.state_estimator.new_mode(
            self.power.arm_voltage().unwrap_or(0),
            self.power.breakwire_open()
        ) {
            self.switch_mode(fm);
        }

        // Set outputs
        let elapsed = self.state_estimator.time_in_mode();
        let recovery_duration = self.settings.outputs_warning_time + self.settings.outputs_high_time;
        let recovery_high = elapsed > self.settings.outputs_warning_time && elapsed < recovery_duration;
        self.recovery.0.set_level(((self.mode == FlightMode::RecoveryDrogue) && recovery_high).into());
        self.recovery.1.set_level(((self.mode == FlightMode::RecoveryMain) && recovery_high).into());

        let (r,y,g) = self.mode.led_state(self.time.0);
        self.leds.0.set_level((!r).into());
        self.leds.1.set_level((!y).into());
        self.leds.2.set_level((!g).into());
        //self.buzzer.tick(self.time);

        if self.time.0 % 10 == 5 {
            let vs: VehicleState = self.into();
            let msg = DownlinkMessage::TelemetryRawSensors(vs.into());
            self.usb.send_downlink_message(msg).await;
        }
        if self.time.0 % 50 == 25 {
            let vs: VehicleState = self.into();
            let msg = DownlinkMessage::TelemetryMain(vs.into());
            self.usb.send_downlink_message(msg).await;
        }
        if self.time.0 % 200 == 0 {
            let vs: VehicleState = self.into();
            let msg = DownlinkMessage::TelemetryDiagnostics(vs.into());
            self.usb.send_downlink_message(msg).await;
        }

        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;

        // get CPU usage
        self.loop_runtime_history.truncate(RUNTIME_HISTORY_LEN - 1);
        let elapsed = start.elapsed().as_micros();
        self.loop_runtime_history.push_front((elapsed as f32) / (1_000_000.0 / MAIN_LOOP_FREQUENCY.0 as f32));
    }

//    #[rustfmt::skip]
//    pub fn init(
//        gps: GPS,
//        mut flash: Flash,
//        mut radio: LoRaRadio<Spi1, Pin<'A', 1, Output>, Pin<'C', 0, Input>, Pin<'C', 1, Input>>,
//        #[cfg(feature = "rev2")]
//        can: MCP2517FD<Spi2, Pin<'B', 12, Output>>,
//        mut buzzer: Buzzer,
//        runcam: RuncamCamera,
//    ) -> Self {
//
//        Self {
//            clocks,
//
//            gps,
//
//            flash,
//            radio,
//            #[cfg(feature = "rev2")]
//            can,
//            buzzer,
//            runcam,
//
//            settings,
//            data_rate,
//        }
//    }

    fn switch_mode(&mut self, new_mode: FlightMode) {
        if new_mode == self.mode {
            return;
        }

        // We are going to or beyond Armed, switch to max tx power
        if new_mode >= FlightMode::Armed && self.mode < FlightMode::Armed {
            //self.radio.set_max_transmit_power();
        }

        self.mode = new_mode;
        //self.buzzer.switch_mode(self.time, new_mode);
    }
//
//    fn handle_command(&mut self, cmd: Command) {
//        log!(Info, "Received command: {:?}", cmd);
//        match cmd {
//            Command::Reboot => reboot(),
//            Command::SetFlightMode(fm) => self.switch_mode(fm),
//            Command::SetTransmitPower(txp) => self.radio.set_transmit_power(txp),
//            Command::SetDataRate(dr) => self.data_rate = dr,
//            Command::EraseFlash => self.flash.erase(),
//            _ => {},
//        }
//    }
//
//    /// Called every MAIN_LOOP_FREQ_HERTZ Hz.
//    #[cfg(not(feature = "gcs"))]
//    pub fn tick(&mut self) {
//        let cycles_before = hal::pac::DWT::cycle_count();
//
//        // Read sensors
//        self.power.tick(self.time);
//        self.barometer.tick();
//        self.gps.tick(self.time, &self.clocks);
//
//        // Handle incoming messages
//        #[cfg(feature = "rev2")]
//        if let Some((id, msg)) = self.can.tick() {
//            match id {
//                0x100 => self.power.handle_battery_can_msg(msg),
//                0x110 => self.recovery_drogue = Some(msg.into()),
//                0x111 => self.recovery_main = Some(msg.into()),
//                _id => {
//                    //log!(Debug, "Message from unknown CAN id (0x{:03x}): {:02x?}", id, msg)
//                }
//            }
//        }
//
//        if let Some(cmd) = self.radio.tick(self.time) {
//            self.handle_command(cmd);
//        }
//
//        if let Some(msg) = self.usb_link.tick(self.time) {
//            match msg {
//                UplinkMessage::Heartbeat => {},
//                UplinkMessage::Command(cmd) => if let Command::RebootToBootloader = cmd {
//                    reboot_to_bootloader()
//                } else {
//                    self.handle_command(cmd)
//                },
//                UplinkMessage::ReadFlash(adr, size) => self.flash.downlink(&mut self.usb_link, adr, size),
//                UplinkMessage::ReadSettings => self.usb_link.send_message(DownlinkMessage::Settings(self.settings.clone())),
//                UplinkMessage::WriteSettings(settings) => {
//                    if let Err(e) = self.flash.write_settings(&settings) {
//                        log!(Error, "Failed to save settings: {:?}", e);
//                    } else {
//                        log!(Info, "Successfully saved settings, rebooting...");
//                        cortex_m::peripheral::SCB::sys_reset();
//                    }
//                },
//                UplinkMessage::ApplyLoRaSettings(_) => {}
//            }
//        }
//
//        // Send telemetry
//        if let Some(msg) = self.next_usb_telem() {
//            self.usb_link.send_message(msg);
//        }
//
//        if let Some(msg) = self.next_lora_telem() {
//            self.radio.send_downlink_message(msg);
//        }
//
//        // Write data to flash
//        let flash_message = (self.mode >= FlightMode::Armed)
//            .then(|| self.next_flash_telem())
//            .flatten();
//        self.flash.tick(self.time, flash_message);
//
//        self.runcam.tick(self.mode);
//    }
//
//    #[cfg(feature = "gcs")]
//    pub fn tick(&mut self) {
//        let downlink_msg = self.radio.tick(self.time);
//        let uplink_msg = self.usb_link.tick(self.time).and_then(|msg| {
//            match msg {
//                UplinkMessage::Heartbeat => None,
//                UplinkMessage::Command(Command::RebootToBootloader) => {
//                    reboot_to_bootloader();
//                    None
//                },
//                UplinkMessage::ApplyLoRaSettings(lora_settings) => {
//                    self.radio.apply_settings(&lora_settings);
//                    None
//                },
//                UplinkMessage::ReadSettings => None,
//                msg => Some(msg)
//            }
//        });
//
//        let rssi_led = (self.time % 100) > (self.radio.rssi as u32);
//        self.leds.0.set_state((!(self.radio.transmit_power >= TransmitPower::P20dBm)).into());
//        self.leds.1.set_state((!rssi_led).into());
//        self.leds.2.set_state((!true).into());
//        self.buzzer.tick(self.time);
//
//        if let Some(msg) = uplink_msg {
//            self.radio.queue_uplink_message(msg);
//        }
//
//        if let Some(msg) = downlink_msg {
//            let gcs_message = DownlinkMessage::TelemetryGCS(TelemetryGCS {
//                time: msg.time(),
//                lora_rssi: self.radio.rssi,
//                lora_rssi_signal: self.radio.rssi_signal,
//                lora_snr: self.radio.snr,
//            });
//
//            self.usb_link.send_message(msg);
//            self.usb_link.send_message(gcs_message);
//        }
//
//        self.time += 1_000 / crate::MAIN_LOOP_FREQ_HERTZ;
//        Logger::update_time(self.time);
//    }
//
//    #[cfg(not(feature = "gcs"))]
//    fn next_usb_telem(&self) -> Option<DownlinkMessage> {
//        if self.time % 100 == 0 {
//            Some(DownlinkMessage::TelemetryGPS(self.into()))
//        } else if self.time % 50 == 0 {
//            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
//        } else if self.time % 50 == 20 {
//            Some(DownlinkMessage::TelemetryMain(self.into()))
//        } else if self.time % 10 == 5 {
//            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
//        } else {
//            None
//        }
//    }
//
//    #[cfg(not(feature = "gcs"))]
//    fn next_lora_telem(&self) -> Option<DownlinkMessage> {
//        if self.time % 1000 == 0 {
//            Some(DownlinkMessage::TelemetryGPS(self.into()))
//        } else if self.time % 200 == 0 {
//            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
//        } else if self.time % 100 == 50 {
//            Some(DownlinkMessage::TelemetryMainCompressed(self.into()))
//        } else if self.time % 50 == 25 && self.data_rate == TelemetryDataRate::High {
//            Some(DownlinkMessage::TelemetryRawSensorsCompressed(self.into()))
//        } else {
//            None
//        }
//    }
//
//    #[cfg(not(feature = "gcs"))]
//    fn next_flash_telem(&self) -> Option<DownlinkMessage> {
//        // Offset everything a little so that flash message writes don't coincide
//        // with lora message writes.
//        let t = self.time + 3;
//        if t % 100 == 0 {
//            Some(DownlinkMessage::TelemetryGPS(self.into()))
//        } else if t % 50 == 0 {
//            Some(DownlinkMessage::TelemetryDiagnostics(self.into()))
//        } else if t % 50 == 20 {
//            Some(DownlinkMessage::TelemetryMain(self.into()))
//        } else if t % 10 == 5 {
//            Some(DownlinkMessage::TelemetryRawSensors(self.into()))
//        } else {
//            None
//        }
//    }
}

//impl Into<TelemetryMainCompressed> for &Vehicle {
//    fn into(self) -> TelemetryMainCompressed {
//        let quat = self.state_estimator.orientation.clone().map(|q| q.coords).map(|q| {
//            (
//                (127.0 + q.x * 127.0) as u8,
//                (127.0 + q.y * 127.0) as u8,
//                (127.0 + q.z * 127.0) as u8,
//                (127.0 + q.w * 127.0) as u8,
//            )
//        });
//        TelemetryMainCompressed {
//            time: self.time,
//            mode: self.mode.clone(),
//            orientation: quat.unwrap_or((127, 127, 127, 127)),
//            vertical_speed: (self.state_estimator.vertical_speed() * 10.0).into(),
//            vertical_accel: self.state_estimator.acceleration_world().map(|a| a.z * 10.0).unwrap_or(0.0).into(),
//            vertical_accel_filtered: (self.state_estimator.vertical_accel() * 10.0).into(),
//            altitude_baro: (self.barometer.altitude().unwrap_or(0.0) * 10.0 + 1000.0) as u16, // TODO: this limits us to 6km AMSL
//            altitude: (self.state_estimator.altitude() * 10.0 + 1000.0) as u16,
//            altitude_max: (self.state_estimator.altitude_max * 10.0 + 1000.0) as u16,
//        }
//    }
//}
//
//impl Into<TelemetryRawSensorsCompressed> for &Vehicle {
//    fn into(self) -> TelemetryRawSensorsCompressed {
//        let gyro = self.imu.gyroscope().unwrap_or_default();
//        let acc1 = self.imu.accelerometer().unwrap_or_default();
//        let acc2 = self.acc.accelerometer().unwrap_or_default();
//        let mag = self.compass.magnetometer().unwrap_or_default();
//        TelemetryRawSensorsCompressed {
//            time: self.time,
//            gyro: (gyro * 10.0).into(),
//            accelerometer1: (acc1 * 100.0).into(),
//            accelerometer2: (acc2 * 10.0).into(),
//            magnetometer: (mag * 10.0).into(),
//            pressure_baro: (self.barometer.pressure().unwrap_or(0.0) * 10.0) as u16,
//        }
//    }
//}
//
//impl Into<TelemetryDiagnostics> for &Vehicle {
//    fn into(self) -> TelemetryDiagnostics {
//        let power_and_dr = ((self.data_rate as u8) << 7) | (self.radio.transmit_power as u8);
//
//        let breakwire = match self.power.breakwire_open() {
//            None => 0b10,
//            Some(b) => b as u8,
//        };
//
//        TelemetryDiagnostics {
//            time: self.time,
//            cpu_utilization: cpu_util as u8,
//            charge_voltage: self.power.charge_voltage().unwrap_or(0),
//            battery_voltage: self.power.battery_voltage().unwrap_or(0) << 2 | breakwire as u16,
//            current: self.power.battery_current().unwrap_or(0),
//            lora_rssi: self.radio.rssi,
//            altitude_ground: (self.state_estimator.altitude_ground * 10.0 + 1000.0) as u16,
//            transmit_power_and_data_rate: power_and_dr,
//            temperature_baro: (self.barometer.temperature().unwrap_or(0.0) * 2.0) as i8,
//            recovery_drogue: self.recovery_drogue.clone().map(|r| r.into()).unwrap_or([0, 0]),
//            recovery_main: self.recovery_main.clone().map(|r| r.into()).unwrap_or([0, 0]),
//        }
//    }
//}
//
//impl Into<TelemetryGPS> for &Vehicle {
//    fn into(self) -> TelemetryGPS {
//        let latitude = self.gps.latitude
//            .map(|lat| ((lat.clamp(-90.0, 90.0) + 90.0) * 16777215.0 / 180.0) as u32)
//            .map(|lat| [(lat >> 16) as u8, (lat >> 8) as u8, lat as u8])
//            .unwrap_or([0, 0, 0]);
//        let longitude = self.gps.longitude
//            .map(|lng| ((lng.clamp(-180.0, 180.0) + 180.0) * 16777215.0 / 360.0) as u32)
//            .map(|lng| [(lng >> 16) as u8, (lng >> 8) as u8, lng as u8])
//            .unwrap_or([0, 0, 0]);
//        let fix_and_sats = ((self.gps.fix.clone() as u8) << 5) + ((self.gps.num_satellites as u8) & 0x1f);
//
//        TelemetryGPS {
//            time: self.time,
//            fix_and_sats,
//            hdop: self.gps.hdop,
//            latitude,
//            longitude,
//            altitude_asl: self.gps.altitude.map(|alt| (alt * 10.0 + 1000.0) as u16).unwrap_or(u16::MAX),
//            flash_pointer: (self.flash.pointer / 1024) as u16,
//        }
//    }
//}
//
//impl Into<TelemetryGCS> for &Vehicle {
//    fn into(self) -> TelemetryGCS {
//        TelemetryGCS {
//            time: self.time,
//            lora_rssi: self.radio.rssi,
//            lora_rssi_signal: self.radio.rssi_signal,
//            lora_snr: self.radio.snr,
//        }
//    }
//}
