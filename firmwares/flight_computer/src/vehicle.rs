//! Main flight logic for flight computer.

use core::convert::Infallible;
use core::num::Wrapping;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_futures::join::{join_array, join3, join5};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::Instant;
use embassy_time::{Duration, Ticker};

use defmt::*;

use shared_types::{
    AcsMode, CanBusMessage, CanBusMessageId, Command, DownlinkMessage, FcReceivedCanBusMessage, FlightMode,
    IoBoardOutputMessage, IoBoardRole, Settings, TelemetryDataRate, TelemetryToPayloadMessage, ThrusterValveState,
    UplinkMessage,
};
use state_estimator::StateEstimator;
use telemetry::{
    AccelerometerId, BarometerId, BatteryId, FLASH_SCHEMA, GyroscopeId, LORA_SCHEMA, MagnetometerId, Metric,
    MetricSource, PressureSensorId, Representation, TelemetryMessageWriter, TemperatureSensorId, USB_SCHEMA,
};

//use crate::buzzer::Buzzer as BuzzerDriver;
use crate::drivers::sensors::*;
use crate::storage::*;
use crate::{BoardOutputs, BoardSensors};
//use crate::lora::*;
//use crate::usb::*;
//use crate::{can::*, BoardOutputs, BoardSensors};

const MAIN_LOOP_FREQUENCY: Hertz = Hertz::hz(1000);

pub struct Vehicle {
    pub time: core::num::Wrapping<u32>,
    mode: FlightMode,
    state_estimator: StateEstimator,
    settings: Settings,

    sensors: BoardSensors,
    outputs: BoardOutputs,

    can1: ((), ()),
    can2: ((), ()),

    usb: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),
    eth: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),
    lora: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),

    loop_runtime: f32,
    //gps: GPSHandle,
    //power: Power,

    //usb: UsbHandle,
    //radio: RadioHandle,
    flash: FlashHandle,
    //can: CanHandle,
}

#[embassy_executor::task]
pub async fn run(mut vehicle: Vehicle, mut iwdg: IndependentWatchdog<'static, IWDG1>) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
    defmt::info!("Starting main loop.");
    loop {
        vehicle.tick().await;
        iwdg.pet();
        ticker.next().await;
    }
}

impl Vehicle {
    #[allow(clippy::too_many_arguments)]
    pub fn init(
        mut sensors: BoardSensors,
        outputs: BoardOutputs,
        can1: ((), ()),
        can2: ((), ()),
        usb: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        eth: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        lora: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        //gps: GPSHandle,
        ////power: Power,
        flash: FlashHandle,
        //mut buzzer: Buzzer,
        settings: Settings,
    ) -> Self {
        //buzzer.apply_settings(&settings.drogue_output_settings, &settings.main_output_settings);

        //// TODO: new sensors
        //sensors.imu1.set_offsets(settings.gyro_offset, settings.acc_offset);
        //sensors.highg.set_offset(settings.acc2_offset);
        //sensors.mag.set_offset(settings.mag_offset);

        Self {
            time: core::num::Wrapping(0),
            mode: FlightMode::Idle,
            state_estimator: StateEstimator::new(MAIN_LOOP_FREQUENCY.0 as f32, settings.clone()),
            settings,

            sensors,
            outputs,

            can1,
            can2,
            usb,
            eth,
            lora,

            //gps,
            //power,
            flash,
            loop_runtime: 0.0,
        }
    }

    async fn tick(&mut self) {
        let start = Instant::now();

        // Query core sensors
        // TODO: should we separate these into separate tasks?
        // TODO: test and decide on correct future joining

        join5(
            self.sensors.imu1.tick(),
            self.sensors.imu2.tick(),
            self.sensors.imu3.tick(),
            self.sensors.highg.tick(),
            self.sensors.mag.tick(),
        )
        .await;

        join3(self.sensors.baro1.tick(), self.sensors.baro2.tick(), self.sensors.baro3.tick()).await;
        // self.power.tick();

        // Update state estimator
        // TODO: incorporate new sensors
        self.state_estimator.update(
            self.time,
            self.mode,
            self.sensors.imu1.gyroscope(),
            self.sensors.imu1.accelerometer(),
            self.sensors.highg.accelerometer(),
            self.sensors.mag.magnetometer(),
            self.sensors.baro1.altitude(),
            //self.sensors.gps.new_datum(),
            None,
        );
        // Switch to new mode if necessary
        // let arm_voltage = self.power.arm_voltage().unwrap_or(0);
        let arm_voltage = 0;
        if let Some(fm) = self.state_estimator.new_mode(arm_voltage) {
            self.switch_mode(fm);
        }

        self.receive();

        self.outputs.recovery_high.set_level((self.mode >= FlightMode::Armed).into());

        let elapsed = self.state_estimator.time_in_mode();
        let drogue_high =
            self.mode == FlightMode::RecoveryDrogue && self.settings.drogue_output_settings.currently_high(elapsed);
        let main_high =
            self.mode == FlightMode::RecoveryMain && self.settings.main_output_settings.currently_high(elapsed);
        self.outputs.recovery_lows.0.set_level(drogue_high.into());
        self.outputs.recovery_lows.1.set_level(main_high.into());

        let (r, y, g) = self.mode.led_state(self.time.0);
        self.outputs.leds.0.set_level((!r).into());
        self.outputs.leds.1.set_level((!y).into());
        self.outputs.leds.2.set_level((!g).into());

        //// Send valve commands via CAN bus
        //self.transmit_output_commands();
        self.transmit_and_store().await;

        // Increase time for next iteration
        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;

        // get CPU usage
        self.loop_runtime = (start.elapsed().as_micros() as f32) / 1000.0;
        // defmt::info!("loop_runtime: {}", self.loop_runtime);
    }

    fn receive(&mut self) {
        if let Ok(msg) = self.lora.1.try_receive() {
            self.handle_uplink_message(msg);
        }

        if let Ok(msg) = self.eth.1.try_receive() {
            self.handle_uplink_message(msg);
        }

        // TODO: usb
    }

    fn handle_uplink_message(&mut self, msg: UplinkMessage) {
        match msg {
            UplinkMessage::Heartbeat => {}
            // TODO: remove bootloader command?
            UplinkMessage::Command(cmd) => {
                if let Command::RebootToBootloader = cmd {
                    //reboot_to_bootloader()
                } else {
                    self.handle_command(cmd)
                }
            }
            //UplinkMessage::ReadFlash(adress, size) => {
            //    let _ = self.flash.read(adress, size);
            //}
            UplinkMessage::ReadSettings => {
                info!("UplinkMessage: ReadSettings");
                // let _ = self.usb.0.try_send(DownlinkMessage::Settings(self.settings.clone()));
                let _ = self.eth.0.try_send(DownlinkMessage::Settings(self.settings.clone()));
                // let _ = self.lora.0.try_send(DownlinkMessage::Settings(self.settings.clone()));
            }
            //UplinkMessage::WriteSettings(settings) => {
            //    let _ = self.flash.write_settings(settings);
            //}
            //UplinkMessage::ApplyLoRaSettings(_) => {}
            _ => {}
        }
    }

    fn handle_command(&mut self, cmd: Command) {
        info!("Received command: {:?}", Debug2Format(&cmd));
        match cmd {
            Command::Reboot => cortex_m::peripheral::SCB::sys_reset(),
            Command::RebootToBootloader => {}
            Command::SetFlightMode(fm) => self.switch_mode(fm),
            //Command::SetTransmitPower(txp) => self.radio.set_transmit_power(txp),
            //Command::EraseFlash => {
            //    let _ = self.flash.erase();
            //}
            _ => {}
        }
    }

    async fn transmit_and_store(&mut self) {
        let usb_msg =
            USB_SCHEMA.message(self, self.time.0).map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer));

        let lora_msg =
            LORA_SCHEMA.message(self, self.time.0).map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer));

        let flash_msg = FLASH_SCHEMA
            .message(self, self.time.0)
            .map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer));

        // Send telemetry via Ethernet
        if let Some(ref msg) = usb_msg {
            let _ = self.eth.0.try_send(msg.clone());
        }

        // Send telemetry via Usb

        if let Some(msg) = usb_msg {
            let _res = self.usb.0.try_send(msg);
            // match res {
            //     Ok(_) => defmt::info!("usb telemetry send successful"),
            //     Err(_) => defmt::info!("usb telemetry send failed"),
            // }
        }

        // Send telemetry via Lora
        if let Some(msg) = lora_msg {
            let _ = self.lora.0.try_send(msg);
        }

        // Store data in flash
        self.flash.tick().await;
        if self.mode >= FlightMode::ArmedLaunchImminent {
            if let Some(msg) = flash_msg {
                let _ = self.flash.write_message(msg);
            }
        }

        //// Broadcast telemetry to payloads
        //self.broadcast_can_telemetry();
    }

    fn switch_mode(&mut self, new_mode: FlightMode) {
        if new_mode == self.mode {
            return;
        }

        // We are going to or beyond Armed, switch to max tx power and arm ACS
        if new_mode >= FlightMode::Armed && self.mode < FlightMode::Armed {
            //self.radio.set_max_transmit_power();
            // TODO:
        }

        self.mode = new_mode;
        crate::FLIGHT_MODE_SIGNAL.signal(self.mode);
    }
}

// TODO: ideally, all implementations of MetricSource should be done with a derive macro.
impl ::telemetry::MetricSource for Vehicle {
    type Error = Infallible;

    fn write_metric<const N: usize>(
        &mut self,
        w: &mut ::telemetry::TelemetryMessageWriter<N>,
        metric: ::telemetry::Metric,
        repr: Representation,
    ) -> Result<(), Self::Error> {
        use Metric::*;

        match metric {
            FlightMode => w.write_enum(repr, self.mode as u8),
            // State estimator
            Orientation(_)
            | Elevation
            | Azimuth
            | AccelerationWorldSpace(_)
            | VelocityWorldSpace(_)
            | PositionWorldSpace(_)
            | Latitude
            | Longitude
            | GroundAltitudeASL
            | ApogeeAltitudeASL
            | GroundSpeed
            | KalmanStateCovariance(_, _) => self.state_estimator.write_metric(w, metric, repr),
            // Raw sensor values
            RawAngularVelocity(GyroscopeId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu1.gyroscope().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu1.accelerometer().unwrap_or_default())
            }
            RawAngularVelocity(GyroscopeId::ICM42688P, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu2.gyroscope().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::ICM42688P, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu2.accelerometer().unwrap_or_default())
            }
            RawAngularVelocity(GyroscopeId::ICM42670P, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu3.gyroscope().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::ICM42670P, dim) => {
                w.write_vector(repr, dim, &self.sensors.imu3.accelerometer().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::H3LIS331, dim) => {
                w.write_vector(repr, dim, &self.sensors.highg.accelerometer().unwrap_or_default())
            }
            RawMagneticFluxDensity(MagnetometerId::LIS3MDL, dim) => {
                w.write_vector(repr, dim, &self.sensors.mag.magnetometer().unwrap_or_default())
            }
            RawBarometricAltitude(BarometerId::MS5611) => {
                w.write_float(repr, self.sensors.baro1.altitude().unwrap_or_default())
            }
            Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611)) => {
                w.write_float(repr, self.sensors.baro1.pressure().unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611)) => {
                w.write_float(repr, self.sensors.baro1.temperature().unwrap_or_default())
            }
            RawBarometricAltitude(BarometerId::LPS22) => {
                w.write_float(repr, self.sensors.baro2.altitude().unwrap_or_default())
            }
            Pressure(PressureSensorId::FlightComputer(BarometerId::LPS22)) => {
                w.write_float(repr, self.sensors.baro2.pressure().unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Barometer(BarometerId::LPS22)) => {
                w.write_float(repr, self.sensors.baro2.temperature().unwrap_or_default())
            }
            RawBarometricAltitude(BarometerId::BMP580) => {
                w.write_float(repr, self.sensors.baro3.altitude().unwrap_or_default())
            }
            Pressure(PressureSensorId::FlightComputer(BarometerId::BMP580)) => {
                w.write_float(repr, self.sensors.baro3.pressure().unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Barometer(BarometerId::BMP580)) => {
                w.write_float(repr, self.sensors.baro3.temperature().unwrap_or_default())
            }
            //GpsFix => w.write_enum(repr, self.gps.fix().unwrap_or_default() as u8),
            //GpsAltitude => w.write_float(repr, self.gps.altitude().unwrap_or_default()),
            // Pressures
            // Temperatures
            Temperature(TemperatureSensorId::Battery(BatteryId::Avionics)) => {
                w.write_float(repr, self.sensors.baro1.temperature().unwrap_or_default())
            }
            // Voltages and Currents
            //BatteryVoltage(BatteryId::Avionics) => {
            //    w.write_float(repr, self.power.battery_voltage().unwrap_or_default())
            //}
            //BatteryCurrent(BatteryId::Avionics) => {
            //    w.write_float(repr, self.power.battery_current().unwrap_or_default())
            //}
            //SupplyVoltage => w.write_float(repr, self.power.charge_voltage().unwrap_or_default()),
            // Misc.
            CpuUtilization => w.write_float(repr, self.loop_runtime),
            //FlashPointer => w.write_float(repr, self.flash.pointer),
            _ => w.write_float(repr, 0.0),
        }
    }
}
