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
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::Mutex as BlMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embassy_time::{Duration, Ticker};

use defmt::*;

use shared_types::can::structure::{Can2aFrame, MessageKind};
use shared_types::can::{CanMessage, SubsystemInfo, TelemetryBroadcast, engine::EngineState};
use shared_types::{
    AcsMode, Command, DownlinkMessage, FlightMode, ProcedureStep, Settings, TelemetryDataRate, ThrusterValveState,
    UplinkMessage,
};
use state_estimator::StateEstimator;
use telemetry::{
    AccelerometerId, BarometerId, BatteryId, FLASH_SCHEMA, GyroscopeId, LORA_SCHEMA, MagnetometerId, Metric,
    MetricSource, PressureSensorId, Representation, TelemetryMessageWriter, TemperatureSensorId, USB_SCHEMA, ValveId,
};

use crate::board::load_outputs;
use crate::can::CanTxPublisher;
//use crate::buzzer::Buzzer as BuzzerDriver;
use crate::drivers::sensors::*;
use crate::lora::TypedLoraLinkSettings;
use crate::storage::*;
use crate::subsystems::engine::EngineSubsystem;
use crate::subsystems::ereg::EregSubsystem;
use crate::{BoardOutputs, BoardSensors, LoadOutputs};
//use crate::lora::*;
//use crate::usb::*;
//use crate::{can::*, BoardOutputs, BoardSensors};

const MAIN_LOOP_FREQUENCY: Hertz = Hertz::hz(1000);

pub struct Subsystems {
    pub engine: EngineSubsystem,
    pub ereg: EregSubsystem,
}

pub struct Vehicle {
    pub time: core::num::Wrapping<u32>,
    mode: FlightMode,
    state_estimator: StateEstimator,
    settings: Settings,
    lora_downlink_settings:
        &'static embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,

    sensors: BoardSensors,
    load_outputs: &'static Mutex<CriticalSectionRawMutex, LoadOutputs>,
    board_leds: (Output<'static>, Output<'static>, Output<'static>),

    can1: (crate::can::CanTxPublisher, crate::can::CanRxSubscriber),
    can2: (crate::can::CanTxPublisher, crate::can::CanRxSubscriber),
    #[cfg(feature = "usb")]
    usb: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),
    #[cfg(feature = "ethernet")]
    eth: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),
    lora: (
        Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
        Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
    ),
    subsystems: Subsystems,
    // lora_power_sig: &'static Signal<CriticalSectionRawMutex, u8>,
    lora_uplink_rssi: &'static BlMutex<CriticalSectionRawMutex, i16>,
    main_recovery_sig: &'static Signal<CriticalSectionRawMutex, bool>,
    parabreaks_sig: &'static Signal<CriticalSectionRawMutex, bool>,

    loop_runtime: f32,
    //gps: GPSHandle,
    //power: Power,

    //usb: UsbHandle,
    //radio: RadioHandle,
    flash: FlashHandle,
    //can: CanHandle,
    display_procedure_step: ProcedureStep,
}

#[embassy_executor::task]
pub async fn run(mut vehicle: Vehicle, mut iwdg: IndependentWatchdog<'static, IWDG1>) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
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
        // outputs: BoardOutputs,
        board_leds: (Output<'static>, Output<'static>, Output<'static>),
        load_outputs: &'static Mutex<CriticalSectionRawMutex, LoadOutputs>,
        can1: (crate::can::CanTxPublisher, crate::can::CanRxSubscriber),
        can2: (crate::can::CanTxPublisher, crate::can::CanRxSubscriber),
        #[cfg(feature = "usb")] usb: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        #[cfg(feature = "ethernet")] eth: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        lora: (
            Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
            Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
        ),
        subsystems: Subsystems,

        main_recovery_sig: &'static Signal<CriticalSectionRawMutex, bool>,
        parabreaks_sig: &'static Signal<CriticalSectionRawMutex, bool>,

        //gps: GPSHandle,
        ////power: Power,
        flash: FlashHandle,
        //mut buzzer: Buzzer,
        settings: Settings,
        lora_downlink_settings: &'static BlMutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,
        rssi_glob: &'static BlMutex<CriticalSectionRawMutex, i16>,
        // lora_power_sig: &'static Signal<CriticalSectionRawMutex, u8>,
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
            lora_downlink_settings,
            lora_uplink_rssi: rssi_glob,
            // lora_power_sig,
            sensors,
            board_leds,
            load_outputs,
            can1,
            can2,
            subsystems,
            #[cfg(feature = "usb")]
            usb,
            #[cfg(feature = "ethernet")]
            eth,
            lora,
            main_recovery_sig,
            parabreaks_sig,

            //gps,
            //power,
            flash,
            loop_runtime: 0.0,
            display_procedure_step: ProcedureStep::default(),
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
        // let arm_voltage = 0;
        // if let Some(fm) = self.state_estimator.new_mode(arm_voltage) {
        //     self.switch_mode(fm);
        // }
        //
        // self.receive_uplink_message();
        // self.receive_can();
        //
        // // TODO: check arm
        // unsafe {
        //     self.load_outputs.lock_mut(|o| o.arm());
        // }
        // // self.outputs.load_output_arm.set_level((self.mode >= FlightMode::Armed).into());
        //
        // // TODO: re-enable timing checks for recovery // recovery invocation moved to fn switch_mode
        // let _elapsed = self.state_estimator.time_in_mode();
        // let (r, y, g) = self.mode.led_state(self.time.0);
        // self.board_leds.0.set_level((!r).into());
        // self.board_leds.1.set_level((!y).into());
        // self.board_leds.2.set_level((!g).into());
        //
        // //// Send valve commands via CAN bus
        // //self.transmit_output_commands();
        // self.transmit_and_store().await;
        //
        // // Increase time for next iteration
        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;
        //
        // // get CPU usage
        // self.loop_runtime = (start.elapsed().as_micros() as f32) / 1000.0;
        // // defmt::info!("loop_runtime: {}", self.loop_runtime);
    }

    fn receive_uplink_message(&mut self) {
        if let Ok(msg) = self.lora.1.try_receive() {
            info!("receive lora uplink message");
            self.handle_uplink_message(msg);
        }

        #[cfg(feature = "ethernet")]
        if let Ok(msg) = self.eth.1.try_receive() {
            self.handle_uplink_message(msg);
        }
        #[cfg(feature = "usb")]
        if let Ok(msg) = self.usb.1.try_receive() {
            self.handle_uplink_message(msg);
        }
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
                #[cfg(feature = "ethernet")]
                let _ = self.eth.0.try_send(DownlinkMessage::Settings(self.settings.clone()));
                #[cfg(feature = "usb")]
                let _ = self.usb.0.try_send(DownlinkMessage::Settings(self.settings.clone()));

                // FIXME: tried re-enabling, but did not work
                let _ = self.lora.0.try_send(DownlinkMessage::Settings(self.settings.clone()));
            }
            UplinkMessage::WriteSettings(settings) => {
                info!("UplinkMessage: WriteSettings");
                // NOTE: this reboots the processor
                let _ = self.flash.write_settings(settings);
            }
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
            Command::SetDisplayStep(step) => self.switch_procedure_step(step),
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
        #[cfg(feature = "ethernet")]
        if let Some(ref msg) = usb_msg {
            let _ = self.eth.0.try_send(msg.clone());
        }

        // Send telemetry via Usb
        #[cfg(feature = "usb")]
        if let Some(msg) = usb_msg {
            let res = self.usb.0.try_send(msg);
            match res {
                Ok(_) => defmt::info!("usb telemetry send successful"),
                Err(_) => defmt::info!("usb telemetry send failed"),
            }
        }

        // Send telemetry via Lora
        if let Some(msg) = lora_msg {
            let _ = self.lora.0.try_send(msg);
        }

        // Store data in flash
        self.flash.tick().await;
        if self.mode >= FlightMode::ArmedLaunchImminent {
            // FIXME: the impl has a bug
            // if let Some(msg) = flash_msg {
            //     let _ = self.flash.write_message(msg);
            // }
        }

        // Broadcast telemetry to payloads
        if self.time.0 % 500 == 0 {
            self.broadcast_can_telemetry();
        }
    }

    /// Reads all CAN messages (CAN1) from the buffer and processes them.
    /// For now only uses CAN1.
    fn receive_can(&mut self) {
        while let Some(msg) = self.can1.1.try_next_message_pure() {
            self.subsystems.ereg.process_message(msg);
            self.subsystems.engine.process_message(msg);
        }
    }

    fn broadcast_can_telemetry(&mut self) {
        self.can1.0.publish_immediate(CanMessage::Telem(TelemetryBroadcast::FlightMode(self.mode)));
        self.can1
            .0
            .publish_immediate(CanMessage::Telem(TelemetryBroadcast::ProcedureStep(self.display_procedure_step)));
        let msg = CanMessage::Telem(TelemetryBroadcast::FlightMode(self.mode));
        let id: u16 = Can2aFrame::from(msg).id.into();
        warn!("can telemetry with id: {} and payload...", id);
    }

    fn switch_mode(&mut self, new_mode: FlightMode) {
        if new_mode == self.mode {
            return;
        }

        // We are going to or beyond Armed, switch to max tx power and arm ACS
        if new_mode >= FlightMode::Armed && self.mode < FlightMode::Armed {
            // TODO: lora max power
            //self.radio.set_max_transmit_power();
            // TODO: think about arm AND de-arm
            unsafe {
                self.load_outputs.lock_mut(|o| o.arm());
            }
            info!("Arming Recovery");
        }
        // TODO:: move maybe
        if new_mode == FlightMode::RecoveryDrogue {
            self.parabreaks_sig.signal(true);
        }
        if new_mode == FlightMode::RecoveryMain {
            self.main_recovery_sig.signal(true);
        }

        self.mode = new_mode;
        crate::BUZZER_SIGNAL.signal(self.mode);
        self.broadcast_can_telemetry();
    }

    /// For now only broadcasts to fins.
    /// Later this should also control the rocket.
    fn switch_procedure_step(&mut self, new_step: ProcedureStep) {
        if new_step != self.display_procedure_step {
            self.broadcast_can_telemetry();
        }
        self.display_procedure_step = new_step;
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
            UplinkRssi => w.write_float(repr, self.lora_uplink_rssi.lock(|x| *x)),

            // --- Engine
            Pressure(PressureSensorId::CombustionChamber) => w.write_float(
                repr,
                self.subsystems.engine.get_engine_measurements().unwrap_or_default().pressure_combustion_chamber,
            ),
            Pressure(PressureSensorId::OxidizerTank) => {
                w.write_float(repr, self.subsystems.engine.get_engine_measurements().unwrap_or_default().pressure_ox)
            }
            ValveState(ValveId::MainValve) => {
                w.write_float(repr, self.subsystems.engine.get_engine_measurements().unwrap_or_default().main_valve)
            }
            ValveState(ValveId::FillAndDumpValve) => w.write_float(
                repr,
                self.subsystems.engine.get_engine_measurements().unwrap_or_default().fill_and_dump_valve,
            ),
            Temperature(TemperatureSensorId::OxidizerTank) => {
                w.write_float(repr, self.subsystems.engine.get_engine_measurements().unwrap_or_default().temp_ox)
            }
            // ---
            _ => w.write_float(repr, 0.0),
        }
    }
}
