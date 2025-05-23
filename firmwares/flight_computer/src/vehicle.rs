//! Main flight logic for flight computer.

use core::convert::Infallible;
use core::num::Wrapping;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
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
    AccelerometerId, BarometerId, BatteryId, GyroscopeId, MagnetometerId, Metric, MetricSource, PressureSensorId,
    Representation, TelemetryMessageWriter, TemperatureSensorId, FLASH_SCHEMA, LORA_SCHEMA, USB_SCHEMA,
};

use crate::buzzer::Buzzer as BuzzerDriver;
use crate::can::*;
use crate::drivers::sensors::*;
use crate::flash::*;
use crate::lora::*;
use crate::usb::*;

type SpiInst = Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>;
type Imu = LSM6<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB15>>>;
type Accelerometer = H3LIS331DL<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PA4>>>;
type Magnetometer = LIS3MDL<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB10>>>;
type Barometer = MS5611<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PC6>>>;
type Power = PowerMonitor<ADC1, PB0, PC5, PC4>;

type RadioHandle = Radio<
    SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PA1>>,
    Input<'static, PC0>,
    Input<'static, PC1>,
>;

type LEDs = (Output<'static, PC13>, Output<'static, PC14>, Output<'static, PC15>);
type Buzzer = BuzzerDriver<TIM3>;
type Recovery = (Output<'static, PC8>, Output<'static, PC9>);

const MAIN_LOOP_FREQUENCY: Hertz = Hertz::hz(1000);

pub struct Vehicle {
    pub time: core::num::Wrapping<u32>,
    // sensors
    imu: Imu,
    acc: Accelerometer,
    mag: Magnetometer,
    baro: Barometer,
    gps: GPSHandle,
    power: Power,
    // other peripherals
    usb: UsbHandle,
    radio: RadioHandle,
    flash: FlashHandle,
    can: CanHandle,
    // outputs
    leds: LEDs,
    buzzer: Buzzer,
    recovery: Recovery,
    // vehicle state
    state_estimator: StateEstimator,
    mode: FlightMode,
    loop_runtime: f32,
    settings: Settings,
    data_rate: TelemetryDataRate,
    // IO board state
    last_acs_message: Option<(Wrapping<u32>, u16, i16, i8)>,
    last_recovery_message: Option<(Wrapping<u32>, u16, i16, i8)>,
    last_payload_message: Option<(Wrapping<u32>, u16, i16, i8)>,
    // Acs
    acs_mode: AcsMode,
    last_manual_thruster_input: Option<(Wrapping<u32>, ThrusterValveState)>,
    last_thruster_valve_state: ThrusterValveState,
    acs_tank_pressure: Option<(Wrapping<u32>, f32)>,
    acs_regulator_pressure: Option<(Wrapping<u32>, f32)>,
    acs_accel_valve_pressure: Option<(Wrapping<u32>, f32)>,
    acs_decel_valve_pressure: Option<(Wrapping<u32>, f32)>,
    // Recovery (and also payload)
    recovery_pressure: Option<(Wrapping<u32>, f32)>,
    main_release_sensor: Option<(Wrapping<u32>, bool)>,
    camera_state: [bool; 3], // R0, R1, P (TODO: this is awful)
    // Fins
    last_fin_message: [Option<Wrapping<u32>>; 3],
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

        // clear io module status if too long ago
        const THRESHOLD: u32 = 1000;

        for io_module in [
            &mut self.last_acs_message,
            &mut self.last_recovery_message,
            &mut self.last_payload_message,
        ] {
            if io_module.map(|(t, _, _, _)| (self.time - t).0 > THRESHOLD).unwrap_or(false) {
                *io_module = None;
            }
        }

        for pressure_sensor in [
            &mut self.acs_tank_pressure,
            &mut self.acs_regulator_pressure,
            &mut self.acs_accel_valve_pressure,
            &mut self.acs_decel_valve_pressure,
            &mut self.recovery_pressure,
        ] {
            if pressure_sensor.map(|(t, _)| (self.time - t).0 > THRESHOLD).unwrap_or(false) {
                *pressure_sensor = None;
            }
        }

        if self.main_release_sensor.map(|(t, _v)| (self.time - t).0 > THRESHOLD).unwrap_or(false) {
            self.main_release_sensor = None;
        }

        // fins_present: Some([
        //     self.last_fin_message[0].map(|t| (self.time - t).0 < THRESHOLD).unwrap_or(false),
        //     self.last_fin_message[1].map(|t| (self.time - t).0 < THRESHOLD).unwrap_or(false),
        //     self.last_fin_message[2].map(|t| (self.time - t).0 < THRESHOLD).unwrap_or(false),
        // ]),
        //
        // camera_state: Some(self.camera_state.clone()),

        let acs_tank_pressure = self.acs_tank_pressure.map(|(_t, v)| v).unwrap_or_default();

        match metric {
            FlightMode => w.write_enum(repr, self.mode as u8),
            ApogeeAltitudeASL => {
                w.write_float(repr, self.state_estimator.apogee_asl(acs_tank_pressure).unwrap_or_default())
            }
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
            | GroundSpeed
            | KalmanStateCovariance(_, _) => self.state_estimator.write_metric(w, metric, repr),
            // Raw sensor values
            RawAngularVelocity(GyroscopeId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.imu.gyroscope().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::LSM6DSR, dim) => {
                w.write_vector(repr, dim, &self.imu.accelerometer().unwrap_or_default())
            }
            RawAcceleration(AccelerometerId::H3LIS331, dim) => {
                w.write_vector(repr, dim, &self.acc.accelerometer().unwrap_or_default())
            }
            RawMagneticFluxDensity(MagnetometerId::LIS3MDL, dim) => {
                w.write_vector(repr, dim, &self.mag.magnetometer().unwrap_or_default())
            }
            RawBarometricAltitude(BarometerId::MS5611) => w.write_float(repr, self.baro.altitude().unwrap_or_default()),
            Pressure(PressureSensorId::FlightComputer(BarometerId::MS5611)) => {
                w.write_float(repr, self.baro.pressure().unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Barometer(BarometerId::MS5611)) => {
                w.write_float(repr, self.baro.temperature().unwrap_or_default())
            }
            GpsFix => w.write_enum(repr, self.gps.fix().unwrap_or_default() as u8),
            GpsAltitude => w.write_float(repr, self.gps.altitude().unwrap_or_default()),
            // Pressures
            Pressure(PressureSensorId::AcsTank) => w.write_float(repr, acs_tank_pressure),
            Pressure(PressureSensorId::AcsPostRegulator) => {
                w.write_float(repr, self.acs_regulator_pressure.map(|(_t, v)| v).unwrap_or_default())
            }
            Pressure(PressureSensorId::AcsValveAccel) => {
                w.write_float(repr, self.acs_accel_valve_pressure.map(|(_t, v)| v).unwrap_or_default())
            }
            Pressure(PressureSensorId::AcsValveDecel) => {
                w.write_float(repr, self.acs_decel_valve_pressure.map(|(_t, v)| v).unwrap_or_default())
            }
            Pressure(PressureSensorId::RecoveryChamber) => {
                w.write_float(repr, self.recovery_pressure.map(|(_t, v)| v).unwrap_or_default())
            }
            Pressure(PressureSensorId::MainRelease) => {
                w.write_float(repr, self.main_release_sensor.map(|(_t, v)| v).unwrap_or_default())
            }
            // Temperatures
            Temperature(TemperatureSensorId::Acs) => {
                w.write_float(repr, self.last_acs_message.map(|(_, _, _, t)| t).unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Recovery) => {
                w.write_float(repr, self.last_recovery_message.map(|(_, _, _, t)| t).unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Payload) => {
                w.write_float(repr, self.last_payload_message.map(|(_, _, _, t)| t).unwrap_or_default())
            }
            Temperature(TemperatureSensorId::Battery(BatteryId::Avionics)) => {
                w.write_float(repr, self.baro.temperature().unwrap_or_default())
            }
            // Voltages and Currents
            BatteryVoltage(BatteryId::Avionics) => {
                w.write_float(repr, self.power.battery_voltage().unwrap_or_default())
            }
            BatteryCurrent(BatteryId::Avionics) => {
                w.write_float(repr, self.power.battery_current().unwrap_or_default())
            }
            SupplyVoltage => w.write_float(repr, self.power.charge_voltage().unwrap_or_default()),
            BatteryVoltage(BatteryId::Acs) => {
                w.write_float(repr, self.last_acs_message.map(|(_, v, _, _)| v).unwrap_or_default())
            }
            BatteryCurrent(BatteryId::Acs) => {
                w.write_float(repr, self.last_acs_message.map(|(_, _, c, _)| c).unwrap_or_default())
            }
            BatteryVoltage(BatteryId::Recovery) => {
                w.write_float(repr, self.last_recovery_message.map(|(_, v, _, _)| v).unwrap_or_default())
            }
            BatteryCurrent(BatteryId::Recovery) => {
                w.write_float(repr, self.last_recovery_message.map(|(_, _, c, _)| c).unwrap_or_default())
            }
            BatteryVoltage(BatteryId::Payload) => {
                w.write_float(repr, self.last_payload_message.map(|(_, v, _, _)| v).unwrap_or_default())
            }
            BatteryCurrent(BatteryId::Payload) => {
                w.write_float(repr, self.last_payload_message.map(|(_, _, c, _)| c).unwrap_or_default())
            }
            // Misc.
            CpuUtilization => w.write_float(repr, self.loop_runtime),
            FlashPointer => w.write_float(repr, self.flash.pointer),
            UplinkRssi => w.write_float(repr, self.radio.trx.rssi),
            TransmitPower => w.write_enum(repr, self.radio.transmit_power as u8),
            _ => w.write_float(repr, 0.0),
        }
    }
}

#[embassy_executor::task]
pub async fn run(mut vehicle: Vehicle, mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
    defmt::info!("Starting main loop.");
    loop {
        vehicle.tick().await;
        iwdg.pet();
        ticker.next().await;
    }
}

impl Vehicle {
    pub fn init(
        mut imu: Imu,
        mut acc: Accelerometer,
        mut mag: Magnetometer,
        baro: Barometer,
        gps: GPSHandle,
        power: Power,
        usb: UsbHandle,
        mut radio: RadioHandle,
        flash: FlashHandle,
        can: CanHandle,
        leds: LEDs,
        mut buzzer: Buzzer,
        recovery: Recovery,
        settings: Settings,
    ) -> Self {
        buzzer.apply_settings(&settings.drogue_output_settings, &settings.main_output_settings);
        radio.apply_settings(&settings.lora);
        imu.set_offsets(settings.gyro_offset, settings.acc_offset);
        acc.set_offset(settings.acc2_offset);
        mag.set_offset(settings.mag_offset);

        let data_rate = settings.default_data_rate;

        Self {
            time: core::num::Wrapping(0),

            imu,
            acc,
            mag,
            baro,
            gps,
            power,

            usb,
            radio,
            flash,
            can,

            leds,
            buzzer,
            recovery,

            state_estimator: StateEstimator::new(MAIN_LOOP_FREQUENCY.0 as f32, settings.clone()),
            mode: FlightMode::Idle,

            loop_runtime: 0.0,
            settings,
            data_rate,

            last_acs_message: None,
            last_recovery_message: None,
            last_payload_message: None,

            acs_mode: AcsMode::Disabled,
            last_manual_thruster_input: None,
            last_thruster_valve_state: ThrusterValveState::Closed,

            acs_tank_pressure: None,
            acs_regulator_pressure: None,
            acs_accel_valve_pressure: None,
            acs_decel_valve_pressure: None,
            recovery_pressure: None,
            main_release_sensor: None,

            camera_state: [false; 3],

            last_fin_message: [None; 3],
        }
    }

    async fn tick(&mut self) {
        if self.time.0 % 5000 == 0 {
            let alt_baro = self.baro.altitude().unwrap_or_default() * 100.0;
            defmt::info!("t={}, alt_baro={}cm", self.time.0, alt_baro as u32);
        }

        let start = Instant::now();

        // Query core sensors
        // TODO: should we separate these into separate tasks?
        self.imu.tick().await;
        self.acc.tick().await;
        self.mag.tick().await;
        self.baro.tick().await;
        self.power.tick();

        // Handle incoming CAN messages
        if let Some(msg) = self.can.receive() {
            // TODO: do we still want to store all can messages?
            match msg {
                FcReceivedCanBusMessage::BatteryTelemetry(_id, bat_msg) => self.power.handle_battery_can_msg(bat_msg),
                FcReceivedCanBusMessage::IoBoardSensor(role, id, sensor_msg) => match (role, id) {
                    (IoBoardRole::Acs, 0) => {
                        self.acs_tank_pressure = self
                            .settings
                            .acs_tank_pressure_sensor_settings
                            .apply(sensor_msg.i2c_sensors[0])
                            .map(|v| (self.time, v));
                        self.acs_regulator_pressure = self
                            .settings
                            .acs_regulator_pressure_sensor_settings
                            .apply(sensor_msg.i2c_sensors[1])
                            .map(|v| (self.time, v));
                        self.acs_accel_valve_pressure = self
                            .settings
                            .acs_accel_valve_pressure_sensor_settings
                            .apply(sensor_msg.i2c_sensors[2])
                            .map(|v| (self.time, v));
                        self.acs_decel_valve_pressure = self
                            .settings
                            .acs_decel_valve_pressure_sensor_settings
                            .apply(sensor_msg.i2c_sensors[3])
                            .map(|v| (self.time, v));
                    }
                    (IoBoardRole::Recovery, 0) => {
                        self.recovery_pressure = self
                            .settings
                            .recovery_pressure_sensor_settings
                            .apply(sensor_msg.i2c_sensors[0])
                            .map(|v| (self.time, v));
                        self.main_release_sensor = sensor_msg.i2c_sensors[3].map(|(v, _)| (self.time, (v & 0b1) == 0));
                    }
                    _ => {}
                },
                FcReceivedCanBusMessage::IoBoardPower(role, power_msg) => {
                    use num_traits::Float;
                    const B: f32 = 3380.0;
                    const B_INV: f32 = 1.0 / B;
                    const T0_INV: f32 = 1.0 / (273.15 + 25.0);
                    const R0: f32 = 10_000.0;
                    let r = power_msg.thermistor_resistance as f32;
                    let temperature = 1.0 / (T0_INV + B_INV * (r / R0).ln());
                    let temperature = ((temperature - 273.15) * 2.0) as i8;
                    let msg = (self.time, power_msg.output_voltage, power_msg.output_current, temperature);

                    match role {
                        IoBoardRole::Acs => {
                            self.last_acs_message = Some(msg);
                        }
                        IoBoardRole::Recovery => {
                            self.last_recovery_message = Some(msg);
                        }
                        IoBoardRole::Payload => {
                            self.last_payload_message = Some(msg);
                        }
                    }
                }
                FcReceivedCanBusMessage::FinBoardData(fin, _, _) => {
                    let fin = fin as usize;
                    if fin < self.last_fin_message.len() {
                        self.last_fin_message[fin] = Some(self.time);
                    }
                }
            }
        }

        // Update state estimator
        self.state_estimator.update(
            self.time,
            self.mode,
            self.imu.gyroscope(),
            self.imu.accelerometer(),
            self.acc.accelerometer(),
            self.mag.magnetometer(),
            self.baro.altitude(),
            self.gps.new_datum(),
        );

        // Switch to new mode if necessary
        let arm_voltage = self.power.arm_voltage().unwrap_or(0);
        if let Some(fm) = self.state_estimator.new_mode(arm_voltage) {
            self.switch_mode(fm);
        }

        // Process incoming commands, both from USB...
        if let Some(msg) = self.usb.next_uplink_message() {
            match msg {
                UplinkMessage::Heartbeat => {}
                // TODO: remove bootloader command?
                UplinkMessage::Command(cmd) => {
                    if let Command::RebootToBootloader = cmd {
                        //reboot_to_bootloader()
                    } else {
                        self.handle_command(cmd).await
                    }
                }
                UplinkMessage::ReadFlash(adress, size) => {
                    let _ = self.flash.read(adress, size);
                }
                UplinkMessage::ReadSettings => self.usb.send_message(DownlinkMessage::Settings(self.settings.clone())),
                UplinkMessage::WriteSettings(settings) => {
                    let _ = self.flash.write_settings(settings);
                }
                UplinkMessage::ApplyLoRaSettings(_) => {}
            }
        }

        // ... and via LoRa
        if let Some(cmd) = self.radio.tick(self.time.0).await {
            self.handle_command(cmd).await;
        }

        // Set output according to flight mode
        let elapsed = self.state_estimator.time_in_mode();
        let drogue_high =
            self.mode == FlightMode::RecoveryDrogue && self.settings.drogue_output_settings.currently_high(elapsed);
        let main_high =
            self.mode == FlightMode::RecoveryMain && self.settings.main_output_settings.currently_high(elapsed);
        self.recovery.0.set_level(drogue_high.into());
        self.recovery.1.set_level(main_high.into());

        let (r, y, g) = self.mode.led_state(self.time.0);
        self.leds.0.set_level((!r).into());
        self.leds.1.set_level((!y).into());
        self.leds.2.set_level((!g).into());

        // Send valve commands via CAN bus
        self.transmit_output_commands();

        // Update buzzer
        self.buzzer.tick(self.time.0, self.power.battery_status());

        // Send telemetry via USB
        if let Some(msg) = self.next_usb_telem() {
            self.usb.send_message(msg);
        }

        // Send telemetry via Lora
        if let Some(msg) = self.next_lora_telem() {
            if let Err(e) = self.radio.send(msg).await {
                error!("Failed to send downlink message: {:?}", Debug2Format(&e));
            }
        }

        // Store data in flash
        self.flash.tick().await;
        if self.mode >= FlightMode::ArmedLaunchImminent {
            if let Some(msg) = self.next_flash_telem() {
                let _ = self.flash.write_message(msg);
            }
        }

        // Broadcast telemetry to payloads
        self.broadcast_can_telemetry();

        // Increase time for next iteration
        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;

        // get CPU usage
        self.loop_runtime = (start.elapsed().as_micros() as f32) / 1000.0;
    }

    fn transmit_output_commands(&mut self) {
        // We send ACS valve output commands every 50Hz
        if self.time.0 % 20 == 0 {
            let valve_state = match self.acs_mode {
                AcsMode::Disabled => ThrusterValveState::Closed,
                AcsMode::Auto => {
                    self.state_estimator.thruster_valve(self.acs_tank_pressure.map(|(_t, v)| v).unwrap_or(300.0))
                }
                AcsMode::Manual => match self.last_manual_thruster_input {
                    Some((time, state)) => {
                        if (self.time - time).0 < 450 {
                            // TODO
                            state
                        } else {
                            self.last_manual_thruster_input = None;
                            ThrusterValveState::Closed
                        }
                    }
                    None => ThrusterValveState::Closed,
                },
            };

            self.last_thruster_valve_state = valve_state;

            let (accel, decel) = match valve_state {
                ThrusterValveState::Closed => (false, false),
                ThrusterValveState::OpenAccel => (true, false),
                ThrusterValveState::OpenDecel => (false, true),
                ThrusterValveState::OpenBoth => (true, true),
            };

            let mut outputs: [bool; 8] = [false; 8];
            outputs[0] = accel;
            outputs[1] = accel;
            outputs[2] = decel;
            outputs[3] = decel;
            let msg = IoBoardOutputMessage { outputs };
            let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardCommand(IoBoardRole::Acs, 0));
            self.can.transmit(id, msg);
        }

        // Recovery cameras
        if self.time.0 % 500 == 210 {
            let mut outputs: [bool; 8] = [false; 8];
            outputs[0] = self.camera_state[0];
            outputs[1] = self.camera_state[0];
            outputs[2] = self.camera_state[1];
            outputs[3] = self.camera_state[1];
            let msg = IoBoardOutputMessage { outputs };
            let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardCommand(IoBoardRole::Recovery, 0));
            self.can.transmit(id, msg);
        }

        // Payload cameras
        if self.time.0 % 500 == 410 {
            let mut outputs: [bool; 8] = [false; 8];
            outputs[0] = self.camera_state[2];
            outputs[1] = self.camera_state[2];
            outputs[2] = self.camera_state[2];
            outputs[3] = self.camera_state[2];
            let msg = IoBoardOutputMessage { outputs };
            let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardCommand(IoBoardRole::Payload, 0));
            self.can.transmit(id, msg);
        }
    }

    fn broadcast_can_telemetry(&mut self) {
        if self.time.0 % 100 != 0 {
            return;
        }

        let msg = TelemetryToPayloadMessage {
            time: self.time.0,
            mode: self.mode,
            altitude: (self.state_estimator.altitude_asl() * 10.0) as u16,
        };

        let (id, msg) = msg.to_frame(CanBusMessageId::TelemetryBroadcast(0));
        self.can.transmit(id, msg);
    }

    async fn handle_command(&mut self, cmd: Command) {
        info!("Received command: {:?}", Debug2Format(&cmd));
        match cmd {
            Command::Reboot => cortex_m::peripheral::SCB::sys_reset(),
            Command::RebootToBootloader => {}
            Command::SetFlightMode(fm) => self.switch_mode(fm),
            Command::SetTransmitPower(txp) => self.radio.set_transmit_power(txp),
            Command::SetDataRate(dr) => self.data_rate = dr,
            Command::SetAcsMode(am) => self.acs_mode = am,
            Command::SetAcsValveState(vs) => {
                if self.acs_mode != AcsMode::Disabled {
                    // We've interferred, set mode to manual.
                    self.acs_mode = AcsMode::Manual;
                    self.last_manual_thruster_input = Some((self.time, vs));
                }
            }
            Command::SetIoModuleOutput(role, output_id, state) if role == IoBoardRole::Recovery && output_id <= 1 => {
                self.camera_state[output_id as usize] = state;
            }
            Command::SetIoModuleOutput(role, _output_id, state) if role == IoBoardRole::Payload => {
                self.camera_state[2] = state;
            }
            Command::SetIoModuleOutput(_role, _output_id, _state) => {}
            Command::EraseFlash => {
                let _ = self.flash.erase();
            }
        }
    }

    fn switch_mode(&mut self, new_mode: FlightMode) {
        if new_mode == self.mode {
            return;
        }

        // We are going to or beyond Armed, switch to max tx power and arm ACS
        if new_mode >= FlightMode::Armed && self.mode < FlightMode::Armed {
            self.radio.set_max_transmit_power();
            self.acs_mode = AcsMode::Auto;
        }

        if new_mode >= FlightMode::ArmedLaunchImminent && self.mode < FlightMode::ArmedLaunchImminent {
            self.camera_state = [true; 3];
        }

        self.mode = new_mode;
        self.buzzer.switch_mode(self.time.0, new_mode);
    }

    fn next_usb_telem(&mut self) -> Option<DownlinkMessage> {
        USB_SCHEMA.message(self, self.time.0).map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer))
    }

    fn next_lora_telem(&mut self) -> Option<DownlinkMessage> {
        LORA_SCHEMA.message(self, self.time.0).map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer))
    }

    fn next_flash_telem(&mut self) -> Option<DownlinkMessage> {
        FLASH_SCHEMA
            .message(self, self.time.0)
            .map(|buffer| DownlinkMessage::Telemetry(self.time.0, buffer))
    }
}
