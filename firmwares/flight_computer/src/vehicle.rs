//! Main flight logic for flight computer.

use alloc::collections::VecDeque;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::gpio::{Output, Input};
use embassy_stm32::peripherals::*;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Instant;
use embassy_time::{Ticker, Duration};

use defmt::*;

use state_estimator::StateEstimator;
use shared_types::*;

use crate::buzzer::Buzzer as BuzzerDriver;
use crate::can::*;
use crate::drivers::sensors::*;
use crate::lora::*;
use crate::flash::*;
use crate::usb::*;

type SpiInst = Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>;
type Imu = LSM6<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB15>>>;
type Accelerometer = H3LIS331DL<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PA4>>>;
type Magnetometer = LIS3MDL<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PB10>>>;
type Barometer = MS5611<SpiDevice<'static, CriticalSectionRawMutex, SpiInst, Output<'static, PC6>>>;
type Power = PowerMonitor<ADC1, PB0, PC5, PC4>;

type RadioHandle = Radio<SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI1, DMA2_CH3, DMA2_CH2>, Output<'static, PA1>>, Input<'static, PC0>,Input<'static, PC1>>;

type LEDs = (Output<'static, PC13>, Output<'static, PC14>, Output<'static, PC15>);
type Buzzer = BuzzerDriver<TIM3>;
type Recovery = (Output<'static, PC8>, Output<'static, PC9>);

const RUNTIME_HISTORY_LEN: usize = 200;
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
    loop_runtime_history: VecDeque<f32>,
    settings: Settings,
    data_rate: TelemetryDataRate,
    // Acs
    acs_mode: AcsMode,
    last_manual_thruster_input: Option<(ThrusterValveState, core::num::Wrapping<u32>)>,
    last_thruster_valve_state: ThrusterValveState,
}

impl Into<VehicleState> for &mut Vehicle {
    fn into(self) -> VehicleState {
        VehicleState {
            time: self.time.0,
            mode: Some(self.mode),
            orientation: self.state_estimator.orientation,
            vertical_speed: Some(self.state_estimator.vertical_speed()),
            vertical_accel: Some(self.state_estimator.vertical_acceleration()),
            altitude_asl: Some(self.state_estimator.altitude_asl()),
            altitude_ground_asl: Some(self.state_estimator.altitude_ground),
            apogee_asl: self.state_estimator.apogee_asl(),

            gyroscope: self.imu.gyroscope(),
            accelerometer1: self.imu.accelerometer(),
            accelerometer2: self.acc.accelerometer(),
            magnetometer: self.mag.magnetometer(),
            pressure_baro: self.baro.pressure(),
            altitude_baro: self.baro.altitude(),
            temperature_baro: self.baro.temperature(),

            charge_voltage: self.power.charge_voltage(),
            battery_voltage: self.power.battery_voltage(),
            current: self.power.battery_current(),

            lora_rssi: Some(self.radio.trx.rssi), // TODO
            transmit_power: Some(self.radio.transmit_power),
            data_rate: Some(self.data_rate),

            cpu_utilization: Some(self.loop_runtime_history.iter().sum::<f32>() / self.loop_runtime_history.len() as f32),
            flash_pointer: Some(self.flash.pointer),

            gps: self.gps.datum(),

            acs_mode: Some(self.acs_mode),
            thruster_valve_state: Some(self.last_thruster_valve_state),

            ..Default::default()

        }
    }
}

#[embassy_executor::task]
pub async fn run(mut vehicle: Vehicle, mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(1_000_000 / MAIN_LOOP_FREQUENCY.0 as u64));
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

            loop_runtime_history: VecDeque::with_capacity(RUNTIME_HISTORY_LEN),
            settings,
            data_rate,

            acs_mode: AcsMode::Disabled,
            last_manual_thruster_input: None,
            last_thruster_valve_state: ThrusterValveState::Closed,
        }
    }

    async fn tick(&mut self) {
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
            self.handle_can_bus_message(&msg);
            match msg {
                FcReceivedCanBusMessage::BatteryTelemetry(_id, bat_msg) => {
                    self.power.handle_battery_can_msg(bat_msg)
                },
                FcReceivedCanBusMessage::IoBoardSensor(_role, _id, _sensor_msg) => {
                    // TODO: parse known sensors
                },
                FcReceivedCanBusMessage::IoBoardPower(_role, _power_msg) => {},
                FcReceivedCanBusMessage::FinBoardData(_fin, _id, _data_msg) => {},
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
                UplinkMessage::Heartbeat => {},
                // TODO: remove bootloader command?
                UplinkMessage::Command(cmd) => if let Command::RebootToBootloader = cmd {
                    //reboot_to_bootloader()
                } else {
                    self.handle_command(cmd).await
                },
                UplinkMessage::ReadFlash(adress, size) => { let _ = self.flash.read(adress, size); }
                UplinkMessage::ReadSettings => self.usb.send_message(DownlinkMessage::Settings(self.settings.clone())),
                UplinkMessage::WriteSettings(settings) => { let _ = self.flash.write_settings(settings); }
                UplinkMessage::ApplyLoRaSettings(_) => {}
            }
        }

        // ... and via LoRa
        if let Some(cmd) = self.radio.tick(self.time.0).await {
            self.handle_command(cmd).await;
        }

        // Set output according to flight mode
        let elapsed = self.state_estimator.time_in_mode();
        let drogue_high = self.mode == FlightMode::RecoveryDrogue && self.settings.drogue_output_settings.currently_high(elapsed);
        let main_high = self.mode == FlightMode::RecoveryMain && self.settings.main_output_settings.currently_high(elapsed);
        self.recovery.0.set_level(drogue_high.into());
        self.recovery.1.set_level(main_high.into());

        let (r,y,g) = self.mode.led_state(self.time.0);
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
        if self.mode >= FlightMode::Armed {
            if let Some(msg) = self.next_flash_telem() {
                let _ = self.flash.write_message(msg);
            }
        }

        // Broadcast telemetry to payloads
        self.broadcast_can_telemetry();

        // Increase time for next iteration
        self.time += 1_000 / MAIN_LOOP_FREQUENCY.0;

        // get CPU usage
        self.loop_runtime_history.truncate(RUNTIME_HISTORY_LEN - 1);
        let elapsed = start.elapsed().as_micros();
        self.loop_runtime_history.push_front((elapsed as f32) / (1_000_000.0 / MAIN_LOOP_FREQUENCY.0 as f32));
    }

    fn handle_can_bus_message(&mut self, msg: &FcReceivedCanBusMessage) {
        let msg = TelemetryCanBusMessage {
            time: self.time.0,
            msg: msg.clone(),
        };
        let msg = DownlinkMessage::TelemetryCanBusMessage(msg);

        let _ = self.usb.send_message(msg.clone());
        if self.mode >= FlightMode::Armed {
            let _ = self.flash.write_message(msg);
        }
    }

    fn transmit_output_commands(&mut self) {
        if !(self.time.0 % 10 == 0) {
            return;
        }

        let valve_state = match self.acs_mode {
            AcsMode::Disabled => ThrusterValveState::Closed,
            AcsMode::Auto => self.state_estimator.thruster_valve(),
            AcsMode::Manual => match self.last_manual_thruster_input {
                Some((state, time)) => {
                    if (self.time - time).0 < 450 { // TODO
                        state
                    } else {
                        self.last_manual_thruster_input = None;
                        ThrusterValveState::Closed
                    }
                },
                None => ThrusterValveState::Closed,
            }
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
            Command::RebootToBootloader => {},
            Command::SetFlightMode(fm) => self.switch_mode(fm),
            Command::SetTransmitPower(txp) => self.radio.set_transmit_power(txp),
            Command::SetDataRate(dr) => self.data_rate = dr,
            Command::SetAcsMode(am) => self.acs_mode = am,
            Command::SetAcsValveState(vs) => if self.acs_mode != AcsMode::Disabled {
                // We've interferred, set mode to manual.
                self.acs_mode = AcsMode::Manual;
                self.last_manual_thruster_input = Some((vs, self.time));
            },
            Command::EraseFlash => { let _ = self.flash.erase(); },
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

        self.mode = new_mode;
        self.buzzer.switch_mode(self.time.0, new_mode);
    }

    // TODO: replace these with a more elegant system
    #[cfg(not(feature = "gcs"))]
    fn next_usb_telem(&mut self) -> Option<DownlinkMessage> {
        if self.time.0 % 1000 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryGPS(vs.into()))
        } else if self.time.0 % 200 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryDiagnostics(vs.into()))
        } else if self.time.0 % 50 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryMain(vs.into()))
        } else if self.time.0 % 50 == 25 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryRawSensors(vs.into()))
        } else {
            None
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_lora_telem(&mut self) -> Option<DownlinkMessage> {
        // TODO
        if self.time.0 % 1000 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryGPS(vs.into()))
        } else if self.time.0 % 200 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryDiagnostics(vs.into()))
        } else if self.time.0 % 100 == 50 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryMainCompressed(vs.into()))
        } else if self.time.0 % 50 == 25 && self.data_rate == TelemetryDataRate::High {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryRawSensorsCompressed(vs.into()))
        } else {
            None
        }
    }

    #[cfg(not(feature = "gcs"))]
    fn next_flash_telem(&mut self) -> Option<DownlinkMessage> {
        // Offset everything a little so that flash message writes don't coincide
        // with lora message writes.
        let t = self.time.0 + 3;
        if t % 100 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryGPS(vs.into()))
        } else if t % 50 == 0 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryDiagnostics(vs.into()))
        } else if t % 50 == 20 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryMain(vs.into()))
        } else if t % 10 == 5 {
            let vs: VehicleState = self.into();
            Some(DownlinkMessage::TelemetryRawSensors(vs.into()))
        } else {
            None
        }
    }
}
