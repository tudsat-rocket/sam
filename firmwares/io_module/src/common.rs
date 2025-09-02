use defmt::{error, Debug2Format};
use embassy_futures::join::join;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_stm32::Peri;
use embassy_time::Ticker;
use embassy_time::{with_timeout, Duration};
use embedded_hal::digital::OutputPin;

use shared_types::*;
use {defmt_rtt as _, panic_probe as _};

use crate::{CanInSubscriper, DriveVoltage, OutputStatePublisher, OutputStateSubscriber};

async fn receive_output_message(
    subscriber: &mut crate::CanInSubscriper,
    role: IoBoardRole,
    command_id: u8,
) -> IoBoardOutputMessage {
    let expected_sid = CanBusMessageId::IoBoardCommand(role, command_id).into();

    loop {
        match subscriber.next_message_pure().await {
            (sid, data) if sid == expected_sid => {
                if let Ok(Some(msg)) = IoBoardOutputMessage::parse(data) {
                    return msg;
                }
            }
            _ => {}
        }
    }
}

async fn receive_telemetry_message(subscriber: &mut crate::CanInSubscriper) -> TelemetryToPayloadMessage {
    let expected_sid = CanBusMessageId::TelemetryBroadcast(0).into();

    loop {
        match subscriber.next_message_pure().await {
            (sid, data) if sid == expected_sid => {
                if let Ok(Some(msg)) = TelemetryToPayloadMessage::parse(data) {
                    return msg;
                }
            }
            _ => {}
        }
    }
}

#[embassy_executor::task]
pub async fn run_outputs(
    // mut output0: (Output<'static, PC7>, Output<'static, PC6>),
    // mut output1: (Output<'static, PC9>, Output<'static, PC8>),
    // mut output2: (Output<'static, PB8>, Output<'static, PB9>),
    // mut output3: (Output<'static, PA0>, Output<'static, PA1>),
    mut output0: (Output<'static>, Output<'static>),
    mut output1: (Output<'static>, Output<'static>),
    mut output2: (Output<'static>, Output<'static>),
    mut output3: (Output<'static>, Output<'static>),
    mut subscriber: OutputStateSubscriber,
) -> ! {
    loop {
        let (outputs, _failsafe) = subscriber.next_message_pure().await;
        output0.0.set_level(outputs[0].into());
        output0.1.set_level(outputs[1].into());
        output1.0.set_level(outputs[2].into());
        output1.1.set_level(outputs[3].into());
        output2.0.set_level(outputs[4].into());
        output2.1.set_level(outputs[5].into());
        output3.0.set_level(outputs[6].into());
        output3.1.set_level(outputs[7].into());
    }
}

#[embassy_executor::task]
pub async fn run_output_control_via_can(
    mut can_subscriber: CanInSubscriper,
    output_publisher: OutputStatePublisher,
    role: IoBoardRole,
    timeout: Option<Duration>,
) -> ! {
    loop {
        let (outputs, failsafe) = if let Some(timeout) = timeout {
            match with_timeout(timeout, receive_output_message(&mut can_subscriber, role, 0)).await {
                Ok(msg) => (msg.outputs, false),
                Err(_) => ([false; 8], true),
            }
        } else {
            (receive_output_message(&mut can_subscriber, role, 0).await.outputs, false)
        };

        output_publisher.publish_immediate((outputs, failsafe));
    }
}

async fn read_i2c_amp_value<I2C: embedded_hal_async::i2c::I2c>(
    i2c: &mut I2C,
    i2c_address: u8,
) -> Result<(u16, bool), I2C::Error> {
    // https://www.ti.com/lit/ds/symlink/adc101c027.pdf
    let mut buffer: [u8; 2] = [0x00, 0x00];
    i2c.read(i2c_address, &mut buffer).await?;

    let conversion_register = u16::from_be_bytes(buffer);
    let alert_flag = (conversion_register >> 15) > 0;
    let conversion_result = (conversion_register >> 2) & 0x3ff;

    Ok((conversion_result, alert_flag))
}

#[embassy_executor::task]
pub async fn run_i2c_sensors(
    mut input1_i2c: Option<I2c<'static, Async, Master>>,
    mut input3_i2c: Option<I2c<'static, Async, Master>>,
    mut input3_gpio: Option<(Input<'static>, Input<'static>)>,
    publisher: crate::CanOutPublisher,
    role: IoBoardRole,
    interval: Duration,
) -> ! {
    // ADC101C027 addresses in order: ADR floating, GND, VCC
    const AMPLIFIER_ADDRESSES: [u8; 3] = [0b1010000, 0b1010001, 0b1010010];

    let mut ticker = Ticker::every(interval);
    loop {
        // If we use both busses for sensors, we assume that each I2C bus has
        // two sensors, allowing us to fill all four slots of a single sensor
        // message. If we use just one, we can do up to 3 sensors per bus.
        let i2c_sensors = match (input1_i2c.as_mut(), input3_i2c.as_mut(), input3_gpio.as_mut()) {
            (Some(i2c2), Some(i2c1), _) => [
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[1]).await.ok(),
            ],
            (Some(i2c2), None, pins) => [
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c2, AMPLIFIER_ADDRESSES[2]).await.ok(),
                pins.map(|(p0, p1)| ((p0.is_high() as u16) << 1 | (p1.is_high() as u16), false)),
            ],
            (None, Some(i2c1), _) => [
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[0]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[1]).await.ok(),
                read_i2c_amp_value(i2c1, AMPLIFIER_ADDRESSES[2]).await.ok(),
                None,
            ],
            (None, None, _) => unreachable!(),
        };

        let msg = IoBoardSensorMessage { i2c_sensors };
        let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardInput(role, 0));
        publisher.publish_immediate((id, msg));

        ticker.next().await;
    }
}

fn to_millivolts(vref_sample: u16, sample: u16) -> u16 {
    const VREFINT_MV: u32 = 1200; // mV
    (u32::from(sample) * VREFINT_MV / u32::from(vref_sample)) as u16
}

fn to_bar(sample: u16) -> f32 {
    return 0.0;
}

#[embassy_executor::task]
pub async fn run_power_report(
    //TODO maybe adjust default pins again
    publisher: crate::CanOutPublisher,
    mut adc: Adc<'static, ADC1>,
    mut battery_voltage_sense_pin: Peri<'static, PA7>,
    mut charge_bus_voltage_sense_pin: Peri<'static, PA6>,
    mut current_sense_pin: Peri<'static, PC5>,     //PB1
    mut temperature_sense_pin: Peri<'static, PC4>, //PA4
    role: IoBoardRole,
    drive_voltage: DriveVoltage,
) -> ! {
    let mut vref = adc.enable_vref();
    const TIMEOUT: Duration = Duration::from_millis(10);

    let mut ticker = Ticker::every(Duration::from_millis(200)); // TODO: adjust?
    let vref_sample = with_timeout(TIMEOUT, adc.read(&mut vref)).await.unwrap_or(1490);

    loop {
        // TODO: (embassy upgrade)
        let thermistor_sample = with_timeout(TIMEOUT, adc.read(&mut temperature_sense_pin)).await;
        let thermistor_vsense = (to_millivolts(vref_sample, thermistor_sample.unwrap_or_default()) as f32) / 1000.0;
        let thermistor_resistance = (10e3 * (thermistor_vsense / (3.3 - thermistor_vsense))) as u16;

        let output_voltage = match drive_voltage {
            // no separate sense line for 5V, so just report the battery voltage in that case
            DriveVoltage::Battery | DriveVoltage::Regulator5V => {
                let sample = with_timeout(TIMEOUT, adc.read(&mut battery_voltage_sense_pin)).await;
                ((to_millivolts(vref_sample, sample.unwrap_or_default()) as f32) * (4.7 + 2.2) / 2.2) as u16
            }
            DriveVoltage::ChargeBus => {
                let sample = with_timeout(TIMEOUT, adc.read(&mut charge_bus_voltage_sense_pin)).await;
                ((to_millivolts(vref_sample, sample.unwrap_or_default()) as f32) * (15.0 + 2.2) / 2.2) as u16
            }
        };

        let current_sense_sample = with_timeout(TIMEOUT, adc.read(&mut current_sense_pin)).await;
        let current_sense_voltage = to_millivolts(vref_sample, current_sense_sample.unwrap_or_default());
        let output_current = (((current_sense_voltage as f32) - 1650.0) / (50.0 * 0.005)) as i16 * -1;

        let msg = IoBoardPowerMessage {
            output_voltage,
            output_current,
            thermistor_resistance,
        };

        let (id, msg) = msg.to_frame(CanBusMessageId::IoBoardInput(role, 0xf));
        publisher.publish_immediate((id, msg));

        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn run_outputs_on_after_flightmode(
    mut can_subscriber: CanInSubscriper,
    output_publisher: OutputStatePublisher,
    threshold_fm: FlightMode,
) -> ! {
    let mut last_fm = FlightMode::Idle;

    loop {
        let fm = receive_telemetry_message(&mut can_subscriber).await.mode;
        if fm != last_fm && fm >= threshold_fm {
            output_publisher.publish_immediate(([true; 8], false));
        }

        last_fm = fm;
    }
}

#[embassy_executor::task]
pub async fn run_leds(
    // leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
    leds: (Output<'static>, Output<'static>, Output<'static>),
    mut output_state_subscriber: OutputStateSubscriber,
    id_pattern: [u8; 8],
) -> ! {
    let (mut led_red, mut led_white, mut led_yellow) = leds;

    let mut any_output_on = false;
    let mut failsafe = false;

    let mut ticker = Ticker::every(Duration::from_hz(id_pattern.len() as u64));
    let mut i = 0;
    loop {
        while let Some((outputs, fs)) = output_state_subscriber.try_next_message_pure() {
            any_output_on = outputs.iter().any(|b| *b);
            failsafe = fs;
        }

        let red = failsafe || (any_output_on && (i < (id_pattern.len() / 2)));
        let white = id_pattern[i] > 0;

        let _ = led_red.set_state((!red).into());
        let _ = led_white.set_state((!white).into());
        let _ = led_yellow.set_state((!true).into());

        i = (i + 1) % id_pattern.len();
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn run_uart_to_can(publisher: crate::CanOutPublisher, mut uart_rx: UartRx<'static, Async>) -> ! {
    const UART_TO_CAN_TELEMETRY_FREQUENCY_HZ: u64 = 1;
    let message_id: u16 = CanBusMessageId::IoBoardInput(IoBoardRole::Payload, 0).into();

    // TODO: convince swiss to stick to 8-byte messages?
    let mut ticker_interval = Ticker::every(Duration::from_hz(UART_TO_CAN_TELEMETRY_FREQUENCY_HZ));
    let mut buffer: [u8; 8] = [0x00; 8];
    loop {
        // This effectively limits us to the ticker frequency.
        match join(uart_rx.read(&mut buffer), ticker_interval.next()).await {
            (Ok(()), _) => {
                publisher.publish_immediate((message_id, buffer));
            }
            (Err(e), _) => {
                error!("Failed to read payload telemetry message from UART: {:?}", Debug2Format(&e));
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run_can_to_uart(
    mut subscriber: crate::CanInSubscriper,
    mut uart_tx: UartTx<'static, embassy_stm32::mode::Async>,
) -> ! {
    loop {
        // These messages are relayed byte-for-byte, including CRC, so we never
        // have to parse them.
        let (sid, data) = subscriber.next_message_pure().await;
        if sid == CanBusMessageId::TelemetryBroadcast(0).into() {
            if let Err(e) = uart_tx.write(&data).await {
                error!("Failed to write message to UART: {:?}", Debug2Format(&e));
            }
        }
    }
}
