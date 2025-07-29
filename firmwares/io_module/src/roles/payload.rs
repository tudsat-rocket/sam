use defmt::*;
use embassy_executor::{SendSpawner, Spawner};
use embassy_futures::join::join;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{UartRx, UartTx};
use embassy_time::{Duration, Ticker};

use shared_types::{CanBusMessageId, IoBoardRole};

use crate::*;

const UART_TO_CAN_TELEMETRY_FREQUENCY_HZ: u64 = 1;
const ID_LED_PATTERN: [u8; 8] = [0, 0, 0, 0, 1, 0, 0, 0];

pub struct Payload {}

impl crate::roles::BoardRole for Payload {
    const ROLE_ID: IoBoardRole = IoBoardRole::Payload;

    fn input1_mode() -> InputMode {
        InputMode::Uart(embassy_stm32::usart::Config::default())
    }

    fn spawn(
        io: BoardIo,
        high_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
        can_in: &'static CanInChannel,
        can_out: &'static CanOutChannel,
        output_state: &'static OutputStateChannel,
    ) {
        if let Input1::Uart(uart) = io.input1 {
            let (uart_tx, uart_rx) = uart.split();

            high_priority_spawner.spawn(run_can_to_uart(can_in.subscriber().unwrap(), uart_tx)).unwrap();
            low_priority_spawner.spawn(run_uart_to_can(can_out.publisher().unwrap(), uart_rx)).unwrap();
        };

        // Set the LEDs based on the output state.
        let led_output_state_sub = output_state.subscriber().unwrap();
        low_priority_spawner.spawn(run_leds(io.leds, led_output_state_sub, ID_LED_PATTERN)).unwrap();
    }
}

#[embassy_executor::task]
pub async fn run_can_to_uart(
    mut subscriber: crate::CanInSubscriper,
    // mut uart_tx: UartTx<'static, USART3, DMA1_CH2>,
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

#[embassy_executor::task]
// pub async fn run_uart_to_can(publisher: crate::CanOutPublisher, mut uart_rx: UartRx<'static, USART3, DMA1_CH3>) -> ! {
pub async fn run_uart_to_can(publisher: crate::CanOutPublisher, mut uart_rx: UartRx<'static, Async>) -> ! {
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
