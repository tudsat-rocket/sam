use defmt::*;
use embassy_executor::SendSpawner;
use embassy_stm32::can::{Can, CanRx, CanTx, Frame};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};

use embedded_can::{Id, StandardId};
use heapless::Vec;
use static_cell::StaticCell;

use shared_types::can::{
    CanMessage,
    structure::{Can2aFrame, CanFrameId},
};

pub const CAN_RX_QUEUE_SIZE: usize = 20;
pub const CAN_TX_QUEUE_SIZE: usize = 20;
pub const NUM_CAN_SUBSCRIBERS: usize = 1;
pub const NUM_CAN_PUBLISHERS: usize = 1;

pub type CanRxChannel = PubSubChannel<CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanRxSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanTxChannel = PubSubChannel<CriticalSectionRawMutex, CanMessage, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;
pub type CanTxPublisher =
    Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_TX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;

// --- can1
pub static CAN1_RX_CH: StaticCell<CanRxChannel> = StaticCell::new();
pub static CAN1_TX_CH: StaticCell<CanTxChannel> = StaticCell::new();

static CAN1_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN1_RX: StaticCell<CanRx<'static>> = StaticCell::new();

// --- can2
pub static CAN2_RX_CH: StaticCell<CanRxChannel> = StaticCell::new();
pub static CAN2_TX_CH: StaticCell<CanTxChannel> = StaticCell::new();

static CAN2_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN2_RX: StaticCell<CanRx<'static>> = StaticCell::new();

// --- dedicated tasks for receiving and sending CAN messages for each hardware Bus
async fn run_can_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                let frame = envelope.frame;
                let Id::Standard(id) = frame.id() else {
                    warn!("Unexpected extended 29bit Can Frame Id, dropping.");
                    continue;
                };
                let Ok(data) = Vec::from_slice(frame.data()) else {
                    warn!("Unexpected Can Frame data of lenght > 8 bytes, dropping.");
                    continue;
                };
                let Ok(id) = CanFrameId::try_from(id.as_raw()) else {
                    warn!(
                        "Can Frame could not parsed into type, either the senders or this software might be outdated."
                    );
                    continue;
                };
                let message = Can2aFrame { id, payload: data };
                let Ok(message_parsed) = CanMessage::try_from(message) else {
                    warn!("Malformed Can Message, message could not be parsed into type, dropping"); //id: {}, payload: {}", message.id, message.payload);
                    continue;
                };
                publisher.publish_immediate(message_parsed); // TODO: think about publish immediate
            }
            Err(e) => {
                error!("Can Bus Error: Failed to read can envelope: {:?}", Debug2Format(&e))
            }
        }
    }
}

async fn run_can_tx(can_tx: &'static mut CanTx<'static>, mut subscriber: CanRxSubscriber) -> ! {
    loop {
        let message = subscriber.next_message_pure().await; // should we care about lag?
        let frame = Can2aFrame::from(message);

        let Some(sid) = StandardId::new(frame.id.into()) else {
            error!("Can2.0A Frame could not be converted to StandardId, this is a bug in the implementation.");
            continue;
        };

        // unwrap is safe, because payload slice is never larger than 8 bytes
        let frame = Frame::new_data(sid, frame.payload.as_slice()).unwrap();
        can_tx.write(&frame).await;
    }
}

// --- CAN1
pub async fn spawn_can1(
    can: Can<'static>,
    spawner: SendSpawner,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
    subscriber: Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN1_TX.init(can_tx);
    let can_rx = CAN1_RX.init(can_rx);

    spawner.spawn(run_can1_tx(can_tx, subscriber)).unwrap();
    spawner.spawn(run_can1_rx(can_rx, publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can1_tx(can_tx: &'static mut CanTx<'static>, mut subscriber: CanRxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can1_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}

// --- CAN2
pub async fn spawn_can2(
    can: Can<'static>,
    spawner: SendSpawner,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
    subscriber: Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_RX_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) {
    let (can_tx, can_rx, _properties) = can.split();
    let can_tx = CAN2_TX.init(can_tx);
    let can_rx = CAN2_RX.init(can_rx);

    spawner.spawn(run_can2_tx(can_tx, subscriber)).unwrap();
    spawner.spawn(run_can2_rx(can_rx, publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_can2_tx(can_tx: &'static mut CanTx<'static>, mut subscriber: CanRxSubscriber) -> ! {
    run_can_tx(can_tx, subscriber).await
}

#[embassy_executor::task]
async fn run_can2_rx(can_rx: &'static mut CanRx<'static>, publisher: CanTxPublisher) -> ! {
    run_can_rx(can_rx, publisher).await
}
