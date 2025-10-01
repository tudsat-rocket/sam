use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::bxcan::{Data, Fifo, Frame, Id, StandardId, filter};
use embassy_stm32::can::{Can, CanRx, CanTx};
use embassy_stm32::peripherals::CAN;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use shared_types::FlightMode;
//use shared_types::{CanBusMessage, CanBusMessageId, FlightMode, TelemetryToPayloadMessage};
use heapless::Vec;
use static_cell::StaticCell;

use shared_types::can::{
    CanMessage,
    structure::{Can2aFrame, CanFrameId},
};

pub const CAN_QUEUE_SIZE: usize = 3; // TODO
pub const NUM_CAN_SUBSCRIBERS: usize = 1; // TODO
pub const NUM_CAN_PUBLISHERS: usize = 1; // TODO

pub type CanRxCh = PubSubChannel<CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanRxSub = Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanTxCh = PubSubChannel<CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;
pub type CanTxPub = Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;

pub static CAN_IN: StaticCell<CanRxCh> = StaticCell::new();
pub static CAN_OUT: StaticCell<CanTxCh> = StaticCell::new();

static CAN: StaticCell<Can<'static, CAN>> = StaticCell::new();
static CAN_TX: StaticCell<CanTx<'static, 'static, CAN>> = StaticCell::new();
static CAN_RX: StaticCell<CanRx<'static, 'static, CAN>> = StaticCell::new();

pub async fn spawn(
    mut can: Can<'static, CAN>,
    spawner: Spawner,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
    subscriber: Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) {
    can.modify_config()
        .set_loopback(false)
        .set_silent(false)
        .set_automatic_retransmit(false)
        .leave_disabled();

    let telemetry_filter = filter::Mask32::frames_with_std_id(
        //StandardId::new(CanBusMessageId::TelemetryBroadcast(0).into()).unwrap(),
        StandardId::new(0x1af).unwrap(),
        StandardId::new(0).unwrap(),
    );

    can.modify_filters().enable_bank(0, Fifo::Fifo0, telemetry_filter);

    can.set_bitrate(125_000);
    can.enable().await;

    let can = CAN.init(can);
    let (can_tx, can_rx) = can.split();
    let can_tx = CAN_TX.init(can_tx);
    let can_rx = CAN_RX.init(can_rx);

    spawner.spawn(run_tx(can_tx, subscriber)).unwrap();
    spawner.spawn(run_rx(can_rx, publisher)).unwrap();
}

#[embassy_executor::task]
async fn run_tx(
    can_tx: &'static mut CanTx<'static, 'static, CAN>,
    mut subscriber: Subscriber<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) -> ! {
    loop {
        let msg: Can2aFrame = subscriber.next_message_pure().await.into();
        let Some(sid) = StandardId::new(msg.id.into()) else {
            continue;
        };

        let frame = Frame::new_data(sid, Data::new(msg.payload.as_slice()).unwrap());
        can_tx.write(&frame).await;
        can_tx.flush_all().await;
    }
}

#[embassy_executor::task]
async fn run_rx(
    can_rx: &'static mut CanRx<'static, 'static, CAN>,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanMessage, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
) -> ! {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                let frame = envelope.frame;
                let Id::Standard(id) = frame.id() else {
                    warn!("Unexpected extended 29bit Can Frame Id, dropping.");
                    continue;
                };
                let Some(frame_data) = frame.data() else {
                    warn!("Unexpected non-data Can Frame, dropping.");
                    continue;
                };

                let Ok(data) = Vec::from_slice(frame_data) else {
                    warn!("Unexpected Can Frame data of lenght > 8 bytes, dropping.");
                    continue;
                };
                let Ok(id) = CanFrameId::try_from(id.as_raw()) else {
                    warn!(
                        "Can Frame id could not be mapped to a type, either the sender's or this software might be outdated. id: {}",
                        id.as_raw()
                    );
                    continue;
                };
                let message = Can2aFrame { id, payload: data };
                let Ok(message_parsed) = CanMessage::try_from(message) else {
                    warn!("Malformed Can Message payload, message could not be parsed into type, dropping.");
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
