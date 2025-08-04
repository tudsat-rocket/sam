use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::{filter, Can, CanRx, CanTx, Fifo};
use embassy_stm32::can::{Frame, Id, StandardId};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use shared_types::{CanBusMessageId, IoBoardRole};
use static_cell::StaticCell;
use crate::roles::BoardRole;

pub const CAN_QUEUE_SIZE: usize = 5; // TODO
pub const NUM_CAN_SUBSCRIBERS: usize = 3; // TODO
pub const NUM_CAN_PUBLISHERS: usize = 5; // TODO

pub type CanFrame = (u16, [u8; 8]);
pub type CanInChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanInSubscriper =
    Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>;
pub type CanOutChannel = PubSubChannel<CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;
pub type CanOutPublisher = Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>;

pub static CAN_IN: StaticCell<CanInChannel> = StaticCell::new();
pub static CAN_OUT: StaticCell<CanOutChannel> = StaticCell::new();

static CAN: StaticCell<Can<'static>> = StaticCell::new();
static CAN_TX: StaticCell<CanTx<'static>> = StaticCell::new();
static CAN_RX: StaticCell<CanRx<'static>> = StaticCell::new();

pub async fn spawn(
    mut can: Can<'static>,
    spawner: Spawner,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
    subscriber: Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) {
    can.modify_config().set_loopback(false).set_silent(false).set_automatic_retransmit(false);
    // .leave_disabled();
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
    can_tx: &'static mut CanTx<'static>,
    mut subscriber: Subscriber<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, 1, NUM_CAN_PUBLISHERS>,
) -> ! {
    loop {
        let (address, data) = subscriber.next_message_pure().await;
        let Some(sid) = StandardId::new(address) else {
            continue;
        };

        // TODO: (embassy upgrade) handle error
        if let Ok(frame) = Frame::new_data(sid, &data) {
            can_tx.write(&frame).await;
            can_tx.flush_all().await;
        }
    }
}

#[embassy_executor::task]
async fn run_rx(
    can_rx: &'static mut CanRx<'static>,
    publisher: Publisher<'static, CriticalSectionRawMutex, CanFrame, CAN_QUEUE_SIZE, NUM_CAN_SUBSCRIBERS, 1>,
) -> ! {
    loop {
        match can_rx.read().await {
            Ok(envelope) => {
                let frame = envelope.frame;
                // let Some(data) = frame.data() else {
                //     continue;
                // };
                // TODO: (embassy upgrade)
                let data = frame.data();

                let Id::Standard(sid) = frame.id() else {
                    // EID package, skipping.
                    continue;
                };
                let id_raw = sid.as_raw();

                let Ok(data_array) = data.as_ref().try_into() else {
                    // frame wasn't 8 bytes long, skip.
                    continue;
                };

                publisher.publish_immediate((id_raw, data_array));
            }
            Err(e) => {
                error!("Failed to read envelope: {:?}", Debug2Format(&e))
            }
        }
    }
}

pub fn configure(can: &mut Can, role: IoBoardRole) {
    let command_prefix = CanBusMessageId::IoBoardCommand(role, 0).into();
    let command_filter = filter::Mask32::frames_with_std_id(
        StandardId::new(command_prefix).unwrap(),
        StandardId::new(0x7f0).unwrap(),
    );

    let telemetry_filter = filter::Mask32::frames_with_std_id(
        StandardId::new(CanBusMessageId::TelemetryBroadcast(0).into()).unwrap(),
        StandardId::new(0x700).unwrap(),
    );

    can.modify_filters()
        .enable_bank(0, Fifo::Fifo0, command_filter)
        .enable_bank(1, Fifo::Fifo1, telemetry_filter);
}
