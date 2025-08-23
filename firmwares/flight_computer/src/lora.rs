use core::hash::Hasher;
use embassy_time::{Delay, Duration, Ticker, with_timeout};
use heapless::{String, Vec};

use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, select};

use embassy_stm32::eth::GenericPhy;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;

use lora_phy::{LoRa, RxMode};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use lora_phy::mod_params::{Bandwidth, CodingRate, SpreadingFactor};
use num_traits::ToBytes;
use shared_types::{DownlinkMessage, Transmit, UplinkMessage};
use static_cell::StaticCell;

use rand::prelude::*;
use rand_chacha::ChaCha20Rng;
use serde::de::DeserializeOwned;
use siphasher::sip::SipHasher;

use shared_types::{LORA_MESSAGE_INTERVAL, LoRaSettings};

use defmt::*;

static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, UplinkMessage, 3>> = StaticCell::new();

const DOWNLINK_TX_POWER: u16 = 10;
const DOWNLINK_SF: SpreadingFactor = SpreadingFactor::_7;
const DOWNLINK_BW: Bandwidth = Bandwidth::_500KHz;
const DOWNLINK_CR: CodingRate = CodingRate::_4_8;
const DOWNLINK_MSG_LEN: usize = 32; // TODo
const DOWNLINK_HMAC_LEN: usize = 2; // TODo

const UPLINK_SF: SpreadingFactor = SpreadingFactor::_7;
const UPLINK_BW: Bandwidth = Bandwidth::_250KHz;
const UPLINK_CR: CodingRate = CodingRate::_4_8;
const UPLINK_MSG_LEN: usize = 16; // TODo
const UPLINK_HMAC_LEN: usize = 8; // TODo

// TO DISCUSS:
//  - time in uplink message hmac

// The available channels for telemetry, assuming a 500kHz band width.
const CHANNELS: [u32; 14] = [
    863_250_000,
    863_750_000,
    864_250_000,
    864_750_000,
    865_250_000,
    865_750_000,
    866_250_000,
    866_750_000,
    867_250_000,
    867_750_000,
    868_250_000,
    868_750_000,
    869_250_000,
    869_750_000,
];

pub fn start_rocket_downlink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    spawner: SendSpawner,
) -> Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3> {
    let downlink = DOWNLINK.init(Channel::new());

    if let Some(sequence) = downlink_sequence(settings.downlink_channels, &settings.binding_phrase) {
        spawner.spawn(run_rocket_downlink(lora, settings, sequence, downlink.receiver())).unwrap();
    }

    downlink.sender()
}

pub fn start_rocket_uplink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    spawner: SendSpawner,
) -> Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3> {
    let uplink = UPLINK.init(Channel::new());

    spawner.spawn(run_rocket_uplink(lora, settings, uplink.sender())).unwrap();

    uplink.receiver()
}

pub fn start_gcs_downlink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    spawner: SendSpawner,
) -> Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3> {
    let downlink = DOWNLINK.init(Channel::new());

    if let Some(sequence) = downlink_sequence(settings.downlink_channels, &settings.binding_phrase) {
        spawner.spawn(run_gcs_downlink(lora, settings, sequence, downlink.sender())).unwrap();
    }

    downlink.receiver()
}

pub fn start_gcs_uplink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    spawner: SendSpawner,
) -> Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3> {
    let uplink = UPLINK.init(Channel::new());

    spawner.spawn(run_gcs_uplink(lora, settings, uplink.receiver())).unwrap();

    uplink.sender()
}

#[embassy_executor::task]
async fn run_rocket_downlink(
    mut lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    sequence: [usize; CHANNELS.len()],
    downlink_receiver: Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();

    loop {
        let mut tx_buffer = [0; DOWNLINK_MSG_LEN];

        let msg = downlink_receiver.receive().await;
        let serialized = serialize_and_hmac::<_, DOWNLINK_HMAC_LEN>(&key, &msg);
        tx_buffer[..serialized.len()].copy_from_slice(&serialized);

        // TODO: figure out how to handle time:None
        let t = msg.time_produced().unwrap_or(0);
        let freq = downlink_frequency(&sequence, t);

        let mod_params = lora.create_modulation_params(DOWNLINK_SF, DOWNLINK_BW, DOWNLINK_CR, freq).unwrap();
        let mut packet_params = lora.create_tx_packet_params(4, false, true, false, &mod_params).unwrap();
        lora.prepare_for_tx(&mod_params, &mut packet_params, DOWNLINK_TX_POWER.into(), &tx_buffer)
            .await
            .unwrap();
        lora.tx().await.unwrap();
        lora.sleep(false).await.unwrap();
    }
}

#[embassy_executor::task]
async fn run_rocket_uplink(
    mut lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    uplink_sender: Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();
    let frequency = CHANNELS[settings.uplink_channel];

    let mod_params = lora.create_modulation_params(UPLINK_SF, UPLINK_BW, UPLINK_CR, frequency).unwrap();
    let packet_params = lora.create_rx_packet_params(4, false, UPLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

    loop {
        let mut rx_buffer = [0; UPLINK_MSG_LEN];
        lora.prepare_for_rx(RxMode::Continuous, &mod_params, &packet_params).await.unwrap();
        let (len, status) = lora.rx(&packet_params, &mut rx_buffer).await.unwrap();

        if let Some(msg) = decode_and_verify::<_, UPLINK_HMAC_LEN>(&key, &mut rx_buffer[..(len as usize)]) {
            info!("Got uplink msg: {}", Debug2Format(&msg));
            uplink_sender.send(msg).await;
        }
    }
}

#[embassy_executor::task]
async fn run_gcs_downlink(
    mut lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    sequence: [usize; CHANNELS.len()],
    downlink_sender: Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();

    // try to find the signal by sweeping through frequencies slowly
    let mut ticker = Ticker::every(Duration::from_millis(1111));
    let mut i = 0;
    loop {
        let f = CHANNELS[i];
        println!("{:?}", f);
        let t = Duration::from_millis(1000);
        if let Some(msg) = attempt_receive_downlink(&mut lora, &key, f, t).await {
            println!("{}", Debug2Format(&msg));
            downlink_sender.send(msg).await;
        }

        i = (i + 1) % CHANNELS.len();
        ticker.next().await;
    }
}

async fn attempt_receive_downlink(
    lora: &mut LoRa<crate::LoraTransceiver, Delay>,
    key: &[u8; 16],
    frequency: u32,
    timeout: Duration,
) -> Option<DownlinkMessage> {
    let mod_params = lora.create_modulation_params(DOWNLINK_SF, DOWNLINK_BW, DOWNLINK_CR, frequency).unwrap();
    let packet_params =
        lora.create_rx_packet_params(4, false, DOWNLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

    let mut rx_buffer = [0; DOWNLINK_MSG_LEN];
    lora.prepare_for_rx(RxMode::Continuous, &mod_params, &packet_params).await.unwrap();

    match with_timeout(timeout, lora.rx(&packet_params, &mut rx_buffer)).await {
        Ok(Ok((len, status))) => decode_and_verify::<_, DOWNLINK_HMAC_LEN>(&key, &mut rx_buffer[..(len as usize)]),
        _ => None,
    }
}

#[embassy_executor::task]
async fn run_gcs_uplink(
    mut lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    uplink_receiver: Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();
    let frequency = CHANNELS[settings.uplink_channel];

    let mod_params = lora.create_modulation_params(UPLINK_SF, UPLINK_BW, UPLINK_CR, frequency).unwrap();
    let packet_params = lora.create_rx_packet_params(4, false, UPLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

    loop {
        let msg = uplink_receiver.receive().await;
        //TODO
    }
}

fn downlink_sequence(channels: [bool; CHANNELS.len()], binding_phrase: &String<64>) -> Option<[usize; CHANNELS.len()]> {
    let mut available: Vec<_, 16> = channels.iter().enumerate().filter_map(|(i, x)| x.then(|| i)).collect();
    if available.len() == 0 {
        return None;
    }

    // Generate a u64 from binding phrase to seed rng
    let mut siphasher = SipHasher::new_with_key(&[0x00; 16]);
    siphasher.write(binding_phrase.as_bytes());
    let seed = siphasher.finish();
    let mut rng = ChaCha20Rng::seed_from_u64(seed);

    // Fill sequence from available channels. If we disable some channels,
    // we repeat the process until we have enough.
    let mut sequence: Vec<usize, 64> = Vec::new();
    while sequence.len() < CHANNELS.len() {
        available.shuffle(&mut rng);
        sequence.extend(available.clone());
    }

    Some(sequence[..CHANNELS.len()].try_into().unwrap())
}

fn downlink_frequency(sequence: &[usize; CHANNELS.len()], t: u32) -> u32 {
    let i = (t / LORA_MESSAGE_INTERVAL) as usize % CHANNELS.len();
    CHANNELS[sequence[i]]
}

fn start_of_interval(t: u32) -> u32 {
    t.wrapping_sub(t % LORA_MESSAGE_INTERVAL)
}

fn serialize_and_hmac<M: Transmit, const H: usize>(key: &[u8; 16], msg: &M) -> Vec<u8, DOWNLINK_MSG_LEN> {
    let serialized = msg.serialize().unwrap_or_default();

    // Prepend message authentication
    let mut siphasher = SipHasher::new_with_key(key);
    //#[cfg(feature = "gcs")] // only include time for uplink messages, prevents replay attacks
    //siphasher.write(&self.start_of_current_interval().to_be_bytes());
    siphasher.write(&serialized);
    let hash = &siphasher.finish().to_be_bytes()[..H];

    let mut msg = Vec::from_slice(hash).unwrap();
    let _ = msg.extend_from_slice(serialized.as_slice());
    msg
}

fn decode_and_verify<M: Transmit + DeserializeOwned, const H: usize>(key: &[u8; 16], buffer: &mut [u8]) -> Option<M> {
    println!("{:#}", buffer);
    let (hmac, serialized) = buffer.split_at_mut(H);
    let serialized_end = serialized.iter().position(|b| *b == 0).map(|i| i + 1).unwrap_or(serialized.len());

    let mut siphasher = SipHasher::new_with_key(key);
    //#[cfg(not(feature = "gcs"))] // only include time for uplink messages, prevents replay attacks
    //siphasher.write(&self.start_of_current_interval().to_be_bytes());
    //TODO
    siphasher.write(&serialized[..serialized_end]);
    let correct = siphasher.finish().to_be_bytes();
    if &correct[..H] != hmac {
        warn!("HMAC mismatch.");
        return None;
    }

    let deserialized = postcard::from_bytes_cobs(serialized).ok();
    if deserialized.is_none() {
        error!("Failed to decode message: {}", &buffer[1..]);
    }

    deserialized
}
