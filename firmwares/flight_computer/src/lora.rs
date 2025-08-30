use core::hash::Hasher;
use embassy_time::{Delay, Duration, Instant, Ticker, with_timeout};
use heapless::{String, Vec};

use embassy_executor::SendSpawner;
use embassy_futures::select::{Either, select};

use embassy_stm32::eth::GenericPhy;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;

use lora_phy::{LoRa, RxMode};

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
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

static DOWNLINK_LORA_SETTINGS: StaticCell<Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>> = StaticCell::new();
static RSSI_GLOB: StaticCell<Mutex<CriticalSectionRawMutex, i16>> = StaticCell::new();

#[derive(Clone, Copy)]
pub struct TypedLoraLinkSettings {
    tx_power: u16,
    spreading_factor: SpreadingFactor,
    bandwidth: Bandwidth,
    coding_rate: CodingRate,
    // message_len: usize, <- must be constant
    // hmac_len: usize, <- still needs to be constant
}
impl TypedLoraLinkSettings {
    pub fn default_downlink() -> Self {
        Self {
            tx_power: 22,
            spreading_factor: SpreadingFactor::_7,
            bandwidth: Bandwidth::_500KHz,
            coding_rate: CodingRate::_4_8,
            // message_len: 32,
            // hmac_len: 2,
        }
    }
    // NOTE: not in use yet
    pub fn default_uplink() -> Self {
        Self {
            tx_power: 10,
            spreading_factor: SpreadingFactor::_7,
            bandwidth: Bandwidth::_250KHz,
            coding_rate: CodingRate::_4_8,
            // hmac_len: 8,
        }
    }
}

// const DOWNLINK_TX_POWER: u16 = 10;
// const DOWNLINK_SF: SpreadingFactor = SpreadingFactor::_7;
// const DOWNLINK_BW: Bandwidth = Bandwidth::_500KHz;
// const DOWNLINK_CR: CodingRate = CodingRate::_4_8;

const DOWNLINK_MSG_LEN: usize = 32; // TODo
const DOWNLINK_HMAC_LEN: usize = 2; // TODo
const DOWNLINK_PACKET_INTERVAL_MILLIS: u64 = 50;

const UPLINK_TX_POWER: u16 = 22;
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
    // downlink_settings: &'static Mutex<CriticalSectionRawMutex, LoraLinkSettings>,
    spawner: SendSpawner,
) -> (
    Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    &'static Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,
) {
    let downlink = DOWNLINK.init(Channel::new());
    let downlink_settings = DOWNLINK_LORA_SETTINGS.init(Mutex::new(TypedLoraLinkSettings::default_downlink()));

    if let Some(sequence) = generate_downlink_sequence(settings.downlink_channels, &settings.binding_phrase) {
        spawner
            .spawn(run_rocket_downlink(lora, settings, sequence, downlink.receiver(), downlink_settings))
            .unwrap();
    } else {
        info!("Not transmitting anything.");
    }

    (downlink.sender(), downlink_settings)
}

pub fn start_rocket_uplink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    spawner: SendSpawner,
) -> (Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>, &'static Mutex<CriticalSectionRawMutex, i16>) {
    let uplink = UPLINK.init(Channel::new());
    let rssi_glob = RSSI_GLOB.init(Mutex::new(0));

    spawner.spawn(run_rocket_uplink(lora, settings, rssi_glob, uplink.sender())).unwrap();

    (uplink.receiver(), rssi_glob)
}

pub fn start_gcs_downlink(
    lora: LoRa<crate::LoraTransceiver, Delay>,
    settings: LoRaSettings,
    // downlink_settings: &'static Mutex<CriticalSectionRawMutex, LoraLinkSettings>,
    spawner: SendSpawner,
) -> (
    Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    &'static Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,
) {
    let downlink = DOWNLINK.init(Channel::new());
    let downlink_settings = DOWNLINK_LORA_SETTINGS.init(Mutex::new(TypedLoraLinkSettings::default_downlink()));

    if let Some(sequence) = generate_downlink_sequence(settings.downlink_channels, &settings.binding_phrase) {
        spawner
            .spawn(run_gcs_downlink(lora, settings, sequence, downlink.sender(), downlink_settings))
            .unwrap();
    }

    (downlink.receiver(), downlink_settings)
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
    downlink_settings: &'static Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();

    loop {
        let local_settings: TypedLoraLinkSettings = downlink_settings.lock(|s| *s);
        let mut tx_buffer = [0; DOWNLINK_MSG_LEN];

        let msg = downlink_receiver.receive().await;
        let serialized = serialize_and_hmac::<_, DOWNLINK_HMAC_LEN>(&key, &msg);
        tx_buffer[..serialized.len()].copy_from_slice(&serialized);

        // TODO: figure out how to handle time:None
        let t = msg.time_produced().unwrap_or(0);
        let freq = downlink_frequency(&sequence, t);

        // TODO: remove unwrap
        let mod_params = lora
            .create_modulation_params(
                local_settings.spreading_factor,
                local_settings.bandwidth,
                local_settings.coding_rate,
                freq,
            )
            .unwrap();
        let mut packet_params = lora.create_tx_packet_params(4, false, true, false, &mod_params).unwrap();
        lora.prepare_for_tx(&mod_params, &mut packet_params, local_settings.tx_power.into(), &tx_buffer)
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
    rssi_glob: &'static Mutex<CriticalSectionRawMutex, i16>,
    uplink_sender: Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();
    let frequency = CHANNELS[settings.uplink_channel];

    let mod_params = lora.create_modulation_params(UPLINK_SF, UPLINK_BW, UPLINK_CR, frequency).unwrap();
    let packet_params = lora.create_rx_packet_params(4, false, UPLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

    loop {
        let mut rx_buffer = [0; UPLINK_MSG_LEN];
        lora.prepare_for_rx(RxMode::Continuous, &mod_params, &packet_params).await.unwrap();
        let (len, _status) = lora.rx(&packet_params, &mut rx_buffer).await.unwrap();

        let rssi = lora.get_rssi().await.unwrap_or(0);
        unsafe {
            rssi_glob.lock_mut(|rssi_glob| *rssi_glob = rssi);
        }

        if let Some(msg) = decode_and_verify::<_, UPLINK_HMAC_LEN>(&key, &mut rx_buffer[..(len as usize)]) {
            info!("Got uplink msg: {}", Debug2Format(&msg));
            // Triple burst uplink 
            uplink_sender.send(msg).await;
            uplink_sender.send(mgs).await;
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
    downlink_settings: &'static Mutex<CriticalSectionRawMutex, TypedLoraLinkSettings>,
) -> ! {
    let key = settings.authentication_key.to_be_bytes();

    // try to find the signal by sweeping through frequencies slowly
    loop {
        let local_settings = downlink_settings.lock(|s| *s);
        // try to find the signal by sweeping through frequencies slowly
        let mut ticker = Ticker::every(Duration::from_millis(1111));
        let mut i = 0;
        let msg = loop {
            let f = CHANNELS[i];
            info!("Listening on {:?}kHz.", f / 1000);
            let timeout = Duration::from_millis(1000);
            if let Some(msg) = attempt_receive_downlink(&mut lora, &key, f, timeout, &local_settings).await {
                break msg;
            }

            i = (i + 1) % CHANNELS.len();
            ticker.next().await;
        };

        let mut t = msg.time_produced().unwrap_or(0);
        let mut ticker = Ticker::every(Duration::from_millis(DOWNLINK_PACKET_INTERVAL_MILLIS));

        info!("Received message with t={}, following sequence.", t);
        downlink_sender.send(msg).await;

        let mut failure_counter = 0;
        loop {
            t += DOWNLINK_PACKET_INTERVAL_MILLIS as u32;

            let f = downlink_frequency(&sequence, t);
            let timeout = Duration::from_millis(DOWNLINK_PACKET_INTERVAL_MILLIS - 10);
            if let Some(msg) = attempt_receive_downlink(&mut lora, &key, f, timeout, &local_settings).await {
                downlink_sender.send(msg).await;
                failure_counter = 0;
            } else {
                failure_counter += 1;
                if failure_counter > (10000 / DOWNLINK_PACKET_INTERVAL_MILLIS) {
                    break;
                }
            }

            ticker.next().await;
        }

        info!("Lost connection, reverting to channel sweep.");
    }
}

async fn attempt_receive_downlink(
    lora: &mut LoRa<crate::LoraTransceiver, Delay>,
    key: &[u8; 16],
    frequency: u32,
    timeout: Duration,
    downlink_settings: &TypedLoraLinkSettings,
) -> Option<DownlinkMessage> {
    let mod_params = lora
        .create_modulation_params(
            downlink_settings.spreading_factor,
            downlink_settings.bandwidth,
            downlink_settings.coding_rate,
            frequency,
        )
        .unwrap();
    let packet_params =
        lora.create_rx_packet_params(4, false, DOWNLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

    let mut rx_buffer = [0; DOWNLINK_MSG_LEN];
    lora.prepare_for_rx(RxMode::Continuous, &mod_params, &packet_params).await.unwrap();

    match with_timeout(timeout, lora.rx(&packet_params, &mut rx_buffer)).await {
        Ok(Ok((len, _status))) => decode_and_verify::<_, DOWNLINK_HMAC_LEN>(&key, &mut rx_buffer[..(len as usize)]),
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

    loop {
        let msg = uplink_receiver.receive().await;
        info!("Sending uplink message: {:?}", Debug2Format(&msg));

        let mut tx_buffer = [0; UPLINK_MSG_LEN];

        let serialized = serialize_and_hmac::<_, UPLINK_HMAC_LEN>(&key, &msg);
        tx_buffer[..serialized.len()].copy_from_slice(&serialized);

        let mod_params = lora.create_modulation_params(UPLINK_SF, UPLINK_BW, UPLINK_CR, frequency).unwrap();
        let mut packet_params =
            lora.create_rx_packet_params(4, false, UPLINK_MSG_LEN as u8, true, false, &mod_params).unwrap();

        lora.prepare_for_tx(&mod_params, &mut packet_params, UPLINK_TX_POWER.into(), &tx_buffer)
            .await
            .unwrap();
        lora.tx().await.unwrap();
        lora.sleep(false).await.unwrap();
    }
}

// Generates a list of active channel indices that represents the channel hopping sequence.
// Returns None, if no channels are active.
// The channel sequence is deterministically derived from the binding_phrase.
fn generate_downlink_sequence(
    channels: [bool; CHANNELS.len()],
    binding_phrase: &String<64>,
) -> Option<[usize; CHANNELS.len()]> {
    let mut available: Vec<usize, 16> =
        channels.iter().enumerate().filter(|&(_, active)| *active).map(|(i, _)| i).collect();
    if available.is_empty() {
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

// TODO: (Felix) check if documentation correct
/// Gives the frequency of the next downlink channel, calculated from the static channel hopping
/// sequence and the current time [ms] (or main loop invocations).
fn downlink_frequency(sequence: &[usize; CHANNELS.len()], t: u32) -> u32 {
    let i = (t / LORA_MESSAGE_INTERVAL) as usize % CHANNELS.len();
    CHANNELS[sequence[i]]
}

// TODO: (Felix) documentation
fn start_of_interval(t: u32) -> u32 {
    t.wrapping_sub(t % LORA_MESSAGE_INTERVAL)
}

/// Generates the ready to transmit downlink message bytes by prepending the message's hash.
fn serialize_and_hmac<M: Transmit, const H: usize>(key: &[u8; 16], msg: &M) -> Vec<u8, DOWNLINK_MSG_LEN> {
    let serialized = msg.serialize().unwrap_or_default();

    // Prepend message authentication
    let mut siphasher = SipHasher::new_with_key(key);
    //#[cfg(feature = "gcs")] // only include time for uplink messages, prevents replay attacks
    //siphasher.write(&self.start_of_current_interval().to_be_bytes());
    siphasher.write(&serialized);
    let hash = &siphasher.finish().to_be_bytes()[..H];

    // NOTE: think about unwrap
    let mut msg = Vec::from_slice(hash).unwrap();
    let _ = msg.extend_from_slice(serialized.as_slice());
    msg
}

fn decode_and_verify<M: Transmit + DeserializeOwned, const H: usize>(key: &[u8; 16], buffer: &mut [u8]) -> Option<M> {
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
