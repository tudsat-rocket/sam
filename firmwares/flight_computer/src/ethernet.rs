use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};

use embassy_stm32::eth::GenericPhy;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;

use embassy_net::StackResources;
use embassy_net::udp::{PacketMetadata, UdpSocket};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use shared_types::{DownlinkMessage, Transmit, UplinkMessage};
use static_cell::StaticCell;

static RX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static RX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static TX_META: StaticCell<[PacketMetadata; 8]> = StaticCell::new();
static TX_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

static DOWNLINK: StaticCell<Channel<CriticalSectionRawMutex, DownlinkMessage, 3>> = StaticCell::new();
static UPLINK: StaticCell<Channel<CriticalSectionRawMutex, UplinkMessage, 3>> = StaticCell::new();

pub fn start(
    device: Ethernet<'static, ETH, GenericPhy>,
    spawner: Spawner,
    seed: u64,
) -> (
    Sender<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    Receiver<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) {
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

    // TODO
    let config = embassy_net::Config::dhcpv4(Default::default());

    let (stack, runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(run_network(runner)).unwrap();

    let socket = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; 8]),
        RX_BUFFER.init([0; 1024]),
        TX_META.init([PacketMetadata::EMPTY; 8]),
        TX_BUFFER.init([0; 1024]),
    );

    let downlink = DOWNLINK.init(Channel::new());
    let uplink = UPLINK.init(Channel::new());

    spawner.spawn(run_socket(socket, downlink.receiver(), uplink.sender())).unwrap();

    (downlink.sender(), uplink.receiver())
}

#[embassy_executor::task]
async fn run_network(mut runner: embassy_net::Runner<'static, Ethernet<'static, ETH, GenericPhy>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn run_socket(
    mut socket: UdpSocket<'static>,
    downlink_receiver: Receiver<'static, CriticalSectionRawMutex, DownlinkMessage, 3>,
    uplink_sender: Sender<'static, CriticalSectionRawMutex, UplinkMessage, 3>,
) -> ! {
    info!("started run_socket");
    let remote_endpoint = (embassy_net::Ipv4Address::new(255, 255, 255, 255), 18355);

    socket.bind(18356).unwrap();
    socket.set_hop_limit(Some(4));
    info!("Ethernet socket bind successful");

    loop {
        let mut recv_buffer = [0; 1024];
        match select(downlink_receiver.receive(), socket.recv_from(&mut recv_buffer)).await {
            // send downlink message via udp
            Either::First(downlink_msg) => {
                let serialized = downlink_msg.serialize().unwrap();
                socket.send_to(&serialized, remote_endpoint).await.unwrap();
            }
            // receive udp packet from PC
            Either::Second(res) => {
                info!("Received some data over ethernet.");
                if let Ok((len, _peer)) = res {
                    let Ok(msg) = postcard::from_bytes_cobs(&mut recv_buffer[..len]) else {
                        info!("Received UplinkMessage");
                        continue;
                    };

                    uplink_sender.send(msg).await;
                }
            }
        }
    }
}
