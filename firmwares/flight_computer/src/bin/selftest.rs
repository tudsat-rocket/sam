#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_mut)]

use core::net::Ipv4Addr;
use core::net::SocketAddr;
use core::net::SocketAddrV4;

use defmt::*;
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{with_timeout, Delay, Duration, Instant, Ticker, Timer};

use embassy_stm32::adc::Adc;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::Can;
use embassy_stm32::can::CanConfigurator;
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::exti::Channel as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, OutputType, Pin, Pull, Speed};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::*;
use embassy_stm32::rcc::*;
use embassy_stm32::rng::Rng;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{interrupt, Config};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_io_async::Write;
use embedded_nal_async::TcpConnect;

use embedded_can::Id;
use embedded_can::StandardId;

use embassy_net::tcp::client::TcpClient;
use embassy_net::tcp::client::TcpClientState;
use embassy_net::Ipv4Cidr;
use embassy_net::StackResources;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::Bandwidth;
use lora_phy::mod_params::CodingRate;
use lora_phy::mod_params::PacketStatus;
use lora_phy::mod_params::SpreadingFactor;
use lora_phy::sx126x::{self, Sx1262, Sx126x};
use lora_phy::{LoRa, RxMode};

use num_traits::float::Float;

const LORA_TEST_TX_POWER: i16 = 10;

async fn run_lora_loopback_test(
    tx: &mut LoRa<flight_computer_firmware::LoraTransceiver, Delay>,
    rx: &mut LoRa<flight_computer_firmware::LoraTransceiver, Delay>,
) -> Option<PacketStatus> {
    const TEST_PACKET: &[u8] = &[0x00, 0x42, 0x12, 0x34, 0x45, 0x56];
    const LORA_FREQUENCY_IN_HZ: u32 = 868_000_000;

    let mut receiving_buffer = [00u8; 100];

    // Start RX
    let mdltn_params = rx
        .create_modulation_params(SpreadingFactor::_7, Bandwidth::_500KHz, CodingRate::_4_8, LORA_FREQUENCY_IN_HZ)
        .unwrap();
    let rx_pkt_params = rx
        .create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params)
        .unwrap();
    rx.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await.unwrap();
    rx.start_rx().await.unwrap();

    // TX
    let mdltn_params = tx
        .create_modulation_params(SpreadingFactor::_7, Bandwidth::_500KHz, CodingRate::_4_8, LORA_FREQUENCY_IN_HZ)
        .unwrap();
    let mut tx_pkt_params = tx.create_tx_packet_params(4, false, true, false, &mdltn_params).unwrap();
    tx.prepare_for_tx(&mdltn_params, &mut tx_pkt_params, LORA_TEST_TX_POWER.into(), &TEST_PACKET)
        .await
        .unwrap();
    tx.tx().await.unwrap();
    tx.sleep(false).await.unwrap();
    Timer::after(Duration::from_millis(1000)).await;
    debug!("Lora self-test timeout");

    // Finish RX
    let (len, status) = rx.complete_rx(&rx_pkt_params, &mut receiving_buffer).await.unwrap();

    (&receiving_buffer[..(len as usize)] == TEST_PACKET).then_some(status)
}

async fn run_can_loopback_test(tx: &mut Can<'_>, rx: &mut Can<'_>) -> bool {
    const TEST_FRAME_ID: u16 = 0x123;
    const TEST_FRAME_CONTENT: &[u8] = &[0x42; 8];

    let tx_future = async move {
        let frame = embassy_stm32::can::frame::Frame::new_standard(TEST_FRAME_ID, TEST_FRAME_CONTENT).unwrap();
        let _ = tx.write(&frame).await;
    };

    let rx_future = async move {
        let envelope = rx.read().await.unwrap();
        envelope.frame
    };

    let Ok((_, frame)) = with_timeout(Duration::from_secs(1), join(tx_future, rx_future)).await else {
        return false;
    };

    let Id::Standard(sid) = frame.id() else {
        return false;
    };

    sid.as_raw() == TEST_FRAME_ID && frame.data() == TEST_FRAME_CONTENT
}

macro_rules! sensor_test {
    ($name:expr, $sensor:expr, $method:ident, $hertz:expr, $nominal_mag:expr, $nominal_std_dev:expr) => {{
        let mut results: heapless::Vec<nalgebra::Vector3<f32>, $hertz> = heapless::Vec::new();
        let mut ticker = Ticker::every(Duration::from_millis(1000 / $hertz));
        for _i in 0..$hertz {
            $sensor.tick().await;
            let _ = results.push($sensor.$method().unwrap());
            ticker.next().await;
        }

        let len = results.len() as f32;

        let mean_x: f32 = results.iter().map(|v| v.x).sum::<f32>() / len;
        let mean_y: f32 = results.iter().map(|v| v.y).sum::<f32>() / len;
        let mean_z: f32 = results.iter().map(|v| v.z).sum::<f32>() / len;

        let var_x = results.iter().map(|v| (mean_x - v.x).powi(2)).sum::<f32>() / len;
        let var_y = results.iter().map(|v| (mean_y - v.y).powi(2)).sum::<f32>() / len;
        let var_z = results.iter().map(|v| (mean_z - v.z).powi(2)).sum::<f32>() / len;

        let mag = (mean_x.powi(2) + mean_y.powi(2) + mean_z.powi(2)).sqrt();
        let std_dev = ((var_x + var_y + var_z) / 3.0).sqrt();

        if $nominal_mag.contains(&mag) && $nominal_std_dev.contains(&std_dev) {
            info!("[SUCCESS] {} (mag: {}, stddev: {})", $name, mag, std_dev);
        } else {
            error!("[FAILED]  {} (mag: {}, stddev: {})", $name, mag, std_dev);
        }
    }};
}

macro_rules! baro_test {
    ($name:expr, $sensor:expr, $hertz:expr) => {{
        let mut pressures: heapless::Vec<f32, $hertz> = heapless::Vec::new();
        let mut temps: heapless::Vec<f32, $hertz> = heapless::Vec::new();
        let mut ticker = Ticker::every(Duration::from_millis(1000 / $hertz));
        for _i in 0..$hertz {
            $sensor.tick().await;
            let _ = pressures.push($sensor.pressure().unwrap_or_default());
            let _ = temps.push($sensor.temperature().unwrap_or_default());
            ticker.next().await;
        }

        let len = pressures.len() as f32;

        let mean_pressure: f32 = pressures.iter().sum::<f32>() / len;
        let mean_temp: f32 = temps.iter().sum::<f32>() / len;

        let var_pressure = pressures.iter().map(|v| (mean_pressure - v).powi(2)).sum::<f32>() / len;
        let std_dev_pressure = var_pressure.sqrt();

        info!("[SUCCESS] {} (pressure: {}, stddev: {}, temp: {})", $name, mean_pressure, std_dev_pressure, mean_temp);
    }};
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let (mut board, settings, seed) = flight_computer_firmware::init_board().await;

    // Init network stack
    static ETH_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let config = embassy_net::Config::dhcpv4(Default::default());
    let (stack, runner) = embassy_net::new(board.ethernet, config, ETH_RESOURCES.init(StackResources::new()), seed);

    // Launch network task
    spawner.spawn(net_task(runner)).unwrap();

    Timer::after(Duration::from_secs(3)).await;

    info!("===== STARTING SELF TEST =====");

    sensor_test!("IMU1 accelerometer", board.sensors.imu1, accelerometer, 1000, 9.5..10.5, 0.0001..1.0);
    sensor_test!("IMU1 gyroscope    ", board.sensors.imu1, gyroscope, 1000, 0.1..5.0, 0.0001..1.0);
    sensor_test!("IMU2 accelerometer", board.sensors.imu2, accelerometer, 1000, 9.5..10.5, 0.0001..1.0);
    sensor_test!("IMU2 gyroscope    ", board.sensors.imu2, gyroscope, 1000, 0.1..5.0, 0.0001..1.0);
    sensor_test!("IMU3 accelerometer", board.sensors.imu3, accelerometer, 1000, 9.5..10.5, 0.0001..1.0);
    sensor_test!("IMU3 gyroscope    ", board.sensors.imu3, gyroscope, 1000, 0.1..5.0, 0.0001..1.0);
    sensor_test!("High-G acc.       ", board.sensors.highg, accelerometer, 1000, 9.5..10.5, 0.0001..0.001);
    sensor_test!("Magnetometer      ", board.sensors.mag, magnetometer, 1000, 1.0..500.0, 0.0001..1.0);
    baro_test!("Baro 1            ", board.sensors.baro1, 1000);
    baro_test!("Baro 2            ", board.sensors.baro2, 1000);
    baro_test!("Baro 3            ", board.sensors.baro3, 1000);

    let result = with_timeout(Duration::from_secs(1), stack.wait_config_up()).await;
    match (result, stack.config_v4()) {
        (Ok(_), Some(config)) => info!("[SUCCESS] Ethernet ({})", config.address),
        _ => error!("[FAILED]  Ethernet"),
    }

    if let Some(s) = run_lora_loopback_test(&mut board.lora1, &mut board.lora2).await {
        info!("[SUCCESS] LoRa 1 -> LoRa 2 (RSSI: {}, SNR: {}, loss: {})", s.rssi, s.snr, s.rssi - LORA_TEST_TX_POWER);
    } else {
        error!("[FAILED]  LoRa 1 -> LoRa 2: FAILED");
    };

    Timer::after(Duration::from_millis(200)).await;

    if let Some(s) = run_lora_loopback_test(&mut board.lora2, &mut board.lora1).await {
        info!("[SUCCESS] LoRa 2 -> LoRa 1 (RSSI: {}, SNR: {}, loss: {})", s.rssi, s.snr, s.rssi - LORA_TEST_TX_POWER);
    } else {
        error!("[FAILED]  LoRa 2 -> LoRa 1");
    };

    if run_can_loopback_test(&mut board.can1, &mut board.can2).await {
        info!("[SUCCESS] CAN 1 -> CAN 2 (1Mbps)");
    } else {
        error!("[FAILED]  CAN 1 -> CAN 2 (1Mbps)");
    }

    if run_can_loopback_test(&mut board.can2, &mut board.can1).await {
        info!("[SUCCESS] CAN 2 -> CAN 1 (1Mbps)");
    } else {
        error!("[FAILED]  CAN 2 -> CAN 1 (1Mbps)");
    }

    info!("===== SELF TEST COMPLETE =====");

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        board.outputs.leds.0.toggle();
        board.outputs.leds.1.toggle();
        board.outputs.leds.2.toggle();
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Ethernet<'static, ETH, GenericSMI>>) -> ! {
    runner.run().await
}
