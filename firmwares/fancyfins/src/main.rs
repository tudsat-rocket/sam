#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::mode;
use embassy_stm32::peripherals::*;
use embassy_stm32::spi;
use embassy_stm32::spi::MODE_1;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::khz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Timer};
use shared_types::can::{
    CanMessage,
    engine::{EngineInfoMsg, EngineState},
    structure::{Can2aFrame, CanFrameId, MessageKind},
};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

mod animations;
mod can;
mod leds;
// mod strain_gauge;

use can::*;

// TODO
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

embassy_stm32::bind_interrupts!(struct Irqs {
    CAN1_RX1 => embassy_stm32::can::Rx1InterruptHandler<CAN>;
    CAN1_SCE => embassy_stm32::can::SceInterruptHandler<CAN>;
    USB_LP_CAN1_RX0 => embassy_stm32::can::Rx0InterruptHandler<CAN>;
    USB_HP_CAN1_TX => embassy_stm32::can::TxInterruptHandler<CAN>;
});

#[embassy_executor::task]
async fn iwdg_task(mut iwdg: IndependentWatchdog<'static, IWDG>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(256));
    loop {
        iwdg.pet();
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(low_priority_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();

    // config.rcc.hse = Some(Hertz::mhz(8));
    // config.rcc.sys_ck = Some(Hertz::mhz(72));
    // config.rcc.hclk = Some(Hertz::mhz(72));
    // config.rcc.pclk1 = Some(Hertz::mhz(36));
    // config.rcc.pclk2 = Some(Hertz::mhz(72));
    // config.rcc.adcclk = Some(Hertz::mhz(14));
    let p = embassy_stm32::init(config);

    // Remap CAN to be on PB8/9
    embassy_stm32::pac::AFIO.mapr().modify(|w| w.set_can1_remap(1));

    // Start watchdog
    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    iwdg.unleash();
    low_priority_spawner.spawn(iwdg_task(iwdg)).unwrap();

    // --- CAN
    let can_rx = CAN1_RX_CH.init(PubSubChannel::new());
    let can_tx = CAN1_TX_CH.init(PubSubChannel::new());
    // Can RX on PB8 can TX on PB9
    // let can = embassy_stm32::can::Can::new(p.CAN, p.PB8, p.PB9, Irqs);
    let can = embassy_stm32::can::Can::new(p.CAN, p.PA11, p.PA12, Irqs);

    // Start main CAN RX/TX tasks
    can::spawn_can1(can, low_priority_spawner, can_rx.publisher().unwrap(), can_tx.subscriber().unwrap()).await;

    low_priority_spawner.spawn(can_heartbeat(can_tx.publisher().unwrap())).unwrap();

    let mut spi_config = spi::Config::default();
    spi_config.frequency = khz(2400);
    spi_config.mode = MODE_1;
    let spi3: Spi<'static, mode::Blocking> = Spi::new_blocking_txonly_nosck(p.SPI3, p.PB5, spi_config);

    // Run LED task
    low_priority_spawner.spawn(leds::run(can_rx.subscriber().unwrap(), spi3)).unwrap();

    // Run strain gauge task. TODO: split flash into separate task?
    // let mut adc = Adc::new(p.ADC1, &mut Delay);
    // adc.set_sample_time(embassy_stm32::adc::SampleTime::Cycles239_5);

    // low_priority_spawner
    //     .spawn(strain_gauge::run(
    //         address,
    //         adc,
    //         p.PC0,
    //         p.PC1,
    //         p.PC2,
    //         p.PC3,
    //         flight_mode.subscriber().unwrap(),
    //         can_out.publisher().unwrap(),
    //     ))
    //     .unwrap();
}

#[embassy_executor::task]
async fn can_heartbeat(can_out_pub: CanTxPublisher) {
    let mut ticker = Ticker::every(Duration::from_secs(1));
    info!("can heartbeat started");
    Timer::after(Duration::from_secs(1)).await;
    loop {
        info!("can heatbeat");
        // let frame = Can2aFrame {id: CanFrameId {message_kind: MessageKind::SubsystemInfo}, payload: EngineState::Ready.into()};
        let message = CanMessage::Info(shared_types::can::SubsystemInfo::Engine(EngineInfoMsg::default()));
        can_out_pub.publish_immediate(message);
        ticker.next().await
    }
}
