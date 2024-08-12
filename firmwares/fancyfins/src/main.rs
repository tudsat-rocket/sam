#![no_std]
#![no_main]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::Adc;
use embassy_stm32::Config;
use embassy_stm32::interrupt::{Priority, InterruptExt};
use embassy_stm32::peripherals::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Ticker, Delay, Timer};

use {defmt_rtt as _, panic_probe as _};

mod fin_tasks;
mod can;

use fin_tasks::*;
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
    let mut config = Config::default();
    config.rcc.hse = Some(Hertz::mhz(8));
    config.rcc.sys_ck = Some(Hertz::mhz(72));
    config.rcc.hclk = Some(Hertz::mhz(72));
    config.rcc.pclk1 = Some(Hertz::mhz(36));
    config.rcc.pclk2 = Some(Hertz::mhz(72));
    config.rcc.adcclk = Some(Hertz::mhz(14));
    let mut p = embassy_stm32::init(config);

    defmt::info!("Fin startup");
    
    // Start watchdog
    let mut iwdg = IndependentWatchdog::new(p.IWDG, 512_000); // 512ms timeout
    iwdg.unleash();

    let can_in = CAN_IN.init(PubSubChannel::new());
    let can_out = CAN_OUT.init(PubSubChannel::new());
    // Can RX on PB8 can TX on PB9
    let mut can = embassy_stm32::can::Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    // startup can
    can::spawn(can, low_priority_spawner, can_in.publisher().unwrap(), can_out.subscriber().unwrap()).await;
    
    // start tasks
    low_priority_spawner.spawn(iwdg_task(iwdg)).unwrap();

    low_priority_spawner.spawn(fin_tasks::flight_state_led(can_in.subscriber().unwrap())).unwrap();
    
    let mut adc = Adc::new(p.ADC1, &mut Delay);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::Cycles239_5);

    low_priority_spawner.spawn(fin_tasks::strain_gauge(adc, p.PC0, p.PC1, p.PC2, p.PC3, can_in.subscriber().unwrap())).unwrap();
    
}
