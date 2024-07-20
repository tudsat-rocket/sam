use embassy_executor::{SendSpawner, Spawner};
use embassy_stm32::{gpio::Output, time::Hertz};
use embassy_stm32::peripherals::*;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, Receiver}};
use embassy_time::{Duration, Ticker};
use embedded_hal::digital::OutputPin;

use shared_types::{CanBusMessageId, IoBoardRole};
use static_cell::StaticCell;

use crate::{BoardIo, CanInChannel, CanOutChannel, InputMode};

const OUTPUT_FAILSAFE_TIMEOUT_MILLIS: u64 = 500;

type OutputStateChannel = Channel::<CriticalSectionRawMutex, ([bool; 8], bool), 5>;
static OUTPUT_STATE_CHANNEL: StaticCell<OutputStateChannel> = StaticCell::new();

pub struct Acs {}

impl crate::roles::BoardRole for Acs {
    const ROLE_ID: IoBoardRole = IoBoardRole::Acs;

    // TODO
    fn input1_mode() -> InputMode {
        InputMode::I2c(Hertz::khz(100), embassy_stm32::i2c::Config::default())
    }

    fn spawn(
        io: BoardIo,
        high_priority_spawner: SendSpawner,
        low_priority_spawner: Spawner,
        can_in: &'static CanInChannel,
        _can_out: &'static CanOutChannel
    ) {
        let output_state_channel = OUTPUT_STATE_CHANNEL.init(Channel::new());

        let output_handle = crate::roles::common::run_outputs(
            io.output1,
            io.output2,
            io.output3,
            io.output4,
            can_in.subscriber().unwrap(),
            CanBusMessageId::IoBoardCommand(Self::ROLE_ID, 0).into(),
            Some(Duration::from_millis(OUTPUT_FAILSAFE_TIMEOUT_MILLIS)),
            Some(output_state_channel.sender())
        );

        //let sensor_handle = run_sensors(
        //    can_out.publisher().unwrap(),
        //);

        let led_handle = run_leds(io.leds, output_state_channel.receiver());

        high_priority_spawner.spawn(output_handle).unwrap();
        low_priority_spawner.spawn(led_handle).unwrap();
        //low_priority_spawner.spawn(sensor_handle).unwrap();

        // TODO: report back general temperature and voltages?
    }
}

#[embassy_executor::task]
pub async fn run_sensors(
    // TODO: input
    _publisher: crate::CanOutPublisher,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    loop {
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn run_leds(
    leds: (Output<'static, PB12>, Output<'static, PB13>, Output<'static, PB14>),
    output_state_receiver: Receiver<'static, CriticalSectionRawMutex, ([bool; 8], bool), 5>
) -> ! {
    let (mut led_red, mut led_white, mut led_yellow) = leds;
    led_red.set_low();
    led_white.set_high();
    led_yellow.set_high();

    loop {
        let (outputs, failsafe) = output_state_receiver.receive().await;
        //defmt::println!("get output report {:?}, {:?}", outputs, failsafe);
        let _ = led_red.set_state((!failsafe).into());
        let _ = led_white.set_state((!outputs[0]).into());
        let _ = led_yellow.set_state((!outputs[1]).into());
    }
}
