use embassy_stm32::gpio::Output;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use embassy_time::{with_timeout, Duration};
use embassy_stm32::peripherals::*;

use shared_types::{IoBoardOutputMessage, CanBusMessage};

fn set_outputs(
    output1: &mut (Output<'static, PB8>, Output<'static, PB9>),
    output2: &mut (Output<'static, PA0>, Output<'static, PA1>),
    output3: &mut (Output<'static, PC9>, Output<'static, PC8>),
    output4: &mut (Output<'static, PC7>, Output<'static, PC6>),
    outputs: &[bool; 8]
) {
    output1.0.set_level(outputs[0].into());
    output1.1.set_level(outputs[1].into());
    output2.0.set_level(outputs[2].into());
    output2.1.set_level(outputs[3].into());
    output3.0.set_level(outputs[4].into());
    output3.1.set_level(outputs[5].into());
    output4.0.set_level(outputs[6].into());
    output4.1.set_level(outputs[7].into());
}

async fn receive_output_message(subscriber: &mut crate::CanInSubscriper, command_id: u16) -> IoBoardOutputMessage {
    loop {
        match subscriber.next_message_pure().await {
            // TODO: properly filter, parse
            (sid, data) if sid == command_id => {
                if let Ok(Some(msg)) = IoBoardOutputMessage::parse(data) {
                    return msg;
                }
            },
            _ => {}
        }
    }
}

#[embassy_executor::task]
pub async fn run_outputs(
    mut output1: (Output<'static, PB8>, Output<'static, PB9>),
    mut output2: (Output<'static, PA0>, Output<'static, PA1>),
    mut output3: (Output<'static, PC9>, Output<'static, PC8>),
    mut output4: (Output<'static, PC7>, Output<'static, PC6>),
    mut subscriber: crate::CanInSubscriper,
    command_id: u16,
    timeout: Option<Duration>,
    notify_sender: Option<Sender<'static, CriticalSectionRawMutex, ([bool; 8], bool), 5>>,
) -> ! {
    loop {
        let (outputs, failsafe) = if let Some(timeout) = timeout {
            match with_timeout(timeout, receive_output_message(&mut subscriber, command_id)).await {
                Ok(msg) => (msg.outputs, false),
                Err(_) => ([false; 8], true)
            }
        } else {
            (receive_output_message(&mut subscriber, command_id).await.outputs, false)
        };

        set_outputs(&mut output1, &mut output2, &mut output3, &mut output4, &outputs);

        if let Some(sender) = notify_sender {
            let _ = sender.send((outputs, failsafe)).await;
        }
    }
}
