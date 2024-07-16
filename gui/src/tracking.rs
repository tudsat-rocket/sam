use std::sync::mpsc::{channel, Sender, Receiver};
use std::net::{IpAddr, UdpSocket};

use egui::{Modifiers, Key};
use frodo_tracking::TrackingInput;

const TRACKER_ADDRESS: &'static str = "10.0.0.99:4000";

pub struct TrackingHandle {
    tracking_input_sender: Sender<TrackingInput>,
}

impl TrackingHandle {
    pub fn process_inputs(&mut self, ctx: &egui::Context) {
        if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::ArrowLeft)) {
            self.tracking_input_sender.send(TrackingInput::ManualControlInput(-1.0, 0.0));
        } else if ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::ArrowRight)) {
            self.tracking_input_sender.send(TrackingInput::ManualControlInput(1.0, 0.0));
        }
    }
}

impl Default for TrackingHandle {
    fn default() -> Self {
        let (input_sender, input_receiver) = channel();

        std::thread::spawn(move || {
            let mut tracker = TrackingInterface::init(input_receiver);
            tracker.run();
        });

        Self {
            tracking_input_sender: input_sender,
        }
    }
}

pub struct TrackingInterface { // TODO: name
    tracking_input_receiver: Receiver<TrackingInput>,
    socket: UdpSocket,
}

impl TrackingInterface {
    pub fn init(receiver: Receiver<TrackingInput>) -> Self {
        let socket = UdpSocket::bind("0.0.0.0:9999").unwrap();
        socket.connect(TRACKER_ADDRESS).unwrap();
        Self {
            tracking_input_receiver: receiver,
            socket,
        }
    }

    pub fn run(self) -> ! {
        loop {
            let input = self.tracking_input_receiver.recv().unwrap();
            let mut buf: [u8; 128] = [0x00; 128];
            let serialized = postcard::to_slice_cobs(&input, &mut buf).unwrap();
            self.socket.send(serialized).unwrap();
        }
    }
}
