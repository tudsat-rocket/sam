//! A serial port data source. The default.

use std::any::Any;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::slice::Iter;
use std::sync::mpsc::{Receiver, SendError, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;
#[cfg(target_arch = "wasm32")]
use web_time::Instant;

use futuresdr::anyhow::Result;
use futuresdr::blocks::seify::SourceBuilder;
use futuresdr::blocks::*;
use futuresdr::macros::connect;
use futuresdr::num_complex::Complex32;
use futuresdr::runtime::buffer::circular::Circular;
use futuresdr::runtime::Flowgraph;
use futuresdr::runtime::Runtime;
use futuresdr::runtime::Pmt;

use lora::Decoder;
use lora::Deinterleaver;
use lora::FftDemod;
use lora::FrameSync;
use lora::GrayMapping;
use lora::HammingDec;
use lora::HeaderDecoder;
use lora::HeaderMode;

use eframe::epaint::Color32;
use log::*;

use mithril::settings::*;
use mithril::telemetry::*;

use crate::data_source::DataSource;
use crate::settings::AppSettings;

const SAMPLE_RATE: u32 = 14_000_000;
const FREQUENCY: u32 = 866_500_000;
const GAIN: f64 = 50.0;

const LORA_BANDWIDTH: u32 = 500_000;
const SPREADING_FACTOR: usize = 7;
const SYNC_WORD: usize = 0x12;
const SOFT_DECODING: bool = false;

const RX_PACKET_SIZE: usize = 26;

pub struct SDRDataSource {
    downlink_rx: Receiver<DownlinkMessage>,
    uplink_tx: Sender<UplinkMessage>,

    lora_settings: LoRaSettings,

    telemetry_log_path: PathBuf,
    telemetry_log_file: Result<File, std::io::Error>,

    vehicle_states: Vec<(Instant, VehicleState)>,
    fc_settings: Option<Settings>,
    message_receipt_times: VecDeque<(Instant, u32)>,
    last_time: Option<Instant>,
}

fn lora_decoder_for_channel(mut fg: &mut Flowgraph, channel_freq: u32) -> Result<(usize, usize), futuresdr::anyhow::Error> {
    let freq_shifter = FrequencyShifter::new(-((channel_freq - FREQUENCY) as f32), SAMPLE_RATE as f32);
    let downsample = FirBuilder::new_resampling::<Complex32, Complex32>(1, (SAMPLE_RATE / LORA_BANDWIDTH) as usize);
    let frame_sync = FrameSync::new(FREQUENCY, LORA_BANDWIDTH, SPREADING_FACTOR, false, vec![SYNC_WORD], 1, None);
    let null_sink = NullSink::<f32>::new();
    let fft_demod = FftDemod::new(SOFT_DECODING, true, SPREADING_FACTOR);
    let gray_mapping = GrayMapping::new(SOFT_DECODING);
    let deinterleaver = Deinterleaver::new(SOFT_DECODING);
    let hamming_dec = HammingDec::new(SOFT_DECODING);
    let header = HeaderMode::Implicit {
        payload_len: RX_PACKET_SIZE,
        has_crc: true,
        code_rate: 0x00,
    };
    let header_decoder = HeaderDecoder::new(header, false);
    let decoder = Decoder::new();

    connect!(fg, freq_shifter > downsample [Circular::with_size(2 * 4 * 8192 * 4)] frame_sync > fft_demod > gray_mapping > deinterleaver > hamming_dec > header_decoder;
        frame_sync.log_out > null_sink;
        header_decoder.frame_info | frame_sync.frame_info;
        header_decoder | decoder);

    Ok((freq_shifter, decoder))
}

fn run_flowgraph(
    ctx: egui::Context,
    downlink_tx: Sender<DownlinkMessage>,
    _uplink_tx: Receiver<UplinkMessage>
) -> Result<(), futuresdr::anyhow::Error> {
    let rt = Runtime::new();
    let mut fg = Flowgraph::new();

    let mut src = SourceBuilder::new()
        .sample_rate(SAMPLE_RATE as f64)
        .frequency(FREQUENCY as f64)
        .gain(GAIN);

    let src = fg.add_block(src.build().unwrap());

    let splitter = SplitN::<Complex32, 7>::new();
    let join = MessageTee::<7, 1>::new();

    let (msg_tx, mut msg_rx) = futures::channel::mpsc::channel(200);
    let msg_sink = MessagePipe::new(msg_tx);

    connect!(fg, src > splitter; join | msg_sink);

    for i in 0..7 {
        let freq = 863_000_000 + LORA_BANDWIDTH/2 + LORA_BANDWIDTH*i;
        let (decoder_in, decoder_out) = lora_decoder_for_channel(&mut fg, freq)?;

        fg.connect_stream(splitter, format!("out{}", i), decoder_in, "in");
        fg.connect_message(decoder_out, "data", join, format!("in{}", i));
    }

    rt.spawn_background(async move {
        loop {
            if let Ok(Some(Pmt::Blob(mut msg))) = msg_rx.try_next() {
                if msg.len() < RX_PACKET_SIZE {
                    continue;
                }
                // downlink messages have 16bit of hmac at the start
                let len = msg.len()-2;
                let (hmac, mut serialized) = msg[..len].split_at_mut(2);
                if let Some(msg) = postcard::from_bytes_cobs(serialized).ok() {
                    downlink_tx.send(msg);
                    ctx.request_repaint();
                }

            }
        }
    });

    let _ = rt.run(fg);

    Ok(())
}

impl SDRDataSource {
    pub fn new(ctx: &egui::Context, lora_settings: LoRaSettings) -> Self {
        let (downlink_tx, downlink_rx) = std::sync::mpsc::channel::<DownlinkMessage>();
        let (uplink_tx, uplink_rx) = std::sync::mpsc::channel::<UplinkMessage>();

        let ctx = ctx.clone();

        std::thread::spawn(move || {
            let _ = run_flowgraph(ctx, downlink_tx, uplink_rx);
        });

        let telemetry_log_path = Self::new_telemetry_log_path();
        let telemetry_log_file = File::create(&telemetry_log_path);

        Self {
            downlink_rx,
            uplink_tx,
            lora_settings,
            telemetry_log_path,
            telemetry_log_file,
            vehicle_states: Vec::new(),
            fc_settings: None,
            message_receipt_times: VecDeque::new(),
            last_time: None,
        }
    }

    /// No telemetry file needed because serial port does not work on
    /// web assembly. TODO: maybe create a NoopDataSource for wasm instead?
    #[cfg(target_arch = "wasm32")]
    fn new_telemetry_log_path() -> PathBuf {
        "telem.log".into()
    }

    /// Comes up with a new, unique path for a telemetry log file.
    #[cfg(not(target_arch = "wasm32"))] // TODO: time doesn't work on wasm
    fn new_telemetry_log_path() -> PathBuf {
        let now: chrono::DateTime<chrono::Utc> = std::time::SystemTime::now().into();

        #[cfg(not(target_os = "windows"))]
        {
            // TODO: put these in XDG directories?
            let name = format!("sam_log_{}.log", now.format("%+"));
            name.into()
        }

        #[cfg(target_os = "windows")]
        {
            let name = format!("sam_log_{}.log", now.format("%Y-%m-%dT%H%M%S"));
            let mut path = home::home_dir().unwrap();
            path.push::<PathBuf>(name.into());
            path
        }
    }

    /// Stores a received message in the telemetry log.
    fn write_to_telemetry_log(&mut self, msg: &DownlinkMessage) {
        // TODO
        if let Ok(f) = self.telemetry_log_file.as_mut() {
            let serialized = msg.serialize().unwrap_or_default(); // TODO
            if let Err(e) = f.write_all(&serialized) {
                error!("Error saving msg: {:?}", e);
            }
        }
    }
}

impl DataSource for SDRDataSource {
    fn update(&mut self, _ctx: &egui::Context) {
        self.message_receipt_times.retain(|(i, _)| i.elapsed() < Duration::from_millis(1000));

        let msgs: Vec<_> = self.downlink_rx.try_iter().collect();
        for msg in msgs.into_iter() {
            self.write_to_telemetry_log(&msg);

            // TODO
            if let DownlinkMessage::TelemetryGCS(..) = msg {
            } else {
                self.message_receipt_times.push_back((Instant::now(), msg.time()));
            }

            match msg {
                DownlinkMessage::Log(..) => {}
                DownlinkMessage::Settings(settings) => {
                    self.fc_settings = Some(settings);
                }
                _ => {
                    let now = Instant::now();
                    let vs: VehicleState = msg.into();
                    self.vehicle_states.push((now, vs.clone()));
                    self.last_time = Some(now);
                }
            }
        }
    }

    fn vehicle_states<'a>(&'a self) -> Iter<'_, (Instant, VehicleState)> {
        self.vehicle_states.iter()
    }

    fn fc_settings<'a>(&'a mut self) -> Option<&'a Settings> {
        self.fc_settings.as_ref()
    }

    fn fc_settings_mut<'a>(&'a mut self) -> Option<&'a mut Settings> {
        self.fc_settings.as_mut()
    }

    fn reset(&mut self) {
        self.telemetry_log_path = Self::new_telemetry_log_path();
        self.telemetry_log_file = File::create(&self.telemetry_log_path);
        self.vehicle_states.truncate(0);
        self.fc_settings = None;
        self.message_receipt_times.truncate(0);
    }

    #[cfg(not(any(target_arch = "wasm32", target_os="android")))]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        self.uplink_tx.send(msg)
    }

    #[cfg(target_os="android")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        unsafe {
            UPLINK_MESSAGE_SENDER.as_mut().unwrap().send(msg)
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn send(&mut self, msg: UplinkMessage) -> Result<(), SendError<UplinkMessage>> {
        Ok(())
    }

    fn send_command(&mut self, cmd: Command) -> Result<(), SendError<UplinkMessage>> {
        self.send(UplinkMessage::Command(cmd))
    }

    fn end(&self) -> Option<Instant> {
        let postroll = Duration::from_secs_f64(10.0);

        self.last_time.map(|t| {
            if t.elapsed() < postroll {
                Instant::now()
            } else {
                t + postroll
            }
        })
    }

    fn apply_settings(&mut self, settings: &AppSettings) {
        self.lora_settings = settings.lora.clone();
        self.send(UplinkMessage::ApplyLoRaSettings(self.lora_settings.clone())).unwrap();
    }

    fn link_quality(&self) -> Option<f32> {
        let telemetry_data_rate = self
            .vehicle_states
            .iter()
            .rev()
            .find_map(|(_t, msg)| msg.data_rate)
            .unwrap_or(TelemetryDataRate::Low);
        let expected = match telemetry_data_rate {
            // TODO: don't hardcode these?
            TelemetryDataRate::Low => 15,
            TelemetryDataRate::High => 35,
        };
        let percentage = ((self.message_receipt_times.len() as f32) / (expected as f32)) * 100.0;
        Some(f32::min(percentage, 100.0))
    }

    fn status_bar_ui(&mut self, ui: &mut egui::Ui) {
        #[cfg(not(target_arch = "wasm32"))]
        if ui.button("⏮  Reset").clicked() {
            self.reset();
        }

        #[cfg(not(target_arch = "wasm32"))]
        ui.separator();

        ui.label(self.fc_settings().map(|s| s.identifier.clone()).unwrap_or_default());

        let telemetry_log_info = match self.telemetry_log_file.as_ref() {
            Ok(_) => self.telemetry_log_path.as_os_str().to_string_lossy().to_string(),
            Err(e) => format!("{:?}", e),
        };

        ui.weak(format!("{}", telemetry_log_info));
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}


// copied from fsdr blocks
use std::marker::Copy;

use futuresdr::blocks::signal_source::NCO;
use futuresdr::runtime::Block;
use futuresdr::runtime::BlockMeta;
use futuresdr::runtime::BlockMetaBuilder;
use futuresdr::runtime::Kernel;
use futuresdr::runtime::MessageIo;
use futuresdr::runtime::MessageIoBuilder;
use futuresdr::runtime::StreamIo;
use futuresdr::runtime::StreamIoBuilder;
use futuresdr::runtime::WorkIo;

use async_trait::async_trait;

pub struct FrequencyShifter<A> where A: Send + 'static + Copy {
    _p1: std::marker::PhantomData<A>,
    nco: NCO,
}

impl<A> FrequencyShifter<A>
where
    A: Send + 'static + Copy,
    FrequencyShifter<A>: futuresdr::runtime::Kernel,
{
    #[allow(clippy::new_ret_no_self)]
    pub fn new(frequency: f32, sample_rate: f32) -> Block {
        let nco = NCO::new(
            0.0f32,
            2.0 * core::f32::consts::PI * frequency / sample_rate,
        );
        Block::new(
            BlockMetaBuilder::new("FrequencyShift").build(),
            StreamIoBuilder::new()
                .add_input::<A>("in")
                .add_output::<A>("out")
                .build(),
            MessageIoBuilder::<Self>::new().build(),
            FrequencyShifter {
                _p1: std::marker::PhantomData,
                nco,
            },
        )
    }
}

#[doc(hidden)]
#[async_trait]
impl Kernel for FrequencyShifter<Complex32> {
    async fn work(
        &mut self,
        io: &mut WorkIo,
        sio: &mut StreamIo,
        _mio: &mut MessageIo<Self>,
        _meta: &mut BlockMeta,
    ) -> Result<()> {
        let i = sio.input(0).slice::<Complex32>();
        let o = sio.output(0).slice::<Complex32>();

        let m = std::cmp::min(i.len(), o.len());
        if m > 0 {
            for (v, r) in i.iter().zip(o.iter_mut()) {
                *r = (*v) * Complex32::new(self.nco.phase.cos(), self.nco.phase.sin());
                self.nco.step();
            }

            sio.input(0).consume(m);
            sio.output(0).produce(m);
        }

        if sio.input(0).finished() && m == i.len() {
            io.finished = true;
        }

        Ok(())
    }
}


// generic split
pub struct SplitN<A, const N: usize> where A: Send + 'static + Copy {
    _p1: std::marker::PhantomData<A>,
}

impl<A, const N: usize> SplitN<A, N>
where
    A: Send + 'static + Copy,
    FrequencyShifter<A>: futuresdr::runtime::Kernel,
{
    #[allow(clippy::new_ret_no_self)]
    pub fn new() -> Block {
        let mut stream_io = StreamIoBuilder::new()
            .add_input::<A>("in");

        for i in 0..N {
            stream_io = stream_io.add_output::<A>(&format!("out{}", i));
        }

        Block::new(
            BlockMetaBuilder::new("FrequencyShift").build(),
            stream_io.build(),
            MessageIoBuilder::<Self>::new().build(),
            SplitN {
                _p1: std::marker::PhantomData,
            },
        )
    }
}

#[doc(hidden)]
#[async_trait]
impl<A, const N: usize> Kernel for SplitN<A, N>
where
    A: Send + 'static + Copy,
{
    async fn work(
        &mut self,
        io: &mut WorkIo,
        sio: &mut StreamIo,
        _mio: &mut MessageIo<Self>,
        _meta: &mut BlockMeta,
    ) -> Result<()> {
        let i = sio.input(0).slice::<A>();

        let mut m = i.len();
        for j in 0..N {
            m = usize::min(m, sio.output(j).slice::<A>().len());
        }

        for j in 0..N {
            let mut o = sio.output(j).slice::<A>();

            for k in 0..m {
                o[k] = i[k];
            }

            sio.output(j).produce(m);
        }

        sio.input(0).consume(m);

        if sio.input(0).finished() && m == i.len() {
            io.finished = true;
        }

        Ok(())
    }
}

