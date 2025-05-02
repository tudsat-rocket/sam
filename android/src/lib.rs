use std::num::NonZeroU32;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;
use std::time::Instant;

use log::*;

use egui::ViewportId;
use egui_wgpu::wgpu;
use egui_wgpu::winit::Painter;
use egui_winit::{winit, State};

use winit::event::Event::*;
use winit::event_loop::{EventLoop, EventLoopBuilder, EventLoopWindowTarget};
use winit::platform::android::activity::AndroidApp;

use sam::data_source::SerialStatus;
use sam::settings::AppSettings;
use sam::Sam;
use shared_types::telemetry::*;

const INITIAL_WIDTH: u32 = 1920;
const INITIAL_HEIGHT: u32 = 1080;

static mut DOWNLINK_MESSAGE_SENDER: Option<Sender<(Instant, DownlinkMessage)>> = None;
static mut UPLINK_MESSAGE_RECEIVER: Option<Receiver<UplinkMessage>> = None;
static mut SERIAL_STATUS_SENDER: Option<Sender<SerialStatus>> = None;
static mut DOWNLINK_BUFFER: Vec<u8> = Vec::new();

/// A custom event type for the winit app.
enum Event {
    RequestRedraw,
}

/// Enable egui to request redraws via a custom Winit event...
#[derive(Clone)]
struct RepaintSignal(std::sync::Arc<std::sync::Mutex<winit::event_loop::EventLoopProxy<Event>>>);

fn create_window<T>(
    event_loop: &EventLoopWindowTarget<T>,
    state: &mut State,
    painter: &mut Painter,
) -> Option<Arc<winit::window::Window>> {
    let window = winit::window::WindowBuilder::new()
        .with_decorations(true)
        .with_resizable(true)
        .with_transparent(false)
        .with_title("egui winit + wgpu example")
        .with_inner_size(winit::dpi::PhysicalSize {
            width: INITIAL_WIDTH,
            height: INITIAL_HEIGHT,
        })
        .build(event_loop)
        .unwrap();
    let window = Arc::new(window);

    if let Err(err) = pollster::block_on(painter.set_window(ViewportId::ROOT, Some(window.clone()))) {
        log::error!("Failed to associate new Window with Painter: {err:?}");
        return None;
    }

    // NB: calling set_window will lazily initialize render state which
    // means we will be able to query the maximum supported texture
    // dimensions
    if let Some(max_size) = painter.max_texture_side() {
        state.set_max_texture_side(max_size);
    }

    window.request_redraw();

    Some(window)
}

fn _main(event_loop: EventLoop<Event>) {
    let (sender, receiver) = std::sync::mpsc::channel::<(Instant, DownlinkMessage)>();
    unsafe {
        DOWNLINK_MESSAGE_SENDER = Some(sender);
        sam::data_source::serial::DOWNLINK_MESSAGE_RECEIVER = Some(receiver);
    }
    let (sender, receiver) = std::sync::mpsc::channel::<UplinkMessage>();
    unsafe {
        UPLINK_MESSAGE_RECEIVER = Some(receiver);
        sam::data_source::serial::UPLINK_MESSAGE_SENDER = Some(sender);
    }
    let (sender, receiver) = std::sync::mpsc::channel::<SerialStatus>();
    unsafe {
        SERIAL_STATUS_SENDER = Some(sender);
        sam::data_source::serial::SERIAL_STATUS_RECEIVER = Some(receiver);
    }

    let mut ctx = egui::Context::default();
    ctx.tessellation_options_mut(|options| {
        options.feathering = false;
    });
    let repaint_signal = RepaintSignal(std::sync::Arc::new(std::sync::Mutex::new(event_loop.create_proxy())));
    ctx.set_request_repaint_callback(move |_info| {
        log::debug!("Request Repaint Callback");
        repaint_signal.0.lock().unwrap().send_event(Event::RequestRedraw).ok();
    });

    let mut state = State::new(ctx.clone(), ViewportId::ROOT, &event_loop, Some(1.0), None); // TODO
    let mut painter = Painter::new(
        egui_wgpu::WgpuConfiguration {
            supported_backends: wgpu::Backends::all(),
            power_preference: wgpu::PowerPreference::LowPower,
            device_descriptor: std::sync::Arc::new(|_adapter| wgpu::DeviceDescriptor {
                label: None,
                required_features: wgpu::Features::default(),
                required_limits: wgpu::Limits {
                    max_compute_workgroup_size_x: 128,
                    max_compute_workgroup_size_y: 128,
                    max_compute_invocations_per_workgroup: 128,
                    ..wgpu::Limits::default()
                },
            }),
            present_mode: wgpu::PresentMode::Fifo,
            ..Default::default()
        },
        1, // msaa samples
        Some(wgpu::TextureFormat::Depth24Plus),
        false,
    );
    let mut window: Option<Arc<winit::window::Window>> = None;

    let settings = AppSettings::load().unwrap_or_default();
    let mut sam = Sam::init(&ctx.clone(), settings.clone(), None);

    event_loop.run(move |event, event_loop| match event {
        Suspended => {
            window = None;
        }
        Resumed => match &window {
            None => {
                window = create_window(event_loop, &mut state, &mut painter);
            }
            Some(window) => {
                pollster::block_on(painter.set_window(ViewportId::ROOT, Some(window.clone()))).unwrap_or_else(|err| {
                    error!("Failed to associate window with painter after resume event: {err:?}")
                });
                window.request_redraw();
            }
        },
        //MainEventsCleared | UserEvent(Event::RequestRedraw) => {
        UserEvent(Event::RequestRedraw) => {
            if let Some(window) = window.as_ref() {
                debug!("Winit event (main events cleared or user event) - request_redraw()");
                window.request_redraw();
            }
        }
        WindowEvent { event, .. } => {
            log::debug!("Window Event: {event:?}");
            match event {
                winit::event::WindowEvent::Resized(size) => {
                    painter.on_window_resized(
                        ViewportId::ROOT,
                        NonZeroU32::new(size.width).unwrap(),
                        NonZeroU32::new(size.height).unwrap(),
                    );
                }
                winit::event::WindowEvent::CloseRequested => {
                    //*control_flow = ControlFlow::Exit;
                }
                winit::event::WindowEvent::RedrawRequested => {
                    if let Some(window) = window.as_ref() {
                        //debug!("RedrawRequested, with window set");
                        let raw_input = state.take_egui_input(window);

                        //debug!("RedrawRequested: calling ctx.run()");
                        let full_output = ctx.run(raw_input, |ctx| {
                            sam.ui(ctx);
                        });
                        //debug!("RedrawRequested: called ctx.run()");
                        // TODO
                        state.handle_platform_output(window, full_output.platform_output);

                        //debug!("RedrawRequested: calling paint_and_update_textures()");
                        painter.paint_and_update_textures(
                            ViewportId::ROOT,
                            window.scale_factor() as f32,
                            [0.0, 0.0, 0.0, 0.0],
                            &ctx.tessellate(full_output.shapes, window.scale_factor() as f32),
                            &full_output.textures_delta,
                            false, // capture
                        );
                    } else {
                        debug!("RedrawRequested, with no window set");
                    }
                }
                _ => {}
            }

            if let Some(window) = window.as_ref() {
                let response = state.on_window_event(window, &event);
                if response.repaint {
                    window.request_redraw();
                }
            }
        }
        _ => (),
    });
}

#[no_mangle]
fn android_main(app: AndroidApp) {
    use winit::platform::android::EventLoopBuilderExtAndroid;

    android_logger::init_once(android_logger::Config::default().with_min_level(log::Level::Debug));

    let event_loop = EventLoopBuilder::with_user_event().with_android_app(app).build().unwrap();
    _main(event_loop);
}

#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn Java_space_tudsat_sam_MainActivity_notifyOnNewIntent<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    _activity: jni::objects::JObject<'local>,
) {
    info!("onNewIntent was called!");
}

#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn Java_space_tudsat_sam_MainActivity_notifyOnNewLocation<'local>(
    _env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    _activity: jni::objects::JObject<'local>,
    lat: f64,
    lng: f64,
    alt: f64,
    acc: f64,
) {
    info!("{:?}", (lat, lng, alt, acc));
}

#[allow(non_snake_case)]
#[no_mangle]
pub extern "system" fn Java_space_tudsat_sam_MainActivity_notifyOnNewUsbData<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    bytes: jni::objects::JByteArray<'local>,
    len: i32,
) {
    let size = env.get_array_length(&bytes).unwrap_or(0) as usize;
    let size = usize::min(size, len as usize);
    let mut vec = Vec::with_capacity(size);
    unsafe {
        let slice = std::slice::from_raw_parts_mut(vec.as_mut_ptr() as *mut jni::sys::jbyte, size);
        let _ = env.get_byte_array_region(bytes, 0, slice);
        vec.set_len(size);
    }

    unsafe {
        DOWNLINK_BUFFER.extend(vec.into_iter().map(|x| x as u8));
    }

    // If there exists a zero in our downlink_buffer, that suggests there
    // is a complete COBS-encoded message in there
    while let Some(index) = unsafe { DOWNLINK_BUFFER.iter().position(|b| *b == 0) } {
        // Split of the first message, including the zero delimiter
        let (serialized, rest) = unsafe { DOWNLINK_BUFFER.split_at_mut(index + 1) };
        let mut serialized = serialized.to_vec();

        // Store the rest in the downlink_buffer, after having removed
        // the current message
        unsafe {
            DOWNLINK_BUFFER = rest.to_vec();
        }

        // Attempt to parse the message, discarding it if unsuccessful
        let msg = match postcard::from_bytes_cobs(serialized.as_mut_slice()) {
            Ok(msg) => msg,
            Err(_e) => continue,
        };

        // If successful, send msg through channel.
        unsafe {
            if let Some(sender) = DOWNLINK_MESSAGE_SENDER.as_mut() {
                let _ = sender.send((Instant::now(), msg));
            }
        }
    }
}

#[allow(non_snake_case)]
#[no_mangle]
pub extern "system" fn Java_space_tudsat_sam_MainActivity_notifyOnNewUsbConnectionStatus<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
    connected: bool,
) {
    unsafe {
        if let Some(sender) = SERIAL_STATUS_SENDER.as_mut() {
            sender.send(connected.then_some(SerialStatus::Connected).unwrap_or(SerialStatus::Error)).unwrap();
        }
    }
}

#[allow(non_snake_case)]
#[no_mangle]
pub extern "system" fn Java_space_tudsat_sam_MainActivity_checkForUplinkData<'local>(
    env: jni::JNIEnv<'local>,
    _class: jni::objects::JClass<'local>,
) -> jni::objects::JByteArray<'local> {
    if let Some(msg) = unsafe { UPLINK_MESSAGE_RECEIVER.as_mut().unwrap().try_iter().next() } {
        let serialized = msg.serialize().unwrap_or_default();
        env.byte_array_from_slice(&serialized).unwrap()
    } else {
        jni::objects::JObject::null().into()
    }
}
