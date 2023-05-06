use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::cell::RefCell;
use core::convert::{TryFrom, TryInto};
use core::ops::DerefMut;

use cortex_m::interrupt::{free, Mutex};
use embedded_hal::serial::Read;
use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::gpio::{Pin};
use hal::pac::{interrupt, Interrupt, USART2};
use hal::rcc::Clocks;
use hal::serial::Serial;

use ublox::{AlignmentToReferenceTime, CfgPrtUartBuilder, CfgRateBuilder, UartPortId};

use crate::prelude::*;
use crate::telemetry::*;

const DESIRED_BAUD_RATE: u32 = 115_200;
const BAUD_RATE_OPTIONS: [u32; 2] = [115_200, 9600];
//const BAUD_RATE_OPTIONS: [u32; 8] = [115_200, 9600, 460800, 230400, 57600, 38400, 19200, 4800];
const MEASUREMENT_RATE_MS: u16 = 100;

static UART: Mutex<RefCell<Option<Serial<USART2>>>> = Mutex::new(RefCell::new(None));
static NMEA_READY: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static mut NMEA_BUFFER: Option<Vec<u8>> = None;

impl TryFrom<&str> for GPSFixType {
    type Error = ();

    fn try_from(x: &str) -> Result<Self, Self::Error> {
        match x {
            "0" => Ok(GPSFixType::NoFix),
            "1" => Ok(GPSFixType::AutonomousFix),
            "2" => Ok(GPSFixType::DifferentialFix),
            "4" => Ok(GPSFixType::RTKFix),
            "5" => Ok(GPSFixType::RTKFloat),
            "6" => Ok(GPSFixType::DeadReckoningFix),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum GPSState {
    Init,
    Active,
    Error,
}

pub struct GPS {
    state: GPSState,
    utc_time: Option<String>,
    /// latitude in decimal degrees
    pub latitude: Option<f32>,
    /// longitude in decimal degrees
    pub longitude: Option<f32>,
    /// altitude in m (ASL)
    pub altitude: Option<f32>,
    pub fix: GPSFixType,
    pub num_satellites: u8,
    pub hdop: u16, // hdop * 100
    last_baud_change: u32,
    baud_rate: u32,
}

impl GPS {
    pub fn init(dp_uart: USART2, tx: Pin<'A', 2>, rx: Pin<'A', 3>, clocks: &Clocks) -> Self {
        let baud = BAUD_RATE_OPTIONS[0];
        log!(Info, "Starting with baud rate {}.", baud);
        let config = hal::serial::config::Config::default()
            .baudrate(baud.bps())
            .parity_none();
        let mut uart = dp_uart.serial(
            (tx.into_alternate(), rx.into_alternate()),
            config,
            clocks
        ).unwrap();
        uart.listen(hal::serial::Event::Rxne);

        free(|cs| {
            *UART.borrow(cs).borrow_mut() = Some(uart);
        });

        // Initialize NMEA buffer for interrupt
        unsafe {
            NMEA_BUFFER.replace(Vec::with_capacity(256));
        }

        // Enable RX interrupt
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        Self {
            state: GPSState::Init,
            utc_time: None,
            latitude: None,
            longitude: None,
            altitude: None,
            fix: GPSFixType::NoFix,
            num_satellites: 0,
            hdop: 9999,
            last_baud_change: 0,
            baud_rate: baud,
        }
    }

    fn send_message(&mut self, buf: &[u8]) -> Result<(), hal::serial::Error> {
        let mut uart = free(|cs| UART.borrow(cs).replace(None)).unwrap();

        uart.bwrite_all(buf)?;
        uart.bflush()?;

        free(|cs| {
            *UART.borrow(cs).borrow_mut() = Some(uart);
        });

        Ok(())
    }

    fn change_baud_rate(&mut self, clocks: &Clocks, new_baud_rate: u32) {
        let uart = free(|cs| UART.borrow(cs).replace(None)).unwrap();
        let (dp_uart, (tx, rx)) = uart.release();
        let config = hal::serial::config::Config::default()
            .baudrate(new_baud_rate.bps())
            .parity_none();
        let mut uart = dp_uart.serial((tx, rx), config, clocks).unwrap();
        uart.listen(hal::serial::Event::Rxne);

        self.baud_rate = new_baud_rate;

        free(|cs| {
            *UART.borrow(cs).borrow_mut() = Some(uart);
        });
    }

    fn configure(&mut self, clocks: &Clocks) -> Result<(), hal::serial::Error> {
        cortex_m::peripheral::NVIC::mask(Interrupt::USART2);

        if self.baud_rate != DESIRED_BAUD_RATE {
            log!(Info, "Changing baud rate to {}...", DESIRED_BAUD_RATE);

            // Sending this NMEA message won't actually change the baud rate
            // for some reason, but it will allow us to send the UBX message,
            // which will.
            let payload = format!("PUBX,41,1,0007,0003,{},0", DESIRED_BAUD_RATE);
            let checksum = payload.chars().fold(0, |a, b| (a as u8) ^ (b as u8));
            let msg = format!("${}*{:02X}\r\n", payload, checksum);
            self.send_message(&msg.into_bytes())?;

            // Now that the RX is listening for UBX, set the baud rate.
            self.send_message(
                &CfgPrtUartBuilder {
                    portid: UartPortId::Uart1,
                    reserved0: 0,
                    tx_ready: 0,
                    mode: 0x8c0,
                    baud_rate: DESIRED_BAUD_RATE,
                    in_proto_mask: 0x07,
                    out_proto_mask: 0x03,
                    flags: 0,
                    reserved5: 0,
                }
                .into_packet_bytes(),
            )?;

            self.change_baud_rate(clocks, DESIRED_BAUD_RATE);
        }

        // Apply non-baud-related config items, such as measurement rate
        log!(Info, "Setting GPS measurement frequency...");
        self.send_message(
            &CfgRateBuilder {
                measure_rate_ms: MEASUREMENT_RATE_MS,
                nav_rate: 1,
                time_ref: AlignmentToReferenceTime::Gps,
            }
            .into_packet_bytes(),
        )?;

        log!(Info, "GPS successfully initialized.");

        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        Ok(())
    }

    fn next_baud_rate(&mut self, clocks: &Clocks) {
        let i = BAUD_RATE_OPTIONS.iter().position(|b| *b == self.baud_rate).unwrap();
        if i >= BAUD_RATE_OPTIONS.len() - 1 {
            self.state = GPSState::Error;
            log!(Error, "Failed to connect to GPS: No baud rate found.");
            return;
        }

        cortex_m::peripheral::NVIC::mask(Interrupt::USART2);
        self.change_baud_rate(clocks, BAUD_RATE_OPTIONS[i + 1]);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }
    }

    pub fn tick(&mut self, time: u32, clocks: &Clocks) {
        // No NMEA line is ready yet
        if !free(|cs| NMEA_READY.borrow(cs).replace(false)) {
            // We are in the setup phase and it's been a while, maybe we need
            // to change baud rates.
            if self.state == GPSState::Init && time - self.last_baud_change > 1500 {
                self.next_baud_rate(clocks);
                self.last_baud_change = time;
            }

            return;
        }

        // Grab the first complete NMEA line from the start of the buffer if
        // possible.
        let nmea: Option<String> = free(|_cs| {
            let buffer = unsafe { NMEA_BUFFER.as_mut().unwrap() };
            buffer
                .iter()
                .position(|b| *b == '\n' as u8)
                .map(|i| buffer.drain(0..=i).collect::<Vec<u8>>())
                .map(|b| String::from_utf8_lossy(&b).to_string())
        });

        if let Some(line) = nmea {
            // We only care about GGA messages, those contain coordinates/fix info
            if line.get(3..=5).unwrap_or("XXX") != "GGA" {
                return;
            }

            let segments: Vec<&str> = line.split(',').collect();
            if segments.len() < 15 {
                return;
            }

            self.utc_time = Some(segments[1].to_string());

            // Latitude needs to converted from degrees and minutes to decimal degrees
            // Lat: DDMM.MM... Lng: DDDMM.MM...
            self.latitude = (segments[2].len() > 2)
                .then(|| segments[2].split_at(2))
                .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok()))
                .flatten()
                .map(|(d, m)| (d + m / 60.0) * if segments[3] == "N" { 1.0 } else { -1.0 });
            self.longitude = (segments[4].len() > 3)
                .then(|| segments[4].split_at(3))
                .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok()))
                .flatten()
                .map(|(d, m)| (d + m / 60.0) * if segments[5] == "E" { 1.0 } else { -1.0 });
            self.altitude = segments[9].parse::<f32>().ok();

            self.fix = segments[6].try_into().unwrap_or(GPSFixType::NoFix);
            self.num_satellites = segments[7].parse().unwrap_or(0);
            self.hdop = (segments[8].parse::<f32>().unwrap_or(99.99) * 100.0) as u16;

            // This seems to be the very first NMEA line we received, so make
            // sure we configure the GPS (and change baud rate if necessary)
            if self.state != GPSState::Active {
                if let Err(e) = self.configure(clocks) {
                    log!(Error, "Failed to configure GPS: {:?}", e);
                    self.state = GPSState::Error;
                } else {
                    self.state = GPSState::Active;
                }
            }
        }
    }
}

#[interrupt]
fn USART2() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::USART2);

    free(|cs| {
        if let Some(ref mut uart) = UART.borrow(cs).borrow_mut().deref_mut() {
            let buffer = unsafe { NMEA_BUFFER.as_mut().unwrap() };

            while let Ok(b) = uart.read() {
                // NMEA sequences always start with a $ ...
                if buffer.len() > 0 || b == '$' as u8 {
                    buffer.push(b);
                    // ... and end with a line feed.
                    if b == '\n' as u8 {
                        // Set the READY flag so the main loop knows a new NMEA
                        // line is ready.
                        *NMEA_READY.borrow(cs).borrow_mut() = true;
                    }
                }

                if buffer.len() >= 256 {
                    buffer.truncate(0);
                }
            }
        }
    });
}
