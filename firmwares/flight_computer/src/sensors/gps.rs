use core::cell::{Cell, RefCell};
use core::ops::DerefMut;
use core::convert::{TryFrom, TryInto};
use alloc::vec::Vec;
use alloc::string::{String, ToString};
use alloc::format;

use stm32f4xx_hal as hal;
use hal::prelude::*;
use hal::pac::{interrupt, Interrupt, USART2};
use hal::gpio::{Pin, Alternate};
use hal::rcc::Clocks;
use hal::serial::{Serial, Rx};
use cortex_m::interrupt::{free, Mutex};
use embedded_hal::serial::Read;

use ublox::{AlignmentToReferenceTime, CfgPrtUartBuilder, CfgRateBuilder, UartPortId};

use crate::CLOCK_FREQ_MEGA_HERTZ;

use crate::logging::*;
use crate::telemetry::*;
use crate::{log, log_every_nth_time};
use crate::watchdog::*;

// TODO: with decent hardware, maybe go up to 460_800 & 50
const BAUD_RATE: u32 = 115_200;
const MEASUREMENT_RATE_MS: u16 = 100;

type SERIAL = Serial<USART2, (Pin<'A', 2, Alternate<7>>, Pin<'A', 3, Alternate<7>>)>;

static RX_PIN: Mutex<RefCell<Option<Rx<USART2, u8>>>> = Mutex::new(RefCell::new(None));
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
            _ => Err(())
        }
    }
}

pub struct GPS {
    //tx: Tx<USART2, u8>,
    utc_time: Option<String>,
    pub latitude: Option<f32>, // TODO: type?
    pub longitude: Option<f32>, // TODO: type?
    pub altitude: Option<f32>, // TODO: unit/type?
    pub fix: GPSFixType,
    pub num_satellites: u8,
    pub hdop: u16, // hdop * 100
}

fn read_complete_nmea_line(uart: &mut SERIAL, timeout_ms: u16) -> Result<String, ()> {
    let mut buffer: Vec<u8> = Vec::with_capacity(256);
    let max_cycles = (timeout_ms as u32) * 1000 * CLOCK_FREQ_MEGA_HERTZ;
    let cycles_start = hal::pac::DWT::cycle_count();

    while hal::pac::DWT::cycle_count().wrapping_sub(cycles_start) < max_cycles && buffer.len() < 256 {
        // This can take quite a while, so make sure to regularly reset IWDG
        reset_watchdog();

        if let Ok(b) = uart.read() {
            if buffer.len() > 0 || b == '*' as u8 {
                buffer.push(b);
            }
        }

        if *buffer.last().unwrap_or(&0) == '\n' as u8 {
            let string = String::from_utf8_lossy(buffer.as_slice()).to_string();
            return Ok(string);
        }
    }

    Err(())
}

fn detect_baud_rate(dp_uart: USART2, uart2_tx: Pin<'A', 2, Alternate<7>>, uart2_rx: Pin<'A', 3, Alternate<7>>, clocks: &Clocks) -> Result<(u32, SERIAL), ()> {
    // Since these are consumed when creating the UART port and recovered after
    // releasing it, we need some Rust magic to store these.
    let uart_cell = Cell::new(Some(dp_uart));
    let tx_cell = Cell::new(Some(uart2_tx));
    let rx_cell = Cell::new(Some(uart2_rx));

    log!(Info, "Detecting baud rate...");

    // All possible baud rates as per UBX-13003221 page 36, ordered by likelihood
    //for baud in &[9600, 115_200, 4800, 19_200, 38_400, 57_600, 230_400, 460_800] {
    for baud in &[9600, 115_200] {
        log!(Debug, "Attempting baud {} ...", baud);

        // Get the peripheral and pins out of their cell.
        let uart = uart_cell.replace(None).unwrap();
        let tx = tx_cell.replace(None).unwrap();
        let rx = rx_cell.replace(None).unwrap();

        // Initialize UART port
        let config = hal::serial::config::Config::default()
            .baudrate(baud.bps())
            .parity_none();
        let mut uart: SERIAL = uart
            .serial((tx, rx), config, clocks)
            .unwrap();

        // If we can read a complete NMEA line, we're good. Unfortunately, the
        // default measurement rate is only 1Hz, so we need a rather large
        // timeout to be sure.
        if read_complete_nmea_line(&mut uart, 1200).is_ok() {
            log!(Info, "Detected baud rate: {}", baud);
            return Ok((*baud, uart));
        }

        // Release the port to get our peripheral and pins back, and put them
        // back in their Cell for the next attempt.
        let (uart, (tx, rx)) = uart.release();
        uart_cell.replace(Some(uart));
        tx_cell.replace(Some(tx));
        rx_cell.replace(Some(rx));
    }

    log!(Error, "Failed to establish connection to GPS RX.");

    Err(())
}

fn send_message(uart: &mut SERIAL, buf: &[u8]) -> Result<(), hal::serial::Error> {
    for b in buf {
        nb::block!(uart.write(*b))?;
    }
    nb::block!(uart.flush())?;
    Ok(())
}

impl GPS {
    pub fn init(dp_uart: USART2, tx: Pin<'A', 2,>, rx: Pin<'A', 3>, clocks: &Clocks) -> Result<Self, hal::serial::Error> {
        // Depending on the config stored onboard the SAM M8Q, the receiver may
        // use a variety of baud rates initially. We have to find out which one
        // it uses first.
        let (current_baud, mut uart) = detect_baud_rate(
            dp_uart,
            tx.into_alternate::<7>(),
            rx.into_alternate::<7>(),
            clocks
        ).map_err(|_e| hal::serial::Error::Other)?;

        // Now that we have established communication, we may want to change
        // the baud rate if it is different to our target
        if current_baud != BAUD_RATE {
            log!(Info, "Changing baud rate to {}...", BAUD_RATE);

            // Sending this NMEA message won't actually change the baud rate
            // for some reason, but it will allow us to send the UBX message,
            // which will.
            let payload = format!("PUBX,41,1,0007,0003,{},0", BAUD_RATE);
            let checksum = payload.chars().fold(0, |a, b| (a as u8) ^ (b as u8));
            let msg = format!("${}*{:02X}\r\n", payload, checksum);
            send_message(&mut uart, &msg.into_bytes())?;

            // Now that the RX is listening for UBX, set the baud rate.
            send_message(&mut uart, &CfgPrtUartBuilder {
                portid: UartPortId::Uart1,
                reserved0: 0,
                tx_ready: 0,
                mode: 0x8c0,
                baud_rate: BAUD_RATE,
                in_proto_mask: 0x07,
                out_proto_mask: 0x03,
                flags: 0,
                reserved5: 0,
            }.into_packet_bytes())?;
        }

        // Build final UART port with desired baud rate
        let (dp_uart, (tx, rx)) = uart.release();
        let final_config = hal::serial::config::Config::default()
            .baudrate(BAUD_RATE.bps())
            .parity_none();
        let mut uart: SERIAL = dp_uart.serial((tx, rx), final_config, clocks).unwrap();

        // Apply non-baud-related config items, such as measurement rate
        log!(Info, "Setting GPS measurement frequency...");
        send_message(&mut uart, &CfgRateBuilder {
            measure_rate_ms: MEASUREMENT_RATE_MS,
            nav_rate: 1,
            time_ref: AlignmentToReferenceTime::Gps
        }.into_packet_bytes())?;

        // TODO: enable galileo?

        // Prepare the UART for regular operation. The RX pin is read by the
        // USART2 interrupt, while we keep the TX pin in the struct for now.
        uart.listen(hal::serial::Event::Rxne);
        let (_uart_tx, uart_rx) = uart.split();

        // Move RX pin to static RefCell so interrupt can access it
        free(|cs| {
            *RX_PIN.borrow(cs).borrow_mut() = Some(uart_rx);
        });

        // Initialize NMEA buffer for interrupt
        unsafe { NMEA_BUFFER.replace(Vec::with_capacity(256)); }

        // Enable RX interrupt
        unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::USART2); }

        log!(Info, "Configuration complete.");

        Ok(Self {
            //tx: uart_tx,
            utc_time: None,
            latitude: None,
            longitude: None,
            altitude: None,
            fix: GPSFixType::NoFix,
            num_satellites: 0,
            hdop: 9999,
        })
    }

    pub fn tick(&mut self) {
        if !free(|cs| { NMEA_READY.borrow(cs).replace(false) }) {
            return;
        }

        // Grab the first complete NMEA line from the start of the buffer if
        // possible.
        let nmea: Option<String> = free(|_cs| {
            let buffer = unsafe { NMEA_BUFFER.as_mut().unwrap() };
            buffer.iter().position(|b| *b == '\n' as u8)
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
            self.latitude = (segments[2].len() > 0).then(|| segments[2].split_at(2))
                .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok())).flatten()
                .map(|(d, m)| (d + m / 60.0) * if segments[3] == "N" { 1.0 } else { -1.0 });
            self.longitude = (segments[4].len() > 0).then(|| segments[4].split_at(3))
                .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok())).flatten()
                .map(|(d, m)| (d + m / 60.0) * if segments[5] == "E" { 1.0 } else { -1.0 });
            self.altitude = segments[9].parse::<f32>().ok();

            self.fix = segments[6].try_into().unwrap_or(GPSFixType::NoFix);
            self.num_satellites = segments[7].parse().unwrap_or(0);
            self.hdop = (segments[8].parse::<f32>().unwrap_or(99.99) * 100.0) as u16;

            log_every_nth_time!(10, Debug, "{:?}, Lat/Lng: ({},{})°, Alt: {}m, Sats: {}, HDOP: {}",
                self.fix,
                self.latitude.map(|x| x.to_string()).unwrap_or("".to_string()),
                self.longitude.map(|x| x.to_string()).unwrap_or("".to_string()),
                self.altitude.map(|x| x.to_string()).unwrap_or("".to_string()),
                self.num_satellites,
                self.hdop as f32 / 100.0
            );
        }
    }
}

#[interrupt]
fn USART2() {
    cortex_m::peripheral::NVIC::unpend(Interrupt::USART2);

    free(|cs| {
        if let Some(ref mut rx) = RX_PIN.borrow(cs).borrow_mut().deref_mut() {
            let buffer = unsafe { NMEA_BUFFER.as_mut().unwrap() };

            while let Ok(b) = rx.read() {
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
