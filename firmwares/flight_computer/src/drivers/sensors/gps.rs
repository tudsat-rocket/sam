use alloc::format;
use alloc::string::{String, ToString};

use alloc::vec::Vec;
use embassy_embedded_hal::SetConfig;
use embassy_stm32::peripherals::*;
use embassy_stm32::usart::{Uart, Error};
use embassy_stm32::bind_interrupts;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Sender, Receiver, Channel};
use embassy_time::{Timer, Duration, Instant};

use ublox::{AlignmentToReferenceTime, CfgPrtUartBuilder, CfgRateBuilder, UartPortId};
use static_cell::make_static;

use defmt::*;

use crate::telemetry::*;

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});

const DESIRED_BAUD_RATE: u32 = 115_200;
const BAUD_RATE_OPTIONS: [u32; 2] = [115_200, 9600];
//const BAUD_RATE_OPTIONS: [u32; 8] = [115_200, 9600, 460800, 230400, 57600, 38400, 19200, 4800];
const MEASUREMENT_RATE_MS: u16 = 100;

#[allow(dead_code)]
struct GPSDatum {
    utc_time: Option<u64>,
    latitude: Option<f32>,
    longitude: Option<f32>,
    altitude: Option<f32>,
    fix: GPSFixType,
    hdop: u16,
    num_satellites: u8,
}

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

pub struct GPS {
    uart: Uart<'static, USART2, DMA1_CH6, DMA1_CH5>,
    sender: Sender<'static, CriticalSectionRawMutex, GPSDatum, 5>,
}

pub struct GPSHandle {
    receiver: Receiver<'static, CriticalSectionRawMutex, GPSDatum, 5>,
    last_datum: Option<(GPSDatum, Instant)>,
}

#[embassy_executor::task]
pub async fn run(mut gps: GPS) -> ! {
    loop {
        if let Err(e) = gps.run().await {
            error!("{:?}", Debug2Format(&e));
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

impl GPS {
    // TODO: dma channels
    pub fn init(p: USART2, tx: PA3, rx: PA2, tx_dma: DMA1_CH6, rx_dma: DMA1_CH5) -> (GPS, GPSHandle) {
        let channel = make_static!(Channel::<CriticalSectionRawMutex, GPSDatum, 5>::new());

        let mut uart_config = embassy_stm32::usart::Config::default();
        uart_config.baudrate = BAUD_RATE_OPTIONS[0];

        let uart = Uart::new(p, tx, rx, Irqs, tx_dma, rx_dma, uart_config).unwrap();

        let gps = GPS {
            uart,
            sender: channel.sender(),
        };

        let handle = GPSHandle {
            receiver: channel.receiver(),
            last_datum: None
        };

        (gps, handle)
    }

    async fn read_gps_packet(&mut self) -> Result<String, Error> {
        let mut buffer = [0; 1024];
        let n = self.uart.read_until_idle(&mut buffer).await?;
        Ok(String::from_utf8_lossy(&buffer[..n]).to_string())
    }

    async fn find_baud_rate(&mut self) -> u32 {
        loop {
            for baud_rate in &BAUD_RATE_OPTIONS {
                let start = Instant::now();
                info!("Trying baud rate {:?} for GPS...", baud_rate);

                let mut uart_config = embassy_stm32::usart::Config::default();
                uart_config.baudrate = *baud_rate;
                self.uart.set_config(&uart_config).unwrap();

                while start.elapsed() < Duration::from_millis(2500) {
                    if let Ok(_str) = self.read_gps_packet().await {
                        return *baud_rate;
                    }
                    Timer::after(Duration::from_millis(10)).await;
                }
            }

            warn!("Failed to find GPS baud rate, retrying.");
        }
    }

    async fn change_gps_baud_rate(&mut self, baud_rate: u32) -> Result<(), Error> {
        info!("Changing baud rate to {:?}", DESIRED_BAUD_RATE);

        // Sending this NMEA message won't actually change the baud rate
        // for some reason, but it will allow us to send the UBX message,
        // which will.
        let payload = format!("PUBX,41,1,0007,0003,{},0", baud_rate);
        let checksum = payload.chars().fold(0, |a, b| (a as u8) ^ (b as u8));
        let msg = format!("${}*{:02X}\r\n", payload, checksum);
        self.uart.write(&msg.into_bytes()).await?;

        // Now that the RX is listening for UBX, set the baud rate.
        self.uart.write(
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
        ).await?;

        let mut uart_config = embassy_stm32::usart::Config::default();
        uart_config.baudrate = baud_rate;
        self.uart.set_config(&uart_config).unwrap();

        Ok(())
    }

    async fn process_nmea_line(&mut self, line: &str) {
        // We only care about GGA messages, those contain coordinates/fix info
        if line.get(3..=5).unwrap_or("XXX") != "GGA" {
            return;
        }

        let segments: Vec<&str> = line.split(',').collect();
        if segments.len() < 15 {
            return;
        }

        //let utc_time = Some(segments[1].to_string());

        // Latitude needs to converted from degrees and minutes to decimal degrees
        // Lat: DDMM.MM... Lng: DDDMM.MM...
        let latitude = (segments[2].len() > 2)
            .then(|| segments[2].split_at(2))
            .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok()))
            .flatten()
            .map(|(d, m)| (d + m / 60.0) * if segments[3] == "N" { 1.0 } else { -1.0 });
        let longitude = (segments[4].len() > 3)
            .then(|| segments[4].split_at(3))
            .map(|(d, m)| d.parse::<f32>().ok().zip(m.parse::<f32>().ok()))
            .flatten()
            .map(|(d, m)| (d + m / 60.0) * if segments[5] == "E" { 1.0 } else { -1.0 });
        let altitude = segments[9].parse::<f32>().ok();

        let fix = segments[6].try_into().unwrap_or(GPSFixType::NoFix);
        let num_satellites = segments[7].parse().unwrap_or(0);
        let hdop = (segments[8].parse::<f32>().unwrap_or(99.99) * 100.0) as u16;

        let datum: GPSDatum = GPSDatum {
            utc_time: None,
            latitude,
            longitude,
            altitude,
            fix,
            hdop,
            num_satellites
        };

        self.sender.send(datum).await;
    }

    async fn run(&mut self) -> Result<(), Error> {
        let current_baud_rate = self.find_baud_rate().await;
        info!("GPS using baud rate {:?}.", current_baud_rate);

        if current_baud_rate != DESIRED_BAUD_RATE {
            self.change_gps_baud_rate(DESIRED_BAUD_RATE).await?;
        }

        info!("Setting GPS measurement frequency...");
        self.uart.write(
            &CfgRateBuilder {
                measure_rate_ms: MEASUREMENT_RATE_MS,
                nav_rate: 1,
                time_ref: AlignmentToReferenceTime::Gps,
            }
            .into_packet_bytes(),
        ).await?;

        loop {
            if let Ok(str) = self.read_gps_packet().await {
                for line in str.split("\r\n").filter(|str| str.len() > 0) {
                    self.process_nmea_line(line).await;
                }
            } else {
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

impl GPSHandle {
    fn check_for_new_values(&mut self) {
        while let Ok(datum) = self.receiver.try_receive() {
            self.last_datum = Some((datum, Instant::now()));
        }

        // we discard our last value after 200ms to avoid reporting stale values
        let value_expired = self.last_datum
            .as_ref()
            .map(|(_d, t)| t.elapsed() > Duration::from_millis(200))
            .unwrap_or(false);
        if value_expired {
            self.last_datum = None;
        }
    }

    pub fn latitude(&mut self) -> Option<f32> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.latitude).flatten()
    }

    pub fn longitude(&mut self) -> Option<f32> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.longitude).flatten()
    }

    pub fn altitude(&mut self) -> Option<f32> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.altitude).flatten()
    }

    pub fn fix(&mut self) -> Option<GPSFixType> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.fix)
    }

    pub fn hdop(&mut self) -> Option<u16> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.hdop)
    }

    pub fn num_satellites(&mut self) -> Option<u8> {
        self.check_for_new_values();
        self.last_datum.as_ref().map(|(d, _)| d.num_satellites)
    }
}
