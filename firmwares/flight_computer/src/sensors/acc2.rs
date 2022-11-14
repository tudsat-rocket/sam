use alloc::vec::Vec;

use hal::gpio::{Alternate, Output, Pin};
use hal::pac::SPI3;
use hal::prelude::*;
use hal::spi::{Master, TransferModeNormal};
use stm32f4xx_hal as hal;

use crate::prelude::*;

type Spi = hal::spi::Spi<
    SPI3,
    (
        Pin<'C', 10, Alternate<6>>,
        Pin<'C', 11, Alternate<6>>,
        Pin<'C', 12, Alternate<6>>,
    ),
    TransferModeNormal,
    Master,
>;
type CsPin = hal::gpio::Pin<'D', 2, Output>;

const G_TO_MS2: f32 = 9.80665;

pub struct Accelerometer {
    spi: Spi,
    cs: CsPin,
    acc: Option<(f32, f32, f32)>,
}

impl Accelerometer {
    pub fn init(spi: Spi, cs: CsPin) -> Result<Self, hal::spi::Error> {
        let mut acc2 = Self { spi, cs, acc: None };

        acc2.configure_power(ADXL375Mode::Measure)?;
        acc2.write_u8(ADXL375Register::DataFormat, 0b00001011)?;

        let device_id = acc2.read_u8(ADXL375Register::DeviceId)?;
        log!(Info, "ADXL375 Device ID: 0x{:02x?}", device_id);

        acc2.configure_data_rate(ADXL375DataRate::OSR1600Hz, false)?;

        Ok(acc2)
    }

    fn read_registers(&mut self, address: ADXL375Register, response_len: usize) -> Result<Vec<u8>, hal::spi::Error> {
        let address = (address as u8) | 0x80 | (((response_len > 1) as u8) << 6);
        let mut payload = [alloc::vec![address], [0x00].repeat(response_len)].concat();

        self.cs.set_low();
        let response = self.spi.transfer(&mut payload);
        self.cs.set_high();

        Ok(response?[1..].to_vec())
    }

    fn read_u8(&mut self, address: ADXL375Register) -> Result<u8, hal::spi::Error> {
        let res = self.read_registers(address, 1)?;
        Ok(res[0])
    }

    fn write_u8(&mut self, address: ADXL375Register, value: u8) -> Result<(), hal::spi::Error> {
        let mut payload = [address as u8, value];

        self.cs.set_low();
        let res = self.spi.transfer(&mut payload);
        self.cs.set_high();

        res?;

        Ok(())
    }

    fn read_sensor_data(&mut self) -> Result<(), hal::spi::Error> {
        let response = self.read_registers(ADXL375Register::DataXL, 6)?;

        let x = ((response[1] as i16) << 8) + (response[0] as i16);
        let y = ((response[3] as i16) << 8) + (response[2] as i16);
        let z = ((response[5] as i16) << 8) + (response[4] as i16);

        self.acc = Some((
            x as f32 * 0.049 * G_TO_MS2,
            z as f32 * 0.049 * G_TO_MS2,
            -y as f32 * 0.049 * G_TO_MS2,
        ));

        Ok(())
    }

    fn configure_power(&mut self, mode: ADXL375Mode) -> Result<(), hal::spi::Error> {
        let val = (mode as u8) << 2;
        self.write_u8(ADXL375Register::PowerControl, val)
    }

    fn configure_data_rate(&mut self, data_rate: ADXL375DataRate, low_power: bool) -> Result<(), hal::spi::Error> {
        let val = data_rate as u8 + ((low_power as u8) << 4);
        self.write_u8(ADXL375Register::DataRateControl, val)
    }

    pub fn tick(&mut self) {
        if let Err(e) = self.read_sensor_data() {
            self.acc = None;
            log!(Error, "{:?}", e);
        }
    }

    pub fn accelerometer(&self) -> Option<(f32, f32, f32)> {
        self.acc
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum ADXL375Register {
    DeviceId = 0x00,
    ShockThreshold = 0x1d,
    OffsetX = 0x1e,
    OffsetY = 0x1f,
    OffsetZ = 0x20,
    ShockDuration = 0x21,
    ShockLatency = 0x22,
    ShockWindow = 0x23,
    ActivityThreshold = 0x24,
    InactivityThreshold = 0x25,
    InactivityTime = 0x26,
    ActivityAxisControl = 0x27,
    ShockAxisControl = 0x2a,
    ShockSource = 0x2b,
    DataRateControl = 0x2c,
    PowerControl = 0x2d,
    InterruptEnable = 0x2e,
    InterruptMapping = 0x2f,
    InterruptSource = 0x30,
    DataFormat = 0x31,
    DataXL = 0x32,
    DataXH = 0x33,
    DataYL = 0x34,
    DataYH = 0x35,
    DataZL = 0x36,
    DataZH = 0x37,
    FifoControl = 0x38,
    FifoStatus = 0x39,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum ADXL375DataRate {
    OSR0Hz10 = 0b0000,
    OSR0Hz20 = 0b0001,
    OSR0Hz39 = 0b0010,
    OSR0Hz78 = 0b0011,
    OSR1Hz56 = 0b0100,
    OSR3Hz13 = 0b0101,
    OSR6Hz25 = 0b0110,
    OSR12Hz5 = 0b0111,
    OSR25Hz = 0b1000,
    OSR50Hz = 0b1001,
    OSR100Hz = 0b1010,
    OSR200Hz = 0b1011,
    OSR400Hz = 0b1100,
    OSR800Hz = 0b1101,
    OSR1600Hz = 0b1110,
    OSR3200Hz = 0b1111,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum ADXL375Mode {
    Measure = 0b10,
    Sleep = 0b01,
}
