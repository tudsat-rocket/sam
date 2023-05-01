use core::cell::RefCell;
use core::ops::DerefMut;

use alloc::sync::Arc;

use embedded_hal_one::spi::blocking::TransferInplace;
use embedded_hal_one::digital::blocking::OutputPin;

use cortex_m::interrupt::{free, Mutex};

use nalgebra::Vector3;

use crate::prelude::*;

pub struct LIS3MDL<SPI, CS> {
    spi: Arc<Mutex<RefCell<SPI>>>,
    cs: CS,
    scale: LIS3MDLFullScale,
    mag: Option<Vector3<f32>>
}

impl<SPI: TransferInplace, CS: OutputPin> LIS3MDL<SPI, CS> {
    pub fn init(
        spi: Arc<Mutex<RefCell<SPI>>>,
        cs: CS
    ) -> Result<Self, SPI::Error> {
        let mut lis3 = Self {
            spi,
            cs,
            scale: LIS3MDLFullScale::Max16Gauss,
            mag: None,
        };

        let whoami = lis3.read_u8(LIS3MDLRegister::WhoAmI)?;

        // Enable temperature sensor and fast output data rate
        lis3.write_u8(LIS3MDLRegister::CtrlReg1, 0b1000_0010)?;
        // Set full scale
        lis3.write_u8(LIS3MDLRegister::CtrlReg2, (lis3.scale as u8) << 5)?;
        // Enable continuous-conversion mode
        lis3.write_u8(LIS3MDLRegister::CtrlReg3, 0b0000_0000)?;

        if whoami != 0x3d {
            log!(Error, "Failed to connect to LIS3MDL (0x{:02x} != 0x3d).", whoami);
        } else {
            log!(Info, "LIS3MDL initialized.");
        }

        Ok(lis3)
    }

    fn read_u8(&mut self, address: LIS3MDLRegister) -> Result<u8, SPI::Error> {
        let mut buffer = [address as u8 | 0x80, 0];

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().ok();
            let res = spi.transfer_inplace(&mut buffer);
            self.cs.set_high().ok();
            res?;

            Ok(buffer[1])
        })
    }

    fn write_u8(&mut self, address: LIS3MDLRegister, value: u8) -> Result<(), SPI::Error> {
        let mut buffer = [address as u8, value];

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().ok();
            let res = spi.transfer_inplace(&mut buffer);
            self.cs.set_high().ok();
            res?;

            Ok(())
        })
    }

    fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let mut buffer: [u8; 9] = [0; 9];
        buffer[0] = (LIS3MDLRegister::OutXL as u8) | 0xc0;

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().ok();
            let res = spi.transfer_inplace(&mut buffer);
            self.cs.set_high().ok();
            res
        })?;

        let mag_x = i16::from_le_bytes([buffer[1], buffer[2]]);
        let mag_y = i16::from_le_bytes([buffer[3], buffer[4]]);
        let mag_z = i16::from_le_bytes([buffer[5], buffer[6]]);
        let _temp = i16::from_le_bytes([buffer[7], buffer[8]]);

        //debug!("{:#04x}, {:#04x}, {:#04x}", mag_x, mag_y, mag_z);
        //debug!("{}, {}, {}", mag_x, mag_y, mag_z);

        //let mag_x = mag_x.saturating_sub(i16::MIN / 2);
        let mag_z = mag_z.saturating_sub(i16::MIN / 2);

        // convert to gauss using LSB/gauss values from datasheet, then to uT
        let lsb_to_gauss = match self.scale {
            LIS3MDLFullScale::Max4Gauss => 6842.0,
            LIS3MDLFullScale::Max8Gauss => 3421.0,
            LIS3MDLFullScale::Max12Gauss => 2281.0,
            LIS3MDLFullScale::Max16Gauss => 1711.0,
        };

        // TODO: configurable calibration values
        let mag_x = (mag_x as f32) / lsb_to_gauss * 100.0 + 522.0;
        let mag_y = (mag_y as f32) / lsb_to_gauss * 100.0 - 30.0;
        let mag_z = (mag_z as f32) / lsb_to_gauss * 100.0 - 760.0;

        self.mag = Some(Vector3::new(-mag_x, mag_z, mag_y));

        Ok(())
    }

    pub fn tick(&mut self) {
        if let Err(_e) = self.read_sensor_data() {
            self.mag = None;
            log!(Error, "Failed to read compass data");
        }
    }

    pub fn magnetometer(&self) -> Option<Vector3<f32>> {
        self.mag
    }
}

#[derive(Clone, PartialEq, Eq)]
#[allow(dead_code)]
enum LIS3MDLRegister {
    WhoAmI = 0x0f,
    CtrlReg1 = 0x20,
    CtrlReg2 = 0x21,
    CtrlReg3 = 0x22,
    CtrlReg4 = 0x23,
    CtrlReg5 = 0x24,
    StatusReg = 0x27,
    OutXL = 0x28,
    OutXH = 0x29,
    OutYL = 0x2a,
    OutYH = 0x2b,
    OutZL = 0x2c,
    OutZH = 0x2d,
    TempOutL = 0x2e,
    TempOutH = 0x2f,
    IntCfg = 0x30,
    IntSrc = 0x31,
    IntThsL = 0x32,
    IntThsH = 0x33
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LIS3MDLFullScale {
    Max4Gauss = 0b00,
    Max8Gauss = 0b01,
    Max12Gauss = 0b10,
    Max16Gauss = 0b11,
}
