use core::{cell::RefCell, cmp::Ordering};
use core::ops::DerefMut;

use alloc::{sync::Arc, collections::VecDeque, vec::Vec};

use embedded_hal_one::spi::blocking::SpiBus;
use embedded_hal_one::digital::blocking::OutputPin;

use cortex_m::interrupt::{free, Mutex};

use nalgebra::Vector3;

use crate::prelude::*;

const G_TO_MS2: f32 = 9.80665;
const OFFSET_ALPHA: f32 = 0.005;

pub struct H3LIS331DL<SPI, CS> {
    spi: Arc<Mutex<RefCell<SPI>>>,
    cs: CS,
    acc: Option<Vector3<f32>>,
    zero_offset_filter: VecDeque<Vector3<f32>>,
    zero_offset: Vector3<f32>
}

impl<SPI: SpiBus, CS: OutputPin> H3LIS331DL<SPI, CS> {
    pub fn init(spi: Arc<Mutex<RefCell<SPI>>>, cs: CS) -> Result<Self, SPI::Error> {
        let mut h3lis = Self {
            spi,
            cs,
            acc: None,
            zero_offset_filter: VecDeque::with_capacity(8),
            zero_offset: Vector3::default(),
        };

        let mut whoami = 0;
        for _i in 0..5 {
            whoami = h3lis.read_u8(H3LIS331DLRegister::WhoAmI)?;
        }

        // set normal mode, high ODR, all axes enabled
        h3lis.write_u8(H3LIS331DLRegister::CtrlReg1, 0b0011_1111)?;
        // set BDU, set +/- 200G scale
        h3lis.write_u8(H3LIS331DLRegister::CtrlReg4, 0b0001_0000)?;

        if whoami != 0x32 {
            log!(Error, "Failed to connect to H3LIS331DL (0x{:02x} != 0x32).", whoami);
        } else {
            log!(Info, "H3LIS331DL initialized.");
        }

        Ok(h3lis)
    }

    fn read_u8(&mut self, address: H3LIS331DLRegister) -> Result<u8, SPI::Error> {
        let mut buffer = [address as u8 | 0x80, 0];

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().unwrap();
            let res = spi.transfer_in_place(&mut buffer);
            self.cs.set_high().unwrap();
            res?;

            Ok(buffer[1])
        })
    }

    fn write_u8(&mut self, address: H3LIS331DLRegister, value: u8) -> Result<(), SPI::Error> {
        let mut buffer = [address as u8, value];

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().unwrap();
            let res = spi.transfer_in_place(&mut buffer);
            self.cs.set_high().unwrap();
            res?;

            Ok(())
        })
    }

    fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let mut buffer: [u8; 7] = [0; 7];
        buffer[0] = H3LIS331DLRegister::OutXL as u8 | 0xc0;

        free(|cs| {
            let mut ref_mut = self.spi.borrow(cs).borrow_mut();
            let spi = ref_mut.deref_mut();

            self.cs.set_low().unwrap();
            let res = spi.transfer_in_place(&mut buffer);
            self.cs.set_high().unwrap();
            res?;

            let acc_x = i16::from_le_bytes([buffer[1], buffer[2]]);
            let acc_y = i16::from_le_bytes([buffer[3], buffer[4]]);
            let acc_z = i16::from_le_bytes([buffer[5], buffer[6]]);

            let acc: Vector3<f32> = Vector3::new(acc_x.saturating_neg() as f32, acc_z as f32, acc_y as f32);
            self.acc = Some(acc * 200.0 / 32768.0 * G_TO_MS2);

            Ok(())
        })
    }

    pub fn tick(&mut self, time: u32, primary_acc: Option<Vector3<f32>>) {
        if let Err(e) = self.read_sensor_data() {
            self.acc = None;
            log!(Error, "Failed to read backup-up acc. data: {:?}", e);
        }

        // Since the zero offset of this accelerometer can be all over the place,
        // we use the values from the primary accelerometer to figure out the zero
        // offset during the first seconds after power-up.
        if time > 3000 {
            return;
        }

        if let (Some(primary), Some(this)) = (primary_acc, self.acc) {
            let diff = primary - this;

            self.zero_offset_filter.truncate(7);
            self.zero_offset_filter.push_front(diff);

            let mut sorted_x: Vec<f32> = self.zero_offset_filter.iter().map(|f| f.x).collect();
            let mut sorted_y: Vec<f32> = self.zero_offset_filter.iter().map(|f| f.y).collect();
            let mut sorted_z: Vec<f32> = self.zero_offset_filter.iter().map(|f| f.z).collect();

            sorted_x.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            sorted_y.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            sorted_z.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));

            let i = self.zero_offset_filter.len() / 2;
            let diff = Vector3::new(sorted_x[i], sorted_y[i], sorted_z[i]);

            self.zero_offset = (1.0 - OFFSET_ALPHA) * self.zero_offset + OFFSET_ALPHA * diff;
        }
    }

    pub fn accelerometer(&self) -> Option<Vector3<f32>> {
        self.acc.map(|acc| acc + self.zero_offset)
        //self.acc
    }
}

#[derive(Clone, PartialEq, Eq)]
#[allow(dead_code)]
enum H3LIS331DLRegister {
    WhoAmI = 0x0f,
    CtrlReg1 = 0x20,
    CtrlReg2 = 0x21,
    CtrlReg3 = 0x22,
    CtrlReg4 = 0x23,
    CtrlReg5 = 0x24,
    HpFilterReset = 0x25,
    Reference = 0x26,
    StatusReg = 0x27,
    OutXL = 0x28,
    OutXH = 0x29,
    OutYL = 0x2a,
    OutYH = 0x2b,
    OutZL = 0x2c,
    OutZH = 0x2d,
    Int1Cfg = 0x30,
    Int1Src = 0x31,
    Int1Ths = 0x32,
    Int1Duration = 0x33,
    Int2Cfg = 0x34,
    Int2Src = 0x35,
    Int2Ths = 0x36,
    Int2Duration = 0x37,
}
