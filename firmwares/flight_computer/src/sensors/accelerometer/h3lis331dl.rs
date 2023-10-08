use embedded_hal_async::spi::{SpiBus, SpiDevice};

use nalgebra::Vector3;

use defmt::*;

const G_TO_MS2: f32 = 9.80665;

pub struct H3LIS331DL<SPI: SpiDevice<u8>> {
    spi: SPI,
    acc: Option<Vector3<f32>>,
    offset: Vector3<f32>
}

impl<SPI: SpiDevice<u8>> H3LIS331DL<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut h3lis = Self {
            spi,
            acc: None,
            offset: Vector3::default()
        };

        let mut whoami = 0;
        for _i in 0..5 {
            whoami = h3lis.read_u8(H3LIS331DLRegister::WhoAmI).await?;
        }

        // set normal mode, high ODR, all axes enabled
        h3lis.write_u8(H3LIS331DLRegister::CtrlReg1, 0b0011_1111).await?;
        // set BDU, set +/- 200G scale
        h3lis.write_u8(H3LIS331DLRegister::CtrlReg4, 0b0001_0000).await?;

        if whoami != 0x32 {
            error!("Failed to connect to H3LIS331DL (0x{:02x} != 0x32).", whoami);
        } else {
            info!("H3LIS331DL initialized.");
        }

        Ok(h3lis)
    }

    async fn read_u8(&mut self, address: H3LIS331DLRegister) -> Result<u8, SPI::Error> {
        let mut buffer = [address as u8 | 0x80, 0];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(buffer[1])
    }

    async fn write_u8(&mut self, address: H3LIS331DLRegister, value: u8) -> Result<(), SPI::Error> {
        let mut buffer = [address as u8, value];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(())
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let mut buffer: [u8; 7] = [0; 7];
        buffer[0] = H3LIS331DLRegister::OutXL as u8 | 0xc0;

        self.spi.transfer_in_place(&mut buffer).await?;

        let acc_x = i16::from_le_bytes([buffer[1], buffer[2]]);
        let acc_y = i16::from_le_bytes([buffer[3], buffer[4]]);
        let acc_z = i16::from_le_bytes([buffer[5], buffer[6]]);

        let acc: Vector3<f32> = Vector3::new(acc_x.saturating_neg() as f32, acc_z as f32, acc_y as f32);
        self.acc = Some(acc * 200.0 / 32768.0 * G_TO_MS2);

        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(e) = self.read_sensor_data().await {
            self.acc = None;
            error!("Failed to read backup-up acc. data: {:?}", Debug2Format(&e));
        }
    }

    pub fn set_offset(&mut self, offset: Vector3<f32>) {
        self.offset = offset;
    }

    pub fn accelerometer(&self) -> Option<Vector3<f32>> {
        self.acc.map(|acc| acc - self.offset)
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
