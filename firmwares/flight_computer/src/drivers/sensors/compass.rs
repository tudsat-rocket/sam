use embedded_hal_async::spi::SpiDevice;

use nalgebra::Vector3;

use defmt::*;

pub struct LIS3MDL<SPI: SpiDevice<u8>> {
    spi: SPI,
    scale: LIS3MDLFullScale,
    mag: Option<Vector3<f32>>,
    offset: Vector3<f32>,
}

impl<SPI: SpiDevice<u8>> LIS3MDL<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut lis3 = Self {
            spi,
            scale: LIS3MDLFullScale::Max16Gauss,
            mag: None,
            offset: Vector3::default(),
        };

        let whoami = lis3.read_u8(LIS3MDLRegister::WhoAmI).await?;

        // Enable temperature sensor and fast output data rate
        lis3.write_u8(LIS3MDLRegister::CtrlReg1, 0b1000_0010).await?;
        // Set full scale
        lis3.write_u8(LIS3MDLRegister::CtrlReg2, (lis3.scale as u8) << 5).await?;
        // Enable continuous-conversion mode
        lis3.write_u8(LIS3MDLRegister::CtrlReg3, 0b0000_0000).await?;

        if whoami != 0x3d {
            error!("Failed to initialize LIS3MDL (0x{:02x} != 0x3d)", whoami);
        } else {
            info!("LIS3MDL initialized");
        }

        Ok(lis3)
    }

    async fn read_u8(&mut self, address: LIS3MDLRegister) -> Result<u8, SPI::Error> {
        let mut buffer = [address as u8 | 0x80, 0];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(buffer[1])
    }

    async fn write_u8(&mut self, address: LIS3MDLRegister, value: u8) -> Result<(), SPI::Error> {
        let mut buffer = [address as u8, value];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(())
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let mut buffer: [u8; 9] = [0; 9];
        buffer[0] = (LIS3MDLRegister::OutXL as u8) | 0xc0;

        self.spi.transfer_in_place(&mut buffer).await?;

        let mag_x = i16::from_le_bytes([buffer[1], buffer[2]]);
        let mag_y = i16::from_le_bytes([buffer[3], buffer[4]]);
        let mag_z = i16::from_le_bytes([buffer[5], buffer[6]]);
        let _temp = i16::from_le_bytes([buffer[7], buffer[8]]);

        // convert to gauss using LSB/gauss values from datasheet, then to uT
        let lsb_to_gauss = match self.scale {
            LIS3MDLFullScale::Max4Gauss => 6842.0,
            LIS3MDLFullScale::Max8Gauss => 3421.0,
            LIS3MDLFullScale::Max12Gauss => 2281.0,
            LIS3MDLFullScale::Max16Gauss => 1711.0,
        };

        let mag_x = (mag_x as f32) / lsb_to_gauss * 100.0;
        let mag_y = (mag_y as f32) / lsb_to_gauss * 100.0;
        let mag_z = (mag_z as f32) / lsb_to_gauss * 100.0;

        self.mag = Some(Vector3::new(-mag_x, mag_z, mag_y));

        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(_e) = self.read_sensor_data().await {
            self.mag = None;
        }
    }

    pub fn set_offset(&mut self, offset: Vector3<f32>) {
        self.offset = offset;
    }

    pub fn magnetometer(&self) -> Option<Vector3<f32>> {
        self.mag.map(|m| m - self.offset)
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
    IntThsH = 0x33,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum LIS3MDLFullScale {
    Max4Gauss = 0b00,
    Max8Gauss = 0b01,
    Max12Gauss = 0b10,
    Max16Gauss = 0b11,
}
