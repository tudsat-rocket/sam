// https://invensense.tdk.com/wp-content/uploads/2021/07/DS-000451-ICM-42670-P-v1.0.pdf

use embassy_time::{Timer, Duration};
use embedded_hal_async::spi::SpiDevice;

use nalgebra::Vector3;

use defmt::*;

const G_TO_MS2: f32 = 9.80665;

pub struct ICM42670P<SPI: SpiDevice<u8>> {
    spi: SPI,
    gyro: Option<Vector3<f32>>,
    accel: Option<Vector3<f32>>,
}

impl<SPI: SpiDevice<u8>> ICM42670P<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut imu = Self {
            spi,
            gyro: None,
            accel: None,
        };

        let mut whoami = 0x00;
        for _i in 0..10 {
            whoami = imu.read_u8(ICM42670PRegister::WhoAmI).await?;
            if whoami == 0x67 {
                break
            }

            Timer::after(Duration::from_micros(100)).await;
        }

        if whoami != 0x67 {
            error!("Failed to initialize ICM-42670-P (0x{:02x} != 0x67)", whoami);
        } else {
            info!("ICM-42670-P initialized");
        }

        imu.configure().await?;

        Ok(imu)
    }

    async fn read_u8(&mut self, address: ICM42670PRegister) -> Result<u8, SPI::Error> {
        let mut payload = [address as u8 | 0x80, 0]; // TODO
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(payload[1])
    }

    async fn write_u8(&mut self, address: ICM42670PRegister, value: u8) -> Result<(), SPI::Error> {
        let mut payload = [address as u8, value];
        self.spi.transfer_in_place(&mut payload).await?;
        Ok(())
    }

    async fn configure(&mut self) -> Result<(), SPI::Error> {
        self.write_u8(ICM42670PRegister::GyroConfig0, 0b00000101).await?;
        self.write_u8(ICM42670PRegister::AccelConfig0, 0b00000101).await?;
        self.write_u8(ICM42670PRegister::PwrMgmt0, 0b00001111).await?;
        Ok(())
    }

    pub fn accelerometer(&self) -> Option<Vector3<f32>> {
        self.accel
    }

    pub fn gyroscope(&self) -> Option<Vector3<f32>> {
        self.gyro
    }

    async fn read_sensor_data(&mut self) -> Result<(), SPI::Error> {
        let mut payload = [(ICM42670PRegister::TempData1 as u8) | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        self.spi.transfer_in_place(&mut payload).await?;

        // TODO: do we need this temp?
        let _temp = ((payload[1] as i16) << 8) + (payload[2] as i16);
        let accel_x = ((payload[3] as i16) << 8) + (payload[4] as i16);
        let accel_y = ((payload[5] as i16) << 8) + (payload[6] as i16);
        let accel_z = ((payload[7] as i16) << 8) + (payload[8] as i16);
        let gyro_x = ((payload[9] as i16) << 8) + (payload[10] as i16);
        let gyro_y = ((payload[11] as i16) << 8) + (payload[12] as i16);
        let gyro_z = ((payload[13] as i16) << 8) + (payload[14] as i16);

        self.gyro = Some(Vector3::new(gyro_y as f32, gyro_z as f32, gyro_x as f32) * 2000.0 / (i16::MAX as f32));
        self.accel = Some(Vector3::new(accel_y as f32, accel_z as f32, accel_x as f32) * 16.0 * G_TO_MS2 / (i16::MAX as f32));

        Ok(())
    }

    pub async fn tick(&mut self) {
        if let Err(_e) = self.read_sensor_data().await {
            self.gyro = None;
            self.accel = None;
        }
    }
}

#[derive(Clone, PartialEq, Eq)]
enum ICM42670PRegister {
    TempData1 = 0x09,
    PwrMgmt0 = 0x1f,
    GyroConfig0 = 0x20,
    AccelConfig0 = 0x21,
    WhoAmI = 0x75,
}
