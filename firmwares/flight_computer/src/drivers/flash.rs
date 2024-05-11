use alloc::vec::Vec;

use embedded_hal_async::spi::SpiDevice;

use defmt::*;

use crate::flash::FlashError;

pub struct W25Q<SPI> {
    spi: SPI,
    size: u32,
}

impl<SPI: SpiDevice> W25Q<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, FlashError<SPI::Error>> {
        let mut w25 = Self {
            spi,
            size: 0,
        };

        let ids = w25.command(W25OpCode::JedecId, &[], 3).await?;
        let (man_id, dev_id) = (ids[0], u16::from_le_bytes([ids[2], ids[1]]));
        let (name, size_mbit) = match (man_id, dev_id) {
            (0xef, 0x4019) => ("W25Q256JV-IQ", 256),
            (0xef, 0x7019) => ("W25Q256JV-IM", 256),
            _ => ("unknown", 0),
        };

        if size_mbit > 0 {
            info!("{} initialized", name);
        } else {
            error!("Failed to initialize flash (0x{:02x}, 0x{:04x}).", man_id, dev_id);
        }

        w25.size = size_mbit * 1024 * 1024 / 8;

        Ok(w25)
    }

    async fn command(&mut self, opcode: W25OpCode, params: &[u8], response_len: usize) -> Result<Vec<u8>, FlashError<SPI::Error>> {
        let mut payload = [&[opcode as u8], params, &[0x00].repeat(response_len)].concat();

        self.spi.transfer_in_place(&mut payload).await?;

        Ok(payload[(1 + params.len())..].to_vec())
    }

    pub fn size(&self) -> u32 {
        self.size
    }

    // TODO: this should not have to be public
    pub async fn is_busy(&mut self) -> bool {
        let response = self.command(W25OpCode::ReadStatusRegister1, &[], 1).await;
        response.map(|resp| resp[0] & 0x01 > 0).unwrap_or(true)
    }

    pub async fn read(&mut self, address: u32, len: u32) -> Result<Vec<u8>, FlashError<SPI::Error>> {
        if self.is_busy().await {
            return Err(FlashError::Busy);
        }

        let address = address.to_be_bytes();
        self.command(W25OpCode::ReadData4BAddress, &address, len as usize).await
    }

    pub async fn write(&mut self, address: usize, data: &[u8]) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await {
            return Err(FlashError::Busy);
        }

        self.command(W25OpCode::WriteEnable, &[], 0).await?;
        let cmd = [&address.to_be_bytes(), data].concat();
        self.command(W25OpCode::PageProgram4BAddress, &cmd, 0).await?;
        Ok(())
    }

    pub async fn erase_sector(&mut self, address: u32) -> Result<(), FlashError<SPI::Error>> {
        self.command(W25OpCode::WriteEnable, &[], 0).await?;
        self.command(W25OpCode::SectorErase4KB4BAddress, &address.to_be_bytes(), 0).await?;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum W25OpCode {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,

    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,

    ReleasePowerDown = 0xab,
    ManufacturerDeviceId = 0x90,
    JedecId = 0x9f,
    ReadUniqueId = 0x48,

    ReadData = 0x03,
    ReadData4BAddress = 0x13,
    FastRead = 0x0b,
    FastRead4BAddress = 0x0c,

    PageProgram = 0x02,
    PageProgram4BAddress = 0x12,

    SectorErase4KB = 0x20,
    SectorErase4KB4BAddress = 0x21,
    BlockErase32KB = 0x52,
    BlockErase64KB = 0xd8,
    BlockErase64KB4BAddress = 0xdc,
    ChipErase = 0xc7,
}
