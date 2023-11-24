//! Driver for the MCP2517FD CAN controller.
//!
//! Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2517FD-External-CAN-FD-Controller-with-SPI-Interface-20005688B.pdf

use embedded_hal_async::spi::SpiDevice;

use defmt::*;

pub struct MCP2517FD<SPI> {
    spi: SPI,
    fifo_index: u32,
}

impl<SPI: SpiDevice<u8>> MCP2517FD<SPI> {
    pub async fn init(spi: SPI) -> Result<Self, SPI::Error> {
        let mut mcp = Self {
            spi,
            fifo_index: 0,
        };

        mcp.reset().await?;

        // Wait for MCP to transition to Configuration mode
        let mut c1con = 0;
        for _i in 0..10 {
            c1con = mcp.sfr_read(MCP2517FDRegister::C1Con).await?;

            if (c1con >> 21) & 0b111 == 0b100 {
                break
            }
        }

        if (c1con >> 21) & 0b111 != 0b100 {
            error!("Failed to transition CAN controller to configuration mode.");
            // TODO: return error
        }

        // Configure 125000 bitrate
        mcp.sfr_write(MCP2517FDRegister::C1NBtCfg, 0x007e1f1f).await?;
        mcp.sfr_write(MCP2517FDRegister::C1DBtCfg, 0x001e0707).await?;
        mcp.sfr_write(MCP2517FDRegister::C1TDC,    0x00001f00).await?;

        // Set mask 0 to receive everything
        mcp.sfr_write(MCP2517FDRegister::C1Mask0, 0x00000000).await?;
        // Set filter 0 to receive everything
        mcp.sfr_write(MCP2517FDRegister::C1FltObj0, 0x00000000).await?;
        // Enable filter 0 and link it to to FiFo 1
        mcp.sfr_write(MCP2517FDRegister::C1FltCon0, 0x00000081).await?;

        // Configure FiFo1 (8 byte payloads, 8 messages in queue)
        mcp.sfr_write(MCP2517FDRegister::C1FiFoCon1, 0x08000000).await?;

        c1con &= !(0b111 << 24); // go back to normal mode
        mcp.sfr_write(MCP2517FDRegister::C1Con, c1con).await?;

        for _in in 0..3 {
            c1con = mcp.sfr_read(MCP2517FDRegister::C1Con).await?;
            if (c1con >> 21) & 0b111 == 0b000 {
                break
            }
        }

        info!("MCP2517FD initalized.");

        Ok(mcp)
    }

    async fn reset(&mut self) -> Result<(), SPI::Error> {
        let mut buffer: [u8; 2] = [0; 2];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(())
    }

    async fn sfr_read(&mut self, register: MCP2517FDRegister) -> Result<u32, SPI::Error> {
        let address = (0b0011 << 12) | (register as u16);
        let mut buffer: [u8; 6] = [0; 6];
        buffer[0] = (address >> 8) as u8;
        buffer[1] = (address & 0xff) as u8;

        self.spi.transfer_in_place(&mut buffer).await?;

        Ok(u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]))
    }

    async fn sfr_write(&mut self, register: MCP2517FDRegister, value: u32) -> Result<(), SPI::Error> {
        let address = (0b0010 << 12) | (register as u16);
        let address = address.to_be_bytes();
        let value = value.to_le_bytes();
        let mut buffer: [u8; 6] = [address[0], address[1], value[0], value[1], value[2], value[3]];

        self.spi.transfer_in_place(&mut buffer).await?;

        Ok(())
    }

    async fn memory_read(&mut self, address: u32) -> Result<[u8; 16], SPI::Error> {
        let address = (0b0011 << 12) | (address as u16);

        let mut buffer: [u8; 16+2] = [0; 16+2];
        buffer[0..2].copy_from_slice(&address.to_be_bytes());

        self.spi.transfer_in_place(&mut buffer).await?;

        // TODO: this is ugly
        let mut response = [0; 16];
        response.copy_from_slice(&buffer[2..]);

        Ok(response)
    }

    pub async fn receive(&mut self) -> Result<Option<(u16, [u8; 8])>, SPI::Error> {
        // Check Status register to find out whether there is a new message
        let fifo_status = self.sfr_read(MCP2517FDRegister::C1FiFoSta1).await?;
        let fifo_index = (fifo_status >> 8) & 0x1f;

        if fifo_index != self.fifo_index {
            // Find out where the message is stored in memory
            let address = 0x400 + self.sfr_read(MCP2517FDRegister::C1FiFoUA1).await?;

            // Read our 8 message bytes plus 8 bytes header
            let frame: [u8; 16] = self.memory_read(address).await?;
            let identifier = u16::from_le_bytes([frame[0], frame[1]]);
            let mut msg = [0; 8];
            msg.copy_from_slice(&frame[8..]);

            // Tell CAN controller to increment FIFO index
            let mut fifo_config = self.sfr_read(MCP2517FDRegister::C1FiFoCon1).await?;
            fifo_config |= 0b1 << 8;
            self.sfr_write(MCP2517FDRegister::C1FiFoCon1, fifo_config).await?;

            self.fifo_index = fifo_index;

            Ok(Some((identifier, msg)))
        } else {
            Ok(None)
        }
    }
}


#[allow(dead_code)]
enum MCP2517FDInstruction {
    Reset = 0b0000,
    Read = 0b0011,
    Write = 0b0010,
    ReadCrc = 0b1011,
    WriteCrc = 0b1010,
    WriteSafe = 0b1100,
}

#[allow(dead_code)]
enum MCP2517FDRegister {
    C1Con = 0x000,
    C1NBtCfg = 0x004,
    C1DBtCfg = 0x008,
    C1TDC= 0x00C,
    C1FiFoCon1 = 0x05c,
    C1FiFoSta1 = 0x060,
    C1FiFoUA1 = 0x064,
    C1FltCon0 = 0x1d0,
    C1FltCon1 = 0x1d4,
    C1FltCon2 = 0x1d8,
    C1FltCon3 = 0x1dc,
    C1FltCon4 = 0x1e0,
    C1FltCon5 = 0x1e4,
    C1FltCon6 = 0x1e8,
    C1FltCon7 = 0x1ec,
    C1FltObj0 = 0x1f0,
    C1Mask0 = 0x1f4,
    Osc = 0xe00,
    IoCon = 0xe04,
    Crc = 0xe08,
    EccCon = 0xe0c,
    EccStat = 0xe10,
}
