//! Driver for the MCP2517FD CAN controller.
//!
//! Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2517FD-External-CAN-FD-Controller-with-SPI-Interface-20005688B.pdf

use alloc::sync::Arc;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embedded_hal_async::spi::SpiDevice;

use defmt::*;

use crate::can::CanDataRate;

pub struct MCP2517FD<SPI> {
    spi: SPI,
    fifo_index: u32,
    seq: u8,
    data_rate: CanDataRate,
}

pub struct MCP2517FDTx<SPI> {
    shared: Arc<Mutex<CriticalSectionRawMutex, MCP2517FD<SPI>>>,
}

pub struct MCP2517FDRx<SPI> {
    shared: Arc<Mutex<CriticalSectionRawMutex, MCP2517FD<SPI>>>,
}

// https://ww1.microchip.com/downloads/en/DeviceDoc/MCP25XXFD-CAN-FD-Controller-Module-Family-Reference-Manual-DS20005678E.pdf (4.3)
#[derive(Default, Debug)]
struct TransmitMessageObject {
    id: u32, // 11 bits for standard ID, 29 bits for extended
    seq: u8, // 7 bits used by MCP2517FD
    error_status_indicator: bool,
    fd_frame: bool,
    bit_rate_switch: bool,
    remote_transmission_request: bool,
    identifier_extension_flag: bool,
    msg: [u8; 8],
}

impl TransmitMessageObject {
    fn serialize(self) -> [u8; 16] {
        let dlc: u32 = 8;
        let t1: u32 = ((self.seq as u32) << 9) |
            ((self.error_status_indicator as u32) << 8) |
            ((self.fd_frame as u32) << 7) |
            ((self.bit_rate_switch as u32) << 6) |
            ((self.remote_transmission_request as u32) << 5) |
            ((self.identifier_extension_flag as u32) << 4) |
            (dlc & 0x0f);

        let mut frame = [0x00; 16];
        frame[..4].copy_from_slice(&self.id.to_le_bytes());
        frame[4..8].copy_from_slice(&t1.to_le_bytes());
        frame[8..].copy_from_slice(&self.msg);
        frame
    }
}

#[derive(Debug)]
pub enum MCP2517Error<SpiError> {
    Initialization,
    TransmitFiFoFull,
    Spi(SpiError),
}

impl<SpiError> From<SpiError> for MCP2517Error<SpiError> {
    fn from(e: SpiError) -> Self {
        MCP2517Error::Spi(e)
    }
}

impl<SPI: SpiDevice<u8>> MCP2517FDTx<SPI> {
    pub async fn transmit(&mut self, id: u16, msg: [u8; 8]) -> Result<(), MCP2517Error<SPI::Error>> {
        self.shared.lock().await.transmit(id, msg).await
    }
}

impl<SPI: SpiDevice<u8>> MCP2517FDRx<SPI> {
    pub async fn try_receive(&mut self) -> Result<Option<(u16, [u8; 8])>, SPI::Error> {
        self.shared.lock().await.try_receive().await
    }
}

impl<SPI: SpiDevice<u8>> MCP2517FD<SPI> {
    fn calculate_nominal_bit_timing_register(data_rate: CanDataRate) -> u32 {
        const SYSCLK: u32 = 20; // MHz

        let nbt = match data_rate {
            CanDataRate::Kbps125 => 8000,
            CanDataRate::Kbps250 => 4000,
            CanDataRate::Kbps500 => 2000,
            CanDataRate::Kbps1000 => 1000,
        }; // ns

        let nbrp: u32 = 1;
        let t_sysclk: u32 = 1_000 / SYSCLK; // ns
        let ntq = nbrp * t_sysclk;

        let nbt_over_ntq = nbt / ntq;
        let nsync = 1;
        let ntseg1 = (nbt_over_ntq as f32 * 0.8) as u32 - 1;
        let ntseg2 = nbt_over_ntq - nsync - ntseg1;
        let nsjw = ntseg2; // TODO?

        ((nbrp - 1) << 24) | ((ntseg1 - 1) << 16) | ((ntseg2 - 1) << 8) | (nsjw - 1)
    }

    pub async fn init(
        spi: SPI,
        data_rate: CanDataRate
    ) -> Result<(MCP2517FDTx<SPI>, MCP2517FDRx<SPI>), MCP2517Error<SPI::Error>> {
        let mut mcp = Self {
            spi,
            seq: 0,
            fifo_index: 0,
            data_rate
        };

        mcp.configure().await?;

        info!("MCP2517FD initalized");

        let shared = Arc::new(Mutex::new(mcp));

        let mcp_tx = MCP2517FDTx { shared: shared.clone() };
        let mcp_rx = MCP2517FDRx { shared };

        Ok((mcp_tx, mcp_rx))
    }

    async fn configure(&mut self) -> Result<(), MCP2517Error<SPI::Error>> {
        self.reset().await?;

        // Wait for MCP to transition to Configuration mode
        let mut c1con = 0;
        for _i in 0..10 {
            c1con = self.sfr_read(MCP2517FDRegister::C1Con).await?;

            if (c1con >> 21) & 0b111 == 0b100 {
                break
            }
        }

        if (c1con >> 21) & 0b111 != 0b100 {
            error!("Failed to initialize MCP2517FD");
            return Err(MCP2517Error::Initialization);
        }

        // Configure 125000 bitrate
        let nbtcfg = Self::calculate_nominal_bit_timing_register(self.data_rate);
        self.sfr_write(MCP2517FDRegister::C1NBtCfg, nbtcfg).await?;
        self.sfr_write(MCP2517FDRegister::C1DBtCfg, nbtcfg).await?;

        // Set mask 0 to receive everything
        self.sfr_write(MCP2517FDRegister::C1Mask0, 0x00000000).await?;
        // Set filter 0 to receive everything
        self.sfr_write(MCP2517FDRegister::C1FltObj0, 0x00000000).await?;
        // Enable filter 0 and link it to to FiFo 1
        self.sfr_write(MCP2517FDRegister::C1FltCon0, 0x00000081).await?;

        // Configure FiFo1 for receiving (8 byte payloads, 8 messages in queue)
        self.sfr_write(MCP2517FDRegister::C1FiFoCon1, 0x08000000).await?;
        // Configure FiFo2 for transmitting (8 byte payloads, 8 messages in queue,
        //  3 retransmission attempts)
        self.sfr_write(MCP2517FDRegister::C1FiFoCon2, 0x08200080).await?;

        c1con |= 0b1 << 16; // restrict retransmission attempts
        c1con &= !(0b111 << 24); // go back to normal mode
        self.sfr_write(MCP2517FDRegister::C1Con, c1con).await?;

        for _in in 0..3 {
            c1con = self.sfr_read(MCP2517FDRegister::C1Con).await?;
            if (c1con >> 21) & 0b111 == 0b000 {
                break
            }
        }

        Ok(())
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
        let address = ((MCP2517FDInstruction::Read as u16) << 12) | (address as u16);

        let mut buffer: [u8; 16+2] = [0; 16+2];
        buffer[0..2].copy_from_slice(&address.to_be_bytes());

        self.spi.transfer_in_place(&mut buffer).await?;

        // TODO: this is ugly
        let mut response = [0; 16];
        response.copy_from_slice(&buffer[2..]);

        Ok(response)
    }

    async fn memory_write(&mut self, address: u32, value: [u8; 16]) -> Result<(), SPI::Error> {
        let address = ((MCP2517FDInstruction::Write as u16) << 12) | (address as u16);

        let mut buffer: [u8; 16+2] = [0; 16+2];
        buffer[0..2].copy_from_slice(&address.to_be_bytes());
        buffer[2..].copy_from_slice(&value);

        self.spi.transfer_in_place(&mut buffer).await?;

        Ok(())
    }

    pub async fn try_receive(&mut self) -> Result<Option<(u16, [u8; 8])>, SPI::Error> {
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

    pub async fn transmit(&mut self, id: u16, msg: [u8; 8]) -> Result<(), MCP2517Error<SPI::Error>> {
        // Check Status register to find out if the FiFo is full
        let fifo_status = self.sfr_read(MCP2517FDRegister::C1FiFoSta2).await?;
        if (fifo_status & 0b1) == 0 {
            warn!("Full transmit FiFo, resetting...");
            let _ = self.configure().await;
            return Err(MCP2517Error::TransmitFiFoFull);
        }

        let address = 0x400 + self.sfr_read(MCP2517FDRegister::C1FiFoUA2).await?;

        self.seq = (self.seq + 1) % 0x7f;
        let tmq = TransmitMessageObject {
            id: id.into(),
            msg,
            seq: self.seq,
            ..Default::default()
        };

        self.memory_write(address, tmq.serialize()).await?;

        let mut fifo_config = self.sfr_read(MCP2517FDRegister::C1FiFoCon2).await?;
        fifo_config |= 0b1 << 8; // increase FiFo counter
        fifo_config |= 0b1 << 9; // request transmission
        self.sfr_write(MCP2517FDRegister::C1FiFoCon2, fifo_config).await?;

        Ok(())
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
    C1FiFoCon2 = 0x068,
    C1FiFoSta2 = 0x06c,
    C1FiFoUA2 = 0x070,
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
