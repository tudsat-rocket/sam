/// Uncomplete driver for the Winbond W25Q128JV Spi Flash.
/// Data sheet: https://www.pjrc.com/teensy/W25Q128FV.pdf
use bitflags::{Flags, bitflags};
use core::{fmt::Debug, future::Future};
use defmt::*;
use embedded_hal_async::spi::{Error as SpiErrorTrait, ErrorKind, ErrorType};
use embedded_hal_async::spi::{Operation, SpiDevice};
use embedded_storage_async::nor_flash::{self, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash};

const _4KB: u32 = 2_u32.pow(12);
const _32KB: u32 = 2_u32.pow(15);
const _64KB: u32 = 2_u32.pow(16);

pub struct W25Q128<SPI> {
    spi: SPI,
}

use crate::storage::Flash;

#[derive(Debug)]
pub enum FlashError<E> {
    Spi(E),
    Init,
    Busy,
    OutOfBounds,
    NotAligned,
    MultipleCommandsRequired,
}

impl<E: core::fmt::Debug> NorFlashError for FlashError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Spi(_) => NorFlashErrorKind::Other,
            Self::Init => NorFlashErrorKind::Other,
            Self::Busy => NorFlashErrorKind::Other,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::MultipleCommandsRequired => NorFlashErrorKind::Other,
        }
    }
}

/// Helper function that produces a 4 byte pattern containing a 1 byte opcode and a 3 byte
/// address in big endian byte order.
/// | opcode | A23-A16 | A15-A8 | A7-A0 |
fn make_op_3b_slice(opcode: OpCode, address: u32) -> [u8; 4] {
    let byte23_16 = ((address >> 16) & 0xFF) as u8;
    let byte15_8 = ((address >> 8) & 0xFF) as u8;
    let byte7_0 = (address & 0xFF) as u8;
    [opcode.into(), byte23_16, byte15_8, byte7_0]
}

fn aligned_4kb(value: u32) -> bool {
    value % _4KB == 0
}
fn aligned_32kb(value: u32) -> bool {
    value % _32KB == 0
}
fn aligned_64kb(value: u32) -> bool {
    value % _64KB == 0
}

impl<SPI: SpiDevice> W25Q128<SPI> {
    pub fn capacity(&self) -> usize {
        (128 / 8) * 1_000_000
    }

    pub async fn new(mut spi: SPI) -> Result<Self, FlashError<SPI::Error>> {
        defmt::info!("started flash new");
        // TODO: check
        // Do we need to check if busy?
        let mut ids: [u8; 3] = [0; 3];
        let res = spi
            .transaction(&mut [Operation::Write(&[OpCode::JedecId.into()]), Operation::Read(&mut ids)])
            .await
            .map_err(FlashError::Spi);
        match res {
            Ok(_) => defmt::info!("flash ok"),
            Err(ref e) => {
                match e {
                    FlashError::Spi(e) => match e.kind() {
                        ErrorKind::Other => defmt::info!("spi error other"),
                        ErrorKind::Overrun => defmt::info!("spi error overrun"),
                        ErrorKind::ModeFault => defmt::info!("spi error mode fault"),
                        ErrorKind::FrameFormat => defmt::info!("spi error FrameFormat"),
                        ErrorKind::ChipSelectFault => defmt::info!("spi error ChipSelectFault"),
                        _ => defmt::info!("spi error non_exhaustive"),
                    },
                    _ => defmt::info!("non spi error"),
                };
            }
        }
        res?;
        // spi.transfer(&mut padded_ids, &[OpCode::JedecId.into()]).await.map_err(FlashError::Spi)?;
        let (man_id, dev_id) = (ids[0], u16::from_le_bytes([ids[2], ids[1]]));
        let (name, size_mbit) = match (man_id, dev_id) {
            (0xef, 0x4019) => ("W25Q256JV-IQ", 256),
            (0xef, 0x7019) => ("W25Q256JV-IM", 256),
            (0x17, 0x7018) => ("W25Q128JV", 128),
            (0xef, 0x4018) => ("OurFlash", 128),
            _ => ("unknown", 0),
        };

        if size_mbit != 128 {
            defmt::error!("Failed to initialize flash (0x{:02x}, 0x{:04x}).", man_id, dev_id);
            return Err(FlashError::Init);
        }
        defmt::info!("{} initialized", name);
        Ok(Self { spi })
    }
    pub async fn erase_sector(&mut self, offset: u32) -> Result<(), FlashError<SPI::Error>> {
        if offset % 4096 != 0 {
            return Err(FlashError::NotAligned);
        }
        self.erase_sector_4kb_unchecked(offset).await
    }

    pub async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError<SPI::Error>> {
        self.single_action_write(offset, bytes).await
    }
    pub async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError<SPI::Error>> {
        // TODO: test out of bounds check
        if (offset as usize) + bytes.len() >= self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        }
        // TODO: fix
        // let padded_bytes: &[u8] =
        self.spi
            .transaction(&mut [
                Operation::Write(&make_op_3b_slice(OpCode::ReadData, offset)),
                Operation::Read(bytes),
            ])
            .await
            .map_err(FlashError::Spi)?;
        // self.spi.transfer(bytes, &[OpCode::ReadData.into()]).await.map_err(FlashError::Spi)?;
        Ok(())
    }

    async fn erase_flexible(&mut self, from: u32, to: u32) -> Result<(), FlashError<SPI::Error>> {
        // TODO: allow not 4kb aligned erases
        if from > to || to as usize > self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        if from == 0 && to as usize == self.capacity() {
            self.chip_erase().await?;
            return Ok(());
        }
        if from == to {
            return Ok(());
        }
        if !aligned_4kb(from) || !aligned_4kb(to) {
            return Err(FlashError::NotAligned);
        }
        let mut from = from;
        loop {
            from = self.erase_next(from, to).await?;
            if from == to {
                break;
            }
        }

        // let mut from = from;
        // while !(aligned_32kb(from) && (from + _32KB >= to)) {
        //     self.erase_sector_4kb_unchecked(from).await?;
        //     from += _4KB;
        // }
        // if !(aligned_64kb(from) && (from + _64KB >= to)) && aligned_32kb(from) && (from + _32KB <= to) {
        //     self.erase_block_32kb_unchecked(from).await?;
        //     from += _32KB;
        // }
        // while aligned_64kb(from) && (from + _64KB <= to) {
        //     self.erase_block_64kb_unchecked(from).await?;
        //     from += _64KB;
        // }
        // while aligned_32kb(from) && (from + _32KB <= to) {
        //     self.erase_block_32kb_unchecked(from).await?;
        //     from += _32KB;
        // }
        // while from + _4KB <= to {
        //     self.erase_sector_4kb_unchecked(from).await?;
        //     from += _4KB;
        // }
        Ok(())
    }
    // returns index of not erased section
    async fn erase_next(&mut self, index: u32, max: u32) -> Result<u32, FlashError<SPI::Error>> {
        // 64kb
        let size: u32 = max - index;

        let max_fits: u32 = match size {
            0.._4KB => 0,
            _4KB.._32KB => _4KB,
            _32KB.._64KB => _32KB,
            _ => _64KB,
        };

        let max_alignment = if aligned_64kb(index) {
            _64KB
        } else if aligned_32kb(index) {
            _32KB
        } else if aligned_4kb(index) {
            _4KB
        } else {
            // unreachable if index is 4KB aligned
            0
        };

        let erase_size = max_fits.min(max_alignment);

        match erase_size {
            _4KB => self.erase_sector_4kb_unchecked(index).await?,
            _32KB => self.erase_block_32kb_unchecked(index).await?,
            _64KB => self.erase_block_32kb_unchecked(index).await?,
            0 => (),
            // unreachable
            _ => (),
        }
        Ok(index + erase_size)
    }

    // private methods

    /// Performs a single erase action to erase the bytes [from..to].
    /// Return an error if the range cannot be erased by a single action or is out of bounds.
    /// Possible ranges are:
    /// Chip Erase, BlockErase (64KB), BlockErase(32KB), SectorErase(4KB).
    async fn single_action_erase(&mut self, from: u32, to: u32) -> Result<(), FlashError<SPI::Error>> {
        if from > to || to as usize > self.capacity() {
            return Err(FlashError::OutOfBounds);
        }
        if from == 0 && to as usize == self.capacity() {
            self.chip_erase().await?;
            return Ok(());
        }
        let range = to - from;
        if range == 0 {
            return Ok(());
        }
        if range % (4 * 1024) != 0 || from % (4 * 1024) != 0 {
            return Err(FlashError::NotAligned);
        }
        // 64kB
        if (from % (64 * 1024) == 0) && (range % (64 * 1024) == 0) {
            self.erase_block_64kb_unchecked(from).await?;
        }
        // 32kB
        else if (from % (32 * 1024) == 0) && (range % (32 * 1024) == 0) {
            self.erase_block_32kb_unchecked(from).await?;
        }
        // 4kB
        else if (from % (4 * 1024) == 0) && range % (4 * 1024) == 0 {
            self.erase_sector_4kb_unchecked(from).await?;
        } else {
            return Err(FlashError::MultipleCommandsRequired);
        }
        Ok(())
    }

    /// Writes 0 to 256 bytes at the given offset.
    ///
    /// If the input is longer than 256 bytes, multiple flash operations would be required and
    /// [`FlashError::MultipleCommandsRequired`] is returned.
    ///
    /// After the bytes are transfered, the flash needs an additional processing time:
    /// First byte: app. 30 µs
    /// Every additional byte: app. 2.5 µs
    async fn single_action_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError<SPI::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }
        if bytes.len() > 256 {
            info!("multiple commands required");
            return Err(FlashError::MultipleCommandsRequired);
        }
        if self.is_busy().await? {
            info!("flash busy");
            return Err(FlashError::Busy);
        }
        self.spi.write(&[OpCode::WriteEnable.into()]).await.map_err(FlashError::Spi)?;
        self.spi
            .transaction(&mut [
                Operation::Write(&make_op_3b_slice(OpCode::PageProgram, offset)),
                Operation::Write(bytes),
            ])
            .await
            .map_err(FlashError::Spi)?;
        Ok(())
    }

    // Wrappers for Flash Instructions:

    /// Returns true if the flash chip is busy.
    /// Reads the ERASE_WRITE_IN_PROGRESS status register.
    pub async fn is_busy(&mut self) -> Result<bool, FlashError<SPI::Error>> {
        let mut reg1: &mut [u8] = &mut [0; 2];
        self.spi
            .transaction(&mut [
                Operation::Write(&[OpCode::ReadStatusRegister1.into()]),
                Operation::Read(reg1),
            ])
            .await
            .map_err(FlashError::Spi)?;
        let reg1 = StatusReg1::from_bits_truncate(reg1[1]);
        Ok(reg1.contains(StatusReg1::ERASE_WRITE_IN_PROGRESS))
    }
    /// Performs a sector erase 4KB without checking for address correctness.
    async fn erase_sector_4kb_unchecked(&mut self, address: u32) -> Result<(), FlashError<SPI::Error>> {
        self.spi.write(&[OpCode::WriteEnable.into()]).await.map_err(FlashError::Spi)?;
        self.spi.write(&make_op_3b_slice(OpCode::SectorErase4KB, address)).await.map_err(FlashError::Spi)
    }
    /// Performs a block erase 32KB without checking for address correctness.
    async fn erase_block_32kb_unchecked(&mut self, address: u32) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        self.spi.write(&[OpCode::WriteEnable.into()]).await.map_err(FlashError::Spi)?;
        self.spi.write(&make_op_3b_slice(OpCode::BlockErase32KB, address)).await.map_err(FlashError::Spi)?;
        Ok(())
    }
    /// Performs a block erase 64KB without checking for address correctness.
    async fn erase_block_64kb_unchecked(&mut self, address: u32) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        self.spi.write(&[OpCode::WriteEnable.into()]).await.map_err(FlashError::Spi)?;
        self.spi.write(&make_op_3b_slice(OpCode::BlockErase64KB, address)).await.map_err(FlashError::Spi)?;
        Ok(())
    }
    /// Performs a chip erase.
    async fn chip_erase(&mut self) -> Result<(), FlashError<SPI::Error>> {
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        };
        self.spi.write(&[OpCode::WriteEnable.into()]).await.map_err(FlashError::Spi)?;
        self.spi.write(&[OpCode::ChipErase.into()]).await.map_err(FlashError::Spi)?;
        Ok(())
    }
}

impl<SPI: SpiDevice> nor_flash::ErrorType for W25Q128<SPI> {
    type Error = FlashError<SPI::Error>;
}

impl<SPI: SpiDevice> ReadNorFlash for W25Q128<SPI> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        // TODO: test out of bounds check
        if self.capacity() >= (offset as usize) + bytes.len() {
            return Err(FlashError::OutOfBounds);
        }
        if self.is_busy().await? {
            return Err(FlashError::Busy);
        }
        self.spi.transfer(bytes, &[OpCode::ReadData.into()]).await.map_err(FlashError::Spi)?;
        Ok(())
    }

    fn capacity(&self) -> usize {
        (128 / 8) * 1_000_000
    }
}

impl<SPI: SpiDevice> NorFlash for W25Q128<SPI> {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = 4096;
    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.single_action_erase(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.single_action_write(offset, bytes).await
    }
}

bitflags! {
    struct StatusReg1: u8 {
        const STATUS_REGISTER_PROTECT = 1 << 7;
        const SECTOR_PROTECT = 1 << 6;
        const TOP_BOTTOM_PROTECT = 1 << 5;
        const BLOCK_PROTECT2 = 1 << 4;
        const BLOCK_PROTECT1 = 1 << 3;
        const BLOCK_PROTECT0 = 1 << 2;
        const WRITE_ENABLE_LATCH = 1 << 1;
        const ERASE_WRITE_IN_PROGRESS = 1 << 0;
    }
}

#[repr(u8)]
enum OpCode {
    WriteEnable = 0x06,
    VolatileSrWriteEnable = 0x50,
    WriteDisable = 0x04,

    ReleasePowerDown = 0xab,
    ManufacturerDeviceId = 0x90,
    JedecId = 0x9f,
    ReadUniqueId = 0x4b,

    ReadData = 0x03,
    FastRead = 0x0b,

    PageProgram = 0x02,

    SectorErase4KB = 0x20,
    BlockErase32KB = 0x52,
    BlockErase64KB = 0xd8,
    ChipErase = 0xc7,

    // Reread data sheet; some variations might be missing.
    ReadStatusRegister1 = 0x05,
    ReadStatusRegister2 = 0x35,
    ReadStatusRegister3 = 0x15,

    ReadSfdpRegister = 0x5a,
    EraseSecurityRegister = 0x44,
    ProgramSecurityRegister = 0x42,
    ReadSecurityRegister = 0x48,

    GlobalBlockLock = 0x7e,
    GlobalBlockUnlock = 0x98,
    ReadBlockLock = 0x3d,
    IndividualBlockLock = 0x36,
    IndividualBlockUnlock = 0x39,

    EraseProgramSuspend = 0x75,
    EraseProgramResume = 0x7a,
    PowerDown = 0xb9,

    EnterQpiMode = 0x38,
    EnableReset = 0x66,
    ResetDevice = 0x99,
}

impl From<OpCode> for u8 {
    fn from(opcode: OpCode) -> u8 {
        opcode as u8
    }
}
