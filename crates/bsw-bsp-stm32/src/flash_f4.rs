//! STM32F413 embedded flash driver and storage backend (package D10).
//!
//! The F413ZH provides 1.5 MiB of flash in 16 sectors (RM0430): sectors
//! 0..=3 are 16 KiB, sector 4 is 64 KiB, sectors 5..=15 are 128 KiB. The
//! two topmost sectors are reserved for persistent storage so the
//! application image can grow contiguously from the bottom of flash:
//!
//! - sector 14: `0x0814_0000..0x0816_0000`
//! - sector 15: `0x0816_0000..0x0818_0000`
//!
//! # Linker ownership
//!
//! The application linker script must cap the code region at
//! [`STORAGE_BASE_ADDR`] (`FLASH LENGTH = 1280K`), which keeps the storage
//! sectors out of reach of the programmer and of `cargo objcopy` images.
//!
//! Programming uses 32-bit words (`PSIZE = x32`, valid for 2.7-3.6 V
//! supplies), so the [`StorageBackend`] program unit is 4 bytes.

use bsw_storage::backend::{StorageBackend, StorageError};

// ---------------------------------------------------------------------------
// Register map (RM0430, FLASH interface)
// ---------------------------------------------------------------------------

const FLASH_KEYR: *mut u32 = 0x4002_3C04 as *mut u32;
const FLASH_SR: *mut u32 = 0x4002_3C0C as *mut u32;
const FLASH_CR: *mut u32 = 0x4002_3C10 as *mut u32;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

const CR_PG: u32 = 1 << 0;
const CR_SER: u32 = 1 << 1;
const CR_SNB_SHIFT: u32 = 3;
const CR_PSIZE_X32: u32 = 0b10 << 8;
const CR_PSIZE_MASK: u32 = 0b11 << 8;
const CR_STRT: u32 = 1 << 16;
const CR_LOCK: u32 = 1 << 31;

const SR_BSY: u32 = 1 << 16;
/// OPERR | WRPERR | PGAERR | PGPERR | PGSERR | RDERR
const SR_ERROR_MASK: u32 = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8);
const SR_EOP: u32 = 1 << 0;
const MAX_READY_POLLS: u32 = 20_000_000;

// ---------------------------------------------------------------------------
// Storage region geometry
// ---------------------------------------------------------------------------

/// First storage sector number.
pub const STORAGE_FIRST_SECTOR: u32 = 14;

/// Number of reserved storage sectors.
pub const STORAGE_SECTOR_COUNT: u32 = 2;

/// Size of one storage sector in bytes (128 KiB).
pub const STORAGE_SECTOR_SIZE: u32 = 128 * 1024;

/// Base address of the reserved storage region (start of sector 14).
pub const STORAGE_BASE_ADDR: u32 = 0x0814_0000;

/// Total reserved storage bytes.
pub const STORAGE_SIZE: u32 = STORAGE_SECTOR_COUNT * STORAGE_SECTOR_SIZE;

#[inline(always)]
unsafe fn read_reg(reg: *const u32) -> u32 {
    // SAFETY: caller guarantees the pointer is a valid MMIO address.
    unsafe { crate::mmio::read(reg) }
}

#[inline(always)]
unsafe fn write_reg(reg: *mut u32, val: u32) {
    // SAFETY: caller guarantees the pointer is a valid MMIO address.
    unsafe { crate::mmio::write(reg, val) }
}

/// Zero-sized driver for the STM32F413 embedded flash controller.
pub struct FlashF4;

impl FlashF4 {
    /// Unlock flash for programming and erase operations.
    ///
    /// # Safety
    ///
    /// Must not race any other flash operation, including from interrupts.
    pub unsafe fn unlock() -> bool {
        // SAFETY: FLASH_CR/KEYR are the documented MMIO addresses.
        unsafe {
            if read_reg(FLASH_CR.cast_const()) & CR_LOCK == 0 {
                return false;
            }
            write_reg(FLASH_KEYR, KEY1);
            write_reg(FLASH_KEYR, KEY2);
            read_reg(FLASH_CR.cast_const()) & CR_LOCK == 0
        }
    }

    /// Re-lock the flash controller.
    ///
    /// # Safety
    ///
    /// Call only when no flash operation is in progress.
    pub unsafe fn lock() {
        // SAFETY: FLASH_CR is the documented MMIO address.
        unsafe {
            let cr = read_reg(FLASH_CR.cast_const());
            write_reg(FLASH_CR, cr | CR_LOCK);
        }
    }

    /// Whether an erase/program operation is in progress.
    #[inline]
    pub fn is_busy() -> bool {
        // SAFETY: FLASH_SR is a valid read-only status address.
        unsafe { read_reg(FLASH_SR.cast_const()) & SR_BSY != 0 }
    }

    /// Spin until idle; returns `false` when an error flag is set.
    ///
    /// # Safety
    ///
    /// Call only after starting a flash operation.
    pub unsafe fn wait_ready() -> bool {
        // SAFETY: polling the status register is side-effect free.
        unsafe {
            let mut polls = MAX_READY_POLLS;
            while Self::is_busy() && polls != 0 {
                core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
                polls -= 1;
            }
            if polls == 0 {
                return false;
            }
            read_reg(FLASH_SR.cast_const()) & SR_ERROR_MASK == 0
        }
    }

    /// Clear error and end-of-operation flags (write-1-to-clear).
    ///
    /// # Safety
    ///
    /// Call only while the controller is idle.
    pub unsafe fn clear_errors() {
        // SAFETY: writing W1C bits to FLASH_SR.
        unsafe {
            write_reg(FLASH_SR, SR_ERROR_MASK | SR_EOP);
        }
    }

    /// Erase one flash sector by number (RM0430 sector erase sequence).
    ///
    /// # Safety
    ///
    /// Flash must be unlocked; the CPU must not execute from the sector;
    /// `sector` must identify an existing sector.
    pub unsafe fn erase_sector(sector: u32) -> bool {
        if sector > 15 {
            return false;
        }
        // SAFETY: documented erase sequence over valid MMIO addresses.
        unsafe {
            if !Self::wait_ready() {
                Self::clear_errors();
            }
            let cr = read_reg(FLASH_CR.cast_const()) & !(0xF << CR_SNB_SHIFT) & !CR_PSIZE_MASK;
            write_reg(
                FLASH_CR,
                cr | CR_SER | (sector << CR_SNB_SHIFT) | CR_PSIZE_X32,
            );
            let cr = read_reg(FLASH_CR.cast_const());
            write_reg(FLASH_CR, cr | CR_STRT);
            let ok = Self::wait_ready();
            let cr = read_reg(FLASH_CR.cast_const());
            write_reg(FLASH_CR, cr & !CR_SER);
            ok
        }
    }

    /// Program one 32-bit word.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked, `addr` word-aligned inside programmable
    /// flash, and the target word previously erased (or the new value only
    /// clears bits).
    pub unsafe fn program_word(addr: u32, value: u32) -> bool {
        if !addr.is_multiple_of(4) {
            return false;
        }
        // SAFETY: documented programming sequence; the caller guarantees
        // the destination address contract.
        unsafe {
            if !Self::wait_ready() {
                Self::clear_errors();
            }
            let cr = read_reg(FLASH_CR.cast_const()) & !CR_PSIZE_MASK;
            write_reg(FLASH_CR, cr | CR_PG | CR_PSIZE_X32);
            crate::mmio::write(addr as *mut u32, value);
            let ok = Self::wait_ready();
            let cr = read_reg(FLASH_CR.cast_const());
            write_reg(FLASH_CR, cr & !CR_PG);
            ok
        }
    }

    /// Program a word-aligned byte slice.
    ///
    /// # Safety
    ///
    /// Same contract as [`FlashF4::program_word`] for the whole range.
    pub unsafe fn program(addr: u32, data: &[u8]) -> bool {
        if !addr.is_multiple_of(4) || !data.len().is_multiple_of(4) {
            return false;
        }
        for (index, chunk) in data.chunks_exact(4).enumerate() {
            let value = u32::from_le_bytes(chunk.try_into().expect("4-byte chunk"));
            if value == u32::MAX {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let target = addr + (index as u32) * 4;
            // SAFETY: forwarded caller contract.
            if !unsafe { Self::program_word(target, value) } {
                return false;
            }
        }
        true
    }

    /// Read bytes from memory-mapped flash.
    pub fn read(addr: u32, buf: &mut [u8]) {
        for (index, slot) in buf.iter_mut().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let source = (addr + index as u32) as *const u8;
            // SAFETY: memory-mapped flash is always readable; the storage
            // backend validates the range before calling.
            *slot = unsafe { crate::mmio::read(source) };
        }
    }
}

/// Storage backend over the reserved F413 sectors.
///
/// Geometry: `region_size = 256 KiB`, `erase_unit = 128 KiB` (one sector),
/// `program_unit = 4` (word programming).
pub struct F4StorageBackend {
    _not_send: core::marker::PhantomData<*mut ()>,
}

fn flash_critical<R>(operation: impl FnOnce() -> R) -> R {
    #[cfg(target_arch = "arm")]
    {
        cortex_m::interrupt::free(|_| operation())
    }
    #[cfg(not(target_arch = "arm"))]
    {
        operation()
    }
}

impl F4StorageBackend {
    /// Create the backend.
    ///
    /// # Safety
    ///
    /// The caller must guarantee exclusive flash-controller ownership while
    /// this instance exists and a linker layout that keeps code out of
    /// sectors 14 and 15.
    #[must_use]
    pub const unsafe fn new() -> Self {
        Self {
            _not_send: core::marker::PhantomData,
        }
    }

    fn check_range(offset: usize, len: usize) -> Result<(), StorageError> {
        let end = offset.checked_add(len).ok_or(StorageError::OutOfRange)?;
        if end > STORAGE_SIZE as usize {
            return Err(StorageError::OutOfRange);
        }
        Ok(())
    }
}

impl StorageBackend for F4StorageBackend {
    fn region_size(&self) -> usize {
        STORAGE_SIZE as usize
    }

    fn erase_unit(&self) -> usize {
        STORAGE_SECTOR_SIZE as usize
    }

    fn program_unit(&self) -> usize {
        4
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        Self::check_range(offset, buf.len())?;
        #[allow(clippy::cast_possible_truncation)]
        FlashF4::read(STORAGE_BASE_ADDR + offset as u32, buf);
        Ok(())
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        if unit_index >= STORAGE_SECTOR_COUNT as usize {
            return Err(StorageError::OutOfRange);
        }
        #[allow(clippy::cast_possible_truncation)]
        let sector = STORAGE_FIRST_SECTOR + unit_index as u32;
        // SAFETY: `new()` establishes exclusive flash ownership; the sector
        // is inside the reserved storage region.
        let ok = flash_critical(|| {
            // SAFETY: backend ownership, checked sector, and interrupt mask
            // make this flash-controller sequence exclusive.
            unsafe {
                FlashF4::unlock();
                let ok = FlashF4::erase_sector(sector);
                FlashF4::lock();
                ok
            }
        });
        if ok {
            Ok(())
        } else {
            Err(StorageError::Io)
        }
    }

    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        if !offset.is_multiple_of(4) || !data.len().is_multiple_of(4) {
            return Err(StorageError::Unaligned);
        }
        Self::check_range(offset, data.len())?;
        let mut old = [0u8; 4];
        for (chunk_index, chunk) in data.chunks_exact(4).enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let address = STORAGE_BASE_ADDR + (offset + chunk_index * 4) as u32;
            FlashF4::read(address, &mut old);
            if old
                .iter()
                .zip(chunk)
                .any(|(&before, &after)| before & after != after)
            {
                return Err(StorageError::NotErased);
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let address = STORAGE_BASE_ADDR + offset as u32;
        // SAFETY: `new()` establishes exclusive flash ownership; the range
        // is inside the reserved storage region and word-aligned.
        let ok = flash_critical(|| {
            // SAFETY: backend ownership, checked range, and interrupt mask
            // make this flash-controller sequence exclusive.
            unsafe {
                FlashF4::unlock();
                let ok = FlashF4::program(address, data);
                FlashF4::lock();
                ok
            }
        });
        if ok {
            Ok(())
        } else {
            Err(StorageError::Io)
        }
    }
}
