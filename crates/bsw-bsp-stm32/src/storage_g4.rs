//! STM32G474 storage backend over the raw flash driver (package D09).
//!
//! Adapts the reserved 8 KiB NVM region (flash pages 252..=255, see
//! [`crate::flash_g4`]) to the geometry-independent
//! [`bsw_storage::backend::StorageBackend`] contract, so the journaled
//! block store from `bsw-storage` runs unchanged on this board. The
//! application-specific block layout that `NvmManager` hard-codes is not
//! used here: block identities and atomicity live entirely in the generic
//! journal layer.
//!
//! Geometry: `region_size = 8192`, `erase_unit = 2048` (one flash page),
//! `program_unit = 8` (double-word programming).

use bsw_storage::backend::{StorageBackend, StorageError};

use crate::flash_g4::{
    FlashG4, FLASH_PAGE_SIZE, NVM_BASE_ADDR, NVM_PAGE_COUNT, NVM_PAGE_START, NVM_SIZE,
};

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

/// Zero-sized backend over the G4 NVM flash region.
///
/// Construction is `unsafe` because exactly one live instance may drive
/// the flash controller: concurrent flash operations (including from
/// interrupts) violate the controller's sequencing rules.
pub struct G4NvmBackend {
    _not_send: core::marker::PhantomData<*mut ()>,
}

impl G4NvmBackend {
    /// Create the backend.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that no other code erases or programs
    /// flash while this instance exists, and that the NVM region is
    /// reserved for storage in the linker layout.
    #[must_use]
    pub const unsafe fn new() -> Self {
        Self {
            _not_send: core::marker::PhantomData,
        }
    }

    /// Check that the option bytes select the 256 x 2 KiB dual-bank geometry.
    #[must_use]
    pub fn dual_bank_geometry_enabled() -> bool {
        const FLASH_OPTR: *const u32 = 0x4002_2020 as *const u32;
        const DBANK: u32 = 1 << 22;
        // SAFETY: OPTR is a read-only flash-controller register.
        unsafe { crate::mmio::read(FLASH_OPTR) & DBANK != 0 }
    }

    fn check_range(offset: usize, len: usize) -> Result<(), StorageError> {
        let end = offset.checked_add(len).ok_or(StorageError::OutOfRange)?;
        if end > NVM_SIZE as usize {
            return Err(StorageError::OutOfRange);
        }
        Ok(())
    }
}

impl StorageBackend for G4NvmBackend {
    fn region_size(&self) -> usize {
        NVM_SIZE as usize
    }

    fn erase_unit(&self) -> usize {
        FLASH_PAGE_SIZE as usize
    }

    fn program_unit(&self) -> usize {
        8
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        Self::check_range(offset, buf.len())?;
        #[allow(clippy::cast_possible_truncation)]
        FlashG4::read(NVM_BASE_ADDR + offset as u32, buf);
        Ok(())
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        if unit_index >= usize::from(NVM_PAGE_COUNT) {
            return Err(StorageError::OutOfRange);
        }
        #[allow(clippy::cast_possible_truncation)]
        let page = NVM_PAGE_START + unit_index as u16;
        // SAFETY: `new()` establishes exclusive flash ownership; the page
        // index is inside the reserved NVM region, and the unlock/erase/
        // lock sequence follows RM0440.
        let ok = flash_critical(|| {
            // SAFETY: backend ownership, checked region, and interrupt mask
            // make this flash-controller sequence exclusive.
            unsafe {
                FlashG4::unlock();
                let ok = FlashG4::erase_page(page);
                FlashG4::lock();
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
        if !offset.is_multiple_of(8) || !data.len().is_multiple_of(8) {
            return Err(StorageError::Unaligned);
        }
        Self::check_range(offset, data.len())?;
        let mut old = [0u8; 8];
        for (chunk_index, chunk) in data.chunks_exact(8).enumerate() {
            FlashG4::read(NVM_BASE_ADDR + (offset + chunk_index * 8) as u32, &mut old);
            if old
                .iter()
                .zip(chunk)
                .any(|(&before, &after)| before & after != after)
            {
                return Err(StorageError::NotErased);
            }
        }
        #[allow(clippy::cast_possible_truncation)]
        let address = NVM_BASE_ADDR + offset as u32;
        // SAFETY: `new()` establishes exclusive flash ownership; the range
        // is inside the reserved NVM region and 8-byte aligned, and the
        // unlock/program/lock sequence follows RM0440.
        let ok = flash_critical(|| {
            // SAFETY: backend ownership, checked region, and interrupt mask
            // make this flash-controller sequence exclusive.
            unsafe {
                FlashG4::unlock();
                let ok = FlashG4::program(address, data);
                FlashG4::lock();
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
