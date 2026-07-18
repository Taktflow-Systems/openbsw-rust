//! In-memory storage backend that faithfully models flash semantics.
//!
//! [`MemBackend`] is the host-test stand-in for a real flash driver: erase
//! sets whole units to `0xFF`, program only clears bits at the configured
//! program granularity, and unaligned or out-of-range accesses are
//! rejected. Per-unit erase counters are exposed for wear tests.

use crate::backend::{
    check_program_range, check_read_range, geometry_is_valid, StorageBackend, StorageError,
};

/// Maximum number of erase units a [`MemBackend`] instantiation may have.
///
/// The counter array is fixed at this capacity because stable Rust cannot
/// yet size an array by `SIZE / ERASE` of the const generics.
pub const MAX_ERASE_UNITS: usize = 128;

/// In-memory flash model.
///
/// - `SIZE`: total region size in bytes.
/// - `ERASE`: bytes per erase unit (power of two dividing `SIZE`).
/// - `PROG`: program granularity (1, 4, or 8, dividing `ERASE`).
///
/// Invalid geometry is rejected at compile time (a post-monomorphization
/// error), so a constructed backend always has a valid geometry.
///
/// The whole region lives inline in the struct; instances of large `SIZE`
/// should be placed in a `static` on embedded targets.
pub struct MemBackend<const SIZE: usize, const ERASE: usize, const PROG: usize> {
    data: [u8; SIZE],
    erase_counts: [u32; MAX_ERASE_UNITS],
}

impl<const SIZE: usize, const ERASE: usize, const PROG: usize> MemBackend<SIZE, ERASE, PROG> {
    /// Number of erase units, with the geometry checked at compile time.
    pub const UNIT_COUNT: usize = {
        assert!(
            geometry_is_valid(SIZE, ERASE, PROG),
            "invalid MemBackend geometry"
        );
        assert!(SIZE / ERASE <= MAX_ERASE_UNITS, "too many erase units");
        SIZE / ERASE
    };

    /// Create a backend in the fully erased state (all bytes `0xFF`).
    pub const fn new() -> Self {
        // Referencing UNIT_COUNT forces the compile-time geometry check for
        // every instantiation of this type.
        let _ = Self::UNIT_COUNT;
        Self {
            data: [0xFF; SIZE],
            erase_counts: [0; MAX_ERASE_UNITS],
        }
    }

    /// Number of times `unit_index` has been erased, or `None` when the
    /// index is out of range.
    pub fn erase_count(&self, unit_index: usize) -> Option<u32> {
        self.erase_counts
            .get(..Self::UNIT_COUNT)?
            .get(unit_index)
            .copied()
    }

    /// Erase counters for all units, indexed by unit.
    pub fn erase_counts(&self) -> &[u32] {
        &self.erase_counts[..Self::UNIT_COUNT]
    }

    /// Raw view of the stored bytes (test and diagnostic hook).
    pub fn raw(&self) -> &[u8] {
        &self.data
    }

    /// Mutable raw view of the stored bytes.
    ///
    /// Intended for fault-injection tests (e.g. flipping a byte to model
    /// bit rot); production code must use [`StorageBackend::program`].
    pub fn raw_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

impl<const SIZE: usize, const ERASE: usize, const PROG: usize> Default
    for MemBackend<SIZE, ERASE, PROG>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const SIZE: usize, const ERASE: usize, const PROG: usize> StorageBackend
    for MemBackend<SIZE, ERASE, PROG>
{
    fn region_size(&self) -> usize {
        SIZE
    }

    fn erase_unit(&self) -> usize {
        ERASE
    }

    fn program_unit(&self) -> usize {
        PROG
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        check_read_range(SIZE, offset, buf.len())?;
        buf.copy_from_slice(&self.data[offset..offset + buf.len()]);
        Ok(())
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        if unit_index >= Self::UNIT_COUNT {
            return Err(StorageError::OutOfRange);
        }
        let start = unit_index * ERASE;
        self.data[start..start + ERASE].fill(0xFF);
        self.erase_counts[unit_index] = self.erase_counts[unit_index].saturating_add(1);
        Ok(())
    }

    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        check_program_range(SIZE, PROG, offset, data.len())?;
        let mut conflict = false;
        for (cell, &byte) in self.data[offset..offset + data.len()].iter_mut().zip(data) {
            let programmed = *cell & byte;
            if programmed != byte {
                // The caller asked for a bit the current state cannot
                // provide (0 -> 1). Real flash silently stores the AND; we
                // store it too so the corruption is detectable, and report
                // the violation.
                conflict = true;
            }
            *cell = programmed;
        }
        if conflict {
            return Err(StorageError::NotErased);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type Small = MemBackend<256, 64, 4>;

    #[test]
    fn starts_fully_erased() {
        let backend = Small::new();
        assert!(backend.raw().iter().all(|&b| b == 0xFF));
        assert_eq!(backend.region_size(), 256);
        assert_eq!(backend.erase_unit(), 64);
        assert_eq!(backend.program_unit(), 4);
    }

    #[test]
    fn program_clears_bits_and_read_returns_them() {
        let mut backend = Small::new();
        backend.program(8, &[0x12, 0x34, 0x56, 0x78]).unwrap();
        let mut buf = [0u8; 4];
        backend.read(8, &mut buf).unwrap();
        assert_eq!(buf, [0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn program_rejects_unaligned_offset_and_length() {
        let mut backend = Small::new();
        assert_eq!(backend.program(2, &[0; 4]), Err(StorageError::Unaligned));
        assert_eq!(backend.program(4, &[0; 3]), Err(StorageError::Unaligned));
    }

    #[test]
    fn program_rejects_out_of_range() {
        let mut backend = Small::new();
        assert_eq!(backend.program(256, &[0; 4]), Err(StorageError::OutOfRange));
        assert_eq!(backend.program(252, &[0; 8]), Err(StorageError::OutOfRange));
    }

    #[test]
    fn read_rejects_out_of_range() {
        let backend = Small::new();
        let mut buf = [0u8; 8];
        assert_eq!(backend.read(252, &mut buf), Err(StorageError::OutOfRange));
    }

    #[test]
    fn conflicting_reprogram_fails_and_corrupts_detectably() {
        let mut backend = Small::new();
        backend.program(0, &[0x0F, 0x0F, 0x0F, 0x0F]).unwrap();
        // Setting bits back to 1 must fail; the cell keeps the AND.
        assert_eq!(
            backend.program(0, &[0xF0, 0x0F, 0x0F, 0x0F]),
            Err(StorageError::NotErased)
        );
        let mut buf = [0u8; 4];
        backend.read(0, &mut buf).unwrap();
        assert_eq!(buf, [0x00, 0x0F, 0x0F, 0x0F]);
    }

    #[test]
    fn reprogram_that_only_clears_bits_succeeds() {
        let mut backend = Small::new();
        backend.program(0, &[0xFF, 0x0F, 0xFF, 0xFF]).unwrap();
        backend.program(0, &[0x0F, 0x0F, 0xFF, 0xFF]).unwrap();
        let mut buf = [0u8; 4];
        backend.read(0, &mut buf).unwrap();
        assert_eq!(buf, [0x0F, 0x0F, 0xFF, 0xFF]);
    }

    #[test]
    fn erase_restores_unit_and_counts_wear() {
        let mut backend = Small::new();
        backend.program(64, &[0u8; 16]).unwrap();
        assert_eq!(backend.erase_count(1), Some(0));
        backend.erase(1).unwrap();
        let mut buf = [0u8; 16];
        backend.read(64, &mut buf).unwrap();
        assert!(buf.iter().all(|&b| b == 0xFF));
        assert_eq!(backend.erase_count(1), Some(1));
        assert_eq!(backend.erase_counts(), &[0, 1, 0, 0]);
    }

    #[test]
    fn erase_rejects_bad_unit_index() {
        let mut backend = Small::new();
        assert_eq!(backend.erase(4), Err(StorageError::OutOfRange));
        assert_eq!(backend.erase_count(4), None);
    }

    #[test]
    fn erase_only_touches_its_unit() {
        let mut backend = Small::new();
        backend.program(60, &[1, 2, 3, 4]).unwrap();
        backend.program(64, &[5, 6, 7, 8]).unwrap();
        backend.erase(1).unwrap();
        let mut buf = [0u8; 4];
        backend.read(60, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn byte_granularity_backend_accepts_any_length() {
        let mut backend = MemBackend::<128, 32, 1>::new();
        backend.program(1, &[0xAA]).unwrap();
        let mut buf = [0u8; 1];
        backend.read(1, &mut buf).unwrap();
        assert_eq!(buf, [0xAA]);
    }
}
