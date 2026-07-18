//! Low-level storage backend contract.
//!
//! [`StorageBackend`] models the raw persistent medium the way NOR flash
//! behaves: a byte-addressed region divided into erase units; erasing sets
//! every byte of a unit to `0xFF`; programming can only clear bits
//! (`1 -> 0`) at a fixed program granularity. A later STM32 flash driver
//! implements this trait directly; [`crate::mem::MemBackend`] and
//! [`crate::file::FileBackend`] implement it for host testing.

/// Error codes shared by backends and block stores.
///
/// The set mirrors the result semantics of the upstream `storage` module
/// (`StorageJob::Result::{Error, DataLoss, Success}`) but with typed causes
/// instead of a coarse error/data-loss split.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageError {
    /// An offset or length falls outside the storage region.
    OutOfRange,
    /// An offset or length violates the program-unit alignment rules.
    Unaligned,
    /// A program operation tried to set bits (`0 -> 1`) that only an erase
    /// can restore; the affected bytes are detectably corrupt.
    NotErased,
    /// The underlying medium failed (hardware fault, file I/O error, or a
    /// simulated power cut).
    Io,
    /// Stored data failed an integrity check (CRC or structural).
    CorruptData,
    /// The requested block does not exist or has been invalidated.
    UnknownBlock,
    /// The store, an area, or a fixed-capacity table cannot hold the
    /// requested data.
    CapacityExceeded,
    /// A parameter violates the API contract (bad geometry, short buffer,
    /// oversized payload).
    InvalidParameter,
    /// The component is busy and cannot accept more work right now.
    Busy,
}

/// Contract for a flash-like storage medium.
///
/// # Geometry invariants
///
/// - [`region_size`](Self::region_size) is a multiple of
///   [`erase_unit`](Self::erase_unit).
/// - [`erase_unit`](Self::erase_unit) is a power of two and a multiple of
///   [`program_unit`](Self::program_unit).
/// - [`program_unit`](Self::program_unit) is 1, 4, or 8 bytes.
///
/// # Data semantics
///
/// - After [`erase`](Self::erase), every byte of the unit reads `0xFF`.
/// - [`program`](Self::program) may only clear bits relative to the current
///   contents. Programming a non-erased area with conflicting bits must
///   either fail with [`StorageError::NotErased`] or corrupt detectably
///   (the stored value is the AND of old and new data).
/// - [`read`](Self::read) has no alignment restrictions.
pub trait StorageBackend {
    /// Total size of the storage region in bytes.
    fn region_size(&self) -> usize;

    /// Bytes per erasable unit (a power of two).
    fn erase_unit(&self) -> usize;

    /// Program granularity in bytes (1, 4, or 8).
    fn program_unit(&self) -> usize;

    /// Read `buf.len()` bytes starting at `offset`.
    ///
    /// Fails with [`StorageError::OutOfRange`] when the range exceeds the
    /// region. Reads are allowed at any offset and length.
    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError>;

    /// Erase one erase unit, restoring every byte in it to `0xFF`.
    ///
    /// `unit_index` counts erase units from the start of the region; it must
    /// be below `region_size() / erase_unit()`.
    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError>;

    /// Program `data` starting at `offset`.
    ///
    /// `offset` and `data.len()` must be multiples of
    /// [`program_unit`](Self::program_unit). Programming may only clear bits
    /// (`1 -> 0`) relative to the erased state `0xFF`; see the trait-level
    /// documentation for the conflicting-bits contract.
    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError>;
}

/// Validate a `(offset, len)` range and its program alignment against a
/// backend geometry.
///
/// Returns [`StorageError::OutOfRange`] when the range exceeds
/// `region_size` and [`StorageError::Unaligned`] when `offset` or `len` is
/// not a multiple of `program_unit`.
pub(crate) fn check_program_range(
    region_size: usize,
    program_unit: usize,
    offset: usize,
    len: usize,
) -> Result<(), StorageError> {
    if !offset.is_multiple_of(program_unit) || !len.is_multiple_of(program_unit) {
        return Err(StorageError::Unaligned);
    }
    check_read_range(region_size, offset, len)
}

/// Validate a `(offset, len)` read range against `region_size`.
pub(crate) fn check_read_range(
    region_size: usize,
    offset: usize,
    len: usize,
) -> Result<(), StorageError> {
    let end = offset.checked_add(len).ok_or(StorageError::OutOfRange)?;
    if end > region_size {
        return Err(StorageError::OutOfRange);
    }
    Ok(())
}

/// Validate a backend geometry triple.
///
/// Shared by [`crate::mem::MemBackend`] (at compile time) and
/// [`crate::file::FileBackend`] (at run time).
pub(crate) const fn geometry_is_valid(size: usize, erase: usize, prog: usize) -> bool {
    size > 0
        && erase > 0
        && erase.is_power_of_two()
        && size.is_multiple_of(erase)
        && (prog == 1 || prog == 4 || prog == 8)
        && erase.is_multiple_of(prog)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn program_range_rejects_misalignment() {
        assert_eq!(
            check_program_range(64, 4, 2, 4),
            Err(StorageError::Unaligned)
        );
        assert_eq!(
            check_program_range(64, 4, 4, 6),
            Err(StorageError::Unaligned)
        );
        assert_eq!(check_program_range(64, 4, 4, 8), Ok(()));
    }

    #[test]
    fn read_range_rejects_overflow_and_out_of_range() {
        assert_eq!(check_read_range(64, 60, 8), Err(StorageError::OutOfRange));
        assert_eq!(
            check_read_range(64, usize::MAX, 2),
            Err(StorageError::OutOfRange)
        );
        assert_eq!(check_read_range(64, 60, 4), Ok(()));
    }

    #[test]
    fn geometry_validation() {
        assert!(geometry_is_valid(1024, 256, 4));
        assert!(geometry_is_valid(1024, 256, 8));
        assert!(geometry_is_valid(1024, 256, 1));
        assert!(!geometry_is_valid(1024, 250, 4));
        assert!(!geometry_is_valid(1000, 256, 4));
        assert!(!geometry_is_valid(1024, 256, 3));
        assert!(!geometry_is_valid(0, 256, 4));
    }
}
