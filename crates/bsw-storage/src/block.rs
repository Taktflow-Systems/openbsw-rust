//! Flash-geometry-independent block storage contract.
//!
//! Upstream OpenBSW addresses persistent data through numeric block IDs
//! routed by `MappingStorage` to an EEPROM (`EepStorage`) or FEE
//! (`FeeStorage`) implementation. This module keeps that shape:
//! [`BlockId`] names a block, [`BlockMetadata`] carries the bookkeeping
//! upstream stores in its block headers, and [`BlockStore`] is the storage
//! interface applications program against — independent of erase units,
//! program units, or any other flash geometry.

use crate::backend::StorageError;

/// Identifier of a stored block.
///
/// Upstream uses `uint32_t` job IDs; the port narrows this to `u16`, which
/// bounds index tables on `no_std` targets while leaving 65536 block names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BlockId(pub u16);

impl BlockId {
    /// Raw numeric value of the identifier.
    pub const fn raw(self) -> u16 {
        self.0
    }
}

impl From<u16> for BlockId {
    fn from(raw: u16) -> Self {
        Self(raw)
    }
}

/// Per-block bookkeeping stored alongside each payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockMetadata {
    /// Application-defined format version of the payload.
    pub version: u16,
    /// Payload length in bytes.
    pub length: usize,
    /// Write generation counter: incremented on every successful write of
    /// any block, so newer records always carry larger generations.
    pub generation: u32,
}

/// Block-level storage interface.
///
/// All operations are synchronous and complete before returning; the
/// asynchronous layer lives in [`crate::jobs`]. Implementations must be
/// free of heap allocation and must never panic on malformed input.
pub trait BlockStore {
    /// `true` when `id` holds valid (non-invalidated) data.
    fn contains(&self, id: BlockId) -> bool;

    /// Metadata of the current record for `id`.
    ///
    /// Fails with [`StorageError::UnknownBlock`] when the block is absent
    /// or invalidated.
    fn metadata(&self, id: BlockId) -> Result<BlockMetadata, StorageError>;

    /// Read the payload of `id` into `buf`, returning the payload length.
    ///
    /// `buf` must be at least as long as the stored payload
    /// ([`StorageError::InvalidParameter`] otherwise). The payload is
    /// integrity-checked on every read; a mismatch yields
    /// [`StorageError::CorruptData`].
    fn read(&self, id: BlockId, buf: &mut [u8]) -> Result<usize, StorageError>;

    /// Atomically replace the payload of `id`.
    ///
    /// On success the new payload with `version` is durable. On failure the
    /// previous payload (if any) remains readable — a write never leaves a
    /// block in a garbage state.
    fn write(&mut self, id: BlockId, version: u16, data: &[u8]) -> Result<(), StorageError>;

    /// Invalidate `id` so that it no longer [`contains`](Self::contains)
    /// data.
    ///
    /// Invalidation is idempotent: invalidating an absent or already
    /// invalidated block succeeds without effect.
    fn invalidate(&mut self, id: BlockId) -> Result<(), StorageError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_id_round_trip() {
        let id = BlockId::from(0x1234);
        assert_eq!(id.raw(), 0x1234);
        assert_eq!(BlockId(0x1234), id);
    }

    #[test]
    fn block_id_ordering() {
        assert!(BlockId(1) < BlockId(2));
    }
}
