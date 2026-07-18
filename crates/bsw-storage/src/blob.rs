//! Bounded streaming of payloads larger than one block.
//!
//! A *blob* is stored as a descriptor block plus a run of fixed-size chunk
//! blocks over any [`BlockStore`]. Two chunk-block *sets* (0 and 1)
//! alternate: [`BlobWriter`] streams the new blob into the set the current
//! descriptor does **not** reference and publishes it atomically by
//! rewriting the descriptor in [`BlobWriter::finalize`]. Abandoning a
//! writer without finalizing therefore leaves the previous blob fully
//! intact. [`BlobReader`] validates the descriptor, supports sequential
//! and chunk-indexed (seek) reads, and detects truncation and corruption
//! via the whole-blob CRC-32 stored in the descriptor.
//!
//! No heap is used; callers provide the chunk buffers.

use bsw_util::crc::{Crc32, CrcDigest32, CRC32_ETHERNET};

use crate::backend::StorageError;
use crate::block::{BlockId, BlockStore};

/// CRC-32 algorithm for whole-blob integrity (CRC-32/ISO-HDLC).
static BLOB_CRC: Crc32 = CRC32_ETHERNET;

/// Descriptor payload magic: `"BLOB"` little-endian.
const DESCRIPTOR_MAGIC: u32 = 0x424C_4F42;
/// Serialized descriptor payload length.
const DESCRIPTOR_LEN: usize = 18;
/// Block version used for descriptor and chunk blocks.
const BLOB_FORMAT_VERSION: u16 = 1;

/// Errors reported by the blob layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlobError {
    /// An underlying block-store operation failed.
    Storage(StorageError),
    /// The [`BlobLayout`] is inconsistent (zero sizes, ID overflow, or the
    /// descriptor ID lies inside the chunk ID range).
    InvalidLayout,
    /// The stored descriptor is missing structure or contradicts the
    /// layout.
    InvalidDescriptor,
    /// The blob does not fit into `max_chunks` chunks.
    TooLarge,
    /// A written chunk does not have the exact expected length.
    ChunkLength,
    /// Chunks were not visited in the order the operation requires.
    Sequence,
    /// Not all payload bytes were streamed before
    /// [`BlobWriter::finalize`] or [`BlobReader::finish`].
    Incomplete,
    /// A chunk block is missing or shorter than the descriptor promises.
    Truncated,
    /// The whole-blob CRC does not match the descriptor.
    CrcMismatch,
}

impl From<StorageError> for BlobError {
    fn from(error: StorageError) -> Self {
        Self::Storage(error)
    }
}

/// Block-ID layout of one blob.
///
/// The blob occupies `descriptor` plus the ID range
/// `[chunk_base, chunk_base + 2 * max_chunks)` — chunk set 0 followed by
/// chunk set 1. The caller is responsible for keeping the ranges of
/// different blobs (and other blocks) disjoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlobLayout {
    /// Block holding the blob descriptor.
    pub descriptor: BlockId,
    /// First block ID of chunk set 0.
    pub chunk_base: BlockId,
    /// Maximum number of chunks per set.
    pub max_chunks: u16,
    /// Fixed chunk size in bytes; only the final chunk may be shorter.
    pub chunk_size: u16,
}

impl BlobLayout {
    fn validate(self) -> Result<(), BlobError> {
        if self.max_chunks == 0 || self.chunk_size == 0 {
            return Err(BlobError::InvalidLayout);
        }
        let base = u32::from(self.chunk_base.raw());
        let span = 2 * u32::from(self.max_chunks);
        if base + span > u32::from(u16::MAX) + 1 {
            return Err(BlobError::InvalidLayout);
        }
        let descriptor = u32::from(self.descriptor.raw());
        if descriptor >= base && descriptor < base + span {
            return Err(BlobError::InvalidLayout);
        }
        Ok(())
    }

    fn chunk_id(self, set: u8, index: u16) -> BlockId {
        BlockId(self.chunk_base.raw() + u16::from(set) * self.max_chunks + index)
    }

    fn chunk_count_for(self, total_len: u32) -> Result<u16, BlobError> {
        let count = total_len.div_ceil(u32::from(self.chunk_size));
        if count > u32::from(self.max_chunks) {
            return Err(BlobError::TooLarge);
        }
        Ok(count as u16)
    }

    fn chunk_len(self, total_len: u32, index: u16) -> usize {
        let start = u32::from(index) * u32::from(self.chunk_size);
        let remaining = total_len.saturating_sub(start);
        remaining.min(u32::from(self.chunk_size)) as usize
    }
}

struct Descriptor {
    total_len: u32,
    chunk_size: u16,
    chunk_count: u16,
    set: u8,
    crc: u32,
}

fn encode_descriptor(descriptor: &Descriptor) -> [u8; DESCRIPTOR_LEN] {
    let mut payload = [0u8; DESCRIPTOR_LEN];
    payload[0..4].copy_from_slice(&DESCRIPTOR_MAGIC.to_le_bytes());
    payload[4..8].copy_from_slice(&descriptor.total_len.to_le_bytes());
    payload[8..10].copy_from_slice(&descriptor.chunk_size.to_le_bytes());
    payload[10..12].copy_from_slice(&descriptor.chunk_count.to_le_bytes());
    payload[12] = descriptor.set;
    payload[13] = 0;
    payload[14..18].copy_from_slice(&descriptor.crc.to_le_bytes());
    payload
}

fn decode_descriptor(payload: &[u8]) -> Option<Descriptor> {
    if payload.len() != DESCRIPTOR_LEN {
        return None;
    }
    if u32::from_le_bytes(payload[0..4].try_into().ok()?) != DESCRIPTOR_MAGIC {
        return None;
    }
    let descriptor = Descriptor {
        total_len: u32::from_le_bytes(payload[4..8].try_into().ok()?),
        chunk_size: u16::from_le_bytes(payload[8..10].try_into().ok()?),
        chunk_count: u16::from_le_bytes(payload[10..12].try_into().ok()?),
        set: payload[12],
        crc: u32::from_le_bytes(payload[14..18].try_into().ok()?),
    };
    if descriptor.set > 1 || descriptor.chunk_size == 0 {
        return None;
    }
    Some(descriptor)
}

/// Read and decode the current descriptor, if a valid one exists.
fn load_descriptor<S: BlockStore>(
    store: &S,
    layout: BlobLayout,
) -> Result<Option<Descriptor>, BlobError> {
    let mut payload = [0u8; DESCRIPTOR_LEN];
    match store.read(layout.descriptor, &mut payload) {
        Ok(len) => Ok(decode_descriptor(&payload[..len])),
        Err(StorageError::UnknownBlock | StorageError::CorruptData) => Ok(None),
        // A too-small buffer means the stored payload is not a descriptor.
        Err(StorageError::InvalidParameter) => Ok(None),
        Err(error) => Err(error.into()),
    }
}

/// Streaming writer for one blob.
///
/// Chunks must be delivered strictly in order, each exactly
/// [`BlobLayout::chunk_size`] bytes except the final chunk, which carries
/// the remainder. The new blob becomes current only when
/// [`finalize`](Self::finalize) succeeds.
pub struct BlobWriter<'a, S: BlockStore> {
    store: &'a mut S,
    layout: BlobLayout,
    total_len: u32,
    chunk_count: u16,
    target_set: u8,
    written: u32,
    next_chunk: u16,
    digest: CrcDigest32<'static>,
}

impl<'a, S: BlockStore> BlobWriter<'a, S> {
    /// Start writing a blob of exactly `total_len` bytes.
    ///
    /// The writer targets the chunk set not referenced by the current
    /// descriptor (set 0 when no valid descriptor exists), so an abandoned
    /// write never disturbs the previous blob.
    pub fn open(store: &'a mut S, layout: BlobLayout, total_len: u32) -> Result<Self, BlobError> {
        layout.validate()?;
        let chunk_count = layout.chunk_count_for(total_len)?;
        let target_set = match load_descriptor(store, layout)? {
            Some(descriptor) => 1 - descriptor.set,
            None => 0,
        };
        Ok(Self {
            store,
            layout,
            total_len,
            chunk_count,
            target_set,
            written: 0,
            next_chunk: 0,
            digest: BLOB_CRC.digest(),
        })
    }

    /// Total number of chunks this blob will occupy.
    pub fn chunk_count(&self) -> u16 {
        self.chunk_count
    }

    /// Bytes streamed so far.
    pub fn bytes_written(&self) -> u32 {
        self.written
    }

    /// Write the next chunk.
    ///
    /// `data` must be exactly the expected length for the current chunk
    /// index ([`BlobError::ChunkLength`] otherwise) and all chunks must
    /// arrive in order ([`BlobError::Sequence`] after the last chunk).
    pub fn write_chunk(&mut self, data: &[u8]) -> Result<(), BlobError> {
        if self.next_chunk >= self.chunk_count {
            return Err(BlobError::Sequence);
        }
        let expected = self.layout.chunk_len(self.total_len, self.next_chunk);
        if data.len() != expected {
            return Err(BlobError::ChunkLength);
        }
        let id = self.layout.chunk_id(self.target_set, self.next_chunk);
        self.store.write(id, BLOB_FORMAT_VERSION, data)?;
        self.digest.update(data);
        self.written += data.len() as u32;
        self.next_chunk += 1;
        Ok(())
    }

    /// Publish the blob: store the descriptor with the total length and
    /// the CRC-32 of the whole payload.
    ///
    /// Fails with [`BlobError::Incomplete`] when not all `total_len` bytes
    /// were streamed; the previous blob then remains current.
    pub fn finalize(self) -> Result<(), BlobError> {
        if self.written != self.total_len {
            return Err(BlobError::Incomplete);
        }
        let descriptor = Descriptor {
            total_len: self.total_len,
            chunk_size: self.layout.chunk_size,
            chunk_count: self.chunk_count,
            set: self.target_set,
            crc: self.digest.finalize(),
        };
        self.store.write(
            self.layout.descriptor,
            BLOB_FORMAT_VERSION,
            &encode_descriptor(&descriptor),
        )?;
        Ok(())
    }
}

/// Reader for a stored blob.
///
/// Supports sequential streaming (chunk 0, 1, 2, … then
/// [`finish`](Self::finish) for the whole-blob CRC check) as well as
/// random chunk access by index; [`verify`](Self::verify) checks
/// integrity independently of the access pattern.
pub struct BlobReader<'a, S: BlockStore> {
    store: &'a S,
    layout: BlobLayout,
    total_len: u32,
    chunk_count: u16,
    set: u8,
    expected_crc: u32,
    next_seq: u16,
    sequential: bool,
    digest: CrcDigest32<'static>,
}

impl<'a, S: BlockStore> BlobReader<'a, S> {
    /// Open the current blob, validating the descriptor against `layout`.
    pub fn open(store: &'a S, layout: BlobLayout) -> Result<Self, BlobError> {
        layout.validate()?;
        let mut payload = [0u8; DESCRIPTOR_LEN];
        let len = match store.read(layout.descriptor, &mut payload) {
            Ok(len) => len,
            // A stored payload larger than a descriptor cannot be one.
            Err(StorageError::InvalidParameter) => return Err(BlobError::InvalidDescriptor),
            Err(error) => return Err(error.into()),
        };
        let descriptor = decode_descriptor(&payload[..len]).ok_or(BlobError::InvalidDescriptor)?;
        if descriptor.chunk_size != layout.chunk_size
            || descriptor.chunk_count > layout.max_chunks
            || layout.chunk_count_for(descriptor.total_len) != Ok(descriptor.chunk_count)
        {
            return Err(BlobError::InvalidDescriptor);
        }
        Ok(Self {
            store,
            layout,
            total_len: descriptor.total_len,
            chunk_count: descriptor.chunk_count,
            set: descriptor.set,
            expected_crc: descriptor.crc,
            next_seq: 0,
            sequential: true,
            digest: BLOB_CRC.digest(),
        })
    }

    /// Total payload length in bytes.
    pub fn total_len(&self) -> u32 {
        self.total_len
    }

    /// Number of stored chunks.
    pub fn chunk_count(&self) -> u16 {
        self.chunk_count
    }

    /// Payload length of chunk `index`.
    pub fn chunk_len(&self, index: u16) -> usize {
        self.layout.chunk_len(self.total_len, index)
    }

    /// Read chunk `index` into `buf`, returning the chunk length.
    ///
    /// Any index below [`chunk_count`](Self::chunk_count) may be read
    /// (seek by chunk index); out-of-order access merely disables the
    /// sequential CRC accumulation used by [`finish`](Self::finish).
    pub fn read_chunk(&mut self, index: u16, buf: &mut [u8]) -> Result<usize, BlobError> {
        if index >= self.chunk_count {
            return Err(StorageError::OutOfRange.into());
        }
        let expected = self.chunk_len(index);
        if buf.len() < expected {
            return Err(StorageError::InvalidParameter.into());
        }
        let id = self.layout.chunk_id(self.set, index);
        let len = match self.store.read(id, &mut buf[..expected]) {
            Ok(len) => len,
            Err(StorageError::UnknownBlock) => return Err(BlobError::Truncated),
            Err(error) => return Err(error.into()),
        };
        if len != expected {
            return Err(BlobError::Truncated);
        }
        if self.sequential && index == self.next_seq {
            self.digest.update(&buf[..len]);
            self.next_seq += 1;
        } else {
            self.sequential = false;
        }
        Ok(len)
    }

    /// Conclude a fully sequential read and verify the whole-blob CRC.
    ///
    /// Fails with [`BlobError::Incomplete`] unless every chunk was read
    /// exactly once, in order, via [`read_chunk`](Self::read_chunk), and
    /// with [`BlobError::CrcMismatch`] when the accumulated CRC differs
    /// from the descriptor.
    pub fn finish(self) -> Result<(), BlobError> {
        if !self.sequential || self.next_seq != self.chunk_count {
            return Err(BlobError::Incomplete);
        }
        if self.digest.finalize() != self.expected_crc {
            return Err(BlobError::CrcMismatch);
        }
        Ok(())
    }

    /// Verify the whole blob using `scratch` (at least one chunk long),
    /// independent of any sequential read state.
    pub fn verify(&self, scratch: &mut [u8]) -> Result<(), BlobError> {
        if scratch.len() < usize::from(self.layout.chunk_size) && self.chunk_count > 0 {
            return Err(StorageError::InvalidParameter.into());
        }
        let mut digest = BLOB_CRC.digest();
        for index in 0..self.chunk_count {
            let expected = self.chunk_len(index);
            let id = self.layout.chunk_id(self.set, index);
            let len = match self.store.read(id, &mut scratch[..expected]) {
                Ok(len) => len,
                Err(StorageError::UnknownBlock) => return Err(BlobError::Truncated),
                Err(error) => return Err(error.into()),
            };
            if len != expected {
                return Err(BlobError::Truncated);
            }
            digest.update(&scratch[..len]);
        }
        if digest.finalize() != self.expected_crc {
            return Err(BlobError::CrcMismatch);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_round_trip() {
        let descriptor = Descriptor {
            total_len: 300,
            chunk_size: 64,
            chunk_count: 5,
            set: 1,
            crc: 0x1234_5678,
        };
        let decoded = decode_descriptor(&encode_descriptor(&descriptor)).unwrap();
        assert_eq!(decoded.total_len, 300);
        assert_eq!(decoded.chunk_size, 64);
        assert_eq!(decoded.chunk_count, 5);
        assert_eq!(decoded.set, 1);
        assert_eq!(decoded.crc, 0x1234_5678);
    }

    #[test]
    fn descriptor_rejects_bad_magic_and_bad_set() {
        let descriptor = Descriptor {
            total_len: 300,
            chunk_size: 64,
            chunk_count: 5,
            set: 0,
            crc: 0,
        };
        let mut payload = encode_descriptor(&descriptor);
        payload[0] ^= 0xFF;
        assert!(decode_descriptor(&payload).is_none());
        let mut payload = encode_descriptor(&descriptor);
        payload[12] = 2;
        assert!(decode_descriptor(&payload).is_none());
        assert!(decode_descriptor(&payload[..17]).is_none());
    }

    #[test]
    fn layout_validation() {
        let good = BlobLayout {
            descriptor: BlockId(1),
            chunk_base: BlockId(10),
            max_chunks: 8,
            chunk_size: 64,
        };
        assert_eq!(good.validate(), Ok(()));
        assert_eq!(
            BlobLayout {
                chunk_size: 0,
                ..good
            }
            .validate(),
            Err(BlobError::InvalidLayout)
        );
        assert_eq!(
            BlobLayout {
                max_chunks: 0,
                ..good
            }
            .validate(),
            Err(BlobError::InvalidLayout)
        );
        // Descriptor inside the chunk range.
        assert_eq!(
            BlobLayout {
                descriptor: BlockId(12),
                ..good
            }
            .validate(),
            Err(BlobError::InvalidLayout)
        );
        // Chunk range overflowing the u16 ID space.
        assert_eq!(
            BlobLayout {
                chunk_base: BlockId(u16::MAX - 4),
                ..good
            }
            .validate(),
            Err(BlobError::InvalidLayout)
        );
    }

    #[test]
    fn chunk_arithmetic() {
        let layout = BlobLayout {
            descriptor: BlockId(1),
            chunk_base: BlockId(10),
            max_chunks: 8,
            chunk_size: 64,
        };
        assert_eq!(layout.chunk_count_for(0), Ok(0));
        assert_eq!(layout.chunk_count_for(64), Ok(1));
        assert_eq!(layout.chunk_count_for(65), Ok(2));
        assert_eq!(layout.chunk_count_for(64 * 8), Ok(8));
        assert_eq!(layout.chunk_count_for(64 * 8 + 1), Err(BlobError::TooLarge));
        assert_eq!(layout.chunk_len(130, 0), 64);
        assert_eq!(layout.chunk_len(130, 1), 64);
        assert_eq!(layout.chunk_len(130, 2), 2);
        assert_eq!(layout.chunk_id(0, 3), BlockId(13));
        assert_eq!(layout.chunk_id(1, 3), BlockId(21));
    }
}
