//! Journaled, power-fail-safe block store (upstream `FeeStorage` analogue).
//!
//! [`JournalStore`] implements [`BlockStore`] on top of any
//! [`StorageBackend`] using a two-area (A/B) copy-on-write design. The
//! region is split into two equally sized areas of whole erase units. Each
//! area is an append-only record log:
//!
//! ```text
//! record := header (24 B) | payload (padded to program unit) | commit mark
//! header := magic u32 | block id u16 | version u16 | generation u32
//!        |  length u16 | flags u16 | payload CRC32 | header CRC32
//! ```
//!
//! A write appends the record into erased space and programs the commit
//! mark **last**, as its own program-unit write; only the durable commit
//! mark makes the record current. When the active area cannot fit a
//! record, live records are copied into the other (fully erased) area with
//! fresh generations, the new record is appended there, and the old area
//! is erased — old data stays intact until the copies are committed.
//!
//! [`JournalStore::mount`] scans both areas, validates header and payload
//! CRCs, picks the newest valid generation per block, discards any
//! interrupted copy, and erases stale areas. Recovery after an interrupted
//! operation therefore always yields the old or the new payload, never
//! garbage: torn records lack a valid commit mark or fail their CRC and
//! are ignored, and generation counters disambiguate area A versus B.

use bsw_util::crc::{Crc32, CRC32_ETHERNET};

use crate::backend::{StorageBackend, StorageError};
use crate::block::{BlockId, BlockMetadata, BlockStore};

/// CRC-32 algorithm used for record integrity (CRC-32/ISO-HDLC).
static RECORD_CRC: Crc32 = CRC32_ETHERNET;

/// Record magic: `"BSWJ"` little-endian.
const RECORD_MAGIC: u32 = 0x4253_574A;
/// Serialized header length; a multiple of every supported program unit.
const HEADER_LEN: usize = 24;
/// Flag bit marking a tombstone (invalidation) record.
const FLAG_TOMBSTONE: u16 = 0x0001;
/// Byte value programmed into every byte of a commit mark.
const COMMIT_BYTE: u8 = 0x33;
/// Chunk size for streaming payload verification and relocation; a
/// multiple of every supported program unit.
// Large enough to keep physical flash relocation bounded without allocating,
// while remaining comfortably below the production task stack budgets.
const COPY_CHUNK: usize = 256;

/// Wear bookkeeping tracked by the store since [`JournalStore::mount`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WearStats {
    /// Erase-unit erase operations performed per area (`[A, B]`) since
    /// mount. Persistent lifetime counters are the backend's concern (see
    /// [`crate::mem::MemBackend::erase_counts`]).
    pub area_erase_counts: [u32; 2],
}

#[derive(Clone, Copy)]
struct IndexEntry {
    id: BlockId,
    area: usize,
    /// Absolute record offset in the backend region.
    offset: usize,
    len: u16,
    version: u16,
    generation: u32,
    invalidated: bool,
}

struct ParsedHeader {
    id: BlockId,
    version: u16,
    generation: u32,
    len: u16,
    flags: u16,
    payload_crc: u32,
}

/// Journaled block store over a [`StorageBackend`].
///
/// `MAX_BLOCKS` bounds the number of distinct block IDs (live or
/// invalidated) the RAM index can track. The store owns the backend; use
/// [`into_inner`](Self::into_inner) to recover it (e.g. to simulate a
/// power cycle in tests).
pub struct JournalStore<B: StorageBackend, const MAX_BLOCKS: usize> {
    backend: B,
    prog: usize,
    units_per_area: usize,
    area_size: usize,
    active: usize,
    cursor: [usize; 2],
    index: [Option<IndexEntry>; MAX_BLOCKS],
    next_generation: u32,
    wear: WearStats,
}

impl<B: StorageBackend, const MAX_BLOCKS: usize> JournalStore<B, MAX_BLOCKS> {
    /// Mount the store: scan both areas, rebuild the block index from the
    /// newest valid committed records, and reclaim invalid or stale areas.
    ///
    /// Fails with [`StorageError::InvalidParameter`] when the backend
    /// geometry cannot host two areas, and with
    /// [`StorageError::CapacityExceeded`] when more distinct blocks exist
    /// than `MAX_BLOCKS`.
    pub fn mount(backend: B) -> Result<Self, StorageError> {
        let prog = backend.program_unit();
        let erase = backend.erase_unit();
        let region = backend.region_size();
        if !crate::backend::geometry_is_valid(region, erase, prog) {
            return Err(StorageError::InvalidParameter);
        }
        let units_per_area = (region / erase) / 2;
        if units_per_area == 0 {
            return Err(StorageError::InvalidParameter);
        }
        let mut store = Self {
            backend,
            prog,
            units_per_area,
            area_size: units_per_area * erase,
            active: 0,
            cursor: [0; 2],
            index: [None; MAX_BLOCKS],
            next_generation: 1,
            wear: WearStats::default(),
        };
        let blank_a = store.scan_area(0)?;
        let blank_b = store.scan_area(1)?;
        store.reclaim([blank_a, blank_b])?;
        Ok(store)
    }

    /// Consume the store and return the backend.
    pub fn into_inner(self) -> B {
        self.backend
    }

    /// Shared access to the backend (diagnostics).
    pub fn backend(&self) -> &B {
        &self.backend
    }

    /// Mutable access to the backend.
    ///
    /// Intended for test harnesses (fault-injection arming); mutating
    /// stored bytes directly invalidates the RAM index.
    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    /// Wear metadata accumulated since mount.
    pub fn wear(&self) -> WearStats {
        self.wear
    }

    /// Index of the area currently receiving appends (0 = A, 1 = B).
    pub fn active_area(&self) -> usize {
        self.active
    }

    /// Size of one journal area in bytes.
    pub fn area_size(&self) -> usize {
        self.area_size
    }

    /// Free bytes remaining in the active area before the next relocation.
    pub fn free_space(&self) -> usize {
        self.area_size - self.cursor[self.active]
    }

    // ------------------------------------------------------------------
    // Mount: scanning and reclamation
    // ------------------------------------------------------------------

    fn area_base(&self, area: usize) -> usize {
        area * self.area_size
    }

    fn record_total(&self, len: usize) -> usize {
        HEADER_LEN + pad_to(len, self.prog) + self.prog
    }

    /// Scan one area, merging committed valid records into the index.
    ///
    /// Returns `true` when everything from the final cursor to the end of
    /// the area is blank (`0xFF`).
    fn scan_area(&mut self, area: usize) -> Result<bool, StorageError> {
        let base = self.area_base(area);
        let mut cursor = 0usize;
        while cursor + HEADER_LEN <= self.area_size {
            let mut hdr = [0u8; HEADER_LEN];
            self.backend.read(base + cursor, &mut hdr)?;
            if hdr.iter().all(|&b| b == 0xFF) {
                break;
            }
            let Some(parsed) = parse_header(&hdr) else {
                // Torn or foreign header: a fixed-size hole. Writes are
                // sequential, so a torn header is the last thing written;
                // skipping its window keeps the scan deterministic.
                cursor += HEADER_LEN;
                continue;
            };
            let total = self.record_total(usize::from(parsed.len));
            if cursor + total > self.area_size {
                cursor += HEADER_LEN;
                continue;
            }
            if self.record_is_committed(base + cursor, &parsed)? {
                self.merge_candidate(&parsed, area, base + cursor)?;
                self.next_generation = self.next_generation.max(parsed.generation.wrapping_add(1));
            }
            cursor += total;
        }
        self.cursor[area] = cursor;
        self.area_is_blank_from(area, cursor)
    }

    /// Check commit mark and payload CRC of the record at `offset`.
    fn record_is_committed(
        &self,
        offset: usize,
        parsed: &ParsedHeader,
    ) -> Result<bool, StorageError> {
        let padded = pad_to(usize::from(parsed.len), self.prog);
        let mut mark = [0u8; 8];
        let mark = &mut mark[..self.prog];
        self.backend.read(offset + HEADER_LEN + padded, mark)?;
        if mark.iter().any(|&b| b != COMMIT_BYTE) {
            return Ok(false);
        }
        let crc = self.payload_crc_at(offset + HEADER_LEN, usize::from(parsed.len))?;
        Ok(crc == parsed.payload_crc)
    }

    fn payload_crc_at(&self, offset: usize, len: usize) -> Result<u32, StorageError> {
        let mut digest = RECORD_CRC.digest();
        let mut chunk = [0u8; COPY_CHUNK];
        let mut done = 0usize;
        while done < len {
            let step = COPY_CHUNK.min(len - done);
            self.backend.read(offset + done, &mut chunk[..step])?;
            digest.update(&chunk[..step]);
            done += step;
        }
        Ok(digest.finalize())
    }

    fn merge_candidate(
        &mut self,
        parsed: &ParsedHeader,
        area: usize,
        offset: usize,
    ) -> Result<(), StorageError> {
        let entry = IndexEntry {
            id: parsed.id,
            area,
            offset,
            len: parsed.len,
            version: parsed.version,
            generation: parsed.generation,
            invalidated: parsed.flags & FLAG_TOMBSTONE != 0,
        };
        if let Some(existing) = self.entry_mut(parsed.id) {
            if parsed.generation > existing.generation {
                *existing = entry;
            }
            return Ok(());
        }
        let Some(slot) = self.index.iter_mut().find(|slot| slot.is_none()) else {
            return Err(StorageError::CapacityExceeded);
        };
        *slot = Some(entry);
        Ok(())
    }

    fn area_is_blank_from(&self, area: usize, from: usize) -> Result<bool, StorageError> {
        let base = self.area_base(area);
        let mut chunk = [0u8; COPY_CHUNK];
        let mut cursor = from;
        while cursor < self.area_size {
            let step = COPY_CHUNK.min(self.area_size - cursor);
            self.backend.read(base + cursor, &mut chunk[..step])?;
            if chunk[..step].iter().any(|&b| b != 0xFF) {
                return Ok(false);
            }
            cursor += step;
        }
        Ok(true)
    }

    /// Restore the mount invariant: all current records live in the active
    /// area and the other area is fully erased.
    fn reclaim(&mut self, blank_after: [bool; 2]) -> Result<(), StorageError> {
        let live = [self.live_count_in(0), self.live_count_in(1)];
        if live[0] > 0 && live[1] > 0 {
            // A copy-on-write flip was interrupted while records were being
            // copied into the newer area (had the flip reached its erase
            // phase, every old record would already be superseded and only
            // one area would hold current records). The older area is still
            // complete, so the safe, idempotent recovery is to discard the
            // partial flip target and rescan the surviving area.
            let newer = self.newest_area();
            let older = 1 - newer;
            self.erase_area(newer)?;
            self.index = [None; MAX_BLOCKS];
            let blank = self.scan_area(older)?;
            if !blank {
                return Err(StorageError::CorruptData);
            }
            self.active = older;
        } else if live[0] > 0 || live[1] > 0 {
            let occupied = usize::from(live[1] > 0);
            self.active = occupied;
            let other = 1 - occupied;
            if self.cursor[other] > 0 || !blank_after[other] {
                self.erase_area(other)?;
            }
            if !blank_after[occupied] {
                // Junk beyond the log tail (e.g. external corruption):
                // relocate live data out, then erase the tainted area.
                self.active = other;
                self.relocate_live_from(occupied, None)?;
                self.erase_area(occupied)?;
            }
        } else {
            self.active = 0;
            for (area, blank) in blank_after.iter().copied().enumerate() {
                if self.cursor[area] > 0 || !blank {
                    self.erase_area(area)?;
                }
            }
        }
        Ok(())
    }

    fn live_count_in(&self, area: usize) -> usize {
        self.index
            .iter()
            .flatten()
            .filter(|entry| entry.area == area)
            .count()
    }

    fn newest_area(&self) -> usize {
        self.index
            .iter()
            .flatten()
            .max_by_key(|entry| entry.generation)
            .map_or(0, |entry| entry.area)
    }

    /// Copy every non-tombstone record currently indexed in `from` into
    /// the active area, skipping `exclude` (a block about to be
    /// superseded). Tombstones are dropped: erasing their record leaves
    /// the block absent, which is semantically identical.
    fn relocate_live_from(
        &mut self,
        from: usize,
        exclude: Option<BlockId>,
    ) -> Result<(), StorageError> {
        for slot in 0..MAX_BLOCKS {
            let Some(entry) = self.index[slot] else {
                continue;
            };
            if entry.area != from || entry.invalidated || Some(entry.id) == exclude {
                continue;
            }
            self.relocate_entry(slot)?;
        }
        Ok(())
    }

    /// Rewrite the record of index slot `slot` into the active area with a
    /// fresh generation, verifying the payload CRC while streaming.
    fn relocate_entry(&mut self, slot: usize) -> Result<(), StorageError> {
        let Some(entry) = self.index[slot] else {
            return Err(StorageError::UnknownBlock);
        };
        let len = usize::from(entry.len);
        let total = self.record_total(len);
        let target = self.active;
        if self.cursor[target] + total > self.area_size {
            return Err(StorageError::CapacityExceeded);
        }
        let mut old_hdr = [0u8; HEADER_LEN];
        self.backend.read(entry.offset, &mut old_hdr)?;
        let old = parse_header(&old_hdr).ok_or(StorageError::CorruptData)?;
        let dst = self.area_base(target) + self.cursor[target];
        self.cursor[target] += total;
        let generation = self.alloc_generation();
        let hdr = build_header(
            entry.id,
            entry.version,
            generation,
            entry.len,
            0,
            old.payload_crc,
        );
        self.backend.program(dst, &hdr)?;
        let crc = self.copy_payload(entry.offset + HEADER_LEN, dst + HEADER_LEN, len)?;
        if crc != old.payload_crc {
            return Err(StorageError::CorruptData);
        }
        self.program_commit_mark(dst + HEADER_LEN + pad_to(len, self.prog))?;
        self.index[slot] = Some(IndexEntry {
            area: target,
            offset: dst,
            generation,
            ..entry
        });
        Ok(())
    }

    /// Stream `len` payload bytes from `src` to `dst`, returning the CRC
    /// of the copied bytes. The final partial program unit is padded with
    /// `0xFF` (a no-op on erased flash).
    fn copy_payload(&mut self, src: usize, dst: usize, len: usize) -> Result<u32, StorageError> {
        let mut digest = RECORD_CRC.digest();
        let mut done = 0usize;
        while done < len {
            let step = COPY_CHUNK.min(len - done);
            let mut chunk = [0xFFu8; COPY_CHUNK];
            self.backend.read(src + done, &mut chunk[..step])?;
            digest.update(&chunk[..step]);
            let programmed = pad_to(step, self.prog);
            self.backend.program(dst + done, &chunk[..programmed])?;
            done += step;
        }
        Ok(digest.finalize())
    }

    fn erase_area(&mut self, area: usize) -> Result<(), StorageError> {
        for unit in 0..self.units_per_area {
            self.backend.erase(area * self.units_per_area + unit)?;
            self.wear.area_erase_counts[area] = self.wear.area_erase_counts[area].saturating_add(1);
        }
        self.cursor[area] = 0;
        // Records that lived in this area are gone; purge their entries.
        for slot in &mut self.index {
            if slot.is_some_and(|entry| entry.area == area) {
                *slot = None;
            }
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // Write path
    // ------------------------------------------------------------------

    fn alloc_generation(&mut self) -> u32 {
        let generation = self.next_generation;
        self.next_generation = self.next_generation.wrapping_add(1);
        generation
    }

    fn entry_mut(&mut self, id: BlockId) -> Option<&mut IndexEntry> {
        self.index.iter_mut().flatten().find(|entry| entry.id == id)
    }

    fn entry(&self, id: BlockId) -> Option<&IndexEntry> {
        self.index.iter().flatten().find(|entry| entry.id == id)
    }

    fn slot_of(&self, id: BlockId) -> Option<usize> {
        (0..MAX_BLOCKS).find(|&slot| self.index[slot].is_some_and(|entry| entry.id == id))
    }

    /// Sum of record sizes that a relocation target would have to hold for
    /// all current blocks except `exclude`.
    fn live_bytes_except(&self, exclude: BlockId) -> usize {
        self.index
            .iter()
            .flatten()
            .filter(|entry| entry.id != exclude && !entry.invalidated)
            .map(|entry| self.record_total(usize::from(entry.len)))
            .sum()
    }

    /// Append a record for `id` into the active area, relocating first
    /// when the active area is full. Updates the index on success.
    fn write_record(
        &mut self,
        id: BlockId,
        version: u16,
        flags: u16,
        data: &[u8],
    ) -> Result<(), StorageError> {
        if data.len() > usize::from(u16::MAX) {
            return Err(StorageError::InvalidParameter);
        }
        let total = self.record_total(data.len());
        if self.live_bytes_except(id) + total > self.area_size {
            return Err(StorageError::CapacityExceeded);
        }
        // Reserve an index slot up front so nothing is programmed when the
        // index is full.
        let slot = match self.slot_of(id) {
            Some(slot) => slot,
            None => (0..MAX_BLOCKS)
                .find(|&slot| self.index[slot].is_none())
                .ok_or(StorageError::CapacityExceeded)?,
        };
        if self.cursor[self.active] + total > self.area_size {
            // Copy-on-write flip: move live records to the erased area,
            // append the new record there, then erase the old area. Old
            // data stays intact until every copy is committed.
            let old = self.active;
            self.active = 1 - old;
            self.relocate_live_from(old, Some(id))?;
            self.append_record(slot, id, version, flags, data)?;
            self.erase_area(old)?;
        } else {
            self.append_record(slot, id, version, flags, data)?;
        }
        Ok(())
    }

    /// Program header, payload, and (last) the commit mark of one record
    /// at the active-area cursor, then update index slot `slot`.
    fn append_record(
        &mut self,
        slot: usize,
        id: BlockId,
        version: u16,
        flags: u16,
        data: &[u8],
    ) -> Result<(), StorageError> {
        let total = self.record_total(data.len());
        let dst = self.area_base(self.active) + self.cursor[self.active];
        // Account the space before programming: a failed attempt may have
        // left partial bytes, so the space is consumed either way.
        self.cursor[self.active] += total;
        let generation = self.alloc_generation();
        let payload_crc = RECORD_CRC.checksum(data);
        let len = data.len() as u16;
        let hdr = build_header(id, version, generation, len, flags, payload_crc);
        self.backend.program(dst, &hdr)?;
        let full = data.len() - data.len() % self.prog;
        if full > 0 {
            self.backend.program(dst + HEADER_LEN, &data[..full])?;
        }
        if full < data.len() {
            let mut tail = [0xFFu8; 8];
            tail[..data.len() - full].copy_from_slice(&data[full..]);
            self.backend
                .program(dst + HEADER_LEN + full, &tail[..self.prog])?;
        }
        self.program_commit_mark(dst + HEADER_LEN + pad_to(data.len(), self.prog))?;
        self.index[slot] = Some(IndexEntry {
            id,
            area: self.active,
            offset: dst,
            len,
            version,
            generation,
            invalidated: flags & FLAG_TOMBSTONE != 0,
        });
        Ok(())
    }

    fn program_commit_mark(&mut self, offset: usize) -> Result<(), StorageError> {
        let mark = [COMMIT_BYTE; 8];
        self.backend.program(offset, &mark[..self.prog])
    }
}

impl<B: StorageBackend, const MAX_BLOCKS: usize> BlockStore for JournalStore<B, MAX_BLOCKS> {
    fn contains(&self, id: BlockId) -> bool {
        self.entry(id).is_some_and(|entry| !entry.invalidated)
    }

    fn metadata(&self, id: BlockId) -> Result<BlockMetadata, StorageError> {
        let entry = self.entry(id).ok_or(StorageError::UnknownBlock)?;
        if entry.invalidated {
            return Err(StorageError::UnknownBlock);
        }
        Ok(BlockMetadata {
            version: entry.version,
            length: usize::from(entry.len),
            generation: entry.generation,
        })
    }

    fn read(&self, id: BlockId, buf: &mut [u8]) -> Result<usize, StorageError> {
        let entry = *self.entry(id).ok_or(StorageError::UnknownBlock)?;
        if entry.invalidated {
            return Err(StorageError::UnknownBlock);
        }
        let len = usize::from(entry.len);
        if buf.len() < len {
            return Err(StorageError::InvalidParameter);
        }
        let mut hdr = [0u8; HEADER_LEN];
        self.backend.read(entry.offset, &mut hdr)?;
        let parsed = parse_header(&hdr).ok_or(StorageError::CorruptData)?;
        self.backend
            .read(entry.offset + HEADER_LEN, &mut buf[..len])?;
        if RECORD_CRC.checksum(&buf[..len]) != parsed.payload_crc {
            return Err(StorageError::CorruptData);
        }
        Ok(len)
    }

    fn write(&mut self, id: BlockId, version: u16, data: &[u8]) -> Result<(), StorageError> {
        self.write_record(id, version, 0, data)
    }

    fn invalidate(&mut self, id: BlockId) -> Result<(), StorageError> {
        match self.entry(id) {
            None => Ok(()),
            Some(entry) if entry.invalidated => Ok(()),
            Some(_) => self.write_record(id, 0, FLAG_TOMBSTONE, &[]),
        }
    }
}

// ----------------------------------------------------------------------
// Header serialization
// ----------------------------------------------------------------------

const fn pad_to(len: usize, unit: usize) -> usize {
    len.div_ceil(unit) * unit
}

fn build_header(
    id: BlockId,
    version: u16,
    generation: u32,
    len: u16,
    flags: u16,
    payload_crc: u32,
) -> [u8; HEADER_LEN] {
    let mut hdr = [0u8; HEADER_LEN];
    hdr[0..4].copy_from_slice(&RECORD_MAGIC.to_le_bytes());
    hdr[4..6].copy_from_slice(&id.raw().to_le_bytes());
    hdr[6..8].copy_from_slice(&version.to_le_bytes());
    hdr[8..12].copy_from_slice(&generation.to_le_bytes());
    hdr[12..14].copy_from_slice(&len.to_le_bytes());
    hdr[14..16].copy_from_slice(&flags.to_le_bytes());
    hdr[16..20].copy_from_slice(&payload_crc.to_le_bytes());
    let header_crc = RECORD_CRC.checksum(&hdr[0..20]);
    hdr[20..24].copy_from_slice(&header_crc.to_le_bytes());
    hdr
}

fn parse_header(hdr: &[u8; HEADER_LEN]) -> Option<ParsedHeader> {
    let magic = u32::from_le_bytes(hdr[0..4].try_into().ok()?);
    if magic != RECORD_MAGIC {
        return None;
    }
    let stored_crc = u32::from_le_bytes(hdr[20..24].try_into().ok()?);
    if RECORD_CRC.checksum(&hdr[0..20]) != stored_crc {
        return None;
    }
    Some(ParsedHeader {
        id: BlockId(u16::from_le_bytes(hdr[4..6].try_into().ok()?)),
        version: u16::from_le_bytes(hdr[6..8].try_into().ok()?),
        generation: u32::from_le_bytes(hdr[8..12].try_into().ok()?),
        len: u16::from_le_bytes(hdr[12..14].try_into().ok()?),
        flags: u16::from_le_bytes(hdr[14..16].try_into().ok()?),
        payload_crc: u32::from_le_bytes(hdr[16..20].try_into().ok()?),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trip() {
        let hdr = build_header(BlockId(7), 3, 42, 5, 0, 0xDEAD_BEEF);
        let parsed = parse_header(&hdr).unwrap();
        assert_eq!(parsed.id, BlockId(7));
        assert_eq!(parsed.version, 3);
        assert_eq!(parsed.generation, 42);
        assert_eq!(parsed.len, 5);
        assert_eq!(parsed.flags, 0);
        assert_eq!(parsed.payload_crc, 0xDEAD_BEEF);
    }

    #[test]
    fn header_rejects_bit_flip() {
        let mut hdr = build_header(BlockId(7), 3, 42, 5, 0, 0xDEAD_BEEF);
        hdr[9] ^= 0x10;
        assert!(parse_header(&hdr).is_none());
    }

    #[test]
    fn header_rejects_bad_magic_and_all_ff() {
        let mut hdr = [0xFFu8; HEADER_LEN];
        assert!(parse_header(&hdr).is_none());
        hdr[0..4].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        assert!(parse_header(&hdr).is_none());
    }

    #[test]
    fn pad_to_rounds_up() {
        assert_eq!(pad_to(0, 4), 0);
        assert_eq!(pad_to(1, 4), 4);
        assert_eq!(pad_to(4, 4), 4);
        assert_eq!(pad_to(5, 8), 8);
        assert_eq!(pad_to(5, 1), 5);
    }
}
