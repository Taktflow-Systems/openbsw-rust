//! Blob streaming over a journaled store: roundtrip, abandoned writes,
//! truncation and corruption detection, and a blob larger than any block
//! the store could hold in one piece.

use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};

use bsw_storage::blob::{BlobError, BlobLayout, BlobReader, BlobWriter};
use bsw_storage::file::{FileBackend, FileGeometry};
use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_storage::{BlockId, BlockStore, StorageError};

type Backend = MemBackend<16384, 1024, 4>;
type Store = JournalStore<Backend, 40>;

const LAYOUT: BlobLayout = BlobLayout {
    descriptor: BlockId(1),
    chunk_base: BlockId(10),
    max_chunks: 15,
    chunk_size: 48,
};

fn store() -> Store {
    JournalStore::mount(Backend::new()).expect("mount")
}

fn pattern(seed: u8, len: usize) -> Vec<u8> {
    (0..len)
        .map(|i| seed ^ (i as u8) ^ ((i >> 8) as u8))
        .collect()
}

fn write_blob(store: &mut Store, seed: u8, total: usize) {
    let data = pattern(seed, total);
    let mut writer = BlobWriter::open(store, LAYOUT, total as u32).expect("open writer");
    for chunk in data.chunks(usize::from(LAYOUT.chunk_size)) {
        writer.write_chunk(chunk).expect("write chunk");
    }
    writer.finalize().expect("finalize");
}

fn read_blob(store: &Store) -> Vec<u8> {
    let mut reader = BlobReader::open(store, LAYOUT).expect("open reader");
    let mut data = Vec::new();
    let mut buf = [0u8; 48];
    for index in 0..reader.chunk_count() {
        let len = reader.read_chunk(index, &mut buf).expect("read chunk");
        data.extend_from_slice(&buf[..len]);
    }
    reader.finish().expect("whole-blob CRC must match");
    data
}

#[test]
fn roundtrip_is_byte_exact() {
    let mut store = store();
    let total = 700;
    write_blob(&mut store, 0x5A, total);
    {
        let reader = BlobReader::open(&store, LAYOUT).expect("open reader");
        assert_eq!(reader.total_len(), 700);
        assert_eq!(reader.chunk_count(), 15);
        assert_eq!(reader.chunk_len(0), 48);
        assert_eq!(reader.chunk_len(14), 700 - 14 * 48);
    }
    assert_eq!(read_blob(&store), pattern(0x5A, total));
    // Independent verification path.
    let reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut scratch = [0u8; 48];
    reader.verify(&mut scratch).expect("verify");
}

#[test]
fn seek_reads_single_chunk() {
    let mut store = store();
    write_blob(&mut store, 0x21, 700);
    let mut reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut buf = [0u8; 48];
    let len = reader.read_chunk(7, &mut buf).expect("seek to chunk 7");
    assert_eq!(len, 48);
    assert_eq!(&buf[..len], &pattern(0x21, 700)[7 * 48..8 * 48]);
    // Out-of-order access disables the streaming CRC conclusion.
    assert_eq!(reader.finish(), Err(BlobError::Incomplete));
}

#[test]
fn abandoned_rewrite_leaves_previous_blob_intact() {
    let mut store = store();
    write_blob(&mut store, 0x33, 700);
    // Start a replacement blob but abandon it before finalize.
    {
        let mut writer = BlobWriter::open(&mut store, LAYOUT, 500).expect("open writer");
        let junk = pattern(0x44, 500);
        writer.write_chunk(&junk[..48]).expect("chunk 0");
        writer.write_chunk(&junk[48..96]).expect("chunk 1");
        writer.write_chunk(&junk[96..144]).expect("chunk 2");
        // Dropped without finalize.
    }
    assert_eq!(
        read_blob(&store),
        pattern(0x33, 700),
        "old blob must be intact"
    );
    // A later complete rewrite becomes current.
    write_blob(&mut store, 0x44, 500);
    assert_eq!(read_blob(&store), pattern(0x44, 500));
}

#[test]
fn finalize_without_all_bytes_is_rejected() {
    let mut store = store();
    let mut writer = BlobWriter::open(&mut store, LAYOUT, 100).expect("open writer");
    writer.write_chunk(&pattern(1, 48)).expect("chunk 0");
    assert_eq!(writer.finalize(), Err(BlobError::Incomplete));
    // No descriptor was published.
    assert!(matches!(
        BlobReader::open(&store, LAYOUT),
        Err(BlobError::Storage(StorageError::UnknownBlock))
    ));
}

#[test]
fn wrong_chunk_length_is_rejected() {
    let mut store = store();
    // 100 bytes split as 48 + 48 + 4.
    let mut writer = BlobWriter::open(&mut store, LAYOUT, 100).expect("open writer");
    assert_eq!(
        writer.write_chunk(&pattern(1, 20)),
        Err(BlobError::ChunkLength)
    );
    writer.write_chunk(&pattern(1, 48)).expect("chunk 0");
    writer.write_chunk(&pattern(2, 48)).expect("chunk 1");
    // Final chunk must be exactly the remainder (4 bytes).
    assert_eq!(
        writer.write_chunk(&pattern(3, 48)),
        Err(BlobError::ChunkLength)
    );
    writer.write_chunk(&pattern(3, 4)).expect("final chunk");
    writer.finalize().expect("finalize");
}

#[test]
fn oversized_blob_is_rejected() {
    let mut store = store();
    let too_big = u32::from(LAYOUT.max_chunks) * u32::from(LAYOUT.chunk_size) + 1;
    assert!(matches!(
        BlobWriter::open(&mut store, LAYOUT, too_big),
        Err(BlobError::TooLarge)
    ));
}

#[test]
fn corruption_of_a_chunk_is_detected() {
    let mut store = store();
    write_blob(&mut store, 0x66, 700);
    // Overwrite chunk 5 in both chunk sets with different content of the
    // right length: the journal record stays valid, so only the
    // whole-blob CRC can catch it.
    let wrong = pattern(0x99, 48);
    store
        .write(BlockId(10 + 5), 1, &wrong)
        .expect("corrupt set 0");
    store
        .write(BlockId(10 + LAYOUT.max_chunks + 5), 1, &wrong)
        .expect("corrupt set 1");
    let reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut scratch = [0u8; 48];
    assert_eq!(reader.verify(&mut scratch), Err(BlobError::CrcMismatch));
    // The streaming path detects it as well.
    let mut reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut buf = [0u8; 48];
    for index in 0..reader.chunk_count() {
        reader
            .read_chunk(index, &mut buf)
            .expect("chunks read fine");
    }
    assert_eq!(reader.finish(), Err(BlobError::CrcMismatch));
}

#[test]
fn truncation_is_detected() {
    let mut store = store();
    write_blob(&mut store, 0x77, 700);
    // Losing a chunk block (both sets, to be independent of the active
    // set) must surface as truncation.
    store.invalidate(BlockId(10 + 9)).expect("drop set 0 chunk");
    store
        .invalidate(BlockId(10 + LAYOUT.max_chunks + 9))
        .expect("drop set 1 chunk");
    let mut reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut buf = [0u8; 48];
    assert_eq!(reader.read_chunk(9, &mut buf), Err(BlobError::Truncated));
    let reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut scratch = [0u8; 48];
    assert_eq!(reader.verify(&mut scratch), Err(BlobError::Truncated));
}

#[test]
fn short_chunk_is_detected_as_truncation() {
    let mut store = store();
    write_blob(&mut store, 0x12, 700);
    let short = pattern(0x12, 10);
    store
        .write(BlockId(10 + 3), 1, &short)
        .expect("shorten set 0");
    store
        .write(BlockId(10 + LAYOUT.max_chunks + 3), 1, &short)
        .expect("shorten set 1");
    let mut reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    let mut buf = [0u8; 48];
    assert_eq!(reader.read_chunk(3, &mut buf), Err(BlobError::Truncated));
}

#[test]
fn corrupt_descriptor_is_rejected() {
    let mut store = store();
    write_blob(&mut store, 0x55, 200);
    store
        .write(LAYOUT.descriptor, 1, b"not a valid descriptor")
        .expect("clobber descriptor");
    assert!(matches!(
        BlobReader::open(&store, LAYOUT),
        Err(BlobError::InvalidDescriptor)
    ));
}

#[test]
fn empty_blob_round_trips() {
    let mut store = store();
    let writer = BlobWriter::open(&mut store, LAYOUT, 0).expect("open writer");
    assert_eq!(writer.chunk_count(), 0);
    writer.finalize().expect("finalize empty");
    let reader = BlobReader::open(&store, LAYOUT).expect("open reader");
    assert_eq!(reader.total_len(), 0);
    reader.finish().expect("empty CRC");
}

// ---------------------------------------------------------------------
// A blob no single block could ever hold (payload length > u16::MAX).
// ---------------------------------------------------------------------

static COUNTER: AtomicU32 = AtomicU32::new(0);

fn temp_path(tag: &str) -> PathBuf {
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "bsw-storage-blob-{}-{n}-{tag}.bin",
        std::process::id()
    ))
}

struct TempFile(PathBuf);

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

#[test]
fn blob_larger_than_any_single_block_streams_through_file_store() {
    let path = temp_path("large");
    let _cleanup = TempFile(path.clone());
    let geometry = FileGeometry {
        region_size: 192 * 1024,
        erase_unit: 16 * 1024,
        program_unit: 4,
    };
    let backend = FileBackend::create(&path, geometry).expect("create");
    let mut store: JournalStore<FileBackend, 160> = JournalStore::mount(backend).expect("mount");

    let total: usize = 70_000;
    let data = pattern(0xC3, total);
    // No single block can hold this payload: block lengths are u16.
    assert_eq!(
        store.write(BlockId(2), 1, &data),
        Err(StorageError::InvalidParameter),
        "a single block must be unable to hold the blob"
    );

    let layout = BlobLayout {
        descriptor: BlockId(1),
        chunk_base: BlockId(10),
        max_chunks: 70,
        chunk_size: 1024,
    };
    let mut writer = BlobWriter::open(&mut store, layout, total as u32).expect("open writer");
    for chunk in data.chunks(usize::from(layout.chunk_size)) {
        writer.write_chunk(chunk).expect("write chunk");
    }
    writer.finalize().expect("finalize");

    // Reopen the file to cross a process-lifetime boundary as well.
    let backend = store.into_inner();
    drop(backend);
    let backend = FileBackend::open(&path).expect("reopen");
    let store: JournalStore<FileBackend, 160> = JournalStore::mount(backend).expect("re-mount");
    let mut reader = BlobReader::open(&store, layout).expect("open reader");
    assert_eq!(reader.total_len() as usize, total);
    let mut out = Vec::with_capacity(total);
    let mut buf = [0u8; 1024];
    for index in 0..reader.chunk_count() {
        let len = reader.read_chunk(index, &mut buf).expect("read chunk");
        out.extend_from_slice(&buf[..len]);
    }
    reader.finish().expect("whole-blob CRC");
    assert_eq!(out, data, "streamed blob must be byte-exact");
}
