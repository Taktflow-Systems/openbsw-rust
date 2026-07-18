//! FileBackend conformance: the same suites as MemBackend, plus reopen
//! behaviour across a (simulated) process-lifetime boundary.

use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};

use bsw_storage::conformance::{
    run_backend_contract_suite, run_block_store_suite, run_cut_point_suite,
};
use bsw_storage::file::{FileBackend, FileGeometry};
use bsw_storage::journal::JournalStore;
use bsw_storage::{BlockId, BlockStore, StorageError};

const GEOMETRY: FileGeometry = FileGeometry {
    region_size: 1024,
    erase_unit: 128,
    program_unit: 4,
};

static COUNTER: AtomicU32 = AtomicU32::new(0);

/// Unique path under the OS temp directory (process id + counter).
fn temp_path(tag: &str) -> PathBuf {
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "bsw-storage-test-{}-{n}-{tag}.bin",
        std::process::id()
    ))
}

/// Removes the backing file on drop.
struct TempFile(PathBuf);

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

fn fresh_backend(tag: &str, files: &mut Vec<TempFile>) -> FileBackend {
    let path = temp_path(tag);
    let backend = FileBackend::create(&path, GEOMETRY).expect("create backing file");
    files.push(TempFile(path));
    backend
}

#[test]
fn backend_contract_file() {
    let mut files = Vec::new();
    let backend = fresh_backend("contract", &mut files);
    assert!(run_backend_contract_suite(backend) > 0);
}

#[test]
fn block_store_suite_file() {
    let mut files = Vec::new();
    assert!(run_block_store_suite(|| fresh_backend("suite", &mut files)) > 0);
}

#[test]
fn cut_point_enumeration_file() {
    let mut files = Vec::new();
    let report = run_cut_point_suite(|| fresh_backend("cut", &mut files));
    assert!(report.total_ops > 30, "script too short: {report:?}");
    assert_eq!(report.replays, 4 * (report.total_ops + 1));
    assert_eq!(report.torn_replays, 3 * (report.total_ops + 1));
    println!("file cut-point report: {report:?}");
}

#[test]
fn reopen_preserves_blocks_across_lifetimes() {
    let path = temp_path("reopen");
    let _cleanup = TempFile(path.clone());

    // Lifetime 1: create, write, invalidate, drop everything.
    let backend = FileBackend::create(&path, GEOMETRY).expect("create");
    let mut store: JournalStore<FileBackend, 8> = JournalStore::mount(backend).expect("mount");
    store
        .write(BlockId(1), 3, b"persistent payload")
        .expect("write 1");
    store.write(BlockId(2), 1, b"second").expect("write 2");
    store
        .write(BlockId(1), 4, b"persistent payload v2")
        .expect("overwrite 1");
    store.invalidate(BlockId(2)).expect("invalidate 2");
    let meta1 = store.metadata(BlockId(1)).expect("meta");
    drop(store);

    // Lifetime 2: reopen the file, validate geometry, mount, verify.
    let backend = FileBackend::open(&path).expect("reopen");
    assert_eq!(backend.geometry(), GEOMETRY, "geometry header must persist");
    let store: JournalStore<FileBackend, 8> = JournalStore::mount(backend).expect("re-mount");
    assert_eq!(store.metadata(BlockId(1)), Ok(meta1));
    let mut buf = [0u8; 64];
    assert_eq!(store.read(BlockId(1), &mut buf), Ok(21));
    assert_eq!(&buf[..21], b"persistent payload v2");
    assert!(!store.contains(BlockId(2)), "invalidation must persist");
}

#[test]
fn open_or_create_rejects_geometry_mismatch() {
    let path = temp_path("geom");
    let _cleanup = TempFile(path.clone());
    drop(FileBackend::create(&path, GEOMETRY).expect("create"));
    let other = FileGeometry {
        region_size: 2048,
        erase_unit: 256,
        program_unit: 8,
    };
    assert!(matches!(
        FileBackend::open_or_create(&path, other),
        Err(StorageError::InvalidParameter)
    ));
    // Matching geometry reopens fine.
    assert!(FileBackend::open_or_create(&path, GEOMETRY).is_ok());
}

#[test]
fn open_missing_file_fails_with_io() {
    let path = temp_path("missing");
    assert!(matches!(FileBackend::open(&path), Err(StorageError::Io)));
}

#[test]
fn open_corrupted_header_fails_with_corrupt_data() {
    let path = temp_path("badheader");
    let _cleanup = TempFile(path.clone());
    drop(FileBackend::create(&path, GEOMETRY).expect("create"));
    // Flip a byte inside the persisted geometry header.
    let mut bytes = fs::read(&path).expect("read file");
    bytes[10] ^= 0xFF;
    fs::write(&path, &bytes).expect("rewrite file");
    assert!(matches!(
        FileBackend::open(&path),
        Err(StorageError::CorruptData)
    ));
}

#[test]
fn create_rejects_invalid_geometry() {
    let path = temp_path("badgeom");
    let bad = FileGeometry {
        region_size: 1000,
        erase_unit: 128,
        program_unit: 4,
    };
    assert!(matches!(
        FileBackend::create(&path, bad),
        Err(StorageError::InvalidParameter)
    ));
    assert!(!path.exists() || fs::remove_file(&path).is_ok());
}
