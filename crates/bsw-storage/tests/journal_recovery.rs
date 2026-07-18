//! Journal-level recovery details: CRC rejection at mount, commit-mark
//! semantics, and capacity behaviour.

use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_storage::{BlockId, BlockStore, StorageError};

type Backend = MemBackend<1024, 128, 4>;
type Store = JournalStore<Backend, 8>;

fn mount(backend: Backend) -> Store {
    JournalStore::mount(backend).expect("mount")
}

/// Find the offset of `needle` in the backend image.
fn find(backend: &Backend, needle: &[u8]) -> usize {
    backend
        .raw()
        .windows(needle.len())
        .position(|window| window == needle)
        .expect("payload must be stored somewhere")
}

#[test]
fn corrupted_payload_is_rejected_at_mount() {
    let mut store = mount(Backend::new());
    let payload = *b"integrity-checked payload";
    store.write(BlockId(1), 1, &payload).expect("write");
    let mut backend = store.into_inner();
    // Flip one payload bit behind the store's back (bit rot / torn write).
    let offset = find(&backend, &payload);
    backend.raw_mut()[offset + 3] ^= 0x40;
    let store = mount(backend);
    assert!(
        !store.contains(BlockId(1)),
        "a record failing its payload CRC must be rejected at mount"
    );
}

#[test]
fn corrupted_payload_is_rejected_at_read() {
    let mut store = mount(Backend::new());
    let payload = *b"read-time integrity check";
    store.write(BlockId(1), 1, &payload).expect("write");
    let offset = find(store.backend(), &payload);
    store.backend_mut().raw_mut()[offset + 5] ^= 0x01;
    let mut buf = [0u8; 32];
    assert_eq!(
        store.read(BlockId(1), &mut buf),
        Err(StorageError::CorruptData)
    );
}

#[test]
fn missing_commit_mark_means_uncommitted() {
    let mut store = mount(Backend::new());
    store
        .write(BlockId(1), 1, b"first record")
        .expect("write 1");
    store.write(BlockId(2), 1, b"second rec").expect("write 2");
    let mut backend = store.into_inner();
    // Damage the commit mark of the *second* record: pad "second rec"
    // (10 bytes) to 12 and clear a commit-mark bit right after it.
    let offset = find(&backend, b"second rec");
    backend.raw_mut()[offset + 12] &= 0x0F;
    let store = mount(backend);
    assert!(store.contains(BlockId(1)), "first record is untouched");
    assert!(
        !store.contains(BlockId(2)),
        "a record without a full commit mark must be ignored"
    );
}

#[test]
fn newest_generation_wins_after_rewrites() {
    let mut store = mount(Backend::new());
    for round in 0..5u8 {
        store
            .write(BlockId(3), u16::from(round), &[round; 16])
            .expect("write");
    }
    let store = mount(store.into_inner());
    let meta = store.metadata(BlockId(3)).expect("metadata");
    assert_eq!(meta.version, 4);
    let mut buf = [0u8; 16];
    assert_eq!(store.read(BlockId(3), &mut buf), Ok(16));
    assert_eq!(buf, [4u8; 16]);
}

#[test]
fn payload_longer_than_u16_is_rejected() {
    let big = vec![0u8; usize::from(u16::MAX) + 1];
    let mut store = mount(Backend::new());
    assert_eq!(
        store.write(BlockId(1), 1, &big),
        Err(StorageError::InvalidParameter)
    );
}

#[test]
fn mount_rejects_backend_too_small_for_two_areas() {
    // A single erase unit cannot host an A and a B area.
    let backend = MemBackend::<128, 128, 4>::new();
    assert!(matches!(
        JournalStore::<_, 4>::mount(backend),
        Err(StorageError::InvalidParameter)
    ));
}

#[test]
fn free_space_and_active_area_are_observable() {
    let mut store = mount(Backend::new());
    assert_eq!(store.area_size(), 512);
    let before = store.free_space();
    store.write(BlockId(1), 1, &[0xAA; 20]).expect("write");
    let after = store.free_space();
    assert_eq!(
        before - after,
        24 + 20 + 4,
        "record accounting must be exact"
    );
    assert!(store.active_area() < 2);
}

#[test]
fn tombstones_are_dropped_by_area_flip() {
    let mut store = mount(Backend::new());
    store.write(BlockId(1), 1, &[1u8; 40]).expect("write 1");
    store.invalidate(BlockId(1)).expect("invalidate");
    // Force an area flip; the tombstone is not copied.
    for round in 0..12u8 {
        store
            .write(BlockId(2), u16::from(round), &[round; 40])
            .expect("churn");
    }
    let wear = store.wear();
    assert!(
        wear.area_erase_counts[0] + wear.area_erase_counts[1] > 0,
        "churn must have flipped areas"
    );
    assert!(
        !store.contains(BlockId(1)),
        "invalidation must survive the flip"
    );
    // And it still survives a remount.
    let store = mount(store.into_inner());
    assert!(!store.contains(BlockId(1)));
    assert!(store.contains(BlockId(2)));
}
