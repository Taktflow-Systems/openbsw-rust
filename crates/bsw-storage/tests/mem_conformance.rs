//! MemBackend conformance: backend contract, BlockStore suite, wear.

use bsw_storage::conformance::{run_backend_contract_suite, run_block_store_suite};
use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_storage::{BlockId, BlockStore, StorageBackend};

type RefBackend = MemBackend<1024, 128, 4>;
type WideBackend = MemBackend<1024, 128, 8>;
type ByteBackend = MemBackend<1024, 128, 1>;

#[test]
fn backend_contract_prog4() {
    assert!(run_backend_contract_suite(RefBackend::new()) > 0);
}

#[test]
fn backend_contract_prog8() {
    assert!(run_backend_contract_suite(WideBackend::new()) > 0);
}

#[test]
fn backend_contract_prog1() {
    assert!(run_backend_contract_suite(ByteBackend::new()) > 0);
}

#[test]
fn block_store_suite_prog4() {
    assert!(run_block_store_suite(RefBackend::new) > 0);
}

#[test]
fn block_store_suite_prog8() {
    assert!(run_block_store_suite(WideBackend::new) > 0);
}

#[test]
fn block_store_suite_prog1() {
    assert!(run_block_store_suite(ByteBackend::new) > 0);
}

#[test]
fn journal_wear_matches_backend_erase_counters() {
    let mut store: JournalStore<RefBackend, 8> =
        JournalStore::mount(RefBackend::new()).expect("mount");
    // Rewrite until both areas have been erased at least once.
    for round in 0..40u8 {
        store
            .write(BlockId(1), u16::from(round), &[round; 48])
            .expect("rewrite");
    }
    let wear = store.wear();
    assert!(
        wear.area_erase_counts[0] > 0,
        "area A must have been reclaimed"
    );
    assert!(
        wear.area_erase_counts[1] > 0,
        "area B must have been reclaimed"
    );
    let backend = store.into_inner();
    let units_per_area = backend.region_size() / backend.erase_unit() / 2;
    let counts = backend.erase_counts();
    let area_a: u32 = counts[..units_per_area].iter().sum();
    let area_b: u32 = counts[units_per_area..].iter().sum();
    assert_eq!(area_a, wear.area_erase_counts[0]);
    assert_eq!(area_b, wear.area_erase_counts[1]);
}
