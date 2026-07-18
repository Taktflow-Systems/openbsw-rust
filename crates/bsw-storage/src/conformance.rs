//! Shared conformance suites (std-only test harness).
//!
//! Every suite is generic over `B: StorageBackend` so that
//! [`crate::mem::MemBackend`] and [`crate::file::FileBackend`] run
//! identical checks:
//!
//! - [`run_backend_contract_suite`] — raw flash-semantics contract.
//! - [`run_block_store_suite`] — full [`BlockStore`] behaviour of a
//!   [`JournalStore`] mounted on the backend, including remount
//!   persistence and copy-on-write wear.
//! - [`run_cut_point_suite`] — exhaustive power-cut enumeration: a
//!   scripted write sequence is replayed once per possible cut point *K*
//!   (0 ..= total mutating operations) and per torn-write variant,
//!   asserting after every re-mount that each block reads either its
//!   pre-write or post-write value, never garbage.
//!
//! Suites panic (assert) on contract violations; they are test harnesses,
//! not production code. The suites are geometry-generic but designed for
//! a region of 1024 bytes with a 128-byte erase unit, which makes the
//! scripted sequence cross at least one copy-on-write area flip.

use std::collections::BTreeMap;

use crate::backend::{StorageBackend, StorageError};
use crate::block::{BlockId, BlockStore};
use crate::fault::{CutPlan, CutPointBackend};
use crate::journal::JournalStore;

/// Index capacity used for every store the suites mount.
const SUITE_MAX_BLOCKS: usize = 8;

type SuiteStore<B> = JournalStore<B, SUITE_MAX_BLOCKS>;

// ---------------------------------------------------------------------
// Backend contract suite
// ---------------------------------------------------------------------

/// Verify the raw [`StorageBackend`] flash-semantics contract on a
/// freshly created (fully erased) backend. Returns the number of checks.
pub fn run_backend_contract_suite<B: StorageBackend>(mut backend: B) -> u32 {
    let region = backend.region_size();
    let erase = backend.erase_unit();
    let prog = backend.program_unit();
    assert!(erase.is_power_of_two(), "erase unit must be a power of two");
    assert_eq!(region % erase, 0, "region must be whole erase units");
    assert!(matches!(prog, 1 | 4 | 8), "program unit must be 1, 4, or 8");
    assert_eq!(erase % prog, 0, "erase unit must be whole program units");

    // Fresh backend reads fully erased.
    let mut buf = vec![0u8; erase];
    for unit in 0..region / erase {
        backend
            .read(unit * erase, &mut buf)
            .expect("read erased unit");
        assert!(
            buf.iter().all(|&b| b == 0xFF),
            "fresh backend must be erased"
        );
    }

    // Program / read round trip.
    let data: Vec<u8> = (0..2 * prog).map(|i| i as u8).collect();
    backend.program(0, &data).expect("aligned program");
    let mut readback = vec![0u8; data.len()];
    backend.read(0, &mut readback).expect("read back");
    assert_eq!(readback, data);

    // Alignment and range violations.
    if prog > 1 {
        assert_eq!(
            backend.program(1, &vec![0u8; prog]),
            Err(StorageError::Unaligned)
        );
        assert_eq!(
            backend.program(prog, &[0u8; 1]),
            Err(StorageError::Unaligned)
        );
    }
    assert_eq!(
        backend.program(region, &vec![0u8; prog]),
        Err(StorageError::OutOfRange)
    );
    let mut over = vec![0u8; prog];
    assert_eq!(
        backend.read(region, &mut over),
        Err(StorageError::OutOfRange)
    );
    assert_eq!(backend.erase(region / erase), Err(StorageError::OutOfRange));

    // Program only clears bits: a conflicting reprogram fails and stores
    // the AND (detectable corruption).
    let zeros = vec![0x00u8; prog];
    backend.program(2 * prog, &zeros).expect("program zeros");
    let ones = vec![0xFFu8; prog];
    assert_eq!(
        backend.program(2 * prog, &ones),
        Err(StorageError::NotErased)
    );
    let mut cell = vec![0u8; prog];
    backend.read(2 * prog, &mut cell).expect("read cell");
    assert_eq!(cell, zeros, "conflicting program must store the AND");

    // A reprogram that only clears bits succeeds.
    backend
        .program(3 * prog, &vec![0x0Fu8; prog])
        .expect("first program");
    backend
        .program(3 * prog, &vec![0x0Du8; prog])
        .expect("clearing reprogram");
    backend.read(3 * prog, &mut cell).expect("read cell");
    assert_eq!(cell, vec![0x0Du8; prog]);

    // Erase restores the unit and only the unit.
    backend.program(erase, &data).expect("program second unit");
    backend.erase(1).expect("erase second unit");
    let mut unit_buf = vec![0u8; erase];
    backend.read(erase, &mut unit_buf).expect("read erased");
    assert!(unit_buf.iter().all(|&b| b == 0xFF));
    backend.read(0, &mut readback).expect("read untouched");
    assert_eq!(readback, data, "erase must not touch other units");
    9
}

// ---------------------------------------------------------------------
// Block store suite
// ---------------------------------------------------------------------

/// Run the full [`BlockStore`] conformance suite; `factory` must return a
/// fresh, fully erased backend on every call. Returns the number of
/// sections executed.
pub fn run_block_store_suite<B, F>(mut factory: F) -> u32
where
    B: StorageBackend,
    F: FnMut() -> B,
{
    check_empty_store(&mut factory);
    check_round_trip_and_metadata(&mut factory);
    check_overwrite_and_isolation(&mut factory);
    check_parameter_errors(&mut factory);
    check_invalidate_cycle(&mut factory);
    check_remount_persistence(&mut factory);
    check_copy_on_write_wear(&mut factory);
    check_index_capacity(&mut factory);
    8
}

fn mount_fresh<B: StorageBackend>(factory: &mut impl FnMut() -> B) -> SuiteStore<B> {
    JournalStore::mount(factory()).expect("mount fresh backend")
}

fn check_empty_store<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let id = BlockId(1);
    assert!(!store.contains(id));
    assert_eq!(store.metadata(id), Err(StorageError::UnknownBlock));
    let mut buf = [0u8; 8];
    assert_eq!(store.read(id, &mut buf), Err(StorageError::UnknownBlock));
    assert_eq!(
        store.invalidate(id),
        Ok(()),
        "invalidate of absent block is idempotent"
    );
}

fn check_round_trip_and_metadata<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let id = BlockId(2);
    let data = *b"twelve bytes";
    store.write(id, 7, &data).expect("write");
    assert!(store.contains(id));
    let meta = store.metadata(id).expect("metadata");
    assert_eq!(meta.version, 7);
    assert_eq!(meta.length, data.len());
    assert!(meta.generation > 0);
    let mut buf = [0u8; 32];
    assert_eq!(store.read(id, &mut buf), Ok(data.len()));
    assert_eq!(&buf[..data.len()], &data);

    // Zero-length payloads are valid blocks.
    let empty = BlockId(3);
    store.write(empty, 1, &[]).expect("empty write");
    assert!(store.contains(empty));
    assert_eq!(store.read(empty, &mut buf), Ok(0));
    assert_eq!(store.metadata(empty).expect("metadata").length, 0);
}

fn check_overwrite_and_isolation<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let a = BlockId(1);
    let b = BlockId(2);
    store.write(a, 1, b"alpha-one").expect("write a");
    store.write(b, 1, b"bravo-one").expect("write b");
    let gen_a1 = store.metadata(a).expect("meta").generation;
    store.write(a, 2, b"alpha-two-longer").expect("overwrite a");
    let meta_a2 = store.metadata(a).expect("meta");
    assert_eq!(meta_a2.version, 2);
    assert!(meta_a2.generation > gen_a1, "generation must increase");
    let mut buf = [0u8; 32];
    assert_eq!(store.read(a, &mut buf), Ok(16));
    assert_eq!(&buf[..16], b"alpha-two-longer");
    // The other block is untouched.
    assert_eq!(store.read(b, &mut buf), Ok(9));
    assert_eq!(&buf[..9], b"bravo-one");
}

fn check_parameter_errors<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let id = BlockId(4);
    store.write(id, 1, b"payload!").expect("write");
    let mut small = [0u8; 4];
    assert_eq!(
        store.read(id, &mut small),
        Err(StorageError::InvalidParameter)
    );
    // A payload that cannot fit one area is rejected up front. Large
    // geometries hit the wire-format length limit first, small ones the
    // area capacity; both are valid up-front rejections.
    let oversized = vec![0xAAu8; store.area_size()];
    let rejection = store.write(BlockId(5), 1, &oversized);
    assert!(
        matches!(
            rejection,
            Err(StorageError::CapacityExceeded | StorageError::InvalidParameter)
        ),
        "oversized payload must be rejected up front, got {rejection:?}"
    );
    // The failed write must not have disturbed existing data.
    let mut buf = [0u8; 8];
    assert_eq!(store.read(id, &mut buf), Ok(8));
    assert_eq!(&buf, b"payload!");
}

fn check_invalidate_cycle<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let id = BlockId(6);
    store.write(id, 3, b"to be invalidated").expect("write");
    store.invalidate(id).expect("invalidate");
    assert!(!store.contains(id));
    assert_eq!(store.metadata(id), Err(StorageError::UnknownBlock));
    let mut buf = [0u8; 32];
    assert_eq!(store.read(id, &mut buf), Err(StorageError::UnknownBlock));
    store.invalidate(id).expect("invalidate is idempotent");
    // Invalidation survives a remount.
    let store = remount(store);
    assert!(!store.contains(id));
    // The block can be written again afterwards.
    let mut store = store;
    store.write(id, 4, b"reborn").expect("rewrite");
    assert_eq!(store.read(id, &mut buf), Ok(6));
    assert_eq!(&buf[..6], b"reborn");
}

fn remount<B: StorageBackend>(store: SuiteStore<B>) -> SuiteStore<B> {
    JournalStore::mount(store.into_inner()).expect("remount")
}

fn check_remount_persistence<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    store.write(BlockId(1), 1, b"first block").expect("write 1");
    store
        .write(BlockId(2), 9, b"second block, longer payload")
        .expect("write 2");
    store
        .write(BlockId(1), 2, b"first block v2")
        .expect("overwrite 1");
    let meta1 = store.metadata(BlockId(1)).expect("meta 1");
    let meta2 = store.metadata(BlockId(2)).expect("meta 2");
    let store = remount(store);
    assert_eq!(
        store.metadata(BlockId(1)),
        Ok(meta1),
        "metadata must survive remount"
    );
    assert_eq!(store.metadata(BlockId(2)), Ok(meta2));
    let mut buf = [0u8; 64];
    assert_eq!(store.read(BlockId(1), &mut buf), Ok(14));
    assert_eq!(&buf[..14], b"first block v2");
    assert_eq!(store.read(BlockId(2), &mut buf), Ok(28));
    assert_eq!(&buf[..28], b"second block, longer payload");
}

fn check_copy_on_write_wear<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    let payload = [0x5Au8; 40];
    // Rewrite until the append log provably wrapped at least once. Scale
    // the round count with the area size so the suite works for any
    // geometry: each round appends well under 256 bytes of records.
    let rounds = (store.area_size() / 64 + 8).min(u32::MAX as usize) as u32;
    let mut last_round = 0u8;
    for round in 0..rounds {
        #[allow(clippy::cast_possible_truncation)]
        let marker = round as u8;
        last_round = marker;
        let mut data = payload;
        data[0] = marker;
        #[allow(clippy::cast_possible_truncation)]
        let version = round as u16;
        store.write(BlockId(1), version, &data).expect("rewrite");
        store
            .write(BlockId(2), version, &data[..20])
            .expect("rewrite 2");
        if store.wear().area_erase_counts[0] + store.wear().area_erase_counts[1] > 0 {
            break;
        }
    }
    let mut buf = [0u8; 64];
    assert_eq!(store.read(BlockId(1), &mut buf), Ok(40));
    assert_eq!(buf[0], last_round);
    assert_eq!(store.read(BlockId(2), &mut buf), Ok(20));
    let wear = store.wear();
    assert!(
        wear.area_erase_counts[0] + wear.area_erase_counts[1] > 0,
        "sustained rewrites must have flipped areas"
    );
    // Data still valid after another remount.
    let store = remount(store);
    assert_eq!(store.read(BlockId(1), &mut buf), Ok(40));
    assert_eq!(buf[0], last_round);
}

fn check_index_capacity<B: StorageBackend>(factory: &mut impl FnMut() -> B) {
    let mut store = mount_fresh(factory);
    for raw in 0..SUITE_MAX_BLOCKS as u16 {
        store
            .write(BlockId(raw), 1, &raw.to_le_bytes())
            .expect("fill index");
    }
    assert_eq!(
        store.write(BlockId(100), 1, b"extra"),
        Err(StorageError::CapacityExceeded),
        "index overflow must be rejected"
    );
    // Existing blocks can still be rewritten.
    store
        .write(BlockId(0), 2, b"rewrite")
        .expect("rewrite indexed block");
}

// ---------------------------------------------------------------------
// Cut-point enumeration suite
// ---------------------------------------------------------------------

/// Outcome of [`run_cut_point_suite`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutPointReport {
    /// Mutating operations of the baseline (uncut) scripted sequence.
    pub total_ops: u64,
    /// Replays executed (every cut point times every variant).
    pub replays: u64,
    /// How many of those replays used a torn-write variant.
    pub torn_replays: u64,
}

/// Torn-write variants exercised at every cut point: a clean cut, a
/// 1-byte prefix, a 3-byte prefix, and a fully written but unacknowledged
/// program.
const TORN_VARIANTS: [Option<usize>; 4] = [None, Some(1), Some(3), Some(usize::MAX)];

#[derive(Debug, Clone, Copy)]
enum Step {
    Write {
        id: u16,
        version: u16,
        seed: u8,
        len: usize,
    },
    Invalidate {
        id: u16,
    },
}

/// Scripted sequence: overwrites, invalidations, and enough volume to
/// cross a copy-on-write area flip on the reference geometry.
const SCRIPT: [Step; 12] = [
    Step::Write {
        id: 1,
        version: 1,
        seed: 0x11,
        len: 40,
    },
    Step::Write {
        id: 2,
        version: 1,
        seed: 0x22,
        len: 40,
    },
    Step::Write {
        id: 3,
        version: 1,
        seed: 0x33,
        len: 20,
    },
    Step::Write {
        id: 1,
        version: 2,
        seed: 0x44,
        len: 40,
    },
    Step::Invalidate { id: 2 },
    Step::Write {
        id: 4,
        version: 1,
        seed: 0x55,
        len: 40,
    },
    Step::Write {
        id: 3,
        version: 2,
        seed: 0x66,
        len: 40,
    },
    Step::Write {
        id: 1,
        version: 3,
        seed: 0x77,
        len: 40,
    },
    Step::Write {
        id: 2,
        version: 2,
        seed: 0x88,
        len: 40,
    },
    Step::Write {
        id: 4,
        version: 2,
        seed: 0x99,
        len: 20,
    },
    Step::Invalidate { id: 3 },
    Step::Write {
        id: 5,
        version: 1,
        seed: 0xAB,
        len: 24,
    },
];

#[derive(Debug, Clone, PartialEq, Eq)]
enum BlockState {
    Absent,
    Present { version: u16, data: Vec<u8> },
}

fn payload(seed: u8, len: usize) -> Vec<u8> {
    (0..len).map(|i| seed ^ (i as u8)).collect()
}

fn step_target(step: Step) -> u16 {
    match step {
        Step::Write { id, .. } | Step::Invalidate { id } => id,
    }
}

fn step_post_state(step: Step) -> BlockState {
    match step {
        Step::Write {
            version, seed, len, ..
        } => BlockState::Present {
            version,
            data: payload(seed, len),
        },
        Step::Invalidate { .. } => BlockState::Absent,
    }
}

fn apply_step<S: BlockStore>(store: &mut S, step: Step) -> Result<(), StorageError> {
    match step {
        Step::Write {
            id,
            version,
            seed,
            len,
        } => store.write(BlockId(id), version, &payload(seed, len)),
        Step::Invalidate { id } => store.invalidate(BlockId(id)),
    }
}

fn observed_state<S: BlockStore>(store: &S, id: u16) -> BlockState {
    if !store.contains(BlockId(id)) {
        return BlockState::Absent;
    }
    let meta = store
        .metadata(BlockId(id))
        .expect("metadata of contained block");
    let mut buf = vec![0u8; meta.length];
    let len = store
        .read(BlockId(id), &mut buf)
        .expect("read of contained block");
    assert_eq!(len, meta.length, "read length must match metadata");
    BlockState::Present {
        version: meta.version,
        data: buf,
    }
}

/// Run the script uncut to measure the total mutating-operation count.
fn baseline_ops<B: StorageBackend>(factory: &mut impl FnMut() -> B) -> u64 {
    let cut = CutPointBackend::new(factory());
    let mut store: SuiteStore<_> = JournalStore::mount(cut).expect("mount baseline");
    for step in SCRIPT {
        apply_step(&mut store, step).expect("baseline step must succeed");
    }
    store.backend().op_count()
}

/// Exhaustively enumerate power-cut points over the scripted sequence.
///
/// `factory` must return a fresh, fully erased backend with identical
/// geometry on every call, so that every replay executes the identical
/// operation sequence. For every `K` in `0 ..= total_ops` and every
/// variant in [`TORN_VARIANTS`], the script runs until the injected cut,
/// the store is dropped, the backend is re-mounted, and every block must
/// read either its last successfully written state or the state of the
/// interrupted step — with valid metadata — never garbage.
pub fn run_cut_point_suite<B, F>(mut factory: F) -> CutPointReport
where
    B: StorageBackend,
    F: FnMut() -> B,
{
    let total_ops = baseline_ops(&mut factory);
    assert!(total_ops > 0, "script must perform mutating operations");
    let mut report = CutPointReport {
        total_ops,
        replays: 0,
        torn_replays: 0,
    };
    for fail_after in 0..=total_ops {
        for torn_prefix in TORN_VARIANTS {
            replay_cut(
                &mut factory,
                CutPlan {
                    fail_after,
                    torn_prefix,
                },
            );
            report.replays += 1;
            if torn_prefix.is_some() {
                report.torn_replays += 1;
            }
        }
    }
    report
}

fn replay_cut<B: StorageBackend>(factory: &mut impl FnMut() -> B, plan: CutPlan) {
    let mut cut = CutPointBackend::new(factory());
    cut.arm(plan);
    let mut store: SuiteStore<_> = JournalStore::mount(cut).expect("mount fresh armed backend");

    // Run the script to (and past) the cut, tracking the last state each
    // block reached through a successful step and the in-flight state of
    // the interrupted step.
    let mut committed: BTreeMap<u16, BlockState> = BTreeMap::new();
    let mut interrupted: Option<(u16, BlockState)> = None;
    for step in SCRIPT {
        match apply_step(&mut store, step) {
            Ok(()) => {
                committed.insert(step_target(step), step_post_state(step));
            }
            Err(_) => {
                if interrupted.is_none() {
                    interrupted = Some((step_target(step), step_post_state(step)));
                }
            }
        }
    }

    // Power comes back: drop the store, disarm, re-mount.
    let mut backend = store.into_inner();
    backend.disarm();
    let store: SuiteStore<_> =
        JournalStore::mount(backend).expect("re-mount after power cut must succeed");

    for id in [1u16, 2, 3, 4, 5] {
        let actual = observed_state(&store, id);
        let old = committed.get(&id).cloned().unwrap_or(BlockState::Absent);
        let acceptable_new =
            matches!(&interrupted, Some((target, new)) if *target == id && *new == actual);
        assert!(
            actual == old || acceptable_new,
            "block {id} after cut {plan:?}: read state is neither pre- nor post-write"
        );
    }

    // The recovered store must remain fully usable.
    let mut store = store;
    store
        .write(BlockId(7), 1, b"post-recovery")
        .expect("write after recovery");
    let mut buf = [0u8; 16];
    assert_eq!(store.read(BlockId(7), &mut buf), Ok(13));
    assert_eq!(&buf[..13], b"post-recovery");
    let store = remount(store);
    assert_eq!(store.read(BlockId(7), &mut buf), Ok(13));
}
