//! Destructive, on-target F413/G474 storage conformance runners (G11/G12).
//!
//! The caller must opt in explicitly: this erases only the selected board's
//! linker-reserved storage region.

use bsw_storage::{BlockId, BlockStore, StorageBackend, StorageError};

#[cfg(feature = "stm32f413")]
use crate::flash_f4::{
    F4StorageBackend, STORAGE_BASE_ADDR as F4_STORAGE_BASE, STORAGE_SECTOR_COUNT,
    STORAGE_SECTOR_SIZE, STORAGE_SIZE as F4_STORAGE_SIZE,
};
#[cfg(feature = "stm32g474")]
use crate::storage_g4::G4NvmBackend;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TargetStorageReport {
    pub checks: u8,
    pub area_erases: [u32; 2],
    pub power_cut_replays: u16,
}

// Const evaluation places this directly in ROM; it is never a runtime stack array.
#[allow(clippy::large_stack_arrays)]
const fn cut_fill() -> [u8; 65_000] {
    let mut bytes = [0xFF; 65_000];
    bytes[0] = 0x5A;
    bytes[1] = 0x5A;
    bytes[2] = 0x5A;
    bytes[3] = 0x5A;
    bytes
}

// The leading program unit makes the torn-payload case physically observable;
// the erased-state tail occupies journal space without wasting flash pulses.
static CUT_FILL: [u8; 65_000] = cut_fill();
static CUT_NEW: [u8; 2_048] = [0xa6; 2_048];
const CUT_OLD: &[u8] = b"old-valid-value";
const JOURNAL_COMMIT_BYTE: u8 = 0x33;

#[allow(dead_code)]
struct TargetCutBackend<B: StorageBackend> {
    inner: B,
    fail_after: u32,
    operations: u32,
    torn: bool,
    hit: bool,
}

#[allow(dead_code)]
impl<B: StorageBackend> TargetCutBackend<B> {
    fn new(inner: B, fail_after: u32, torn: bool) -> Self {
        Self {
            inner,
            fail_after,
            operations: 0,
            torn,
            hit: false,
        }
    }
    fn cut(&mut self) -> bool {
        let cut = self.operations == self.fail_after;
        self.operations = self.operations.saturating_add(1);
        if cut {
            self.hit = true;
        }
        cut
    }
    fn into_inner(self) -> B {
        self.inner
    }
}

impl<B: StorageBackend> StorageBackend for TargetCutBackend<B> {
    fn region_size(&self) -> usize {
        self.inner.region_size()
    }
    fn erase_unit(&self) -> usize {
        self.inner.erase_unit()
    }
    fn program_unit(&self) -> usize {
        self.inner.program_unit()
    }
    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        self.inner.read(offset, buf)
    }
    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        if self.cut() {
            Err(StorageError::Io)
        } else {
            self.inner.erase(unit_index)
        }
    }
    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        if !self.cut() {
            return self.inner.program(offset, data);
        }
        if self.torn {
            let prefix = self.inner.program_unit().min(data.len());
            let _ = self.inner.program(offset, &data[..prefix]);
        }
        Err(StorageError::Io)
    }
}

#[allow(dead_code)]
fn run_power_cut_campaign<B, F>(
    mut factory: F,
    erase_units: usize,
    fill_len: usize,
    new_len: usize,
) -> Result<u16, StorageError>
where
    B: StorageBackend,
    F: FnMut() -> B,
{
    fn prepare<B: StorageBackend>(
        mut backend: B,
        erase_units: usize,
        fill_len: usize,
    ) -> Result<bsw_storage::journal::JournalStore<B, 8>, StorageError> {
        for unit in 0..erase_units {
            backend.erase(unit)?;
        }
        let mut store = bsw_storage::journal::JournalStore::mount(backend)?;
        store.write(BlockId(1), 1, CUT_OLD)?;
        store.write(BlockId(2), 1, &CUT_FILL[..fill_len])?;
        store.write(BlockId(3), 1, &CUT_FILL[..fill_len])?;
        Ok(store)
    }

    // Count one complete relocation, then replay representative boundaries from
    // every transition class.  Exhaustive per-operation interruption belongs in
    // the storage crate's host tests; repeating every F4 payload-copy operation
    // on physical flash would add thousands of redundant erase cycles.
    let baseline = prepare(factory(), erase_units, fill_len)?;
    let backend = TargetCutBackend::new(baseline.into_inner(), u32::MAX, false);
    let mut store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    store.write(BlockId(1), 2, &CUT_NEW[..new_len])?;
    let total_operations = store.backend().operations;
    if total_operations < 4 {
        return Err(StorageError::CorruptData);
    }
    let cut_specs = [
        (0, false),                                  // clean relocation-header cut
        (1, true),                                   // torn relocation payload
        (total_operations.saturating_sub(4), false), // clean source-area erase cut
        (total_operations.saturating_sub(1), true),  // torn new-record commit
        (total_operations, false),                   // successful relocation
    ];

    let mut replays = 0u16;
    for (index, &(cut, torn)) in cut_specs.iter().enumerate() {
        if cut_specs[..index]
            .iter()
            .any(|&(previous, _)| previous == cut)
        {
            continue;
        }
        let baseline = prepare(factory(), erase_units, fill_len)?;
        let backend = TargetCutBackend::new(baseline.into_inner(), cut, torn);
        let mut store: bsw_storage::journal::JournalStore<_, 8> =
            bsw_storage::journal::JournalStore::mount(backend)?;
        let write_result = store.write(BlockId(1), 2, &CUT_NEW[..new_len]);
        let hit = store.backend().hit;
        let backend = store.into_inner().into_inner();
        let recovered: bsw_storage::journal::JournalStore<_, 8> =
            bsw_storage::journal::JournalStore::mount(backend)?;
        let mut value = [0u8; 2_048];
        let len = recovered.read(BlockId(1), &mut value)?;
        let old_valid = len == CUT_OLD.len() && &value[..len] == CUT_OLD;
        let new_valid = len == new_len && value[..len] == CUT_NEW[..new_len];
        if !old_valid && !new_valid {
            return Err(StorageError::CorruptData);
        }
        replays = replays.saturating_add(1);
        if hit {
            if write_result.is_ok() {
                return Err(StorageError::CorruptData);
            }
        } else {
            write_result?;
        }
    }
    Ok(replays)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JournalTransition {
    RelocationHeader,
    RelocationPayload,
    RelocationCommit,
    NewRecordHeader,
    NewRecordPayload,
    NewRecordCommit,
    SourceAreaErase,
}

const JOURNAL_TRANSITIONS: [JournalTransition; 7] = [
    JournalTransition::RelocationHeader,
    JournalTransition::RelocationPayload,
    JournalTransition::RelocationCommit,
    JournalTransition::NewRecordHeader,
    JournalTransition::NewRecordPayload,
    JournalTransition::NewRecordCommit,
    JournalTransition::SourceAreaErase,
];

struct ResetCutBackend<B: StorageBackend> {
    inner: B,
    selected: JournalTransition,
    record: Option<bool>,
    payload_seen: bool,
}

impl<B: StorageBackend> ResetCutBackend<B> {
    fn new(inner: B, selected: JournalTransition) -> Self {
        Self {
            inner,
            selected,
            record: None,
            payload_seen: false,
        }
    }

    fn reset_now(&mut self, partial: Option<(usize, &[u8])>) -> ! {
        if let Some((offset, data)) = partial {
            let prefix = self.inner.program_unit().min(data.len());
            if prefix != 0 {
                let _ = self.inner.program(offset, &data[..prefix]);
            }
        }
        cortex_m::asm::dsb();
        cortex_m::peripheral::SCB::sys_reset()
    }
}

impl<B: StorageBackend> StorageBackend for ResetCutBackend<B> {
    fn region_size(&self) -> usize {
        self.inner.region_size()
    }

    fn erase_unit(&self) -> usize {
        self.inner.erase_unit()
    }

    fn program_unit(&self) -> usize {
        self.inner.program_unit()
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        self.inner.read(offset, buf)
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        if self.selected == JournalTransition::SourceAreaErase {
            self.reset_now(None);
        }
        self.inner.erase(unit_index)
    }

    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        let is_header = data.len() == 24 && data[0..4] == 0x4253_574au32.to_le_bytes();
        if is_header {
            let id = u16::from_le_bytes([data[4], data[5]]);
            let version = u16::from_le_bytes([data[6], data[7]]);
            let is_new = id == 1 && version == 2;
            self.record = Some(is_new);
            self.payload_seen = false;
            let transition = if is_new {
                JournalTransition::NewRecordHeader
            } else {
                JournalTransition::RelocationHeader
            };
            if transition == self.selected {
                self.reset_now(Some((offset, data)));
            }
        } else if data.iter().all(|byte| *byte == JOURNAL_COMMIT_BYTE) {
            let transition = if self.record == Some(true) {
                JournalTransition::NewRecordCommit
            } else {
                JournalTransition::RelocationCommit
            };
            if transition == self.selected {
                self.reset_now(None);
            }
            self.record = None;
        } else if !self.payload_seen {
            self.payload_seen = true;
            let transition = if self.record == Some(true) {
                JournalTransition::NewRecordPayload
            } else {
                JournalTransition::RelocationPayload
            };
            if transition == self.selected {
                self.reset_now(Some((offset, data)));
            }
        }
        self.inner.program(offset, data)
    }
}

fn prepare_reset_campaign<B: StorageBackend>(
    mut backend: B,
    erase_units: usize,
    fill_len: usize,
) -> Result<bsw_storage::journal::JournalStore<B, 8>, StorageError> {
    for unit in 0..erase_units {
        backend.erase(unit)?;
    }
    let mut store = bsw_storage::journal::JournalStore::mount(backend)?;
    store.write(BlockId(1), 1, CUT_OLD)?;
    store.write(BlockId(2), 1, &CUT_FILL[..fill_len])?;
    store.write(BlockId(3), 1, &CUT_FILL[..fill_len])?;
    Ok(store)
}

fn validate_reset_recovery<B: StorageBackend>(
    backend: B,
    new_len: usize,
) -> Result<(), StorageError> {
    let recovered: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    let mut value = [0u8; 2_048];
    let len = recovered.read(BlockId(1), &mut value)?;
    let old_valid = len == CUT_OLD.len() && &value[..len] == CUT_OLD;
    let new_valid = len == new_len && value[..len] == CUT_NEW[..new_len];
    if old_valid || new_valid {
        Ok(())
    } else {
        Err(StorageError::CorruptData)
    }
}

fn run_reset_safe_campaign<B, F>(
    mut factory: F,
    board: u8,
    erase_units: usize,
    fill_len: usize,
    new_len: usize,
    wear: [u32; 2],
) -> Result<u16, StorageError>
where
    B: StorageBackend,
    F: FnMut() -> B,
{
    let retained = crate::fault::retained_storage_campaign();
    let next_cut = if let Some(state) = retained.filter(|state| state.board == board) {
        if !state.pending_recovery {
            return Err(StorageError::CorruptData);
        }
        validate_reset_recovery(factory(), new_len)?;
        usize::from(state.cut_index) + 1
    } else {
        0
    };

    if next_cut < JOURNAL_TRANSITIONS.len() {
        let baseline = prepare_reset_campaign(factory(), erase_units, fill_len)?;
        let backend = ResetCutBackend::new(baseline.into_inner(), JOURNAL_TRANSITIONS[next_cut]);
        crate::fault::write_retained_storage_campaign(crate::fault::RetainedStorageCampaign {
            board,
            cut_index: next_cut as u8,
            pending_recovery: true,
            wear: [
                wear[0].min(u32::from(u16::MAX)) as u16,
                wear[1].min(u32::from(u16::MAX)) as u16,
            ],
        });
        let mut store: bsw_storage::journal::JournalStore<_, 8> =
            bsw_storage::journal::JournalStore::mount(backend)?;
        let result = store.write(BlockId(1), 2, &CUT_NEW[..new_len]);
        crate::fault::clear_retained_storage_campaign();
        result?;
        return Err(StorageError::CorruptData);
    }

    let mut store = prepare_reset_campaign(factory(), erase_units, fill_len)?;
    store.write(BlockId(1), 2, &CUT_NEW[..new_len])?;
    validate_reset_recovery(store.into_inner(), new_len)?;
    crate::fault::clear_retained_storage_campaign();
    Ok(JOURNAL_TRANSITIONS.len() as u16)
}

unsafe extern "C" {
    static _storage_start: u8;
    static _storage_end: u8;
}

fn resolved_storage_bounds() -> (usize, usize) {
    (
        core::ptr::addr_of!(_storage_start) as usize,
        core::ptr::addr_of!(_storage_end) as usize,
    )
}

/// Run geometry, erase/program, conflict, recovery, remount, and wear checks.
///
/// # Safety
/// No interrupt handler or other owner may access flash while this runs.
/// All previous data in the reserved storage region is destroyed.
#[cfg(feature = "stm32g474")]
pub unsafe fn run_g4_target_conformance(
    _token: crate::board::Flash,
) -> Result<TargetStorageReport, StorageError> {
    let (start, end) = resolved_storage_bounds();
    if start != crate::flash_g4::NVM_BASE_ADDR as usize
        || end != (crate::flash_g4::NVM_BASE_ADDR + crate::flash_g4::NVM_SIZE) as usize
    {
        return Err(StorageError::InvalidParameter);
    }
    if !G4NvmBackend::dual_bank_geometry_enabled() {
        return Err(StorageError::InvalidParameter);
    }
    // SAFETY: propagated caller exclusivity.
    let mut backend = unsafe { G4NvmBackend::new() };
    if backend.region_size() != 8 * 1024
        || backend.erase_unit() != 2 * 1024
        || backend.program_unit() != 8
    {
        return Err(StorageError::InvalidParameter);
    }
    if let Some(state) = crate::fault::retained_storage_campaign().filter(|state| state.board == 2)
    {
        let wear = [u32::from(state.wear[0]), u32::from(state.wear[1])];
        let power_cut_replays =
            run_reset_safe_campaign(|| unsafe { G4NvmBackend::new() }, 2, 4, 1_650, 680, wear)?;
        return Ok(TargetStorageReport {
            checks: 9,
            area_erases: wear,
            power_cut_replays,
        });
    }

    for unit in 0..4 {
        backend.erase(unit)?;
    }
    let mut erased = [0u8; 32];
    for offset in (0..backend.region_size()).step_by(erased.len()) {
        backend.read(offset, &mut erased)?;
        if erased.iter().any(|&byte| byte != 0xFF) {
            return Err(StorageError::CorruptData);
        }
    }
    let pattern = *b"G11TEST!";
    backend.program(0, &pattern)?;
    let mut readback = [0u8; 8];
    backend.read(0, &mut readback)?;
    if readback != pattern {
        return Err(StorageError::CorruptData);
    }
    if backend.program(0, &[0xFF; 8]) != Err(StorageError::NotErased) {
        return Err(StorageError::CorruptData);
    }

    for unit in 0..4 {
        backend.erase(unit)?;
    }
    // An uncommitted/invalid record models loss of power before the commit mark.
    backend.program(0, &[0; 24])?;
    let mut store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    if store.contains(BlockId(1)) {
        return Err(StorageError::CorruptData);
    }
    store.write(BlockId(1), 1, b"persistent-g474")?;
    let backend = store.into_inner();
    let mut store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    let mut payload = [0u8; 32];
    if store.read(BlockId(1), &mut payload)? != 15 || &payload[..15] != b"persistent-g474" {
        return Err(StorageError::CorruptData);
    }
    for version in 2..=96u16 {
        let mut next = [0xA5; 48];
        next[0..2].copy_from_slice(&version.to_le_bytes());
        store.write(BlockId(1), version, &next)?;
    }
    let wear = store.wear().area_erase_counts;
    if wear[0] == 0 && wear[1] == 0 {
        return Err(StorageError::CorruptData);
    }
    let backend = store.into_inner();
    let store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    if store.metadata(BlockId(1))?.version != 96 {
        return Err(StorageError::CorruptData);
    }
    let power_cut_replays =
        run_reset_safe_campaign(|| unsafe { G4NvmBackend::new() }, 2, 4, 1_650, 680, wear)?;
    Ok(TargetStorageReport {
        checks: 9,
        area_erases: wear,
        power_cut_replays,
    })
}

/// Run the F413 backend conformance only inside linker-reserved sectors 14-15.
///
/// # Safety
/// No interrupt handler or other owner may access flash while this runs.
/// All previous data in the resolved linker storage region is destroyed.
#[cfg(feature = "stm32f413")]
pub unsafe fn run_f4_target_conformance(
    _token: crate::board::Flash,
) -> Result<TargetStorageReport, StorageError> {
    let (start, end) = resolved_storage_bounds();
    if start != F4_STORAGE_BASE as usize || end != (F4_STORAGE_BASE + F4_STORAGE_SIZE) as usize {
        return Err(StorageError::InvalidParameter);
    }
    // SAFETY: propagated caller exclusivity and verified linker bounds.
    let mut backend = unsafe { F4StorageBackend::new() };
    if backend.region_size() != F4_STORAGE_SIZE as usize
        || backend.erase_unit() != STORAGE_SECTOR_SIZE as usize
        || backend.program_unit() != 4
    {
        return Err(StorageError::InvalidParameter);
    }
    if let Some(state) = crate::fault::retained_storage_campaign().filter(|state| state.board == 1)
    {
        let wear = [u32::from(state.wear[0]), u32::from(state.wear[1])];
        let power_cut_replays = run_reset_safe_campaign(
            || unsafe { F4StorageBackend::new() },
            1,
            STORAGE_SECTOR_COUNT as usize,
            64_500,
            1_984,
            wear,
        )?;
        return Ok(TargetStorageReport {
            checks: 9,
            area_erases: wear,
            power_cut_replays,
        });
    }

    for unit in 0..STORAGE_SECTOR_COUNT as usize {
        backend.erase(unit)?;
    }
    let mut erased = [0u8; 32];
    for offset in (0..backend.region_size()).step_by(erased.len()) {
        backend.read(offset, &mut erased)?;
        if erased.iter().any(|&byte| byte != 0xFF) {
            return Err(StorageError::CorruptData);
        }
    }
    let pattern = *b"G12!";
    backend.program(0, &pattern)?;
    let mut readback = [0u8; 4];
    backend.read(0, &mut readback)?;
    if readback != pattern {
        return Err(StorageError::CorruptData);
    }
    if backend.program(0, &[0xFF; 4]) != Err(StorageError::NotErased) {
        return Err(StorageError::CorruptData);
    }

    for unit in 0..STORAGE_SECTOR_COUNT as usize {
        backend.erase(unit)?;
    }
    // An invalid record models loss of power before the commit mark.
    backend.program(0, &[0; 24])?;
    let mut store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    if store.contains(BlockId(1)) {
        return Err(StorageError::CorruptData);
    }
    store.write(BlockId(1), 1, b"persistent-f413")?;
    let backend = store.into_inner();
    let mut store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    let mut payload = [0u8; 32];
    if store.read(BlockId(1), &mut payload)? != 15 || &payload[..15] != b"persistent-f413" {
        return Err(StorageError::CorruptData);
    }

    // Fill one 128 KiB area to force a copy-on-write relocation and erase.
    let mut next = [0xA5; 4_092];
    let mut version = 2u16;
    while store.wear().area_erase_counts == [0, 0] && version < 96 {
        next[0..2].copy_from_slice(&version.to_le_bytes());
        store.write(BlockId(1), version, &next)?;
        version += 1;
    }
    let wear = store.wear().area_erase_counts;
    if wear == [0, 0] {
        return Err(StorageError::CorruptData);
    }
    let expected_version = version - 1;
    let backend = store.into_inner();
    let store: bsw_storage::journal::JournalStore<_, 8> =
        bsw_storage::journal::JournalStore::mount(backend)?;
    if store.metadata(BlockId(1))?.version != expected_version {
        return Err(StorageError::CorruptData);
    }
    let power_cut_replays = run_reset_safe_campaign(
        || unsafe { F4StorageBackend::new() },
        1,
        STORAGE_SECTOR_COUNT as usize,
        64_500,
        1_984,
        wear,
    )?;
    Ok(TargetStorageReport {
        checks: 9,
        area_erases: wear,
        power_cut_replays,
    })
}
