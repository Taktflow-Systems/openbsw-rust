//! Destructive, on-target G474 storage conformance runner (G11).
//!
//! The caller must opt in explicitly: this erases the reserved 8 KiB region.

use bsw_storage::{BlockId, BlockStore, StorageBackend, StorageError};

use crate::storage_g4::G4NvmBackend;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TargetStorageReport {
    pub checks: u8,
    pub area_erases: [u32; 2],
}

/// Run geometry, erase/program, conflict, recovery, remount, and wear checks.
///
/// # Safety
/// No interrupt handler or other owner may access flash while this runs.
/// All previous data in the reserved storage region is destroyed.
pub unsafe fn run_g4_target_conformance(
    _token: crate::board::Flash,
) -> Result<TargetStorageReport, StorageError> {
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
    Ok(TargetStorageReport {
        checks: 8,
        area_erases: wear,
    })
}
