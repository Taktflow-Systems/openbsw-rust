//! Host conformance runs with the exact STM32 storage geometries
//! (packages D09/D10).
//!
//! The board backends (`bsw-bsp-stm32::storage_g4::G4NvmBackend` and
//! `bsw-bsp-stm32::flash_f4::F4StorageBackend`) expose these geometries;
//! running the full block-store and cut-point suites against in-memory
//! backends with identical geometry proves the journal layer's layout,
//! recovery, and wear behavior for both boards without hardware.

#![cfg(feature = "std")]

use bsw_storage::conformance::{run_block_store_suite, run_cut_point_suite};
use bsw_storage::mem::MemBackend;

/// STM32G474 NVM region: 8 KiB, 2 KiB pages, 8-byte double-word programs.
type G4Geometry = MemBackend<8192, 2048, 8>;

/// STM32F413 reserved sectors 14/15: 2 x 128 KiB, 4-byte word programs.
type F4Geometry = MemBackend<262_144, 131_072, 4>;

#[test]
fn g4_nvm_geometry_passes_block_store_suite() {
    let sections = run_block_store_suite(G4Geometry::new);
    assert_eq!(sections, 8);
}

#[test]
fn g4_nvm_geometry_survives_every_cut_point() {
    let report = run_cut_point_suite(G4Geometry::new);
    assert!(report.total_ops > 0);
    assert!(report.replays > report.total_ops);
}

#[test]
fn f4_sector_geometry_passes_block_store_suite() {
    // 256 KiB backends move by value through the suite; give the test a
    // dedicated stack with room to spare.
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(|| {
            let sections = run_block_store_suite(F4Geometry::new);
            assert_eq!(sections, 8);
        })
        .expect("spawn")
        .join()
        .expect("join");
}

#[test]
fn f4_sector_geometry_survives_every_cut_point() {
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(|| {
            let report = run_cut_point_suite(F4Geometry::new);
            assert!(report.total_ops > 0);
            assert!(report.replays > report.total_ops);
        })
        .expect("spawn")
        .join()
        .expect("join");
}
