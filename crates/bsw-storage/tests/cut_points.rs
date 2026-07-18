//! Exhaustive power-cut enumeration over MemBackend.
//!
//! Every replay arms `CutPointBackend` to cut after exactly K mutating
//! operations (clean and torn variants), runs the scripted sequence, then
//! re-mounts and asserts every block reads its pre-write or post-write
//! value. The suite enumerates every K from 0 to the total operation
//! count — see `bsw_storage::conformance::run_cut_point_suite`.

use bsw_storage::conformance::run_cut_point_suite;
use bsw_storage::mem::MemBackend;

#[test]
fn cut_point_enumeration_mem_prog4() {
    let report = run_cut_point_suite(MemBackend::<1024, 128, 4>::new);
    assert!(report.total_ops > 30, "script too short: {report:?}");
    assert_eq!(report.replays, 4 * (report.total_ops + 1));
    assert_eq!(report.torn_replays, 3 * (report.total_ops + 1));
    println!("mem prog4 cut-point report: {report:?}");
}

#[test]
fn cut_point_enumeration_mem_prog8() {
    let report = run_cut_point_suite(MemBackend::<1024, 128, 8>::new);
    assert!(report.total_ops > 30, "script too short: {report:?}");
    assert_eq!(report.replays, 4 * (report.total_ops + 1));
    assert_eq!(report.torn_replays, 3 * (report.total_ops + 1));
    println!("mem prog8 cut-point report: {report:?}");
}

#[test]
fn cut_point_enumeration_mem_prog1() {
    let report = run_cut_point_suite(MemBackend::<1024, 128, 1>::new);
    assert!(report.total_ops > 30, "script too short: {report:?}");
    assert_eq!(report.replays, 4 * (report.total_ops + 1));
    println!("mem prog1 cut-point report: {report:?}");
}
