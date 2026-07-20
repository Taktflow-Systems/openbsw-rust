//! Drift-derived shared-composition workflow tests (U04 follow-up,
//! 2026-07-19).
//!
//! Derived from upstream drift commits 9d3e89a2 (Add TimestampProvider: one
//! central time source feeds LifecycleManager timestamps and the DemoSystem
//! cadence) and 7a3c3f3f (DemoSystem shutdown cancels the cyclic timeout
//! first), observed at drift tip be0029b.
//!
//! These tests are NOT pinned-oracle (ddbcf88a) evidence. The Rust shared
//! composition injects `bsw_time::Instant` everywhere, which is the port's
//! native equivalent of the tip's centralized TimestampProvider; see
//! docs/port/composition-drift-review-2026-07-19.md.

use bsw_reference_core::{DiagnosticTransport, ProductionComposition};
use bsw_time::Instant;

/// The drift tip routes all lifecycle and demo timestamps through one
/// provider (9d3e89a2) and stops cyclic demo activity before teardown
/// (7a3c3f3f). Port equivalent: the composition observes only injected time,
/// and shutdown mutates nothing besides the running flag and the shutdown
/// timestamp - no trailing I/O or diagnostic activity.
#[test]
fn drift_9d3e89a2_shutdown_uses_injected_time_only_and_is_quiescent() {
    let mut core = ProductionComposition::new();
    core.start(Instant::from_nanos(10));
    assert!(core.is_running());

    let snapshot = core.cycle_io(2_048, Instant::from_nanos(20)).unwrap();
    assert_eq!(snapshot.pwm_permille, 500);
    core.diagnostics_mut().dispatch_at(
        DiagnosticTransport::DoCan,
        &[0x3e, 0x00],
        Instant::from_nanos(30),
    );
    let dispatches = core.diagnostics().dispatch_count();
    let io_before = core.io_snapshot();

    core.shutdown(Instant::from_nanos(40));
    assert!(!core.is_running());
    // Timestamp comes from the injected clock, not any ambient time source.
    assert_eq!(core.last_cycle(), Instant::from_nanos(40));
    // No trailing diagnostic dispatch or I/O mutation during shutdown.
    assert_eq!(core.diagnostics().dispatch_count(), dispatches);
    assert_eq!(core.io_snapshot(), io_before);

    // Restart keeps deterministic injected-time behavior.
    core.start(Instant::from_nanos(50));
    assert!(core.is_running());
    assert_eq!(core.last_cycle(), Instant::from_nanos(50));
}
