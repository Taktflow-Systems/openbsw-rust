//! Drift-derived reference-application workflow tests (U04 follow-up,
//! 2026-07-19).
//!
//! Derived from upstream drift commits 366d993e (Keep lwIP netif persistent
//! across lifecycle transitions), b119bf9b (Fix Tap interface restart failure
//! during lifecycle transition on POSIX), and 7a3c3f3f (DemoSystem shutdown
//! cancels the cyclic timeout before subsystem teardown), all observed at
//! drift tip be0029b.
//!
//! These tests are NOT pinned-oracle (ddbcf88a) evidence. They assert that
//! the Rust composition already exhibits the drift-tip observable behavior
//! through its native host-socket and pull-based demo design; see
//! docs/port/composition-drift-review-2026-07-19.md.

use bsw_time::Instant;
use openbsw_reference_app::network::{tcp_echo_once, udp_echo_once};
use openbsw_reference_app::{AppConfig, ReferenceApp};

/// Upstream fixed netif/TAP teardown so networking keeps working across
/// lifecycle transitions and restarts (366d993e, b119bf9b). The Rust port
/// uses host loopback sockets (a documented platform difference in
/// docs/port/reference-app-parity.md), so the equivalent observable is:
/// UDP/TCP echo works while running, after a console reboot, and after a full
/// shutdown/start cycle.
#[test]
fn drift_366d993e_network_echo_survives_reboot_and_full_restart() {
    let mut app = ReferenceApp::new(AppConfig::default()).unwrap();
    app.start(Instant::from_nanos(0)).unwrap();
    assert_eq!(app.level(), 9);
    assert_eq!(udp_echo_once(b"up-1").unwrap(), b"up-1");
    assert_eq!(tcp_echo_once(b"tp-1").unwrap(), b"tp-1");

    // Console-driven reboot: shutdown to level 0 and back to level 9.
    assert_eq!(app.command("lc reboot", Instant::from_nanos(1)), "rebooted");
    assert_eq!(app.level(), 9);
    assert_eq!(udp_echo_once(b"up-2").unwrap(), b"up-2");
    assert_eq!(tcp_echo_once(b"tp-2").unwrap(), b"tp-2");

    // Full application shutdown and restart.
    app.shutdown(Instant::from_nanos(2)).unwrap();
    assert_eq!(app.level(), 0);
    app.start(Instant::from_nanos(3)).unwrap();
    assert_eq!(app.level(), 9);
    assert_eq!(udp_echo_once(b"up-3").unwrap(), b"up-3");
    assert_eq!(tcp_echo_once(b"tp-3").unwrap(), b"tp-3");
}

/// Upstream 7a3c3f3f moves `_timeout.cancel()` to the start of
/// `DemoSystem::shutdown`, so no cyclic demo/diagnostic activity trails into
/// subsystem teardown. The Rust demo is pull-based on injected time, so the
/// equivalent observable is: shutdown itself performs no diagnostic dispatch
/// and preserves the shared diagnostic state for the next start.
#[test]
fn drift_7a3c3f3f_shutdown_is_quiescent_and_preserves_diagnostic_state() {
    let mut app = ReferenceApp::new(AppConfig::default()).unwrap();
    app.start(Instant::from_nanos(0)).unwrap();
    let response = app.command("diag 3e00", Instant::from_nanos(1));
    assert_eq!(response, "7e00");
    let dispatches_before = app.diagnostics_mut().dispatch_count();
    assert_eq!(dispatches_before, 1);

    app.shutdown(Instant::from_nanos(2)).unwrap();
    assert_eq!(app.level(), 0);
    // Shutdown must not have generated trailing diagnostic dispatches.
    assert_eq!(app.diagnostics_mut().dispatch_count(), dispatches_before);

    // Restart keeps the shared dispatch counter (one DiagnosticCore owned by
    // the application, per the parity table) and stays fully operational.
    app.start(Instant::from_nanos(3)).unwrap();
    assert_eq!(app.diagnostics_mut().dispatch_count(), dispatches_before);
    assert_eq!(app.command("diag 3e00", Instant::from_nanos(4)), "7e00");
    assert_eq!(
        app.diagnostics_mut().dispatch_count(),
        dispatches_before + 1
    );
}
