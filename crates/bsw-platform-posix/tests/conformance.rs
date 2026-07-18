//! Shared conformance suites for the six platform contracts (F01 done
//! condition).
//!
//! Each `*_contract` function is generic over the trait only and encodes
//! the exact observable behavior a conforming implementation must have.
//! Every suite runs twice in this file — once against the deterministic
//! `bsw_platform::mock` types and once against the POSIX implementations —
//! proving the adapters share conformance with the mocks. Implementation-
//! specific observability (request counters, time advancement) is injected
//! through closures so the contract body stays identical.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;

use bsw_platform_posix::clock::{Clock, Duration, Instant};
use bsw_platform_posix::mock::{
    MockCriticalSection, MockEntropy, MockPlatformInfo, MockReset, MockUniqueId, MockWatchdog,
};
use bsw_platform_posix::test_control::SharedClock;
use bsw_platform_posix::{
    with_critical, CriticalSection, Entropy, FixedEntropy, PlatformInfo, PosixCriticalSection,
    PosixEntropy, PosixPlatformInfo, PosixResetControl, PosixUniqueId, ResetControl, ResetKind,
    ResetReason, SoftWatchdog, UniqueId, Watchdog, WatchdogError, UNIQUE_ID_MAX,
};

const MS: Duration = Duration::from_nanos(1_000_000);

/// Contract: the latched reason is reported until cleared (then
/// `Unknown`), and every software-reset request is observable.
fn reset_contract<R: ResetControl>(reset: &mut R, observed_requests: impl Fn(&R) -> u32) {
    assert_eq!(reset.reset_reason(), ResetReason::PowerOn);
    assert_eq!(observed_requests(reset), 0);
    reset.request_reset();
    reset.request_reset();
    assert_eq!(observed_requests(reset), 2);
    // Requesting a reset must not disturb the latched reason on hosts.
    assert_eq!(reset.reset_reason(), ResetReason::PowerOn);
    reset.clear_reset_reason();
    assert_eq!(reset.reset_reason(), ResetReason::Unknown);
}

/// Contract for a watchdog accepting timeouts in 1 ms..=100 ms: range
/// validation, start-once semantics, service deferring expiry, and expiry
/// at *exactly* the deadline instant (the mock's `>=` boundary semantic).
fn watchdog_contract<W: Watchdog>(
    watchdog: &mut W,
    mut advance: impl FnMut(&mut W, Duration),
    expired: impl Fn(&W) -> bool,
) {
    assert!(!watchdog.is_running());
    assert_eq!(
        watchdog.start(Duration::from_nanos(10)),
        Err(WatchdogError::UnsupportedTimeout)
    );
    assert_eq!(
        watchdog.start(Duration::from_nanos(200_000_000)),
        Err(WatchdogError::UnsupportedTimeout)
    );
    assert!(!watchdog.is_running());
    watchdog.start(Duration::from_millis(10).unwrap()).unwrap();
    assert!(watchdog.is_running());
    assert_eq!(watchdog.start(MS), Err(WatchdogError::AlreadyRunning));
    advance(watchdog, Duration::from_millis(5).unwrap());
    watchdog.service();
    advance(watchdog, Duration::from_millis(9).unwrap());
    assert!(!expired(watchdog), "9 ms after service must not expire");
    advance(watchdog, MS);
    assert!(expired(watchdog), "expiry at exactly the 10 ms deadline");
    watchdog.service();
    assert!(!expired(watchdog), "service must defer expiry");
    assert!(watchdog.is_running());
}

/// Contract: acquires nest, releases unwind one level each, and
/// `with_critical` leaves the section balanced.
fn critical_section_contract<C: CriticalSection>(section: &mut C, depth: impl Fn(&C) -> u32) {
    assert_eq!(depth(section), 0);
    let value = with_critical(section, || 41) + 1;
    assert_eq!(value, 42);
    assert_eq!(depth(section), 0);
    section.acquire();
    section.acquire();
    section.acquire();
    assert_eq!(depth(section), 3);
    section.release();
    assert_eq!(depth(section), 2);
    section.release();
    section.release();
    assert_eq!(depth(section), 0);
}

/// Contract: `fill` succeeds for arbitrary buffer lengths, produces
/// non-zero data, and successive fills differ (statistical smoke for the
/// random sources, exact for the deterministic ones).
fn entropy_contract<E: Entropy>(entropy: &mut E) {
    let mut first = [0_u8; 32];
    entropy.fill(&mut first).unwrap();
    assert_ne!(first, [0_u8; 32]);
    let mut second = [0_u8; 32];
    entropy.fill(&mut second).unwrap();
    assert_ne!(first, second);
    let mut odd = [0_u8; 13];
    entropy.fill(&mut odd).unwrap();
    assert_ne!(odd, [0_u8; 13]);
}

/// Contract: the id fits [`UNIQUE_ID_MAX`], is non-empty, and is stable
/// across calls.
fn unique_id_contract<U: UniqueId>(identity: &U, expected_length: usize) {
    let mut first = [0_u8; UNIQUE_ID_MAX];
    let first_length = identity.unique_id(&mut first);
    assert_eq!(first_length, expected_length);
    assert!(first_length > 0);
    assert!(first_length <= UNIQUE_ID_MAX);
    let mut second = [0_u8; UNIQUE_ID_MAX];
    assert_eq!(identity.unique_id(&mut second), first_length);
    assert_eq!(first, second);
}

/// Contract: a stable, non-empty platform name and the documented host
/// frequency.
fn platform_info_contract<P: PlatformInfo>(info: &P, expected_name: &str, expected_hz: u32) {
    assert!(!info.platform_name().is_empty());
    assert_eq!(info.platform_name(), expected_name);
    assert_eq!(info.cpu_frequency_hz(), expected_hz);
}

// --- ResetControl -------------------------------------------------------

#[test]
fn reset_conformance_mock() {
    let mut reset = MockReset::new(ResetReason::PowerOn);
    reset_contract(&mut reset, MockReset::requests);
}

#[test]
fn reset_conformance_posix() {
    let mut reset = PosixResetControl::new();
    reset_contract(&mut reset, PosixResetControl::requests);
    // POSIX extra: the requests recorded by the contract are drainable in
    // FIFO order — the reset simulation's test control.
    assert_eq!(reset.pending(), 2);
    assert_eq!(reset.take_requested(), Some(ResetKind::Software));
    assert_eq!(reset.take_requested(), Some(ResetKind::Software));
    assert_eq!(reset.take_requested(), None);
}

// --- Watchdog -----------------------------------------------------------

#[test]
fn watchdog_conformance_mock() {
    let mut watchdog = MockWatchdog::new(MS, Duration::from_nanos(100_000_000));
    let mut now = Instant::from_nanos(0);
    watchdog_contract(
        &mut watchdog,
        |watchdog, delta| {
            now = now.wrapping_add(delta);
            watchdog.advance_to(now);
        },
        MockWatchdog::expired,
    );
}

#[test]
fn watchdog_conformance_posix() {
    let clock = SharedClock::new(Instant::from_nanos(0));
    let mut watchdog = SoftWatchdog::new(clock.clone(), MS, Duration::from_nanos(100_000_000));
    watchdog_contract(
        &mut watchdog,
        |_watchdog, delta| clock.advance(delta),
        |watchdog| watchdog.expired(clock.now()),
    );
}

// --- CriticalSection ----------------------------------------------------

#[test]
fn critical_section_conformance_mock() {
    let mut section = MockCriticalSection::new();
    critical_section_contract(&mut section, MockCriticalSection::depth);
    assert_eq!(section.unbalanced_releases(), 0);
}

#[test]
fn critical_section_conformance_posix() {
    let mut section = PosixCriticalSection::new();
    critical_section_contract(&mut section, PosixCriticalSection::depth);
    assert_eq!(section.unbalanced_releases(), 0);
}

#[test]
fn posix_critical_section_excludes_a_second_thread() {
    let mut holder = PosixCriticalSection::new();
    let mut contender = holder.clone();
    let entered = Arc::new(AtomicBool::new(false));
    let entered_by_worker = Arc::clone(&entered);
    let (ready_sender, ready_receiver) = mpsc::channel();

    holder.acquire();
    let worker = thread::spawn(move || {
        ready_sender
            .send(())
            .expect("main thread stopped listening");
        contender.acquire();
        entered_by_worker.store(true, Ordering::SeqCst);
        contender.release();
    });
    ready_receiver
        .recv()
        .expect("worker died before signalling");
    // The worker cannot be inside the section while we hold it,
    // regardless of scheduling.
    assert!(!entered.load(Ordering::SeqCst));
    holder.release();
    worker.join().expect("worker panicked");
    assert!(entered.load(Ordering::SeqCst));
    assert_eq!(holder.depth(), 0);
}

// --- Entropy ------------------------------------------------------------

#[test]
fn entropy_conformance_mock() {
    let mut entropy = MockEntropy::new(7);
    entropy_contract(&mut entropy);
}

#[test]
fn entropy_conformance_posix() {
    let mut entropy = PosixEntropy::new();
    entropy_contract(&mut entropy);
}

#[test]
fn entropy_conformance_fixed_seeded() {
    let mut entropy = FixedEntropy::from_seed(7);
    entropy_contract(&mut entropy);
    // Determinism on top of the shared contract: the same seed replays
    // the same stream.
    let mut replay = FixedEntropy::from_seed(7);
    let mut expected = [0_u8; 32];
    replay.fill(&mut expected).unwrap();
    let mut observed = [0_u8; 32];
    FixedEntropy::from_seed(7).fill(&mut observed).unwrap();
    assert_eq!(expected, observed);
}

// --- UniqueId -----------------------------------------------------------

#[test]
fn unique_id_conformance_mock() {
    let identity = MockUniqueId::new(&[1, 2, 3]);
    unique_id_contract(&identity, 3);
}

#[test]
fn unique_id_conformance_posix() {
    let identity = PosixUniqueId::new();
    unique_id_contract(&identity, UNIQUE_ID_MAX);
}

// --- PlatformInfo -------------------------------------------------------

#[test]
fn platform_info_conformance_mock() {
    platform_info_contract(&MockPlatformInfo, "posix-host", 0);
}

#[test]
fn platform_info_conformance_posix() {
    platform_info_contract(&PosixPlatformInfo::new(), "posix", 0);
}
