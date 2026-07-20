//! Pinned-baseline parity evidence for STM32 CAN adapter semantics.
//!
//! Pinned upstream baseline: `be0029bbb79fe901048a24c2665f2ba854328734`
//! ("Add STM32 FDCAN transceiver adapter"). Promoted 2026-07-20 from the
//! 2026-07-19 drift tranche (formerly `drift_stm32_can_semantics.rs`; the
//! oracle re-pin `ddbcf88a` -> `be0029b` made these contracts baseline
//! behavior, see `docs/port/repin-2026-07-20.md`). The upstream behaviors
//! pinned here entered upstream in commits `1a11d135` ("Add STM32 CAN
//! drivers and bxCAN transceiver"), `be0029bb`, and `6cafa673` ("Fix logger
//! to work for extended canId"). Each test pins a behavioral contract of
//! the upstream STM32 adapters that the Rust port implements (see
//! `docs/port/can-parity.md` and
//! `docs/port/stm32-can-drift-comparison-2026-07-19.md`).

use bsw_can::can_id::{CanId, EXTENDED_QUALIFIER_BIT};
use bsw_can::dlc::dlc_to_length;
use bsw_can::error_state::{ErrorStateTracker, BUS_OFF_THRESHOLD, ERROR_PASSIVE_THRESHOLD};
use bsw_can::filter::{BitFieldFilter, Filter};
use bsw_can::transceiver::{AbstractTransceiver, ErrorCode, State, TransceiverState};
use bsw_time::{Duration, Instant};

type Trx = AbstractTransceiver<8>;

/// Upstream baseline `1a11d135`/`be0029bb`: `BxCanTransceiver::init()` and
/// `FdCanTransceiver::init()` return `CAN_ERR_ILLEGAL_STATE` unless the
/// transceiver is CLOSED; `mute()` requires OPEN; `unmute()` requires MUTED;
/// `close()` from CLOSED succeeds (idempotent); the bus-off supervision task
/// moves MUTED back to OPEN on recovery.
///
/// The Rust state machine enforces the same guard set. One recorded parity
/// decision is pinned here (`docs/port/can-parity.md`, comparison row 21):
/// upstream `open()` also accepts CLOSED and re-runs device init internally,
/// while the Rust port requires an explicit `init()` first (Closed -> Open
/// is rejected).
#[test]
fn lifecycle_guards_match_upstream_adapter_state_machine() {
    let mut t = Trx::new(0);

    // init: only from Closed.
    assert_eq!(t.transition_to_initialized(), ErrorCode::Ok);
    assert_eq!(t.transition_to_initialized(), ErrorCode::IllegalState);

    // mute: only from Open (here still Initialized).
    assert_eq!(t.transition_to_muted(), ErrorCode::IllegalState);

    // open: from Initialized.
    assert_eq!(t.transition_to_open(), ErrorCode::Ok);

    // mute from Open, then the MUTED -> OPEN recovery path used by the
    // upstream cyclic bus-off supervision task.
    assert_eq!(t.transition_to_muted(), ErrorCode::Ok);
    assert_eq!(t.transition_to_open(), ErrorCode::Ok);

    // close always succeeds and is idempotent, like upstream close().
    assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
    assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
    assert_eq!(t.state(), State::Closed);

    // Recorded parity decision (can-parity.md, comparison row 21): Rust
    // rejects Closed -> Open without init(); upstream open() would re-init
    // from CLOSED instead.
    assert_eq!(t.transition_to_open(), ErrorCode::IllegalState);
}

/// Upstream baseline `be0029bb`/`1a11d135`: the M_CAN core behind FDCAN has no
/// automatic bus-off recovery (no bxCAN-style ABOM); on bus-off the hardware
/// latches and the caller must explicitly re-init to rejoin the bus.
///
/// The Rust `ErrorStateTracker` implements exactly that policy: bus-off is
/// latched (further traffic is ignored), and the node only rejoins through an
/// explicit `begin_recovery` plus an exact injected deadline.
#[test]
fn bus_off_is_latched_until_explicit_timed_recovery() {
    let mut tracker = ErrorStateTracker::new();

    // Walk TEC to bus-off through the ISO thresholds (128 passive, 256 off).
    let mut seen = Vec::new();
    while tracker.state() != TransceiverState::BusOff {
        if let Some(state) = tracker.record_tx_error() {
            seen.push(state);
        }
    }
    assert_eq!(seen, [TransceiverState::Passive, TransceiverState::BusOff]);
    assert_eq!(tracker.tec(), BUS_OFF_THRESHOLD);
    assert!(tracker.tec() >= ERROR_PASSIVE_THRESHOLD);

    // Latched against further errors: while bus-off the error paths are
    // ignored (upstream: the controller is off the bus; the adapter only
    // polls isBusOff()). Note: the success paths are NOT guarded — a bus-off
    // controller cannot produce them, so the drivers never call them in this
    // state (recorded in the drift comparison memo).
    assert_eq!(tracker.record_tx_error(), None);
    assert_eq!(tracker.record_rx_error(), None);
    assert_eq!(tracker.state(), TransceiverState::BusOff);

    // No implicit recovery: polling without begin_recovery never rejoins.
    // Recorded parity decision (can-parity.md, comparison row 27): upstream
    // bxCAN relies on ABOM hardware auto-recovery; the Rust port uses
    // explicit timed recovery on both controllers.
    assert_eq!(
        tracker.poll_recovery(Instant::from_nanos(u64::MAX / 2)),
        None
    );
    assert_eq!(tracker.state(), TransceiverState::BusOff);

    // Explicit recovery with an exact deadline, then counters clear.
    let now = Instant::from_nanos(1_000);
    tracker.begin_recovery(now, Duration::from_nanos(1_000_000));
    assert_eq!(tracker.poll_recovery(Instant::from_nanos(1_000_999)), None);
    assert_eq!(
        tracker.poll_recovery(Instant::from_nanos(1_001_000)),
        Some(TransceiverState::Active)
    );
    assert_eq!((tracker.tec(), tracker.rec()), (0, 0));
}

/// Upstream baseline `1a11d135`: the ISR-level software bit-field filter is
/// indexed by CAN ID and therefore applies to standard 11-bit IDs only;
/// extended frames bypass it ("a 29-bit index would be out of range").
///
/// The Rust `BitFieldFilter` pins the same domain contract: its universe is
/// 0..=0x7FF, out-of-range adds are silently ignored, and out-of-range
/// lookups never match, so extended-frame acceptance can never be delegated
/// to the bit field — exactly the assumption the upstream adapters encode by
/// passing accept-all at ISR level and filtering per listener.
#[test]
fn bit_field_filter_domain_is_standard_ids_only() {
    let mut filter = BitFieldFilter::new();

    filter.add(0x7FF);
    assert!(filter.matches(0x7FF));

    // A 29-bit extended raw ID is outside the bit-field domain.
    let extended_raw = 0x18DA_F110_u32;
    filter.add(extended_raw);
    assert!(!filter.matches(extended_raw));

    // Even a fully open 11-bit field says nothing about extended IDs.
    filter.open();
    assert!(filter.matches(0));
    assert!(filter.matches(0x7FF));
    assert!(!filter.matches(0x800));
    assert!(!filter.matches(extended_raw));
}

/// Upstream baseline `1a11d135`: both receive paths clamp wire DLC values 9..15
/// to 8 data bytes for classic CAN ("Classic CAN allows DLC 9-15 on the wire
/// (all mean 8 data bytes); clamp to the payload buffer size").
///
/// The Rust port encodes the identical rule in `bsw_can::dlc`.
#[test]
fn classic_dlc_nine_to_fifteen_clamp_to_eight_bytes() {
    for dlc in 0..=8u8 {
        assert_eq!(dlc_to_length(dlc, false), Some(usize::from(dlc)));
    }
    for dlc in 9..=15u8 {
        assert_eq!(dlc_to_length(dlc, false), Some(8));
    }
    assert_eq!(dlc_to_length(16, false), None);
}

/// Upstream baseline `6cafa673` ("Fix logger to work for extended canId"): the
/// POSIX transceiver logged the raw encoded id word, so extended frames
/// printed with the embedded extended-qualifier flag set; the fix routes the
/// value through `CanId::rawId()`.
///
/// The Rust `CanId` pins the same split: `raw_id()` returns only the 29
/// arbitration bits, while `value()` keeps the qualifier encoding.
#[test]
fn extended_id_raw_value_strips_qualifier_bits_for_display() {
    let ext = CanId::extended(0x18DA_F110);
    assert_eq!(ext.raw_id(), 0x18DA_F110);
    assert_eq!(ext.value(), 0x18DA_F110 | EXTENDED_QUALIFIER_BIT);
    assert_ne!(ext.value(), ext.raw_id());

    let base = CanId::base(0x123);
    assert_eq!(base.raw_id(), 0x123);
    assert_eq!(base.value(), base.raw_id());
}

/// Upstream baseline `1a11d135`/`be0029bb`: on TX-path saturation the adapters
/// increment an overrun counter and return `CAN_ERR_TX_HW_QUEUE_FULL`; when
/// the transceiver is not OPEN (or muted) writes return `CAN_ERR_TX_OFFLINE`.
///
/// The Rust contract carries the same result vocabulary (`TxHwQueueFull`,
/// `TxOffline`) and per-transceiver drop statistics; this test pins that the
/// accounting fields exist, start at zero, and count independently.
#[test]
fn tx_saturation_vocabulary_and_drop_accounting_exist() {
    let mut t = Trx::new(1);
    assert_eq!(t.statistics().tx_dropped, 0);
    assert_eq!(t.statistics().rx_dropped, 0);

    // The drivers increment tx_dropped on hardware queue-full before
    // returning TxHwQueueFull; rx_dropped counts software queue overflow.
    t.statistics_mut().tx_dropped += 1;
    t.statistics_mut().rx_dropped += 2;
    assert_eq!(t.statistics().tx_dropped, 1);
    assert_eq!(t.statistics().rx_dropped, 2);

    // The two error codes are distinct outcomes, as upstream distinguishes
    // TX_HW_QUEUE_FULL from TX_OFFLINE.
    assert_ne!(ErrorCode::TxHwQueueFull, ErrorCode::TxOffline);
}
