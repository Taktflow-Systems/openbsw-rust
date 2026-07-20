//! Pinned-baseline parity evidence for the STM32 CAN health/queue seams.
//!
//! Pinned upstream baseline: `be0029bbb79fe901048a24c2665f2ba854328734`
//! ("Add STM32 FDCAN transceiver adapter"). Promoted 2026-07-20 from the
//! 2026-07-19 drift tranche (formerly `drift_stm32_can_semantics.rs`; the
//! oracle re-pin `ddbcf88a` -> `be0029b` made these contracts baseline
//! behavior, see `docs/port/repin-2026-07-20.md`). The upstream behaviors
//! pinned here entered upstream in commits `1a11d135` ("Add STM32 CAN
//! drivers and bxCAN transceiver"), `be0029bb`, and `f2f01102` ("Fix STM32
//! CAN unit tests on 64-bit hosts").
//!
//! Only the feature-ungated host-testable modules (`can_health`, `can_isr`)
//! are exercised, following this crate's existing host-test convention
//! (`cargo test -p bsw-bsp-stm32`, also used by `tools/port/check_miri.ps1`).
//! Register-level driver behavior stays target/HIL scope; see
//! `docs/port/stm32-can-drift-comparison-2026-07-19.md`.

use bsw_bsp_stm32::can_health::{
    service_registers, CanHealth, CanInterruptFlags, ControllerKind, RegisterBank,
};
use bsw_bsp_stm32::can_isr::InterruptQueue;
use bsw_can::{CanFrame, CanId, TransceiverState};
use bsw_time::{Duration, Instant};

/// Mock register bank that models write-1-to-clear flag registers and counts
/// every write, so the tests can pin the upstream rc_w1 discipline.
struct W1cRegisters {
    values: [u32; 32],
    writes: u32,
}

impl W1cRegisters {
    fn new() -> Self {
        Self {
            values: [0; 32],
            writes: 0,
        }
    }
}

impl RegisterBank for W1cRegisters {
    fn read(&self, offset: usize) -> u32 {
        self.values[offset / 4]
    }

    fn write(&mut self, offset: usize, value: u32) {
        // Hardware rc_w1 semantics: writing 1 clears the flag, writing 0
        // leaves it untouched.
        self.values[offset / 4] &= !value;
        self.writes += 1;
    }
}

const FDCAN_IR: usize = 0x050;
const BXCAN_MSR: usize = 0x004;
const BXCAN_ESR: usize = 0x018;

/// Upstream baseline `1a11d135`: bxCAN ESR/MSR are status registers that the
/// adapter only reads (`isBusOff()` reads `ESR.BOFF`; error counters come
/// from ESR fields); no read-modify-write is ever performed on them.
///
/// The Rust `service_registers` bxCAN path pins the same discipline: it
/// decodes bus-off (ESR bit 2), error-passive (ESR bit 1), protocol error
/// (ESR LEC bits) and TX-complete (MSR bit 8) from plain reads and performs
/// zero register writes.
#[test]
fn bxcan_status_decode_is_read_only() {
    let mut bank = W1cRegisters::new();
    bank.values[BXCAN_ESR / 4] = (1 << 2) | (1 << 1) | 0x30; // BOFF | EPVF | LEC
    bank.values[BXCAN_MSR / 4] = 1 << 8; // TXOK-style TX complete

    let flags = service_registers(&mut bank, ControllerKind::BxCan);
    assert!(flags.bus_off);
    assert!(flags.error_passive);
    assert!(flags.protocol_error);
    assert!(flags.tx_complete);
    assert!(!flags.rx);

    // No write happened: the status flags are still latched in the bank.
    assert_eq!(bank.writes, 0);
    assert_eq!(bank.values[BXCAN_ESR / 4], (1 << 2) | (1 << 1) | 0x30);
}

/// Upstream baseline `1a11d135`/`f2f01102`: rc_w1 interrupt registers are
/// cleared with a direct write of the read snapshot, never a
/// read-modify-write ("a read-modify-write would write back any flag that
/// happens to be set and silently clear it").
///
/// The Rust FDCAN path in `service_registers` pins the same shape: one read
/// snapshot, one write of exactly that snapshot, and the acknowledged edge is
/// consumed so a second service pass observes no stale flags.
#[test]
fn fdcan_ir_acknowledge_is_single_snapshot_write_back() {
    let raw = (1 << 0) | (1 << 23) | (1 << 25); // RF0N | EP | BO
    let mut bank = W1cRegisters::new();
    bank.values[FDCAN_IR / 4] = raw;

    let flags = service_registers(&mut bank, ControllerKind::FdCan);
    assert!(flags.rx);
    assert!(flags.error_passive);
    assert!(flags.bus_off);

    // Exactly one W1C write, and it cleared precisely the snapshot.
    assert_eq!(bank.writes, 1);
    assert_eq!(bank.values[FDCAN_IR / 4], 0);

    // The edge is consumed: a second pass reports nothing.
    let again = service_registers(&mut bank, ControllerKind::FdCan);
    assert_eq!(again, CanInterruptFlags::default());
}

/// Upstream baseline `1a11d135`: when the software RX queue is full the ISR
/// releases the hardware FIFO entry without storing it — the newest frame is
/// dropped, previously queued frames are preserved in FIFO order, and the
/// drain continues.
///
/// The Rust `InterruptQueue` pins the same drop-newest policy and adds
/// explicit accounting (`dropped()`), which upstream leaves silent. One
/// recorded parity decision is pinned here (`docs/port/can-parity.md`,
/// comparison row 30): the Rust SPSC ring reserves one slot, so
/// `InterruptQueue<N>` stores N-1 frames (upstream's 32-entry array
/// stores 32).
#[test]
fn rx_queue_full_drops_newest_preserves_order_and_accounts() {
    fn frame(id: u16) -> CanFrame {
        CanFrame::with_data(CanId::base(id), &[id as u8])
    }

    let queue: InterruptQueue<8> = InterruptQueue::new();

    // Usable capacity is N-1 = 7.
    for id in 1..=7u16 {
        assert!(queue.push(frame(id)), "slot {id} must accept");
    }
    assert_eq!(queue.len(), 7);

    // Queue full: newest frames are rejected and each rejection is counted.
    assert!(!queue.push(frame(100)));
    assert!(!queue.push(frame(101)));
    assert_eq!(queue.dropped(), 2);
    assert_eq!(queue.len(), 7, "stored frames are untouched by drops");

    // Draining yields the preserved frames in arrival order — the dropped
    // frames never appear.
    for id in 1..=7u16 {
        assert_eq!(queue.pop().unwrap().id().raw_id(), u32::from(id));
    }
    assert!(queue.pop().is_none());

    // After frees, new frames are accepted again (upstream: next ISR pass
    // stores normally once the task drained the queue).
    assert!(queue.push(frame(8)));
    assert_eq!(queue.dropped(), 2, "drop count is cumulative, not reset");
}

/// Upstream baseline `1a11d135`/`be0029bb`: the transceivers poll bus-off every
/// 10 ms; on bus-off the logical state leaves OPEN, and it only returns after
/// the controller has actually recovered. On FDCAN the M_CAN core never
/// recovers by itself — the caller must re-init after its own delay policy.
///
/// The Rust `CanHealth` seam pins that policy with exact injected deadlines:
/// a bus-off flag latches `BusOff`, recovery is refused one nanosecond before
/// the deadline, granted exactly at it, and the queue-overflow high-water
/// accounting is monotonic.
#[test]
fn bus_off_supervision_uses_exact_deadline_and_monotonic_overflow_accounting() {
    let mut health = CanHealth::new(Duration::from_nanos(2_000));
    let t0 = Instant::from_nanos(100);

    let state = health.handle(
        CanInterruptFlags {
            bus_off: true,
            ..CanInterruptFlags::default()
        },
        t0,
    );
    assert_eq!(state, TransceiverState::BusOff);
    assert_eq!(health.error_interrupts(), 1);

    // One nanosecond early: still bus-off (no premature rejoin).
    assert_eq!(health.poll_recovery(Instant::from_nanos(2_099)), None);
    assert_eq!(health.state(), TransceiverState::BusOff);

    // Exactly at the deadline: active again (the driver then re-inits the
    // controller, matching the FDCAN manual-restart requirement).
    assert_eq!(
        health.poll_recovery(Instant::from_nanos(2_100)),
        Some(TransceiverState::Active)
    );

    // Overflow accounting is a monotonic high-water mark.
    health.observe_queue_drops(5);
    health.observe_queue_drops(3);
    assert_eq!(health.queue_overflows(), 5);
    health.observe_queue_drops(9);
    assert_eq!(health.queue_overflows(), 9);
}

/// Upstream baseline `1a11d135`: the bxCAN adapter only exposes `isBusOff()`;
/// error-passive is observable solely through the raw REC/TEC counters. The
/// Rust port maps the ESR error-passive flag into the transceiver state as
/// well — a strictly richer health surface, recorded as a parity decision
/// (`docs/port/can-parity.md`, comparison row 18). This test pins that
/// mapping so the strengthening cannot silently regress.
#[test]
fn error_passive_flag_maps_to_passive_state() {
    let flags = CanInterruptFlags::from_bxcan(0, 1 << 1); // ESR.EPVF
    assert!(flags.error_passive);
    assert!(!flags.bus_off);

    let mut health = CanHealth::new(Duration::from_nanos(1_000));
    let state = health.handle(flags, Instant::from_nanos(0));
    assert_eq!(state, TransceiverState::Passive);
    assert_eq!(health.error_interrupts(), 1);
}
