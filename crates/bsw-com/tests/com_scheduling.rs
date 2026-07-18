// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Fake-clock scenario tests for COM TX scheduling, RX deadline monitoring,
//! update/invalid flags, and error reporting (package E33).
//!
//! Semantics under test (see `bsw_com::com` module docs):
//!
//! - cyclic TX PDUs fire on a drift-free grid anchored at `start(now)`;
//!   deadlines are hit at their exact boundary (`is_at_or_after`);
//! - late ticks fire once, count missed periods, and keep the grid;
//! - event transmissions honor per-signal transfer properties, explicit
//!   triggers, and `min_event_interval` throttling;
//! - RX deadlines re-arm on each reception, report each miss exactly once,
//!   and apply the configured timeout-default action;
//! - all errors surface through the polled bounded event queue.

use bsw_com::{
    ByteOrder, ComEvent, ComManager, PduDescriptor, RxTimeoutAction, SignalDescriptor, SignalType,
    SignalValue, TransferProperty, TxMode, EVENT_QUEUE_CAP,
};
use bsw_time::{Clock, Duration, FakeClock, Instant};

const fn ms(value: u64) -> Duration {
    Duration::from_nanos(value * 1_000_000)
}

const fn at_ms(value: u64) -> Instant {
    Instant::from_nanos(value * 1_000_000)
}

fn u8_signal(id: u16, byte: u16, init: u32) -> SignalDescriptor {
    SignalDescriptor::new(
        id,
        byte * 8,
        8,
        SignalType::Uint8,
        ByteOrder::LittleEndian,
        init,
    )
}

fn triggered_u8_signal(id: u16, byte: u16, init: u32) -> SignalDescriptor {
    u8_signal(id, byte, init).with_transfer_property(TransferProperty::Triggered)
}

fn tx_ids<const P: usize, const S: usize>(com: &mut ComManager<P, S>, now: Instant) -> Vec<u32> {
    com.tick(now).map(|(can_id, _)| can_id).collect()
}

// ---------------------------------------------------------------------------
// Cyclic grid
// ---------------------------------------------------------------------------

#[test]
fn cyclic_grid_fires_on_exact_boundaries() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::tx(0x100, 8, TxMode::Cyclic(ms(100))),
        &[u8_signal(1, 0, 0)],
    );
    com.start(at_ms(0));

    // Grid anchored at start: due immediately.
    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x100]);
    // 1 ns before the grid point: not due.
    let just_before = Instant::from_nanos(ms(100).as_nanos() - 1);
    assert_eq!(tx_ids(&mut com, just_before), Vec::<u32>::new());
    // Exactly at the grid point: due.
    assert_eq!(tx_ids(&mut com, at_ms(100)), vec![0x100]);
    // Next grid point.
    assert_eq!(tx_ids(&mut com, at_ms(200)), vec![0x100]);
    assert_eq!(com.take_event(), None);
    assert_eq!(com.tx_overruns(0), Some(0));
}

#[test]
fn late_tick_counts_overruns_and_keeps_grid() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let pdu = com
        .add_pdu(
            PduDescriptor::tx(0x100, 8, TxMode::Cyclic(ms(100))),
            &[u8_signal(1, 0, 0)],
        )
        .unwrap();
    com.start(at_ms(0));
    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x100]);

    // Late tick at t=350 ms: deadlines 100, 200, 300 are all due — one
    // transmission, two skipped periods.
    assert_eq!(tx_ids(&mut com, at_ms(350)), vec![0x100]);
    assert_eq!(
        com.take_event(),
        Some(ComEvent::TxOverrun {
            can_id: 0x100,
            missed: 2,
        })
    );
    assert_eq!(com.take_event(), None);
    assert_eq!(com.tx_overruns(pdu), Some(2));

    // Next deadline is back on the grid at 400 ms — no drift.
    assert_eq!(tx_ids(&mut com, at_ms(399)), Vec::<u32>::new());
    assert_eq!(tx_ids(&mut com, at_ms(400)), vec![0x100]);
    assert_eq!(com.take_event(), None);
}

// ---------------------------------------------------------------------------
// Event / mixed transmission modes
// ---------------------------------------------------------------------------

#[test]
fn event_only_pdu_never_fires_cyclically() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::tx(0x200, 8, TxMode::EventOnly),
        &[triggered_u8_signal(5, 0, 0)],
    );
    com.start(at_ms(0));

    for t in [0, 50, 100, 1_000] {
        assert_eq!(tx_ids(&mut com, at_ms(t)), Vec::<u32>::new());
    }

    // A write to the Triggered signal requests transmission on the next
    // tick regardless of any grid.
    assert!(com.write_signal(5, SignalValue::U8(0x42)));
    assert_eq!(tx_ids(&mut com, at_ms(1_017)), vec![0x200]);
    // One-shot: nothing pending afterwards.
    assert_eq!(tx_ids(&mut com, at_ms(1_020)), Vec::<u32>::new());
}

#[test]
fn pending_transfer_property_does_not_trigger() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let pdu = com
        .add_pdu(
            PduDescriptor::tx(0x200, 8, TxMode::EventOnly),
            &[u8_signal(5, 0, 0)], // TransferProperty::Pending (default)
        )
        .unwrap();
    com.start(at_ms(0));

    assert!(com.write_signal(5, SignalValue::U8(0x42)));
    assert_eq!(tx_ids(&mut com, at_ms(10)), Vec::<u32>::new());

    // The explicit trigger API still works for Pending signals.
    assert!(com.trigger(pdu));
    assert_eq!(tx_ids(&mut com, at_ms(20)), vec![0x200]);
}

#[test]
fn mixed_event_fires_off_grid_and_grid_is_unaffected() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::tx(0x300, 8, TxMode::Mixed(ms(100))),
        &[triggered_u8_signal(7, 0, 0)],
    );
    com.start(at_ms(0));

    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x300]); // cyclic
    assert!(com.write_signal(7, SignalValue::U8(1)));
    assert_eq!(tx_ids(&mut com, at_ms(30)), vec![0x300]); // event, off grid
                                                          // The event transmission does not move the cyclic grid.
    assert_eq!(tx_ids(&mut com, at_ms(99)), Vec::<u32>::new());
    assert_eq!(tx_ids(&mut com, at_ms(100)), vec![0x300]); // cyclic on grid
}

#[test]
fn mixed_cyclic_fire_folds_pending_event_into_one_transmission() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::tx(0x300, 8, TxMode::Mixed(ms(100))),
        &[triggered_u8_signal(7, 0, 0)],
    );
    com.start(at_ms(0));
    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x300]);

    // Event pending exactly when the grid point is due: a single yield.
    assert!(com.write_signal(7, SignalValue::U8(2)));
    assert_eq!(tx_ids(&mut com, at_ms(100)), vec![0x300]);
    // The pending event was consumed by the fold.
    assert_eq!(tx_ids(&mut com, at_ms(110)), Vec::<u32>::new());
}

#[test]
fn min_event_interval_throttles_and_reports_once() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let pdu = com
        .add_pdu(
            PduDescriptor::tx(0x200, 8, TxMode::EventOnly).with_min_event_interval(ms(50)),
            &[triggered_u8_signal(5, 0, 0)],
        )
        .unwrap();
    com.start(at_ms(0));

    // First event: no previous transmission, no throttle.
    assert!(com.trigger(pdu));
    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x200]);

    // Second event 10 ms later: throttled, reported exactly once.
    assert!(com.trigger(pdu));
    assert_eq!(tx_ids(&mut com, at_ms(10)), Vec::<u32>::new());
    assert_eq!(
        com.take_event(),
        Some(ComEvent::EventTxThrottled { can_id: 0x200 })
    );
    assert_eq!(tx_ids(&mut com, at_ms(20)), Vec::<u32>::new());
    assert_eq!(com.take_event(), None); // no duplicate throttle event

    // Still throttled 1 ms before the interval elapses.
    assert_eq!(tx_ids(&mut com, at_ms(49)), Vec::<u32>::new());
    // Fires exactly at the interval boundary (last tx at 0 + 50 ms).
    assert_eq!(tx_ids(&mut com, at_ms(50)), vec![0x200]);
    assert_eq!(com.take_event(), None);
}

#[test]
fn event_and_cyclic_interleave_in_registration_order() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let event = com
        .add_pdu(
            PduDescriptor::tx(0x200, 8, TxMode::EventOnly),
            &[u8_signal(5, 0, 0)],
        )
        .unwrap();
    com.add_pdu(
        PduDescriptor::tx(0x100, 8, TxMode::Cyclic(ms(50))),
        &[u8_signal(1, 0, 0)],
    );
    com.start(at_ms(0));

    // Both due on the same tick: yielded in registration order (event PDU
    // was registered first), not grid-vs-event order.
    assert!(com.trigger(event));
    assert_eq!(tx_ids(&mut com, at_ms(0)), vec![0x200, 0x100]);
}

// ---------------------------------------------------------------------------
// RX deadline monitoring
// ---------------------------------------------------------------------------

#[test]
fn rx_deadline_resets_on_reception_and_restores_init_on_miss() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::rx(0x500, 8).with_rx_timeout(ms(100), RxTimeoutAction::RestoreInit),
        &[u8_signal(40, 0, 0xAA)],
    );
    com.start(at_ms(0));

    // Reception at t=50 ms resets the deadline to t=150 ms.
    com.receive(0x500, &[0x11, 0, 0, 0, 0, 0, 0, 0], at_ms(50));
    assert_eq!(com.read_signal(40), Some(SignalValue::U8(0x11)));

    // 1 ns before the deadline: no miss.
    let just_before = Instant::from_nanos(ms(150).as_nanos() - 1);
    assert_eq!(com.tick(just_before).count(), 0);
    assert_eq!(com.take_event(), None);

    // Exactly at the deadline: miss, RestoreInit, signals invalid.
    assert_eq!(com.tick(at_ms(150)).count(), 0);
    assert_eq!(
        com.take_event(),
        Some(ComEvent::RxDeadlineMissed {
            can_id: 0x500,
            at: at_ms(150),
        })
    );
    assert_eq!(com.read_signal(40), Some(SignalValue::U8(0xAA))); // init value
    assert_eq!(com.is_signal_valid(40), Some(false));

    // The miss disarms monitoring: no repeated events.
    assert_eq!(com.tick(at_ms(500)).count(), 0);
    assert_eq!(com.take_event(), None);

    // Recovery: the next reception clears invalid and re-arms the deadline.
    com.receive(0x500, &[0x22, 0, 0, 0, 0, 0, 0, 0], at_ms(600));
    assert_eq!(com.read_signal(40), Some(SignalValue::U8(0x22)));
    assert_eq!(com.is_signal_valid(40), Some(true));
    assert_eq!(com.tick(at_ms(699)).count(), 0);
    assert_eq!(com.take_event(), None);
    assert_eq!(com.tick(at_ms(700)).count(), 0);
    assert_eq!(
        com.take_event(),
        Some(ComEvent::RxDeadlineMissed {
            can_id: 0x500,
            at: at_ms(700),
        })
    );
}

#[test]
fn rx_keep_last_retains_bytes_and_validity_on_miss() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::rx(0x501, 8).with_rx_timeout(ms(100), RxTimeoutAction::KeepLast),
        &[u8_signal(41, 0, 0xAA)],
    );
    com.start(at_ms(0));

    com.receive(0x501, &[0x33, 0, 0, 0, 0, 0, 0, 0], at_ms(10));
    assert_eq!(com.tick(at_ms(110)).count(), 0);
    assert_eq!(
        com.take_event(),
        Some(ComEvent::RxDeadlineMissed {
            can_id: 0x501,
            at: at_ms(110),
        })
    );
    // KeepLast: bytes retained, validity unaffected.
    assert_eq!(com.read_signal(41), Some(SignalValue::U8(0x33)));
    assert_eq!(com.is_signal_valid(41), Some(true));
}

#[test]
fn rx_first_deadline_runs_from_start_without_reception() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::rx(0x502, 8).with_rx_timeout(ms(100), RxTimeoutAction::KeepLast),
        &[u8_signal(42, 0, 0)],
    );
    com.start(at_ms(0));

    // Nothing was ever received: the first deadline is start + timeout.
    assert_eq!(com.tick(at_ms(99)).count(), 0);
    assert_eq!(com.take_event(), None);
    assert_eq!(com.tick(at_ms(100)).count(), 0);
    assert_eq!(
        com.take_event(),
        Some(ComEvent::RxDeadlineMissed {
            can_id: 0x502,
            at: at_ms(100),
        })
    );
    // Reported once, then disarmed until a reception re-arms it.
    assert_eq!(com.tick(at_ms(10_000)).count(), 0);
    assert_eq!(com.take_event(), None);
}

#[test]
fn unmonitored_rx_pdu_never_times_out() {
    let mut com: ComManager<4, 16> = ComManager::new();
    // Default policy: rx_timeout = None → not monitored.
    com.add_pdu(PduDescriptor::rx(0x503, 8), &[u8_signal(43, 0, 0)]);
    com.start(at_ms(0));
    assert_eq!(com.tick(at_ms(1_000_000)).count(), 0);
    assert_eq!(com.take_event(), None);
}

// ---------------------------------------------------------------------------
// Update flags
// ---------------------------------------------------------------------------

#[test]
fn tx_update_flag_cleared_exactly_when_pdu_is_yielded() {
    let mut com: ComManager<4, 16> = ComManager::new();
    com.add_pdu(
        PduDescriptor::tx(0x100, 8, TxMode::Cyclic(ms(100))),
        &[u8_signal(1, 0, 0)],
    );
    com.start(at_ms(0));

    assert!(com.write_signal(1, SignalValue::U8(9)));
    assert_eq!(com.is_signal_updated(1), Some(true));

    // The PDU is due but the iterator is dropped unconsumed: the flag is
    // cleared on yield, not on scheduling, so it survives.
    {
        let _unconsumed = com.tick(at_ms(0));
    }
    assert_eq!(com.is_signal_updated(1), Some(true));

    // Consuming the iterator yields the PDU and clears the flag.
    assert_eq!(tx_ids(&mut com, at_ms(100)), vec![0x100]);
    assert_eq!(com.is_signal_updated(1), Some(false));
}

#[test]
fn rx_take_updated_is_read_and_clear() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let pdu = com
        .add_pdu(PduDescriptor::rx(0x400, 8), &[u8_signal(30, 0, 0)])
        .unwrap();

    assert_eq!(com.take_updated(30), None); // nothing received yet
    com.receive(0x400, &[0x55, 0, 0, 0, 0, 0, 0, 0], at_ms(10));

    // Non-consuming peek, then read-and-clear.
    assert_eq!(com.is_signal_updated(30), Some(true));
    assert_eq!(com.take_updated(30), Some(SignalValue::U8(0x55)));
    assert_eq!(com.is_signal_updated(30), Some(false));
    assert_eq!(com.take_updated(30), None);
    // Plain reads still see the value.
    assert_eq!(com.read_signal(30), Some(SignalValue::U8(0x55)));

    // Per-PDU received indication is independent read-and-clear.
    assert_eq!(com.take_pdu_received(pdu), Some(true));
    assert_eq!(com.take_pdu_received(pdu), Some(false));

    // A new reception sets both again.
    com.receive(0x400, &[0x66, 0, 0, 0, 0, 0, 0, 0], at_ms(20));
    assert_eq!(com.take_updated(30), Some(SignalValue::U8(0x66)));
    assert_eq!(com.take_pdu_received(pdu), Some(true));
}

// ---------------------------------------------------------------------------
// Wraparound
// ---------------------------------------------------------------------------

#[test]
fn cyclic_and_rx_deadline_survive_instant_wraparound() {
    let mut com: ComManager<4, 16> = ComManager::new();
    let tx = com
        .add_pdu(
            PduDescriptor::tx(0x100, 8, TxMode::Cyclic(ms(10))),
            &[u8_signal(1, 0, 0)],
        )
        .unwrap();
    com.add_pdu(
        PduDescriptor::rx(0x500, 8).with_rx_timeout(ms(10), RxTimeoutAction::KeepLast),
        &[u8_signal(40, 0, 0)],
    );

    let near_wrap = Instant::from_nanos(u64::MAX - ms(5).as_nanos());
    let mut clock = FakeClock::new(near_wrap);
    com.start(clock.now());
    assert_eq!(tx_ids(&mut com, clock.now()), vec![0x100]);
    com.receive(0x500, &[1, 0, 0, 0, 0, 0, 0, 0], clock.now());

    // Advance across the wrap: the cyclic PDU fires on its wrapped grid
    // point with no overruns, and the RX deadline miss fires at its wrapped
    // boundary.
    clock.advance(ms(10));
    assert_eq!(tx_ids(&mut com, clock.now()), vec![0x100]);
    assert_eq!(com.tx_overruns(tx), Some(0));
    assert_eq!(
        com.take_event(),
        Some(ComEvent::RxDeadlineMissed {
            can_id: 0x500,
            at: near_wrap.wrapping_add(ms(10)),
        })
    );
    assert_eq!(com.take_event(), None);
}

// ---------------------------------------------------------------------------
// Event queue
// ---------------------------------------------------------------------------

#[test]
fn event_queue_overflow_drops_newest_and_counts() {
    // 17 monitored RX PDUs missing their deadline in one tick produce one
    // more event than the queue can hold.
    let mut com: ComManager<20, 4> = ComManager::new();
    for i in 0..=u32::try_from(EVENT_QUEUE_CAP).unwrap() {
        assert!(com
            .add_pdu(
                PduDescriptor::rx(0x500 + i, 8).with_rx_timeout(ms(10), RxTimeoutAction::KeepLast),
                &[],
            )
            .is_some());
    }
    com.start(at_ms(0));
    assert_eq!(com.tick(at_ms(10)).count(), 0);

    // Exactly EVENT_QUEUE_CAP events queued (FIFO), one dropped and counted.
    let mut drained = 0;
    while let Some(event) = com.take_event() {
        assert!(matches!(event, ComEvent::RxDeadlineMissed { .. }));
        drained += 1;
    }
    assert_eq!(drained, EVENT_QUEUE_CAP);
    assert_eq!(com.events_dropped(), 1);
    assert_eq!(com.take_event(), None);
}
