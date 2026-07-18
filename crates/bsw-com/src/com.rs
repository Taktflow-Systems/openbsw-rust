// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! COM manager — owns the signal shadow buffers and PDU state.
//!
//! [`ComManager`] is the central entity of the COM layer.  Application code
//! calls [`write_signal`](ComManager::write_signal) /
//! [`read_signal`](ComManager::read_signal) to exchange typed signal values.
//! The manager handles packing/unpacking to/from PDU shadow buffers and
//! drives TX scheduling and RX deadline monitoring via
//! [`tick`](ComManager::tick).
//!
//! # Timing model (package E33)
//!
//! All time is injected as [`bsw_time::Instant`] — the manager never reads a
//! clock itself.  Scheduling starts at an explicit [`start`](ComManager::start)
//! call:
//!
//! - **Cyclic TX grid** — each cyclic/mixed TX PDU fires on a drift-free
//!   grid anchored at `start(now)`: the first transmission is due
//!   immediately at `now`, then at `now + period`, `now + 2*period`, …
//!   Deadlines advance from the scheduled deadline, not the observed tick
//!   time.  When a tick arrives late, the PDU fires once, every fully
//!   skipped period is counted as an overrun, and a
//!   [`ComEvent::TxOverrun`] is emitted with the newly missed count.
//! - **RX deadlines** — monitoring of an RX PDU with a configured
//!   `rx_timeout` is armed at `start(now)` (first deadline `now + timeout`)
//!   and re-armed by each reception (`rx_time + timeout`).  A deadline
//!   fires at its exact boundary (`is_at_or_after`).  On a miss the PDU is
//!   *disarmed* — exactly one [`ComEvent::RxDeadlineMissed`] per miss — and
//!   the next reception re-arms it.
//! - Before `start`, [`tick`](ComManager::tick) yields nothing and no
//!   deadline is monitored.  `start` may be called again to re-anchor the
//!   grid; it resets overrun counters, pending events, and throttle state.
//!
//! # Event reporting (polling contract)
//!
//! Errors are reported through a bounded FIFO of [`ComEvent`]s drained with
//! [`take_event`](ComManager::take_event) — there are no callbacks, which
//! keeps the layer no_std-friendly and deterministic.  Poll `take_event`
//! after each `tick` (once the TX iterator is consumed).  When the queue is
//! full new events are **dropped** and counted in
//! [`events_dropped`](ComManager::events_dropped).
//!
//! # TX ordering
//!
//! Within one tick, due PDUs are yielded in **registration order** — event
//! and cyclic transmissions interleave deterministically on that order, and
//! each PDU is yielded at most once per tick (a pending event is folded
//! into a coinciding cyclic transmission).
//!
//! # No-std / no-heap
//!
//! All storage is inline and statically sized via const generics
//! (`MAX_PDUS`, `MAX_SIGNALS`).  No allocator is required.

use crate::packer::{pack_signal, unpack_signal};
use crate::pdu::{PduDescriptor, PduDirection, RxTimeoutAction, TxMode};
use crate::signal::{SignalDescriptor, SignalValue, TransferProperty};
use bsw_can::dlc::FD_MAX_LENGTH;
use bsw_time::{Duration, Instant};
use bsw_util::e2e::{E2eChecker, E2eError, E2eProtector, E2eResult};

/// Shadow-buffer capacity per PDU — sized for CAN FD payloads (64 bytes).
pub const PDU_BUFFER_LEN: usize = FD_MAX_LENGTH;

/// Capacity of the [`ComEvent`] queue.  When full, new events are dropped
/// (drop-newest) and counted in [`ComManager::events_dropped`].
pub const EVENT_QUEUE_CAP: usize = 16;

/// Integration error for the project-specific E2E extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComE2eError {
    /// The requested PDU index or CAN ID is not configured.
    UnknownPdu,
    /// Protection was requested for RX, or checking for TX.
    WrongDirection,
    /// The received payload does not exactly match the configured PDU length.
    LengthMismatch,
    /// The project E2E codec rejected the protected buffer shape.
    Codec(E2eError),
}

// ---------------------------------------------------------------------------
// ComEvent
// ---------------------------------------------------------------------------

/// Error/diagnostic event emitted by the COM layer (package E33).
///
/// Drained via [`ComManager::take_event`]; see the module docs for the
/// polling contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComEvent {
    /// An RX PDU's reception deadline was missed.  `at` is the scheduled
    /// deadline instant that was missed (not the tick time).
    RxDeadlineMissed {
        /// CAN ID of the RX PDU.
        can_id: u32,
        /// The missed deadline.
        at: Instant,
    },
    /// A late tick skipped cyclic transmissions.  `missed` is the number of
    /// periods newly skipped in this tick (the cumulative count is available
    /// via [`ComManager::tx_overruns`]).
    TxOverrun {
        /// CAN ID of the TX PDU.
        can_id: u32,
        /// Newly missed periods in this tick.
        missed: u32,
    },
    /// An event transmission was deferred by the PDU's
    /// `min_event_interval`.  Emitted at most once per pending event; the
    /// transmission happens once the interval has elapsed.
    EventTxThrottled {
        /// CAN ID of the TX PDU.
        can_id: u32,
    },
    /// A received payload was longer than the registered PDU length; the
    /// extra bytes were dropped.
    BufferOverflow {
        /// CAN ID of the RX PDU.
        can_id: u32,
    },
}

// ---------------------------------------------------------------------------
// ComTxIterator
// ---------------------------------------------------------------------------

/// Iterator yielding `(can_id, data)` pairs for PDUs due for transmission.
///
/// Produced by [`ComManager::tick`].  PDUs are yielded in registration
/// order.  Yielding a PDU clears the TX update flags of its signals
/// ("written since last transmit"), so flags are cleared exactly when the
/// payload is handed out — a PDU scheduled but never consumed keeps its
/// flags.  **Consume the iterator before the next call to `tick`.**
pub struct ComTxIterator<'m> {
    pdus: &'m [PduDescriptor],
    buffers: &'m [[u8; PDU_BUFFER_LEN]],
    ready: &'m [bool],
    sig_updated: &'m mut [bool],
    pdu_count: usize,
    index: usize,
}

impl<'m> Iterator for ComTxIterator<'m> {
    type Item = (u32, &'m [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.pdu_count {
            let i = self.index;
            self.index += 1;
            if self.ready[i] {
                let pdu = &self.pdus[i];
                // Clear "written since last transmit" for this PDU's signals
                // at the moment the payload is yielded.
                let start = usize::from(pdu.signal_start_index);
                let end = start + usize::from(pdu.signal_count);
                for flag in &mut self.sig_updated[start..end] {
                    *flag = false;
                }
                let data = &self.buffers[i][..usize::from(pdu.length)];
                return Some((pdu.can_id, data));
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// ComManager
// ---------------------------------------------------------------------------

/// COM manager.
///
/// # Type parameters
///
/// - `MAX_PDUS` — maximum number of PDUs that can be registered.
/// - `MAX_SIGNALS` — maximum total signals across all PDUs.
pub struct ComManager<const MAX_PDUS: usize, const MAX_SIGNALS: usize> {
    /// Registered PDU descriptors.
    pdus: [PduDescriptor; MAX_PDUS],
    /// Registered signal descriptors.
    signals: [SignalDescriptor; MAX_SIGNALS],
    /// Number of registered PDUs.
    pdu_count: usize,
    /// Number of registered signals.
    signal_count: usize,
    /// Shadow buffer for each PDU (packed bytes, up to 64 bytes CAN FD).
    pdu_buffers: [[u8; PDU_BUFFER_LEN]; MAX_PDUS],
    /// Next grid deadline for each cyclic/mixed TX PDU.
    tx_next: [Instant; MAX_PDUS],
    /// Cumulative missed cyclic periods per TX PDU.
    tx_overruns: [u32; MAX_PDUS],
    /// Event transmission requested (trigger or Triggered-signal write).
    tx_event_pending: [bool; MAX_PDUS],
    /// Throttle event already emitted for the currently pending event.
    tx_throttle_reported: [bool; MAX_PDUS],
    /// Instant of the most recent transmission of an event-capable TX PDU.
    tx_last_event: [Option<Instant>; MAX_PDUS],
    /// Armed reception deadline per RX PDU (`None` = unmonitored/disarmed).
    rx_deadline: [Option<Instant>; MAX_PDUS],
    /// Per-PDU "received since last check" indication.
    rx_received: [bool; MAX_PDUS],
    /// Scratch array: which PDUs are ready to TX on the current tick.
    tx_ready: [bool; MAX_PDUS],
    /// Per-signal update flags (TX: written since last transmit;
    /// RX: updated since last take).
    sig_updated: [bool; MAX_SIGNALS],
    /// Per-signal invalid markers.
    sig_invalid: [bool; MAX_SIGNALS],
    /// Bounded event queue (ring buffer, drop-newest).
    events: [Option<ComEvent>; EVENT_QUEUE_CAP],
    /// Ring-buffer head index.
    event_head: usize,
    /// Number of queued events.
    event_len: usize,
    /// Events dropped because the queue was full (saturating).
    events_dropped: u32,
    /// Whether `start` has been called.
    started: bool,
}

// Const-default sentinel PDU/signal for array initialisation.
const SENTINEL_PDU: PduDescriptor = PduDescriptor::tx(0, 0, TxMode::EventOnly);

use crate::signal::{ByteOrder, SignalType};
const SENTINEL_SIGNAL: SignalDescriptor = SignalDescriptor::new(
    u16::MAX,
    0,
    8,
    SignalType::Uint8,
    ByteOrder::LittleEndian,
    0,
);

impl<const MAX_PDUS: usize, const MAX_SIGNALS: usize> ComManager<MAX_PDUS, MAX_SIGNALS> {
    // ---------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------

    /// Create an empty COM manager (all-zero state).
    pub const fn new() -> Self {
        Self {
            pdus: [SENTINEL_PDU; MAX_PDUS],
            signals: [SENTINEL_SIGNAL; MAX_SIGNALS],
            pdu_count: 0,
            signal_count: 0,
            pdu_buffers: [[0u8; PDU_BUFFER_LEN]; MAX_PDUS],
            tx_next: [Instant::from_nanos(0); MAX_PDUS],
            tx_overruns: [0u32; MAX_PDUS],
            tx_event_pending: [false; MAX_PDUS],
            tx_throttle_reported: [false; MAX_PDUS],
            tx_last_event: [None; MAX_PDUS],
            rx_deadline: [None; MAX_PDUS],
            rx_received: [false; MAX_PDUS],
            tx_ready: [false; MAX_PDUS],
            sig_updated: [false; MAX_SIGNALS],
            sig_invalid: [false; MAX_SIGNALS],
            events: [None; EVENT_QUEUE_CAP],
            event_head: 0,
            event_len: 0,
            events_dropped: 0,
            started: false,
        }
    }

    // ---------------------------------------------------------------------------
    // Registration
    // ---------------------------------------------------------------------------

    /// Register a PDU together with its signals.
    ///
    /// Returns the assigned PDU index (0-based) wrapped in `Some`, or `None`
    /// if:
    ///
    /// - the manager was already [`start`](Self::start)ed (configuration is
    ///   static once scheduling runs);
    /// - there is no room for the PDU or its signals;
    /// - `pdu.length` exceeds the 64-byte CAN FD maximum;
    /// - a cyclic/mixed TX PDU has a zero period, or an RX PDU has a zero
    ///   `rx_timeout` (both would be due forever);
    /// - any signal descriptor fails [`SignalDescriptor::validate`] against
    ///   `pdu.length` (E32: invalid layouts are rejected up front so later
    ///   shadow-buffer pack/unpack cannot fail).
    ///
    /// The signals slice is appended to the internal signal table in order;
    /// `pdu.signal_start_index` is **overwritten** with the actual start index.
    pub fn add_pdu(&mut self, mut pdu: PduDescriptor, signals: &[SignalDescriptor]) -> Option<u16> {
        if self.started {
            return None;
        }
        if self.pdu_count >= MAX_PDUS {
            return None;
        }
        if self.signal_count + signals.len() > MAX_SIGNALS {
            return None;
        }
        let pdu_len = usize::from(pdu.length);
        if pdu_len > PDU_BUFFER_LEN {
            return None;
        }
        match pdu.direction {
            PduDirection::Tx => {
                if pdu.tx_mode.period() == Some(Duration::ZERO) {
                    return None;
                }
            }
            PduDirection::Rx => {
                if pdu.rx_timeout == Some(Duration::ZERO) {
                    return None;
                }
            }
        }
        for sig in signals {
            if sig.validate(pdu_len).is_err() {
                return None;
            }
        }

        let pdu_index = self.pdu_count;
        let sig_start = self.signal_count;

        // Append signals (caller provides real IDs — no re-mapping needed).
        for sig in signals {
            self.signals[self.signal_count] = *sig;
            self.signal_count += 1;
        }

        // Rewrite signal_start_index and signal_count to actual positions.
        pdu.signal_start_index = sig_start as u16;
        pdu.signal_count = signals.len() as u8;

        // Apply signal init values into the shadow buffer.  Descriptors
        // were validated against pdu_len above, so packing cannot fail.
        for i in sig_start..self.signal_count {
            let desc = &self.signals[i];
            let init = SignalValue::from_u32(desc.init_value, desc.signal_type);
            let _ = pack_signal(&mut self.pdu_buffers[pdu_index][..pdu_len], desc, init);
        }

        self.pdus[pdu_index] = pdu;
        self.pdu_count += 1;

        Some(pdu_index as u16)
    }

    // ---------------------------------------------------------------------------
    // Start
    // ---------------------------------------------------------------------------

    /// Start (or restart) COM scheduling at `now`.
    ///
    /// Anchors the cyclic TX grid at `now` (each cyclic/mixed PDU is due
    /// immediately on the first tick at or after `now`), arms the reception
    /// deadline of every monitored RX PDU at `now + rx_timeout`, and resets
    /// overrun counters, pending events, and throttle state.
    pub fn start(&mut self, now: Instant) {
        self.started = true;
        for i in 0..self.pdu_count {
            let pdu = self.pdus[i];
            match pdu.direction {
                PduDirection::Tx => {
                    self.tx_next[i] = now;
                    self.tx_overruns[i] = 0;
                    self.tx_event_pending[i] = false;
                    self.tx_throttle_reported[i] = false;
                    self.tx_last_event[i] = None;
                }
                PduDirection::Rx => {
                    self.rx_deadline[i] = pdu.rx_timeout.map(|t| now.wrapping_add(t));
                }
            }
        }
    }

    /// Whether [`start`](Self::start) has been called.
    #[must_use]
    pub const fn is_started(&self) -> bool {
        self.started
    }

    // ---------------------------------------------------------------------------
    // Signal access
    // ---------------------------------------------------------------------------

    /// Write `value` to the signal's shadow buffer slot inside its PDU.
    ///
    /// On success this also marks the signal updated ("written since last
    /// transmit" for TX signals), clears its invalid marker, and — when the
    /// signal's transfer property is [`TransferProperty::Triggered`] and its
    /// TX PDU's mode accepts events — requests an event transmission of the
    /// PDU on the next tick.
    ///
    /// Returns `true` if the signal was found and written, `false` otherwise.
    pub fn write_signal(&mut self, signal_id: u16, value: SignalValue) -> bool {
        let Some(sig_idx) = self.find_signal_index(signal_id) else {
            return false;
        };
        let desc = self.signals[sig_idx];

        let Some(pdu_idx) = self.pdu_for_signal(sig_idx) else {
            return false;
        };
        let pdu = self.pdus[pdu_idx];

        let pdu_len = usize::from(pdu.length).min(PDU_BUFFER_LEN);
        if pack_signal(&mut self.pdu_buffers[pdu_idx][..pdu_len], &desc, value).is_err() {
            return false;
        }
        self.sig_updated[sig_idx] = true;
        self.sig_invalid[sig_idx] = false;
        if desc.transfer == TransferProperty::Triggered
            && pdu.direction == PduDirection::Tx
            && pdu.tx_mode.accepts_events()
        {
            self.tx_event_pending[pdu_idx] = true;
        }
        true
    }

    /// Read the current shadow-buffer value of a signal (non-consuming —
    /// update flags are not touched).
    ///
    /// Returns `None` if no signal with `signal_id` is registered.
    pub fn read_signal(&self, signal_id: u16) -> Option<SignalValue> {
        let sig_idx = self.find_signal_index(signal_id)?;
        let desc = &self.signals[sig_idx];
        let pdu_idx = self.pdu_for_signal(sig_idx)?;
        let pdu_len = usize::from(self.pdus[pdu_idx].length).min(PDU_BUFFER_LEN);
        unpack_signal(&self.pdu_buffers[pdu_idx][..pdu_len], desc).ok()
    }

    // ---------------------------------------------------------------------------
    // Update flags
    // ---------------------------------------------------------------------------

    /// Read-and-clear: return the signal's value if its update flag is set,
    /// clearing the flag.
    ///
    /// Intended for RX signals ("updated since last take"); calling it on a
    /// TX signal consumes that signal's "written since last transmit" flag.
    /// Returns `None` for unknown IDs and for signals that are not updated.
    pub fn take_updated(&mut self, signal_id: u16) -> Option<SignalValue> {
        let sig_idx = self.find_signal_index(signal_id)?;
        if !self.sig_updated[sig_idx] {
            return None;
        }
        let desc = self.signals[sig_idx];
        let pdu_idx = self.pdu_for_signal(sig_idx)?;
        let pdu_len = usize::from(self.pdus[pdu_idx].length).min(PDU_BUFFER_LEN);
        let value = unpack_signal(&self.pdu_buffers[pdu_idx][..pdu_len], &desc).ok()?;
        self.sig_updated[sig_idx] = false;
        Some(value)
    }

    /// Non-consuming peek at a signal's update flag.
    ///
    /// Returns `None` if no signal with `signal_id` is registered.
    #[must_use]
    pub fn is_signal_updated(&self, signal_id: u16) -> Option<bool> {
        let sig_idx = self.find_signal_index(signal_id)?;
        Some(self.sig_updated[sig_idx])
    }

    /// Read-and-clear the per-PDU "received since last check" indication.
    ///
    /// Returns `None` if `pdu_index` names no registered PDU.
    pub fn take_pdu_received(&mut self, pdu_index: u16) -> Option<bool> {
        let i = usize::from(pdu_index);
        if i >= self.pdu_count {
            return None;
        }
        let flag = self.rx_received[i];
        self.rx_received[i] = false;
        Some(flag)
    }

    // ---------------------------------------------------------------------------
    // Project-specific E2E extension
    // ---------------------------------------------------------------------------

    /// Stamp the configured TX PDU using the project-specific CRC/counter
    /// extension before it is returned by [`tick`](Self::tick).
    ///
    /// This is not an AUTOSAR E2E profile and does not use a Data ID.
    pub fn protect_project_e2e(
        &mut self,
        pdu_index: u16,
        protector: &mut E2eProtector,
    ) -> Result<(), ComE2eError> {
        let i = usize::from(pdu_index);
        if i >= self.pdu_count {
            return Err(ComE2eError::UnknownPdu);
        }
        let pdu = self.pdus[i];
        if pdu.direction != PduDirection::Tx {
            return Err(ComE2eError::WrongDirection);
        }
        let len = usize::from(pdu.length);
        protector
            .protect_checked(&mut self.pdu_buffers[i][..len], len)
            .map_err(ComE2eError::Codec)
    }

    /// Validate a protected RX PDU before accepting it into COM state.
    ///
    /// `Initial` and `Ok` frames are passed to normal receive processing,
    /// including update flags and RX-deadline re-arming. CRC, counter, and
    /// replay failures invalidate the PDU's signals and are not accepted.
    pub fn receive_project_e2e(
        &mut self,
        can_id: u32,
        data: &[u8],
        now: Instant,
        checker: &mut E2eChecker,
    ) -> Result<E2eResult, ComE2eError> {
        let Some(i) = (0..self.pdu_count).find(|&index| self.pdus[index].can_id == can_id) else {
            return Err(ComE2eError::UnknownPdu);
        };
        let pdu = self.pdus[i];
        if pdu.direction != PduDirection::Rx {
            return Err(ComE2eError::WrongDirection);
        }
        let len = usize::from(pdu.length);
        if data.len() != len {
            return Err(ComE2eError::LengthMismatch);
        }
        let status = checker
            .check_checked(data, len)
            .map_err(ComE2eError::Codec)?;
        if matches!(status, E2eResult::Initial | E2eResult::Ok) {
            self.receive(can_id, data, now);
        } else {
            let start = usize::from(pdu.signal_start_index);
            let end = start + usize::from(pdu.signal_count);
            self.sig_invalid[start..end].fill(true);
        }
        Ok(status)
    }

    // ---------------------------------------------------------------------------
    // Invalid flags
    // ---------------------------------------------------------------------------

    /// Mark a signal invalid.  The marker is cleared by the next reception
    /// of the signal's PDU or by a successful
    /// [`write_signal`](Self::write_signal).
    ///
    /// Returns `true` if the signal exists.
    pub fn invalidate_signal(&mut self, signal_id: u16) -> bool {
        let Some(sig_idx) = self.find_signal_index(signal_id) else {
            return false;
        };
        self.sig_invalid[sig_idx] = true;
        true
    }

    /// Whether a signal is currently valid (not marked invalid).
    ///
    /// Signals become invalid via [`invalidate_signal`](Self::invalidate_signal)
    /// or when an RX deadline miss applies [`RxTimeoutAction::RestoreInit`].
    /// Returns `None` if no signal with `signal_id` is registered.
    #[must_use]
    pub fn is_signal_valid(&self, signal_id: u16) -> Option<bool> {
        let sig_idx = self.find_signal_index(signal_id)?;
        Some(!self.sig_invalid[sig_idx])
    }

    // ---------------------------------------------------------------------------
    // Event triggering
    // ---------------------------------------------------------------------------

    /// Explicitly request an event transmission of a TX PDU on the next
    /// tick (subject to `min_event_interval` throttling).
    ///
    /// Returns `true` if `pdu_index` names a TX PDU whose mode accepts
    /// events ([`TxMode::EventOnly`] or [`TxMode::Mixed`]); `false` for
    /// unknown indices, RX PDUs, and [`TxMode::Cyclic`] PDUs.
    pub fn trigger(&mut self, pdu_index: u16) -> bool {
        let i = usize::from(pdu_index);
        if i >= self.pdu_count {
            return false;
        }
        let pdu = self.pdus[i];
        if pdu.direction != PduDirection::Tx || !pdu.tx_mode.accepts_events() {
            return false;
        }
        self.tx_event_pending[i] = true;
        true
    }

    // ---------------------------------------------------------------------------
    // Tick
    // ---------------------------------------------------------------------------

    /// Advance the COM scheduler to `now`.
    ///
    /// Polls RX deadlines first (emitting [`ComEvent::RxDeadlineMissed`] and
    /// applying the configured timeout action), then schedules TX PDUs
    /// (cyclic grid + pending events, honoring `min_event_interval`).
    ///
    /// Before [`start`](Self::start), this yields nothing and monitors
    /// nothing.
    ///
    /// Returns a [`ComTxIterator`] over all PDUs due for transmission on
    /// this tick, in registration order.  **Consume the iterator before the
    /// next call to `tick`.**
    pub fn tick(&mut self, now: Instant) -> ComTxIterator<'_> {
        for flag in &mut self.tx_ready[..self.pdu_count] {
            *flag = false;
        }

        if self.started {
            self.poll_rx_deadlines(now);
            self.schedule_tx(now);
        }

        ComTxIterator {
            pdus: &self.pdus,
            buffers: &self.pdu_buffers,
            ready: &self.tx_ready,
            sig_updated: &mut self.sig_updated,
            pdu_count: self.pdu_count,
            index: 0,
        }
    }

    /// Check armed RX deadlines against `now`.
    fn poll_rx_deadlines(&mut self, now: Instant) {
        for i in 0..self.pdu_count {
            let pdu = self.pdus[i];
            if pdu.direction != PduDirection::Rx {
                continue;
            }
            let Some(deadline) = self.rx_deadline[i] else {
                continue;
            };
            if !now.is_at_or_after(deadline) {
                continue;
            }
            // Miss: disarm until the next reception re-arms the deadline,
            // so each miss is reported exactly once.
            self.rx_deadline[i] = None;
            self.push_event(ComEvent::RxDeadlineMissed {
                can_id: pdu.can_id,
                at: deadline,
            });
            if pdu.timeout_action == RxTimeoutAction::RestoreInit {
                self.restore_init(i);
            }
        }
    }

    /// Apply init values to an RX PDU's shadow buffer and mark its signals
    /// invalid (timeout-default `RestoreInit`).
    fn restore_init(&mut self, pdu_idx: usize) {
        let pdu = self.pdus[pdu_idx];
        let pdu_len = usize::from(pdu.length);
        let start = usize::from(pdu.signal_start_index);
        let end = start + usize::from(pdu.signal_count);
        for sig_idx in start..end {
            let desc = self.signals[sig_idx];
            let init = SignalValue::from_u32(desc.init_value, desc.signal_type);
            let _ = pack_signal(&mut self.pdu_buffers[pdu_idx][..pdu_len], &desc, init);
            self.sig_invalid[sig_idx] = true;
        }
    }

    /// Mark due TX PDUs ready: drift-free cyclic grid plus pending events.
    fn schedule_tx(&mut self, now: Instant) {
        for i in 0..self.pdu_count {
            let pdu = self.pdus[i];
            if pdu.direction != PduDirection::Tx {
                continue;
            }
            let mut due = false;

            // Cyclic grid: fire once, then advance past every deadline that
            // is already due; each skipped deadline is one overrun.
            if let Some(period) = pdu.tx_mode.period() {
                if now.is_at_or_after(self.tx_next[i]) {
                    due = true;
                    self.tx_next[i] = self.tx_next[i].wrapping_add(period);
                    let mut missed = 0u32;
                    while now.is_at_or_after(self.tx_next[i]) {
                        missed = missed.saturating_add(1);
                        self.tx_next[i] = self.tx_next[i].wrapping_add(period);
                    }
                    if missed > 0 {
                        self.tx_overruns[i] = self.tx_overruns[i].saturating_add(missed);
                        self.push_event(ComEvent::TxOverrun {
                            can_id: pdu.can_id,
                            missed,
                        });
                    }
                }
            }

            // Pending event: fold into a coinciding cyclic transmission,
            // otherwise transmit unless throttled by min_event_interval.
            if pdu.tx_mode.accepts_events() && self.tx_event_pending[i] {
                if due {
                    self.tx_event_pending[i] = false;
                    self.tx_throttle_reported[i] = false;
                } else if self.event_throttled(i, now) {
                    if !self.tx_throttle_reported[i] {
                        self.tx_throttle_reported[i] = true;
                        self.push_event(ComEvent::EventTxThrottled { can_id: pdu.can_id });
                    }
                } else {
                    due = true;
                    self.tx_event_pending[i] = false;
                    self.tx_throttle_reported[i] = false;
                }
            }

            if due {
                self.tx_ready[i] = true;
                if pdu.tx_mode.accepts_events() {
                    self.tx_last_event[i] = Some(now);
                }
            }
        }
    }

    /// Whether an event transmission of PDU `i` at `now` violates its
    /// minimum event interval.
    fn event_throttled(&self, i: usize, now: Instant) -> bool {
        let interval = self.pdus[i].min_event_interval;
        if interval == Duration::ZERO {
            return false;
        }
        match self.tx_last_event[i] {
            Some(last) => !now.is_at_or_after(last.wrapping_add(interval)),
            None => false,
        }
    }

    // ---------------------------------------------------------------------------
    // RX processing
    // ---------------------------------------------------------------------------

    /// Process a received CAN frame at `now`: copy the payload into the
    /// matching RX PDU's shadow buffer, mark the PDU's signals updated and
    /// valid, set the per-PDU received indication, and re-arm the reception
    /// deadline (`now + rx_timeout`, monitored PDUs after
    /// [`start`](Self::start) only).
    ///
    /// A payload longer than the registered PDU length is truncated and
    /// reported as [`ComEvent::BufferOverflow`].  Does nothing if no
    /// registered RX PDU has `can_id`.
    pub fn receive(&mut self, can_id: u32, data: &[u8], now: Instant) {
        for i in 0..self.pdu_count {
            let pdu = self.pdus[i];
            if pdu.can_id != can_id || pdu.direction != PduDirection::Rx {
                continue;
            }
            let pdu_len = usize::from(pdu.length);
            if data.len() > pdu_len {
                self.push_event(ComEvent::BufferOverflow { can_id });
            }
            let copy_len = data.len().min(pdu_len).min(PDU_BUFFER_LEN);
            self.pdu_buffers[i][..copy_len].copy_from_slice(&data[..copy_len]);
            self.rx_received[i] = true;

            let start = usize::from(pdu.signal_start_index);
            let end = start + usize::from(pdu.signal_count);
            for (updated, invalid) in self.sig_updated[start..end]
                .iter_mut()
                .zip(self.sig_invalid[start..end].iter_mut())
            {
                *updated = true;
                *invalid = false;
            }

            if self.started {
                if let Some(timeout) = pdu.rx_timeout {
                    self.rx_deadline[i] = Some(now.wrapping_add(timeout));
                }
            }
            return;
        }
    }

    // ---------------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------------

    /// Drain the oldest queued [`ComEvent`], if any (FIFO).
    pub fn take_event(&mut self) -> Option<ComEvent> {
        if self.event_len == 0 {
            return None;
        }
        let event = self.events[self.event_head].take();
        self.event_head = (self.event_head + 1) % EVENT_QUEUE_CAP;
        self.event_len -= 1;
        event
    }

    /// Number of events dropped because the queue was full (saturating).
    #[must_use]
    pub const fn events_dropped(&self) -> u32 {
        self.events_dropped
    }

    /// Cumulative missed cyclic periods of a TX PDU since the last
    /// [`start`](Self::start).
    ///
    /// Returns `None` if `pdu_index` names no registered PDU (RX PDUs
    /// report 0).
    #[must_use]
    pub fn tx_overruns(&self, pdu_index: u16) -> Option<u32> {
        let i = usize::from(pdu_index);
        if i >= self.pdu_count {
            return None;
        }
        Some(self.tx_overruns[i])
    }

    // ---------------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------------

    /// Queue an event; drop-newest with a saturating counter when full.
    fn push_event(&mut self, event: ComEvent) {
        if self.event_len == EVENT_QUEUE_CAP {
            self.events_dropped = self.events_dropped.saturating_add(1);
            return;
        }
        let tail = (self.event_head + self.event_len) % EVENT_QUEUE_CAP;
        self.events[tail] = Some(event);
        self.event_len += 1;
    }

    fn find_signal_index(&self, signal_id: u16) -> Option<usize> {
        (0..self.signal_count).find(|&i| self.signals[i].id == signal_id)
    }

    fn pdu_for_signal(&self, sig_idx: usize) -> Option<usize> {
        for p in 0..self.pdu_count {
            let start = usize::from(self.pdus[p].signal_start_index);
            let end = start + usize::from(self.pdus[p].signal_count);
            if (start..end).contains(&sig_idx) {
                return Some(p);
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<const MAX_PDUS: usize, const MAX_SIGNALS: usize> Default
    for ComManager<MAX_PDUS, MAX_SIGNALS>
{
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{ComEvent, ComManager};
    use crate::pdu::{PduDescriptor, RxTimeoutAction, TxMode};
    use crate::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
    use bsw_time::{Duration, Instant};

    const fn ms(value: u64) -> Duration {
        Duration::from_nanos(value * 1_000_000)
    }

    const fn at_ms(value: u64) -> Instant {
        Instant::from_nanos(value * 1_000_000)
    }

    // Helper: build a standard 8-byte cyclic TX PDU descriptor.
    fn tx_pdu(can_id: u32, cycle_ms: u64) -> PduDescriptor {
        PduDescriptor::tx(can_id, 8, TxMode::Cyclic(ms(cycle_ms)))
    }

    fn rx_pdu(can_id: u32) -> PduDescriptor {
        PduDescriptor::rx(can_id, 8)
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

    fn u16_signal(id: u16, byte: u16) -> SignalDescriptor {
        SignalDescriptor::new(
            id,
            byte * 8,
            16,
            SignalType::Uint16,
            ByteOrder::LittleEndian,
            0,
        )
    }

    fn bool_signal(id: u16, byte: u16, init: u32) -> SignalDescriptor {
        SignalDescriptor::new(
            id,
            byte * 8,
            8,
            SignalType::Boolean,
            ByteOrder::LittleEndian,
            init,
        )
    }

    // 1 — write/read round-trip for u8 signal
    #[test]
    fn write_read_u8_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(1, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs);
        assert!(com.write_signal(1, SignalValue::U8(0x42)));
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0x42)));
    }

    // 2 — write/read round-trip for u16 signal
    #[test]
    fn write_read_u16_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u16_signal(2, 0)];
        com.add_pdu(tx_pdu(0x200, 20), &sigs);
        assert!(com.write_signal(2, SignalValue::U16(0xABCD)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U16(0xABCD)));
    }

    // 3 — signal init values are applied at add_pdu time
    #[test]
    fn signal_init_value_applied() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(3, 2, 0xFF)];
        com.add_pdu(tx_pdu(0x300, 5), &sigs);
        assert_eq!(com.read_signal(3), Some(SignalValue::U8(0xFF)));
    }

    // 4 — unknown signal ID returns None on read
    #[test]
    fn unknown_signal_read_returns_none() {
        let com: ComManager<4, 16> = ComManager::new();
        assert_eq!(com.read_signal(999), None);
    }

    // 5 — unknown signal ID returns false on write
    #[test]
    fn unknown_signal_write_returns_false() {
        let mut com: ComManager<4, 16> = ComManager::new();
        assert!(!com.write_signal(999, SignalValue::U8(0)));
    }

    // 6 — cyclic TX: PDU not ready before deadline
    #[test]
    fn cyclic_tx_not_ready_before_deadline() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(10, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs); // cycle = 10 ms
        com.start(at_ms(0));

        // The grid is anchored at start: the first tick fires immediately.
        assert_eq!(com.tick(at_ms(0)).count(), 1);
        // Now the deadline is 10 ms.  At t=5 ms it should NOT be ready.
        assert_eq!(com.tick(at_ms(5)).count(), 0);
    }

    // 7 — cyclic TX: PDU ready at/after deadline
    #[test]
    fn cyclic_tx_ready_at_deadline() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(10, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs);
        com.start(at_ms(0));

        assert_eq!(com.tick(at_ms(0)).count(), 1); // first fire, deadline → 10
        let mut it = com.tick(at_ms(10));
        let (can_id, _) = it.next().unwrap();
        assert_eq!(can_id, 0x100);
        assert!(it.next().is_none());
    }

    // 8 — TX iterator returns correct CAN ID and data
    #[test]
    fn tx_iterator_data_correct() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(20, 0, 0xAB)];
        com.add_pdu(tx_pdu(0x123, 10), &sigs);
        com.start(at_ms(0));
        assert_eq!(com.tick(at_ms(0)).count(), 1); // first fire
        let mut it = com.tick(at_ms(10));
        let (can_id, data) = it.next().unwrap();
        assert_eq!(can_id, 0x123);
        assert_eq!(data[0], 0xAB);
        assert!(it.next().is_none());
    }

    // 9 — receive updates shadow buffer
    #[test]
    fn receive_updates_shadow_buffer() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(30, 0, 0)];
        com.add_pdu(rx_pdu(0x400), &sigs);
        com.receive(0x400, &[0x55, 0, 0, 0, 0, 0, 0, 0], at_ms(100));
        assert_eq!(com.read_signal(30), Some(SignalValue::U8(0x55)));
    }

    // 10 — receive on unknown CAN ID does nothing (no panic)
    #[test]
    fn receive_unknown_can_id_no_panic() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.receive(0xDEAD, &[1, 2, 3, 4, 5, 6, 7, 8], at_ms(0)); // must not panic
    }

    // 11 — RX deadline miss emits an event
    #[test]
    fn rx_deadline_miss_emits_event() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(40, 0, 0)];
        com.add_pdu(
            rx_pdu(0x500).with_rx_timeout(ms(100), RxTimeoutAction::KeepLast),
            &sigs,
        );
        com.start(at_ms(0));
        // Receive at t=0 → deadline 100 ms; tick at t=150 ms → missed.
        com.receive(0x500, &[1, 0, 0, 0, 0, 0, 0, 0], at_ms(0));
        assert_eq!(com.tick(at_ms(150)).count(), 0);
        assert_eq!(
            com.take_event(),
            Some(ComEvent::RxDeadlineMissed {
                can_id: 0x500,
                at: at_ms(100),
            })
        );
        assert_eq!(com.take_event(), None);
    }

    // 12 — RX deadline NOT missed within the window
    #[test]
    fn rx_no_timeout_within_window() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(41, 0, 0)];
        com.add_pdu(
            rx_pdu(0x501).with_rx_timeout(ms(100), RxTimeoutAction::KeepLast),
            &sigs,
        );
        com.start(at_ms(0));
        com.receive(0x501, &[1, 0, 0, 0, 0, 0, 0, 0], at_ms(100));
        assert_eq!(com.tick(at_ms(150)).count(), 0);
        assert_eq!(com.take_event(), None);
    }

    // 13 — PDU capacity overflow returns None
    #[test]
    fn pdu_capacity_overflow() {
        let mut com: ComManager<2, 32> = ComManager::new();
        assert!(com
            .add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)])
            .is_some());
        assert!(com
            .add_pdu(tx_pdu(0x200, 20), &[u8_signal(2, 0, 0)])
            .is_some());
        // 3rd PDU exceeds MAX_PDUS=2.
        assert!(com
            .add_pdu(tx_pdu(0x300, 30), &[u8_signal(3, 0, 0)])
            .is_none());
    }

    // 14 — multiple signals in one PDU, all round-trip correctly
    #[test]
    fn multiple_signals_same_pdu_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(50, 0, 0), u8_signal(51, 1, 0), u16_signal(52, 2)];
        com.add_pdu(tx_pdu(0x600, 10), &sigs);

        com.write_signal(50, SignalValue::U8(0x11));
        com.write_signal(51, SignalValue::U8(0x22));
        com.write_signal(52, SignalValue::U16(0x3344));

        assert_eq!(com.read_signal(50), Some(SignalValue::U8(0x11)));
        assert_eq!(com.read_signal(51), Some(SignalValue::U8(0x22)));
        assert_eq!(com.read_signal(52), Some(SignalValue::U16(0x3344)));
    }

    // 15 — boolean signal init value and round-trip
    #[test]
    fn boolean_signal_init_and_write() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [bool_signal(60, 0, 1)]; // init = true
        com.add_pdu(tx_pdu(0x700, 10), &sigs);
        assert_eq!(com.read_signal(60), Some(SignalValue::Bool(true)));

        com.write_signal(60, SignalValue::Bool(false));
        assert_eq!(com.read_signal(60), Some(SignalValue::Bool(false)));
    }

    // 16 — signal capacity overflow returns None
    #[test]
    fn signal_capacity_overflow() {
        // MAX_SIGNALS = 2; try to add 3 signals in one PDU.
        let mut com: ComManager<4, 2> = ComManager::new();
        let sigs = [u8_signal(1, 0, 0), u8_signal(2, 1, 0), u8_signal(3, 2, 0)];
        assert!(com.add_pdu(tx_pdu(0x100, 10), &sigs).is_none());
    }

    // 17 — two PDUs on different CAN IDs are independent
    #[test]
    fn two_pdus_independent() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0xAA)]);
        com.add_pdu(tx_pdu(0x200, 20), &[u8_signal(2, 0, 0xBB)]);
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0xAA)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U8(0xBB)));

        com.write_signal(1, SignalValue::U8(0x11));
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0x11)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U8(0xBB))); // unchanged
    }

    // 18 — CAN FD PDU (64 bytes): signal in the upper payload round-trips
    #[test]
    fn fd_pdu_signal_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let pdu = PduDescriptor::tx(0x800, 64, TxMode::Cyclic(ms(10)));
        // u16 signal in byte 60/61 of a 64-byte FD payload.
        let sigs = [u16_signal(80, 60)];
        assert!(com.add_pdu(pdu, &sigs).is_some());
        assert!(com.write_signal(80, SignalValue::U16(0xBEEF)));
        assert_eq!(com.read_signal(80), Some(SignalValue::U16(0xBEEF)));

        // TX data slice carries the full 64-byte payload.
        com.start(at_ms(0));
        let mut it = com.tick(at_ms(0));
        let (can_id, data) = it.next().unwrap();
        assert_eq!(can_id, 0x800);
        assert_eq!(data.len(), 64);
        assert_eq!(&data[60..62], &[0xEF, 0xBE]);
        assert!(it.next().is_none());
    }

    // 19 — FD RX: receive copies payload bytes beyond the classic 8
    #[test]
    fn fd_receive_updates_upper_bytes() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let pdu = PduDescriptor::rx(0x801, 64);
        let sigs = [u8_signal(81, 63, 0)];
        assert!(com.add_pdu(pdu, &sigs).is_some());
        let mut data = [0u8; 64];
        data[63] = 0x7E;
        com.receive(0x801, &data, at_ms(5));
        assert_eq!(com.read_signal(81), Some(SignalValue::U8(0x7E)));
    }

    // 20 — add_pdu rejects a signal that does not fit the PDU length
    #[test]
    fn add_pdu_rejects_out_of_range_signal() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let pdu = PduDescriptor::tx(0x900, 4, TxMode::EventOnly);
        // Signal at byte 6 does not fit a 4-byte PDU.
        assert!(com.add_pdu(pdu, &[u8_signal(90, 6, 0)]).is_none());
        assert_eq!(com.read_signal(90), None);
    }

    // 21 — add_pdu rejects invalid descriptors and oversized PDU lengths
    #[test]
    fn add_pdu_rejects_invalid_descriptor_and_length() {
        let mut com: ComManager<4, 16> = ComManager::new();
        // Zero-width signal descriptor.
        let bad_sig =
            SignalDescriptor::new(91, 0, 0, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        assert!(com.add_pdu(tx_pdu(0x901, 10), &[bad_sig]).is_none());
        // PDU length above the 64-byte CAN FD maximum.
        let pdu = PduDescriptor::tx(0x902, 65, TxMode::EventOnly);
        assert!(com.add_pdu(pdu, &[u8_signal(92, 0, 0)]).is_none());
    }

    // 22 — add_pdu rejects zero cyclic periods and zero RX timeouts
    #[test]
    fn add_pdu_rejects_zero_period_and_zero_timeout() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let pdu = PduDescriptor::tx(0x903, 8, TxMode::Cyclic(Duration::ZERO));
        assert!(com.add_pdu(pdu, &[u8_signal(93, 0, 0)]).is_none());
        let pdu = PduDescriptor::tx(0x904, 8, TxMode::Mixed(Duration::ZERO));
        assert!(com.add_pdu(pdu, &[u8_signal(94, 0, 0)]).is_none());
        let pdu =
            PduDescriptor::rx(0x905, 8).with_rx_timeout(Duration::ZERO, RxTimeoutAction::KeepLast);
        assert!(com.add_pdu(pdu, &[u8_signal(95, 0, 0)]).is_none());
    }

    // 23 — add_pdu is rejected after start (static configuration)
    #[test]
    fn add_pdu_rejected_after_start() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.start(at_ms(0));
        assert!(com
            .add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)])
            .is_none());
    }

    // 24 — trigger validates direction and mode
    #[test]
    fn trigger_validates_direction_and_mode() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let cyclic = com
            .add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)])
            .unwrap();
        let event = com
            .add_pdu(
                PduDescriptor::tx(0x200, 8, TxMode::EventOnly),
                &[u8_signal(2, 0, 0)],
            )
            .unwrap();
        let rx = com.add_pdu(rx_pdu(0x300), &[u8_signal(3, 0, 0)]).unwrap();
        assert!(!com.trigger(cyclic)); // Cyclic mode ignores events
        assert!(com.trigger(event));
        assert!(!com.trigger(rx)); // RX PDUs cannot be triggered
        assert!(!com.trigger(99)); // unknown index
    }

    // 25 — tick before start yields nothing and monitors nothing
    #[test]
    fn tick_before_start_yields_nothing() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)]);
        com.add_pdu(
            rx_pdu(0x500).with_rx_timeout(ms(10), RxTimeoutAction::KeepLast),
            &[u8_signal(2, 0, 0)],
        );
        assert_eq!(com.tick(at_ms(1_000)).count(), 0);
        assert_eq!(com.take_event(), None);
        assert!(!com.is_started());
    }

    // 26 — take_event on an empty queue returns None
    #[test]
    fn take_event_empty_returns_none() {
        let mut com: ComManager<4, 16> = ComManager::new();
        assert_eq!(com.take_event(), None);
        assert_eq!(com.events_dropped(), 0);
    }

    // 27 — invalidate_signal / is_signal_valid round-trip
    #[test]
    fn invalidate_and_revalidate_signal() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(rx_pdu(0x400), &[u8_signal(30, 0, 0)]);
        assert_eq!(com.is_signal_valid(30), Some(true));
        assert!(com.invalidate_signal(30));
        assert_eq!(com.is_signal_valid(30), Some(false));
        // Reception clears the invalid marker.
        com.receive(0x400, &[9, 0, 0, 0, 0, 0, 0, 0], at_ms(0));
        assert_eq!(com.is_signal_valid(30), Some(true));
        // Unknown signals report None / false.
        assert_eq!(com.is_signal_valid(999), None);
        assert!(!com.invalidate_signal(999));
    }

    // 28 — write_signal clears the invalid marker
    #[test]
    fn write_signal_clears_invalid_marker() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)]);
        assert!(com.invalidate_signal(1));
        assert_eq!(com.is_signal_valid(1), Some(false));
        assert!(com.write_signal(1, SignalValue::U8(7)));
        assert_eq!(com.is_signal_valid(1), Some(true));
    }

    // 29 — take_pdu_received is read-and-clear
    #[test]
    fn take_pdu_received_read_and_clear() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let rx = com.add_pdu(rx_pdu(0x400), &[u8_signal(30, 0, 0)]).unwrap();
        assert_eq!(com.take_pdu_received(rx), Some(false));
        com.receive(0x400, &[1, 0, 0, 0, 0, 0, 0, 0], at_ms(0));
        assert_eq!(com.take_pdu_received(rx), Some(true));
        assert_eq!(com.take_pdu_received(rx), Some(false));
        assert_eq!(com.take_pdu_received(99), None);
    }

    // 30 — over-long RX payload is truncated and reported
    #[test]
    fn buffer_overflow_event_on_long_frame() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(rx_pdu(0x400), &[u8_signal(30, 0, 0)]);
        com.receive(0x400, &[0xAA, 0, 0, 0, 0, 0, 0, 0, 0xFF], at_ms(0));
        assert_eq!(com.read_signal(30), Some(SignalValue::U8(0xAA)));
        assert_eq!(
            com.take_event(),
            Some(ComEvent::BufferOverflow { can_id: 0x400 })
        );
    }
}
