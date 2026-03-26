// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! CAN transceiver traits and abstract base implementation.
//!
//! Provides:
//! - [`ErrorCode`] — operation result codes
//! - [`State`] — logical CAN bus state machine
//! - [`TransceiverState`] — low-level hardware bus state
//! - [`Statistics`] — runtime counters
//! - [`CanTransceiver`] — pure hardware-interface trait
//! - [`AbstractTransceiver`] — hardware-independent base with embedded state
//!   machine, filter, and statistics management

use crate::filter::BitFieldFilter;
use crate::frame::CanFrame;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default receive queue capacity (number of frames).
pub const RX_QUEUE_SIZE: usize = 32;

/// Low-speed CAN baudrate: 100 kbit/s.
pub const BAUDRATE_LOWSPEED: u32 = 100_000;

/// High-speed CAN baudrate: 500 kbit/s.
pub const BAUDRATE_HIGHSPEED: u32 = 500_000;

/// Sentinel value indicating an absent or unassigned frame ID.
pub const INVALID_FRAME_ID: u32 = 0xFFFF;

// ---------------------------------------------------------------------------
// ErrorCode
// ---------------------------------------------------------------------------

/// Result codes returned by CAN transceiver operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ErrorCode {
    /// Operation completed successfully.
    #[default]
    Ok,
    /// Frame could not be transmitted (generic TX failure).
    TxFail,
    /// Transmit hardware queue is full.
    TxHwQueueFull,
    /// Transmit attempted while transceiver is offline / closed.
    TxOffline,
    /// Operation is not valid in the current state.
    IllegalState,
    /// No more listener slots available.
    NoMoreListenersPossible,
    /// Requested baudrate is not supported by hardware.
    UnsupportedBaudrate,
    /// Hardware initialisation failed.
    InitFailed,
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// Logical CAN transceiver state.
///
/// The normal lifecycle is:
/// `Closed` → `Initialized` → `Open` ⇌ `Muted`
/// and any state → `Closed` on shutdown.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum State {
    /// Transceiver has not been initialised. This is the power-on default.
    #[default]
    Closed,
    /// Hardware initialisation complete; bus not yet joined.
    Initialized,
    /// Waking from sleep / low-power state.
    Waking,
    /// Actively receiving and transmitting on the bus.
    Open,
    /// Reception enabled but transmission suppressed (silent / listen-only).
    Muted,
}

// ---------------------------------------------------------------------------
// TransceiverState
// ---------------------------------------------------------------------------

/// Hardware-level bus participation state reported by the CAN controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransceiverState {
    /// Controller is actively participating in bus arbitration.
    Active,
    /// Controller has elevated error counts; still participating.
    Passive,
    /// Controller has entered bus-off state; no longer transmitting.
    BusOff,
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/// Per-transceiver runtime traffic and error counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Statistics {
    /// Number of bus errors detected.
    pub errors: u32,
    /// Number of frames successfully received.
    pub rx: u32,
    /// Number of frames successfully transmitted.
    pub tx: u32,
    /// Number of received frames dropped (e.g. queue overflow).
    pub rx_dropped: u32,
    /// Number of transmit requests dropped (e.g. queue full).
    pub tx_dropped: u32,
}

// ---------------------------------------------------------------------------
// CanTransceiver trait
// ---------------------------------------------------------------------------

/// Pure hardware-interface trait for a CAN transceiver.
///
/// This maps directly to the `ICanTransceiver` pure-virtual C++ class.
/// Platform BSPs implement this trait to provide the actual hardware driver.
pub trait CanTransceiver {
    /// Initialises the hardware transceiver.
    fn init(&mut self) -> ErrorCode;

    /// Shuts down the hardware transceiver.
    fn shutdown(&mut self);

    /// Opens the transceiver for communication (transitions to [`State::Open`]).
    fn open(&mut self) -> ErrorCode;

    /// Opens the transceiver and queues an initial frame for transmission.
    fn open_with_frame(&mut self, frame: &CanFrame) -> ErrorCode;

    /// Closes the transceiver (transitions to [`State::Closed`]).
    fn close(&mut self) -> ErrorCode;

    /// Mutes the transceiver — reception continues, transmission is suppressed.
    fn mute(&mut self) -> ErrorCode;

    /// Unmutes the transceiver — resumes transmission.
    fn unmute(&mut self) -> ErrorCode;

    /// Returns the current logical [`State`].
    fn state(&self) -> State;

    /// Returns the configured baudrate in bits per second.
    fn baudrate(&self) -> u32;

    /// Returns the hardware transmit queue timeout in milliseconds.
    fn hw_queue_timeout(&self) -> u16;

    /// Returns the bus identifier (index within the system).
    fn bus_id(&self) -> u8;

    /// Writes a frame to the hardware transmit queue.
    fn write(&mut self, frame: &CanFrame) -> ErrorCode;

    /// Returns the low-level [`TransceiverState`] from the CAN controller.
    fn transceiver_state(&self) -> TransceiverState;
}

// ---------------------------------------------------------------------------
// Platform-abstraction seams
// ---------------------------------------------------------------------------

/// Abstraction for a system monotonic timer (BSP seam).
///
/// Implementations must return a monotonically increasing microsecond counter.
pub trait SystemTimer {
    /// Returns the current system time in microseconds.
    fn system_time_us(&self) -> u32;
}

/// Abstraction for entering/leaving a critical section (BSP seam).
///
/// Used to protect shared state from interrupt-context access.
pub trait InterruptLock {
    /// Enters the critical section (disables relevant interrupts).
    fn lock(&self);

    /// Leaves the critical section (re-enables interrupts).
    fn unlock(&self);
}

// ---------------------------------------------------------------------------
// AbstractTransceiver
// ---------------------------------------------------------------------------

/// Hardware-independent CAN transceiver base.
///
/// Manages:
/// - Logical state machine ([`State`])
/// - Hardware bus state ([`TransceiverState`])
/// - Receive ID filter ([`BitFieldFilter`])
/// - Runtime statistics ([`Statistics`])
/// - Bus and baudrate configuration
///
/// Concrete platform drivers embed this struct and delegate non-hardware
/// operations (state queries, filter management, statistics tracking) to it.
///
/// # Const generic
///
/// `MAX_LISTENERS` reserves slots for future listener management without heap
/// allocation.  Set to `8` for most embedded targets.
pub struct AbstractTransceiver<const MAX_LISTENERS: usize> {
    bus_id: u8,
    baudrate: u32,
    state: State,
    transceiver_state: TransceiverState,
    filter: BitFieldFilter,
    statistics: Statistics,
    // Listener count tracking (actual dispatch is handled by the platform
    // layer through callbacks; this only tracks how many are registered).
    listener_count: usize,
}

impl<const MAX_LISTENERS: usize> AbstractTransceiver<MAX_LISTENERS> {
    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /// Creates a new abstract transceiver for the given bus.
    ///
    /// - `state` is initialised to [`State::Closed`].
    /// - `baudrate` defaults to [`BAUDRATE_HIGHSPEED`].
    /// - The receive filter starts in reject-all state.
    #[inline]
    pub const fn new(bus_id: u8) -> Self {
        Self {
            bus_id,
            baudrate: BAUDRATE_HIGHSPEED,
            state: State::Closed,
            transceiver_state: TransceiverState::Active,
            filter: BitFieldFilter::new(),
            statistics: Statistics {
                errors: 0,
                rx: 0,
                tx: 0,
                rx_dropped: 0,
                tx_dropped: 0,
            },
            listener_count: 0,
        }
    }

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------

    /// Returns the current logical [`State`].
    #[inline]
    pub const fn state(&self) -> State {
        self.state
    }

    /// Unconditionally sets the logical state.
    ///
    /// Prefer the validated transition helpers below for safety-critical code.
    #[inline]
    pub fn set_state(&mut self, state: State) {
        self.state = state;
    }

    /// Returns `true` if the transceiver is currently in `state`.
    #[inline]
    pub fn is_in_state(&self, state: State) -> bool {
        self.state == state
    }

    /// Attempts the transition `Closed → Initialized`.
    ///
    /// Returns [`ErrorCode::Ok`] on success or [`ErrorCode::IllegalState`] if
    /// the current state is not `Closed`.
    pub fn transition_to_initialized(&mut self) -> ErrorCode {
        if self.state == State::Closed {
            self.state = State::Initialized;
            ErrorCode::Ok
        } else {
            ErrorCode::IllegalState
        }
    }

    /// Attempts the transition `Initialized → Open`.
    pub fn transition_to_open(&mut self) -> ErrorCode {
        if self.state == State::Initialized || self.state == State::Muted {
            self.state = State::Open;
            ErrorCode::Ok
        } else {
            ErrorCode::IllegalState
        }
    }

    /// Attempts the transition `Open → Muted`.
    pub fn transition_to_muted(&mut self) -> ErrorCode {
        if self.state == State::Open {
            self.state = State::Muted;
            ErrorCode::Ok
        } else {
            ErrorCode::IllegalState
        }
    }

    /// Transitions to `Closed` from any state.
    ///
    /// Always succeeds.
    pub fn transition_to_closed(&mut self) -> ErrorCode {
        self.state = State::Closed;
        ErrorCode::Ok
    }

    // -----------------------------------------------------------------------
    // Bus / hardware identity
    // -----------------------------------------------------------------------

    /// Returns the bus identifier.
    #[inline]
    pub const fn bus_id(&self) -> u8 {
        self.bus_id
    }

    /// Returns the configured baudrate.
    #[inline]
    pub const fn baudrate(&self) -> u32 {
        self.baudrate
    }

    /// Sets the baudrate.
    #[inline]
    pub fn set_baudrate(&mut self, baudrate: u32) {
        self.baudrate = baudrate;
    }

    // -----------------------------------------------------------------------
    // Hardware bus state
    // -----------------------------------------------------------------------

    /// Returns the hardware [`TransceiverState`].
    #[inline]
    pub const fn transceiver_state(&self) -> TransceiverState {
        self.transceiver_state
    }

    /// Sets the hardware [`TransceiverState`].
    #[inline]
    pub fn set_transceiver_state(&mut self, ts: TransceiverState) {
        self.transceiver_state = ts;
    }

    // -----------------------------------------------------------------------
    // Receive filter
    // -----------------------------------------------------------------------

    /// Returns a reference to the receive [`BitFieldFilter`].
    #[inline]
    pub const fn filter(&self) -> &BitFieldFilter {
        &self.filter
    }

    /// Returns a mutable reference to the receive [`BitFieldFilter`].
    #[inline]
    pub fn filter_mut(&mut self) -> &mut BitFieldFilter {
        &mut self.filter
    }

    // -----------------------------------------------------------------------
    // Statistics
    // -----------------------------------------------------------------------

    /// Returns a reference to the runtime [`Statistics`].
    #[inline]
    pub const fn statistics(&self) -> &Statistics {
        &self.statistics
    }

    /// Returns a mutable reference to the runtime [`Statistics`].
    #[inline]
    pub fn statistics_mut(&mut self) -> &mut Statistics {
        &mut self.statistics
    }

    // -----------------------------------------------------------------------
    // Listener tracking
    // -----------------------------------------------------------------------

    /// Returns the number of currently registered listeners.
    #[inline]
    pub const fn listener_count(&self) -> usize {
        self.listener_count
    }

    /// Attempts to register one more listener.
    ///
    /// Returns [`ErrorCode::Ok`] if `listener_count < MAX_LISTENERS`, or
    /// [`ErrorCode::NoMoreListenersPossible`] when the slot array is full.
    pub fn add_listener(&mut self) -> ErrorCode {
        if self.listener_count < MAX_LISTENERS {
            self.listener_count += 1;
            ErrorCode::Ok
        } else {
            ErrorCode::NoMoreListenersPossible
        }
    }

    /// Removes one listener registration.
    ///
    /// Saturates at zero (calling this on an empty set is a no-op).
    pub fn remove_listener(&mut self) {
        if self.listener_count > 0 {
            self.listener_count -= 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{
        AbstractTransceiver, ErrorCode, State, Statistics, TransceiverState, BAUDRATE_HIGHSPEED,
    };
    use crate::filter::Filter;

    type Trx = AbstractTransceiver<8>;

    // 1 — new transceiver starts in Closed state
    #[test]
    fn new_transceiver_is_closed() {
        let t = Trx::new(0);
        assert_eq!(t.state(), State::Closed);
        assert!(t.is_in_state(State::Closed));
    }

    // 2 — set_state changes the state
    #[test]
    fn set_state() {
        let mut t = Trx::new(0);
        t.set_state(State::Open);
        assert_eq!(t.state(), State::Open);
    }

    // 3 — is_in_state returns correct boolean
    #[test]
    fn is_in_state() {
        let t = Trx::new(0);
        assert!(t.is_in_state(State::Closed));
        assert!(!t.is_in_state(State::Open));
    }

    // 4 — bus_id is stored and retrieved correctly
    #[test]
    fn bus_id_stored() {
        let t = Trx::new(3);
        assert_eq!(t.bus_id(), 3);
    }

    // 5 — default filter rejects every ID
    #[test]
    fn default_filter_rejects_all() {
        let t = Trx::new(0);
        assert!(!t.filter().matches(0x100));
        assert!(!t.filter().matches(0x7FF));
    }

    // 6 — Closed → Initialized transition succeeds
    #[test]
    fn state_machine_closed_to_initialized() {
        let mut t = Trx::new(0);
        assert_eq!(t.transition_to_initialized(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Initialized);
    }

    // 7 — Initialized → Open transition succeeds
    #[test]
    fn state_machine_initialized_to_open() {
        let mut t = Trx::new(0);
        let _ = t.transition_to_initialized();
        assert_eq!(t.transition_to_open(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Open);
    }

    // 8 — Open → Muted transition succeeds
    #[test]
    fn state_machine_open_to_muted() {
        let mut t = Trx::new(0);
        t.set_state(State::Open);
        assert_eq!(t.transition_to_muted(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Muted);
    }

    // 9 — Muted → Open transition succeeds
    #[test]
    fn state_machine_muted_to_open() {
        let mut t = Trx::new(0);
        t.set_state(State::Muted);
        assert_eq!(t.transition_to_open(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Open);
    }

    // 10 — any state → Closed via transition_to_closed
    #[test]
    fn state_machine_open_to_closed() {
        let mut t = Trx::new(0);
        t.set_state(State::Open);
        assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Closed);
    }

    // 11 — Closed → Closed transition via transition_to_closed is a no-op
    #[test]
    fn state_machine_muted_to_closed() {
        let mut t = Trx::new(0);
        t.set_state(State::Muted);
        assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Closed);
    }

    // 12 — illegal transitions return IllegalState
    #[test]
    fn illegal_state_transitions() {
        let mut t = Trx::new(0);
        // Can't go Closed → Open (must init first)
        assert_eq!(t.transition_to_open(), ErrorCode::IllegalState);
        // Can't go Closed → Muted
        assert_eq!(t.transition_to_muted(), ErrorCode::IllegalState);
        // Can't go Initialized → Initialized (already there)
        let _ = t.transition_to_initialized();
        assert_eq!(t.transition_to_initialized(), ErrorCode::IllegalState);
    }

    // 13 — Statistics default is all-zero
    #[test]
    fn statistics_default() {
        let t = Trx::new(0);
        assert_eq!(*t.statistics(), Statistics::default());
        assert_eq!(t.statistics().rx, 0);
        assert_eq!(t.statistics().tx, 0);
        assert_eq!(t.statistics().errors, 0);
    }

    // 14 — set_transceiver_state persists
    #[test]
    fn transceiver_state_tracking() {
        let mut t = Trx::new(0);
        assert_eq!(t.transceiver_state(), TransceiverState::Active);
        t.set_transceiver_state(TransceiverState::BusOff);
        assert_eq!(t.transceiver_state(), TransceiverState::BusOff);
    }

    // 15 — filter_mut allows adding IDs; filter() reflects the change
    #[test]
    fn filter_merge_on_add() {
        let mut t = Trx::new(0);
        t.filter_mut().add(0x1A0);
        assert!(t.filter().matches(0x1A0));
        assert!(!t.filter().matches(0x1A1));
    }

    // Bonus — baudrate defaults to BAUDRATE_HIGHSPEED
    #[test]
    fn baudrate_default() {
        let t = Trx::new(0);
        assert_eq!(t.baudrate(), BAUDRATE_HIGHSPEED);
    }

    // Bonus — listener slot management
    #[test]
    fn listener_slots() {
        let mut t: AbstractTransceiver<2> = AbstractTransceiver::new(0);
        assert_eq!(t.add_listener(), ErrorCode::Ok);
        assert_eq!(t.add_listener(), ErrorCode::Ok);
        assert_eq!(t.add_listener(), ErrorCode::NoMoreListenersPossible);
        t.remove_listener();
        assert_eq!(t.add_listener(), ErrorCode::Ok);
    }
}

