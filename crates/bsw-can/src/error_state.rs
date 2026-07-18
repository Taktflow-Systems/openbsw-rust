//! CAN error-state tracking, bus-off recovery, and bus-load estimation
//! (package D14).
//!
//! [`ErrorStateTracker`] models the ISO 11898-1 fault-confinement counters
//! (TEC/REC) with the standard thresholds: error passive at 128 or above,
//! bus off when the transmit counter reaches 256. Recovery from bus off is
//! time-based through an injected instant, standing in for the 128×11
//! recessive-bit observation that real hardware performs.
//!
//! [`BusLoadEstimator`] accumulates transmitted/received bits over a
//! caller-defined window, using the same frame-overhead constant as the
//! upstream statistics helpers.

use bsw_time::{Duration, Instant};

use crate::frame::{CanFrame, CAN_OVERHEAD_BITS};
use crate::transceiver::TransceiverState;

/// Error-passive threshold for either counter.
pub const ERROR_PASSIVE_THRESHOLD: u16 = 128;

/// Bus-off threshold for the transmit error counter.
pub const BUS_OFF_THRESHOLD: u16 = 256;

/// ISO 11898-1 style fault-confinement tracker.
#[derive(Debug, Clone)]
pub struct ErrorStateTracker {
    tec: u16,
    rec: u16,
    state: TransceiverState,
    recovery_until: Option<Instant>,
}

impl ErrorStateTracker {
    /// Create an error-active tracker with zero counters.
    pub const fn new() -> Self {
        Self {
            tec: 0,
            rec: 0,
            state: TransceiverState::Active,
            recovery_until: None,
        }
    }

    /// Current error state.
    pub const fn state(&self) -> TransceiverState {
        self.state
    }

    /// Transmit error counter.
    pub const fn tec(&self) -> u16 {
        self.tec
    }

    /// Receive error counter.
    pub const fn rec(&self) -> u16 {
        self.rec
    }

    /// Record a transmit error (+8, per ISO 11898-1).
    ///
    /// Returns the new state when the state changed.
    pub fn record_tx_error(&mut self) -> Option<TransceiverState> {
        if self.state == TransceiverState::BusOff {
            return None;
        }
        self.tec = self.tec.saturating_add(8);
        self.update_state()
    }

    /// Record a receive error (+1).
    pub fn record_rx_error(&mut self) -> Option<TransceiverState> {
        if self.state == TransceiverState::BusOff {
            return None;
        }
        self.rec = self.rec.saturating_add(1);
        self.update_state()
    }

    /// Record a successful transmission (-1).
    pub fn record_tx_success(&mut self) -> Option<TransceiverState> {
        self.tec = self.tec.saturating_sub(1);
        self.update_state()
    }

    /// Record a successful reception (-1).
    pub fn record_rx_success(&mut self) -> Option<TransceiverState> {
        self.rec = self.rec.saturating_sub(1);
        self.update_state()
    }

    /// Begin bus-off recovery; the node rejoins after `recovery_time`.
    ///
    /// No-op unless the tracker is actually bus off.
    pub fn begin_recovery(&mut self, now: Instant, recovery_time: Duration) {
        if self.state == TransceiverState::BusOff && self.recovery_until.is_none() {
            self.recovery_until = Some(now.wrapping_add(recovery_time));
        }
    }

    /// Poll a running recovery; on completion the tracker returns to
    /// error-active with cleared counters and reports the new state.
    pub fn poll_recovery(&mut self, now: Instant) -> Option<TransceiverState> {
        let deadline = self.recovery_until?;
        if !now.is_at_or_after(deadline) {
            return None;
        }
        self.recovery_until = None;
        self.tec = 0;
        self.rec = 0;
        self.state = TransceiverState::Active;
        Some(TransceiverState::Active)
    }

    fn update_state(&mut self) -> Option<TransceiverState> {
        let new_state = if self.tec >= BUS_OFF_THRESHOLD {
            TransceiverState::BusOff
        } else if self.tec >= ERROR_PASSIVE_THRESHOLD || self.rec >= ERROR_PASSIVE_THRESHOLD {
            TransceiverState::Passive
        } else {
            TransceiverState::Active
        };
        if new_state == self.state {
            return None;
        }
        self.state = new_state;
        Some(new_state)
    }
}

impl Default for ErrorStateTracker {
    fn default() -> Self {
        Self::new()
    }
}

/// Approximate frame size on the wire in bits.
///
/// Classic frames use the upstream overhead constant; CAN FD frames use a
/// documented approximation (arbitration plus data-phase overhead) that is
/// adequate for load supervision, not for exact stuff-bit accounting.
pub fn frame_bits(frame: &CanFrame, fd: bool) -> u32 {
    let payload_bits = u32::from(frame.payload_length()) * 8;
    if fd {
        70 + payload_bits
    } else {
        u32::from(CAN_OVERHEAD_BITS) + payload_bits
    }
}

/// Windowed bus-load accumulator.
#[derive(Debug, Clone, Default)]
pub struct BusLoadEstimator {
    bits: u64,
}

impl BusLoadEstimator {
    /// Create an empty estimator.
    pub const fn new() -> Self {
        Self { bits: 0 }
    }

    /// Account one frame.
    pub fn record_frame(&mut self, frame: &CanFrame, fd: bool) {
        self.bits = self.bits.saturating_add(u64::from(frame_bits(frame, fd)));
    }

    /// Bits accumulated in the current window.
    pub const fn bits(&self) -> u64 {
        self.bits
    }

    /// Close the window: return the load in permille of `bitrate` over
    /// `window` and reset the accumulator.
    ///
    /// Returns `None` for a zero-length window or zero bitrate.
    pub fn take_load_permille(&mut self, window: Duration, bitrate: u32) -> Option<u16> {
        if window == Duration::ZERO || bitrate == 0 {
            return None;
        }
        let bits = core::mem::take(&mut self.bits);
        let capacity = u128::from(bitrate) * u128::from(window.as_nanos()) / 1_000_000_000;
        if capacity == 0 {
            return None;
        }
        let permille = (u128::from(bits) * 1000 / capacity).min(1000);
        #[allow(clippy::cast_possible_truncation)]
        Some(permille as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_id::CanId;

    const MS: Duration = Duration::from_nanos(1_000_000);

    #[test]
    fn tx_errors_walk_active_passive_bus_off() {
        let mut tracker = ErrorStateTracker::new();
        assert_eq!(tracker.state(), TransceiverState::Active);
        let mut transitions = Vec::new();
        for _ in 0..32 {
            if let Some(state) = tracker.record_tx_error() {
                transitions.push(state);
            }
        }
        assert_eq!(
            transitions,
            [TransceiverState::Passive, TransceiverState::BusOff]
        );
        assert_eq!(tracker.tec(), 256);
        // Further traffic is ignored while bus off.
        assert_eq!(tracker.record_tx_error(), None);
        assert_eq!(tracker.tec(), 256);
    }

    #[test]
    fn success_decrements_and_reactivates() {
        let mut tracker = ErrorStateTracker::new();
        for _ in 0..16 {
            tracker.record_tx_error();
        }
        assert_eq!(tracker.state(), TransceiverState::Passive);
        let mut recovered = None;
        for _ in 0..8 {
            if let Some(state) = tracker.record_tx_success() {
                recovered = Some(state);
            }
        }
        assert_eq!(recovered, Some(TransceiverState::Active));
        assert_eq!(tracker.tec(), 120);
    }

    #[test]
    fn rx_errors_reach_passive_but_never_bus_off() {
        let mut tracker = ErrorStateTracker::new();
        for _ in 0..1000 {
            tracker.record_rx_error();
        }
        assert_eq!(tracker.state(), TransceiverState::Passive);
        for _ in 0..1000 {
            tracker.record_rx_success();
        }
        assert_eq!(tracker.state(), TransceiverState::Active);
    }

    #[test]
    fn bus_off_recovery_is_time_based_and_exact() {
        let mut tracker = ErrorStateTracker::new();
        for _ in 0..32 {
            tracker.record_tx_error();
        }
        assert_eq!(tracker.state(), TransceiverState::BusOff);
        let start = Instant::from_nanos(1_000);
        tracker.begin_recovery(start, MS);
        assert_eq!(tracker.poll_recovery(Instant::from_nanos(1_000_999)), None);
        assert_eq!(
            tracker.poll_recovery(Instant::from_nanos(1_001_000)),
            Some(TransceiverState::Active)
        );
        assert_eq!((tracker.tec(), tracker.rec()), (0, 0));
        // Polling again is idempotent.
        assert_eq!(tracker.poll_recovery(Instant::from_nanos(2_000_000)), None);
    }

    #[test]
    fn recovery_requires_bus_off() {
        let mut tracker = ErrorStateTracker::new();
        tracker.begin_recovery(Instant::from_nanos(0), MS);
        assert_eq!(tracker.poll_recovery(Instant::from_nanos(5_000_000)), None);
        assert_eq!(tracker.state(), TransceiverState::Active);
    }

    #[test]
    fn bus_load_accounts_frames_and_resets_per_window() {
        let mut estimator = BusLoadEstimator::new();
        let frame = CanFrame::with_data(CanId::base(0x100), &[0; 8]);
        // Classic: 47 + 64 = 111 bits per frame.
        for _ in 0..100 {
            estimator.record_frame(&frame, false);
        }
        assert_eq!(estimator.bits(), 11_100);
        // 100 frames in 100ms at 500 kbit/s: capacity 50_000 bits.
        let load = estimator
            .take_load_permille(Duration::from_millis(100).unwrap(), 500_000)
            .unwrap();
        assert_eq!(load, 222);
        assert_eq!(estimator.bits(), 0, "window reset");
        assert_eq!(
            estimator.take_load_permille(Duration::ZERO, 500_000),
            None,
            "zero window rejected"
        );
    }

    #[test]
    fn bus_load_saturates_at_full_utilisation() {
        let mut estimator = BusLoadEstimator::new();
        let frame = CanFrame::with_data(CanId::base(1), &[0; 8]);
        for _ in 0..10_000 {
            estimator.record_frame(&frame, true);
        }
        let load = estimator.take_load_permille(MS, 500_000).unwrap();
        assert_eq!(load, 1000);
    }
}
