//! Reset simulation for POSIX hosts.
//!
//! The upstream POSIX reference application maps `softwareSystemReset` to a
//! lifecycle restart, not a process kill. [`PosixResetControl`] therefore
//! *records* reset requests into an inspectable queue by default; the
//! scenario harness (or the application's lifecycle loop) drains the queue
//! with [`PosixResetControl::take_requested`] and performs the restart
//! itself. Actually terminating the process is an explicit, documented
//! escape hatch ([`PosixResetControl::with_process_exit`]) reserved for the
//! real host binary — it is never the default.

use std::collections::VecDeque;

use bsw_platform::{ResetControl, ResetReason};

/// Kind of reset requested through [`ResetControl`].
///
/// The [`ResetControl`] trait only issues software resets today; the enum
/// exists so the queue records *what* was requested and can grow additional
/// kinds without changing the queue API.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetKind {
    /// Software-requested system reset ([`ResetControl::request_reset`]).
    Software,
}

/// Reset simulation implementing [`ResetControl`] for POSIX hosts.
///
/// - `request_reset` appends a [`ResetKind`] to an inspectable FIFO queue
///   (the test control) instead of dying; drain it with
///   [`Self::take_requested`].
/// - The latched reset reason defaults to [`ResetReason::PowerOn`] on a
///   fresh start and is settable via [`Self::set_reset_reason`] /
///   [`Self::reboot_with`] so scenarios can simulate any boot cause.
/// - [`Self::with_process_exit`] opts into really exiting the process on
///   `request_reset` for the real binary.
#[derive(Debug)]
pub struct PosixResetControl {
    reason: ResetReason,
    queue: VecDeque<ResetKind>,
    requests: u32,
    exit_code: Option<i32>,
}

impl PosixResetControl {
    /// Create a reset simulation reporting [`ResetReason::PowerOn`] for the
    /// current boot. Reset requests are recorded, never executed.
    #[must_use]
    pub fn new() -> Self {
        Self {
            reason: ResetReason::PowerOn,
            queue: VecDeque::new(),
            requests: 0,
            exit_code: None,
        }
    }

    /// Escape hatch for the real host binary: `request_reset` terminates
    /// the process with `exit_code` instead of recording the request.
    ///
    /// This is intentionally a separate constructor so simulation stays the
    /// default; tests and the reference application's lifecycle harness
    /// must use [`Self::new`].
    #[must_use]
    pub fn with_process_exit(exit_code: i32) -> Self {
        Self {
            exit_code: Some(exit_code),
            ..Self::new()
        }
    }

    /// Latch the reset reason reported for the current boot.
    pub fn set_reset_reason(&mut self, reason: ResetReason) {
        self.reason = reason;
    }

    /// Pop the oldest recorded reset request, if any (test control).
    pub fn take_requested(&mut self) -> Option<ResetKind> {
        self.queue.pop_front()
    }

    /// Number of recorded reset requests not yet drained.
    #[must_use]
    pub fn pending(&self) -> usize {
        self.queue.len()
    }

    /// Total reset requests observed since construction or the last
    /// [`Self::reboot_with`].
    #[must_use]
    pub const fn requests(&self) -> u32 {
        self.requests
    }

    /// Simulate the next boot: latch a new reason, clear the request queue
    /// and counters (mirrors `bsw_platform::mock::MockReset::reboot_with`).
    pub fn reboot_with(&mut self, reason: ResetReason) {
        self.reason = reason;
        self.queue.clear();
        self.requests = 0;
    }
}

impl Default for PosixResetControl {
    fn default() -> Self {
        Self::new()
    }
}

impl ResetControl for PosixResetControl {
    fn request_reset(&mut self) {
        if let Some(code) = self.exit_code {
            std::process::exit(code);
        }
        self.requests += 1;
        self.queue.push_back(ResetKind::Software);
    }

    fn reset_reason(&self) -> ResetReason {
        self.reason
    }

    fn clear_reset_reason(&mut self) {
        self.reason = ResetReason::Unknown;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_start_reports_power_on_with_empty_queue() {
        let mut reset = PosixResetControl::new();
        assert_eq!(reset.reset_reason(), ResetReason::PowerOn);
        assert_eq!(reset.pending(), 0);
        assert_eq!(reset.requests(), 0);
        assert_eq!(reset.take_requested(), None);
    }

    #[test]
    fn requests_are_queued_in_fifo_order_and_drained() {
        let mut reset = PosixResetControl::new();
        reset.request_reset();
        reset.request_reset();
        assert_eq!(reset.pending(), 2);
        assert_eq!(reset.requests(), 2);
        assert_eq!(reset.take_requested(), Some(ResetKind::Software));
        assert_eq!(reset.pending(), 1);
        assert_eq!(reset.take_requested(), Some(ResetKind::Software));
        assert_eq!(reset.take_requested(), None);
        // Draining does not reset the total counter.
        assert_eq!(reset.requests(), 2);
    }

    #[test]
    fn reason_is_settable_clearable_and_relatched_on_reboot() {
        let mut reset = PosixResetControl::new();
        reset.set_reset_reason(ResetReason::Watchdog);
        assert_eq!(reset.reset_reason(), ResetReason::Watchdog);
        reset.clear_reset_reason();
        assert_eq!(reset.reset_reason(), ResetReason::Unknown);
        reset.request_reset();
        reset.reboot_with(ResetReason::Software);
        assert_eq!(reset.reset_reason(), ResetReason::Software);
        assert_eq!(reset.pending(), 0);
        assert_eq!(reset.requests(), 0);
    }
}
