//! Injected-clock P2/P2-star/S3 diagnostic timing (E17).

use bsw_time::{Duration, Instant};

/// Diagnostic timing configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimingConfig {
    /// Time until first response or NRC 0x78.
    pub p2: Duration,
    /// Absolute maximum time for the asynchronous request.
    pub p2_star: Duration,
    /// Repetition interval for NRC 0x78 while pending.
    pub response_pending_interval: Duration,
    /// Inactivity time before session reset.
    pub s3: Duration,
}

/// Timing state action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimingAction {
    /// Nothing is due.
    None,
    /// Send negative response 0x78.
    ResponsePending,
    /// P2-star elapsed; terminate the connection.
    RequestTimeout,
    /// S3 elapsed; reset diagnostic session/security state.
    SessionTimeout,
}

/// Per-connection/request timing tracker.
pub struct TimingState {
    config: TimingConfig,
    last_activity: Instant,
    request_started: Option<Instant>,
    next_pending: Instant,
    pending_count: u16,
}

impl TimingState {
    /// Create an idle tracker at `now`.
    pub const fn new(config: TimingConfig, now: Instant) -> Self {
        Self {
            config,
            last_activity: now,
            request_started: None,
            next_pending: now,
            pending_count: 0,
        }
    }

    /// Mark a request received and start P2/P2-star.
    pub fn start_request(&mut self, now: Instant) {
        self.last_activity = now;
        self.request_started = Some(now);
        self.next_pending = now.wrapping_add(self.config.p2);
        self.pending_count = 0;
    }

    /// Mark final positive/negative response complete.
    pub fn complete_request(&mut self, now: Instant) {
        self.last_activity = now;
        self.request_started = None;
    }

    /// TesterPresent or other traffic refreshes S3 without starting a job.
    pub fn activity(&mut self, now: Instant) {
        self.last_activity = now;
    }

    /// Evaluate exact protocol boundaries.
    pub fn poll(&mut self, now: Instant) -> TimingAction {
        if let Some(started) = self.request_started {
            if now.is_at_or_after(started.wrapping_add(self.config.p2_star)) {
                self.request_started = None;
                return TimingAction::RequestTimeout;
            }
            if now.is_at_or_after(self.next_pending) {
                self.next_pending = now.wrapping_add(self.config.response_pending_interval);
                self.pending_count = self.pending_count.saturating_add(1);
                return TimingAction::ResponsePending;
            }
            return TimingAction::None;
        }
        if now.is_at_or_after(self.last_activity.wrapping_add(self.config.s3)) {
            self.last_activity = now;
            TimingAction::SessionTimeout
        } else {
            TimingAction::None
        }
    }

    /// Number of NRC 0x78 responses emitted for this request.
    pub const fn response_pending_count(&self) -> u16 {
        self.pending_count
    }
}
