//! Software watchdog for POSIX hosts, deterministic under a fake clock.
//!
//! [`SoftWatchdog`] implements [`Watchdog`] with an injected [`Clock`]:
//! `start` and `service` stamp the current clock instant, and a supervisor
//! loop polls [`SoftWatchdog::expired`] / [`SoftWatchdog::check`] with an
//! explicit `now`. There are no background threads, so behavior under
//! `bsw_time::FakeClock` (via
//! [`SharedClock`](crate::test_control::SharedClock)) is fully
//! deterministic.
//!
//! # Boundary semantic
//!
//! The watchdog counts as expired at *exactly* the deadline instant
//! (`now - last_service >= timeout`) — the same `>=` semantic as
//! `bsw_platform::mock::MockWatchdog`, so both pass the identical shared
//! conformance suite.

use bsw_platform::{Watchdog, WatchdogError};
use bsw_time::{Clock, Duration, Instant};

/// Supervisor-facing watchdog state reported by [`SoftWatchdog::check`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogHealth {
    /// The watchdog was never started.
    Stopped,
    /// Running and serviced within its timeout.
    Alive,
    /// Running and the timeout elapsed since the last service.
    Expired,
}

/// Injected-clock software watchdog (see module docs).
#[derive(Debug)]
pub struct SoftWatchdog<C: Clock> {
    clock: C,
    timeout: Option<Duration>,
    last_service: Instant,
    services: u32,
    min_timeout: Duration,
    max_timeout: Duration,
}

impl<C: Clock> SoftWatchdog<C> {
    /// Create a stopped watchdog accepting timeouts in
    /// `min_timeout..=max_timeout`.
    pub fn new(clock: C, min_timeout: Duration, max_timeout: Duration) -> Self {
        Self {
            clock,
            timeout: None,
            last_service: Instant::from_nanos(0),
            services: 0,
            min_timeout,
            max_timeout,
        }
    }

    /// Whether the watchdog has expired at `now` (see module docs for the
    /// exact boundary semantic). A stopped watchdog never expires.
    pub fn expired(&self, now: Instant) -> bool {
        self.timeout
            .is_some_and(|timeout| now.duration_since(self.last_service) >= timeout)
    }

    /// Classify the watchdog state at `now` for a supervisor loop.
    pub fn check(&self, now: Instant) -> WatchdogHealth {
        match self.timeout {
            None => WatchdogHealth::Stopped,
            Some(timeout) => {
                if now.duration_since(self.last_service) >= timeout {
                    WatchdogHealth::Expired
                } else {
                    WatchdogHealth::Alive
                }
            }
        }
    }

    /// Instant at which the watchdog expires, when running.
    pub fn deadline(&self) -> Option<Instant> {
        self.timeout
            .map(|timeout| self.last_service.wrapping_add(timeout))
    }

    /// Instant of the most recent `start`/`service` stamp (test control).
    pub const fn last_service(&self) -> Instant {
        self.last_service
    }

    /// Number of `service` calls observed (test control).
    pub const fn services(&self) -> u32 {
        self.services
    }
}

impl<C: Clock> Watchdog for SoftWatchdog<C> {
    fn start(&mut self, timeout: Duration) -> Result<(), WatchdogError> {
        if self.timeout.is_some() {
            return Err(WatchdogError::AlreadyRunning);
        }
        if timeout < self.min_timeout || timeout > self.max_timeout {
            return Err(WatchdogError::UnsupportedTimeout);
        }
        self.timeout = Some(timeout);
        self.last_service = self.clock.now();
        Ok(())
    }

    fn service(&mut self) {
        self.services += 1;
        self.last_service = self.clock.now();
    }

    fn is_running(&self) -> bool {
        self.timeout.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_control::SharedClock;

    const MS: Duration = Duration::from_nanos(1_000_000);

    fn watchdog_at_zero() -> (SharedClock, SoftWatchdog<SharedClock>) {
        let clock = SharedClock::new(Instant::from_nanos(0));
        let watchdog = SoftWatchdog::new(clock.clone(), MS, Duration::from_nanos(100_000_000));
        (clock, watchdog)
    }

    #[test]
    fn start_validates_range_and_rejects_restart() {
        let (_clock, mut watchdog) = watchdog_at_zero();
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
    }

    #[test]
    fn expiry_boundary_is_exactly_the_deadline_instant() {
        let (clock, mut watchdog) = watchdog_at_zero();
        watchdog.start(Duration::from_millis(10).unwrap()).unwrap();
        assert_eq!(watchdog.deadline(), Some(Instant::from_nanos(10_000_000)));
        // One nanosecond before the deadline: not expired.
        clock.set(Instant::from_nanos(9_999_999));
        assert!(!watchdog.expired(clock.now()));
        assert_eq!(watchdog.check(clock.now()), WatchdogHealth::Alive);
        // Exactly the deadline instant: expired (mock-matching `>=`).
        clock.set(Instant::from_nanos(10_000_000));
        assert!(watchdog.expired(clock.now()));
        assert_eq!(watchdog.check(clock.now()), WatchdogHealth::Expired);
    }

    #[test]
    fn service_stamps_the_injected_clock_and_defers_expiry() {
        let (clock, mut watchdog) = watchdog_at_zero();
        watchdog.start(Duration::from_millis(10).unwrap()).unwrap();
        clock.advance(Duration::from_millis(5).unwrap());
        watchdog.service();
        assert_eq!(watchdog.last_service(), Instant::from_nanos(5_000_000));
        clock.advance(Duration::from_millis(9).unwrap());
        assert!(!watchdog.expired(clock.now()));
        clock.advance(MS);
        assert!(watchdog.expired(clock.now()));
        watchdog.service();
        assert!(!watchdog.expired(clock.now()));
        assert_eq!(watchdog.services(), 2);
    }

    #[test]
    fn stopped_watchdog_never_expires() {
        let (clock, watchdog) = watchdog_at_zero();
        clock.advance(Duration::from_secs(1).unwrap());
        assert!(!watchdog.expired(clock.now()));
        assert_eq!(watchdog.check(clock.now()), WatchdogHealth::Stopped);
        assert_eq!(watchdog.deadline(), None);
    }
}
