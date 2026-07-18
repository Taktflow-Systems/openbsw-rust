//! Deterministic POSIX mocks covering every platform contract (D03).
//!
//! Host integration tests compose these instead of board adapters. Every
//! mock records its interactions so tests can assert platform usage
//! precisely, and time-dependent behavior is driven by explicit instants
//! rather than the wall clock.

use bsw_time::{Duration, Instant};

use crate::{
    CriticalSection, Entropy, EntropyError, PlatformInfo, ResetControl, ResetReason, UniqueId,
    Watchdog, WatchdogError, UNIQUE_ID_MAX,
};

/// Records reset requests instead of resetting.
#[derive(Debug)]
pub struct MockReset {
    reason: ResetReason,
    requests: u32,
}

impl MockReset {
    /// Create a mock that reports `reason` for the current boot.
    pub const fn new(reason: ResetReason) -> Self {
        Self {
            reason,
            requests: 0,
        }
    }

    /// Number of reset requests observed.
    pub const fn requests(&self) -> u32 {
        self.requests
    }

    /// Simulate the next boot: latch a new reason and clear counters.
    pub fn reboot_with(&mut self, reason: ResetReason) {
        self.reason = reason;
        self.requests = 0;
    }
}

impl ResetControl for MockReset {
    fn request_reset(&mut self) {
        self.requests += 1;
    }

    fn reset_reason(&self) -> ResetReason {
        self.reason
    }

    fn clear_reset_reason(&mut self) {
        self.reason = ResetReason::Unknown;
    }
}

/// Watchdog driven by an explicit mock time.
///
/// Tests move time with [`MockWatchdog::advance_to`]; the [`Watchdog`]
/// trait calls then observe that time, so expiry is fully deterministic.
#[derive(Debug)]
pub struct MockWatchdog {
    timeout: Option<Duration>,
    last_service: Instant,
    now: Instant,
    services: u32,
    min_timeout: Duration,
    max_timeout: Duration,
}

impl MockWatchdog {
    /// Create a stopped watchdog accepting timeouts in the given range.
    pub const fn new(min_timeout: Duration, max_timeout: Duration) -> Self {
        Self {
            timeout: None,
            last_service: Instant::from_nanos(0),
            now: Instant::from_nanos(0),
            services: 0,
            min_timeout,
            max_timeout,
        }
    }

    /// Set the mock's current time.
    pub fn advance_to(&mut self, now: Instant) {
        self.now = now;
    }

    /// Whether the watchdog would have fired at the current mock time.
    pub fn expired(&self) -> bool {
        self.timeout
            .is_some_and(|timeout| self.now.duration_since(self.last_service) >= timeout)
    }

    /// Number of service calls observed.
    pub const fn services(&self) -> u32 {
        self.services
    }
}

impl Watchdog for MockWatchdog {
    fn start(&mut self, timeout: Duration) -> Result<(), WatchdogError> {
        if self.timeout.is_some() {
            return Err(WatchdogError::AlreadyRunning);
        }
        if timeout < self.min_timeout || timeout > self.max_timeout {
            return Err(WatchdogError::UnsupportedTimeout);
        }
        self.timeout = Some(timeout);
        self.last_service = self.now;
        Ok(())
    }

    fn service(&mut self) {
        self.services += 1;
        self.last_service = self.now;
    }

    fn is_running(&self) -> bool {
        self.timeout.is_some()
    }
}

/// Nesting-checked critical section.
#[derive(Debug, Default)]
pub struct MockCriticalSection {
    depth: u32,
    max_depth: u32,
    entries: u32,
    unbalanced_releases: u32,
}

impl MockCriticalSection {
    /// Create a released section.
    pub const fn new() -> Self {
        Self {
            depth: 0,
            max_depth: 0,
            entries: 0,
            unbalanced_releases: 0,
        }
    }

    /// Current nesting depth.
    pub const fn depth(&self) -> u32 {
        self.depth
    }

    /// Deepest nesting observed.
    pub const fn max_depth(&self) -> u32 {
        self.max_depth
    }

    /// Total acquires observed.
    pub const fn entries(&self) -> u32 {
        self.entries
    }

    /// Releases without a matching acquire (a caller bug).
    pub const fn unbalanced_releases(&self) -> u32 {
        self.unbalanced_releases
    }
}

impl CriticalSection for MockCriticalSection {
    fn acquire(&mut self) {
        self.depth += 1;
        self.entries += 1;
        self.max_depth = self.max_depth.max(self.depth);
    }

    fn release(&mut self) {
        if self.depth == 0 {
            self.unbalanced_releases += 1;
        } else {
            self.depth -= 1;
        }
    }
}

/// Deterministic pseudo-random entropy (xorshift64*).
#[derive(Debug)]
pub struct MockEntropy {
    state: u64,
    fail_after: Option<u32>,
    fills: u32,
}

impl MockEntropy {
    /// Create a generator from a non-zero seed.
    pub const fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 1 } else { seed },
            fail_after: None,
            fills: 0,
        }
    }

    /// Make `fill` fail after `count` further successful calls.
    pub fn fail_after(&mut self, count: u32) {
        self.fail_after = Some(count);
    }

    fn next(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

impl Entropy for MockEntropy {
    fn fill(&mut self, buffer: &mut [u8]) -> Result<(), EntropyError> {
        if let Some(remaining) = self.fail_after {
            if remaining == 0 {
                return Err(EntropyError);
            }
            self.fail_after = Some(remaining - 1);
        }
        self.fills += 1;
        for chunk in buffer.chunks_mut(8) {
            let word = self.next().to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        Ok(())
    }
}

/// Fixed unique identity.
#[derive(Debug)]
pub struct MockUniqueId {
    bytes: [u8; UNIQUE_ID_MAX],
    length: usize,
}

impl MockUniqueId {
    /// Create an identity from up to [`UNIQUE_ID_MAX`] bytes.
    pub fn new(id: &[u8]) -> Self {
        let length = id.len().min(UNIQUE_ID_MAX);
        let mut bytes = [0; UNIQUE_ID_MAX];
        bytes[..length].copy_from_slice(&id[..length]);
        Self { bytes, length }
    }
}

impl UniqueId for MockUniqueId {
    fn unique_id(&self, out: &mut [u8; UNIQUE_ID_MAX]) -> usize {
        out[..self.length].copy_from_slice(&self.bytes[..self.length]);
        self.length
    }
}

/// Host platform description.
#[derive(Debug, Clone, Copy)]
pub struct MockPlatformInfo;

impl PlatformInfo for MockPlatformInfo {
    fn platform_name(&self) -> &'static str {
        "posix-host"
    }

    fn cpu_frequency_hz(&self) -> u32 {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::with_critical;

    const MS: Duration = Duration::from_nanos(1_000_000);

    fn at(millis: u64) -> Instant {
        Instant::from_nanos(millis * 1_000_000)
    }

    #[test]
    fn reset_mock_records_requests_and_reasons() {
        let mut reset = MockReset::new(ResetReason::PowerOn);
        assert_eq!(reset.reset_reason(), ResetReason::PowerOn);
        reset.request_reset();
        reset.request_reset();
        assert_eq!(reset.requests(), 2);
        reset.clear_reset_reason();
        assert_eq!(reset.reset_reason(), ResetReason::Unknown);
        reset.reboot_with(ResetReason::Watchdog);
        assert_eq!(reset.reset_reason(), ResetReason::Watchdog);
        assert_eq!(reset.requests(), 0);
    }

    #[test]
    fn watchdog_mock_enforces_range_and_expiry() {
        let mut watchdog = MockWatchdog::new(MS, Duration::from_millis(100).unwrap());
        assert_eq!(
            watchdog.start(Duration::from_nanos(10)),
            Err(WatchdogError::UnsupportedTimeout)
        );
        assert!(!watchdog.is_running());
        watchdog.start(Duration::from_millis(10).unwrap()).unwrap();
        assert!(watchdog.is_running());
        assert_eq!(watchdog.start(MS), Err(WatchdogError::AlreadyRunning));
        watchdog.advance_to(at(5));
        watchdog.service();
        watchdog.advance_to(at(14));
        assert!(!watchdog.expired());
        watchdog.advance_to(at(15));
        assert!(watchdog.expired());
        watchdog.service();
        assert!(!watchdog.expired());
        assert_eq!(watchdog.services(), 2);
    }

    #[test]
    fn critical_section_mock_checks_balance() {
        let mut section = MockCriticalSection::new();
        let value = with_critical(&mut section, || 7);
        assert_eq!(value, 7);
        section.acquire();
        section.acquire();
        section.release();
        section.release();
        section.release();
        assert_eq!(section.depth(), 0);
        assert_eq!(section.unbalanced_releases(), 1);
        assert_eq!(section.max_depth(), 2);
        assert_eq!(section.entries(), 3);
    }

    #[test]
    fn entropy_mock_is_deterministic_and_fails_on_demand() {
        let mut a = MockEntropy::new(7);
        let mut b = MockEntropy::new(7);
        let mut buf_a = [0u8; 13];
        let mut buf_b = [0u8; 13];
        a.fill(&mut buf_a).unwrap();
        b.fill(&mut buf_b).unwrap();
        assert_eq!(buf_a, buf_b);
        assert_ne!(buf_a, [0u8; 13]);
        let mut c = MockEntropy::new(9);
        c.fail_after(1);
        assert!(c.fill(&mut buf_a).is_ok());
        assert_eq!(c.fill(&mut buf_a), Err(EntropyError));
    }

    #[test]
    fn zero_seed_is_coerced_to_nonzero() {
        let mut entropy = MockEntropy::new(0);
        let mut buf = [0u8; 8];
        entropy.fill(&mut buf).unwrap();
        assert_ne!(buf, [0u8; 8]);
    }

    #[test]
    fn unique_id_mock_bounds_and_copies() {
        let identity = MockUniqueId::new(&[1, 2, 3]);
        let mut out = [0u8; UNIQUE_ID_MAX];
        assert_eq!(identity.unique_id(&mut out), 3);
        assert_eq!(&out[..3], &[1, 2, 3]);
        let oversized = [9u8; 32];
        let identity = MockUniqueId::new(&oversized);
        assert_eq!(identity.unique_id(&mut out), UNIQUE_ID_MAX);
    }

    #[test]
    fn platform_info_mock_names_the_host() {
        let info = MockPlatformInfo;
        assert_eq!(info.platform_name(), "posix-host");
        assert_eq!(info.cpu_frequency_hz(), 0);
    }
}
