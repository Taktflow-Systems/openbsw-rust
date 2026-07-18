//! Deterministic test controls for scenario tests, mirroring the
//! ergonomics of `bsw_platform::mock`.
//!
//! [`TestControls`] bundles everything a host scenario needs to drive the
//! POSIX platform deterministically: a shared fake clock, the reset
//! simulation's inspection queue, an injected-clock software watchdog, and
//! a scripted entropy source.

use std::sync::{Arc, Mutex};

use bsw_platform::{Entropy, EntropyError};
use bsw_time::{Clock, Duration, FakeClock, Instant};

use crate::reset::PosixResetControl;
use crate::watchdog::SoftWatchdog;

/// Cloneable handle to one shared [`FakeClock`].
///
/// [`SoftWatchdog`] and the scenario harness must observe the *same*
/// deterministic time; `FakeClock` itself is a value type, so this wraps
/// it in `Arc<Mutex<..>>` and implements [`Clock`] on the handle. Advance
/// or set the time through any clone and every observer sees it.
#[derive(Debug, Clone, Default)]
pub struct SharedClock {
    inner: Arc<Mutex<FakeClock>>,
}

impl SharedClock {
    /// Create a shared clock at `start`.
    #[must_use]
    pub fn new(start: Instant) -> Self {
        Self {
            inner: Arc::new(Mutex::new(FakeClock::new(start))),
        }
    }

    /// Advance the clock (wrapping arithmetic).
    pub fn advance(&self, duration: Duration) {
        self.inner
            .lock()
            .expect("shared clock mutex poisoned")
            .advance(duration);
    }

    /// Set the current time explicitly.
    pub fn set(&self, now: Instant) {
        self.inner
            .lock()
            .expect("shared clock mutex poisoned")
            .set(now);
    }
}

impl Clock for SharedClock {
    fn now(&self) -> Instant {
        self.inner
            .lock()
            .expect("shared clock mutex poisoned")
            .now()
    }
}

#[derive(Debug, Clone)]
enum Mode {
    Seeded { state: u64 },
    Script { bytes: Vec<u8>, position: usize },
}

/// Deterministic entropy for scenario tests, implementing [`Entropy`].
///
/// Two modes:
/// - [`FixedEntropy::from_seed`] — an endless xorshift64* stream (the same
///   generator family as `bsw_platform::mock::MockEntropy`), fully
///   reproducible from the seed.
/// - [`FixedEntropy::from_script`] — replays exact bytes; once the script
///   cannot cover a whole `fill`, the call fails with [`EntropyError`]
///   without consuming anything, giving tests a deterministic failure
///   path.
#[derive(Debug, Clone)]
pub struct FixedEntropy {
    mode: Mode,
}

impl FixedEntropy {
    /// Endless deterministic stream from a seed (0 is coerced to 1).
    #[must_use]
    pub const fn from_seed(seed: u64) -> Self {
        Self {
            mode: Mode::Seeded {
                state: if seed == 0 { 1 } else { seed },
            },
        }
    }

    /// Replay `bytes` exactly, then fail (see type docs).
    #[must_use]
    pub fn from_script(bytes: &[u8]) -> Self {
        Self {
            mode: Mode::Script {
                bytes: bytes.to_vec(),
                position: 0,
            },
        }
    }

    /// Unconsumed script bytes; `None` in seeded mode (endless).
    #[must_use]
    pub fn remaining(&self) -> Option<usize> {
        match &self.mode {
            Mode::Seeded { .. } => None,
            Mode::Script { bytes, position } => Some(bytes.len() - position),
        }
    }
}

fn xorshift64_star(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    x.wrapping_mul(0x2545_F491_4F6C_DD1D)
}

impl Entropy for FixedEntropy {
    fn fill(&mut self, buffer: &mut [u8]) -> Result<(), EntropyError> {
        match &mut self.mode {
            Mode::Seeded { state } => {
                for chunk in buffer.chunks_mut(8) {
                    let word = xorshift64_star(state).to_le_bytes();
                    chunk.copy_from_slice(&word[..chunk.len()]);
                }
                Ok(())
            }
            Mode::Script { bytes, position } => {
                if bytes.len() - *position < buffer.len() {
                    return Err(EntropyError);
                }
                buffer.copy_from_slice(&bytes[*position..*position + buffer.len()]);
                *position += buffer.len();
                Ok(())
            }
        }
    }
}

/// Default lower watchdog timeout bound for [`TestControls`]: 1 ms.
pub const DEFAULT_WATCHDOG_MIN: Duration = Duration::from_nanos(1_000_000);
/// Default upper watchdog timeout bound for [`TestControls`]: 10 s.
pub const DEFAULT_WATCHDOG_MAX: Duration = Duration::from_nanos(10_000_000_000);
/// Default [`FixedEntropy`] seed used by [`TestControls::new`].
pub const DEFAULT_ENTROPY_SEED: u64 = 0x00B5_00B5_00B5_00B5;

/// Bundle of deterministic platform controls for scenario tests.
///
/// All members share [`TestControls::clock`]: advancing it moves the
/// watchdog's view of time; the reset queue and entropy script are
/// inspectable and reproducible. Fields are public — scenarios wire them
/// into the code under test directly, mirroring `bsw_platform::mock`.
#[derive(Debug)]
pub struct TestControls {
    /// Shared deterministic clock driving every time-dependent control.
    pub clock: SharedClock,
    /// Reset simulation with an inspectable request queue.
    pub reset: PosixResetControl,
    /// Software watchdog observing [`Self::clock`].
    pub watchdog: SoftWatchdog<SharedClock>,
    /// Deterministic entropy (seeded mode by default).
    pub entropy: FixedEntropy,
}

impl TestControls {
    /// Create controls at time zero with the default watchdog range
    /// ([`DEFAULT_WATCHDOG_MIN`]..=[`DEFAULT_WATCHDOG_MAX`]) and the
    /// default entropy seed.
    #[must_use]
    pub fn new() -> Self {
        Self::with_watchdog_range(DEFAULT_WATCHDOG_MIN, DEFAULT_WATCHDOG_MAX)
    }

    /// Create controls with an explicit watchdog timeout range.
    #[must_use]
    pub fn with_watchdog_range(min_timeout: Duration, max_timeout: Duration) -> Self {
        let clock = SharedClock::new(Instant::from_nanos(0));
        let watchdog = SoftWatchdog::new(clock.clone(), min_timeout, max_timeout);
        Self {
            clock,
            reset: PosixResetControl::new(),
            watchdog,
            entropy: FixedEntropy::from_seed(DEFAULT_ENTROPY_SEED),
        }
    }
}

impl Default for TestControls {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_platform::{ResetControl, Watchdog};

    use crate::reset::ResetKind;

    #[test]
    fn shared_clock_clones_observe_one_time() {
        let clock = SharedClock::new(Instant::from_nanos(5));
        let observer = clock.clone();
        clock.advance(Duration::from_nanos(10));
        assert_eq!(observer.now(), Instant::from_nanos(15));
        observer.set(Instant::from_nanos(100));
        assert_eq!(clock.now(), Instant::from_nanos(100));
    }

    #[test]
    fn seeded_entropy_is_reproducible() {
        let mut a = FixedEntropy::from_seed(7);
        let mut b = FixedEntropy::from_seed(7);
        let mut buf_a = [0_u8; 13];
        let mut buf_b = [0_u8; 13];
        a.fill(&mut buf_a).unwrap();
        b.fill(&mut buf_b).unwrap();
        assert_eq!(buf_a, buf_b);
        assert_ne!(buf_a, [0_u8; 13]);
        assert_eq!(a.remaining(), None);
    }

    #[test]
    fn scripted_entropy_replays_then_fails_without_partial_reads() {
        let mut entropy = FixedEntropy::from_script(&[1, 2, 3, 4, 5]);
        let mut buf = [0_u8; 3];
        entropy.fill(&mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3]);
        assert_eq!(entropy.remaining(), Some(2));
        assert_eq!(entropy.fill(&mut buf), Err(EntropyError));
        // The failed fill consumed nothing.
        assert_eq!(entropy.remaining(), Some(2));
        let mut rest = [0_u8; 2];
        entropy.fill(&mut rest).unwrap();
        assert_eq!(rest, [4, 5]);
        assert_eq!(entropy.remaining(), Some(0));
    }

    #[test]
    fn controls_drive_watchdog_and_reset_deterministically() {
        let mut controls = TestControls::new();
        controls
            .watchdog
            .start(Duration::from_millis(10).unwrap())
            .unwrap();
        controls.clock.advance(Duration::from_millis(9).unwrap());
        assert!(!controls.watchdog.expired(controls.clock.now()));
        controls.clock.advance(Duration::from_millis(1).unwrap());
        assert!(controls.watchdog.expired(controls.clock.now()));
        controls.reset.request_reset();
        assert_eq!(controls.reset.take_requested(), Some(ResetKind::Software));
        assert_eq!(controls.reset.take_requested(), None);
    }
}
