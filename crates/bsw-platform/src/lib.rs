//! Platform abstraction contracts for the OpenBSW Rust port (package D03).
//!
//! Every trait here is a seam between hardware-independent BSW code and a
//! platform adapter (POSIX host, STM32 BSP, future MCUs). Protocol and
//! service crates depend on these contracts only; boards implement them.
//!
//! The [`mock`] module (std) provides a full set of deterministic POSIX
//! mocks — one per contract — used by host integration tests.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "std")]
pub mod mock;

use bsw_time::Duration;

/// Cause of the most recent system reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    /// Power applied or brown-out recovery.
    PowerOn,
    /// Software requested the reset.
    Software,
    /// A watchdog expired.
    Watchdog,
    /// External reset pin.
    Pin,
    /// Low-power or wakeup related reset.
    LowPower,
    /// The platform cannot classify the reset.
    Unknown,
}

/// System reset control and reset-cause bookkeeping.
pub trait ResetControl {
    /// Request a system reset. Platforms perform the reset as soon as
    /// possible; hosts and mocks record it instead of dying, so callers
    /// must not assume the call diverges.
    fn request_reset(&mut self);

    /// Cause of the most recent reset, as latched at startup.
    fn reset_reason(&self) -> ResetReason;

    /// Clear the latched reset cause so the next boot reports fresh data.
    fn clear_reset_reason(&mut self);
}

/// Watchdog control error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogError {
    /// The timeout is outside the platform's supported range.
    UnsupportedTimeout,
    /// The watchdog is already running and cannot be reconfigured.
    AlreadyRunning,
    /// The watchdog hardware did not acknowledge configuration in time.
    HardwareFault,
}

/// Hardware or simulated watchdog.
pub trait Watchdog {
    /// Start the watchdog with the given timeout. Platforms may not be
    /// able to stop it afterwards.
    fn start(&mut self, timeout: Duration) -> Result<(), WatchdogError>;

    /// Service ("kick") the watchdog.
    fn service(&mut self);

    /// Whether the watchdog is currently running.
    fn is_running(&self) -> bool;
}

/// Critical-section control: a nestable interrupt lock.
///
/// Platform implementations disable interrupts (or take a process-wide
/// lock) on the first `acquire` and restore on the matching `release`.
/// Calls must nest; the section ends when every acquire was released.
pub trait CriticalSection {
    /// Enter the critical section (nests).
    fn acquire(&mut self);

    /// Leave the critical section (must match one `acquire`).
    fn release(&mut self);
}

/// Run `body` inside a critical section, releasing on every path.
pub fn with_critical<C: CriticalSection + ?Sized, R>(
    section: &mut C,
    body: impl FnOnce() -> R,
) -> R {
    section.acquire();
    let result = body();
    section.release();
    result
}

/// Entropy acquisition failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntropyError;

/// Source of random data (hardware RNG, OS entropy, test PRNG).
pub trait Entropy {
    /// Fill `buffer` with random bytes.
    fn fill(&mut self, buffer: &mut [u8]) -> Result<(), EntropyError>;
}

/// Maximum unique-identifier length in bytes.
pub const UNIQUE_ID_MAX: usize = 16;

/// Device-unique identity (silicon UID, host machine identity).
pub trait UniqueId {
    /// Write the unique ID into `out`, returning the number of bytes used
    /// (at most [`UNIQUE_ID_MAX`]).
    fn unique_id(&self, out: &mut [u8; UNIQUE_ID_MAX]) -> usize;
}

/// Static platform description.
pub trait PlatformInfo {
    /// Human-readable platform name, e.g. `"posix-host"` or `"stm32g474"`.
    fn platform_name(&self) -> &'static str;

    /// Core clock frequency in hertz (0 when meaningless on a host).
    fn cpu_frequency_hz(&self) -> u32;
}

#[cfg(test)]
mod tests {
    use super::*;

    struct CountingSection {
        depth: u32,
        max_depth: u32,
    }

    impl CriticalSection for CountingSection {
        fn acquire(&mut self) {
            self.depth += 1;
            self.max_depth = self.max_depth.max(self.depth);
        }

        fn release(&mut self) {
            self.depth -= 1;
        }
    }

    #[test]
    fn with_critical_balances_nested_sections() {
        let mut section = CountingSection {
            depth: 0,
            max_depth: 0,
        };
        let value = with_critical(&mut section, || 41) + 1;
        assert_eq!(value, 42);
        assert_eq!(section.depth, 0);
        assert_eq!(section.max_depth, 1);
    }
}
