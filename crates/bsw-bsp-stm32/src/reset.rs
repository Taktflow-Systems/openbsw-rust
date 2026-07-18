//! Reset-cause decoding and injected-time deferred reset support (G10).

use bsw_platform::ResetReason;
use bsw_time::{Duration, Instant};

/// RCC reset-status layout selected by the active STM32 family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetStatusLayout {
    F4,
    G4,
}

/// Decode the latched RCC reset flags, using safety-relevant causes first.
#[must_use]
pub const fn decode_reset_reason(layout: ResetStatusLayout, raw: u32) -> ResetReason {
    let watchdog = (raw & ((1 << 30) | (1 << 29))) != 0;
    let software = raw & (1 << 28) != 0;
    let low_power = raw & (1 << 31) != 0;
    let (power, pin) = match layout {
        ResetStatusLayout::F4 => (raw & ((1 << 27) | (1 << 25)) != 0, raw & (1 << 26) != 0),
        ResetStatusLayout::G4 => (raw & (1 << 27) != 0, raw & (1 << 26) != 0),
    };
    if watchdog {
        ResetReason::Watchdog
    } else if software {
        ResetReason::Software
    } else if power {
        ResetReason::PowerOn
    } else if pin {
        ResetReason::Pin
    } else if low_power {
        ResetReason::LowPower
    } else {
        ResetReason::Unknown
    }
}

/// A reset request armed against an injected monotonic clock.
#[derive(Debug, Clone, Copy, Default)]
pub struct DeferredReset {
    deadline: Option<Instant>,
}

impl DeferredReset {
    #[must_use]
    pub const fn new() -> Self {
        Self { deadline: None }
    }

    pub fn schedule(&mut self, now: Instant, delay: Duration) {
        self.deadline = Some(now.wrapping_add(delay));
    }

    pub fn cancel(&mut self) {
        self.deadline = None;
    }

    #[must_use]
    pub const fn is_pending(&self) -> bool {
        self.deadline.is_some()
    }

    /// Request reset once at or after the deadline.
    pub fn poll<R: bsw_platform::ResetControl>(&mut self, now: Instant, reset: &mut R) -> bool {
        let Some(deadline) = self.deadline else {
            return false;
        };
        if !now.is_at_or_after(deadline) {
            return false;
        }
        self.deadline = None;
        reset.request_reset();
        true
    }
}

/// Hardware reset control. The status is latched before RCC flags are cleared.
#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
pub struct Stm32ResetControl {
    reason: ResetReason,
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
impl Stm32ResetControl {
    const RCC_CSR: *mut u32 = {
        #[cfg(feature = "stm32f413")]
        {
            0x4002_3874 as *mut u32
        }
        #[cfg(feature = "stm32g474")]
        {
            0x4002_1094 as *mut u32
        }
    };

    const LAYOUT: ResetStatusLayout = {
        #[cfg(feature = "stm32f413")]
        {
            ResetStatusLayout::F4
        }
        #[cfg(feature = "stm32g474")]
        {
            ResetStatusLayout::G4
        }
    };

    /// Latch the cause left by the previous boot. Exactly one owner must call this.
    ///
    /// # Safety
    /// The selected MCU feature must match the executing silicon.
    #[must_use]
    pub unsafe fn new(_token: crate::board::Reset) -> Self {
        // SAFETY: the active feature selects the matching RCC CSR address.
        let raw = unsafe { crate::mmio::read(Self::RCC_CSR) };
        Self {
            reason: decode_reset_reason(Self::LAYOUT, raw),
        }
    }
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
impl bsw_platform::ResetControl for Stm32ResetControl {
    fn request_reset(&mut self) {
        cortex_m::peripheral::SCB::sys_reset()
    }

    fn reset_reason(&self) -> ResetReason {
        self.reason
    }

    fn clear_reset_reason(&mut self) {
        const RMVF: u32 = 1 << 24;
        // SAFETY: writing RMVF to the selected RCC CSR clears reset flags only.
        unsafe {
            let raw = crate::mmio::read(Self::RCC_CSR);
            crate::mmio::write(Self::RCC_CSR, raw | RMVF);
        }
        self.reason = ResetReason::Unknown;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_platform::ResetControl;

    struct MockReset {
        requested: bool,
    }

    impl ResetControl for MockReset {
        fn request_reset(&mut self) {
            self.requested = true;
        }

        fn reset_reason(&self) -> ResetReason {
            ResetReason::PowerOn
        }

        fn clear_reset_reason(&mut self) {}
    }

    #[test]
    fn reset_causes_decode_for_both_register_layouts() {
        assert_eq!(
            decode_reset_reason(ResetStatusLayout::F4, 1 << 29),
            ResetReason::Watchdog
        );
        assert_eq!(
            decode_reset_reason(ResetStatusLayout::G4, 1 << 28),
            ResetReason::Software
        );
        assert_eq!(
            decode_reset_reason(ResetStatusLayout::F4, 1 << 27),
            ResetReason::PowerOn
        );
        assert_eq!(
            decode_reset_reason(ResetStatusLayout::G4, 1 << 26),
            ResetReason::Pin
        );
        assert_eq!(
            decode_reset_reason(ResetStatusLayout::G4, 0),
            ResetReason::Unknown
        );
    }

    #[test]
    fn deferred_reset_fires_once_at_exact_injected_deadline() {
        let mut deferred = DeferredReset::new();
        let mut reset = MockReset { requested: false };
        deferred.schedule(Instant::from_nanos(10), Duration::from_nanos(5));
        assert!(!deferred.poll(Instant::from_nanos(14), &mut reset));
        assert!(!reset.requested);
        assert!(deferred.poll(Instant::from_nanos(15), &mut reset));
        assert!(reset.requested);
        assert!(!deferred.poll(Instant::from_nanos(16), &mut reset));
    }
}
