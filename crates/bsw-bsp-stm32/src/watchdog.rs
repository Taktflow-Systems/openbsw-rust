//! Independent Watchdog (IWDG) driver for STM32 F4/G4.
//!
//! The IWDG is clocked from the LSI (~32 kHz on F4, ~32 kHz on G4).
//! Once started, it cannot be stopped — only fed (kicked) to prevent reset.
//!
//! Default timeout: ~1 second (prescaler /32, reload 999 → 32000/32/1000 ≈ 1s).

/// IWDG register base address (same for F4 and G4).
const IWDG_BASE: usize = 0x4000_3000;

/// Register offsets.
const KR_OFFSET: usize = 0x00;
const PR_OFFSET: usize = 0x04;
const RLR_OFFSET: usize = 0x08;
const SR_OFFSET: usize = 0x0C;

/// Key register magic values.
const KEY_ENABLE: u32 = 0xCCCC;
const KEY_RELOAD: u32 = 0xAAAA;
const KEY_UNLOCK: u32 = 0x5555;

/// Prescaler divider /32 → LSI(32kHz)/32 = 1 kHz IWDG counter clock.
const PRESCALER_DIV32: u32 = 0b011;

/// Reload value for ~1 second timeout: 999 → 1000 ticks at 1 kHz = 1.0s.
const RELOAD_1S: u32 = 999;
const MAX_UPDATE_POLLS: u32 = 1_000_000;

#[inline(always)]
unsafe fn reg_write(offset: usize, val: u32) {
    unsafe { crate::mmio::write((IWDG_BASE + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_read(offset: usize) -> u32 {
    unsafe { crate::mmio::read((IWDG_BASE + offset) as *const u32) }
}

/// Independent watchdog driver.
///
/// Once [`start`](Self::start) is called, [`kick`](Self::kick) must be called
/// periodically (within the configured timeout) to prevent a system reset.
pub struct Iwdg {
    started: bool,
}

/// Reset-safe adapter for the startup fast-test state machine. The caller
/// supplies the previously latched reset cause before RCC flags are cleared.
pub struct IwdgFastTestBackend<'a> {
    watchdog: &'a mut Iwdg,
    watchdog_reset_observed: bool,
}

impl<'a> IwdgFastTestBackend<'a> {
    pub fn new(watchdog: &'a mut Iwdg, watchdog_reset_observed: bool) -> Self {
        Self {
            watchdog,
            watchdog_reset_observed,
        }
    }
}

impl bsw_safety::mechanisms::WatchdogFastTestBackend for IwdgFastTestBackend<'_> {
    fn arm_shortest_timeout(&mut self) -> bool {
        #[allow(clippy::cast_possible_truncation)]
        self.watchdog
            .start_with_timeout(crate::resource_contract::WATCHDOG_FAST_TEST_RESET_MS as u32)
    }
    fn reset_marker_observed(&self) -> bool {
        self.watchdog_reset_observed
    }
    fn request_controlled_reset(&mut self) {
        cortex_m::peripheral::SCB::sys_reset()
    }
    fn retained_state(&self) -> Option<bsw_safety::RetainedWatchdogFastTest> {
        crate::fault::retained_watchdog_test()
    }
    fn store_retained_state(&mut self, state: bsw_safety::RetainedWatchdogFastTest) {
        crate::fault::write_retained_watchdog_test(state);
    }
}

impl Iwdg {
    pub const fn from_token(_token: crate::board::Watchdog) -> Self {
        Self::new()
    }

    /// Create a new watchdog instance (not yet started).
    #[must_use]
    pub const fn new() -> Self {
        Self { started: false }
    }

    /// Start the IWDG with ~1 second timeout.
    ///
    /// **WARNING**: Once started, the IWDG cannot be stopped.
    /// The only way to disable it is a system reset.
    pub fn start(&mut self) -> bool {
        unsafe {
            // Start the watchdog first — this auto-enables LSI oscillator
            // which is needed for PR/RLR register updates to propagate.
            reg_write(KR_OFFSET, KEY_ENABLE);

            // Unlock PR and RLR registers.
            reg_write(KR_OFFSET, KEY_UNLOCK);

            // Set prescaler to /32.
            reg_write(PR_OFFSET, PRESCALER_DIV32);

            // Set reload value.
            reg_write(RLR_OFFSET, RELOAD_1S);

            // Wait for PVU and RVU flags to clear (registers updated).
            // LSI is now running (started by KEY_ENABLE above).
            let mut polls = 0;
            while (reg_read(SR_OFFSET) & 0x03) != 0 {
                polls += 1;
                if polls >= MAX_UPDATE_POLLS {
                    return false;
                }
            }

            // Initial kick.
            reg_write(KR_OFFSET, KEY_RELOAD);
        }

        self.started = true;
        true
    }

    /// Start the IWDG with a custom timeout in milliseconds.
    ///
    /// Actual timeout = `timeout_ms` (clamped to 1..=4096 ms at /32 prescaler).
    pub fn start_with_timeout(&mut self, timeout_ms: u32) -> bool {
        let reload = timeout_ms.clamp(1, 4096).saturating_sub(1);

        unsafe {
            reg_write(KR_OFFSET, KEY_ENABLE);
            reg_write(KR_OFFSET, KEY_UNLOCK);
            reg_write(PR_OFFSET, PRESCALER_DIV32);
            reg_write(RLR_OFFSET, reload);
            let mut polls = 0;
            while (reg_read(SR_OFFSET) & 0x03) != 0 {
                polls += 1;
                if polls >= MAX_UPDATE_POLLS {
                    return false;
                }
            }
            reg_write(KR_OFFSET, KEY_RELOAD);
        }

        self.started = true;
        true
    }

    /// Kick (reload) the watchdog to prevent reset.
    ///
    /// Must be called within the configured timeout period.
    /// Safe to call even if the IWDG hasn't been started (no-op in hardware).
    #[inline]
    pub fn kick(&self) {
        unsafe { reg_write(KR_OFFSET, KEY_RELOAD) };
    }

    /// Returns whether the watchdog has been started.
    #[must_use]
    pub const fn is_started(&self) -> bool {
        self.started
    }
}

impl Default for Iwdg {
    fn default() -> Self {
        Self::new()
    }
}

impl bsw_platform::Watchdog for Iwdg {
    fn start(&mut self, timeout: bsw_time::Duration) -> Result<(), bsw_platform::WatchdogError> {
        if self.started {
            return Err(bsw_platform::WatchdogError::AlreadyRunning);
        }
        let nanos = timeout.as_nanos();
        if nanos == 0 || !nanos.is_multiple_of(1_000_000) {
            return Err(bsw_platform::WatchdogError::UnsupportedTimeout);
        }
        let millis = timeout.as_millis();
        if !(1..=4096).contains(&millis) {
            return Err(bsw_platform::WatchdogError::UnsupportedTimeout);
        }
        #[allow(clippy::cast_possible_truncation)]
        if self.start_with_timeout(millis as u32) {
            Ok(())
        } else {
            Err(bsw_platform::WatchdogError::HardwareFault)
        }
    }

    fn service(&mut self) {
        self.kick();
    }

    fn is_running(&self) -> bool {
        self.started
    }
}
