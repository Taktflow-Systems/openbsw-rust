//! PRIMASK-based interrupt lock — implements `InterruptLock` trait.
//!
//! Uses CPSID I / CPSIE I with full memory barriers (ISB+DSB+DMB)
//! for critical-section protection on Cortex-M4 cores.
//!
//! Maps to C++: `contribution/platforms/stm32/bsp/bspInterruptsImpl/`

use bsw_can::transceiver::InterruptLock;

/// Global interrupt lock using ARM PRIMASK register.
///
/// `lock()` disables all maskable interrupts (CPSID I).
/// `unlock()` re-enables them (CPSIE I).
///
/// # Safety
/// Calls must be balanced. Nested locking is NOT supported — the first
/// `unlock()` will re-enable interrupts regardless of nesting depth.
/// For nested critical sections, use a counter-based wrapper.
pub struct PrimaskLock;

impl PrimaskLock {
    /// Create a new interrupt lock (zero-sized, no state).
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl InterruptLock for PrimaskLock {
    fn lock(&self) {
        // CPSID I — disable IRQs
        cortex_m::interrupt::disable();
        // Barriers after disable (ensure no reordering past the lock)
        cortex_m::asm::isb();
        cortex_m::asm::dsb();
        cortex_m::asm::dmb();
    }

    fn unlock(&self) {
        // Barriers before enable (flush all writes before releasing)
        cortex_m::asm::isb();
        cortex_m::asm::dsb();
        cortex_m::asm::dmb();
        // CPSIE I — re-enable IRQs
        // SAFETY: Caller guarantees balanced lock/unlock and that enabling
        // interrupts at this point is safe.
        unsafe { cortex_m::interrupt::enable() };
    }
}

impl Default for PrimaskLock {
    fn default() -> Self {
        Self::new()
    }
}

/// Nestable critical section which restores the entry PRIMASK state.
pub struct Stm32CriticalSection {
    depth: u32,
    was_masked: bool,
}

impl Stm32CriticalSection {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            depth: 0,
            was_masked: false,
        }
    }
}

impl Default for Stm32CriticalSection {
    fn default() -> Self {
        Self::new()
    }
}

impl bsw_platform::CriticalSection for Stm32CriticalSection {
    fn acquire(&mut self) {
        if self.depth == 0 {
            self.was_masked = cortex_m::register::primask::read().is_active();
            cortex_m::interrupt::disable();
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
        self.depth = self.depth.saturating_add(1);
    }

    fn release(&mut self) {
        assert!(self.depth != 0, "unbalanced critical-section release");
        self.depth -= 1;
        if self.depth == 0 && !self.was_masked {
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            // SAFETY: the outermost acquire observed interrupts enabled.
            unsafe { cortex_m::interrupt::enable() };
        }
    }
}
