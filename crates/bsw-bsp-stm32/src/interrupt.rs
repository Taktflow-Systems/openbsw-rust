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
