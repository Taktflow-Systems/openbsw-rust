//! Nestable CPSR-based critical sections for the Cortex-R5F.
//!
//! The outermost acquire records both IRQ and FIQ mask bits, then masks both
//! classes with architectural barriers. The matching outermost release only
//! re-enables classes that were enabled on entry. Nested calls never overwrite
//! the saved entry state.

use core::cell::Cell;

use bsw_can::transceiver::InterruptLock;
use bsw_platform::CriticalSection;

use crate::board;

const FIQ_MASK: u32 = 1 << 6;
const IRQ_MASK: u32 = 1 << 7;

/// Saved architectural interrupt-mask state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct InterruptMask(u32);

#[cfg(any(target_arch = "arm", test))]
impl InterruptMask {
    const fn irq_was_enabled(self) -> bool {
        self.0 & IRQ_MASK == 0
    }

    const fn fiq_was_enabled(self) -> bool {
        self.0 & FIQ_MASK == 0
    }
}

/// Single-owner, nestable CPU interrupt critical section.
pub struct Tms570CriticalSection {
    _token: board::InterruptController,
    depth: Cell<u32>,
    entry_mask: Cell<InterruptMask>,
}

impl Tms570CriticalSection {
    /// Bind the unique board interrupt-control token.
    pub const fn from_token(token: board::InterruptController) -> Self {
        Self {
            _token: token,
            depth: Cell::new(0),
            entry_mask: Cell::new(InterruptMask(IRQ_MASK | FIQ_MASK)),
        }
    }

    fn acquire_inner(&self) {
        let depth = self.depth.get();
        assert!(depth != u32::MAX, "critical-section nesting overflow");
        if depth == 0 {
            self.entry_mask.set(read_and_disable_interrupts());
        }
        self.depth.set(depth + 1);
    }

    fn release_inner(&self) {
        let depth = self.depth.get();
        assert!(depth != 0, "unbalanced critical-section release");
        let next = depth - 1;
        self.depth.set(next);
        if next == 0 {
            restore_interrupts(self.entry_mask.get());
        }
    }
}

impl CriticalSection for Tms570CriticalSection {
    fn acquire(&mut self) {
        self.acquire_inner();
    }

    fn release(&mut self) {
        self.release_inner();
    }
}

impl InterruptLock for Tms570CriticalSection {
    fn lock(&self) {
        self.acquire_inner();
    }

    fn unlock(&self) {
        self.release_inner();
    }
}

#[cfg(target_arch = "arm")]
fn read_and_disable_interrupts() -> InterruptMask {
    let cpsr: u32;
    // SAFETY: MRS only observes CPSR. CPSID changes IRQ/FIQ masks for the
    // uniquely owned CPU lock; no memory or stack operand is accessed.
    unsafe {
        core::arch::asm!(
            "mrs {saved}, cpsr",
            "cpsid if",
            "dsb sy",
            "isb",
            saved = out(reg) cpsr,
            options(nomem, nostack, preserves_flags)
        );
    }
    InterruptMask(cpsr)
}

#[cfg(not(target_arch = "arm"))]
fn read_and_disable_interrupts() -> InterruptMask {
    InterruptMask(0)
}

#[cfg(target_arch = "arm")]
fn restore_interrupts(mask: InterruptMask) {
    // Complete protected memory operations before changing either mask.
    // SAFETY: only interrupt mask bits observed by the matching outer acquire
    // are restored. CPU mode, flags, and every other CPSR field are untouched.
    unsafe {
        core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
        if mask.irq_was_enabled() {
            core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
        }
        if mask.fiq_was_enabled() {
            core::arch::asm!("cpsie f", options(nomem, nostack, preserves_flags));
        }
        core::arch::asm!("isb", options(nomem, nostack, preserves_flags));
    }
}

#[cfg(not(target_arch = "arm"))]
fn restore_interrupts(_mask: InterruptMask) {}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_platform::CriticalSection;

    #[test]
    fn saved_mask_classifies_irq_and_fiq_independently() {
        assert!(InterruptMask(0).irq_was_enabled());
        assert!(InterruptMask(0).fiq_was_enabled());
        assert!(!InterruptMask(IRQ_MASK).irq_was_enabled());
        assert!(InterruptMask(IRQ_MASK).fiq_was_enabled());
        assert!(InterruptMask(FIQ_MASK).irq_was_enabled());
        assert!(!InterruptMask(FIQ_MASK).fiq_was_enabled());
    }

    #[test]
    fn nesting_restores_only_after_outer_release() {
        let mut section = Tms570CriticalSection::from_token(crate::board::interrupt_test_token());
        section.acquire();
        section.acquire();
        assert_eq!(section.depth.get(), 2);
        section.release();
        assert_eq!(section.depth.get(), 1);
        section.release();
        assert_eq!(section.depth.get(), 0);
    }

    #[test]
    #[should_panic(expected = "unbalanced critical-section release")]
    fn unbalanced_release_fails_closed() {
        let mut section = Tms570CriticalSection::from_token(crate::board::interrupt_test_token());
        section.release();
    }
}
