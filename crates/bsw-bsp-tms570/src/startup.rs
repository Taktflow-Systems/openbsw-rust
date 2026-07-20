//! Startup ordering and exception-mode stack layout model.
//!
//! The target-only reset/vector implementation follows this same state model.
//! Its compile-only inspection image is deliberately not authorized for a
//! target load: physical validation remains pending. The model makes the
//! critical LC4357 rule testable on the host: no Rust stack or static SRAM
//! access is valid until level-2 SRAM and ECC auto-initialization completes.

use crate::device::{MemoryRegion, SRAM};

const STACK_ALIGNMENT: u32 = 8;

/// Startup sequencing error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartupError {
    /// A phase was attempted before its prerequisite.
    WrongOrder,
    /// Stack size was zero or not eight-byte aligned.
    InvalidStackSize,
    /// Reserved static data or stacks do not fit implemented SRAM.
    SramExhausted,
    /// Static-data end was outside implemented SRAM or misaligned.
    InvalidStaticEnd,
}

/// Bring-up phase with SRAM/ECC initialization ahead of any stack use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartupPhase {
    /// Reset handler is using registers only.
    RegisterOnly,
    /// SRAM and its ECC contain a valid zero codeword.
    SramInitialized,
    /// Every required banked mode SP has been installed.
    ModeStacksInstalled,
    /// `.data`/`.bss` initialization may use the system stack.
    RustMemoryInitialized,
}

/// Checked startup phase tracker used by bring-up tests and the future reset
/// implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartupSequence {
    phase: StartupPhase,
}

impl StartupSequence {
    /// Begin at reset without touching SRAM.
    pub const fn at_reset() -> Self {
        Self {
            phase: StartupPhase::RegisterOnly,
        }
    }

    /// Current phase.
    pub const fn phase(self) -> StartupPhase {
        self.phase
    }

    /// Record completion of hardware SRAM/ECC auto-initialization.
    pub fn mark_sram_initialized(&mut self) -> Result<(), StartupError> {
        self.advance(StartupPhase::RegisterOnly, StartupPhase::SramInitialized)
    }

    /// Record installation of SVC, SYS, IRQ, FIQ, abort and undefined stacks.
    pub fn mark_mode_stacks_installed(&mut self) -> Result<(), StartupError> {
        self.advance(
            StartupPhase::SramInitialized,
            StartupPhase::ModeStacksInstalled,
        )
    }

    /// Record completion of `.data` copy and `.bss` clearing.
    pub fn mark_rust_memory_initialized(&mut self) -> Result<(), StartupError> {
        self.advance(
            StartupPhase::ModeStacksInstalled,
            StartupPhase::RustMemoryInitialized,
        )
    }

    fn advance(&mut self, expected: StartupPhase, next: StartupPhase) -> Result<(), StartupError> {
        if self.phase != expected {
            return Err(StartupError::WrongOrder);
        }
        self.phase = next;
        Ok(())
    }
}

/// One downward-growing banked stack range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StackRange {
    /// Lowest byte reserved for the stack.
    pub bottom: u32,
    /// Initial SP, one byte past the reserved range.
    pub top: u32,
}

impl StackRange {
    /// Stack capacity in bytes.
    pub const fn len(self) -> u32 {
        self.top - self.bottom
    }

    /// Stack ranges produced by this module are never empty.
    pub const fn is_empty(self) -> bool {
        self.bottom == self.top
    }
}

/// Requested stack capacities for every required Cortex-R5F mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeStackSizes {
    /// System/user runtime stack.
    pub system: u32,
    /// Supervisor stack used by reset and SVC.
    pub supervisor: u32,
    /// IRQ stack.
    pub irq: u32,
    /// FIQ stack.
    pub fiq: u32,
    /// Data/prefetch abort stack.
    pub abort: u32,
    /// Undefined-instruction stack.
    pub undefined: u32,
}

/// Concrete non-overlapping ranges allocated from the top of SRAM downward.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeStackLayout {
    /// System/user runtime stack.
    pub system: StackRange,
    /// Supervisor stack.
    pub supervisor: StackRange,
    /// IRQ stack.
    pub irq: StackRange,
    /// FIQ stack.
    pub fiq: StackRange,
    /// Abort stack.
    pub abort: StackRange,
    /// Undefined-instruction stack.
    pub undefined: StackRange,
}

impl ModeStackLayout {
    /// Allocate aligned stacks above `static_end` in exact LC4357 SRAM.
    pub fn allocate(static_end: u32, sizes: ModeStackSizes) -> Result<Self, StartupError> {
        if !SRAM.contains(static_end) || !static_end.is_multiple_of(STACK_ALIGNMENT) {
            return Err(StartupError::InvalidStaticEnd);
        }
        validate_sizes(sizes)?;
        let mut cursor = SRAM.end;
        let system = take_stack(&mut cursor, sizes.system, static_end)?;
        let supervisor = take_stack(&mut cursor, sizes.supervisor, static_end)?;
        let irq = take_stack(&mut cursor, sizes.irq, static_end)?;
        let fiq = take_stack(&mut cursor, sizes.fiq, static_end)?;
        let abort = take_stack(&mut cursor, sizes.abort, static_end)?;
        let undefined = take_stack(&mut cursor, sizes.undefined, static_end)?;
        Ok(Self {
            system,
            supervisor,
            irq,
            fiq,
            abort,
            undefined,
        })
    }

    /// Entire stack reservation.
    pub const fn region(self) -> MemoryRegion {
        MemoryRegion::new(self.undefined.bottom, self.system.top)
    }
}

fn validate_sizes(sizes: ModeStackSizes) -> Result<(), StartupError> {
    for size in [
        sizes.system,
        sizes.supervisor,
        sizes.irq,
        sizes.fiq,
        sizes.abort,
        sizes.undefined,
    ] {
        if size == 0 || size % STACK_ALIGNMENT != 0 {
            return Err(StartupError::InvalidStackSize);
        }
    }
    Ok(())
}

fn take_stack(cursor: &mut u32, size: u32, static_end: u32) -> Result<StackRange, StartupError> {
    let top = *cursor;
    let Some(bottom) = top.checked_sub(size) else {
        return Err(StartupError::SramExhausted);
    };
    if bottom < static_end || !SRAM.contains(bottom) {
        return Err(StartupError::SramExhausted);
    }
    *cursor = bottom;
    Ok(StackRange { bottom, top })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIZES: ModeStackSizes = ModeStackSizes {
        system: 4096,
        supervisor: 1024,
        irq: 2048,
        fiq: 512,
        abort: 512,
        undefined: 512,
    };

    #[test]
    fn startup_forbids_stack_use_before_ecc_initialization() {
        let mut sequence = StartupSequence::at_reset();
        assert_eq!(
            sequence.mark_mode_stacks_installed(),
            Err(StartupError::WrongOrder)
        );
        sequence.mark_sram_initialized().unwrap();
        sequence.mark_mode_stacks_installed().unwrap();
        sequence.mark_rust_memory_initialized().unwrap();
        assert_eq!(sequence.phase(), StartupPhase::RustMemoryInitialized);
    }

    #[test]
    fn every_banked_stack_is_aligned_and_non_overlapping() {
        let layout = ModeStackLayout::allocate(0x0801_0000, SIZES).unwrap();
        let stacks = [
            layout.system,
            layout.supervisor,
            layout.irq,
            layout.fiq,
            layout.abort,
            layout.undefined,
        ];
        for stack in stacks {
            assert_eq!(stack.bottom % STACK_ALIGNMENT, 0);
            assert_eq!(stack.top % STACK_ALIGNMENT, 0);
            assert!(SRAM.contains_range(stack.bottom, stack.len()));
        }
        for pair in stacks.windows(2) {
            assert_eq!(pair[1].top, pair[0].bottom);
        }
        assert_eq!(layout.region().end, SRAM.end);
    }

    #[test]
    fn invalid_or_exhausted_layout_fails_closed() {
        let mut invalid = SIZES;
        invalid.fiq = 7;
        assert_eq!(
            ModeStackLayout::allocate(0x0801_0000, invalid),
            Err(StartupError::InvalidStackSize)
        );
        assert_eq!(
            ModeStackLayout::allocate(0x0807_f000, SIZES),
            Err(StartupError::SramExhausted)
        );
    }
}
