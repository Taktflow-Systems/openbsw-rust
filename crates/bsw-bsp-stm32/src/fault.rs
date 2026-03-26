//! HardFault handler — register dump to NO_INIT RAM for post-mortem.
//!
//! Captures the full CPU state (R0-R12, SP, LR, PC, xPSR) plus 240 bytes
//! of stack content into a fixed RAM region with XOR checksum.
//!
//! Maps to C++: `contribution/platforms/stm32/hardFaultHandler/src/hardFaultHandler.s`

use core::ptr;

/// NO_INIT RAM base address for F4 (320 KB SRAM, top 1 KB).
#[cfg(feature = "stm32f413")]
pub const NOINIT_BASE: u32 = 0x2004_FC00;

/// NO_INIT RAM base address for G4 (128 KB SRAM, top 1 KB).
#[cfg(feature = "stm32g474")]
pub const NOINIT_BASE: u32 = 0x2001_FC00;

/// Fallback for no-feature builds (host check).
#[cfg(not(any(feature = "stm32f413", feature = "stm32g474")))]
pub const NOINIT_BASE: u32 = 0x2000_0000;

/// Size of the fault dump in bytes (0x140 = 320 bytes).
pub const DUMP_SIZE: usize = 0x140;

/// Number of stack-content bytes captured after the exception frame.
pub const STACK_CAPTURE_BYTES: usize = 240;

/// Fault dump structure layout offsets (in 32-bit word indices).
#[allow(dead_code)]
mod offset {
    /// Handler xPSR at dump start.
    pub const HANDLER_PSR: usize = 0;
    /// Exception return LR (EXC_RETURN).
    pub const HANDLER_LR: usize = 1;
    /// Origin R0 (from exception frame).
    pub const ORIGIN_R0: usize = 2;
    /// Origin R1.
    pub const ORIGIN_R1: usize = 3;
    /// Origin R2.
    pub const ORIGIN_R2: usize = 4;
    /// Origin R3.
    pub const ORIGIN_R3: usize = 5;
    /// Origin R4 (manually saved).
    pub const ORIGIN_R4: usize = 6;
    /// Origin R5.
    pub const ORIGIN_R5: usize = 7;
    /// Origin R6.
    pub const ORIGIN_R6: usize = 8;
    /// Origin R7.
    pub const ORIGIN_R7: usize = 9;
    /// Origin R8.
    pub const ORIGIN_R8: usize = 10;
    /// Origin R9.
    pub const ORIGIN_R9: usize = 11;
    /// Origin R10.
    pub const ORIGIN_R10: usize = 12;
    /// Origin R11.
    pub const ORIGIN_R11: usize = 13;
    /// Origin R12 (from exception frame).
    pub const ORIGIN_R12: usize = 14;
    /// Origin SP (calculated: frame SP + 0x20).
    pub const ORIGIN_SP: usize = 15;
    /// Origin LR (from exception frame).
    pub const ORIGIN_LR: usize = 16;
    /// Origin PC (from exception frame — the faulting instruction).
    pub const ORIGIN_PC: usize = 17;
    /// Origin xPSR (from exception frame).
    pub const ORIGIN_PSR: usize = 18;
    /// Start of captured stack contents (60 words = 240 bytes).
    pub const STACK_START: usize = 19;
    /// XOR checksum (word index = DUMP_SIZE/4 - 1 = 79).
    pub const CHECKSUM: usize = DUMP_SIZE_WORDS - 1;

    const DUMP_SIZE_WORDS: usize = super::DUMP_SIZE / 4;
}

/// Write the fault dump to NO_INIT RAM.
///
/// # Safety
/// Called from the HardFault exception handler. `ef` must point to a valid
/// Cortex-M exception frame on the appropriate stack (MSP or PSP).
#[allow(clippy::too_many_lines)]
unsafe fn write_fault_dump(ef: &cortex_m_rt::ExceptionFrame) {
    let dump = NOINIT_BASE as *mut u32;
    let words = DUMP_SIZE / 4;

    // Zero the dump region
    for i in 0..words {
        unsafe { ptr::write_volatile(dump.add(i), 0) };
    }

    // Handler PSR
    let psr: u32;
    unsafe { core::arch::asm!("mrs {}, xpsr", out(reg) psr) };
    unsafe { ptr::write_volatile(dump.add(offset::HANDLER_PSR), psr) };

    // Exception return LR
    let exc_lr: u32;
    unsafe { core::arch::asm!("mov {}, lr", out(reg) exc_lr) };
    unsafe { ptr::write_volatile(dump.add(offset::HANDLER_LR), exc_lr) };

    // R0-R3 from exception frame (stacked by hardware)
    unsafe {
        ptr::write_volatile(dump.add(offset::ORIGIN_R0), ef.r0());
        ptr::write_volatile(dump.add(offset::ORIGIN_R1), ef.r1());
        ptr::write_volatile(dump.add(offset::ORIGIN_R2), ef.r2());
        ptr::write_volatile(dump.add(offset::ORIGIN_R3), ef.r3());
    }

    // R12, LR, PC, xPSR from exception frame
    unsafe {
        ptr::write_volatile(dump.add(offset::ORIGIN_R12), ef.r12());
        ptr::write_volatile(dump.add(offset::ORIGIN_LR), ef.lr());
        ptr::write_volatile(dump.add(offset::ORIGIN_PC), ef.pc());
        ptr::write_volatile(dump.add(offset::ORIGIN_PSR), ef.xpsr());
    }

    // Calculate original SP (exception frame pointer + 8 words)
    let frame_ptr = ef as *const cortex_m_rt::ExceptionFrame as u32;
    let original_sp = frame_ptr.wrapping_add(0x20); // 8 words = 32 bytes
    unsafe { ptr::write_volatile(dump.add(offset::ORIGIN_SP), original_sp) };

    // Capture stack contents (240 bytes / 60 words after the exception frame)
    let stack_ptr = original_sp as *const u32;
    let stack_words = STACK_CAPTURE_BYTES / 4;
    for i in 0..stack_words {
        let val = unsafe { ptr::read_volatile(stack_ptr.add(i)) };
        unsafe { ptr::write_volatile(dump.add(offset::STACK_START + i), val) };
    }

    // XOR checksum over all words except the checksum word itself
    let mut checksum: u32 = 0;
    for i in 0..(words - 1) {
        checksum ^= unsafe { ptr::read_volatile(dump.add(i)) };
    }
    unsafe { ptr::write_volatile(dump.add(offset::CHECKSUM), checksum) };
}

/// HardFault exception handler.
///
/// Dumps CPU state to NO_INIT RAM at [`NOINIT_BASE`], then enters an
/// infinite loop. A debugger or post-mortem tool can read the dump
/// and verify integrity via the XOR checksum.
#[cortex_m_rt::exception]
#[allow(non_snake_case)]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe { write_fault_dump(ef) };

    loop {
        cortex_m::asm::bkpt();
    }
}
