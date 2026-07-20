//! First physical LC4357 bring-up role: a register-only SRAM marker.
//!
//! The entry point executes three A32 instructions from a linker-bounded SRAM
//! window: write the public marker to R12 and spin. It uses no stack, performs
//! no data access or MMIO, and contains no flash section.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .arm
    .section .ram_marker,"ax",%progbits
    .global _tms570_ram_marker
    .type _tms570_ram_marker,%function
_tms570_ram_marker:
    movw r12, #0x5332
    movt r12, #0x544d
1:
    b 1b
    .size _tms570_ram_marker, . - _tms570_ram_marker
"#
);

#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
