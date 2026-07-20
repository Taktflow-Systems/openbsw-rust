//! A32 reset and exception-vector implementation for TMS570LC4357 revision B.
//!
//! This code is assembled only for the exact feature-selected Arm target. It
//! executes entirely from flash until SRAM and VIM RAM ECC initialization has
//! completed, uses registers only during that phase, preserves only the fixed
//! retained record on warm reset, and installs every required banked stack
//! before copying Rust data or calling hard-float Rust.

core::arch::global_asm!(
    r#"
    .syntax unified
    .arch armv7-r
    .arm
    .fpu vfpv3-d16

    .section .vectors.tms570, "ax", %progbits
    .balign 32
    .global __tms570_vector_table
    .type __tms570_vector_table, %object
__tms570_vector_table:
    b       _tms570_reset
    b       _tms570_undefined
    b       _tms570_svc
    b       _tms570_prefetch_abort
    b       _tms570_data_abort
    b       _tms570_reserved
    ldr     pc, [pc, #-0x1b0]
    ldr     pc, [pc, #-0x1b0]
    .size __tms570_vector_table, . - __tms570_vector_table

    .section .startup.tms570, "ax", %progbits
    .balign 4
    .global _tms570_reset
    .type _tms570_reset, %function
_tms570_reset:
    cpsid   aif

    /* Select the linker-owned A32 vector table before any exception can run. */
    ldr     r0, =__tms570_vector_table
    mcr     p15, 0, r0, c12, c0, 0
    isb

    /* Program the indeterminate VIM fallback before enabling VIM RAM ECC. */
    ldr     r0, =0xfffffdf8
    ldr     r1, =_tms570_default_interrupt
    str     r1, [r0]
    ldr     r0, =0xfffffdf0
    ldr     r1, =0x050a0a0a
    str     r1, [r0]

    /*
     * SYSESR.PORST is the cold/warm discriminator. Cold reset initializes all
     * eight SRAM banks. Warm reset skips bank 7 so the fixed retained record
     * survives; the remainder of that bank is then initialized by aligned
     * 64-bit CPU stores before any stack access. r4 remains register-only.
     */
    ldr     r0, =0xffffffe4
    ldr     r1, [r0]
    tst     r1, #0x8000
    moveq   r4, #1                 /* warm reset */
    movne   r4, #0                 /* power-on reset */

    ldr     r0, =0xfffff93c
    cmp     r4, #0
    moveq   r1, #0xff
    movne   r1, #0x7f
    str     r1, [r0]

    /* SYS.MINITGCR=Ah, then initialize L2 SRAM (bit 0) and VIM RAM (bit 2). */
    ldr     r0, =0xffffff5c
    mov     r1, #0x0a
    str     r1, [r0]
    ldr     r0, =0xffffff60
    mov     r1, #0x05
    str     r1, [r0]
    dsb     sy

    /* Fail closed after a finite register-only observation budget. */
    ldr     r0, =0xffffff68
    ldr     r2, =0x01000000
1:
    ldr     r1, [r0]
    tst     r1, #0x100
    bne     2f
    subs    r2, r2, #1
    bne     1b
    b       _tms570_memory_init_failed
2:
    ldr     r0, =0xffffff5c
    mov     r1, #0x05
    str     r1, [r0]
    dsb     sy
    isb

    /*
     * L2 SRAM writes generate ECC. On warm reset this covers every 64-bit
     * word in bank 7 except the linker-owned 160-byte retained record.
     */
    cmp     r4, #0
    beq     7f
    ldr     r0, =__retained_end
    ldr     r1, =0x08080000
    mov     r2, #0
    mov     r3, #0
6:
    strd    r2, r3, [r0], #8
    cmp     r0, r1
    blo     6b
    dsb     sy
7:
    /* Clear the consumed POR indication so a later software reset is warm. */
    ldr     r0, =0xffffffe4
    mov     r1, #0x8000
    str     r1, [r0]
    dsb     sy

    /* Install an aligned SP for every required banked exception mode. */
    cps     #0x13
    ldr     sp, =__stack_supervisor_top
    cps     #0x17
    ldr     sp, =__stack_abort_top
    cps     #0x1b
    ldr     sp, =__stack_undefined_top
    cps     #0x12
    ldr     sp, =__stack_irq_top
    cps     #0x11
    ldr     sp, =__stack_fiq_top
    cps     #0x1f
    ldr     sp, =__stack_system_top

    /* Permit CP10/CP11 and enable the VFP before hard-float Rust is called. */
    mrc     p15, 0, r0, c1, c0, 2
    orr     r0, r0, #0x00f00000
    mcr     p15, 0, r0, c1, c0, 2
    isb
    mov     r0, #0x40000000
    vmsr    fpexc, r0

    /* Copy .data using aligned words. */
    ldr     r0, =__data_load
    ldr     r1, =__data_start
    ldr     r2, =__data_end
3:
    cmp     r1, r2
    ldrlo   r3, [r0], #4
    strlo   r3, [r1], #4
    blo     3b

    /* Clear .bss using aligned words; .noinit remains untouched. */
    mov     r3, #0
    ldr     r1, =__bss_start
    ldr     r2, =__bss_end
4:
    cmp     r1, r2
    strlo   r3, [r1], #4
    blo     4b

    /* Auto-init created valid ECC; now install all 128 VIM vector words. */
    ldr     r0, =0xfff82000
    ldr     r1, =_tms570_default_interrupt
    mov     r2, #128
5:
    str     r1, [r0], #4
    subs    r2, r2, #1
    bne     5b
    dsb     sy

    bl      tms570_rust_entry
    b       _tms570_rust_returned
    .size _tms570_reset, . - _tms570_reset

    .global _tms570_reserved
    .type _tms570_reserved, %function
_tms570_reserved:
    b       _tms570_reserved

    .global _tms570_default_interrupt
    .type _tms570_default_interrupt, %function
_tms570_default_interrupt:
    b       _tms570_default_interrupt

    .global _tms570_memory_init_failed
    .type _tms570_memory_init_failed, %function
_tms570_memory_init_failed:
    b       _tms570_memory_init_failed

    .global _tms570_rust_returned
    .type _tms570_rust_returned, %function
_tms570_rust_returned:
    b       _tms570_rust_returned
"#
);

/// Keep the target startup object in compile-only inspection images.
///
/// Calling this function has no hardware effect. The startup probe references
/// it solely so the linker must extract this object and its vector assembly
/// from the BSP archive.
#[inline(never)]
pub extern "C" fn link_startup_probe() {}
