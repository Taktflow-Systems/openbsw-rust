//! RAM-only LC4357 PLL/DCC bring-up role.
//!
//! This no-stack A32 role implements the bounded SSWF021#45 PLL1 sequence,
//! then configures the exact 300/150/75-MHz clock plan and selects PLL1 only
//! after CSVSTAT, slip status, and DCC agree. It touches only system clock,
//! ESM slip-status, and DCC1 registers. It does not access flash-control,
//! pinmux, GPIO, PHY, EMAC, CAN, watchdog, security, or nonvolatile storage.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .arm
    .section .clock_probe,"ax",%progbits
    .global _tms570_clock_probe
    .global _tms570_clock_probe_pass
    .type _tms570_clock_probe,%function

    .equ SYS1,               0xffffff00
    .equ SYS2,               0xffffe100
    .equ ESM1,               0xfffff500
    .equ DCC1,               0xffffec00
    .equ PLL1_BIT,           0x00000002
    .equ PLL1_SLIP,          0x00000400
    .equ LOOP_LIMIT_HI,      0x0010
    .equ DCC_DISABLED,       0x00005a55
    .equ DCC_ENABLED,        0x00005a5a

_tms570_clock_probe:
    movw r0, #0xff00
    movt r0, #0xffff
    movw r1, #0xf500
    movt r1, #0xffff
    movw r2, #0xec00
    movt r2, #0xffff

    /* Fail closed unless the CPU-visible ID is LC4357 revision B. */
    ldr r3, [r0, #0xf0]
    movw r4, #0xad0d
    movt r4, #0x8044
    cmp r3, r4
    bne .Lfail_identity

    /* Keep every synchronous domain on OSCIN during PLL qualification. */
    mov r3, #0
    str r3, [r0, #0x48]
    dsb sy
    isb

    /* Peripheral-enable is required for DCC1 register access. */
    ldr r3, [r0, #0xd0]
    orr r3, r3, #0x100
    str r3, [r0, #0xd0]
    dsb sy

    mov r10, #1

.Ladvisory_attempt:
    /* Disable PLL1 and bound the wait for CSVSTAT to clear. */
    mov r3, #PLL1_BIT
    str r3, [r0, #0x34]
    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Ladvisory_disable_wait:
    ldr r5, [r0, #0x54]
    tst r5, #PLL1_BIT
    beq .Ladvisory_disabled
    subs r4, r4, #1
    bne .Ladvisory_disable_wait
    b .Lfail_advisory_disable

.Ladvisory_disabled:
    /* Clear both global and ESM PLL1 slip latches while PLL1 is off. */
    movw r3, #0x0301
    str r3, [r0, #0xec]
    movw r3, #PLL1_SLIP
    str r3, [r1, #0x18]

    /* TI SPNA233B LC4357 workaround operating point. */
    movw r3, #0x1a00
    movt r3, #0x2000
    str r3, [r0, #0x70]
    movw r3, #0x723d
    movt r3, #0x3fc0
    str r3, [r0, #0x74]
    mov r3, #PLL1_BIT
    str r3, [r0, #0x38]
    dsb sy

    /* Wait for either valid, slip, or the software bound. */
    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Ladvisory_lock_wait:
    ldr r5, [r0, #0x54]
    tst r5, #PLL1_BIT
    bne .Ladvisory_lock_observed
    ldr r6, [r1, #0x18]
    tst r6, #PLL1_SLIP
    bne .Ladvisory_failed
    subs r4, r4, #1
    bne .Ladvisory_lock_wait
    b .Ladvisory_failed

.Ladvisory_lock_observed:
    ldr r6, [r1, #0x18]
    tst r6, #PLL1_SLIP
    bne .Ladvisory_failed
    ldr r7, [r0, #0xec]
    tst r7, #0x300
    bne .Ladvisory_failed

    /* DCC1 single-shot: OSCIN against the workaround PLL1 frequency. */
    movw r3, #DCC_DISABLED
    str r3, [r2, #0x00]
    mov r3, #3
    str r3, [r2, #0x14]
    mov r3, #68
    str r3, [r2, #0x08]
    mov r3, #4
    str r3, [r2, #0x0c]
    movw r3, #972
    str r3, [r2, #0x10]
    movw r3, #0xa000
    str r3, [r2, #0x24]
    mov r3, #0xf
    str r3, [r2, #0x28]
    movw r3, #DCC_ENABLED
    str r3, [r2, #0x00]
    dsb sy
    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Ladvisory_dcc_wait:
    ldr r9, [r2, #0x14]
    cmp r9, #0
    bne .Ladvisory_dcc_observed
    subs r4, r4, #1
    bne .Ladvisory_dcc_wait
    b .Ladvisory_failed
.Ladvisory_dcc_observed:
    tst r9, #1
    bne .Ladvisory_failed
    tst r9, #2
    bne .Ladvisory_passed

.Ladvisory_failed:
    cmp r10, #5
    beq .Lfail_advisory_exhausted
    add r10, r10, #1
    b .Ladvisory_attempt

.Ladvisory_passed:
    /* Disable the workaround operating point before final PLL programming. */
    mov r3, #PLL1_BIT
    str r3, [r0, #0x34]
    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Lfinal_disable_wait:
    ldr r5, [r0, #0x54]
    tst r5, #PLL1_BIT
    beq .Lfinal_disabled
    subs r4, r4, #1
    bne .Lfinal_disable_wait
    b .Lfail_final_disable

.Lfinal_disabled:
    movw r3, #0x0301
    str r3, [r0, #0xec]
    movw r3, #PLL1_SLIP
    str r3, [r1, #0x18]

    /* 16 MHz / 8 * 150 / 1 / 32 during lock, then R becomes 1. */
    movw r3, #0x9500
    movt r3, #0x3f07
    str r3, [r0, #0x70]
    movw r3, #0x703d
    movt r3, #0x3fc0
    str r3, [r0, #0x74]
    mov r3, #PLL1_BIT
    str r3, [r0, #0x38]
    dsb sy

    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Lfinal_lock_wait:
    ldr r5, [r0, #0x54]
    tst r5, #PLL1_BIT
    bne .Lfinal_lock_observed
    ldr r6, [r1, #0x18]
    tst r6, #PLL1_SLIP
    bne .Lfail_final_slip
    subs r4, r4, #1
    bne .Lfinal_lock_wait
    b .Lfail_final_lock

.Lfinal_lock_observed:
    ldr r6, [r1, #0x18]
    tst r6, #PLL1_SLIP
    bne .Lfail_final_slip
    ldr r7, [r0, #0xec]
    tst r7, #0x300
    bne .Lfail_final_slip

    movw r3, #0x9500
    movt r3, #0x2007
    str r3, [r0, #0x70]
    dsb sy

    /* DCC1 single-shot: 300 MHz PLL1 within about +/-0.5% of 16 MHz. */
    movw r3, #DCC_DISABLED
    str r3, [r2, #0x00]
    mov r3, #3
    str r3, [r2, #0x14]
    movw r3, #1600
    str r3, [r2, #0x08]
    mov r3, #16
    str r3, [r2, #0x0c]
    movw r3, #0x75c6
    str r3, [r2, #0x10]
    movw r3, #0xa000
    str r3, [r2, #0x24]
    mov r3, #0xf
    str r3, [r2, #0x28]
    movw r3, #DCC_ENABLED
    str r3, [r2, #0x00]
    dsb sy
    movw r4, #0
    movt r4, #LOOP_LIMIT_HI
.Lfinal_dcc_wait:
    ldr r9, [r2, #0x14]
    cmp r9, #0
    bne .Lfinal_dcc_observed
    subs r4, r4, #1
    bne .Lfinal_dcc_wait
    b .Lfail_final_dcc
.Lfinal_dcc_observed:
    tst r9, #1
    bne .Lfail_final_dcc
    tst r9, #2
    beq .Lfail_final_dcc

    /* Program HCLK /2 and VCLK/VCLK2/VCLK3 /2 before selecting PLL1. */
    movw r3, #0xe100
    movt r3, #0xffff
    mov r4, #1
    str r4, [r3, #0x54]

    ldr r5, [r0, #0xd0]
    movw r4, #0
    movt r4, #0x0f0f
    bic r5, r5, r4
    movw r4, #0
    movt r4, #0x0101
    orr r5, r5, r4
    orr r5, r5, #0x100
    str r5, [r0, #0xd0]

    ldr r4, [r3, #0x3c]
    bic r4, r4, #0xf
    orr r4, r4, #1
    str r4, [r3, #0x3c]
    dsb sy

    movw r4, #0x0001
    movt r4, #0x0101
    str r4, [r0, #0x48]
    dsb sy
    isb

    /* Export only public status/register values through core registers. */
    ldr r8, [r0, #0x54]
    ldr r7, [r0, #0xec]
    ldr r6, [r1, #0x18]
    ldr r9, [r2, #0x14]
    ldr r5, [r0, #0xd0]
    ldr r4, [r0, #0x48]
    ldr r3, [r3, #0x54]
    movw r12, #0x4b50
    movt r12, #0x434c
_tms570_clock_probe_pass:
    b _tms570_clock_probe_pass

.Lfail_identity:
    movw r12, #0x0001
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_advisory_disable:
    movw r12, #0x0002
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_advisory_exhausted:
    movw r12, #0x0003
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_final_disable:
    movw r12, #0x0011
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_final_lock:
    movw r12, #0x0012
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_final_slip:
    movw r12, #0x0013
    movt r12, #0x4346
    b .Lfail_loop
.Lfail_final_dcc:
    movw r12, #0x0014
    movt r12, #0x4346
.Lfail_loop:
    b .Lfail_loop

    .size _tms570_clock_probe, . - _tms570_clock_probe
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
