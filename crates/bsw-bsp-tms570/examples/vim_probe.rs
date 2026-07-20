//! RAM-only LC4357 VIM IRQ/FIQ dispatch role.
//!
//! This A32 role runs after the separately qualified 75-MHz RTICLK. It
//! initializes only VIM RAM, installs banked stacks, and uses RTI compare 0
//! (fixed VIM channel 2) as an internal stimulus first for IRQ and then FIQ.
//! The physical script first verifies the fixed flash IRQ/FIQ words are the
//! standard VIM trampolines; this role never writes nonvolatile memory.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .arch armv7-r
    .arm

    .equ SYS1,               0xffffff00
    .equ RTI,                0xfffffc00
    .equ VIM,                0xfffffd00
    .equ VIM_RAM,            0xfff82000
    .equ STATE,              0x08075c00
    .equ IRQ_STACK_TOP,      0x08075e00
    .equ FIQ_STACK_TOP,      0x08075f00
    .equ LOOP_LIMIT_HI,      0x0400
    .equ RTI_COMPARE_TICKS,  10000
    .equ RTI_CHANNEL_BIT,    0x00000004

    .section .vim_probe,"ax",%progbits
    .global _tms570_vim_probe
    .global _tms570_vim_probe_pass
    .type _tms570_vim_probe,%function

_tms570_vim_probe:
    movw r12, #0x0001
    movt r12, #0x5653
    cpsid if

    /* Install bounded banked stacks before either exception is enabled. */
    cps #0x12
    movw r0, #0x5e00
    movt r0, #0x0807
    mov sp, r0
    cps #0x11
    movw r0, #0x5f00
    movt r0, #0x0807
    mov sp, r0
    cps #0x13
    movw r12, #0x0002
    movt r12, #0x5653

    /* This device configuration uses fixed vectors; flash is read-only. */
    movw r12, #0x0003
    movt r12, #0x5653

    /* Clear private observation words. */
    movw r7, #0x5c00
    movt r7, #0x0807
    mov r6, #0
    str r6, [r7, #0]
    str r6, [r7, #4]
    str r6, [r7, #8]
    str r6, [r7, #12]
    str r6, [r7, #16]
    str r6, [r7, #20]
    str r6, [r7, #24]
    str r6, [r7, #28]
    str r6, [r7, #32]
    str r6, [r7, #36]
    str r6, [r7, #40]
    str r6, [r7, #44]
    str r6, [r7, #48]
    str r6, [r7, #52]
    movw r12, #0x0004
    movt r12, #0x5653

    /*
     * DEVICE#56 may assert only ESM Group-2 channel 2 on initial debugger
     * connection. Reject every other live or shadow Group-2 bit, preserve all
     * Group-1/3 status, acknowledge both Group-2 latches, and use the
     * DEVICE#60-safe error-pin recovery only when required.
     */
    movw r6, #0xf500
    movt r6, #0xffff
    ldr r4, [r6, #0x1c]
    str r4, [r7, #16]
    bic r5, r4, #0x04
    cmp r5, #0
    bne .Lfail_esm_precondition
    ldr r5, [r6, #0x3c]
    str r5, [r7, #20]
    bic r5, r5, #0x04
    cmp r5, #0
    bne .Lfail_esm_precondition
    ldr r5, [r6, #0x24]
    str r5, [r7, #24]
    ldr r8, [r6, #0x18]
    str r8, [r7, #36]
    ldr r8, [r6, #0x20]
    str r8, [r7, #40]
    ldr r8, [r6, #0x58]
    str r8, [r7, #44]
    ldr r8, [r6, #0x5c]
    str r8, [r7, #48]
    ldr r8, [r6, #0x60]
    str r8, [r7, #52]
    ldr r5, [r7, #16]
    ldr r4, [r7, #20]
    orrs r5, r5, r4
    beq .Lesm_status_clear
    mov r5, #0x04
    str r5, [r6, #0x1c]
    str r5, [r6, #0x3c]
.Lesm_status_clear:
    ldr r5, [r7, #24]
    tst r5, #1
    bne .Lesm_recovered
    mov r5, #0x0a
    str r5, [r6, #0x38]
    mov r5, #0x05
    str r5, [r6, #0x38]
    dsb sy
.Lesm_recovered:
    ldr r4, [r6, #0x1c]
    cmp r4, #0
    bne .Lfail_esm_recovery
    ldr r4, [r6, #0x3c]
    cmp r4, #0
    bne .Lfail_esm_recovery
    ldr r4, [r6, #0x24]
    tst r4, #1
    beq .Lfail_esm_recovery

    movw r2, #0xfd00
    movt r2, #0xffff
    adr r4, _tms570_default_interrupt
    str r4, [r2, #0xf8]

    /* Generate valid VIM RAM ECC without touching the executing SRAM bank. */
    movw r1, #0xff00
    movt r1, #0xffff
    mov r4, #0x0a
    str r4, [r1, #0x5c]
    mov r4, #0x04
    str r4, [r1, #0x60]
    dsb sy
    movw r5, #0
    movt r5, #LOOP_LIMIT_HI
.Lvim_init_wait:
    ldr r4, [r1, #0x68]
    tst r4, #0x100
    bne .Lvim_init_done
    subs r5, r5, #1
    bne .Lvim_init_wait
    b .Lfail_vim_init
.Lvim_init_done:
    mov r4, #0x05
    str r4, [r1, #0x5c]
    dsb sy
    isb

    movw r12, #0x0005
    movt r12, #0x5653

    /* Register RTI channel 2 at word 3 (word 0 is phantom). */
    movw r3, #0x2000
    movt r3, #0xfff8
    adr r4, _tms570_irq_vector_entry
    str r4, [r3, #12]

    /*
     * Remove only this role's channel-2 state from any interrupted prior run,
     * then refresh any debugger-connect arbitration. SPNU563A specifies that
     * reading FIQINDEX refreshes the latch from currently pending inputs.
     * Perform one bounded refresh while FIQ remains masked and require
     * quiescence before generating fresh stimuli.
     */
    mov r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x140]
    movw r0, #0xfc00
    movt r0, #0xffff
    mov r4, #1
    str r4, [r0, #0x84]
    str r4, [r0, #0x88]
    mov r4, #0
    str r4, [r0, #0x00]
    dsb sy
    ldr r4, [r2, #0x104]
    str r4, [r7, #28]
    ldr r5, [r2, #0x104]
    str r5, [r7, #32]
    cmp r4, #1
    bhi .Lfail_fiq_refresh
    cmp r5, #0
    bne .Lfail_fiq_refresh

    /* Route channel 2 to IRQ and enable exactly that request. */
    ldr r4, [r2, #0x110]
    bic r4, r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x110]
    mov r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x140]
    str r4, [r2, #0x130]
    dsb sy
    movw r12, #0x0006
    movt r12, #0x5653

    /* One-shot RTI compare 0 at 10 ms using the proven 1-MHz counter. */
    movw r0, #0xfc00
    movt r0, #0xffff
    mov r4, #0
    str r4, [r0, #0x00]
    str r4, [r0, #0x04]
    str r4, [r0, #0x0c]
    str r4, [r0, #0x10]
    str r4, [r0, #0x14]
    mov r4, #74
    str r4, [r0, #0x18]
    movw r4, #RTI_COMPARE_TICKS
    str r4, [r0, #0x50]
    mov r4, #0
    str r4, [r0, #0x54]
    mov r4, #1
    str r4, [r0, #0x88]
    str r4, [r0, #0x80]
    str r4, [r0, #0x00]
    dsb sy
    movw r12, #0x0007
    movt r12, #0x5653
    cpsie i

    movw r5, #0
    movt r5, #LOOP_LIMIT_HI
.Lirq_wait:
    ldr r4, [r7, #0]
    cmp r4, #1
    beq .Lirq_passed
    subs r5, r5, #1
    bne .Lirq_wait
    b .Lfail_irq
.Lirq_passed:
    cpsid i
    movw r12, #0x0008
    movt r12, #0x5653
    mov r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x140]

    /* Re-register the same source as FIQ, then generate a fresh one-shot. */
    adr r4, _tms570_fiq_vector_entry
    str r4, [r3, #12]
    ldr r4, [r2, #0x110]
    orr r4, r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x110]
    mov r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x130]

    mov r4, #0
    str r4, [r0, #0x00]
    str r4, [r0, #0x10]
    str r4, [r0, #0x14]
    movw r4, #RTI_COMPARE_TICKS
    str r4, [r0, #0x50]
    mov r4, #1
    str r4, [r0, #0x88]
    str r4, [r0, #0x00]
    dsb sy
    movw r12, #0x0009
    movt r12, #0x5653
    cpsie f

    movw r5, #0
    movt r5, #LOOP_LIMIT_HI
.Lfiq_wait:
    ldr r4, [r7, #4]
    cmp r4, #1
    beq .Lfiq_passed
    subs r5, r5, #1
    bne .Lfiq_wait
    b .Lfail_fiq
.Lfiq_passed:
    cpsid if
    mov r4, #RTI_CHANNEL_BIT
    str r4, [r2, #0x140]
    mov r4, #1
    str r4, [r0, #0x84]
    mov r4, #0
    str r4, [r0, #0x00]
    dsb sy

    /* Both hardware index registers must have identified channel 2 as 3. */
    ldr r8, [r7, #0]
    ldr r9, [r7, #4]
    ldr r10, [r7, #8]
    ldr r11, [r7, #12]
    ldr r6, [r7, #28]
    ldr r5, [r7, #32]
    cmp r8, #1
    bne .Lfail_result
    cmp r9, #1
    bne .Lfail_result
    cmp r10, #3
    bne .Lfail_result
    cmp r11, #3
    bne .Lfail_result
    cmp r6, #1
    bhi .Lfail_result
    cmp r5, #0
    bne .Lfail_result
    movw r12, #0x4d50
    movt r12, #0x5649
_tms570_vim_probe_pass:
    b _tms570_vim_probe_pass

    .size _tms570_vim_probe, . - _tms570_vim_probe

    .type _tms570_irq_vector_entry,%function
    .global _tms570_irq_vector_entry
_tms570_irq_vector_entry:
    stmdb sp!, {{r0-r3, r12, lr}}
    movw r0, #0xfe00
    movt r0, #0xffff
    ldr r1, [r0]
    str r1, [r7, #8]
    movw r0, #0xfc00
    movt r0, #0xffff
    mov r1, #1
    str r1, [r0, #0x88]
    ldr r1, [r7, #0]
    add r1, r1, #1
    str r1, [r7, #0]
    dsb sy
    ldmia sp!, {{r0-r3, r12, lr}}
    subs pc, lr, #4

    .type _tms570_fiq_vector_entry,%function
    .global _tms570_fiq_vector_entry
_tms570_fiq_vector_entry:
    movw r10, #0xfe04
    movt r10, #0xffff
    ldr r11, [r10]
    movw r12, #0x5c00
    movt r12, #0x0807
    str r11, [r12, #12]
    movw r10, #0xfc00
    movt r10, #0xffff
    mov r11, #1
    str r11, [r10, #0x88]
    ldr r11, [r12, #4]
    add r11, r11, #1
    str r11, [r12, #4]
    dsb sy
    subs pc, lr, #4

_tms570_default_interrupt:
    cpsid if
    movw r12, #0x0010
    movt r12, #0x5646
    b .Lfail_loop

.Lfail_vim_init:
    movw r12, #0x0001
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_esm_precondition:
    movw r12, #0x0005
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_esm_recovery:
    movw r12, #0x0006
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_fiq_refresh:
    movw r12, #0x0007
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_irq:
    cpsid if
    movw r12, #0x0002
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_fiq:
    cpsid if
    movw r12, #0x0003
    movt r12, #0x5646
    b .Lfail_loop
.Lfail_result:
    movw r12, #0x0004
    movt r12, #0x5646
.Lfail_loop:
    b .Lfail_loop
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
