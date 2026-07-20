//! RAM-only LC4357 RTI counter-0 bring-up role.
//!
//! This no-stack A32 role runs only after the separately inspected clock role
//! has established 75-MHz RTICLK. It divides counter 0 to 1 MHz, observes at
//! least 100,000 ticks with a software timeout, and then publishes a marker.
//! Counter 1 and every digital-windowed-watchdog register remain untouched.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .arm
    .section .rti_probe,"ax",%progbits
    .global _tms570_rti_probe
    .global _tms570_rti_probe_pass
    .type _tms570_rti_probe,%function

    .equ RTI,                0xfffffc00
    .equ RTI_100MS_TICKS,    100000

_tms570_rti_probe:
    movw r0, #0xfc00
    movt r0, #0xffff

    /* Prove privileged CPSR IRQ/FIQ masking and exact entry-state restore. */
    mrs r4, cpsr
    cpsid if
    dsb sy
    isb
    mrs r5, cpsr
    and r5, r5, #0xc0
    cmp r5, #0xc0
    bne .Lfail_critical_mask
    tst r4, #0x80
    bne .Lkeep_irq_masked
    cpsie i
.Lkeep_irq_masked:
    tst r4, #0x40
    bne .Lkeep_fiq_masked
    cpsie f
.Lkeep_fiq_masked:
    isb
    mrs r5, cpsr
    eor r5, r5, r4
    tst r5, #0xc0
    bne .Lfail_critical_restore

    /* Counter 0 only: stop, internal timebase, clear, divide 75 MHz by 75. */
    mov r1, #0
    str r1, [r0, #0x00]
    str r1, [r0, #0x04]
    str r1, [r0, #0x10]
    str r1, [r0, #0x14]
    mov r1, #74
    str r1, [r0, #0x18]
    mov r1, #1
    str r1, [r0, #0x00]
    dsb sy
    isb

    /* Require 100 ms of monotonic progress, but never wait forever. */
    movw r1, #0x86a0
    movt r1, #0x0001
    movw r2, #0
    movt r2, #0x0400
.Lwait_100ms:
    ldr r3, [r0, #0x10]
    cmp r3, r1
    bhs .Lpassed
    subs r2, r2, #1
    bne .Lwait_100ms

    movw r12, #0x0001
    movt r12, #0x5254
    b .Lfail_loop

.Lfail_critical_mask:
    movw r12, #0x0002
    movt r12, #0x5254
    b .Lfail_loop

.Lfail_critical_restore:
    movw r12, #0x0003
    movt r12, #0x5254
    b .Lfail_loop

.Lpassed:
    /* FRC0 then UC0 is the TRM-defined coherent observation order. */
    ldr r11, [r0, #0x10]
    ldr r10, [r0, #0x14]
    movw r12, #0x4950
    movt r12, #0x5254
_tms570_rti_probe_pass:
    b _tms570_rti_probe_pass

.Lfail_loop:
    b .Lfail_loop

    .size _tms570_rti_probe, . - _tms570_rti_probe
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
