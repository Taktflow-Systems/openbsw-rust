//! RAM-only LC4357 retained exception and abort qualification role.
//!
//! The debugger supplies a bounded selector and generation in r0/r1. The role
//! installs private banked stacks, arms the production retained record with a
//! debug-vector provenance flag, and injects exactly one non-destructive A32
//! undefined, SVC, instruction-fetch abort, or data-read abort. Because this
//! device uses fixed flash vectors, the private runner catches the relevant
//! vector after architectural entry and redirects only PC to the matching RAM
//! handler. The production handler then captures context and requests a full
//! software reset. No probe instruction writes MMIO or nonvolatile memory;
//! only the shared terminal handler writes the documented reset request.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
core::arch::global_asm!(
    r#"
    .syntax unified
    .arch armv7-r
    .arm

    .equ RECORD_MAGIC,           0x544d5346
    .equ RECORD_FORMAT,          0x00010028
    .equ STATE_ARMED,            0x41524d44
    .equ COMMIT_ARMED,           0x41524d43
    .equ FLAG_DEBUG_REDIRECT,    0x00000008
    .equ RECORD_CRC_BYTES,       144

    .section .exception_probe,"ax",%progbits
    .balign 4
    .global _tms570_exception_probe
    .global _tms570_exception_probe_fail
    .global _tms570_exception_inject_undefined
    .global _tms570_exception_inject_svc
    .global _tms570_exception_inject_prefetch_abort
    .global _tms570_exception_inject_data_abort
    .global _tms570_exception_fault_undefined
    .global _tms570_exception_fault_svc
    .global _tms570_exception_fault_data_abort
    .type _tms570_exception_probe,%function

_tms570_exception_probe:
    cpsid   aif
    mov     r8, r0                  /* selector: 1..4 */
    mov     r9, r1                  /* private monotonic generation */

    /* Require the proven BE32 A32 source state before touching the record. */
    mrs     r2, cpsr
    tst     r2, #0x200
    beq     _tms570_exception_probe_fail
    tst     r2, #0x20
    bne     _tms570_exception_probe_fail

    /* Install all modes used by this role before arming an exception. */
    cps     #0x1b
    movw    sp, #0x4200
    movt    sp, #0x0807
    cps     #0x17
    movw    sp, #0x4400
    movt    sp, #0x0807
    cps     #0x13
    movw    sp, #0x4600
    movt    sp, #0x0807
    cps     #0x1f
    movw    sp, #0x4800
    movt    sp, #0x0807

    /* Build the Armed envelope; the commit and inverse are written last. */
    ldr     r10, =__tms570_exception_record
    mov     r0, #0
    mov     r1, #40
    mov     r2, r10
.Larm_zero:
    str     r0, [r2], #4
    subs    r1, r1, #1
    bne     .Larm_zero
    ldr     r0, =RECORD_MAGIC
    str     r0, [r10, #0]
    ldr     r0, =RECORD_FORMAT
    str     r0, [r10, #4]
    ldr     r0, =STATE_ARMED
    str     r0, [r10, #8]
    mvn     r0, r0
    str     r0, [r10, #12]
    str     r9, [r10, #16]
    mvn     r0, r9
    str     r0, [r10, #20]
    mov     r0, #0
    str     r0, [r10, #24]
    mvn     r0, r0
    str     r0, [r10, #28]
    mov     r0, #FLAG_DEBUG_REDIRECT
    str     r0, [r10, #140]
    mov     r0, r10
    mov     r1, #RECORD_CRC_BYTES
    bl      _tms570_exception_crc32
    str     r0, [r10, #144]
    mvn     r1, r0
    str     r1, [r10, #148]
    dsb     sy
    ldr     r0, =COMMIT_ARMED
    str     r0, [r10, #152]
    mvn     r0, r0
    str     r0, [r10, #156]
    dsb     sy
    isb

    cmp     r8, #1
    beq     _tms570_exception_inject_undefined
    cmp     r8, #2
    beq     _tms570_exception_inject_svc
    cmp     r8, #3
    beq     _tms570_exception_inject_prefetch_abort
    cmp     r8, #4
    beq     _tms570_exception_inject_data_abort
    b       _tms570_exception_probe_fail

    /* Distinct r0-r12 words make the entry-frame copy independently visible. */
    .macro SOURCE_REGISTERS
    movw    r0,  #0x0000
    movt    r0,  #0xa500
    movw    r1,  #0x0001
    movt    r1,  #0xa501
    movw    r2,  #0x0002
    movt    r2,  #0xa502
    movw    r3,  #0x0003
    movt    r3,  #0xa503
    movw    r4,  #0x0004
    movt    r4,  #0xa504
    movw    r5,  #0x0005
    movt    r5,  #0xa505
    movw    r6,  #0x0006
    movt    r6,  #0xa506
    movw    r7,  #0x0007
    movt    r7,  #0xa507
    movw    r8,  #0x0008
    movt    r8,  #0xa508
    movw    r9,  #0x0009
    movt    r9,  #0xa509
    movw    r10, #0x000a
    movt    r10, #0xa50a
    movw    r11, #0x000b
    movt    r11, #0xa50b
    movw    r12, #0x000c
    movt    r12, #0xa50c
    movw    lr,  #0x001e
    movt    lr,  #0xa51e
    .endm

_tms570_exception_inject_undefined:
    SOURCE_REGISTERS
_tms570_exception_fault_undefined:
    udf     #0x13

_tms570_exception_inject_svc:
    SOURCE_REGISTERS
_tms570_exception_fault_svc:
    svc     #0x13

_tms570_exception_inject_prefetch_abort:
    SOURCE_REGISTERS
    /* Read/fetch-only unimplemented gap; no peripheral write is attempted. */
    movw    lr, #0x0000
    movt    lr, #0x0400
    bx      lr

_tms570_exception_inject_data_abort:
    SOURCE_REGISTERS
    /* DEVICE#40 preserves abort behavior for reads from unimplemented space. */
    movw    lr, #0x0000
    movt    lr, #0x0400
_tms570_exception_fault_data_abort:
    ldr     lr, [lr]

_tms570_exception_probe_fail:
    cpsid   aif
    movw    r12, #0x464c
    movt    r12, #0x5845
    b       _tms570_exception_probe_fail
    .size _tms570_exception_probe, . - _tms570_exception_probe
"#
);

/// Link-only anchor which extracts the production exception object from the
/// BSP archive. The probe entry never calls this function.
#[cfg(target_os = "none")]
#[no_mangle]
#[link_section = ".exception_probe_anchor"]
pub extern "C" fn tms570_exception_link_anchor() {
    bsw_bsp_tms570::link_exception_capture();
}

#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(not(target_os = "none"))]
fn main() {}
