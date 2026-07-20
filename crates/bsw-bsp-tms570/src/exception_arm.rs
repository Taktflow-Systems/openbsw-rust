//! Terminal A32 exception entries for the exact LC4357 revision-B target.
//!
//! # Invariants
//!
//! - Undefined, SVC, prefetch-abort, and data-abort vectors enter distinct
//!   banked modes with initialized stacks before these symbols can run.
//! - Every source r0-r12 and the banked exception LR is stacked before a
//!   general register or `bl` can destroy it.
//! - SPSR holds the source CPSR; CPSR holds the handler mode and BE32 state.
//! - User/System banked SP/LR are copied with the privileged `^` transfer only
//!   when SPSR identifies that shared bank.
//! - DFSR/DFAR/IFSR/IFAR and both implementation-defined auxiliary status
//!   registers are read from their ARMv7-R PMSA CP15 encodings.
//! - The retained record is accepted only from a CRC-checked Armed state,
//!   marked Capturing before context publication, CRC-sealed, and committed
//!   last. Its five 32-byte lines are cleaned to the point of coherency before
//!   `dsb sy`, so a later write-back cache policy cannot lose it at reset.
//! - Capture is terminal. A committed record is followed by a software system
//!   reset request; there is no attempt to resume a faulting instruction.
//! - The only MMIO write is the documented SYS1.SYSECR software-reset key.

core::arch::global_asm!(
    r#"
    .syntax unified
    .arch armv7-r
    .arm

    .equ RECORD_MAGIC,             0x544d5346
    .equ RECORD_FORMAT,            0x00010028
    .equ STATE_ARMED,              0x41524d44
    .equ STATE_CAPTURING,          0x43415054
    .equ STATE_COMPLETE,           0x434f4d50
    .equ STATE_POISONED,           0x504f4953
    .equ COMMIT_ARMED,             0x41524d43
    .equ COMMIT_COMPLETE,          0x434f4d54
    .equ COMMIT_POISONED,          0x504f434d
    .equ FLAG_SOURCE_BANK_VALID,   0x00000001
    .equ FLAG_DFAR_VALID,          0x00000002
    .equ FLAG_IFAR_VALID,          0x00000004
    .equ FLAG_DEBUG_REDIRECT,      0x00000008
    .equ FLAG_POISON_CORRUPT,      0x00010000
    .equ FLAG_POISON_RECURSIVE,    0x00020000
    .equ FLAG_POISON_UNKNOWN,      0x00040000

    .equ REC_MAGIC,                0
    .equ REC_FORMAT,               4
    .equ REC_STATE,                8
    .equ REC_STATE_INV,            12
    .equ REC_GENERATION,           16
    .equ REC_GENERATION_INV,       20
    .equ REC_CLASS,                24
    .equ REC_CLASS_INV,            28
    .equ REC_ENTRY_CPSR,           32
    .equ REC_SOURCE_SPSR,          36
    .equ REC_EXCEPTION_LR,         40
    .equ REC_EXCEPTION_PC,         44
    .equ REC_RESUME_PC,            48
    .equ REC_EXCEPTION_SP,         52
    .equ REC_SOURCE_SP,            56
    .equ REC_SOURCE_LR,            60
    .equ REC_REGISTERS,            64
    .equ REC_DFSR,                 116
    .equ REC_DFAR,                 120
    .equ REC_IFSR,                 124
    .equ REC_IFAR,                 128
    .equ REC_ADFSR,                132
    .equ REC_AIFSR,                136
    .equ REC_FLAGS,                140
    .equ REC_CRC,                  144
    .equ REC_CRC_INV,              148
    .equ REC_COMMIT,               152
    .equ REC_COMMIT_INV,           156
    .equ REC_CRC_BYTES,            144
    .equ REC_BYTES,                160

    .section .exceptions.tms570, "ax", %progbits
    .balign 4

    .macro TERMINAL_ENTRY symbol, class
    .global \symbol
    .type \symbol, %function
\symbol:
    /* A masks asynchronous aborts; I/F remain closed during publication. */
    cpsid   aif
    stmdb   sp!, {{r0-r12, lr}}
    mov     r0, #\class
    mov     r1, sp
    b       _tms570_exception_capture_common
    .size \symbol, . - \symbol
    .endm

    TERMINAL_ENTRY _tms570_undefined, 1
    TERMINAL_ENTRY _tms570_svc, 2
    TERMINAL_ENTRY _tms570_prefetch_abort, 3
    TERMINAL_ENTRY _tms570_data_abort, 4
    TERMINAL_ENTRY _tms570_unknown_exception, 0

    .global _tms570_exception_capture_common
    .type _tms570_exception_capture_common, %function
_tms570_exception_capture_common:
    /* Preserve handler inputs in callee-saved registers before CRC calls. */
    mov     r9, r0                  /* raw class */
    mov     r10, r1                 /* entry frame */
    mrs     r11, cpsr
    mrs     r7, spsr
    add     r6, r10, #56            /* banked SP before entry frame */
    ldr     r5, [r10, #52]          /* banked exception LR */
    ldr     r8, =__tms570_exception_record

    /* A recursive entry observes Capturing before any existing field changes. */
    ldr     r0, [r8, #REC_STATE]
    ldr     r1, =STATE_CAPTURING
    cmp     r0, r1
    beq     .Lpoison_recursive

    /* Validate the complete Armed envelope and every inverse field. */
    ldr     r1, [r8, #REC_MAGIC]
    ldr     r2, =RECORD_MAGIC
    cmp     r1, r2
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_FORMAT]
    ldr     r2, =RECORD_FORMAT
    cmp     r1, r2
    bne     .Lpoison_corrupt
    ldr     r1, =STATE_ARMED
    cmp     r0, r1
    bne     .Lpoison_corrupt
    ldr     r2, [r8, #REC_STATE_INV]
    mvn     r2, r2
    cmp     r0, r2
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_GENERATION]
    ldr     r2, [r8, #REC_GENERATION_INV]
    mvn     r2, r2
    cmp     r1, r2
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_CLASS]
    ldr     r2, [r8, #REC_CLASS_INV]
    mvn     r2, r2
    cmp     r1, r2
    bne     .Lpoison_corrupt
    cmp     r1, #0
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_FLAGS]
    bic     r1, r1, #FLAG_DEBUG_REDIRECT
    cmp     r1, #0
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_CRC]
    ldr     r2, [r8, #REC_CRC_INV]
    mvn     r2, r2
    cmp     r1, r2
    bne     .Lpoison_corrupt
    ldr     r1, [r8, #REC_COMMIT]
    ldr     r2, =COMMIT_ARMED
    cmp     r1, r2
    bne     .Lpoison_corrupt
    ldr     r2, [r8, #REC_COMMIT_INV]
    mvn     r2, r2
    cmp     r1, r2
    bne     .Lpoison_corrupt
    mov     r0, r8
    mov     r1, #REC_CRC_BYTES
    bl      _tms570_exception_crc32
    ldr     r1, [r8, #REC_CRC]
    cmp     r0, r1
    bne     .Lpoison_corrupt

    /* Unknown/reserved vectors are poisoned rather than classified. */
    cmp     r9, #1
    blo     .Lpoison_unknown
    cmp     r9, #4
    bhi     .Lpoison_unknown

    /* Publish the in-progress state before touching context fields. */
    ldr     r0, =STATE_CAPTURING
    str     r0, [r8, #REC_STATE]
    mvn     r0, r0
    str     r0, [r8, #REC_STATE_INV]
    mov     r0, #0
    str     r0, [r8, #REC_COMMIT]
    str     r0, [r8, #REC_COMMIT_INV]
    dsb     sy

    str     r9, [r8, #REC_CLASS]
    mvn     r0, r9
    str     r0, [r8, #REC_CLASS_INV]
    str     r11, [r8, #REC_ENTRY_CPSR]
    str     r7, [r8, #REC_SOURCE_SPSR]
    str     r5, [r8, #REC_EXCEPTION_LR]
    str     r6, [r8, #REC_EXCEPTION_SP]

    /* A32 architectural LR corrections: UND/PABT -4, SVC next LR, DABT -8. */
    cmp     r9, #4
    subeq   r0, r5, #8
    subne   r0, r5, #4
    str     r0, [r8, #REC_EXCEPTION_PC]
    cmp     r9, #2
    moveq   r0, r5
    str     r0, [r8, #REC_RESUME_PC]

    /* Preserve the physical role's debug-assisted provenance bit only. */
    ldr     r4, [r8, #REC_FLAGS]
    and     r4, r4, #FLAG_DEBUG_REDIRECT

    /* The ^ form selects the User/System SP and LR banks without mode switch. */
    and     r0, r7, #0x1f
    cmp     r0, #0x10
    beq     .Lsource_bank_valid
    cmp     r0, #0x1f
    bne     .Lsource_bank_invalid
.Lsource_bank_valid:
    add     r0, r8, #REC_SOURCE_SP
    stmia   r0, {{sp, lr}}^
    orr     r4, r4, #FLAG_SOURCE_BANK_VALID
    b       .Lsource_bank_done
.Lsource_bank_invalid:
    mov     r0, #0
    str     r0, [r8, #REC_SOURCE_SP]
    str     r0, [r8, #REC_SOURCE_LR]
.Lsource_bank_done:

    /* ARMv7-R PMSA fault-status and address registers. */
    mrc     p15, 0, r0, c5, c0, 0
    str     r0, [r8, #REC_DFSR]
    mrc     p15, 0, r1, c6, c0, 0
    str     r1, [r8, #REC_DFAR]
    mrc     p15, 0, r1, c5, c0, 1
    str     r1, [r8, #REC_IFSR]
    mrc     p15, 0, r1, c6, c0, 2
    str     r1, [r8, #REC_IFAR]
    mrc     p15, 0, r1, c5, c1, 0
    str     r1, [r8, #REC_ADFSR]
    mrc     p15, 0, r1, c5, c1, 1
    str     r1, [r8, #REC_AIFSR]

    cmp     r9, #3
    orreq   r4, r4, #FLAG_IFAR_VALID
    cmp     r9, #4
    bne     .Lfault_flags_done
    and     r1, r0, #0x0f
    tst     r0, #0x0400
    orrne   r1, r1, #0x10
    cmp     r1, #0x16              /* asynchronous external abort */
    beq     .Lfault_flags_done
    cmp     r1, #0x18              /* asynchronous parity error */
    orrne   r4, r4, #FLAG_DFAR_VALID
.Lfault_flags_done:
    str     r4, [r8, #REC_FLAGS]

    /* Copy the source r0-r12 frame after all working values are retained. */
    add     lr, r8, #REC_REGISTERS
    ldmia   r10, {{r0-r12}}
    stmia   lr, {{r0-r12}}
    ldr     r8, =__tms570_exception_record

    ldr     r0, =STATE_COMPLETE
    str     r0, [r8, #REC_STATE]
    mvn     r0, r0
    str     r0, [r8, #REC_STATE_INV]
    mov     r0, r8
    mov     r1, #REC_CRC_BYTES
    bl      _tms570_exception_crc32
    str     r0, [r8, #REC_CRC]
    mvn     r1, r0
    str     r1, [r8, #REC_CRC_INV]
    dsb     sy
    ldr     r0, =COMMIT_COMPLETE
    str     r0, [r8, #REC_COMMIT]
    mvn     r0, r0
    str     r0, [r8, #REC_COMMIT_INV]
    dsb     sy
    bl      _tms570_exception_clean_record
    b       _tms570_exception_request_reset

.Lpoison_recursive:
    ldr     r9, =FLAG_POISON_RECURSIVE
    b       .Lpoison
.Lpoison_corrupt:
    ldr     r9, =FLAG_POISON_CORRUPT
    b       .Lpoison
.Lpoison_unknown:
    ldr     r9, =FLAG_POISON_UNKNOWN

.Lpoison:
    /* Preserve generation only when its inverse still agrees. */
    ldr     r4, [r8, #REC_GENERATION]
    ldr     r0, [r8, #REC_GENERATION_INV]
    mvn     r0, r0
    cmp     r4, r0
    moveq   r5, r4
    movne   r5, #0
    mov     r0, #0
    mov     r1, #40
    mov     r2, r8
.Lpoison_zero:
    str     r0, [r2], #4
    subs    r1, r1, #1
    bne     .Lpoison_zero
    ldr     r0, =RECORD_MAGIC
    str     r0, [r8, #REC_MAGIC]
    ldr     r0, =RECORD_FORMAT
    str     r0, [r8, #REC_FORMAT]
    ldr     r0, =STATE_POISONED
    str     r0, [r8, #REC_STATE]
    mvn     r0, r0
    str     r0, [r8, #REC_STATE_INV]
    str     r5, [r8, #REC_GENERATION]
    mvn     r0, r5
    str     r0, [r8, #REC_GENERATION_INV]
    mov     r0, #0
    str     r0, [r8, #REC_CLASS]
    mvn     r0, r0
    str     r0, [r8, #REC_CLASS_INV]
    str     r9, [r8, #REC_FLAGS]
    mov     r0, r8
    mov     r1, #REC_CRC_BYTES
    bl      _tms570_exception_crc32
    str     r0, [r8, #REC_CRC]
    mvn     r1, r0
    str     r1, [r8, #REC_CRC_INV]
    dsb     sy
    ldr     r0, =COMMIT_POISONED
    str     r0, [r8, #REC_COMMIT]
    mvn     r0, r0
    str     r0, [r8, #REC_COMMIT_INV]
    dsb     sy
    bl      _tms570_exception_clean_record
    b       _tms570_exception_request_reset
    .size _tms570_exception_capture_common, . - _tms570_exception_capture_common

    /* The aligned 160-byte record occupies exactly five 32-byte cache lines. */
    .global _tms570_exception_clean_record
    .type _tms570_exception_clean_record, %function
_tms570_exception_clean_record:
    ldr     r0, =__tms570_exception_record
    add     r1, r0, #REC_BYTES
.Lclean_record_line:
    mcr     p15, 0, r0, c7, c10, 1
    add     r0, r0, #32
    cmp     r0, r1
    blo     .Lclean_record_line
    dsb     sy
    bx      lr
    .size _tms570_exception_clean_record, . - _tms570_exception_clean_record

    /* CRC-32/BZIP2 over target-order bytes: non-reflected 04c11db7. */
    .global _tms570_exception_crc32
    .type _tms570_exception_crc32, %function
_tms570_exception_crc32:
    mvn     r2, #0
    ldr     r12, =0x04c11db7
.Lcrc_byte:
    ldrb    r3, [r0], #1
    eor     r2, r2, r3, lsl #24
    mov     r3, #8
.Lcrc_bit:
    movs    r2, r2, lsl #1
    eorcs   r2, r2, r12
    subs    r3, r3, #1
    bne     .Lcrc_bit
    subs    r1, r1, #1
    bne     .Lcrc_byte
    mvn     r0, r2
    bx      lr
    .size _tms570_exception_crc32, . - _tms570_exception_crc32

    .global _tms570_exception_request_reset
    .global _tms570_exception_reset_failed
    .type _tms570_exception_request_reset, %function
_tms570_exception_request_reset:
    /* SYS1.SYSECR RESET1=1 requests a documented software system reset. */
    movw    r0, #0xffe0
    movt    r0, #0xffff
    movw    r1, #0x8000
    str     r1, [r0]
    dsb     sy
    isb
    movw    r12, #0x4652
    movt    r12, #0x4553
_tms570_exception_reset_failed:
    b       _tms570_exception_reset_failed
    .size _tms570_exception_request_reset, . - _tms570_exception_request_reset
"#
);

/// Keep the target exception object in startup and RAM-role inspection ELFs.
///
/// Calling this function only creates a link dependency. It performs no MMIO
/// and does not arm the retained state machine.
#[inline(never)]
pub extern "C" fn link_exception_capture() {}
