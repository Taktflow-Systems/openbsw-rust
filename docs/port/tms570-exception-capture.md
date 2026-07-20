# TMS570 retained exception capture

This note fixes the T13 architectural contract for the exact
TMS570LC4357 revision-B target. It does not authorize flash programming or
claim that the compile-only flash startup image has executed on hardware.

## Primary requirements

The implementation uses these primary sources:

- Arm Architecture Reference Manual ARMv7-A and ARMv7-R edition, DDI0406C.d,
  sections B1.8 and B5.5: exception entry copies CPSR to the banked SPSR;
  A32 undefined and prefetch-abort fault PCs are `LR-4`, the SVC instruction
  is `LR-4` with the next instruction at `LR`, and the data-abort PC is
  `LR-8`. DFSR/DFAR describe data aborts, IFSR/IFAR describe prefetch aborts,
  and DFAR is not valid for asynchronous external/parity aborts.
- TMS570LC43x Technical Reference Manual SPNU563A, sections 2.3 and 2.5.1.45:
  prefetch aborts are precise; data aborts can be precise or imprecise;
  illegal/protected/timeout/parity/ECC accesses can generate aborts; and
  `SYS1.SYSECR.RESET1` requests a global software reset.
- TMS570LC4357 data sheet SPNS195C, section 6.11: L2 SRAM ECC is enabled by
  default, hardware auto-initialization establishes valid zero codewords, and
  CPU writes generate corresponding ECC.
- TMS570LC4x revision-B errata SPNZ232B: DEVICE#40 means writes to some
  unimplemented peripheral locations need not abort, so T13 injects only reads
  from an unimplemented gap; AHB_ACCESS_PORT#3 forbids relying on debugger
  system-view reads while reset is asserted; DEVICE#31 precludes debugger
  access to the SRAM ECC alias; and DEVICE#48 rules out a CPU-only reset for
  this evidence path.
- TI application report SPNA236: Hercules exception vectors are fixed at low
  flash in the configuration used here. The RAM probe therefore catches the
  fixed vector after architectural entry and redirects only PC to the reviewed
  RAM handler. Every such record carries a debug-redirection provenance flag.

The CP15 encodings captured by the assembly entry are DFSR
`p15,0,c5,c0,0`, IFSR `p15,0,c5,c0,1`, ADFSR `p15,0,c5,c1,0`, AIFSR
`p15,0,c5,c1,1`, DFAR `p15,0,c6,c0,0`, and IFAR `p15,0,c6,c0,2`.

## Record and publication contract

`RetainedExceptionRecord` is 160 bytes, aligned to 32 bytes, and contains
magic/format, state and inverse, generation and inverse, class and inverse,
entry CPSR, source SPSR, corrected PCs, banked exception and source SP/LR,
r0-r12, all six fault registers, flags, a CRC and inverse, and a commit word
and inverse. CRC-32/BZIP2 is evaluated over the first 144 target-order bytes.
The commit pair is written last. The target entry then cleans exactly the five
aligned record cache lines to the point of coherency and executes `dsb sy`
before requesting reset; it does not enable or reconfigure a cache.

Only an integrity-valid Armed record may transition to Capturing and then
Complete. Empty, corrupt, stale-generation, already-Capturing, poisoned,
unknown-class, Thumb/Jazelle, little-endian, wrong-mode, or inconsistent-PC
records are rejected. A target entry that sees corrupt, recursive, or unknown
state publishes a reason-tagged Poisoned record and resets; it never attempts
to resume the faulting instruction. Safe APIs arm and classify the private
linker symbol without exposing a raw record or MMIO pointer.

The compile-only production linker reserves the first 160 bytes of SRAM bank
7 at `0x08070000`. On power-on reset startup auto-initializes all eight SRAM
banks. On a warm reset it auto-initializes banks 0-6, preserves only that
record, and writes aligned 64-bit zeroes from the record end through the end of
bank 7 before any stack use. Linker assertions fix the record size/alignment,
separate it from `.data`, `.bss`, `.noinit`, and all mode stacks, and ensure the
software ECC-generation loop begins on an eight-byte boundary.

## Physical evidence boundary

The RAM role occupies `[0x08073000,0x0807372c)`, with a private 160-byte record
at `0x08074000` and separate Undefined, Abort, Supervisor, and System stacks.
It uses `UDF`, `SVC`, an instruction fetch from `0x04000000`, and a data read
from `0x04000000`; the injection blocks contain no store. The shared handler's
only MMIO write is the documented full software-reset request.

Two controlled-reset runs of each class passed. In every run the debugger
caught the resident fixed vector, checked the architectural banked mode,
redirected PC to the corresponding production entry, stopped at the reset
request, validated the committed record, then stopped at reset vector zero and
validated the same 160 bytes again. The eight post-reset classifications agree
on class, generation, BE32/A32 state, banked stack/source context, r0-r12,
corrected LR/PC values, applicable fault address, CRC/inverses, and commit.

This proves the shared handler and a retained SRAM handoff across controlled
system reset under debugger observation. It does not prove execution of the
flash-mapped startup, direct production-vector ownership, an application
consumer after startup, imprecise-abort recovery, ECC fault injection,
MPU/cache/ESM policy, watchdog behavior, or any Ethernet/DoIP capability.
Raw logs, record words, run selectors, maps, binaries, and hashes remain in
ignored private evidence.
