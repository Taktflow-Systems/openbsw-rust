# Non-certification safety-mechanism report

This report closes H01-H09 against pinned openBSW commit
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`. The implementation is a
non-certified diagnostic support layer. It is not a safety case, ASIL claim,
FMEDA, or proof of freedom from interference.

| Package | Implemented mechanism | Verification and limit |
|---|---|---|
| H01 | heap-free trigger, value, sequence, register, and watchdog monitors | latch/reset/repeated-event tests in `bsw-safety::monitor` |
| H02 | injected severity policy, event log, limp-home and reset sinks | policy/sink fakes prove routing; no hard-coded product reaction |
| H03 | bounded watchdog startup and reset-safe fast-test state machine | versioned/CRC attempts survive reboot, one target reset proves the test, attempts are bounded, and the inherited IWDG is adopted after reboot |
| H04 | versioned CRC retained normal-reset and HardFault events handed to journal block `0x5afe` | corrupt/version records are rejected; HardFault requests a system reset; retained state clears only after durable handoff |
| H05 | incremental CRC over linker-bounded executable ROM with release-image expected CRC | independent finalization updates the dedicated four-byte section; linker assertions and physical positive/corruption paths verify the exact artifact |
| H06 | fixed MPU regions plus receive-ISR pre/post hooks | linker assertions and both per-MCU negative executable-ROM writes produce retained faults; storage remains a higher-priority RW/XN overlay |
| H07 | per-MCU ECC capability observation routed through the safety task | F413 reports unsupported; G474 routes FLASH_ECCR observation only, with no RAM ECC or safe injection claim |
| H08 | production-composed safety task, supervisor, policy and sink | monitor, retained-event, ROM, watchdog and reaction scenarios pass in both production board applications and the physical H01-H08 campaign |
| H09 | claims and residual work | this document and the final evidence matrix |

The hard-fault handler writes only a fixed `.noinit` record and requests reset.
Startup validates its version and CRC before storage handoff. Raw MMIO remains
inside reviewed BSP modules. The release campaign executes a negative
executable-ROM write on each MCU and requires its retained HardFault to survive
reset and complete durable handoff.

## Assumptions and reactions

The mechanisms assume the linker description matches the selected MCU, the
clock and watchdog declarations are correct for that board, retained RAM is not
cleared except by the fixture's explicit isolation step, flash operations stay
inside the reserved journal region, and callers preserve the reviewed MMIO and
single-owner queue contracts. A monitor warning is logged, a degraded event
routes to the injected limp-home hook, and a critical event routes to retained
logging plus the injected reset request. The product integrator owns the final
limp-home behavior and reset policy.

## Diagnostic-coverage limits

No quantitative diagnostic-coverage percentage is claimed. The ROM CRC detects
latent changes in the linked executable region but is neither authenticity nor
secure-boot verification. The watchdog proves bounded startup/reset behavior,
not every possible task deadline. MPU evidence covers the declared executable
ROM/storage/no-init layout and installed ISR hooks; it is not a general
freedom-from-interference proof. F413 exposes no claimed ECC diagnostic, while
G474 observes only supported flash-ECC status and does not inject ECC faults.

## Unsafe boundaries

All production unsafe sites are listed with source fingerprints, invariants,
caller obligations, target scope, approval status, and verification in
`unsafe-inventory.json`. The approval ledger groups those site reviews by the
owning boundary: hardware/MMIO, BSP concurrency, containers, allocator, SPSC,
I/O concurrency, and POSIX CAN FFI. The release gates fail if the inventory or
an approval becomes stale.

Residual production work is product-specific: safety goals, timing/FMEA
analysis, independent review, proof-test intervals, production watchdog window,
fault-injection authority, linker-map qualification, and certification evidence.
Unsupported ECC diagnostics remain explicit and must not be inferred.

## Optional LAUNCHXL2-570LC43 capability status

No production TMS570 safety-mechanism coverage is claimed yet. The operator-confirmed
carrier is LAUNCHXL2-570LC43; only the public board model is retained. The
operator also confirmed a bare board, permitting one controlled reset and two
RAM-only executions of a 12-byte register marker. Both clean-start runs reached
the expected marker and self-loop, but this is debugger/execution evidence, not
a safety-mechanism verification.
The subsequent RAM-only clock role also passed twice: all three required PLL
observations agreed, DCC accepted 300 MHz, and the 300/150/75-MHz clock plan was
read back. Its five-attempt fail-closed policy is host-tested, but two normal
runs do not establish diagnostic coverage or prove the rare advisory failure
path on silicon.
The subsequent 224-byte SRAM role passed twice after the clock role: it
verified privileged IRQ/FIQ mask and exact entry-mask restoration, configured
only RTI counter 0 for one-microsecond ticks, and crossed a bounded 100-ms
firmware threshold. This establishes basic critical-section and monotonic-timer
operation, not interrupt latency, safety diagnostic coverage, or watchdog
behavior.
The subsequent 1,152-byte SRAM VIM role passed twice. It physically proves
one RTI channel-2 IRQ and one FIQ, separate banked stacks, hardware index 3,
reviewed context save/restore, barriers, and architectural exception returns.
Its DEVICE#56 path acknowledges only Group-2 channel 2 in live and shadow ESM
status and verifies unrelated status words are unchanged. This is interrupt
dispatch evidence, not a production ESM handler, diagnostic-coverage claim,
fault-injection result, latency bound, or proof of every VIM channel.
The subsequent 1,836-byte retained-exception role passed eight controlled
reset runs: two each for undefined, SVC, prefetch abort, and data abort. It
physically proves banked entry/context capture, A32 LR correction, applicable
fault address, integrity-protected commit, and debugger classification of the
byte-identical record at reset vector zero. The fixed-vector PC redirection is
explicitly flagged in the record. This is bounded diagnostic-support evidence,
not a diagnostic-coverage metric, imprecise-abort recovery claim, ECC fault
injection, production vector/startup execution, or application recovery proof.
Compile-time and host-tested models cover its exact 16 MHz oscillator,
MII/DP83630 wiring,
PHY address and quiescent power-down state plus the exact main/data-flash
geometry, implemented SRAM/ECC aliases, mandatory SRAM-before-stack startup
order, all required banked-mode
stack reservations, bounded MDIO selection, and CPPI descriptor ownership.
An A32 inspection ELF additionally proves the compile/link ordering of reset,
bounded SRAM/VIM ECC auto-initialization, warm-reset record preservation plus
bank-7 ECC regeneration, all banked SP values, hard-float
enable, `.data`/`.bss`, VBAR, VIM fallback, and 128 default vectors. That
flash-mapped inspection ELF has not run on the connected hardware and does not
prove full ESM handling, MPU/cache, watchdog, direct production-vector/startup
fault handling, flash programming, DCAN, EMAC, PHY, or physical recovery. RTI
counter 0 and VIM channel 2 are now
proven only to the limited extents described above.

The revision-B safety baseline must incorporate SPNZ232B CORTEX-R5#7,
DEVICE#40, DEVICE#54, DEVICE#56, DEVICE#60, GCM#58-60, L2FMC#5 and
SSWF021#45. In particular, CPPI access is byte-swapped in software and PLL
selection must be preceded by bounded slip checks and DCC frequency
measurement. The final report remains a non-certification diagnostic-support
record and cannot make ASIL, diagnostic-coverage, or freedom-from-interference
claims.
