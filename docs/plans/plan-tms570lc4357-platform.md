# Optional TMS570LC4357 platform and physical DoIP plan

This plan is an optional expansion of the completed mandatory Rust port. It
does not change the 1,092/1,092 mandatory package record. T01-T13 preserved the
37/37 release parity checkpoint; the independently governed oracle re-pin on
`origin/main` subsequently added the current 38th mandatory row, as recorded
in `plan-rust-bsw-full-integration.md`.

## Proven baseline and safety boundary

Non-destructive discovery found one XDS110-class debug path and one six-bit
ICEPick-C JTAG path. A TAP-only scan returned the public LC4357 revision-B JTAG
identification code `0x1b95a02f`, which matches SPNS195C table 6-52 exactly.
During identity discovery no CPU session was opened and no target state was
changed. After the operator confirmed that the connected board was bare, a
separate controlled RAM-only bring-up opened a CPU session, reset the target,
loaded a 12-byte register-only marker at `0x08078000`, and ran it twice. No
flash erase, unlock, program, option, OTP, EEPROM, or security operation was
performed.

The starting HEAD, local `origin/main`, and live remote `origin/main` agreed.
The final read-only remote check found that the live remote had changed while
HEAD still matched the unchanged local tracking ref. The pinned baseline was
not fetched, moved, merged, or rebased.

The exact MCU is therefore proven as TMS570LC4357 silicon revision B. A
follow-up query-only pass identified the probe firmware as an embedded XDS110
in Standard configuration and correlated four USB functions in one container,
including debug and application/auxiliary serial functions.
TI's installed desktop `DeviceDetector` was then run with its serial output
suppressed. It found one XDS110 but returned no board name. Inspection of its
150-entry `board_ids.json` explains why: it contains two older Hercules
LaunchPads but no LC43 or RM57 LaunchPad entry. The private four-character
board-prefix comparison is therefore inconclusive, not evidence against the TI
kit. The current cloud UniFlash path was also checked, but it requires a myTI
login before detection; no credentials, permissions, or device session were
provided. No serial or prefix value is publishable evidence. The operator then
provided a procurement record identifying the connected carrier as TI product
`LAUNCHXL2-570LC43`. Only that public model identification is retained; invoice,
shipment, tracking, and order metadata are neither copied nor persisted.

The operator identification closes the carrier gate, and the observed embedded
XDS110, serial functions, 100 Mbit/s physical link, and MCU independently agree
with that model. SPRR397 A1 is now the board baseline. It proves a 16 MHz MCU
crystal, four-bit MII, DP83630 PHY at address 1, ECLK1T-to-XIN reference-clock
route, active-low PHY reset on GIOA[4], active-low power-down/interrupt on
GIOA[3], and direct RJ-45 magnetics. The host-side Ethernet adapter is separate
from the USB probe.
Passive counters prove that its 100 Mbit/s link partner transmits a sustained
stream of small Layer-2 broadcast frames, so the physical link is active in
both directions. The interface has no IPv4 address or reachable neighbor, and
the frame metadata does not identify the remote PHY or establish DoIP.
Passive link metadata alone still does not prove PHY ownership or DoIP, so
those remain future physical tests. The exact oscillator and Ethernet wiring
are selected only by feature `launchxl2-570lc43`; no board-local writable flash
region has been selected.

The operator confirmed that this is a bare board with no external actuator or
unsafe expansion load. The first physical image was therefore RAM-loaded. Two
clean-start runs of the same inspected 12-byte image reached its self-loop with
the expected register marker. The first pre-reset trial entered a prefetch
abort because the inherited demonstration image had left execution protection
active; a controlled target reset cleared that inherited MPU/cache state, and
both subsequent runs passed. Flash remains prohibited until the existing
application, protection state, complete image range, linker map, and explicitly
authorized writable application region are known.

## Authoritative inputs

| Input | Revision used | Role |
|---|---|---|
| TMS570LC4357 Hercules Microcontroller Based on the Arm Cortex-R Core data sheet, SPNS195 | C, June 2016 | exact memory map, flash geometry, JTAG and device IDs, clocks, terminals |
| TMS570LC43x 16/32-Bit RISC Flash Microcontroller Technical Reference Manual, SPNU563 | A, March 2018 | startup, memory initialization, VIM, RTI, DCAN, EMAC and MDIO |
| TMS570LC4x Silicon Revision B Silicon Errata, SPNZ232 | B, June 2018 | revision-B workarounds and residual exceptions |
| Arm Architecture Reference Manual, ARMv7-A and ARMv7-R edition, DDI0406 | C.d | A32 exception entry, banked SPSR/LR, LR correction and PMSA fault registers |
| Software Relocation of Exception Vectors on Hercules, SPNA236 | A, August 2016 | fixed low-vector constraint and debugger-assisted RAM qualification boundary |
| Hercules PLL Advisory SSWF021#45 Workaround, SPNA233 | B, February 2020 | bounded power-on retry sequence and DCC qualification |
| Hercules TMS570LC43x Development Kit Quick Start Guide, SPNU617 | June 2015 | selected LaunchPad setup and connector overview |
| TMS570LC43x and RM57Lx LaunchPad Schematic, SPRR397 | A1, May 2015 | exact oscillator, MII, DP83630, reset and connector wiring |
| DP83630 Precision PHYTER data sheet, SNLS335 | B, April 2013 | reference clock, address/mode straps, reset and power-down behavior |
| XDS110 Support Notes and XDS110 Debug Probe documentation | installed with CCS 20.4.1 and current TI online documentation | query-only enumeration, embedded/standalone distinction, mutable board-prefix and external-target limitations |
| Rust `armebv7r-none-eabi{,hf}` platform support | Rust 1.94 baseline | Armv7-R BE32 target and GNU `-mbe32` link requirement |
| lwIP upstream tag `STABLE-2_2_1_RELEASE` | 2.2.1 | pinned TCP/IP stack candidate; source and configuration enter the tree only with its BSD notice |

Device facts already locked down are Cortex-R5F Armv7-R, BE32 execution,
single- and double-precision VFP, 4 MiB main flash at
`[0x00000000, 0x00400000)`, 512 KiB SRAM at
`[0x08000000, 0x08080000)`, SRAM ECC alias at
`[0x08400000, 0x08480000)`, 128 KiB data flash bank 7, two lockstep VIMs,
RTI/DWWD, four DCAN instances, and one 10/100 EMAC with CPPI RAM and MDIO.
SRAM and its ECC must be initialized before Rust uses a stack or static data.

Revision-B design exceptions are requirements, not optional optimizations:

- DEVICE#54 requires byte swapping for CPU access to EMAC CPPI descriptors.
- SSWF021#45 requires PLL slip observation, DCC measurement, and up to five
  bounded startup attempts before selecting the PLL.
- CORTEX-R5#7 constrains write-back data-cache use when cache ECC is enabled.
- DEVICE#40 affects abort expectations for unimplemented peripheral writes.
- DEVICE#56 means a debugger connection can assert `nERROR`.
- DEVICE#60 changes post-reset `nERROR` recovery.
- GCM#58 through GCM#60 constrain clock ratios and reset recovery.
- L2FMC#5 constrains MPU attributes for flash ECC, OTP, and data-flash reads.

## Architecture

`bsw-bsp-tms570` owns the device register model, startup, board adapters, and
hardware drivers. Device and board modules remain separate. Exactly one proven
device/board feature is allowed for a target build; no generic `tms570` feature
or family-wide memory map is permitted.

The shared application remains in `openbsw-reference-app` and
`bsw-reference-core`. DoIP, UDS, lifecycle, storage, safety, and transport logic
remain in their existing shared crates. `bsw-ethernet` supplies opaque,
allocation-free TCP and UDP contracts. `bsw-doip::EmbeddedDoIpEntity` owns the
hardware-independent, heap-free DoIP server state. The BSP may implement the
EMAC netif and lwIP adapters, but it must not contain DoIP parsing, routing
activation, UDS dispatch, or diagnostic client policy.

The selected lwIP configuration is planned as cooperative `NO_SYS=1`: RX ISR
work is bounded to descriptor acknowledgement and queue publication; the main
network task feeds pbufs to `netif->input` and calls `sys_check_timeouts`.
All C types and callbacks stop at the platform adapter. Pool counts, pbufs,
TCP PCBs, UDP PCBs, netconns, sockets, segments, and timeouts are fixed and
measured before the configuration becomes a production claim.

## Packages

Every package is capped at eight hours. A package that cannot close inside the
cap must split before work continues. Status values describe this checkout,
not intended future capability.

| ID | Hours | Depends | Status | Package and acceptance condition |
|---|---:|---|---|---|
| T01 | 8 | mandatory release | done | Snapshot Git and perform privacy-safe USB, PnP, serial, network, TI-tool, probe, and topology discovery. Done when raw evidence is ignored and public reporting contains no unique selectors. |
| T02 | 8 | T01 | done | Run TAP-only JTAG identification. Done when the public IDCODE proves exactly one LC4357 revision-B target without a CPU halt or reset. |
| T03 | 8 | T02 | done | Review SPNS195C, SPNU563A, SPNZ232B, SPRR397 A1 and SNLS335B, then bind the operator-identified carrier. Done when proven device and board facts are separated from unsupported capabilities. |
| T04 | 8 | D20,E26-E31 | done | Complete the generic embedded UDP contract, discovery service, and heap-free TCP/UDP DoIP entity. Done when no-std, fake end-to-end UDS/DoIP, restart, formatting, and clippy checks pass. |
| T05 | 8 | T03 | done | Pin `armebv7r-none-eabihf`, a dated nightly with `rust-src` for tier-3 `core`, and the GNU BE32 linker path. Done when a minimal object and linked probe disassemble as BE32 Armv7-R hard-float. |
| T06 | 8 | T03,T05 | done | Add the LC4357 revision-B memory/register model, exact LaunchPad wiring, range checks, startup stack model, MDIO clock model, and CPPI descriptor codec. Done when host tests prove board constants, sizes, alignment, bounds, ownership, and DEVICE#54 conversion. |
| T07 | 8 | T05,T06 | done | Implement the reset vector and no-stack SRAM/ECC auto-initialization path. The flash-mapped inspection ELF proves eight A32 vectors, register-only bounded SYS/L2RAMW/VIM initialization, and no stack/SRAM access before `MINIDONE`. It remains prohibited from target loading. |
| T08 | 8 | T07 | done | Initialize banked SP for SVC, SYS, IRQ, FIQ, abort, and undefined modes and enable the R5F FPU before hard-float Rust calls. Linker symbols and A32 disassembly prove all six aligned ranges and the CP10/CP11/FPEXC transition before Rust entry. |
| T09 | 8 | T08 | done | Copy `.data`, clear `.bss`, preserve bounded `.noinit`, install VBAR/VIM fallback/default vectors, and enter Rust. Map and disassembly assertions pass. A separate no-stack, no-data, 12-byte SRAM role then proved physical debugger load, A32 execution, register observation, and a stable self-loop in two agreeing clean-start runs. This RAM role does not claim that the flash-mapped startup probe has run. |
| T10 | 8 | T03,T09 | done | Implement bounded PLL/clock bring-up including SSWF021#45, GCM#58-60, slip observation and DCC measurement. Host tests cover all three advisory failure signatures and five-attempt fail-closed behavior. Two physical RAM-only runs independently passed the workaround operating point, 300 MHz DCC window, clean slip state, and 300/150/75-MHz GCLK/HCLK/VCLK register plan on the proven board. |
| T11 | 8 | T10 | done | Implement RTI monotonic time and critical sections. A single-owner driver implements the shared clock, CAN timer, and critical-section seams; host tests prove the exact 75-to-1-MHz prescaler, wrap extension, nesting, entry-mask classification, and unbalanced-release failure. Two current-artifact RAM runs physically passed CPSR IRQ/FIQ mask/restore, counter-0-only setup, a 100,000-us firmware threshold, coherent FRC0/UC0 observation, and coarse debugger interval agreement. |
| T12 | 8 | T09,T11 | done | Implement VIM registration/dispatch, IRQ/FIQ entry, barriers, and default capture. Host tests prove the 128-channel table, reserved channels, class selection, enable/disable, duplicate rejection, index decoding, and bounded default capture. A linker- and disassembly-checked 1,152-byte SRAM role then passed two controlled-reset runs through fixed read-only flash trampolines: RTI compare 0 reached channel-2 IRQ and FIQ entries once each with hardware index 3, banked stacks and architectural returns. The role acknowledges only DEVICE#56 Group-2 channel 2 in live/shadow ESM status and proves unrelated ESM words unchanged. |
| T13 | 8 | T09,T12 | done | Implement separate undefined, SVC, prefetch-abort and data-abort entries with full r0-r12/LR frames, banked SPSR/CPSR and System SP/LR capture, exact A32 LR correction, all six PMSA fault registers, commit-last CRC protection, generation rejection, poison states, cache-line cleaning, and terminal software reset. The compile-only startup preserves one linker-fixed 160-byte bank-7 record while regenerating ECC for every other skipped word. A 1,836-byte RAM role used fixed-vector catch and PC-only redirection because flash is read-only; two runs per class (8/8) reclassified byte-identical committed records at reset vector zero after bounded UDF, SVC, unimplemented fetch, and unimplemented read injections. |
| T14 | 8 | confirmed carrier,T10 | pending | Implement the proven board's SCI console and bounded queues. Done when scripted 115200 8N1 output and overflow recovery pass. |
| T15 | 8 | confirmed carrier,T10,T12 | pending | Implement typed GPIO ownership for confirmed safe signals only. Done when a user-approved non-hazardous signal passes input/output tests. |
| T16 | 8 | T09,T11 | pending | Implement reset reason, controlled reset, DWWD and retained attempt bounding with DEVICE#56/#60 handling. Done when bounded reset and watchdog scenarios pass. |
| T17 | 8 | T09,T13 | pending | Implement MPU, cache, ESM and L2RAMW observation with CORTEX-R5#7, DEVICE#40 and L2FMC#5 constraints. Done when host policy tests and safe physical observations pass. |
| T18 | 8 | T03,T17 | pending | Implement LC4357 flash geometry and protection-state queries without erase/program support. Done when every region and sector boundary matches SPNS195C and protected/OTP regions are rejected. |
| T19 | 8 | T18,explicit flash authorization | pending | Add linker-reserved flash primitives and storage only for an authorized application region. Done when every operation rechecks range, alignment, protection, map, and power-loss recovery. |
| T20 | 8 | T10,T12 | pending | Implement DCAN register ownership, bit timing, mailboxes and local loopback. Done when register-model tests and physical internal loopback pass. |
| T21 | 8 | T20 | pending | Add DCAN interrupts, error state, bus-off recovery, DoCAN and shared UDS routing. Done when shared diagnostic state passes physical CAN scenarios. |
| T22 | 8 | confirmed carrier,T10,T12 | pending | Implement EMAC power, pinmux and interrupt routing for the proven board only. Done when no unproven pin or clock constant remains. |
| T23 | 8 | confirmed carrier,T22 | pending | Implement MDIO, exact PHY reset/discovery/configuration and bounded link observation. Done when the physical PHY ID and negotiated link are read without guessing. |
| T24 | 8 | T06,T22 | pending | Implement static CPPI rings/buffers, barriers, DEVICE#54 swapping and DMA ownership. Done when host tests, map checks, and EMAC local TX/RX pass. |
| T25 | 8 | T23,T24 | pending | Implement RX/TX recovery, bounded polling fallback, overflow counters and link down/up restart. Done when forced exhaustion and physical cable recovery pass. |
| T26 | 8 | T04,T25 | pending | Import pinned lwIP 2.2.1 with BSD notice and fixed `NO_SYS` configuration. Done when source identity, configuration, pool budgets and threading rules are reproducible. |
| T27 | 8 | T26 | pending | Implement netif, pbuf, TCP and UDP adapters without exposing lwIP types. Done when fake EMAC events and target-independent adapter conformance pass. |
| T28 | 8 | T21,T27,F02 | pending | Compose the shared production application with one UDS state for DoCAN and DoIP. Done when only declared platform adapters/capabilities differ. |
| T29 | 8 | T28 | pending | Add privacy-safe exact-artifact HIL roles for RAM boot, clock, console, VIM, reset, watchdog, PHY, EMAC, IP, UDP, TCP, DoIP and shared-state scenarios. Done when two clean smoke runs agree. |
| T30 | 8 | T29 | pending | Run the physical DoIP conformance, malformed input, fragmentation, concurrency, exhaustion, recovery and repeated-reset matrix. Done when every physical row has structured evidence. |
| T31 | 8 | T30 | pending | Run the time-bounded physical soak with explicit request/response totals and final recovery. Done when thresholds pass against the selected immutable artifact. |
| T32 | 8 | T28 | pending | Enforce flash, SRAM, mode stacks, ISR stack, CPPI, pbuf, lwIP, socket, DoIP, DoCAN and timing limits. Done when over-budget builds fail. |
| T33 | 8 | T28,T32 | pending | Reconstruct two independent clean roots and build each selected TMS570 artifact twice. Done when binaries compare byte-for-byte and private identities stay ignored. |
| T34 | 8 | T28-T33 | pending | Run the full Windows, pinned Linux, clippy, no-std, policy, dependency, unsafe, privacy, STM32, POSIX/DoIP, Miri, fuzz, cross-link, disassembly and exact-artifact regression matrix. Done when mandatory baselines remain clean. |
| T35 | 8 | T34 | pending | Publish the optional evidence index, board/build/resource/safety guides and final handoff. Done when privacy/schema validation passes and proven behavior is distinct from unsupported capability. |

The optional tranche is 280 hours. T01-T13 are complete and T14-T35 remain
pending. Physical claims are limited to the bounded SRAM marker, RAM-only
PLL/DCC clock role, staged RAM-only RTI/critical-section role, and SRAM-only
VIM channel-2 IRQ/FIQ role, plus the RAM-only retained exception role described
in `docs/port/tms570-exception-capture.md`; the flash-mapped startup image
remains an inspection artifact.

## Physical acceptance gate

Physical acceptance requires discovery and announcements; entity status and
power mode; TCP routing activation; diagnostic positive/negative
acknowledgements; UDS responses; alive checks; fragmented/coalesced input;
malformed and oversized messages; wrong addresses; concurrent clients; pool
exhaustion; reconnect; link interruption; repeated reset; shared DoCAN/DoIP
state; two agreeing clean starts; an enforced reset count; and a time-bounded
soak with explicit totals. Fake, loopback, POSIX, or host simulation results
are never labeled physical.

## Current external inputs

The carrier gate is closed as `LAUNCHXL2-570LC43`, and the privacy-safe board
model now encodes the SPRR397 A1/SNLS335B constants without any MMIO action.
A target-side PHY query was not used for identity: an MDIO transaction requires
peripheral writes, while CCS provides no documented no-halt target connection
and revision-B errata warns that debugger connection can assert `nERROR`.
The operator confirmed that the connected target is a bare board, permitting
the controlled reset, RAM-marker, and clock-probe runs. The clock role touched
only system clock, DCC1, and PLL-slip status registers and selected a measured
PLL only after all checks passed. GPIO, PWM, CAN, watchdog, and PHY control
still require signal-specific review before use. Flash remains gated
by independent proof of the existing image, protection state, linker ranges,
recoverability, and authorized writable area.
