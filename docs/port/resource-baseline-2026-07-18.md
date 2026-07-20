# Resource baseline - 2026-07-19

`tools/port/measure_resources.ps1 -Check` rebuilt the final two production
firmware images with compiler stack-size records and enforced
`resource-limits.json` after G13 and H01-H09.

| Target/example | Flash (`text + data`) | BSS | Largest recorded function frame |
|---|---:|---:|---:|
| STM32F413 `app_f413` | 25,036 B | 12 B | 2,808 B |
| STM32G474 `app_g474` | 25,852 B | 12 B | 2,488 B |

| Target | Task path + indirect reserve | ISR increment | Combined MSP requirement | Enforced maximum |
|---|---:|---:|---:|---:|
| STM32F413 | 6,696 B | 248 B | 6,944 B | 7,424 B |
| STM32G474 | 6,040 B | 248 B | 6,288 B | 7,424 B |

Including 1,024 bytes of linker-reserved `.noinit` state, total required RAM is
7,980 bytes on F413 and 7,324 bytes on G474; both are enforced below the 8,192
byte release ceiling. Static `.bss` is 12 bytes on each target.

The representative fixed-footprint protocol values are:

| Type | Host `size_of` |
|---|---:|
| transport pool, 4 x 4,095-byte messages | 16,464 B |
| DoCAN TX session, 4,095-byte payload | 4,192 B |
| DoCAN RX session, 4,095-byte payload | 4,192 B |
| UDS pool, 4 x request/response 4,095 bytes | 32,936 B |
| middleware message, 256 inline bytes | 296 B |
| DoIP discovery entity | 80 B |
| STM32 CAN interrupt queue, 32 slots | 664 B |
| STM32 cycle accumulator | 24 B |
| STM32 CAN health/recovery state | 40 B |
| shared reference core | 80 B |
| STM32 diagnostic static buffers | 512 B |
| dynamic client registry (8 x 4) | 256 B |

The release configuration also enforces the declared production queue budgets:
F413/G474 CAN receive capacities are 32/16 frames, console TX is 512 bytes,
diagnostic payload is 256 bytes, and diagnostic response is 128 bytes. Timing
contracts are checked from the same production constants: 1 ms reference
cycle, 1 s heartbeat, at most 1.1 s CAN bus-off recovery, 100 ms UDS response,
1 s watchdog test reset, 1.5 s watchdog test deadline, and a ten-minute soak.

The call-graph analyzer adds a 1,024-byte indirect-call reserve and a 104-byte
exception frame, then combines the longest task path with ISR nesting. Cycles
are rejected. This is a deterministic static budget, not a measurement of every
possible compiler or silicon effect. Production configurations must keep pool
counts and payload capacities within the checked limits.

## Optional TMS570 foundation (not a production budget)

The compile-only LC4357 BE32 hard-float link probe contains 28 bytes of text
and no data or BSS. It exists only to prove target, linker, entry address and
instruction endianness; its size is not a firmware estimate.

Feature `launchxl2-570lc43` adds a const-only board description for the
oscillator, MII/DP83630 path, PHY control lines, and connector. It allocates no
runtime storage in the current inspection probes.

The flash-mapped startup inspection probe contains 2,388 bytes of flash text,
an 8-byte `.noinit` marker, a separate 160-byte aligned retained-exception
record, and an 8,704-byte linker-enforced stack reservation. The latter
allocates non-overlapping, eight-byte-aligned SYS,
SVC, IRQ, FIQ, abort and undefined-mode stacks. These are inspection values,
not an accepted production stack budget or a loadable image.

The physical SRAM-marker role is exactly 12 bytes in a linker-enforced
256-byte window at the top of implemented SRAM. It contains no data, BSS,
stack reservation, packet storage, or MMIO access. Two agreeing clean-start
runs proved this exact footprint on the board, but it is not a production RAM
or stack budget.

The RAM-only clock probe is 880 bytes in a linker-enforced 4-KiB SRAM window
immediately below the marker window. It has no data, BSS, stack, heap, packet
buffer, or nonvolatile section. Its bounded DCC and PLL polling loops passed in
two controlled-reset runs, but this remains bring-up code rather than the
production clock/RTI resource budget.

The staged RTI/critical-section probe is 224 bytes with no data, BSS, stack,
heap, packet buffer, or nonvolatile section. Its linker-enforced 256-byte SRAM
window is `[0x08076000, 0x08076100)`, disjoint from the clock and marker roles.
Two current-artifact runs passed, but this bring-up footprint is not an
accepted production timer or interrupt-stack budget.

The staged VIM role has 1,152 bytes of text and no data or BSS in a
linker-enforced 3-KiB code window at `[0x08075000, 0x08075c00)`. It reserves 56
bytes of private observation state plus separate 256-byte IRQ and FIQ stacks;
all ranges are disjoint from the RTI, clock, and marker roles. Two
controlled-reset runs passed, but these bring-up reservations are not accepted
production ISR-stack or interrupt-latency budgets.

The retained-exception role has 1,836 bytes of text and a 160-byte NOBITS
record in linker-enforced SRAM windows. It reserves 256 bytes for Undefined,
512 bytes for Abort, 512 bytes for Supervisor, and 512 bytes for System stack
space. The record and stack ranges are disjoint from its code and from the VIM,
RTI, clock, and marker roles. Eight controlled-reset classifications passed,
but this bring-up footprint is not an accepted production reset-retention,
stack-depth, or latency budget.

The current model also enforces a 16-byte, 16-byte-aligned CPPI descriptor and
at most 512 descriptors in the 8 KiB internal CPPI RAM. Physical high-water
measurements, ISR nesting,
EMAC buffers, lwIP pools, TCP/UDP sockets, DoIP messages, DoCAN queues and the
shared application are not yet available and remain pending T28/T32.
