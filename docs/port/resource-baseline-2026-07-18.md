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
