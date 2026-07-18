# Resource baseline - 2026-07-18

`tools/port/measure_resources.ps1 -Check` rebuilt the two representative
firmware images with compiler stack-size records and enforced
`resource-limits.json` after packages G01-G11.

| Target/example | Flash (`text + data`) | BSS | Largest recorded function frame |
|---|---:|---:|---:|
| STM32F413 `bsw_stack_f413` | 10,024 B | 4 B | 1,696 B |
| STM32G474 `bsw_stack_g474` | 10,476 B | 4 B | 1,376 B |

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

Stack records represent individual function frames, not complete call-chain or
interrupt-nesting proofs. Those whole-system budgets remain assigned to G13.
The protocol sizes are explicit configuration costs: production configurations
must select pool count and payload capacity within target RAM limits.
