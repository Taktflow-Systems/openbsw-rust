# Resource baseline - 2026-07-17

Run `tools/port/measure_resources.ps1 -Check` to rebuild both representative
firmware images with stack-size records, measure host type footprints and
microbenchmarks, and enforce `resource-limits.json`.

The initial measurements are:

| Target/example | Flash (`text + data`) | BSS | Largest recorded function frame |
|---|---:|---:|---:|
| STM32F413 `bsw_stack_f413` | 9,496 B | 4 B | 2,288 B |
| STM32G474 `bsw_stack_g474` | 16,012 B | 4 B | 2,096 B |

Stack values come from compiler-emitted `.stack_sizes` records and represent
individual function frames, not a whole-call-chain proof. Interrupt nesting,
library assembly, and dynamic call chains still require the later target stack
budget analysis before release.

Worst-case buffer measurements are actual host `size_of` values for the
representative `CanFrame`, `MemoryQueue<1024,64>`, `FixedVec<u8,4096>`,
`ObjectPool<[u8;64],16>`, `BoundedString<128>`, and `TimerQueue<32>` types. The
generated JSON in `target/resource-baseline.json` records exact current values.

Host baselines process 16 MiB through CRC-32 and 1,024,000 fixed-vector
push/pop operations. CI uses deliberately conservative floors, so it detects
large regressions without treating shared-runner timing noise as a failure.
