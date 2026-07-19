# Safety and robustness verification - 2026-07-19

## Miri

Pinned nightly: `nightly-2026-03-14`.

| Selection | Result |
|---|---:|
| `bsw-estd --lib` | 287 passed |
| `bsw-util --lib spsc::tests` | 14 passed |
| `bsw-io memory_queue` plus handle conformance | 23 passed |
| `bsw-bsp-stm32 can_isr::tests` | 2 passed |
| `bsw-safety --lib` | 11 passed |

Miri found and drove repairs for three concrete alias/provenance defects:

1. intrusive lists reconstructed a container pointer from a field pointer;
2. object-pool release created an aliased `&T` while holding `&mut self`;
3. fixed-vector shifts derived source and destination pointers from
   incompatible shared/exclusive borrow provenance.

The final selected suite is clean. `tools/port/check_miri.ps1` is the CI entry.

## Controlled concurrency

`crates/bsw-util/tests/spsc_concurrency.rs` transfers 20,000 ordered values
through repeated wraparound and exercises a capacity-one full/empty handoff
with barriers. `bsw-bsp-stm32::can_isr` adds a 10,000-frame controlled
interrupt/main-loop producer-consumer model. Together they cover ordering,
overflow/full observation, empty observation, and wraparound.

## Fuzz smoke

All targets completed 100 AddressSanitizer/libFuzzer runs on the pinned Linux
nightly used by CI:

| Target | Surface | Result |
|---|---|---:|
| `can_frame` | CAN ID/frame construction | 100 passed |
| `com_packing` | COM descriptor packing/unpacking | 100 passed |
| `docan_state` | ISO-TP state-machine transitions | 100 passed |
| `doip_diagnostic` | DoIP diagnostic messages | 100 passed |
| `isotp_codec` | ISO-TP decode | 100 passed |
| `uds_dispatch` | UDS router | 100 passed |
| `doip_header` | DoIP generic header | 100 passed |
| `doip_payload` | DoIP payload variants | 100 passed |
| `doip_tcp` | bounded DoIP TCP stream framing | 100 passed |
| `storage_record` | bounded record queue | 100 passed |
| `fixed_containers` | `FixedVec` mutation/drop paths | 100 passed |
| `memory_queue` | byte-queue framing/wraparound | 100 passed |
| `uds_connections` | connection allocation/state | 100 passed |

The host has only the Visual Studio 2019 MSVC sanitizer ABI, so Windows
execution is rejected with a clear requirement for Visual Studio 2022. The
authoritative smoke ran in a disposable Linux container matching the Ubuntu CI
sanitizer model. `tools/port/check_fuzz.ps1` is the CI entry.

## Functional-safety support

`bsw-safety` adds 11 deterministic tests for monitor latch/reset/repeat,
injected supervisor routing, bounded watchdog fast test, retained-record CRC,
incremental ROM corruption, MPU layout, capability reporting and composed task
faults. The crate cross-checks `no_std` on ARM. Claims, physical MPU evidence and
ECC exclusions are frozen in `safety-mechanisms-2026-07-18.md`.
