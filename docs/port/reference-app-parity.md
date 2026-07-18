# POSIX reference-application parity - packages F01-F10

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`,
`executables/referenceApp` and `platforms/posix`.

| Host behavior | Rust composition | Evidence |
|---|---|---|
| Nine run levels, startup, level changes, reboot, clean shutdown | `LifecycleSystem` over `RunLevelManager` | stable-log and repeated-restart tests |
| Console help, lifecycle, logger, stats, CAN, diagnostic, storage, I/O, trace commands | bounded `bsw-console` tokenization and application command tree | scripted getting-started workflow |
| CAN counter and 0x123/0x124 echo | injected-clock `CanDemo` | exact-boundary and echo tests |
| UDS over DoCAN | production ISO-TP `VirtualCanTransport` over `VirtualCanBus` | 24-byte CF01 multi-frame and configured DID/routine tests |
| UDS over DoIP | real `bsw-doip::Packet` frontend | cross-transport session-state test |
| Shared diagnostic state | one `DiagnosticCore` owned by `ReferenceApp` | session changed via CAN and observed via DoIP; shared dispatch count |
| UDP/TCP echo | real POSIX loopback sockets | automated socket workflow |
| Persistence and blob | `JournalStore` plus `BlobWriter`/`BlobReader` | remount and torn-write recovery tests |
| Simulated ADC/PWM/GPIO | deterministic middleware-bound `SimulatedIo` | range/boundary workflow |
| Resource exhaustion, malformed clients, concurrency, soak | bounded client admission and response buffers | F10 capacity, 513 malformed cases, 2,000 concurrent requests, and 10,000-command restart soak |

The pinned C++ reference ELF was rebuilt and run through
`tools/port/openbsw_oracle.ps1 -Action Reference`. The normalized F09 fixture
records cover the mandatory observable lifecycle, command-tree, CAN,
diagnostic, persistence, and shutdown rows; the Rust binary emits its stable
form with `cargo run -p openbsw-reference-app -- --oracle`.

Intentional platform differences are explicit:

- upstream POSIX uses TAP/lwIP, while Rust uses the already-conformant host
  socket adapters;
- upstream POSIX has no ADC/PWM/GPIO demo, so the deterministic I/O model is a
  project extension used to share application behavior with STM32;
- timestamps and implementation-specific log text are not parity keys; ordered
  lifecycle state and command outcomes are.

These differences do not weaken protocol packet parity, which is covered by
the DoCAN, UDS, and DoIP fixture suites separately.
