# Shared reference-application parity - packages F01-G15

Pinned upstream: `be0029bbb79fe901048a24c2665f2ba854328734`,
`executables/referenceApp` and `platforms/posix`.
(Re-pinned 2026-07-20 from `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; see
the re-pin section below and docs/port/repin-2026-07-20.md.)

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

## Re-pin 2026-07-20: dispositions at be0029b (rows F02-F10)

Oracle re-pin ddbcf88a -> `be0029bbb79fe901048a24c2665f2ba854328734`
(be0029b), governed by docs/port/upstream-repin-decision-2026-07-19.md;
tranche record: docs/port/repin-2026-07-20.md. Upstream oracle checkout at
be0029b: `target/oracle/openbsw` (suite 1813/1813; the POSIX reference app
boots and runs the middleware Foo demo). Dispositions from
docs/port/composition-drift-review-2026-07-19.md (D1-D5, RP-1, RP-2):

- **Service-middleware framework rework (119 new paths at the tip; U04 D3,
  RP-2) - declined shm runtime, native difference recorded.** The Rust
  `bsw-middleware` crate keeps its transport-abstraction design. The upstream
  shm-simulation runtime and the Foo demo composition wiring in DemoSystem
  (shm layout at startApp, cluster connections, cyclic cadence,
  proxy-before-skeleton deinit) are DECLINED: the port's transport contract
  is backend-agnostic by design, and a shared-memory simulation runtime is a
  POSIX demo vehicle, not observable middleware contract behavior. The
  observable contract is pinned by the promoted baseline tests
  (`crates/bsw-middleware/tests/baseline_be0029b.rs`): `ErrorState` values
  0/4/5/6/7/8/9, `MessageKind` bits 1/2/4/8/16, `MAX_MEMBER_ID` 128
  (upstream `MAX_METHOD_ID = 128U`), and the 64-byte inline message capacity
  (upstream `MAX_MESSAGE_SIZE = 64U`), which the const-generic
  `Message<INLINE>` expresses as `Message<64>`. The Foo demo cadence workflow
  (1 s broadcast, 2 s getter request, wrong-source-cluster rejection) is
  proven expressible on the shared contract by the promoted cadence test.
- **Shutdown quiescence and injected-time-only shutdown (7a3c3f3f,
  9d3e89a2) - conforming baseline behavior.** The Rust composition already
  exhibits the baseline observables (no trailing diagnostic dispatch during
  shutdown, shared diagnostic state preserved across restart, all timestamps
  from the injected clock as the native equivalent of the centralized
  TimestampProvider); pinned by the promoted tests
  `crates/openbsw-reference-app/tests/baseline_be0029b.rs` and
  `crates/bsw-reference-core/tests/baseline_be0029b.rs`.
- **lwIP netif persistence / Tap lifecycle (366d993e, b119bf9b) -
  conforming observable, registry not modeled.** Network echo survives a
  console reboot and a full shutdown/start cycle, matching the tip; pinned by
  the promoted test. The upstream netif config registry itself is a recorded
  native difference in docs/port/ethernet-parity.md (D17-D21).
- **RoutingSystem (RP-1) - explicit exclusion for this re-pin.** The
  upstream tip composition gains `RoutingSystem` at run level 6, guarded by
  `PLATFORM_SUPPORT_ETHERNET && PLATFORM_SUPPORT_CAN`, backed by the new
  `libs/bsw/routing` and `libs/bsw/blob` libraries. The Rust reference
  composition records this as an explicit exclusion for this re-pin: the new
  libraries do not enter the mandatory surface in this tranche (excluded rows
  are recorded in the manifest by the tranche record); see
  docs/port/repin-2026-07-20.md.

The reference-app oracle fixture pair
(`docs/port/oracle-fixtures/reference-app-{openbsw,rust}.json`) was
realigned to be0029b on 2026-07-20; the previous ddbcf88a pair is preserved
under `docs/port/oracle-fixtures/historical-ddbcf88/`. The seven mandatory
observable records (level-9 startup, console command tree, CAN
`124:0102 -> 125:0102` echo, DoCAN `22cf01 -> 62cf01` 24-byte DID, DoIP
`3e00 -> 7e00`, storage restart restore, level-0 shutdown) were re-verified
against the upstream sources at be0029b (`app.cpp` `MaxNumLevels = 9`,
`CanDemoListener` id+1 echo with 0x123/0x124 filter, `UdsSystem`
24-byte `responseData22Cf01`).

`bsw-reference-core` is the heap-free common application state used by POSIX,
F413 and G474. Both MCU production examples select `DemoRole::Application` and
compose the same lifecycle, I/O cycle, diagnostic router, static storage and
watchdog behavior in `board_apps.rs`. Dynamic diagnostic clients use the typed
fixed-capacity registry in `dynamic_client.rs`. No embedded example contains an
inline ISO-TP or UDS implementation.
