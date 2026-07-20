# Middleware and reference-application composition drift review - 2026-07-19

Follow-up package U04 from `upstream-drift-2026-07-18.md`. This memo compares
upstream middleware (`libs/bsw/middleware`, lifecycle-related libraries) and
`executables/referenceApp` composition changes between the pinned oracle
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` and the drift tip `be0029bb`
against the Rust `bsw-middleware`, `openbsw-reference-app`, and
`bsw-reference-core` crates.

Sources: read-only bare checkout `target/drift/openbsw-drift.git` and the
analysis artifacts under `target/drift/analysis/` (`commits.tsv`,
`area-map.tsv`, `per-commit-files.txt`). The pin is the merge base of the
drift tip, so every commit listed below is drift-exclusive.

**The pinned oracle, `parity-manifest.json`, `status.md`, and all release and
evidence documents are untouched by this review.** Everything added is
drift-labeled test code plus this memo; drift-derived tests are explicitly
marked as NOT pinned-oracle evidence in their file headers.

## 1. Observable composition state at the drift tip

`executables/referenceApp/application/src/app/app.cpp` at `be0029bb` versus
the pin:

- Run levels 1-9 and the existing component order are unchanged.
- One new lifecycle component: `"routing"` (`RoutingSystem`) added at run
  level 6, guarded by `PLATFORM_SUPPORT_ETHERNET && PLATFORM_SUPPORT_CAN`.
- `LifecycleManager` timestamps now come from a central
  `::bsw::time::TimestampProvider` instead of `getSystemTimeUs32Bit`.
- `PLATFORM_SUPPORT_MIDDLEWARE=ON` for POSIX (`platforms/posix/Options.cmake`)
  composes the Foo middleware service demo into `DemoSystem` and creates the
  middleware shared-memory layout at the start of `startApp()`.
- New logger components `RUST` and `ROUTING`; lwIP logging routed through the
  logger's `va_list` overload instead of a fixed 100-byte buffer.
- `UdsSystem::addDiagJobs()` additionally registers 0x14
  ClearDiagnosticInformation and 0x19 ReadDTCInformation.
- `DemoSystem::shutdown()` cancels the cyclic timeout first, before CAN
  listener shutdown and socket close, and deinitializes the middleware proxy
  before the skeleton.

## 2. Commits reviewed and classification

Classes: (a) port already exhibits the behavior (test cited);
(b) in scope and testable now (drift-labeled test added);
(c) re-pin-dependent or out of the port's pinned scope (recorded only).

### Middleware (`libs/bsw/middleware`, `libs/bsw/time`) rework

| Commit | Subject (abridged) | Observable change | Class |
|---|---|---|---|
| `6dc89c53` | Port middleware modules | Inline message object 32 -> 64 bytes (`MAX_MESSAGE_SIZE = 64U`); `Future` state type formalized | (b) new test |
| `34fbf932` | Fix proxy dispatch missing error | Dispatch errors propagate via `ErrorState`/`Future` values 4..9 | (b) new test |
| `7a3c3f3f` | Integrate Foo service into DemoSystem | Skeleton broadcast every 1 s, proxy getter every 2 s at 10 ms cyclic; proxy deinit before skeleton | (b) contracts-level test; composition wiring is (c) |
| `dcfc215f` | Rename clusters Core0/Core1 -> Cluster0/Cluster1 | Generated names only; Rust routes by numeric cluster id | (a) neutral |
| `763e7f27` | Allocator generated from deployment model | Generation-from-model, no runtime change | (a) `representative_model_generation_is_deterministic_and_golden` |
| `4daedef7`, `c4401a13`, `24393a44`, `d4517940`, `175f6f28` | Generator/build/venv/pytest infra | Build-time generation and test infra | (a) same golden-generator test |
| `9d3e89a2` | Add TimestampProvider (`libs/bsw/time`) | One central time source for lifecycle and demo cadence | (a) injected-time design; plus new drift test |
| `b6633ce1`, `fdf7872d`, `0d42afeb` (middleware part) | POSIX shm simulation, proxy/skeleton/cluster connections | Runtime shm transport and connection processing | (c) no Rust shm runtime; contract only |
| `04096c32` | PayloadBuilder for non-trivially-copyable types | C++ type-system internals; Rust inline payloads are plain bytes | not observable in port |
| `179c96bc`, `0514bedc`, `b02a793b`, `db073661`, `00a8a021`, `96ba360a`, `67a454e9`, `52c9252d`, `fd6b9573`, `73207573` | const-ref payload, etl qualification, RAII/ctor init, lock and memory-interface cleanups | Internal; no workflow-observable change | none needed |

Contract cross-check at tip: `MAX_METHOD_ID = 128U` (unchanged) matches
`bsw_middleware::MAX_MEMBER_ID`; `ErrorState` values 0/4/5/6/7/8/9 and
`MessageType` bits 1/2/4/8/16 are unchanged from the pin and match the Rust
`ErrorState`/`MessageKind` discriminants exactly.

### Reference-application composition

| Commit | Subject (abridged) | Observable change | Class |
|---|---|---|---|
| `0d42afeb` + `175f6f28` | Add routing and blob | New `RoutingSystem` at run level 6, new `libs/bsw/routing` + `libs/bsw/blob`, `ROUTING` logger component | (c) new post-pin subsystem |
| `9558c245` | UDS 0x14 ClearDiagnosticInformation | Registered in `UdsSystem` job tree | (c), see divergence D1 |
| `f8132091` | UDS 0x19 ReadDTCInformation | Registered in `UdsSystem` job tree | (c), see divergence D1 |
| `120f5688` | Tester address translation in TP-Router | 2-byte DoIP tester addresses 0x0EF0-0x0EFB translated to 1-byte 0xF0-0xFB at bus boundaries | (c), see divergence D5 |
| `330514aa` | Correct number of protocol send jobs | `DoIpServerSystem` job-pool constant | (c) structural DoIP frontend difference; DoIP protocol parity owned by DoIP fixture suites |
| `b9f738bf` | Guard ETH_1 in transportRouterSimple | Second-Ethernet-interface configs only | (c) not applicable to port composition |
| `366d993e` | Keep lwIP netif persistent across lifecycle transitions | Networking survives lifecycle transitions | (b) new test |
| `b119bf9b` | Fix Tap restart failure during lifecycle transition | Restart robustness on POSIX | (b) same new test |
| `7a3c3f3f` (app part) | DemoSystem shutdown reorder | Cyclic timeout canceled before subsystem teardown | (b) new tests |
| `bd0078d0` | Fix posix stdin read error handling | Console input robustness | (a) bounded console: `f03_console_tree_and_log_level_are_scriptable`, tokenizer error path in `ReferenceApp::command` |
| `c9850632`, `b574834a` | Rust logging bridge, console_out crate | `RUST` logger component; `BUILD_RUST` startup prints reordered | (c)/native difference, see D4 |
| `489448c1`, `8857f6b9` | Uart.h include, Bazel target removal | Build-only | none needed |
| `39212d01` | User-specific configuration of module uds | UDS config surface (`UdsConfig.h`) | (c) config surface; service behavior owned by UDS drift survey |
| `aa9eb3d5` | Vararg review feedback (LifecycleManager.cpp) | Log formatting only; log text is not a parity key per `reference-app-parity.md` | (a) by policy |

## 3. Tests added (all drift-labeled, not pinned-oracle evidence)

1. `crates/bsw-middleware/tests/middleware_drift_2026_07_19.rs`
   - `drift_6dc89c53_tip_inline_capacity_of_64_bytes_is_bounded_and_transactional`:
     tip inline capacity (64 bytes) is representable via the const-generic
     `Message<64>`; oversize writes fail transactionally.
   - `drift_tip_error_state_and_message_kind_values_match_upstream`: pins the
     `ErrorState`/`MessageKind` discriminants and `MAX_MEMBER_ID` against the
     drift-tip values so a re-pin cannot silently change wire-observable
     numbers.
   - `drift_7a3c3f3f_foo_demo_cadence_workflow_is_expressible_on_the_shared_contract`:
     the Foo demo cadence (Event broadcast every 100th 10 ms cycle from
     cluster 0, getter Request every 200th cycle from cluster 1; 10 events and
     5 requests over 1000 cycles) runs on the shared
     `MiddlewareTransport` contract, including wrong-source-cluster rejection.
2. `crates/openbsw-reference-app/tests/reference_app_drift_2026_07_19.rs`
   - `drift_366d993e_network_echo_survives_reboot_and_full_restart`: UDP/TCP
     echo works while running, after console `lc reboot`, and after a full
     shutdown/start cycle (port equivalent of persistent netif/TAP).
   - `drift_7a3c3f3f_shutdown_is_quiescent_and_preserves_diagnostic_state`:
     shutdown generates no trailing diagnostic dispatches and the single
     shared `DiagnosticCore` counter survives restart.
3. `crates/bsw-reference-core/tests/composition_drift_2026_07_19.rs`
   - `drift_9d3e89a2_shutdown_uses_injected_time_only_and_is_quiescent`: the
     shared composition observes only injected `Instant` values (port
     equivalent of the centralized TimestampProvider) and shutdown mutates
     nothing beyond the running flag and shutdown timestamp.

All six pass with
`cargo test -p bsw-middleware -p openbsw-reference-app -p bsw-reference-core --all-features`.

## 4. Divergences (no production code changed)

- **D1 - UDS 0x14/0x19 in the composition** (`9558c245`, `f8132091`):
  drift-tip `UdsSystem` answers ClearDiagnosticInformation and
  ReadDTCInformation positively; the Rust reference-app `DiagnosticCore`
  routes 0x10/0x22/0x2e/0x31 plus TesterPresent/EcuReset and returns an NRC
  for 0x14/0x19. `bsw-uds::ServiceId` already enumerates both services, so
  only composition-level routing is missing. Classification:
  **upstream-behavior-change** (post-pin feature), re-pin-dependent. Not a
  bug in the port against the pinned oracle.
- **D2 - No RoutingSystem / routing / blob subsystem** (`0d42afeb`): the
  drift tip adds a PDU routing system at run level 6 with new libraries. No
  Rust counterpart exists and `reference-app-parity.md` has no routing row.
  Classification: **upstream-behavior-change**, re-pin-dependent.
- **D3 - No cyclic middleware demo in the Rust reference app** (`7a3c3f3f`,
  `b6633ce1`, POSIX `PLATFORM_SUPPORT_MIDDLEWARE=ON`): the drift-tip POSIX
  app runs the Foo service demo (shm layout at startApp, cluster connections
  in DemoSystem init, cadence in cyclic, proxy-before-skeleton deinit).
  `bsw-middleware` covers the contracts (and the new cadence test shows the
  workflow is expressible), but the composition wiring and a shm runtime do
  not exist. Classification: **upstream-behavior-change**, re-pin-dependent.
- **D4 - Rust-in-C++ bridge and startup prints** (`c9850632`, `b574834a`):
  upstream embeds Rust crates into the C++ app and reorders `BUILD_RUST`
  startup prints; the port is entirely Rust, and startup text is not a parity
  key. Classification: **deliberate-native-difference**.
- **D5 - Tester address translation** (`120f5688`): drift-tip TP-Router
  translates 2-byte DoIP tester addresses (0x0EF0-0x0EFB) to 1-byte
  (0xF0-0xFB) at bus boundaries; the Rust router matches full 16-bit logical
  addresses without a translation table. Classification:
  **upstream-behavior-change**, re-pin-dependent.

No bug-in-port findings were identified: every difference traces to post-pin
upstream additions or documented intentional platform differences, and no
production-code change is warranted before a re-pin decision.

## 5. Re-pin-dependent list

Work that only becomes meaningful if/when the oracle is re-pinned at or past
`be0029bb`:

1. Routing subsystem: `RoutingSystem` at run level 6, `libs/bsw/routing`,
   `libs/bsw/blob`, `ROUTING` logger component (`0d42afeb`, `175f6f28`,
   `b9f738bf`).
2. Middleware demo composition: POSIX middleware flag, shm memory layout at
   startup, DemoSystem cluster-connection init/cyclic/deinit ordering
   (`7a3c3f3f`, `b6633ce1`, `dcfc215f`).
3. Composition routing for UDS 0x14/0x19 in the reference app (`9558c245`,
   `f8132091`) - service-level protocol drift is covered by the UDS-area
   drift survey.
4. Tester address translation in the transport router (`120f5688`).
5. DoIP send-job pool sizing (`330514aa`) and user-specific UDS configuration
   surface (`39212d01`).

## 6. Verification

- `cargo test -p bsw-middleware -p openbsw-reference-app -p bsw-reference-core
  --all-features`: all suites pass, including the six new drift tests.
- `cargo fmt -p bsw-middleware -p openbsw-reference-app -p bsw-reference-core
  -- --check`: clean.
- `python tools/port/check_privacy.py`: pass.
- The pinned oracle fixtures, `parity-manifest.json`, `status.md`, and
  release/evidence documents were not modified.
