# Master Plan: Complete the Eclipse OpenBSW Rust Port

- Status: mandatory execution complete; packages A01-I09 completed
- Plan date: 2026-07-17
- Execution checkpoint: 1,092/1,092 mandatory package-hours (100%) completed on 2026-07-19
- Next package: none in mandatory scope; S01-S18 remain optional and require separate authorization
- Workspace: `openbsw-rust`
- Upstream parity baseline: Eclipse OpenBSW commit [`be0029bbb79fe901048a24c2665f2ba854328734`](https://github.com/eclipse-openbsw/openbsw/commit/be0029bbb79fe901048a24c2665f2ba854328734), re-pinned 2026-07-20 by the governed re-pin tranche (`docs/port/repin-2026-07-20.md`, scope per `docs/port/upstream-repin-decision-2026-07-19.md`). The plan was written against, and the completed mandatory release was evidenced at, the previous baseline `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; that release evidence is unchanged.

The completed mandatory checkpoint is backed by the versioned G12-I09 matrix,
two independent cache-free source roots, byte-identical pinned POSIX and MCU
release artifacts, full host/Linux/target/fuzz/Miri/policy gates, and the
exact-artifact two-board storage, safety, recovery, reset, and soak campaign.
Exact build identities remain in ignored private evidence.

## 1. Outcome

Finish a maintainable Rust implementation of the platform-independent Eclipse OpenBSW stack and its reference-application behavior, prove it first on a deterministic POSIX host, and then ship equivalent reference applications for the STM32G474RE and STM32F413ZH boards already supported by this repository.

"Finished" means all mandatory acceptance criteria in section 11 are met. It does not mean ISO 26262 certification, AUTOSAR conformance certification, or support for every microcontroller that could run OpenBSW.

## 2. Current Repository Baseline

The repository is not a greenfield port. It contains 13 Rust crates and useful implementations of fixed-capacity containers, CAN types, ISO-TP codecs and state machines, a simplified transport/UDS stack, Ethernet and DoIP types, lifecycle/runtime utilities, STM32 drivers, storage, and HIL tests.

Evidence collected while preparing this plan:

- `cargo test --workspace --exclude bsw-bsp-stm32` passes.
- Cargo lists 1,062 host and documentation tests; three documentation examples are ignored.
- Host `--all-features` checking passes.
- STM32F413 and STM32G474 cross-checks pass for `thumbv7em-none-eabihf`.
- The F4 build emits 27 warnings and the G4 build emits 32 warnings, including Rust 2024 `static_mut_refs` warnings in the interrupt queue.
- The BSP has 14 example binaries totaling about 7,500 lines. Several retain inline or local MMIO, ISO-TP, UDS, and application logic that should live in reusable crates.
- `DiagCanTransport` advertises full ISO-TP conceptually but currently caps messages at 256 bytes and contains placeholder timing state.
- `bsw-com` handles only byte-aligned signals.
- the UDS routine uptime is a placeholder and stop-routine behavior is incomplete.
- `bsw-doip` provides packet types and routing structures but not a complete live DoIP entity.
- There is no production POSIX reference application, SocketCAN platform crate, logging/console stack, complete async layer, storage abstraction with power-loss semantics, middleware/configuration path, or functional-safety stack.
- Existing README and verification documents disagree on test totals and HIL scope. Historical HIL reports are evidence of prior runs, not a current release result.

Therefore, the current repository is a strong vertical slice, not yet upstream feature parity.

## 3. Scope and Finish-Line Decisions

### 3.1 Mandatory scope

Port or provide a behaviorally equivalent Rust design for the upstream modules that participate in the reference application:

- common, util, time, timer, async execution, runtime
- lifecycle and platform abstractions
- logger, logger integration, console, and command handling
- I/O, blob, and persistent storage
- CAN, Ethernet, and socket abstractions
- transport, simple transport routing, routing, and middleware interfaces
- DoCAN, UDS, and DoIP
- shared BSP input/output abstractions
- safe monitor, safe utilities, supervision, watchdog checks, ROM checks, and documented platform safety limits
- the reference application's lifecycle, console, CAN/UDS, Ethernet/DoIP, storage, ADC/PWM/GPIO demonstration, and tests

The mandatory runtime platforms are:

1. POSIX host, used as the deterministic behavioral oracle and integration target.
2. STM32G474RE, using the existing FDCAN-oriented BSP.
3. STM32F413ZH, using the existing bxCAN-oriented BSP.

### 3.2 Native Rust substitutions

Parity is behavioral, not a mechanical class-for-class translation. Native Rust constructs replace C++ helpers when they preserve required semantics:

- `Option`, enums, slices, arrays, traits, `core::fmt`, and RAII replace corresponding `estd`/utility wrappers.
- const generics and fixed-capacity storage replace dynamic allocation on embedded targets.
- safe ownership replaces listener-pointer and object-pool patterns where possible.
- unsafe code is restricted to hardware, FFI, allocator internals, and concurrency primitives with explicit safety contracts.

Every intentional non-port is recorded in the parity manifest with its Rust replacement and equivalence evidence.

### 3.3 Explicit exclusions

- ISO 26262 certification or a safety case suitable for production approval
- AUTOSAR Classic API compatibility beyond behavior already provided by OpenBSW
- bootloader, secure boot, OTA update, or production cryptographic key management
- proprietary vehicle DIDs, routines, calibration data, or network databases
- Ethernet hardware on boards that do not expose usable Ethernet hardware
- translation of third-party libraries when a suitable licensed Rust or existing C dependency is retained behind a platform boundary
- unpinned changes made upstream after the selected parity baseline; they are handled by the final delta assessment

### 3.4 Optional upstream-platform parity

The upstream S32K148 platform is not part of the mandatory STM32 finish line. Section 10 provides a separate package tranche for it. Selecting that tranche adds platform parity without blocking completion of this repository's existing product direction.

## 4. Target Architecture

The end state should have dependency direction from application toward protocols, abstractions, and finally platform adapters:

```text
openbsw-reference-app
  |-- bsw-lifecycle / bsw-async / bsw-runtime
  |-- bsw-console / bsw-logger
  |-- bsw-uds -- bsw-transport -- bsw-routing
  |             |-- bsw-docan -- bsw-can
  |             `-- bsw-doip  -- bsw-ethernet
  |-- bsw-storage -- bsw-blob -- bsw-io
  |-- bsw-safety
  `-- platform adapter
        |-- bsw-platform-posix
        `-- bsw-bsp-stm32
```

Required architectural properties:

- Protocol and state-machine crates are `no_std`, heap-free by default, and hardware independent.
- `std` enables host adapters, test utilities, files, threads, and sockets; it does not alter protocol behavior.
- UDS does not depend on CAN. It consumes the generic transport interface.
- DoCAN and DoIP are transport adapters and can be tested without physical hardware.
- Storage clients do not know flash geometry. Atomicity and recovery are defined by the storage contract.
- Board binaries contain configuration and composition only. They do not contain duplicate protocol or driver implementations.
- Timeouts use an injected monotonic clock. No protocol crate reads a device timer directly.
- Logging never becomes a hidden allocation or blocking dependency in embedded builds.
- Safety mechanisms are separated into generic, platform-specific, and application-specific layers.

## 5. Execution Rules

### 5.1 Work-package cap

Every package below is estimated at 8 focused engineering hours or less, including implementation, tests, local documentation, and self-review. Review by a second person is listed separately where it is required.

If a package reaches its estimate without satisfying its done condition:

1. stop at a compiling, documented boundary;
2. record the discovery in the parity tracker;
3. split the remainder into packages of 8 hours or less;
4. do not silently extend the package.

Estimates exclude queue time for CI, hardware availability, procurement, and organizational approval. They are planning estimates with approximately +/-50% uncertainty until phase A is complete.

### 5.2 Definition of done for every package

A package is done only when:

- the named output exists and is scoped to the package;
- new or changed behavior has automated tests;
- host tests, relevant feature checks, and relevant cross-checks pass;
- warnings do not increase;
- public API or behavior changes update local documentation and the parity manifest;
- each new unsafe block has a `// SAFETY:` justification and is linked to an invariant test or review item;
- no private bench identifiers, credentials, personal data, or private network addresses enter tracked files;
- the change is reviewable as one focused commit or pull request.

### 5.3 Parity proof hierarchy

Use the strongest available proof for each feature:

1. identical execution of an upstream unit test port;
2. differential execution against the pinned C++ POSIX oracle;
3. common golden vectors or packet traces;
4. property/fuzz tests for parsers and state machines;
5. POSIX system scenario;
6. target HIL scenario.

Hardware success alone does not establish protocol parity, and unit tests alone do not establish board integration.

## 6. Phase Gates

| Gate | Exit condition |
|---|---|
| A - Controlled baseline | Pinned upstream, reproducible oracle, parity manifest, architecture decisions, and clean baseline evidence exist. |
| B - Trustworthy workspace | Clean warning policy, feature matrix, unsafe/concurrency controls, fuzz entry points, and CI quality gates are active. |
| C - Runtime foundation | Common, time, async, runtime, logger, and console layers pass host and `no_std` tests. |
| D - Services foundation | Lifecycle, I/O, storage, CAN, and Ethernet abstractions have host implementations and conformance tests. |
| E - Protocol parity | Transport, routing, middleware, DoCAN, UDS, DoIP, COM, and E2E behavior meet the pinned parity manifest. |
| F - POSIX reference | A Rust POSIX reference application passes differential and fault-injection scenarios. No C++ is used in the production path. |
| G - STM32 convergence | Both board applications use the same production crates and pass the defined HIL matrix. |
| H - Safety mechanisms | Generic safety mechanisms and platform limits are implemented, tested, and explicitly non-certified. |
| I - Release | Mandatory parity rows are closed, duplicate prototypes are gone, evidence is reproducible, and release acceptance passes. |

Do not begin the main protocol expansion before gate B. Host integration may proceed in parallel across independent lanes after gate C. Hardware work may start before gate F only where it does not force protocol behavior into a BSP.

## 7. Mandatory Work Packages

All packages begin in `TODO` state.

### Phase A - Control the baseline (64 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| A01 | 4 | - | Record the upstream commit, license, NOTICE obligations, supported features, and platform list. Done when one reviewed baseline document names every source of truth. |
| A02 | 8 | A01 | Create a machine-readable parity manifest mapping every upstream BSW/BSP/safety/reference-app module to `done`, `partial`, `missing`, `native replacement`, or `excluded`. Done when every upstream module has an owner row and evidence field. |
| A03 | 6 | A01 | Capture reproducible current Rust baseline commands and results for tests, all features, F4/G4 cross-checks, warnings, binary sizes, and ignored tests. Done when a dated report can be regenerated locally. |
| A04 | 6 | A01 | Pin Rust toolchain, target components, formatter, linter, Python HIL dependencies, and minimum supported Rust version policy. Done when a clean machine has one documented setup path. |
| A05 | 8 | A01,A04 | Build the pinned upstream POSIX reference application and unit-test entry point outside the Rust production graph. Done when a script can build and run the C++ oracle from its pinned checkout. |
| A06 | 8 | A05 | Inventory upstream unit tests and reference-app scenarios by module; identify tests that can be ported or driven differentially. Done when each parity row cites candidate tests. |
| A07 | 8 | A05 | Define a versioned oracle protocol for feeding inputs and capturing outputs, state transitions, logs, and packet traces. Done when one smoke scenario compares C++ and Rust-shaped fixtures deterministically. |
| A08 | 6 | A02 | Write architecture decision records for crate boundaries, `std`/`no_std`, allocation, time, error handling, configuration, and platform ownership. Done when prohibited dependencies are explicit. |
| A09 | 6 | A02,A08 | Define the test/evidence directory layout, naming, result schema, HIL metadata, and trace-retention policy. Done when one sample result validates against the schema. |
| A10 | 4 | A02,A09 | Add a parity/status report generator that reads the manifest rather than hard-coded README numbers. Done when it produces consistent module and test summaries. |

### Phase B - Make the workspace trustworthy (90 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| B01 | 8 | A03,A04 | Remove F4/G4 compiler warnings caused by inactive-target constants and feature scoping. Done when both cross-checks are warning-free except separately accepted findings. |
| B02 | 8 | B01 | Replace references to `static mut` in the ISR queue with raw pointers, atomics, or a proven SPSC primitive. Done when Rust 2024 compatibility warnings are zero and queue tests pass. |
| B03 | 6 | A04 | Define and test valid feature combinations, including mutual exclusion of STM32 chip features, classic CAN/CAN FD, `std`, and test utilities. Done when invalid combinations fail with clear diagnostics. |
| B04 | 8 | A08 | Audit public APIs for panic-on-input behavior and introduce typed errors where malformed external data can reach them. Done when the policy and first migration set are tested. |
| B05 | 6 | A04 | Normalize workspace metadata, editions, lint inheritance, documentation lint policy, panic behavior, and release profiles. Done when all manifests follow one policy. |
| B06 | 6 | B03 | Add `no_std` build checks and an allocation detector for protocol crates. Done when CI fails on an accidental allocator or `std` dependency. |
| B07 | 8 | A02 | Produce an unsafe-code inventory with owner, invariant, caller obligations, target, and review status. Done when every production unsafe site is classified. |
| B08 | 8 | B07 | Run Miri-compatible tests over fixed containers, object pools, queues, and adapters; repair findings within package scope. Done when the selected host suite is clean. |
| B09 | 8 | B02,B08 | Add concurrency model tests for SPSC queues and interrupt/main-loop handoff, using model checking or controlled interleavings. Done when wrap, overflow, and ordering invariants are covered. |
| B10 | 8 | A06 | Establish fuzz targets for CAN IDs/frames, ISO-TP, UDS dispatch, DoIP headers, storage records, and COM packing. Done when each target completes a bounded CI smoke run. |
| B11 | 8 | A03 | Add reproducible code-size, stack-usage, worst-case buffer, and host benchmark baselines. Done when regressions can be compared per target and crate. |
| B12 | 8 | B01-B11 | Upgrade CI to enforce formatting, warning-free clippy, host/all-feature tests, `no_std`, both MCU checks, fuzz smoke, dependency/license policy, and generated status consistency. Done when all mandatory jobs pass. |

### Phase C - Runtime, logging, and console foundations (132 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| C01 | 8 | B12 | Add or designate a common crate for result codes, bounded byte views, identifiers, and shared traits that are duplicated today. Done when dependency direction stays acyclic. |
| C02 | 4 | A02,B12 | Complete the `estd` parity decision table, including explicit native Rust replacements. Done when no upstream utility type is unclassified. |
| C03 | 8 | C02 | Implement the first missing fixed-capacity primitives required by later modules, such as bounded strings/spans or intrusive hooks. Done when upstream-equivalent vectors pass. |
| C04 | 8 | C02,C03 | Harden existing fixed vectors, maps, lists, pools, endian types, and iterators for drop, aliasing, capacity, and zero-sized edge cases. Done when Miri/fuzz regressions are covered. |
| C05 | 6 | A02,C01 | Close the util parity table and document native substitutions for formatting, enum, defer, and string helpers. Done when downstream modules have no unresolved utility dependency. |
| C06 | 6 | C05 | Align CRC parameterization, initial/final values, reflection, tables, and upstream golden vectors. Done when Rust and C++ oracle vectors match. |
| C07 | 8 | C01 | Create monotonic instant, duration, deadline, wraparound, and clock traits usable in host and embedded code. Done when wrap and conversion tests pass without float arithmetic. |
| C08 | 8 | C07 | Implement timer registration, cancellation, periodic timers, ordering, and timeout dispatch. Done when deterministic fake-clock tests cover boundary ordering. |
| C09 | 8 | C07,C08 | Port the platform-independent async context/runnable/executor interfaces without heap allocation. Done when task posting and cancellation match oracle scenarios. |
| C10 | 8 | C09 | Implement the POSIX executor with deterministic single-thread mode and optional threaded wakeup. Done when fake-clock and real-clock integration tests pass. |
| C11 | 8 | C09 | Adapt the current embedded scheduler into the common async contract. Done when no protocol component depends on the BSP scheduler type. |
| C12 | 8 | C07,C09 | Complete runtime statistics, task/function monitoring, jitter, reset/snapshot behavior, and overflow handling. Done when oracle and property tests pass. |
| C13 | 8 | C01 | Implement no-allocation log records, component IDs, levels, compile-time filtering, timestamps, and format arguments. Done when disabled logging has measured negligible cost. |
| C14 | 8 | C13 | Implement bounded buffering, overflow policy, sinks, and logger integration hooks. Done when slow/missing sinks cannot block safety-critical callers. |
| C15 | 8 | C01,C13 | Implement console line editing, tokenization, command registry, help, errors, and bounded input. Done when malformed and maximum-length inputs are tested. |
| C16 | 8 | C09,C15 | Integrate console input/output with async execution and lifecycle. Done when commands can be posted without polling protocol code directly. |
| C17 | 6 | C10,C16 | Add POSIX stdin/stdout console adapters and capture-friendly test sinks. Done when scripted command sessions are deterministic. |
| C18 | 6 | C11,C16 | Add the common UART console adapter contract for embedded platforms. Done when a mock UART proves partial write, overflow, and recovery behavior. |

### Phase D - Lifecycle, data services, CAN, and Ethernet (160 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| D01 | 8 | C09,C13 | Extend lifecycle to upstream run levels, ordering, registration, callbacks, and shutdown semantics. Done when the pinned lifecycle scenarios match. |
| D02 | 8 | D01,C08 | Add pending transitions, completion callbacks, timeout/error propagation, rollback policy, and idempotence. Done when failed and asynchronous transitions are deterministic. |
| D03 | 6 | C01,C07 | Define reset, watchdog, interrupt lock, critical section, random/unique data, and platform information traits. Done when POSIX mocks cover every contract. |
| D04 | 8 | C01,C04 | Complete missing I/O reader/writer adapters and semantics from upstream. Done when allocate/commit and peek/release contracts have conformance tests. |
| D05 | 8 | D04,B08-B10 | Prove I/O queue and adapter soundness under wrap, partial operation, backpressure, and concurrency. Done when Miri/model/fuzz suites are clean. |
| D06 | 8 | D04,C06 | Define storage block IDs, metadata, versioning, integrity, result codes, and asynchronous operations independent of flash geometry. Done when an in-memory backend passes. |
| D07 | 8 | D06 | Implement two-copy or journaled atomic update, startup recovery, wear metadata, and torn-write handling. Done when every simulated cut point restores old or new valid data. |
| D08 | 6 | D07,C10 | Add a POSIX file-backed storage backend with deterministic fault injection. Done when it passes the same backend conformance suite. |
| D09 | 8 | D07 | Adapt the G4 flash/NvM code to the storage contract and remove application-specific block handling. Done when host geometry tests and G4 cross-check pass. |
| D10 | 8 | D07 | Implement the F4 flash/storage backend with explicit sector ownership and linker protection. Done when geometry/recovery tests and F4 cross-check pass. |
| D11 | 8 | D04,D06 | Port blob streaming, bounded descriptors, read/write completion, and storage integration. Done when large and partial blobs work without heap allocation. |
| D12 | 6 | A02,C01 | Close the CAN parity map for IDs, frames, filters, transceiver state, listeners, callbacks, statistics, and errors. Done when every upstream API behavior is assigned. |
| D13 | 8 | D12,C09 | Implement receive/transmit listener events, callback ordering, registration/removal, and ownership safely. Done when reentrant and capacity cases pass. |
| D14 | 8 | D12,D13 | Complete CAN FD DLC mapping, timestamps, error states, bus load/statistics, mute/open/close, and bus-off semantics. Done when classic and FD conformance suites pass. |
| D15 | 8 | D13,D14,C10 | Add a SocketCAN backend with filters, nonblocking I/O, timestamps, loopback control, and error-frame decoding. Done when virtual CAN integration passes on Linux CI. |
| D16 | 8 | D14 | Add an in-process virtual CAN bus with arbitration-neutral delivery, delay/drop/error injection, and bus-off recovery tests. Done when protocol tests no longer require shell subprocess timing. |
| D17 | 6 | A02,C01 | Close Ethernet parity for addresses, endpoints, configurations, TCP/UDP contracts, listeners, and errors. Done when every upstream behavior is mapped. |
| D18 | 8 | D17,C10 | Implement POSIX UDP bind/send/receive/multicast/broadcast adapters. Done when loopback and failure conformance tests pass. |
| D19 | 8 | D17,C10 | Implement POSIX TCP client/server lifecycle, partial send, backpressure, close reasons, and multiple connections. Done when loopback conformance tests pass. |
| D20 | 8 | D17 | Define and implement the lwIP socket boundary needed by upstream without leaking C types into protocol crates. Done when a fake lwIP backend passes the socket contract. |
| D21 | 8 | D18-D20,D01 | Integrate network configuration, startup/shutdown, address changes, and logging into lifecycle. Done when host network restart scenarios recover cleanly. |

### Phase E - Routing, diagnostics, DoIP, COM, and E2E (272 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| E01 | 8 | D04,C04 | Complete transport message addressing, payload ownership, fixed pools, provider errors, and release semantics. Done when pool exhaustion and reuse match oracle tests. |
| E02 | 8 | E01,C09 | Implement transport layer listeners, processed callbacks, send cancellation, shutdown, and asynchronous completion. Done when callback ordering is deterministic. |
| E03 | 8 | E01,E02 | Port the simple transport router with source/target matching, broadcast rules, and backpressure. Done when multi-layer routing tests pass. |
| E04 | 8 | E03 | Port the general routing module's route declarations, endpoints, status, and error paths. Done when representative upstream routes are expressible without application glue. |
| E05 | 8 | E04,C01 | Port middleware interfaces and typed message/service contracts required by the reference application. Done when simulation and production implementations share one API. |
| E06 | 8 | E05 | Define the middleware configuration schema and checked intermediate representation. Done when invalid names, IDs, types, and routes fail with useful errors. |
| E07 | 8 | E06 | Implement Rust code/config generation for one representative middleware model. Done when generated code compiles in `std` and `no_std` modes. |
| E08 | 8 | E06,E07 | Add golden/differential generator fixtures and deterministic output checks. Done when regeneration is stable and CI detects drift. |
| E09 | 8 | D14,E01 | Complete DoCAN normal, extended, and mixed addressing plus classic CAN/CAN FD length handling. Done when codec vectors cover all selected formats. |
| E10 | 8 | E09,C08 | Complete DoCAN RX allocation retries, block sizes, sequence wrap, N_Br/N_Cr timers, overflow, abort, and recovery. Done when fake-clock state tests pass. |
| E11 | 8 | E09,C08 | Complete DoCAN TX N_As/N_Bs/N_Cs timers, STmin pacing, block-size pauses, WAIT limits, overflow, cancellation, and confirmation. Done when fake-clock state tests pass. |
| E12 | 8 | E10,E11,E02 | Build the platform-independent DoCAN transport layer with connection configuration and message pools. Done when it runs over virtual CAN without BSP dependencies. |
| E13 | 8 | E12,A07 | Differential-test DoCAN against pinned C++ traces and fuzz malformed transitions. Done when all supported addressing/timing rows close in the parity manifest. |
| E14 | 8 | E01,C04 | Complete UDS service IDs, NRCs, sessions, request matching, diagnostic jobs, and bounded configuration primitives. Done when constants and matching tests mirror upstream. |
| E15 | 8 | E14,E02 | Port the diagnostic dispatcher, job tree ordering, functional/physical addressing rules, and transport registration. Done when overlapping-job and unsupported-service cases match. |
| E16 | 8 | E15 | Port incoming/outgoing diagnostic connection pools, ownership, termination, and processed notifications. Done when exhaustion, cancellation, and reuse pass. |
| E17 | 8 | E16,C08 | Implement P2/P2*, S3, response-pending, suppress-positive-response, and timeout behavior with injected time. Done when boundary-time scenarios pass. |
| E18 | 8 | E17,D06 | Implement diagnostic session state, persistence hooks, authentication/security state, attempt counters, delays, and reset policy. Done when reset and lockout scenarios match configuration. |
| E19 | 8 | E15,E17 | Port TesterPresent, DiagnosticSessionControl, ControlDTCSetting, and CommunicationControl. Done when positive, negative, suppress, session, and length cases match. |
| E20 | 8 | E15,D06 | Replace local example handlers with configurable ReadDataByIdentifier and WriteDataByIdentifier registries. Done when multi-DID, permissions, persistence, and length cases pass. |
| E21 | 8 | E18,D03 | Port ECUReset and SecurityAccess with deferred reset and pluggable project-owned key derivation. Done when sequence, delay, attempts, and post-response reset are tested. |
| E22 | 8 | E15 | Port RoutineControl start/stop/results and InputOutputControl with handler registries. Done when placeholder uptime and hard-coded LED/CAN routines are removed from protocol code. |
| E23 | 8 | E15,D06 | Complete DEM adapter, ReadDTCInformation, and ClearDiagnosticInformation behavior used upstream. Done when status masks, groups, persistence, and disabled recording pass. |
| E24 | 8 | E16-E23 | Port upstream UDS async, resume, and authentication jobs not covered by individual services. Done when the parity manifest has no unassigned UDS production path. |
| E25 | 8 | E14-E24,A07 | Port upstream UDS unit scenarios, run differential traces, and fuzz request dispatch/connections. Done when all mandatory UDS parity rows close. |
| E26 | 8 | D17,E01 | Complete DoIP headers, payload parsing/serialization, generic NACKs, version checks, and size bounds. Done when golden and fuzz tests cover every payload type used upstream. |
| E27 | 8 | E26,D18 | Implement UDP vehicle identification requests, announcements, discovery timing, and entity status/power-mode messages. Done when multiple-client loopback tests pass. |
| E28 | 8 | E26,D19,C08 | Implement TCP routing activation, activation policy, alive checks, inactivity timers, connection limits, and teardown. Done when fake-clock connection tests pass. |
| E29 | 8 | E28,E02 | Implement diagnostic messages, positive/negative acknowledgements, source/target validation, and generic transport bridging. Done when UDS works over a loopback DoIP connection. |
| E30 | 8 | E27-E29,D21 | Compose a live POSIX DoIP entity under lifecycle control. Done when discovery, activation, diagnostics, restart, and malformed-client scenarios pass. |
| E31 | 8 | E26-E30,A07 | Differential-test and fuzz the DoIP entity against upstream packet traces. Done when all mandatory DoIP parity rows close. |
| E32 | 8 | D14,B10 | Finish `bsw-com` non-byte-aligned Intel/Motorola packing, signed scaling boundaries, invalid descriptors, and CAN FD payloads. Done when property tests round-trip supported signals. |
| E33 | 8 | E32,C07 | Complete COM cyclic/event TX, RX deadlines, update/invalid flags, timeout defaults, and error reporting required by the reference app. Done when fake-clock scenarios pass. |
| E34 | 8 | C06,E33 | Define supported E2E profiles and close CRC/counter/data-ID/status semantics, or mark the current algorithm as a project extension. Done when profile vectors and COM integration pass. |

### Phase F - POSIX platform and reference application (80 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| F01 | 8 | C10,D03,D15,D18,D19 | Create `bsw-platform-posix` with clock, executor, reset simulation, console, SocketCAN, TCP/UDP, file storage, and test controls. Done when adapters share conformance suites with mocks. |
| F02 | 8 | E34,F01 | Create a configuration/composition crate for the Rust reference application with capacities, routes, jobs, storage blocks, network settings, and feature switches. Done when configuration is separate from logic. |
| F03 | 8 | F02,D02,C14,C16 | Compose lifecycle, async runtime, logging, console, commands, and clean shutdown. Done when scripted startup/run/shutdown logs are stable. |
| F04 | 8 | F02,E05 | Implement simulated ADC, PWM, GPIO inputs/outputs and middleware bindings matching the upstream demo behavior. Done when values can be driven and observed deterministically. |
| F05 | 8 | F02,E13,E25 | Compose SocketCAN, DoCAN, UDS, configured DIDs/routines, and diagnostic console commands. Done when all diagnostics work over virtual CAN. |
| F06 | 8 | F02,E30 | Compose Ethernet, UDP/TCP, and DoIP with the same UDS dispatcher. Done when CAN and IP diagnostics coexist without duplicate UDS state. |
| F07 | 8 | F02,D08,D11 | Compose storage/blob services and demonstrate persistence/recovery through application commands and DIDs. Done when restart and injected torn-write tests pass. |
| F08 | 8 | F03-F07 | Reproduce the upstream getting-started workflows for console, lifecycle, logging, commands, CAN, Ethernet, UDS, hardware I/O simulation, and tracing. Done when examples are automated scenarios. |
| F09 | 8 | F08,A07 | Run C++/Rust reference-app differential scenarios and document intentional output differences. Done when all mandatory host behavior rows close. |
| F10 | 8 | F09 | Run host restart, resource exhaustion, malformed traffic, concurrent clients, and bounded soak tests. Done when there are no leaks, hangs, panics, or unexplained divergence. |

### Phase G - STM32 BSP and HIL convergence (156 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| G01 | 8 | D03,F02 | Create typed board configurations and shared STM32 peripheral ownership; enforce exactly one MCU feature. Done when examples no longer invent pin/clock constants independently. |
| G02 | 8 | G01,B07 | Centralize volatile register access and remove raw RCC/GPIO/CAN MMIO helpers from application examples. Done when remaining unsafe MMIO is confined to reviewed driver modules. |
| G03 | 8 | G01,D13,B02 | Replace the BSP-local `CanReceiver` split with the common CAN receive contract and integrate the ISR queue into both drivers. Done when polling and interrupt modes use one API. |
| G04 | 8 | G03,D14 | Implement and test F4/G4 interrupt flags, queue overflow accounting, error interrupts, bus-off detection/recovery, and restart timing. Done when mock-register tests and cross-checks pass. |
| G05 | 6 | G01,C07 | Consolidate clock and DWT time implementations, including remainder/wrap behavior and measured clock assertions. Done when both targets expose the common monotonic clock. |
| G06 | 6 | G01,C18 | Put both UART drivers behind the console/log sink contract with bounded nonblocking behavior. Done when board apps contain no UART formatting loop. |
| G07 | 8 | G01,F04 | Implement shared GPIO input/output managers, debouncing/configuration, and typed pin ownership for both boards. Done when demo I/O uses common application code. |
| G08 | 8 | G01,F04 | Implement PWM abstraction and one verified PWM channel per board, or record a board-level hardware exclusion. Done when duty/frequency boundaries are tested. |
| G09 | 8 | G01,F04 | Implement ADC abstraction and one verified ADC channel per board, or record a board-level hardware exclusion. Done when calibration/range/error behavior is tested. |
| G10 | 8 | G01,D03 | Unify watchdog, reset reason, deferred reset, hard-fault capture, and no-init ownership across F4/G4. Done when reset causes can be observed after reboot. |
| G11 | 8 | D09,G01 | Complete G4 storage integration, linker reservation, recovery, wear behavior, and interrupt/power constraints. Done when storage backend conformance runs on target. |
| G12 | 8 | D10,G01 | Complete F4 storage integration, linker reservation, recovery, wear behavior, and interrupt/power constraints. Done when storage backend conformance runs on target. |
| G13 | 8 | G02-G12,B11 | Measure and enforce flash, RAM, stack, ISR stack, queue, and worst-case protocol buffer budgets for both release binaries. Done when documented limits have CI thresholds. |
| G14 | 8 | F02,G03-G13 | Build the G4 production reference application using only shared application/protocol crates plus the G4 platform adapter. Done when no inline protocol implementation remains. |
| G15 | 8 | F02,G03-G13 | Build the F4 production reference application using the same configuration and behavior, varying only board capabilities. Done when no inline protocol implementation remains. |
| G16 | 8 | A09,G14,G15 | Make the HIL fixture deterministic: unique IDs, target selection, reset/flash control, CAN state reset, timestamps, result schema, and per-test isolation. Done when two consecutive smoke runs agree. |
| G17 | 8 | G16 | Run and stabilize the G4 smoke matrix for boot, clock, UART, GPIO, CAN, lifecycle, UDS SF/MF, reset, and storage. Done when all mandatory smoke cases pass from clean flash. |
| G18 | 8 | G16 | Run and stabilize the F4 smoke matrix for boot, clock, UART, GPIO, CAN, lifecycle, UDS SF/MF, reset, and storage. Done when all mandatory smoke cases pass from clean flash. |
| G19 | 8 | G17,G18 | Execute controlled storage reset/power-interruption campaigns on both boards across each journal transition. Done when every reboot yields old or new valid data. |
| G20 | 8 | G17,G18 | Execute burst, queue-overflow, malformed ISO-TP, bus-off/recovery, repeated-reset, and bounded soak tests on both boards. Done when thresholds are explicit and all failures are explained. |

### Phase H - Functional-safety support mechanisms (72 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| H01 | 8 | C01,C13 | Port generic safe monitor condition/check/event primitives and safe utilities. Done when trigger, latch, reset, and repeated-event behavior is tested. |
| H02 | 8 | H01,D02 | Implement safe supervisor event routing, severity policy, logging, limp-home hooks, and reset requests. Done when policy is injected rather than hard-coded. |
| H03 | 8 | H02,G10 | Implement watchdog startup/fast-test state machine with a reset-safe test backend and board-specific capability declarations. Done when success/failure/reset loops are bounded. |
| H04 | 8 | H02,G10 | Persist reset/fault safety events through no-init memory and hand them to storage after startup. Done when integrity/version checks reject corrupt records. |
| H05 | 8 | H02,C06,G13 | Implement incremental ROM-region CRC checks with linker-provided bounds and idle scheduling. Done when corruption injection is detected without blocking startup. |
| H06 | 8 | H02,G01 | Define MPU regions and ISR pre/post hooks for each MCU, with explicit writable contexts and linker assertions. Done when negative access tests or debugger-assisted evidence exists. |
| H07 | 8 | H02,G01 | Implement feasible RAM/flash ECC reporting and document unsupported diagnostics for each MCU. Done when capabilities and limitations are testable and not overstated. |
| H08 | 8 | H03-H07,D02 | Compose the safety task/lifecycle and verify watchdog servicing, monitor events, ROM checks, and shutdown behavior under load. Done when fault-injection scenarios pass. |
| H09 | 8 | H08 | Produce a non-certification safety-mechanism report: assumptions, diagnostic coverage limits, unsafe boundaries, failure reactions, and remaining production work. Done when claims match evidence. |

### Phase I - Close parity and release (66 hours)

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| I01 | 8 | F10,G20,H09 | Audit every mandatory parity row and attach its strongest evidence. Done when no row is `missing`, `partial`, or lacks an approved native-replacement rationale. |
| I02 | 8 | I01 | Delete or clearly archive duplicate inline ISO-TP/UDS/MMIO examples and obsolete helpers; retain only focused driver demos and production apps. Done when the production path has one implementation per behavior. |
| I03 | 8 | I02 | Run the complete host, virtual CAN, POSIX network, fuzz smoke, cross-target, HIL, storage-cut, and safety fault matrix from clean state. Done when one versioned report indexes all artifacts. |
| I04 | 8 | I02 | Finish crate API docs, architecture guide, porting guide, board setup, POSIX quick start, test guide, and upstream parity notes. Done when a new developer can reproduce the reference app. |
| I05 | 8 | I02 | Generate dependency inventory/SBOM, audit licenses/advisories, verify Apache headers/NOTICE, and scan authored artifacts for private data. Done when release policy passes. |
| I06 | 8 | I03-I05 | Produce reproducible POSIX and two-board release artifacts with locked dependencies, map/size reports, checksums, source commit, and build instructions. Done when a second clean build matches policy. |
| I07 | 8 | I06 | Run release-candidate regression and a time-bounded soak on the exact artifacts. Done when no open severity-1/2 defects or unexplained flaky mandatory tests remain. |
| I08 | 6 | I07 | Compare the pinned upstream baseline with current upstream, classify the delta, and create follow-up packages without moving the release baseline. Done when drift is explicit. |
| I09 | 4 | I07,I08 | Conduct final acceptance review, freeze the parity/evidence manifest, write release notes, and mark the plan complete. Done when every section 11 criterion has an evidence link. |

## 8. Schedule, Dependencies, and Parallel Work

Mandatory package estimate: **1,092 focused engineering hours**, or about **27.3 ideal 40-hour engineering weeks**. With review, integration, hardware scheduling, rework, and normal project overhead, plan for roughly **8-12 calendar months for one engineer**, or **4-7 months for a coordinated team of three**. Phase A should recalibrate this estimate.

Recommended parallel lanes after gate B:

| Lane | Packages | Merge point |
|---|---|---|
| Runtime/tooling | C01-C18, D01-D03 | Gate C |
| Data/storage | D04-D11 | Gate D |
| CAN/network | D12-D21 | Gate D |
| Transport/diagnostics | E01-E31 | Gate E |
| COM extension | E32-E34 | Gate E |
| Host application | F01-F10 | Gate F |
| STM32 platform | G01-G20 | Gate G |
| Safety | H01-H09 | Gate H |

The critical path is expected to be A -> B -> C07-C11 -> D12-D16 -> E01-E25 -> F -> G -> I. DoIP can progress in parallel with UDS after transport interfaces stabilize. Storage and middleware can also progress independently after gates C and D respectively.

## 9. Risk Register

| Risk | Early indicator | Mitigation / decision point |
|---|---|---|
| Existing Rust APIs differ semantically from upstream despite high test counts | Oracle cases require application-specific workarounds | Close behavior in host differential tests before adding more board code. |
| Simplified UDS architecture cannot support upstream connection semantics | Response pending, concurrent connections, or cancellation require global state | Complete E14-E18 before adding services; permit an internal redesign while preserving public migration notes. |
| Embedded stack/RAM cannot hold configured transport pools | G13 exceeds RAM or stack budgets | Move large buffers to static owned pools, reduce configured concurrency, and document capacity; never hide heap allocation. |
| Flash geometry or power loss corrupts storage | D07 cut-point simulation or G19 fails | Keep old valid copy until commit record is durable; reserve linker regions and add recovery versioning. |
| ISR queue is unsound or loses bursts | B09 or G20 reveals order/overflow anomalies | Use a proven SPSC primitive with atomic/raw-pointer invariants and explicit overflow policy. |
| HIL remains flaky because the bench is shared or stateful | Consecutive clean runs disagree | G16 must isolate target, reset bus/server state, and record fixture identity generically before full HIL. |
| Ethernet cannot be HIL-tested on current STM32 boards | No PHY/interface is available | Prove Ethernet/DoIP on POSIX; document embedded hardware exclusion without weakening protocol parity. |
| Safety claims exceed MCU evidence | MPU/ECC/watchdog behavior is only compile-tested | Limit claims per board capability and mark mechanisms as non-certified. |
| Upstream changes during the project | New upstream modules appear | Keep the pinned baseline fixed; assess drift only in I08 or in deliberately scheduled rebases. |
| Package estimates are too coarse | A package reaches 8 hours unfinished | Apply the mandatory split rule; update the total rather than expanding a package. |

## 10. Optional S32K148 Platform-Parity Tranche

Execute only after gate F unless the project explicitly changes its target priority. These packages add **144 hours** and do not change the platform-independent parity baseline.

| ID | Hours | Depends | Package and done condition |
|---|---:|---|---|
| S01 | 8 | F10 | Pin S32K148 Rust target/PAC/toolchain, board docs, probe, and linker inputs. Done when a minimal binary links reproducibly. |
| S02 | 8 | S01 | Implement startup, vector table, memory init, and linker sections. Done when boot reaches a retained marker. |
| S03 | 8 | S02 | Implement clock, power, and measured monotonic timer. Done when clock assertions and timing smoke pass. |
| S04 | 8 | S03,C18 | Implement UART and console/log sink. Done when scripted serial smoke passes. |
| S05 | 8 | S03,G07 | Implement typed GPIO input/output and board LEDs/switches. Done when I/O smoke passes. |
| S06 | 8 | S03,G09 | Implement ADC channel and calibration path. Done when range and demo tests pass. |
| S07 | 8 | S03,G08 | Implement PWM channel and output manager path. Done when duty/frequency tests pass. |
| S08 | 8 | S03,D14 | Implement FlexCAN initialization, TX/RX, filters, and common CAN traits. Done when loopback passes. |
| S09 | 8 | S08 | Implement FlexCAN interrupts, error states, bus-off recovery, and queue statistics. Done when fault scenarios pass. |
| S10 | 8 | S02,D07 | Implement flash primitives and protected storage regions. Done when geometry and erase/program tests pass. |
| S11 | 8 | S10 | Integrate atomic storage and run reset/power-cut recovery. Done when backend conformance passes. |
| S12 | 8 | S03,C11 | Implement the selected RTOS/cooperative executor, task stacks, timers, and critical sections. Done when lifecycle tasks run deterministically. |
| S13 | 8 | S03,D03 | Implement watchdog, reset reasons, deferred reset, and fault capture. Done when reset scenarios are observable. |
| S14 | 8 | S12 | Integrate upstream-equivalent FreeRTOS/ThreadX behavior only if required by the selected S32K reference configuration. Done when the async contract remains unchanged. |
| S15 | 8 | H06,H07,S02 | Implement S32K MPU/ECC platform safety support and ISR hooks. Done when evidence matches the upstream safety scenario. |
| S16 | 8 | S04-S15,F02 | Compose the shared Rust reference application for S32K148. Done when only platform adapters differ. |
| S17 | 8 | S16,A09 | Add deterministic flash/reset/serial/CAN HIL fixture control for S32K. Done when two smoke runs agree. |
| S18 | 8 | S17 | Run boot, I/O, CAN/UDS, storage, watchdog, safety, and bounded soak HIL. Done when the optional platform parity report closes. |

## 11. Final Acceptance Criteria

The mandatory port is complete only when all items below are true:

1. The parity manifest has no mandatory `missing` or `partial` entries against the pinned upstream baseline.
2. Every native Rust replacement has a rationale and equivalence evidence.
3. The production dependency graph contains no C++ implementation of an OpenBSW module. Third-party C or platform libraries, if retained, are isolated and documented.
4. All platform-independent production crates build `no_std` without allocation.
5. Formatting, warning-free clippy, host tests, all-feature checks, `no_std` checks, fuzz smoke, and both MCU cross-checks pass in CI.
6. Unsafe code has documented invariants and completed review; Miri/model checks cover applicable abstractions.
7. The POSIX Rust reference application demonstrates lifecycle, console, logging, commands, CAN, UDS/DoCAN, Ethernet/DoIP, storage/blob, and simulated ADC/PWM/GPIO behavior.
8. Differential host scenarios match the pinned C++ oracle or have reviewed intentional-difference records.
9. The G4 and F4 production applications share protocol/application crates and differ only in configuration and platform adapters.
10. Both boards pass boot, clock/time, UART/console, GPIO, CAN, DoCAN, UDS, reset, storage-recovery, bus-off, burst, and bounded-soak HIL acceptance.
11. Unsupported hardware capabilities such as on-board Ethernet, MPU details, or ECC diagnostics are explicitly documented per board and do not weaken host protocol parity claims.
12. Flash, RAM, stack, queue, timing, and code-size budgets are measured and within documented thresholds.
13. No placeholder behavior, duplicate inline protocol implementation, unresolved high-severity defect, or flaky mandatory test remains.
14. License/NOTICE, dependency audit, SBOM, private-data scan, reproducible build metadata, and release evidence pass.
15. Documentation can take a new developer from checkout to host reference app and either STM32 release image without tribal knowledge.

## 12. First Executable Sequence

Start with A01 and proceed through A10. Do not begin by adding another protocol service or board example. The first concrete milestone is a controlled baseline and parity ledger; it is what turns all later work into measurable port completion rather than feature accumulation.

## 13. Mandatory completion record

The mandatory A01-I09 sequence closed on 2026-07-19 at 1,092/1,092 package
hours and 37/37 mandatory parity rows. Gate G is supported by enforced total
RAM/stack/queue/timing budgets, shared production composition, deterministic
fixture isolation, two-board smoke, seven journal reboot cuts, overflow,
error-passive/bus-off recovery, ten resets, actual resource exhaustion, and a
ten-minute soak per board. Gate H is supported by the non-certification H01-H09
report and exact-artifact fault matrix. Gate I is supported by clean-root
regression, privacy/unsafe/dependency policies, independent reproducibility,
current-upstream drift, and final acceptance.

The canonical publishable index is
`docs/test-evidence/samples/g12-i09-final-matrix.json`. Exact source/artifact
digests, maps, binaries, bench selectors, and raw logs remain only in ignored
private CI evidence. Optional S32K148 packages S01-S18 are not part of this
mandatory completion claim.

## 14. Baseline re-pin record (2026-07-20)

How to read this section: it records the executed U06 re-pin tranche and the
two remaining bench-bound steps. The audience is a future AI worker landing
cold; step structure follows the plan-writing rule; gates and dispositions
live in `docs/port/repin-2026-07-20.md` and the parity docs it names.

The pinned oracle moved `ddbcf88a62df -> be0029bbb79f` on 2026-07-20 under
tranche packages U07-U16 (`docs/port/repin-2026-07-20.md`). Mandatory rows
moved 37/37 -> 38/38 closed by one explicit decision (new upstream
`libs/bsw/time` module covered by the existing native `bsw-time` port);
package-hours remain 1,092/1,092; rows `bsw.routing` and `bsw.blob` were
added as optional/excluded by explicit decision. The
2026-07-18 release evidence remains the valid record at the old pin. The
tranche stopped at the software boundary because no HIL bench was available;
the open steps are:

- **Step ID**: RP-HIL-01
  - **Goal**: Re-verify the full physical matrix on both boards at the
    `be0029b` baseline.
  - **Inputs**: `hil/` suite, `tools/port/run_physical_matrix.ps1`, release
    artifacts rebuilt at the re-pinned working tree, both reference boards.
  - **Deliverables**: `docs/test-evidence/samples/repin-hil-matrix.json`
    (schema per `tools/port/validate_evidence.py`).
  - **Acceptance criteria**: all matrix roles pass at the thresholds in
    `docs/port/test-guide.md`; evidence file validates.
  - **Gate / review reference**: final-acceptance HIL gate
    (`docs/port/final-acceptance-2026-07-18.md` scope) at the new pin.
  - **Definition of done**: the evidence file exists and
    `python tools/port/validate_evidence.py` passes with it included.
- **Step ID**: RP-HIL-02
  - **Goal**: Decide adopt/decline for STM32 CAN register-level gaps 33-37
    with target verification available.
  - **Inputs**: `docs/port/can-parity.md` 2026-07-20 deferral records,
    `docs/port/stm32-can-drift-comparison-2026-07-19.md`, HIL bench.
  - **Deliverables**: per-gap decision entries in `docs/port/can-parity.md`;
    driver changes (if adopted) in `crates/bsw-bsp-stm32/src/` with tests;
    updated `docs/test-evidence/samples/repin-hil-matrix.json` roles.
  - **Acceptance criteria**: each gap carries adopt/decline with bus-off,
    overflow, and recovery roles re-run on both boards for any adoption.
  - **Gate / review reference**: same HIL gate as RP-HIL-01.
  - **Definition of done**: no gap remains in "deferred" state in
    `docs/port/can-parity.md`.
