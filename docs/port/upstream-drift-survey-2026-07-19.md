# Upstream drift survey (U01) - 2026-07-19

This document is the U01 deliverable from
`docs/port/upstream-drift-2026-07-18.md`: a module/file-level survey of the
upstream drift between the pinned parity baseline and the current public
upstream tip, plus an exhaustive classification of every mandatory-surface
changed path. The pinned behavioral oracle was **not** moved for this survey,
and this survey does **not** re-pin the baseline; no parity claim beyond the
pinned baseline is made or implied here.

## Baseline, drift tip, and window

| Fact | Value |
|---|---|
| Pinned parity baseline (oracle) | `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` (unchanged) |
| Observed upstream drift tip | `be0029bbb79fe901048a24c2665f2ba854328734` |
| Ahead / behind | 166 commits ahead, 0 behind |
| Changed paths (total) | 3,556 |
| Changed paths (mandatory surface) | 1,854 |
| Recorded span | 2026-03-05 .. 2026-06-02 |
| Full author-date range | 2026-01-20 .. 2026-07-17 |

Date-span note: the recorded span 2026-03-05..2026-06-02 in the I08 drift
assessment is the first-log-line-to-tip author-date span. Because upstream
rebases merge older-authored commits after newer ones, the true author-date
minimum/maximum across all 166 commits is 2026-01-20..2026-07-17. Both facts
are recorded here; neither changes the commit count.

### Mandatory-surface routing rule

A changed path is selected for the mandatory surface iff it is under one of:

- `libs/` (excluding `libs/3rdparty/`)
- `platforms/posix/`
- `platforms/stm32/`
- `executables/`
- `test/pyTest/`

This rule yields exactly 1,854 of the 3,556 changed paths.

### Drift checkout recreation

The drift input is a clean, gitignored bare clone of the public upstream
Eclipse openBSW repository placed under `target/drift/` (repo-relative). It is
read-only input; recreating it is a plain `git clone --bare` of the public
upstream into that ignored location followed by resolving the two commits
above. The pinned read-only oracle checkout is not touched by this procedure.

## Method and honesty note

Inputs: the per-commit name-status log, the exhaustive
baseline-to-tip path diff, and an exclusive per-commit area map (each of the
166 commits assigned to exactly one of the eight delta areas from
`docs/port/upstream-drift-2026-07-18.md`, plus `other`). Classification was
done per path group (module directory plus change kind), driven by which
commits touched each path:

- Full diffs were read for the significant behavioral candidates: the UDS
  service additions and configuration rework, the tester-address-translation
  transport/TP-router change, representative DoIP estd-to-ETL migration
  commits, the STM32 CAN driver/transceiver additions, the storage
  (EEPROM-size) change, the POSIX stdin/UART/Tap/SocketCAN fixes, the
  lwIP-netif lifecycle change, the util `StringBufferOutputStream` fix, and
  representative middleware-framework commits.
- The bulk sweeps were verified by diff shape, not by reading every hunk: the
  automated Eclipse copyright-header commit `c21dcc3a` (1,743 files; spot
  checks confirm license-header-block rewrites only), the clang-tidy
  mechanical sweeps, the Bazel BUILD roll-out commits, and the treefmt /
  formatting commits. Their classification is subject-plus-diff-stat based
  with per-group spot checks, and is recorded as such.
- Tie-break when several commits touched one path: a testable behavioral
  change dominates mechanical edits; a re-pin-only change dominates
  mechanical edits; paths deleted (or moved away) by the drift are classified
  as no-behavioral-effect because no artifact exists at the tip to test.
- Build/doc/config file kinds (`CMakeLists.txt`, `BUILD.bazel`, `module.spec`,
  `*.rst`, docs, lockfiles, profiles) are classified no-behavioral-effect
  regardless of module, including inside new modules.

## Area survey (all 166 commits, each in exactly one section)

Per-area commit counts:

| Area | Commits |
|---|---:|
| build/toolchain | 46 |
| UDS/diagnostics | 6 |
| DoCAN/ISO-TP | 1 |
| DoIP | 18 |
| CAN/STM32 | 9 |
| reference application | 10 |
| async/executors | 4 |
| storage/safety | 1 |
| other | 71 |
| total | 166 |

### 1. build/toolchain (46 commits)

Main modules touched: repo-wide build infrastructure; `BUILD.bazel` files
across `libs/bsw/`, `libs/bsp/`, `executables/`, `platforms/`; CI workflows;
docker; formatting and license tooling.

What changed at module level: a Bazel build-system roll-out in parallel to
CMake (BUILD files, lockfile, buildifier, toolchain configs, an
arm-none-eabi GCC Bazel toolchain proof of concept); treefmt/copyright
tooling with the automated Eclipse license-header sweep `c21dcc3a` touching
1,743 files (header blocks only); clang-tidy CI gating and scope tuning;
docker and venv infrastructure; toolchain floors raised (default Rust
1.96.0, project minimum C++17); ETL NOTICE updates for 20.47.1/20.48.1;
cmake-format and linker-script formatting; `.gitattributes`/`.gitignore`
hygiene. None of these change runtime source semantics on the mandatory
surface.

| Commit | Date | Subject |
|---|---|---|
| `49933365` | 2026-03-11 | Fix CT-P1 clang-tidy violations in cpp2ethernet production paths |
| `e0cb9558` | 2026-03-13 | Limit clang-tidy CI to changed files on branch workflows (#381) |
| `58ef9c2e` | 2026-04-01 | Bazel gcc arm_none_eabi toolchain POC |
| `e45eee34` | 2026-04-06 | Upgrade project minimum supported version to C++17 |
| `823fc846` | 2026-04-07 | Update NOTICE for new ETL Version 20.47.1 |
| `4f3a4bf9` | 2026-04-09 | Fix clang-tidy CI scope |
| `026a7630` | 2026-04-10 | Adjust clang-tidy workflow scope |
| `5f33d338` | 2026-04-10 | Format the linker script |
| `1844a9d4` | 2026-04-13 | Use ETL formatting for example and logger time |
| `76045328` | 2026-04-21 | Fix s32k clang-tidy build graph issues |
| `339f0dab` | 2026-05-01 | Apply cmake-format to s32k1xx bspMcu CMakeLists |
| `d0bbffaf` | 2026-05-07 | Implement reviewer suggestions |
| `5df5c25b` | 2026-05-08 | Remove artifact analysis documentation and results |
| `c45100ba` | 2026-05-08 | Add BUILD file to etl's rim ignore list |
| `c73614fc` | 2026-05-08 | Create cache dir with correct write permissions |
| `b3cd5e77` | 2026-05-19 | Add Bazel CI workflow and buildifier formatting |
| `0f3bf40c` | 2026-05-20 | Adjust etl's ignored rim files after renaming BUILD to BUILD.bazel |
| `0fac881c` | 2026-05-20 | Apply Bazel formatting fixes |
| `a3749e83` | 2026-05-20 | Fix docker bind mount permission problem |
| `d611c0f3` | 2026-05-20 | Regenerate Bazel lockfile |
| `68aa1fd1` | 2026-05-22 | Add BUILD.bazel for libs/bsw targets |
| `0afc5daa` | 2026-05-25 | Add RIM metadata for CMSIS |
| `b4b49806` | 2026-05-26 | Remove additional context messages from erros in bazel.yml |
| `cb12f1d3` | 2026-05-26 | Add buildifier to treefmt setup |
| `02b9a4b1` | 2026-05-27 | Add copyright checker documentation and treefmt integration |
| `6e65ea94` | 2026-05-27 | Add cr_checker copyright tooling and CI workflow |
| `9f61d48b` | 2026-05-27 | Add rustfmt to treefmt config |
| `b352492a` | 2026-05-27 | Fix formatting: remove double blank lines and preserve cmake copyright headers |
| `c21dcc3a` | 2026-05-27 | Apply Eclipse copyright headers to all source files (automated) |
| `f240b04f` | 2026-05-27 | Add /target/ to .gitignore |
| `232c1bba` | 2026-06-01 | Make cargo and rustup homes accessible to the runtime UID |
| `c2f709ba` | 2026-06-01 | Add a .gitattributes file |
| `8bcfb3dc` | 2026-06-02 | bump default Rust version to 1.96.0 |
| `2371385d` | 2026-06-03 | Reintroduce clang-tidy gating and default context |
| `a32e0f0e` | 2026-06-04 | Add Bazel BUILD files for common, bsp, and configuration targets |
| `b32ed8e5` | 2026-06-11 | Fix treefmt copyright config: add missing extensions and use current year |
| `dfd96950` | 2026-06-11 | Default docker compose proxy vars to empty to silence warnings |
| `e39372a3` | 2026-06-16 | Add BUILD files for printf and bsw libraries |
| `425b1f5a` | 2026-06-18 | Replicate configuration, common CMakeLists.txt to BUILD.bazel |
| `adc0309a` | 2026-06-18 | Fix formatting |
| `1211c9c8` | 2026-06-19 | Update formatting guidelines for pragma once and copyright header |
| `b703c4a5` | 2026-06-19 | Use minimal cache mode for docker image |
| `5cb140d3` | 2026-06-23 | Add printf BUILD file to ignored rim files. |
| `c4401a13` | 2026-07-03 | Move generated middleware code to build-time generation |
| `24393a44` | 2026-07-09 | Add automatic venv setup and wrapper scripts for blob regeneration |
| `17a5d461` | 2026-07-11 | Update NOTICE.md for ETL 20.48.1 |

### 2. UDS/diagnostics (6 commits)

Main modules touched: `libs/bsw/uds`, `executables/referenceApp/udsConfiguration`,
`executables/referenceApp/application` (UdsSystem), `test/pyTest/uds`.

What changed at module level: two new UDS services were added with unit
tests and DemoSystem/UdsSystem registration - ClearDiagnosticInformation
(0x14, `9558c245`) and ReadDTCInformation (0x19, `f8132091`). `39212d01`
moved fixed diagnostic constants (session timings, reset behavior IDs) out of
`DiagCodes.h` into user-overridable `UdsConfig.h` headers (reference app,
unit tests, test mock), changing how ECU-reset and session-control services
are parameterized. `4beb3f8b` only adds log output when diagnostic jobs are
added. `57d27137` is Bazel BUILD files only; `b3fb00f0` reworks the pyTest
udstool harness (`run_process()`), test infrastructure only.

| Commit | Date | Subject |
|---|---|---|
| `4beb3f8b` | 2026-03-16 | Enhance log when adding diagnostic jobs |
| `39212d01` | 2026-05-05 | User specific configuration of module uds |
| `b3fb00f0` | 2026-05-27 | Update run_process() implementation for udstool based tests |
| `f8132091` | 2026-06-12 | feat(uds): add ReadDTCInformation service (0x19) |
| `9558c245` | 2026-06-15 | Add UDS service ClearDiagnosticInformation (0x14) |
| `57d27137` | 2026-06-23 | Migrate transport, transportRouterSimple and uds to Bazel |

### 3. DoCAN/ISO-TP (1 commit)

Main modules touched: `libs/bsw/docan` (BUILD file only).

What changed at module level: the single DoCAN-area commit adds
`BUILD.bazel` files for docan, doip, loggerIntegration, storage and the
etlImpl platforms. There is **no** C++ source change to `libs/bsw/docan` in
the whole 166-commit window other than the automated copyright-header sweep;
the DoCAN/ISO-TP protocol surface at the tip is source-identical to the
pinned baseline modulo license headers.

| Commit | Date | Subject |
|---|---|---|
| `c9ec30bc` | 2026-06-17 | Migration of bsw libs docan, doip, logger_integration |

### 4. DoIP (18 commits)

Main modules touched: `libs/bsw/doip` (server sources and tests), with
spillover into `libs/bsw/cpp2ethernet` tests.

What changed at module level: a systematic estd-to-ETL container migration
across the DoIP server: `estd::slice` to `etl::span`, `estd::vector`,
`object_pool`, `bitset`, `array`, `optional`, `variant`, `forward_list`,
`ordered_map` (to `etl::imap`), `function`/`by_ref` (to `etl::delegate`),
big-endian helpers, memory functions, `std`-to-ETL algorithm and assert
switches, plus partial test migration. C++ API signatures change throughout,
but read diffs (e.g. the OEM message-handler lookup in
`DoIpServerVehicleIdentification`) show the protocol logic is preserved
one-to-one; no DoIP wire-behavior change was found in the read samples. No
new DoIP protocol features or vectors were added in the window.

| Commit | Date | Subject |
|---|---|---|
| `8d18b1a1` | 2026-01-20 | Change estd::slice to etl::span in DoIP |
| `0dd375fd` | 2026-01-22 | Switch bitset from estd to ETL in doip server |
| `118ea4f2` | 2026-01-22 | Switch memory fns from estd to ETL in doip server |
| `bc20e058` | 2026-01-22 | Switch object pool from estd to ETL in doip server |
| `895adbe9` | 2026-01-26 | Switch vector from estd to ETL in doip server |
| `2cd7e37e` | 2026-01-28 | Switch array from estd to ETL in doip server |
| `76b31e1d` | 2026-01-28 | Switch by_ref from estd to ETL in doip server |
| `a025a75c` | 2026-01-28 | Switch optional from estd to ETL in doip server |
| `b99b1de6` | 2026-01-28 | Switch lists from estd to ETL in doip server |
| `f6049da5` | 2026-01-28 | Switch variant from estd to ETL in doip server |
| `1719a648` | 2026-01-29 | Switch assert from std to ETL in doip server |
| `29ba5c06` | 2026-01-29 | Switch be-helpers from estd to ETL in doip server |
| `721cbb6b` | 2026-01-29 | Switch constructor from estd to ETL in doip server |
| `a510e3b6` | 2026-01-29 | Switch function from estd to ETL in doip server |
| `a67de092` | 2026-01-29 | Switch algorithm from std to ETL in doip server |
| `acc49143` | 2026-01-29 | Switch ordered_map from estd to ETL in doip server |
| `6c3dc32b` | 2026-04-07 | Partially migrate Tests from estd::slice to etl::span |
| `baa9589d` | 2026-04-17 | Switch remaining estd usages in DoIP/ETH to ETL |

### 5. CAN/STM32 (9 commits)

Main modules touched: new `platforms/stm32/` platform tree
(`bsp/bspCan`, `bsp/bxCanTransceiver`, `bsp/fdCanTransceiver`, `bsp/bspClock`,
`bsp/bspIo`, `bsp/bspUart`, `bsp/bspTimer`, `bsp/bspInterruptsImpl`,
`etlImpl`, device headers, cmake), plus one fix in
`platforms/posix/bsp/socketCanTransceiver`.

What changed at module level: upstream added an STM32 platform from scratch:
MCU foundation and CMake scaffolding (`b581e80a`), device headers via RIM
(`56af88ec`), BSP peripheral drivers for clock (F4/G4), GPIO, UART, timer and
interrupts (`cf3832d5`), then CAN support - `BxCanDevice`/`FdCanDevice`
drivers with unit tests plus a bxCAN transceiver adapter (`1a11d135`) and an
FDCAN transceiver adapter with tests (`be0029bb`), followed by unit-test CI
(`2f3e19e0`), 64-bit host test fixes (`f2f01102`) and an ETL clock fix after
a SystemTimer API removal (`830b9c5e`). Separately, `6cafa673` fixes the
POSIX SocketCAN transceiver so logging works for extended CAN IDs. The STM32
CAN drivers/transceivers are directly comparable with this port's native
bxCAN/FDCAN drivers (health, overflow, bus-off contracts).

| Commit | Date | Subject |
|---|---|---|
| `56af88ec` | 2026-06-01 | Add STM32 device headers via RIM |
| `b581e80a` | 2026-06-01 | Add STM32 MCU foundation |
| `1a11d135` | 2026-06-02 | Add STM32 CAN drivers and bxCAN transceiver |
| `be0029bb` | 2026-06-02 | Add STM32 FDCAN transceiver adapter |
| `cf3832d5` | 2026-06-02 | Add STM32 BSP peripheral drivers |
| `6cafa673` | 2026-06-08 | Fix logger to work for extended canId |
| `2f3e19e0` | 2026-07-08 | Add STM32 unit-test builds to CI |
| `f2f01102` | 2026-07-16 | Fix STM32 CAN unit tests on 64-bit hosts |
| `830b9c5e` | 2026-07-17 | Fix removed SystemTimer calls in STM32 ETL clock implementation |

### 6. reference application (10 commits)

Main modules touched: `executables/referenceApp/application`,
`executables/referenceApp/transportConfiguration`,
`executables/referenceApp/platforms/s32k148evb`, linker scripts.

What changed at module level: behavioral items are the tester-address
translation added to the TP-Router (`120f5688`: `LogicalAddress` rework in
`libs/bsw/transport`, translation logic in `TransportRouterSimple`, new
router unit tests, reference-app transport configuration); keeping the lwIP
netif persistent across lifecycle transitions (`366d993e`,
`EthernetSystem.cpp`); and correcting the number of DoIP protocol send jobs
(`330514aa`, `DoIpServerSystem.h`). S32K148EVB board work (TJA1101 MDIO
timing/RMII `7a005d17`, ENET ISR race fix `1c1450ea`, linker-script memory
region work `63efd0d2`/`57fe872d`, SRAM_L file list `c7bc7573`) is
out-of-scope board code for this port. `7a3c3f3f` wires the new Foo
middleware demo service into DemoSystem (new-framework feature);
`8857f6b9` removes a Bazel headers target.

| Commit | Date | Subject |
|---|---|---|
| `366d993e` | 2026-01-30 | Keep lwIP netif persistent across lifecycle transitions |
| `1c1450ea` | 2026-03-09 | Fix ENET ISR race during Ethernet driver initialization |
| `330514aa` | 2026-03-17 | Correct the number of protocol send jobs |
| `57fe872d` | 2026-04-10 | Adjust RAM regions in linker script |
| `63efd0d2` | 2026-04-10 | Assign symbols to specific memory regions in ld script |
| `120f5688` | 2026-05-05 | Add tester address translation to referenceApp TP-Router |
| `c7bc7573` | 2026-06-15 | Define a single file list for hot SRAM_L |
| `8857f6b9` | 2026-06-25 | remove //executables/referenceApp/application:application_headers target |
| `7a3c3f3f` | 2026-07-03 | Integrate Foo middleware service into referenceApp DemoSystem |
| `7a005d17` | 2026-07-10 | Fix TJA1101 MDIO timing and configure RMII on S32K148EVB |

### 7. async/executors (4 commits)

Main modules touched: `libs/bsw/asyncFreeRtos`, `libs/bsw/asyncThreadX`,
`libs/bsw/asyncImpl`, `libs/os` FreeRTOS/ThreadX layers (Bazel files).

What changed at module level: all four commits migrate the FreeRTOS core,
ports, BSP/MCU layers, the async/runtime FreeRTOS binding, and the ThreadX
RTOS to the Bazel build system, including a `select`-based
`async_core_configuration`. These add BUILD files; no scheduling-contract or
executor source semantics change on the mandatory surface in this area.

| Commit | Date | Subject |
|---|---|---|
| `6c074682` | 2026-06-11 | Migrate async/runtime FreeRTOS binding to Bazel |
| `87b8ad1d` | 2026-06-11 | Migrate FreeRTOS core, ports and BSP/MCU layers to Bazel |
| `a63bab57` | 2026-06-18 | Use select for async_core_configuration |
| `9e4fbd4b` | 2026-06-25 | Migrate ThreadX RTOS to Bazel build system |

### 8. storage/safety (1 commit)

Main modules touched: `platforms/posix/bsp/bspEepromDriver`,
`executables/*/bsp*Configuration` EEPROM configuration headers.

What changed at module level: `1578c18f` makes the emulated EEPROM size for
POSIX builds configurable via `EepromConfiguration.h` instead of a hardcoded
driver constant. Default-build behavior is unchanged; the configurability
itself is a new post-baseline feature. No safety-module
(`libs/safety/*`) source change exists in the window beyond the automated
copyright-header sweep.

| Commit | Date | Subject |
|---|---|---|
| `1578c18f` | 2026-05-20 | Make EEPROM size for Posix builds configurable |

### 9. other (71 commits)

Main modules touched: `libs/bsw/middleware` (new service framework),
`libs/bsw/routing` and `libs/bsw/blob` (new modules), `libs/bsw/estd` and
`libs/bsw/util` (removals in favor of ETL), `libs/bsw/time` (new
TimestampProvider), `libs/bsw/logger/rust`, `libs/rust/*`,
`executables/referenceApp/rustHelloWorld` (upstream Rust experiments),
`platforms/posix/bsp` (fixes), plus repo-wide clang-tidy sweeps.

What changed at module level, by subgroup:

- **New service-middleware framework** (proxy/skeleton/cluster model, queues,
  lock strategies, allocator generation, jinja2cpp bindings, POSIX
  shared-memory simulation, Foo demo service, pytest integration, and the
  Core-to-Cluster rename): `6dc89c53`, `fdf7872d`, `763e7f27`, `04096c32`,
  `0514bedc`, `179c96bc`, `d4517940`, `b6633ce1`, `4daedef7`, `34fbf932`,
  `96ba360a`, `52c9252d`, `fd6b9573`, `73207573`, `67a454e9`, `b02a793b`,
  `db073661`, `00a8a021`, `ca1b283c`, `1b2f8d0e`, `dcfc215f`, `5928fc47`.
  Almost the entire upstream middleware tree is new relative to the pinned
  baseline (119 of the 130 changed middleware paths did not exist at the
  baseline).
- **New routing/blob modules and their reference-app integration**:
  `0d42afeb`, `175f6f28`.
- **estd/util removal in favor of ETL** (containers deleted from
  `libs/bsw/estd`, CRC and spsc queue removed from `libs/bsw/util`,
  usages migrated): `eb956887`, `c1d18f8d`, `db24855a`, `5c0dedaf`,
  `4980b7c3`, plus ETL version-bump adaptations `23d0da68`, `0aa77ad5`,
  `8e4ba7d5`, `d7bf47c3`, `b3ab7a80`.
- **Upstream Rust experiments** (console_out and panic-handler crates, Rust
  logging bridge onto the C++ logger, vendored crates): `b574834a`,
  `da6b3285`, `c9850632`, `1cf01d02`.
- **POSIX/platform fixes, transport-config refactors and Bazel spillover**:
  `bd0078d0`, `f381dd46` (stdin/UART read-error handling), `490aff02`
  (64-bit microsecond timestamp), `b119bf9b` (Tap restart across
  lifecycle), `9d3e89a2` (new TimestampProvider module plus SystemTimer API
  rework), `b9f738bf` (moves the previously unguarded ETH_1 reference in
  `TransportRouterSimple` behind the transport configuration; no semantic
  change for existing configurations), `6a70d48c`, `262f8d23`, `489448c1`,
  `7423fab8`, `3597e8d6`, `415a05cf` (lwIP Bazel BUILD files).
- **clang-tidy/mechanical sweeps and small cleanups**:
  `c2e895c2`, `93770a4f`, `4d145b41`, `aa9eb3d5`, `cfc0f2e5`, `52314fe6`,
  `6ed4d9fd`, `9ff800c4`, `1c0ef628`, `399ed9b2`, `07b75511`, `8f31f107`
  (the last two are a behavioral `StringBufferOutputStream`
  finalization/destructor fix in `libs/bsw/util`, with tests), `bcb862fb`,
  `9bc381ae`, `ca5ecbb3`, `dc02abe8`, `135d04c1`, `5bf14407`, `4246a530`,
  `a4c01b1c`, `67e94d82`.

| Commit | Date | Subject |
|---|---|---|
| `b574834a` | 2026-02-25 | Add console_out Rust crate |
| `da6b3285` | 2026-02-25 | Add openbsw_panic_handler crate |
| `b119bf9b` | 2026-03-05 | Fix Tap interface restart failure during lifecycle transition on POSIX |
| `4246a530` | 2026-03-06 | docs: Add note about known Ethernet issue |
| `c2e895c2` | 2026-03-12 | Fix logger CT-P1 vararg warnings |
| `fdf7872d` | 2026-03-16 | Add middleware proxy, skeleton and cluster connections |
| `1c0ef628` | 2026-03-19 | Remove unreachable code after return |
| `399ed9b2` | 2026-03-19 | Remove duplicated call to setDataListener |
| `6dc89c53` | 2026-03-24 | Port middleware modules |
| `c9850632` | 2026-04-01 | Add Rust logging bridge to the C++ logger |
| `8e4ba7d5` | 2026-04-07 | Update to ETL 20.47.1 in OpenBSW |
| `8f31f107` | 2026-04-09 | Fix StringBufferOutputStream destructor |
| `07b75511` | 2026-04-10 | Finalize StringBufferOutputStream safely |
| `52314fe6` | 2026-04-10 | Enable cppcoreguidelines-pro-type-reinterpret-cast |
| `aa9eb3d5` | 2026-04-10 | Address vararg review feedback |
| `cfc0f2e5` | 2026-04-10 | Enable cppcoreguidelines-pro-type-vararg |
| `4980b7c3` | 2026-04-13 | Convert VariantQueueExample to ETL print |
| `4d145b41` | 2026-04-14 | Fix cert-dcl21-cpp and missing vararg NOLINT |
| `93770a4f` | 2026-04-14 | Suppress cppcoreguidelines-pro-type-vararg in platform files |
| `f381dd46` | 2026-04-14 | Address CodeRabbit review: fix both fcntl calls in Uart.cpp |
| `6ed4d9fd` | 2026-04-15 | Fix cppcoreguidelines-pro-type-reinterpret-cast violations in s32k1xx BSP |
| `5928fc47` | 2026-04-16 | Use nested namespaces in middleware |
| `9ff800c4` | 2026-04-17 | Use if constexpr instead of sfinae where possible |
| `eb956887` | 2026-04-17 | Remove estd containers |
| `67e94d82` | 2026-04-22 | Extract shared CMSIS to libs/3rdparty/cmsis |
| `6a70d48c` | 2026-04-23 | Unify TransportConfiguration for unit tests |
| `a4c01b1c` | 2026-05-08 | Fix link to DoxygenMainPage.md |
| `dc02abe8` | 2026-05-08 | Minor improvements |
| `34fbf932` | 2026-05-18 | Fix proxy dispatch missing error |
| `489448c1` | 2026-05-18 | Application dependency on Uart.h instead of UartConfig.h |
| `52c9252d` | 2026-05-18 | Move AllocatorBase to memory |
| `67a454e9` | 2026-05-18 | Remove maxServiceId check on DB Manipualtor |
| `73207573` | 2026-05-18 | Remote Shm interface and rename shm/Config.h |
| `96ba360a` | 2026-05-18 | Use ETL_ERROR_GENERIC in Queue.h |
| `fd6b9573` | 2026-05-18 | Remove memory interface |
| `ca1b283c` | 2026-05-25 | Decouple lock types from LockStrategies header |
| `bcb862fb` | 2026-05-26 | Replace long srcs list with glob |
| `00a8a021` | 2026-05-27 | Remove suspendAllInterrupts from middleware |
| `bd0078d0` | 2026-05-27 | Fix posix stdin read error handling |
| `1cf01d02` | 2026-06-01 | Vendor Rust dependencies for offline builds |
| `1b2f8d0e` | 2026-06-09 | Fix incorrect ETL_ASSERT_FAIL argument types |
| `490aff02` | 2026-06-11 | Add 64-bit microsecond timestamp function for posix |
| `b02a793b` | 2026-06-12 | Apply RAII idiom to Middleware Aggregator constructor |
| `3597e8d6` | 2026-06-16 | Remove redundant commonImpl target from configuration |
| `9bc381ae` | 2026-06-17 | Move deps to implementation_deps |
| `ca5ecbb3` | 2026-06-17 | Implement reviewer suggestions |
| `135d04c1` | 2026-06-18 | Let FutureDispatcherTestSuite pass instead of skip |
| `5bf14407` | 2026-06-18 | Update rule FILE-000 to use <> for standard headers only |
| `415a05cf` | 2026-06-19 | Migrate lwip |
| `7423fab8` | 2026-06-19 | Break a circular dependency common<->configuration |
| `db073661` | 2026-06-22 | Initialize queues in constructor |
| `0d42afeb` | 2026-06-24 | Add routing and blob |
| `5c0dedaf` | 2026-06-29 | Remove dead code: spsc/ReadWrite.h |
| `9d3e89a2` | 2026-06-29 | Add TimestampProvider |
| `db24855a` | 2026-06-29 | Replace spsc/Queue.h with etl/queue_spsc_atomic.h |
| `0aa77ad5` | 2026-06-30 | Update to ETL 20.48.0 in OpenBSW |
| `179c96bc` | 2026-06-30 | Return Middleware payload by const reference |
| `b3ab7a80` | 2026-06-30 | Adjust to new ETL interfaces |
| `c1d18f8d` | 2026-07-01 | Replace local CRC implementation with using ETL |
| `4daedef7` | 2026-07-03 | Add jinja2cpp code generator for middleware service bindings |
| `b6633ce1` | 2026-07-03 | Add POSIX shared memory simulation for middleware services |
| `d4517940` | 2026-07-03 | Add pytest integration tests for middleware Foo service |
| `04096c32` | 2026-07-07 | Fix MessagePayloadBuilder routing for non-trivially-copyable types |
| `0514bedc` | 2026-07-07 | Use fully qualified ::etl namespace throughout middleware |
| `262f8d23` | 2026-07-08 | Fix race condition in parallel test by exception for file access |
| `763e7f27` | 2026-07-08 | Replace static Allocator.h with generated type from deployment model |
| `175f6f28` | 2026-07-09 | Improve documentation and config file generation |
| `b9f738bf` | 2026-07-09 | Fix unguarded ETH_1 reference in transportRouterSimple |
| `23d0da68` | 2026-07-11 | Update to ETL 20.48.1 |
| `dcfc215f` | 2026-07-13 | Rename middleware clusters from Core0/Core1 to Cluster0/Cluster1 |
| `d7bf47c3` | 2026-07-14 | Adjust to new ETL interface |

## Mandatory-surface path classification (all 1,854 paths)

Every one of the 1,854 mandatory-surface paths is assigned to exactly one of
three buckets. Totals:

| Bucket | Paths |
|---|---:|
| A - no behavioral effect | 1525 |
| B - behavioral effect, testable now | 55 |
| C - behavioral effect, requires re-pin | 274 |
| **Total** | **1854** |

Bucket B by follow-up package:

| Package | Paths |
|---|---:|
| U02 (UDS/DoCAN/DoIP vectors) | 31 |
| U03 (STM32/posix CAN) | 16 |
| U04 (middleware/reference app/platform) | 8 |

### Bucket B - behavioral effect testable now (55 paths)

These carry behavior changes demonstrable as a test or vector against the
current Rust implementation without moving the oracle: the new/reworked UDS
services and configuration, tester-address translation in the transport
router, the STM32 CAN drivers and
transceiver adapters (comparable against the native Rust bxCAN/FDCAN
contracts), the POSIX SocketCAN extended-ID fix, POSIX stdio/UART error
handling, the Tap/netif lifecycle behavior, and the util
`StringBufferOutputStream` fix. Import/differential-test targets: UDS, DoCAN
and DoIP vectors in U02; STM32/posix CAN comparisons in U03;
middleware/reference-app and platform behavior in U04.

- `executables/referenceApp/application/include/systems/UdsSystem.h` (U02; commits `9558c245`, `f8132091`)
- `executables/referenceApp/application/src/systems/EthernetSystem.cpp` (U04; commits `366d993e`)
- `executables/referenceApp/application/src/systems/UdsSystem.cpp` (U02; commits `9558c245`, `f8132091`)
- `executables/referenceApp/platforms/posix/main/src/systems/TapEthernetSystem.cpp` (U04; commits `b119bf9b`)
- `executables/referenceApp/transportConfiguration/include/transport/TransportConfiguration.h` (U02; commits `120f5688`)
- `executables/referenceApp/transportConfiguration/src/transport/TransportConfiguration.cpp` (U02; commits `120f5688`)
- `executables/referenceApp/udsConfiguration/include/uds/UdsConfig.h` (U02; commits `39212d01`)
- `executables/unitTest/configuration/common/include/busid/BusId.h` (U02; commits `120f5688`)
- `executables/unitTest/configuration/common/src/busid/BusId.cpp` (U02; commits `120f5688`)
- `executables/unitTest/transportConfiguration/include/transport/TransportConfiguration.h` (U02; commits `120f5688`)
- `executables/unitTest/udsConfiguration/include/uds/UdsConfig.h` (U02; commits `39212d01`)
- `libs/bsw/transport/include/transport/LogicalAddress.h` (U02; commits `120f5688`)
- `libs/bsw/transport/src/LogicalAddress.cpp` (U02; commits `120f5688`)
- `libs/bsw/transport/test/src/TesterAddressTest.cpp` (U02; commits `120f5688`)
- `libs/bsw/transportRouterSimple/src/transport/routing/TransportRouterSimple.cpp` (U02; commits `120f5688`)
- `libs/bsw/transportRouterSimple/test/src/TransportRouterSimpleTest.cpp` (U02; commits `120f5688`)
- `libs/bsw/uds/include/uds/DiagCodes.h` (U02; commits `39212d01`)
- `libs/bsw/uds/include/uds/services/cleardiagnosticinformation/ClearDiagnosticInformation.h` (U02; commits `9558c245`)
- `libs/bsw/uds/include/uds/services/ecureset/EnableRapidPowerShutdown.h` (U02; commits `39212d01`)
- `libs/bsw/uds/include/uds/services/ecureset/HardReset.h` (U02; commits `39212d01`)
- `libs/bsw/uds/include/uds/services/ecureset/SoftReset.h` (U02; commits `39212d01`)
- `libs/bsw/uds/include/uds/services/readdtcinformation/ReadDTCInformation.h` (U02; commits `f8132091`)
- `libs/bsw/uds/include/uds/services/sessioncontrol/DiagnosticSessionControl.h` (U02; commits `39212d01`)
- `libs/bsw/uds/src/uds/services/cleardiagnosticinformation/ClearDiagnosticInformation.cpp` (U02; commits `9558c245`)
- `libs/bsw/uds/src/uds/services/ecureset/EnableRapidPowerShutdown.cpp` (U02; commits `39212d01`)
- `libs/bsw/uds/src/uds/services/ecureset/HardReset.cpp` (U02; commits `39212d01`)
- `libs/bsw/uds/src/uds/services/ecureset/SoftReset.cpp` (U02; commits `39212d01`)
- `libs/bsw/uds/src/uds/services/readdtcinformation/ReadDTCInformation.cpp` (U02; commits `f8132091`)
- `libs/bsw/uds/src/uds/services/sessioncontrol/DiagnosticSessionControl.cpp` (U02; commits `39212d01`)
- `libs/bsw/uds/test/mock/include/uds/UdsConfig.h` (U02; commits `39212d01`)
- `libs/bsw/uds/test/src/uds/services/cleardiagnosticinformation/ClearDiagnosticInformationTest.cpp` (U02; commits `9558c245`)
- `libs/bsw/uds/test/src/uds/services/readdtcinformation/ReadDTCInformationTest.cpp` (U02; commits `f8132091`)
- `libs/bsw/uds/test/src/uds/services/sessioncontrol/DiagnosticSessionControlTest.cpp` (U02; commits `39212d01`)
- `libs/bsw/util/include/util/stream/StringBufferOutputStream.h` (U04; commits `07b75511`, `8f31f107`)
- `libs/bsw/util/src/util/stream/StringBufferOutputStream.cpp` (U04; commits `07b75511`, `8f31f107`)
- `libs/bsw/util/test/src/util/stream/StringBufferOutputStreamTest.cpp` (U04; commits `07b75511`, `8f31f107`)
- `platforms/posix/bsp/bspStdio/src/bsp/stdIo/stdIo.cpp` (U04; commits `bd0078d0`)
- `platforms/posix/bsp/bspUart/src/bsp/Uart.cpp` (U04; commits `bd0078d0`, `f381dd46`)
- `platforms/posix/bsp/socketCanTransceiver/src/can/SocketCanTransceiver.cpp` (U03; commits `6cafa673`)
- `platforms/posix/bsp/tapEthernetDriver/src/TapEthernetDriver.cpp` (U04; commits `b119bf9b`)
- `platforms/stm32/bsp/bspCan/include/can/BxCanDevice.h` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bspCan/include/can/FdCanDevice.h` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bspCan/src/can/BxCanDevice.cpp` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bspCan/src/can/FdCanDevice.cpp` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bspCan/test/include/mcu/mcu.h` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bspCan/test/src/can/BxCanDeviceTest.cpp` (U03; commits `1a11d135`, `f2f01102`)
- `platforms/stm32/bsp/bspCan/test/src/can/FdCanDeviceTest.cpp` (U03; commits `1a11d135`, `f2f01102`)
- `platforms/stm32/bsp/bxCanTransceiver/include/can/transceiver/bxcan/BxCanTransceiver.h` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bxCanTransceiver/src/can/transceiver/bxcan/BxCanTransceiver.cpp` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bxCanTransceiver/test/mock/include/can/BxCanDevice.h` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/bxCanTransceiver/test/src/can/BxCanTransceiverTest.cpp` (U03; commits `1a11d135`)
- `platforms/stm32/bsp/fdCanTransceiver/include/can/transceiver/fdcan/FdCanTransceiver.h` (U03; commits `be0029bb`)
- `platforms/stm32/bsp/fdCanTransceiver/src/can/transceiver/fdcan/FdCanTransceiver.cpp` (U03; commits `be0029bb`)
- `platforms/stm32/bsp/fdCanTransceiver/test/mock/include/can/FdCanDevice.h` (U03; commits `be0029bb`)
- `platforms/stm32/bsp/fdCanTransceiver/test/src/can/FdCanTransceiverTest.cpp` (U03; commits `be0029bb`)

### Bucket C - behavioral effect requiring re-pin (274 paths)

New modules or features the port does not implement, out-of-scope platform
code, or changes only meaningful against the post-drift tree: the new
service-middleware framework, the new routing and blob modules and their
configuration/generation, the new STM32 platform tree outside the CAN
surface, the TimestampProvider/SystemTimer API rework, upstream's own Rust
experiments (logger bridge, crates, rustHelloWorld), S32K148EVB board
changes, the EEPROM-size configurability feature, and the DoIP
protocol-send-job pool sizing in the reference-app DoIP frontend
(`330514aa`, reclassified from bucket B on 2026-07-19; see changelog).
These are input to a deliberate future re-pin decision (U06); none blocks
the pinned baseline.

- `executables/referenceApp/application/include/logger/` (1): logger.h
- `executables/referenceApp/application/include/middleware/` (2): FooProxyWrapper.h, FooSkeletonWrapper.h
- `executables/referenceApp/application/include/systems/` (3): DemoSystem.h, DoIpServerSystem.h, RoutingSystem.h
- `executables/referenceApp/application/src/app/` (1): app.cpp
- `executables/referenceApp/application/src/logger/` (1): logger.cpp
- `executables/referenceApp/application/src/systems/` (3): DemoSystem.cpp, DoCanSystem.cpp, RoutingSystem.cpp
- `executables/referenceApp/configuration/` (1): routing.jsonl
- `executables/referenceApp/configuration/include/blob/` (2): ConfigType.h, configuration.h
- `executables/referenceApp/configuration/include/routing/` (2): channelId.h, constants.h
- `executables/referenceApp/middlewareConfiguration/include/` (1): MemoryLayout.h
- `executables/referenceApp/middlewareConfiguration/model/` (1): deployment.yaml
- `executables/referenceApp/middlewareConfiguration/platform_integration/concurrency/include/middleware/concurrency/` (1): lock_types.h
- `executables/referenceApp/middlewareConfiguration/platform_integration/logger/src/` (1): LoggerImpl.cpp
- `executables/referenceApp/middlewareConfiguration/platform_integration/os/src/` (1): OsDefinitions.cpp
- `executables/referenceApp/middlewareConfiguration/platform_integration/time/src/` (1): SystemTimeProvider.cpp
- `executables/referenceApp/middlewareConfiguration/src/` (1): MemoryLayout.cpp
- `executables/referenceApp/platforms/posix/bspConfiguration/include/bsp/` (1): EepromConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/io/io/` (1): ioConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/io/output/` (2): outputConfiguration.h, outputConfigurationStrings.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/src/bsp/SystemTimer/` (1): SystemTimer.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/bsp/` (1): startUp.S
- `executables/referenceApp/platforms/s32k148evb/main/src/systems/` (1): S32K148EvbEthernetSystem.cpp
- `executables/referenceApp/rustHelloWorld/include/` (1): rust_hello_world.h
- `executables/referenceApp/rustHelloWorld/src/` (1): lib.rs
- `executables/unitTest/bsp/bspConfiguration/include/bsp/` (1): EepromConfiguration.h
- `libs/bsw/blob/include/blob/` (5): Blob.h, Config.h, Header.h, Logger.h, util.h
- `libs/bsw/blob/src/blob/` (5): Blob.cpp, Config.cpp, Header.cpp, Logger.cpp, util.cpp
- `libs/bsw/blob/test/` (1): routing.jsonl
- `libs/bsw/blob/test/include/blob/` (2): ConfigType.h, configuration.h
- `libs/bsw/blob/test/src/` (1): BlobTest.cpp
- `libs/bsw/bsp/include/bsp/timer/` (2): SystemTimer.h, isEqualAfterTimeout.h
- `libs/bsw/bsp/mock/include/bsp/timer/` (1): SystemTimerMock.h
- `libs/bsw/bsp/mock/src/bsp/timer/` (1): SystemTimerMock.cpp
- `libs/bsw/bsp/test/src/bsp/timer/` (1): IsEqualAfterTimeoutTest.cpp
- `libs/bsw/logger/rust/include/` (1): BswLogger.h
- `libs/bsw/logger/rust/src/` (6): bsw_logger.rs, component.rs, ffi.rs, lib.rs, log_buf.rs, macros.rs
- `libs/bsw/middleware/include/middleware/core/` (26): ClusterConnection.h, ClusterConnectionBase.h, DatabaseManipulator.h, EventSender.h, Future.h, FutureDispatcher.h, FutureDispatcherBase.h, IClusterConnection.h, IClusterConnectionConfigurationBase.h, ITimeoutHandler.h, InstancesDatabase.h, LoggerApi.h, Message.h, MessagePayloadBuilder.h, ProxyAttribute.h, ProxyAttributeBase.h, ProxyBase.h, ProxyEventBase.h, ResponseBuffer.h, ResponseBufferBase.h, SkeletonAttribute.h, SkeletonBase.h, SkeletonEvent.h, TransceiverBase.h, TransceiverContainer.h, types.h
- `libs/bsw/middleware/include/middleware/memory/` (6): Aggregator.h, AllocatorBase.h, AllocatorSelector.h, AllocatorTypes.h, Pool.h, PoolBase.h
- `libs/bsw/middleware/include/middleware/memory/impl/` (2): PoolIndexBySize.h, TupleSelectionSort.h
- `libs/bsw/middleware/include/middleware/queue/` (2): Queue.h, QueueBase.h
- `libs/bsw/middleware/include/middleware/shm/` (1): Initialization.h
- `libs/bsw/middleware/interfaces/include/middleware/concurrency/` (1): LockStrategies.h
- `libs/bsw/middleware/interfaces/include/middleware/logger/` (1): Logger.h
- `libs/bsw/middleware/interfaces/include/middleware/os/` (1): TaskIdProvider.h
- `libs/bsw/middleware/interfaces/include/middleware/time/` (1): SystemTimerProvider.h
- `libs/bsw/middleware/simulation/include/` (2): Logger.h, etl_profile.h
- `libs/bsw/middleware/simulation/include/foo/` (2): FooProxyWrapper.h, FooSkeletonWrapper.h
- `libs/bsw/middleware/simulation/model/` (1): deployment-test.yaml
- `libs/bsw/middleware/simulation/platform_integration/concurrency/include/middleware/concurrency/` (1): lock_types.h
- `libs/bsw/middleware/simulation/platform_integration/logger/src/` (1): LoggerImpl.cpp
- `libs/bsw/middleware/simulation/platform_integration/os/src/` (1): OsDefinitions.cpp
- `libs/bsw/middleware/simulation/platform_integration/time/src/` (1): SystemTimeProvider.cpp
- `libs/bsw/middleware/simulation/src/` (6): Logger.cpp, MainCore0.cpp, MainCore1.cpp, Sample.cpp, ShmWrapper.cpp, ShmWrapper.h
- `libs/bsw/middleware/src/middleware/core/` (11): ClusterConnection.cpp, ClusterConnectionBase.cpp, DatabaseManipulator.cpp, EventSender.cpp, FutureDispatcherBase.cpp, IClusterConnectionConfigurationBase.cpp, LoggerApi.cpp, MessagePayloadBuilder.cpp, ProxyBase.cpp, ResponseBufferBase.cpp, SkeletonBase.cpp
- `libs/bsw/middleware/src/middleware/memory/` (1): PoolBase.cpp
- `libs/bsw/middleware/test/include/middleware/concurrency/` (1): lock_types.h
- `libs/bsw/middleware/test/src/core/` (17): ClusterConfigurationTest.cpp, ConnectionTest.cpp, DbManipulatorTest.cpp, FutureDispatcherTest.cpp, InstancesDatabase.h, LoggerApi.cpp, MessagePayloadBuilderTest.cpp, MessageTest.cpp, Proxy.h, ProxyAttributesTest.cpp, ProxyAttributesUnderTest.h, ProxyBaseTest.cpp, ReferenceApp.h, ResponseBufferTest.cpp, Skeleton.h, SkeletonAttributeTest.cpp, SkeletonBaseTest.cpp
- `libs/bsw/middleware/test/src/core/mock/` (2): ClusterConnectionMock.h, ProxyMock.h
- `libs/bsw/middleware/test/src/logger/` (1): DslLogger.h
- `libs/bsw/middleware/test/src/logger/mock/` (2): LoggerMock.cpp, LoggerMock.h
- `libs/bsw/middleware/test/src/memory/` (3): AggregatorTest.cpp, AllocatorBaseTest.cpp, PoolsTest.cpp
- `libs/bsw/middleware/test/src/memory/mock/` (2): AllocatorMock.cpp, AllocatorMock.h
- `libs/bsw/middleware/test/src/os/` (1): OsDefinitions.cpp
- `libs/bsw/middleware/test/src/queue/` (1): QueueTest.cpp
- `libs/bsw/middleware/test/src/time/mock/` (2): SystemTimerProviderMock.cpp, SystemTimerProviderMock.h
- `libs/bsw/middleware/tools/cpp_generator/` (2): jinja2cpp.py, requirements.txt
- `libs/bsw/middleware/tools/cpp_generator/templates/jinja/` (11): allocator_selector_definitions.cpp.jinja, cluster_connections_srcCluster_to_tgtCluster.h.jinja, cluster_id.h.jinja, cluster_srcCluster.cpp.jinja, cluster_srcCluster.h.jinja, service_common.cpp.jinja, service_common.h.jinja, service_proxy.cpp.jinja, service_proxy.h.jinja, service_skeleton.cpp.jinja, service_skeleton.h.jinja
- `libs/bsw/middleware/tools/cpp_generator/templates/jinja/macros/` (1): transmission_mode.jinja
- `libs/bsw/middleware/tools/cpp_generator/templates/jinja/shm/` (4): allocators_definitions.h.jinja, config.cpp.jinja, config.h.jinja, queue_definitions.h.jinja
- `libs/bsw/middleware/tools/cpp_generator/templates/schemas/` (1): deployment.yaml
- `libs/bsw/routing/include/io/udp/` (2): Receiver.h, Sender.h
- `libs/bsw/routing/include/routing/` (20): ChannelNames.h, ClampedCounter.h, ErrorHandler.h, Header.h, Integration.h, LegacyRxAdapter.h, LegacyTxAdapter.h, Logger.h, PduRoutingTable.h, PduTransportBufferedWriter.h, PduTransportConfig.h, PduTransportIntegration.h, PduTransportRxAdapter.h, PduTransportTxAdapter.h, Router.h, RxAdapter.h, RxAdapterTable.h, TxAdapterTable.h, pduRouting.h, util.h
- `libs/bsw/routing/src/routing/` (12): ChannelNames.cpp, Header.cpp, LegacyTxAdapter.cpp, Logger.cpp, PduTransportBufferedWriter.cpp, PduTransportConfig.cpp, PduTransportTxAdapter.cpp, RxAdapter.cpp, RxAdapterTable.cpp, TxAdapterTable.cpp, pduRouting.cpp, util.cpp
- `libs/bsw/routing/test/` (1): routing.jsonl
- `libs/bsw/routing/test/include/blob/` (2): ConfigType.h, configuration.h
- `libs/bsw/routing/test/include/routing/` (3): channelId.h, constants.h, definition.h
- `libs/bsw/routing/test/src/io/udp/` (2): ReceiverTest.cpp, SenderTest.cpp
- `libs/bsw/routing/test/src/routing/` (17): ChannelNamesTest.cpp, ClampedCounterTest.cpp, HeaderTest.cpp, IntegrationTest.cpp, LegacyRxAdapterTest.cpp, LegacyTxAdapterTest.cpp, PduRoutingTableTest.cpp, PduTransportBufferedWriterTest.cpp, PduTransportConfigTest.cpp, PduTransportIntegrationTest.cpp, PduTransportRxAdapterTest.cpp, PduTransportTxAdapterTest.cpp, RouterTest.cpp, RxAdapterTableTest.cpp, RxAdapterTest.cpp, TxAdapterTableTest.cpp, definition.cpp
- `libs/bsw/time/include/time/` (1): TimestampProvider.h
- `libs/bsw/time/src/time/` (1): TimestampProvider.cpp
- `libs/bsw/time/test/src/time/` (1): TimestampProviderTest.cpp
- `libs/bsw/util/include/util/logger/` (1): Logger.h
- `libs/bsw/util/src/util/logger/` (1): Logger.cpp
- `libs/rust/console_out/src/` (1): lib.rs
- `libs/rust/panic_handler/src/` (1): lib.rs
- `platforms/posix/bsp/bspEepromDriver/include/eeprom/` (1): EepromDriver.h
- `platforms/posix/bsp/bspSystemTime/src/bsp/time/` (1): systemTimer.cpp
- `platforms/stm32/bsp/bspClock/include/clock/` (1): clockConfig.h
- `platforms/stm32/bsp/bspClock/src/` (2): clockConfig_f4.cpp, clockConfig_g4.cpp
- `platforms/stm32/bsp/bspInterruptsImpl/freertos/include/interrupts/` (1): suspendResumeAllInterrupts.h
- `platforms/stm32/bsp/bspInterruptsImpl/include/interrupts/` (1): disableEnableAllInterrupts.h
- `platforms/stm32/bsp/bspInterruptsImpl/src/` (1): interrupt_manager.c
- `platforms/stm32/bsp/bspInterruptsImpl/threadx/include/interrupts/` (1): suspendResumeAllInterrupts.h
- `platforms/stm32/bsp/bspIo/include/io/` (1): Gpio.h
- `platforms/stm32/bsp/bspIo/src/io/` (1): Gpio.cpp
- `platforms/stm32/bsp/bspMcu/include/3rdparty/st/` (1): LICENSE
- `platforms/stm32/bsp/bspMcu/include/3rdparty/st/stm32f4/` (3): stm32f413xx.h, stm32f4xx.h, system_stm32f4xx.h
- `platforms/stm32/bsp/bspMcu/include/3rdparty/st/stm32g4/` (3): stm32g474xx.h, stm32g4xx.h, system_stm32g4xx.h
- `platforms/stm32/bsp/bspMcu/include/mcu/` (1): mcu.h
- `platforms/stm32/bsp/bspMcu/include/reset/` (1): softwareSystemReset.h
- `platforms/stm32/bsp/bspMcu/src/reset/` (1): softwareSystemReset.cpp
- `platforms/stm32/bsp/bspMcu/startup/` (2): startup_stm32f413xx.s, startup_stm32g474xx.s
- `platforms/stm32/bsp/bspTimer/src/bsp/SystemTimer/` (1): SystemTimer.cpp
- `platforms/stm32/bsp/bspUart/include/bsp/` (2): Uart.h, UartParams.h
- `platforms/stm32/bsp/bspUart/src/` (1): Uart.cpp
- `platforms/stm32/etlImpl/src/` (2): clocks.cpp, print.cpp
- `test/pyTest/middleware/` (1): test_middleware.py

### Bucket A - no behavioral effect (1525 paths)

In gross terms, 1,157 of the 1,854 mandatory-surface paths were touched by
no commit other than the automated copyright-header sweep `c21dcc3a`. The
sub-groups below are disjoint (precedence deleted > build/doc > header-only >
mechanical), so build/doc files that also only saw the header sweep are
counted once under build/doc:

| Sub-group | Paths |
|---|---:|
| License-header-only change (automated sweep `c21dcc3a`) | 991 |
| Build/doc/config files (CMake, Bazel, module.spec, docs, profiles) | 315 |
| Deleted or moved away during the drift (nothing to test at tip) | 67 |
| Mechanical source edits (clang-tidy, formatting, ETL type swaps, dead-code removal, test-only refactors) | 152 |
| **Total** | **1525** |

Full enumeration, grouped by directory (directory + listed file name =
full path):

- `executables/referenceApp/` (1): CMakeLists.txt
- `executables/referenceApp/application/` (1): CMakeLists.txt
- `executables/referenceApp/application/doc/` (1): index.rst
- `executables/referenceApp/application/doc/systems/` (10): DemoSystem.rst, DoCanSystem.rst, DoIpServerSystem.rst, EthernetSystem.rst, RuntimeSystem.rst, SafetySystem.rst, StorageSystem.rst, SysAdminSystem.rst, TransportSystem.rst, UdsSystem.rst
- `executables/referenceApp/application/include/app/` (3): CanDemoListener.h, DemoLogger.h, app.h
- `executables/referenceApp/application/include/console/` (1): console.h
- `executables/referenceApp/application/include/systems/` (7): DoCanSystem.h, EthernetSystem.h, RuntimeSystem.h, SafetySystem.h, StorageSystem.h, SysAdminSystem.h, TransportSystem.h
- `executables/referenceApp/application/include/transport/` (1): ITransportSystem.h
- `executables/referenceApp/application/include/uds/` (1): ReadIdentifierPot.h
- `executables/referenceApp/application/src/` (1): main.cpp
- `executables/referenceApp/application/src/app/` (1): CanDemoListener.cpp
- `executables/referenceApp/application/src/console/` (1): console.cpp
- `executables/referenceApp/application/src/systems/` (6): DoIpServerSystem.cpp, RuntimeSystem.cpp, SafetySystem.cpp, StorageSystem.cpp, SysAdminSystem.cpp, TransportSystem.cpp
- `executables/referenceApp/application/src/uds/` (1): ReadIdentifierPot.cpp
- `executables/referenceApp/asyncBinding/` (1): BUILD.bazel
- `executables/referenceApp/asyncBinding/doc/` (1): index.rst
- `executables/referenceApp/asyncBinding/include/async/` (1): AsyncBinding.h
- `executables/referenceApp/asyncCoreConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/asyncCoreConfiguration/doc/` (1): index.rst
- `executables/referenceApp/asyncCoreConfiguration/include/async/` (1): Config.h
- `executables/referenceApp/configuration/` (3): BUILD.bazel, CMakeLists.txt, regenerate.sh
- `executables/referenceApp/configuration/common/src/busid/` (1): BusId.cpp
- `executables/referenceApp/configuration/doc/` (4): app.rst, busid.rst, index.rst, systems.rst
- `executables/referenceApp/configuration/include/app/` (1): appConfig.h
- `executables/referenceApp/configuration/include/busid/` (1): BusId.h
- `executables/referenceApp/configuration/include/systems/` (2): ICanSystem.h, IEthernetDriverSystem.h
- `executables/referenceApp/consoleCommands/doc/` (1): index.rst
- `executables/referenceApp/consoleCommands/include/can/console/` (1): CanCommand.h
- `executables/referenceApp/consoleCommands/include/lifecycle/console/` (2): LifecycleControlCommand.h, StatisticsCommand.h
- `executables/referenceApp/consoleCommands/include/safety/console/` (1): SafetyCommand.h
- `executables/referenceApp/consoleCommands/src/can/console/` (1): CanCommand.cpp
- `executables/referenceApp/consoleCommands/src/lifecycle/console/` (2): LifecycleControlCommand.cpp, StatisticsCommand.cpp
- `executables/referenceApp/consoleCommands/src/safety/console/` (1): SafetyCommand.cpp
- `executables/referenceApp/etl_profile/` (2): BUILD.bazel, etl_profile.h
- `executables/referenceApp/lwipConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/lwipConfiguration/include/` (2): lwipopts.h, rng.h
- `executables/referenceApp/lwipConfiguration/src/` (1): rng.cpp
- `executables/referenceApp/middlewareConfiguration/` (1): CMakeLists.txt
- `executables/referenceApp/platforms/posix/` (1): Options.cmake
- `executables/referenceApp/platforms/posix/bspConfiguration/include/bsp/uart/` (1): UartConfig.h
- `executables/referenceApp/platforms/posix/ethConfiguration/include/` (1): ethConfig.h
- `executables/referenceApp/platforms/posix/freeRtosCoreConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/platforms/posix/freeRtosCoreConfiguration/doc/` (1): index.rst
- `executables/referenceApp/platforms/posix/freeRtosCoreConfiguration/include/os/` (1): FreeRtosPlatformConfig.h
- `executables/referenceApp/platforms/posix/main/` (1): BUILD.bazel
- `executables/referenceApp/platforms/posix/main/doc/` (1): index.rst
- `executables/referenceApp/platforms/posix/main/doc/user/` (1): index.rst
- `executables/referenceApp/platforms/posix/main/include/lifecycle/` (1): StaticBsp.h
- `executables/referenceApp/platforms/posix/main/include/systems/` (2): CanSystem.h, TapEthernetSystem.h
- `executables/referenceApp/platforms/posix/main/src/` (1): main.cpp
- `executables/referenceApp/platforms/posix/main/src/lifecycle/` (1): StaticBsp.cpp
- `executables/referenceApp/platforms/posix/main/src/osHooks/freertos/` (1): osHooks.cpp
- `executables/referenceApp/platforms/posix/main/src/osHooks/threadx/` (1): osHooks.cpp
- `executables/referenceApp/platforms/posix/main/src/systems/` (1): CanSystem.cpp
- `executables/referenceApp/platforms/posix/safety/safeLifecycle/include/safeLifecycle/` (1): SafetyManager.h
- `executables/referenceApp/platforms/posix/safety/safeLifecycle/src/safeLifecycle/` (1): SafetyManager.cpp
- `executables/referenceApp/platforms/posix/safety/safeLifecycle/test/src/safeLifecycle/` (1): include_test.cpp
- `executables/referenceApp/platforms/posix/threadXCoreConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/platforms/posix/threadXCoreConfiguration/include/` (1): tx_user.h
- `executables/referenceApp/platforms/s32k148evb/` (1): Options.cmake
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/doc/user/` (10): bspAdc.rst, bspCan.rst, bspCharInputOutput.rst, bspClock.rst, bspIo.rst, bspIo_input.rst, bspIo_output.rst, bspIo_outputPwm.rst, bspTimer.rst, index.rst
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/adc/` (5): AnalogInput.h, AnalogInputScale.h, analogInputConfiguration.h, analogInputConfigurationStrings.h, analogInputScaleConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/can/` (1): canConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/charInputOutput/` (1): CharIOSerialCfg.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/clock/` (1): boardClock.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/eeprom/` (1): EepromConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/io/input/` (2): inputConfiguration.h, inputConfigurationStrings.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/io/outputPwm/` (4): PwmSupport.h, PwmSupportConfiguration.hpp, outputPwmConfiguration.h, outputPwmConfigurationStrings.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/phy/` (1): phyConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/sci/` (1): sciConfiguration.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/timer/` (1): ftmConfiguration.hpp
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/include/bsp/uart/` (1): UartConfig.h
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/src/bsp/adc/` (1): AnalogInput.cpp
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/src/bsp/io/outputPwm/` (1): PwmSupport.cpp
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/src/bsp/stdIo/` (1): stdIo.cpp
- `executables/referenceApp/platforms/s32k148evb/bspConfiguration/src/bsp/uart/` (1): UartConfig.cpp
- `executables/referenceApp/platforms/s32k148evb/ethConfiguration/include/` (1): ethConfig.h
- `executables/referenceApp/platforms/s32k148evb/freeRtosCoreConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/platforms/s32k148evb/freeRtosCoreConfiguration/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/freeRtosCoreConfiguration/include/os/` (1): FreeRtosPlatformConfig.h
- `executables/referenceApp/platforms/s32k148evb/main/` (2): BUILD.bazel, CMakeLists.txt
- `executables/referenceApp/platforms/s32k148evb/main/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/main/doc/user/` (5): StaticBsp.rst, index.rst, linkerScript.rst, main.rst, startup.rst
- `executables/referenceApp/platforms/s32k148evb/main/doc/user/systems/` (3): BspSystem.rst, CanSystem.rst, systems.rst
- `executables/referenceApp/platforms/s32k148evb/main/include/lifecycle/` (1): StaticBsp.h
- `executables/referenceApp/platforms/s32k148evb/main/include/osHooks/threadx/` (1): osHooks.h
- `executables/referenceApp/platforms/s32k148evb/main/include/systems/` (3): BspSystem.h, CanSystem.h, S32K148EvbEthernetSystem.h
- `executables/referenceApp/platforms/s32k148evb/main/linkerscript/` (3): application.dld, application.dld.in, hot-files.inc
- `executables/referenceApp/platforms/s32k148evb/main/src/` (1): main.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/bsp/threadx/` (1): tx_execution_initialize.c
- `executables/referenceApp/platforms/s32k148evb/main/src/lifecycle/` (1): StaticBsp.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/os/isr/` (6): isr_can.cpp, isr_enet.cpp, isr_erm.cpp, isr_ftfc.cpp, isr_sys.cpp, isr_wdg.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/osHooks/freertos/` (1): osHooks.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/osHooks/threadx/` (1): osHooks.cpp
- `executables/referenceApp/platforms/s32k148evb/main/src/systems/` (2): BspSystem.cpp, CanSystem.cpp
- `executables/referenceApp/platforms/s32k148evb/safety/safeIo/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeIo/doc/user/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeIo/include/safeIo/` (2): SafeIo.h, SafeState.h
- `executables/referenceApp/platforms/s32k148evb/safety/safeIo/src/safeIo/` (2): SafeIo.cpp, SafeState.cpp
- `executables/referenceApp/platforms/s32k148evb/safety/safeLifecycle/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeLifecycle/doc/user/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeLifecycle/include/safeLifecycle/` (3): IsrHooks.h, IsrWrapper.h, SafetyManager.h
- `executables/referenceApp/platforms/s32k148evb/safety/safeLifecycle/src/safeLifecycle/` (3): IsrHooks.cpp, IsrWrapper.cpp, SafetyManager.cpp
- `executables/referenceApp/platforms/s32k148evb/safety/safeMemory/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeMemory/doc/user/` (3): ecc.rst, mpu.rst, romCheck.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeMemory/include/safeMemory/` (8): IntegrityEccHandler.h, MemoryLocations.h, MemoryProtection.h, ProtectedRamScopedUnlock.h, RomCheck.h, SafeMemory.h, SafetyShell.h, mpu.h
- `executables/referenceApp/platforms/s32k148evb/safety/safeMemory/src/safeMemory/` (4): IntegrityEccHandler.cpp, MemoryProtection.cpp, RomCheck.cpp, SafeMemory.cpp
- `executables/referenceApp/platforms/s32k148evb/safety/safeSystem/doc/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeSystem/doc/user/` (1): index.rst
- `executables/referenceApp/platforms/s32k148evb/safety/safeSystem/include/safeSystem/` (1): SafeSystem.h
- `executables/referenceApp/platforms/s32k148evb/safety/safeSystem/src/safeSystem/` (1): SafeSystem.cpp
- `executables/referenceApp/platforms/s32k148evb/threadXCoreConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/platforms/s32k148evb/threadXCoreConfiguration/include/` (1): tx_user.h
- `executables/referenceApp/rustHelloWorld/` (1): Cargo.toml
- `executables/referenceApp/safety/safeSupervisor/doc/` (1): index.rst
- `executables/referenceApp/safety/safeSupervisor/include/safeSupervisor/` (1): SafeSupervisor.h
- `executables/referenceApp/safety/safeSupervisor/include/safeSupervisor/utils/` (1): explicitly_constructible.h
- `executables/referenceApp/safety/safeSupervisor/src/safeSupervisor/` (1): SafeSupervisor.cpp
- `executables/referenceApp/safety/safeWatchdog/doc/` (1): index.rst
- `executables/referenceApp/safety/safeWatchdog/doc/user/` (1): index.rst
- `executables/referenceApp/safety/safeWatchdog/include/safeWatchdog/` (1): SafeWatchdog.h
- `executables/referenceApp/safety/safeWatchdog/src/safeWatchdog/` (1): SafeWatchdog.cpp
- `executables/referenceApp/transportConfiguration/` (2): BUILD.bazel, CMakeLists.txt
- `executables/referenceApp/transportConfiguration/doc/` (1): index.rst
- `executables/referenceApp/udsConfiguration/` (1): BUILD.bazel
- `executables/referenceApp/udsConfiguration/doc/` (5): diagSession.rst, dummySessionPersistence.rst, index.rst, udsConfig.rst, udsLifecycleConnector.rst
- `executables/referenceApp/udsConfiguration/include/uds/` (2): DummySessionPersistence.h, UdsLifecycleConnector.h
- `executables/referenceApp/udsConfiguration/include/uds/session/` (1): DiagSession.h
- `executables/referenceApp/udsConfiguration/src/uds/session/` (1): DiagSession.cpp
- `executables/unitTest/bsp/bspConfiguration/doc/` (1): index.rst
- `executables/unitTest/bsp/bspConfiguration/include/bsp/io/input/` (2): inputConfiguration.h, inputConfigurationStrings.h
- `executables/unitTest/bsp/bspConfiguration/include/bsp/io/io/` (1): ioConfiguration.h
- `executables/unitTest/bsp/bspConfiguration/include/bsp/io/output/` (2): outputConfiguration.h, outputConfigurationStrings.h
- `executables/unitTest/bsp/bspConfiguration/include/bsp/io/outputPwm/` (2): outputPwmConfiguration.h, outputPwmConfigurationStrings.h
- `executables/unitTest/bsp/bspConfiguration/include/bsp/uart/` (1): UartConfig.h
- `executables/unitTest/bsp/bspIo/doc/` (1): index.rst
- `executables/unitTest/bsp/bspIo/include/io/` (2): Io.h, ioPorts.h
- `executables/unitTest/bsp/bspIo/src/io/` (1): Io.cpp
- `executables/unitTest/bsp/bspMcu/doc/` (1): index.rst
- `executables/unitTest/bsp/bspMcu/include/mcu/` (1): mcu.h
- `executables/unitTest/bsp/bspMcu/include/reset/` (1): softwareSystemReset.h
- `executables/unitTest/common/doc/` (1): index.rst
- `executables/unitTest/common/include/util/` (1): EcuIdList.h
- `executables/unitTest/common/include/util/crc/` (1): CRCCalculator.h
- `executables/unitTest/common/include/util/timeout/` (1): AbstractTimeoutMock.h
- `executables/unitTest/common/src/util/` (1): EcuIdList.cpp
- `executables/unitTest/common/src/util/crc/` (1): CRCCalculator.cpp
- `executables/unitTest/configuration/` (2): BUILD.bazel, CMakeLists.txt
- `executables/unitTest/configuration/common/include/common/mock/busid/` (1): BusId.h
- `executables/unitTest/configuration/common/src/common/mock/busid/` (1): BusId.cpp
- `executables/unitTest/etl_profile/` (2): BUILD.bazel, etl_profile.h
- `executables/unitTest/lwipConfiguration/include/` (1): lwipopts.h
- `executables/unitTest/transportConfiguration/` (1): CMakeLists.txt
- `executables/unitTest/transportConfiguration/src/transport/` (1): TransportConfiguration.cpp
- `executables/unitTest/udsConfiguration/include/uds/session/` (1): DiagSession.h
- `libs/bsp/bspCharInputOutput/doc/` (1): index.rst
- `libs/bsp/bspCharInputOutput/include/charInputOutput/` (5): CharIOSerialCfg.h, bspIoStd.h, charIo.h, charIoSerial.h, printfPragma.hpp
- `libs/bsp/bspCharInputOutput/src/charInputOutput/` (3): bspIo.cpp, charIo.cpp, charIoSerial.cpp
- `libs/bsp/bspCharInputOutput/test/src/` (1): IncludeTest.cpp
- `libs/bsp/bspDynamicClient/doc/` (1): index.rst
- `libs/bsp/bspDynamicClient/doc/user/` (2): DynamicClientCfg.rst, index.rst
- `libs/bsp/bspDynamicClient/include/io/` (1): DynamicClientCfg.h
- `libs/bsp/bspDynamicClient/test/src/` (1): DynamicClientTest.cpp
- `libs/bsp/bspInputManager/doc/` (1): index.rst
- `libs/bsp/bspInputManager/doc/user/` (1): index.rst
- `libs/bsp/bspInputManager/include/inputManager/` (3): AlternativeDigitalInput.h, DigitalInput.h, DigitalInputTester.h
- `libs/bsp/bspInputManager/src/inputManager/` (3): AlternativeDigitalInput.cpp, DigitalInput.cpp, DigitalInputTester.cpp
- `libs/bsp/bspInputManager/test/src/` (1): IncludeTest.cpp
- `libs/bsp/bspInterrupts/` (1): BUILD.bazel
- `libs/bsp/bspInterrupts/doc/` (1): index.rst
- `libs/bsp/bspInterrupts/doc/user/` (1): index.rst
- `libs/bsp/bspInterrupts/include/interrupts/` (2): SuspendResumeAllInterruptsLock.h, SuspendResumeAllInterruptsScopedLock.h
- `libs/bsp/bspInterrupts/mock/gmock/include/interrupts/` (2): SuspendResumeAllInterruptsMock.h, disableEnableAllInterruptsMock.h
- `libs/bsp/bspInterrupts/mock/gmock/src/` (2): SuspendResumeAllInterruptsMock.cpp, disableEnableAllInterruptsMock.cpp
- `libs/bsp/bspInterrupts/mock/include/interrupts/` (2): disableEnableAllInterrupts.h, suspendResumeAllInterrupts.h
- `libs/bsp/bspInterrupts/test/src/` (2): IncludeTest.cpp, InterruptTest.cpp
- `libs/bsp/bspOutputManager/doc/` (1): index.rst
- `libs/bsp/bspOutputManager/doc/user/` (3): Output.rst, OutputTester.rst, index.rst
- `libs/bsp/bspOutputManager/include/outputManager/` (2): Output.h, OutputTester.h
- `libs/bsp/bspOutputManager/src/outputManager/` (2): Output.cpp, OutputTester.cpp
- `libs/bsp/bspOutputManager/test/src/` (1): IncludeTest.cpp
- `libs/bsp/bspOutputPwm/` (1): CMakeLists.txt
- `libs/bsp/bspOutputPwm/doc/` (1): index.rst
- `libs/bsp/bspOutputPwm/include/outputPwm/` (2): OutputPwm.h, OutputPwmTester.h
- `libs/bsp/bspOutputPwm/src/outputPwm/` (2): OutputPwm.cpp, OutputPwmTester.cpp
- `libs/bsp/bspOutputPwm/test/src/` (1): IncludeTest.cpp
- `libs/bsw/` (1): CMakeLists.txt
- `libs/bsw/async/` (1): BUILD.bazel
- `libs/bsw/async/doc/` (1): index.rst
- `libs/bsw/async/doc/user/` (1): index.rst
- `libs/bsw/async/include/async/` (1): Async.h
- `libs/bsw/async/include/async/util/` (2): Call.h, MemberCall.h
- `libs/bsw/async/mock/gmock/include/async/` (6): AsyncMock.h, LockMock.h, RunnableMock.h, TestContext.h, TimeoutMock.h, Types.h
- `libs/bsw/async/mock/gmock/src/async/` (3): AsyncMock.cpp, TestContext.cpp, Types.cpp
- `libs/bsw/async/test/src/async/` (2): TestContextTest.cpp, TypesTest.cpp
- `libs/bsw/async/test/src/async/util/` (2): CallTest.cpp, MemberCallTest.cpp
- `libs/bsw/asyncConsole/` (1): BUILD.bazel
- `libs/bsw/asyncConsole/doc/` (1): index.rst
- `libs/bsw/asyncConsole/doc/user/` (1): index.rst
- `libs/bsw/asyncConsole/include/console/` (3): AsyncCommandWrapper.h, AsyncConsole.h, SyncCommandWrapper.h
- `libs/bsw/asyncConsole/include/logger/` (1): ConsoleLogger.h
- `libs/bsw/asyncConsole/src/console/` (3): AsyncCommandWrapper.cpp, AsyncConsole.cpp, SyncCommandWrapper.cpp
- `libs/bsw/asyncConsole/src/logger/` (1): ConsoleLogger.cpp
- `libs/bsw/asyncConsole/test/include/demo/` (1): DemoCommand.h
- `libs/bsw/asyncConsole/test/src/console/` (2): DemoTest.cpp, IncludeTest.cpp
- `libs/bsw/asyncConsole/test/src/demo/` (1): DemoCommand.cpp
- `libs/bsw/asyncFreeRtos/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/asyncFreeRtos/doc/` (1): index.rst
- `libs/bsw/asyncFreeRtos/doc/developer/` (3): 01_initialization.rst, 02_runnables.rst, index.rst
- `libs/bsw/asyncFreeRtos/doc/user/` (5): asynchronous_execution.rst, index.rst, names_hooking.rst, startup_sequence.rst, synchronization_mechanisms.rst
- `libs/bsw/asyncFreeRtos/freeRtosConfiguration/` (2): FreeRTOSConfig.h, freertos_tasks_c_additions.h
- `libs/bsw/asyncFreeRtos/freeRtosConfiguration/async/` (1): Hook.h
- `libs/bsw/asyncFreeRtos/include/async/` (10): EventManager.h, FreeRtosAdapter.h, FutureSupport.h, Lock.h, ModifiableLock.h, StaticContextHook.h, StaticRunnable.h, TaskContext.h, TaskInitializer.h, Types.h
- `libs/bsw/asyncFreeRtos/src/async/` (4): Async.cpp, FutureSupport.cpp, Hook.cpp, Types.cpp
- `libs/bsw/asyncFreeRtos/test/mock/include/async/` (5): AsyncBinding.h, Config.h, ContextHookMock.h, TickHookMock.h, TimeoutMock.h
- `libs/bsw/asyncFreeRtos/test/mock/include/os/` (1): FreeRtosPlatformConfig.h
- `libs/bsw/asyncFreeRtos/test/mock/src/async/` (2): ContextHookMock.cpp, TickHookMock.cpp
- `libs/bsw/asyncFreeRtos/test/src/async/` (9): AsyncTest.cpp, FreeRtosAdapterTest.cpp, FutureSupportTest.cpp, LockTest.cpp, ModifiableLockTest.cpp, StaticRunnableTest.cpp, TaskContextTest.cpp, TaskInitializerMinimumStackTest.cpp, TaskInitializerTest.cpp
- `libs/bsw/asyncImpl/` (1): BUILD.bazel
- `libs/bsw/asyncImpl/doc/` (1): index.rst
- `libs/bsw/asyncImpl/doc/user/` (1): index.rst
- `libs/bsw/asyncImpl/examples/include/` (1): example.h
- `libs/bsw/asyncImpl/examples/include/async/` (1): Types.h
- `libs/bsw/asyncImpl/examples/src/` (1): example.cpp
- `libs/bsw/asyncImpl/include/async/` (6): EventDispatcher.h, EventPolicy.h, IRunnable.h, Queue.h, QueueNode.h, RunnableExecutor.h
- `libs/bsw/asyncImpl/mock/include/async/` (1): RunnableMock.h
- `libs/bsw/asyncImpl/test/include/async/` (2): Async.h, Types.h
- `libs/bsw/asyncImpl/test/src/async/` (5): EventDispatcherTest.cpp, EventPolicyTest.cpp, QueueNodeTest.cpp, QueueTest.cpp, RunnableExecutorTest.cpp
- `libs/bsw/asyncThreadX/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/asyncThreadX/include/async/` (11): EventManager.h, FutureSupport.h, Lock.h, ModifiableLock.h, StaticContextHook.h, StaticRunnable.h, StaticTickHook.h, TaskContext.h, TaskInitializer.h, ThreadXAdapter.h, Types.h
- `libs/bsw/asyncThreadX/src/async/` (4): Async.cpp, FutureSupport.cpp, Hook.cpp, Types.cpp
- `libs/bsw/asyncThreadX/threadXConfiguration/` (1): ThreadXConfig.h
- `libs/bsw/asyncThreadX/threadXConfiguration/async/` (1): Hook.h
- `libs/bsw/blob/` (2): CMakeLists.txt, module.spec
- `libs/bsw/blob/doc/` (7): api.rst, configuration.rst, design.rst, examples.rst, generator.rst, index.rst, integration.rst
- `libs/bsw/blob/test/` (2): CMakeLists.txt, regenerate.sh
- `libs/bsw/bsp/` (1): BUILD.bazel
- `libs/bsw/bsp/doc/` (1): index.rst
- `libs/bsw/bsp/include/bsp/` (2): Bsp.h, SystemTime.h
- `libs/bsw/bsp/include/bsp/can/canTransceiver/` (1): CanPhy.h
- `libs/bsw/bsp/include/bsp/charInputOutput/` (1): Bspio.h
- `libs/bsw/bsp/include/bsp/eeprom/` (1): IEepromDriver.h
- `libs/bsw/bsp/include/bsp/eepromemulation/` (1): IEepromEmulationDriver.h
- `libs/bsw/bsp/include/bsp/flash/` (1): IFlashDriver.h
- `libs/bsw/bsp/include/bsp/memory/` (1): safeMemoryAccess.h
- `libs/bsw/bsp/include/bsp/power/` (1): IEcuPowerStateController.h
- `libs/bsw/bsp/include/bsp/uart/` (1): UartConcept.h
- `libs/bsw/bsp/mock/include/bsp/` (1): Uart.h
- `libs/bsw/bsp/mock/include/bsp/can/canTransceiver/` (1): CanPhyMock.h
- `libs/bsw/bsp/mock/include/bsp/eeprom/` (1): EepromDriverMock.h
- `libs/bsw/bsp/mock/include/bsp/eepromemulation/` (1): EepromEmulationDriverMock.h
- `libs/bsw/bsp/mock/include/bsp/flash/` (2): FlashDriverMock.h, FlashMock.h
- `libs/bsw/bsp/mock/include/bsp/power/` (1): EcuPowerStateControllerMock.h
- `libs/bsw/bsp/mock/include/inputManager/` (1): DigitalInput.h
- `libs/bsw/bsp/mock/include/outputManager/` (1): Output.h
- `libs/bsw/bsp/test/src/bsp/` (1): IncludeTest.cpp
- `libs/bsw/bsp/test/src/bsp/can/canTransceiver/` (1): CanPhyTest.cpp
- `libs/bsw/common/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/common/doc/` (1): index.rst
- `libs/bsw/common/doc/user/` (5): busid.rst, ifuture_support.rst, index.rst, itimeout_manager2.rst, mask.rst
- `libs/bsw/common/include/common/busid/` (1): BusId.h
- `libs/bsw/common/include/util/` (1): Mask.h
- `libs/bsw/common/include/util/concurrent/` (1): IFutureSupport.h
- `libs/bsw/common/include/util/timeout/` (1): ITimeoutManager2.h
- `libs/bsw/cpp2can/` (1): BUILD.bazel
- `libs/bsw/cpp2can/doc/` (1): index.rst
- `libs/bsw/cpp2can/doc/user/` (1): index.rst
- `libs/bsw/cpp2can/include/can/` (1): CanLogger.h
- `libs/bsw/cpp2can/include/can/canframes/` (3): CANFrame.h, CanId.h, ICANFrameSentListener.h
- `libs/bsw/cpp2can/include/can/filter/` (5): AbstractStaticBitFieldFilter.h, BitFieldFilter.h, IFilter.h, IMerger.h, IntervalFilter.h
- `libs/bsw/cpp2can/include/can/framemgmt/` (4): AbstractBitFieldFilteredCANFrameListener.h, AbstractIntervalFilteredCANFrameListener.h, ICANFrameListener.h, IFilteredCANFrameSentListener.h
- `libs/bsw/cpp2can/include/can/transceiver/` (4): AbstractCANTransceiver.h, ICANTransceiverStateListener.h, ICanTransceiver.h, statistics.h
- `libs/bsw/cpp2can/mock/include/can/canframes/` (1): CANFrameSentListenerMock.h
- `libs/bsw/cpp2can/mock/include/can/filter/` (2): FilterMock.h, MergerMock.h
- `libs/bsw/cpp2can/mock/include/can/framemgmt/` (2): CANFrameListenerMock.h, FilteredCANFrameSentListenerMock.h
- `libs/bsw/cpp2can/mock/include/can/transceiver/` (2): AbstractCANTransceiverMock.h, ICanTransceiverMock.h
- `libs/bsw/cpp2can/mock/src/can/transceiver/` (1): AbstractCANTransceiverMock.cpp
- `libs/bsw/cpp2can/src/can/` (1): CanLogger.cpp
- `libs/bsw/cpp2can/src/can/canframes/` (1): CANFrame.cpp
- `libs/bsw/cpp2can/src/can/filter/` (3): AbstractStaticBitFieldFilter.cpp, BitFieldFilter.cpp, IntervalFilter.cpp
- `libs/bsw/cpp2can/src/can/transceiver/` (1): AbstractCANTransceiver.cpp
- `libs/bsw/cpp2can/test/src/can/canframes/` (2): CANFrameTest.cpp, CanIdTest.cpp
- `libs/bsw/cpp2can/test/src/can/filter/` (2): BitFieldFilterTest.cpp, IntervalFilterTest.cpp
- `libs/bsw/cpp2can/test/src/can/transceiver/` (1): AbstractCANTransceiverTest.cpp
- `libs/bsw/cpp2ethernet/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/cpp2ethernet/doc/` (1): index.rst
- `libs/bsw/cpp2ethernet/include/ethernet/` (1): EthernetLogger.h
- `libs/bsw/cpp2ethernet/include/ip/` (5): INetworkInterfaceConfigRegistry.h, IPAddress.h, IPEndpoint.h, NetworkInterfaceConfig.h, to_str.h
- `libs/bsw/cpp2ethernet/include/tcp/` (3): IDataListener.h, IDataSendNotificationListener.h, TcpLogger.h
- `libs/bsw/cpp2ethernet/include/tcp/socket/` (3): AbstractServerSocket.h, AbstractSocket.h, ISocketProvidingConnectionListener.h
- `libs/bsw/cpp2ethernet/include/tcp/util/` (3): BandwidthTestSocket.h, LoopbackTestServer.h, TcpIperf2Server.h
- `libs/bsw/cpp2ethernet/include/udp/` (4): DatagramPacket.h, IDataListener.h, IDataSentListener.h, UdpLogger.h
- `libs/bsw/cpp2ethernet/include/udp/socket/` (1): AbstractDatagramSocket.h
- `libs/bsw/cpp2ethernet/include/udp/util/` (2): UdpEchoTestServer.h, UdpIperf2Server.h
- `libs/bsw/cpp2ethernet/mock/include/cpp2ethernet/` (1): gtest_extensions.h
- `libs/bsw/cpp2ethernet/mock/include/ip/` (2): NetworkInterfaceConfigMock.h, NetworkInterfaceConfigRegistryMock.h
- `libs/bsw/cpp2ethernet/mock/include/tcp/` (2): DataListenerMock.h, DataSendNotificationListenerMock.h
- `libs/bsw/cpp2ethernet/mock/include/tcp/socket/` (3): AbstractServerSocketMock.h, AbstractSocketMock.h, SocketProvidingConnectionListenerMock.h
- `libs/bsw/cpp2ethernet/mock/include/udp/` (2): DataListenerMock.h, DataSentListenerMock.h
- `libs/bsw/cpp2ethernet/mock/include/udp/socket/` (1): AbstractDatagramSocketMock.h
- `libs/bsw/cpp2ethernet/src/ethernet/` (1): EthernetLogger.cpp
- `libs/bsw/cpp2ethernet/src/ip/` (2): NetworkInterfaceConfig.cpp, to_str.cpp
- `libs/bsw/cpp2ethernet/src/tcp/` (1): TcpLogger.cpp
- `libs/bsw/cpp2ethernet/src/tcp/socket/` (2): AbstractServerSocket.cpp, AbstractSocket.cpp
- `libs/bsw/cpp2ethernet/src/tcp/util/` (3): BandwidthTestSocket.cpp, LoopbackTestServer.cpp, TcpIperf2Server.cpp
- `libs/bsw/cpp2ethernet/src/udp/` (2): DatagramPacket.cpp, UdpLogger.cpp
- `libs/bsw/cpp2ethernet/src/udp/socket/` (1): AbstractDatagramSocket.cpp
- `libs/bsw/cpp2ethernet/src/udp/util/` (2): UdpEchoTestServer.cpp, UdpIperf2Server.cpp
- `libs/bsw/cpp2ethernet/test/mock/include/logger/` (1): Logger.h
- `libs/bsw/cpp2ethernet/test/mock/src/logger/` (1): Logger.cpp
- `libs/bsw/cpp2ethernet/test/src/` (1): include_test.cpp
- `libs/bsw/cpp2ethernet/test/src/ip/` (4): IPAddressTest.cpp, IPEndpointTest.cpp, NetworkInterfaceConfigTest.cpp, to_strTest.cpp
- `libs/bsw/cpp2ethernet/test/src/tcp/socket/` (2): AbstractServerSocketTest.cpp, AbstractSocketTest.cpp
- `libs/bsw/cpp2ethernet/test/src/udp/` (1): DatagramPacketTest.cpp
- `libs/bsw/cpp2ethernet/test/src/udp/socket/` (1): AbstractDatagramSocketTest.cpp
- `libs/bsw/docan/` (1): BUILD.bazel
- `libs/bsw/docan/doc/` (1): index.rst
- `libs/bsw/docan/doc/user/` (1): index.rst
- `libs/bsw/docan/include/docan/addressing/` (3): DoCanNormalAddressing.h, DoCanNormalAddressingFilter.h, IDoCanAddressConverter.h
- `libs/bsw/docan/include/docan/can/` (2): DoCanPhysicalCanTransceiver.h, DoCanPhysicalCanTransceiverContainer.h
- `libs/bsw/docan/include/docan/common/` (7): DoCanConnection.h, DoCanConstants.h, DoCanJobHandle.h, DoCanLogger.h, DoCanParameters.h, DoCanTimerManagement.h, DoCanTransportAddressPair.h
- `libs/bsw/docan/include/docan/datalink/` (15): DoCanCanDataLinkLayer.h, DoCanDataLinkAddressPair.h, DoCanDataLinkLayer.h, DoCanDefaultFrameSizeMapper.h, DoCanFdFrameSizeMapper.h, DoCanFrameCodec.h, DoCanFrameCodecConfig.h, DoCanFrameCodecConfigPresets.h, DoCanFrameDecoder.h, IDoCanDataFrameTransmitter.h, IDoCanDataFrameTransmitterCallback.h, IDoCanFlowControlFrameTransmitter.h, IDoCanFrameReceiver.h, IDoCanFrameSizeMapper.h, IDoCanPhysicalTransceiver.h
- `libs/bsw/docan/include/docan/receiver/` (3): DoCanMessageReceiveProtocolHandler.h, DoCanMessageReceiver.h, DoCanReceiver.h
- `libs/bsw/docan/include/docan/transmitter/` (4): DoCanMessageTransmitProtocolHandler.h, DoCanMessageTransmitter.h, DoCanTransmitter.h, IDoCanTickGenerator.h
- `libs/bsw/docan/include/docan/transport/` (3): DoCanTransportLayer.h, DoCanTransportLayerConfig.h, DoCanTransportLayerContainer.h
- `libs/bsw/docan/mock/gmock/include/docan/addressing/` (1): DoCanAddressConverterMock.h
- `libs/bsw/docan/mock/gmock/include/docan/datalink/` (5): DoCanDataFrameTransmitterCallbackMock.h, DoCanDataFrameTransmitterMock.h, DoCanFlowControlFrameTransmitterMock.h, DoCanFrameReceiverMock.h, DoCanPhysicalTransceiverMock.h
- `libs/bsw/docan/mock/gmock/include/docan/transmitter/` (1): DoCanTickGeneratorMock.h
- `libs/bsw/docan/src/docan/common/` (1): DoCanLogger.cpp
- `libs/bsw/docan/src/docan/datalink/` (1): DoCanFrameCodecConfigPresets.cpp
- `libs/bsw/docan/test/benchmark/` (1): benchmark.cpp
- `libs/bsw/docan/test/src/docan/` (1): DoCanIncludeTest.cpp
- `libs/bsw/docan/test/src/docan/addressing/` (2): DoCanNormalAddressingFilterTest.cpp, DoCanNormalAddressingTest.cpp
- `libs/bsw/docan/test/src/docan/can/` (2): DoCanPhysicalCanTransceiverContainerTest.cpp, DoCanPhysicalCanTransceiverTest.cpp
- `libs/bsw/docan/test/src/docan/common/` (3): DoCanConnectionTest.cpp, DoCanParametersTest.cpp, DoCanTransportAddressPairTest.cpp
- `libs/bsw/docan/test/src/docan/datalink/` (3): DoCanDataLinkAddressPairTest.cpp, DoCanFrameCodecTest.cpp, DoCanFrameDecoderTest.cpp
- `libs/bsw/docan/test/src/docan/integration/` (1): DemoTest.cpp
- `libs/bsw/docan/test/src/docan/receiver/` (3): DoCanMessageReceiveProtocolHandlerTest.cpp, DoCanMessageReceiverTest.cpp, DoCanReceiverTest.cpp
- `libs/bsw/docan/test/src/docan/transmitter/` (3): DoCanMessageTransmitProtocolHandlerTest.cpp, DoCanMessageTransmitterTest.cpp, DoCanTransmitterTest.cpp
- `libs/bsw/docan/test/src/docan/transport/` (3): DoCanTransportLayerConfigTest.cpp, DoCanTransportLayerContainerTest.cpp, DoCanTransportLayerTest.cpp
- `libs/bsw/doip/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/doip/doc/` (5): architecture_server_ident.puml, architecture_server_transport.puml, index.rst, server_connection.puml, transport_router.puml
- `libs/bsw/doip/include/doip/common/` (23): DoIpCommonLogger.h, DoIpConstants.h, DoIpCyclicTaskGenerator.h, DoIpHeader.h, DoIpLock.h, DoIpResult.h, DoIpSendJobHelper.h, DoIpSimplePayloadSendJob.h, DoIpStaticPayloadSendJob.h, DoIpTcpConnection.h, DoIpTransportMessageProvidingListenerAdapter.h, DoIpTransportMessageProvidingListenerHelper.h, DoIpTransportMessageRef.h, DoIpTransportMessageSendJob.h, DoIpUdpConnection.h, DoIpVehicleIdentificationRequestSendJob.h, IDoIpConnection.h, IDoIpConnectionHandler.h, IDoIpSendJob.h, IDoIpSendJobCallback.h, IDoIpTcpConnection.h, IDoIpTransportMessageProvidingListener.h, IDoIpVehicleAnnouncementListener.h
- `libs/bsw/doip/include/doip/server/` (31): DoIpServerConnectionHandler.h, DoIpServerLogger.h, DoIpServerSocketHandler.h, DoIpServerTransportConnection.h, DoIpServerTransportConnectionConfig.h, DoIpServerTransportConnectionPool.h, DoIpServerTransportConnectionProvider.h, DoIpServerTransportLayer.h, DoIpServerTransportLayerParameters.h, DoIpServerTransportMessageHandler.h, DoIpServerVehicleIdentification.h, DoIpServerVehicleIdentificationConfig.h, DoIpServerVehicleIdentificationParameters.h, DoIpServerVehicleIdentificationRequest.h, DoIpServerVehicleIdentificationService.h, DoIpServerVehicleIdentificationSocketHandler.h, IDoIpServerConnection.h, IDoIpServerConnectionFilter.h, IDoIpServerConnectionHandlerCallback.h, IDoIpServerConnectionStateCallback.h, IDoIpServerEntityStatusCallback.h, IDoIpServerMessageHandler.h, IDoIpServerSocketHandler.h, IDoIpServerSocketHandlerListener.h, IDoIpServerTransportConnectionCreator.h, IDoIpServerTransportConnectionPool.h, IDoIpServerTransportConnectionProviderCallback.h, IDoIpServerTransportLayerCallback.h, IDoIpServerVehicleAnnouncementParameterProvider.h, IDoIpServerVehicleIdentificationCallback.h, IDoIpUdpOemMessageHandler.h
- `libs/bsw/doip/mock/gmock/include/doip/common/` (7): DoIpConnectionHandlerMock.h, DoIpConnectionMock.h, DoIpSendJobCallbackMock.h, DoIpSendJobMock.h, DoIpTcpConnectionMock.h, DoIpTransportMessageProvidingListenerMock.h, DoIpVehicleAnnouncementListenerMock.h
- `libs/bsw/doip/mock/gmock/include/doip/server/` (13): DoIpServerConnectionFilterMock.h, DoIpServerConnectionHandlerCallbackMock.h, DoIpServerConnectionMock.h, DoIpServerEntityStatusCallbackMock.h, DoIpServerMessageHandlerMock.h, DoIpServerSocketHandlerListenerMock.h, DoIpServerSocketHandlerMock.h, DoIpServerTransportConnectionPoolMock.h, DoIpServerTransportConnectionProviderCallbackMock.h, DoIpServerTransportLayerCallbackMock.h, DoIpServerVehicleAnnouncementParameterProviderMock.h, DoIpServerVehicleIdentificationCallbackMock.h, DoIpUdpOemMessageHandlerMock.h
- `libs/bsw/doip/src/doip/common/` (11): DoIpCommonLogger.cpp, DoIpCyclicTaskGenerator.cpp, DoIpSendJobHelper.cpp, DoIpSimplePayloadSendJob.cpp, DoIpStaticPayloadSendJob.cpp, DoIpTcpConnection.cpp, DoIpTransportMessageProvidingListenerAdapter.cpp, DoIpTransportMessageProvidingListenerHelper.cpp, DoIpTransportMessageSendJob.cpp, DoIpUdpConnection.cpp, DoIpVehicleIdentificationRequestSendJob.cpp
- `libs/bsw/doip/src/doip/server/` (8): DoIpServerConnectionHandler.cpp, DoIpServerLogger.cpp, DoIpServerTransportConnection.cpp, DoIpServerTransportConnectionProvider.cpp, DoIpServerTransportLayer.cpp, DoIpServerTransportMessageHandler.cpp, DoIpServerVehicleIdentification.cpp, DoIpServerVehicleIdentificationSocketHandler.cpp
- `libs/bsw/doip/test/` (1): CMakeLists.txt
- `libs/bsw/doip/test/include/doip/common/` (1): DoIpTestHelpers.h
- `libs/bsw/doip/test/src/doip/common/` (14): DoIpCyclicTaskGeneratorTest.cpp, DoIpHeaderTest.cpp, DoIpResultTest.cpp, DoIpSendJobHelperTest.cpp, DoIpSimplePayloadSendJobTest.cpp, DoIpStaticPayloadSendJobTest.cpp, DoIpTcpConnectionTest.cpp, DoIpTestHelpers.cpp, DoIpTransportMessageProvidingAdapterTest.cpp, DoIpTransportMessageProvidingListenerHelperTest.cpp, DoIpTransportMessageRefTest.cpp, DoIpTransportMessageSendJobTest.cpp, DoIpUdpConnectionTest.cpp, DoIpVehicleIdentificationRequestSendJobTest.cpp
- `libs/bsw/doip/test/src/doip/server/` (16): DoIpServerConnectionHandlerTest.cpp, DoIpServerSocketHandlerTest.cpp, DoIpServerTransportConnectionConfigTest.cpp, DoIpServerTransportConnectionPoolTest.cpp, DoIpServerTransportConnectionProviderTest.cpp, DoIpServerTransportConnectionTest.cpp, DoIpServerTransportLayerParametersTest.cpp, DoIpServerTransportLayerTest.cpp, DoIpServerTransportMessageHandlerTest.cpp, DoIpServerVehicleIdentificationConfigTest.cpp, DoIpServerVehicleIdentificationRequestTest.cpp, DoIpServerVehicleIdentificationServiceTest.cpp, DoIpServerVehicleIdentificationSocketHandlerTest.cpp, DoIpServerVehicleIdentificationTest.cpp, IDoIpServerConnectionFilterTest.cpp, IDoIpServerEntityStatusCallbackTest.cpp
- `libs/bsw/estd/` (1): CMakeLists.txt
- `libs/bsw/estd/include/estd/` (20): algorithm.h, array.h, big_endian.h, bitset.h, constructor.h, forward_list.h, functional.h, iterator.h, memory.h, none.h, object_pool.h, optional.h, ordered_map.h, slice.h, type_list.h, type_traits.h, type_utils.h, uncopyable.h, variant.h, vector.h
- `libs/bsw/io/` (1): BUILD.bazel
- `libs/bsw/io/benchmark/src/` (1): main.cpp
- `libs/bsw/io/doc/` (1): index.rst
- `libs/bsw/io/doc/user/` (9): buffered_writer.rst, forwarding_reader.rst, index.rst, ireader.rst, iwriter.rst, join_reader.rst, memory_queue.rst, split_writer.rst, variant_queue.rst
- `libs/bsw/io/examples/` (6): BufferedWriterExample.cpp, ForwardingReaderExample.cpp, JoinReaderExample.cpp, MemoryQueueExample.cpp, SplitWriterExample.cpp, VariantQueueExample.cpp
- `libs/bsw/io/include/io/` (8): BufferedWriter.h, ForwardingReader.h, IReader.h, IWriter.h, JoinReader.h, MemoryQueue.h, SplitWriter.h, VariantQueue.h
- `libs/bsw/io/mock/include/io/` (2): IReaderMock.h, IWriterMock.h
- `libs/bsw/io/src/io/` (1): BufferedWriter.cpp
- `libs/bsw/io/test/src/io/` (6): BufferedWriterTest.cpp, ForwardingReaderTest.cpp, JoinReaderTest.cpp, MemoryQueueTest.cpp, SplitWriterTest.cpp, VariantQueueTest.cpp
- `libs/bsw/lifecycle/` (1): BUILD.bazel
- `libs/bsw/lifecycle/doc/` (1): index.rst
- `libs/bsw/lifecycle/examples/include/` (1): example.h
- `libs/bsw/lifecycle/examples/src/` (1): examples.cpp
- `libs/bsw/lifecycle/include/lifecycle/` (12): AsyncLifecycleComponent.h, ILifecycleComponent.h, ILifecycleComponentCallback.h, ILifecycleListener.h, ILifecycleManager.h, LifecycleComponent.h, LifecycleLogger.h, LifecycleManager.h, LifecycleManagerForwarder.h, LifecycleManagerInitializer.h, SimpleLifecycleComponent.h, SingleContextLifecycleComponent.h
- `libs/bsw/lifecycle/mock/gmock/include/lifecycle/` (4): LifecycleComponentCallbackMock.h, LifecycleComponentMock.h, LifecycleListenerMock.h, LifecycleManagerMock.h
- `libs/bsw/lifecycle/src/lifecycle/` (7): AsyncLifecycleComponent.cpp, LifecycleComponent.cpp, LifecycleLogger.cpp, LifecycleManager.cpp, LifecycleManagerForwarder.cpp, SimpleLifecycleComponent.cpp, SingleContextLifecycleComponent.cpp
- `libs/bsw/lifecycle/test/src/lifecycle/` (7): AsyncLifecycleComponentTest.cpp, LifecycleComponentTest.cpp, LifecycleManagerForwarderTest.cpp, LifecycleManagerInitializerTest.cpp, LifecycleManagerTest.cpp, SimpleLifecycleComponentTest.cpp, SingleContextLifecycleComponentTest.cpp
- `libs/bsw/logger/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/logger/doc/` (1): index.rst
- `libs/bsw/logger/include/logger/` (19): BufferedLoggerOutput.h, BufferedLoggerOutputClient.h, ComponentConfig.h, ComponentMapping.h, ConsoleEntryOutput.h, DefaultEntryFormatter.h, DefaultLoggerCommand.h, DefaultLoggerTime.h, EntryBuffer.h, EntrySerializer.h, IBufferedLoggerOutputClient.h, IComponentConfig.h, IEntryFormatter.h, IEntryOutput.h, ILoggerListener.h, ILoggerTime.h, IPersistenceManager.h, PersistentComponentConfig.h, SharedStreamEntryOutput.h
- `libs/bsw/logger/mock/include/logger/` (1): PersistenceManagerMock.h
- `libs/bsw/logger/rust/` (2): Cargo.toml, cbindgen.toml
- `libs/bsw/logger/src/logger/` (2): DefaultLoggerCommand.cpp, EntrySerializer.cpp
- `libs/bsw/logger/test/src/logger/` (12): BufferedLoggerOutputClientTest.cpp, BufferedLoggerOutputTest.cpp, ComponentConfigTest.cpp, ComponentMappingTest.cpp, ConsoleEntryOutputTest.cpp, DefaultEntryFormatterTest.cpp, DefaultLoggerCommandTest.cpp, DefaultLoggerTimeTest.cpp, EntryBufferTest.cpp, EntrySerializerTest.cpp, PersistentComponentConfigTest.cpp, SharedStreamEntryOutputTest.cpp
- `libs/bsw/loggerIntegration/` (1): BUILD.bazel
- `libs/bsw/loggerIntegration/doc/` (1): index.rst
- `libs/bsw/loggerIntegration/include/logger/` (4): Config.h, ConsoleEntryFormatter.h, LoggerComposition.h, LoggerTime.h
- `libs/bsw/loggerIntegration/src/logger/` (2): LoggerComposition.cpp, LoggerTime.cpp
- `libs/bsw/lwipSocket/` (1): BUILD.bazel
- `libs/bsw/lwipSocket/doc/` (1): index.rst
- `libs/bsw/lwipSocket/include/lwipSocket/netif/` (1): LwipNetworkInterface.h
- `libs/bsw/lwipSocket/include/lwipSocket/tcp/` (2): LwipServerSocket.h, LwipSocket.h
- `libs/bsw/lwipSocket/include/lwipSocket/udp/` (1): LwipDatagramSocket.h
- `libs/bsw/lwipSocket/include/lwipSocket/utils/` (3): LwipHelper.h, LwipLogger.h, TaskAssert.h
- `libs/bsw/lwipSocket/src/lwipSocket/netif/` (1): LwipNetworkInterface.cpp
- `libs/bsw/lwipSocket/src/lwipSocket/tcp/` (2): LwipServerSocket.cpp, LwipSocket.cpp
- `libs/bsw/lwipSocket/src/lwipSocket/udp/` (1): LwipDatagramSocket.cpp
- `libs/bsw/lwipSocket/src/lwipSocket/utils/` (2): FilterFrame.cpp, TaskAssert.cpp
- `libs/bsw/lwipSocket/test/src/lwipSocket/` (1): LwipSocketTest.cpp
- `libs/bsw/middleware/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/middleware/doc/` (1): index.rst
- `libs/bsw/middleware/doc/dd/` (3): core.rst, index.rst, queue.rst
- `libs/bsw/middleware/simulation/` (1): CMakeLists.txt
- `libs/bsw/middleware/simulation/docs/` (1): README.md
- `libs/bsw/middleware/test/` (1): CMakeLists.txt
- `libs/bsw/middleware/test/src/core/` (1): middleware_message_unittest.cpp
- `libs/bsw/middleware/tools/cpp_generator/` (1): README.md
- `libs/bsw/platform/` (1): BUILD.bazel
- `libs/bsw/platform/doc/` (1): index.rst
- `libs/bsw/platform/doc/user/` (2): estdint.rst, index.rst
- `libs/bsw/platform/include/platform/` (2): config.h, estdint.h
- `libs/bsw/platform/include/platform/config/` (2): clang.h, gnu.h
- `libs/bsw/platform/test/src/platform/` (4): atomic.cpp, config.cpp, estdint.cpp, estdint_c.c
- `libs/bsw/routing/` (2): CMakeLists.txt, module.spec
- `libs/bsw/routing/doc/` (4): design.rst, index.rst, integration.rst, interface.rst
- `libs/bsw/routing/doc/resources/` (1): routing_example.dot
- `libs/bsw/routing/test/` (2): CMakeLists.txt, regenerate.sh
- `libs/bsw/runtime/` (1): BUILD.bazel
- `libs/bsw/runtime/doc/` (1): index.rst
- `libs/bsw/runtime/doc/user/` (1): index.rst
- `libs/bsw/runtime/include/runtime/` (13): FunctionExecutionMonitor.h, FunctionRuntimeStatistics.h, NestedRuntimeEntry.h, RuntimeMonitor.h, RuntimeStack.h, RuntimeStackEntry.h, RuntimeStatistics.h, SharedStatisticsContainer.h, SimpleRuntimeEntry.h, StatisticsContainer.h, StatisticsIterator.h, StatisticsWriter.h, Tracer.h
- `libs/bsw/runtime/src/runtime/` (2): StatisticsWriter.cpp, Tracer.cpp
- `libs/bsw/runtime/test/src/` (12): FunctionExecutionMonitorTest.cpp, FunctionRuntimeStatisticsTest.cpp, NestedRuntimeEntryTest.cpp, RuntimeMonitorTest.cpp, RuntimeStackEntryTest.cpp, RuntimeStackTest.cpp, RuntimeStatisticsTest.cpp, SharedStatisticsContainerTest.cpp, SimpleRuntimeEntryTest.cpp, StatisticsContainerTest.cpp, StatisticsIteratorTest.cpp, StatisticsWriterTest.cpp
- `libs/bsw/stdioConsoleInput/` (1): BUILD.bazel
- `libs/bsw/stdioConsoleInput/doc/` (1): index.rst
- `libs/bsw/stdioConsoleInput/doc/user/` (1): index.rst
- `libs/bsw/stdioConsoleInput/include/console/` (1): StdioConsoleInput.h
- `libs/bsw/stdioConsoleInput/src/console/` (1): StdioConsoleInput.cpp
- `libs/bsw/storage/` (1): BUILD.bazel
- `libs/bsw/storage/doc/` (1): index.rst
- `libs/bsw/storage/include/storage/` (7): EepStorage.h, FeeStorage.h, IStorage.h, MappingStorage.h, QueuingStorage.h, StorageJob.h, StorageTester.h
- `libs/bsw/storage/mock/include/storage/` (1): IStorageMock.h
- `libs/bsw/storage/src/storage/` (4): EepStorage.cpp, MappingStorage.cpp, QueuingStorage.cpp, StorageTester.cpp
- `libs/bsw/storage/test/src/` (1): StorageTest.cpp
- `libs/bsw/time/` (3): BUILD.bazel, CMakeLists.txt, module.spec
- `libs/bsw/time/test/` (1): CMakeLists.txt
- `libs/bsw/timer/` (1): BUILD.bazel
- `libs/bsw/timer/doc/` (1): index.rst
- `libs/bsw/timer/doc/user/` (1): index.rst
- `libs/bsw/timer/include/timer/` (2): Timeout.h, Timer.h
- `libs/bsw/timer/test/src/` (1): TimerTest.cpp
- `libs/bsw/transport/` (1): BUILD.bazel
- `libs/bsw/transport/doc/` (1): index.rst
- `libs/bsw/transport/doc/helpers/` (1): LogicalAddress.rst
- `libs/bsw/transport/include/transport/` (9): AbstractTransportLayer.h, BufferedTransportMessage.h, ITransportMessageListener.h, ITransportMessageProcessedListener.h, ITransportMessageProvider.h, ITransportMessageProvidingListener.h, TransportLogger.h, TransportMessage.h, TransportMessageSendJob.h
- `libs/bsw/transport/mock/gmock/include/transport/` (5): AbstractTransportLayerMock.h, TransportMessageListenerMock.h, TransportMessageProcessedListenerMock.h, TransportMessageProviderMock.h, TransportMessageProvidingListenerMock.h
- `libs/bsw/transport/mock/gmock/src/` (1): TransportMessageProvidingListenerMock.cpp
- `libs/bsw/transport/src/` (3): AbstractTransportLayer.cpp, TransportLogger.cpp, TransportMessage.cpp
- `libs/bsw/transport/test/` (1): CMakeLists.txt
- `libs/bsw/transport/test/include/transport/` (1): TransportConfiguration.h
- `libs/bsw/transport/test/src/` (5): AbstractTransportLayerTest.cpp, IncludeTest.cpp, Logger.cpp, TransportConfiguration.cpp, TransportMessageTest.cpp
- `libs/bsw/transportRouterSimple/` (1): BUILD.bazel
- `libs/bsw/transportRouterSimple/doc/` (1): index.rst
- `libs/bsw/transportRouterSimple/include/transport/` (1): TpRouterLogger.h
- `libs/bsw/transportRouterSimple/include/transport/routing/` (1): TransportRouterSimple.h
- `libs/bsw/transportRouterSimple/src/transport/` (1): TpRouterLogger.cpp
- `libs/bsw/transportRouterSimple/test/` (1): CMakeLists.txt
- `libs/bsw/uds/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/uds/application_config_example/` (2): DiagSession.cpp, DiagSession.h
- `libs/bsw/uds/doc/` (1): index.rst
- `libs/bsw/uds/doc/user/` (5): connection.rst, dispatcher.rst, index.rst, runtime_diag_tree.puml, sessions.rst
- `libs/bsw/uds/include/uds/` (10): DefaultEepromConstants.h, DiagDispatcher.h, DiagReturnCode.h, DiagnosisConfiguration.h, ICommunicationStateListener.h, ICommunicationSubStateListener.h, IDiagDispatcher.h, IUdsResetConfiguration.h, UdsConstants.h, UdsLogger.h
- `libs/bsw/uds/include/uds/async/` (4): AsyncDiagHelper.h, AsyncDiagJob.h, AsyncDiagJobHelper.h, IAsyncDiagHelper.h
- `libs/bsw/uds/include/uds/authentication/` (2): DefaultDiagAuthenticator.h, IDiagAuthenticator.h
- `libs/bsw/uds/include/uds/base/` (10): AbstractDiagJob.h, DiagJobRoot.h, DiagJobWithAuthentication.h, DiagJobWithAuthenticationAndSessionControl.h, Service.h, ServiceWithAuthentication.h, ServiceWithAuthenticationAndSessionControl.h, Subfunction.h, SubfunctionWithAuthentication.h, SubfunctionWithAuthenticationAndSessionControl.h
- `libs/bsw/uds/include/uds/connection/` (4): ErrorCode.h, IncomingDiagConnection.h, NestedDiagRequest.h, PositiveResponse.h
- `libs/bsw/uds/include/uds/jobs/` (6): DataIdentifierJob.h, ReadIdentifierFromMemory.h, ReadIdentifierFromMemoryWithAuthentication.h, ReadIdentifierFromSliceRef.h, RoutineControlJob.h, WriteIdentifierToMemory.h
- `libs/bsw/uds/include/uds/lifecycle/` (1): IUdsLifecycleConnector.h
- `libs/bsw/uds/include/uds/resume/` (2): IResumableResetDriverPersistence.h, ResumableResetDriver.h
- `libs/bsw/uds/include/uds/services/communicationcontrol/` (2): CommunicationControl.h, ICommunicationStateManager.h
- `libs/bsw/uds/include/uds/services/controldtcsetting/` (1): ControlDTCSetting.h
- `libs/bsw/uds/include/uds/services/ecureset/` (2): ECUReset.h, PowerDown.h
- `libs/bsw/uds/include/uds/services/inputoutputcontrol/` (1): InputOutputControlByIdentifier.h
- `libs/bsw/uds/include/uds/services/readdata/` (2): MultipleReadDataByIdentifier.h, ReadDataByIdentifier.h
- `libs/bsw/uds/include/uds/services/routinecontrol/` (4): RequestRoutineResults.h, RoutineControl.h, StartRoutine.h, StopRoutine.h
- `libs/bsw/uds/include/uds/services/securityaccess/` (1): SecurityAccess.h
- `libs/bsw/uds/include/uds/services/sessioncontrol/` (1): ISessionPersistence.h
- `libs/bsw/uds/include/uds/services/testerpresent/` (1): TesterPresent.h
- `libs/bsw/uds/include/uds/services/writedata/` (1): WriteDataByIdentifier.h
- `libs/bsw/uds/include/uds/session/` (5): ApplicationDefaultSession.h, ApplicationExtendedSession.h, IDiagSessionChangedListener.h, IDiagSessionManager.h, ProgrammingSession.h
- `libs/bsw/uds/include/util/` (1): RoutineControlOptionParser.h
- `libs/bsw/uds/mock/gmock/include/` (1): StubMock.h
- `libs/bsw/uds/mock/gmock/include/uds/async/` (1): AsyncDiagHelperMock.h
- `libs/bsw/uds/mock/gmock/include/uds/authentication/` (1): DiagAuthenticatorMock.h
- `libs/bsw/uds/mock/gmock/include/uds/base/` (2): AbstractDiagJobMock.h, DiagJobMock.h
- `libs/bsw/uds/mock/gmock/include/uds/connection/` (4): IncomingDiagConnectionMock.h, NestedDiagRequestMock.h, OutgoingDiagConnectionProviderMock.h, ResponseMock.h
- `libs/bsw/uds/mock/gmock/include/uds/jobs/` (1): JobMocks.h
- `libs/bsw/uds/mock/gmock/include/uds/lifecycle/` (1): UdsLifecycleConnectorMock.h
- `libs/bsw/uds/mock/gmock/include/uds/resume/` (2): DiagDispatcherMock.h, ResumableResetDriverPersistenceMock.h
- `libs/bsw/uds/mock/gmock/include/uds/services/communicationcontrol/` (1): CommunicationStateManagerMock.h
- `libs/bsw/uds/mock/gmock/include/uds/services/sessioncontrol/` (1): SessionPersistenceMock.h
- `libs/bsw/uds/mock/gmock/include/uds/session/` (3): DiagSessionChangedListenerMock.h, DiagSessionManagerMock.h, DiagSessionMock.h
- `libs/bsw/uds/mock/gmock/src/uds/connection/` (2): IncomingDiagConnectionMock.cpp, ResponseMock.cpp
- `libs/bsw/uds/mock/gmock/src/uds/jobs/` (1): JobMocks.cpp
- `libs/bsw/uds/mock/gmock/src/uds/session/` (1): DiagSessionMock.cpp
- `libs/bsw/uds/mock/stub/include/` (1): UdsStub.h
- `libs/bsw/uds/mock/stub/src/` (1): UdsStub.cpp
- `libs/bsw/uds/src/uds/` (2): DiagDispatcher.cpp, UdsLogger.cpp
- `libs/bsw/uds/src/uds/async/` (2): AsyncDiagHelper.cpp, AsyncDiagJobHelper.cpp
- `libs/bsw/uds/src/uds/authentication/` (1): DefaultDiagAuthenticator.cpp
- `libs/bsw/uds/src/uds/base/` (10): AbstractDiagJob.cpp, DiagJobRoot.cpp, DiagJobWithAuthentication.cpp, DiagJobWithAuthenticationAndSessionControl.cpp, Service.cpp, ServiceWithAuthentication.cpp, ServiceWithAuthenticationAndSessionControl.cpp, Subfunction.cpp, SubfunctionWithAuthentication.cpp, SubfunctionWithAuthenticationAndSessionControl.cpp
- `libs/bsw/uds/src/uds/connection/` (3): IncomingDiagConnection.cpp, NestedDiagRequest.cpp, PositiveResponse.cpp
- `libs/bsw/uds/src/uds/jobs/` (6): DataIdentifierJob.cpp, ReadIdentifierFromMemory.cpp, ReadIdentifierFromMemoryWithAuthentication.cpp, ReadIdentifierFromSliceRef.cpp, RoutineControlJob.cpp, WriteIdentifierToMemory.cpp
- `libs/bsw/uds/src/uds/resume/` (1): ResumableResetDriver.cpp
- `libs/bsw/uds/src/uds/services/communicationcontrol/` (1): CommunicationControl.cpp
- `libs/bsw/uds/src/uds/services/controldtcsetting/` (1): ControlDTCSetting.cpp
- `libs/bsw/uds/src/uds/services/ecureset/` (2): ECUReset.cpp, PowerDown.cpp
- `libs/bsw/uds/src/uds/services/inputoutputcontrol/` (1): InputOutputControlByIdentifier.cpp
- `libs/bsw/uds/src/uds/services/readdata/` (2): MultipleReadDataByIdentifier.cpp, ReadDataByIdentifier.cpp
- `libs/bsw/uds/src/uds/services/routinecontrol/` (4): RequestRoutineResults.cpp, RoutineControl.cpp, StartRoutine.cpp, StopRoutine.cpp
- `libs/bsw/uds/src/uds/services/securityaccess/` (1): SecurityAccess.cpp
- `libs/bsw/uds/src/uds/services/testerpresent/` (1): TesterPresent.cpp
- `libs/bsw/uds/src/uds/services/writedata/` (1): WriteDataByIdentifier.cpp
- `libs/bsw/uds/src/util/` (1): RoutineControlOptionParser.cpp
- `libs/bsw/uds/test/` (1): CMakeLists.txt
- `libs/bsw/uds/test/mock/include/logger/` (1): Logger.h
- `libs/bsw/uds/test/mock/include/transport/` (2): TransportConfiguration.h, TransportMessageWithBuffer.h
- `libs/bsw/uds/test/mock/include/uds/session/` (1): DiagSession.h
- `libs/bsw/uds/test/mock/src/` (1): Logger.cpp
- `libs/bsw/uds/test/mock/src/transport/` (1): TransportMessageWithBuffer.cpp
- `libs/bsw/uds/test/mock/src/uds/session/` (1): DiagSession.cpp
- `libs/bsw/uds/test/src/uds/` (2): IncludeTest.cpp, IntegrationTest.cpp
- `libs/bsw/uds/test/src/uds/async/` (3): AsyncDiagHelperTest.cpp, AsyncDiagJobHelperTest.cpp, AsyncDiagJobTest.cpp
- `libs/bsw/uds/test/src/uds/authentication/` (1): DefaultDiagAuthenticatorTest.cpp
- `libs/bsw/uds/test/src/uds/base/` (11): AbstractDiagJobTest.cpp, AbstractDiagJobWithDiagRoot.cpp, DiagJobRootTest.cpp, DiagJobWithAuthenticationAndSessionControlTest.cpp, DiagJobWithAuthenticationTest.cpp, ServiceTest.cpp, ServiceWithAuthenticationAndSessionControlTest.cpp, ServiceWithAuthenticationTest.cpp, SubfunctionTest.cpp, SubfunctionWithAuthenticationAndSessionControlTest.cpp, SubfunctionWithAuthenticationTest.cpp
- `libs/bsw/uds/test/src/uds/connection/` (3): ManagedIncomingDiagConnectionTest.cpp, NestedDiagRequestTest.cpp, PositiveResponseTest.cpp
- `libs/bsw/uds/test/src/uds/jobs/` (6): DataIdentifierJobTest.cpp, ReadIdentifierFromMemoryJobTest.cpp, ReadIdentifierFromMemoryWithAuthenticationTest.cpp, ReadIdentifierFromSliceRefTest.cpp, RoutineControlJobTest.cpp, WritedentifierToMemoryJobTest.cpp
- `libs/bsw/uds/test/src/uds/resume/` (1): ResumableResetDriverTest.cpp
- `libs/bsw/uds/test/src/uds/services/` (1): CommunicationControlTest.cpp
- `libs/bsw/uds/test/src/uds/services/controldtcsetting/` (1): ControlDTCSettingTest.cpp
- `libs/bsw/uds/test/src/uds/services/readdata/` (2): MultipleReadDataByIdentifierTest.cpp, ReadDataByIdentifierTest.cpp
- `libs/bsw/uds/test/src/uds/services/routinecontrol/` (4): RequestRoutineResultsTest.cpp, RoutineControlTest.cpp, StartRoutineTest.cpp, StopRoutineTest.cpp
- `libs/bsw/uds/test/src/uds/services/securityaccess/` (1): SecurityAccessTest.cpp
- `libs/bsw/uds/test/src/uds/services/testerpresent/` (1): TesterPresentTest.cpp
- `libs/bsw/uds/test/src/uds/services/writedata/` (1): WriteDataByIdentifierTest.cpp
- `libs/bsw/uds/test/src/util/` (1): RoutineControlOptionParserTest.cpp
- `libs/bsw/util/` (2): BUILD.bazel, CMakeLists.txt
- `libs/bsw/util/doc/` (1): index.rst
- `libs/bsw/util/doc/user/` (10): command.rst, crc.rst, defer.rst, format.rst, index.rst, logger.rst, memory.rst, meta.rst, stream.rst, string.rst
- `libs/bsw/util/examples/` (2): Crc8Example.cpp, Crc8RohcExample.cpp
- `libs/bsw/util/include/util/buffer/` (1): LinkedBuffer.h
- `libs/bsw/util/include/util/command/` (7): CommandContext.h, GroupCommand.h, HelpCommand.h, ICommand.h, IParentCommand.h, ParentCommand.h, SimpleCommand.h
- `libs/bsw/util/include/util/crc/` (7): Crc.h, Crc16.h, Crc32.h, Crc8.h, LookupTable.h, Reflect.h, Xor.h
- `libs/bsw/util/include/util/defer/` (1): Defer.h
- `libs/bsw/util/include/util/estd/` (6): assert.h, block_pool.h, derived_object_pool.h, intrusive.h, signal.h, va_list_ref.h
- `libs/bsw/util/include/util/format/` (9): AttributedString.h, IPrintfArgumentReader.h, Printf.h, PrintfArgumentReader.h, PrintfFormatScanner.h, PrintfFormatter.h, SharedStringWriter.h, StringWriter.h, Vt100AttributedStringFormatter.h
- `libs/bsw/util/include/util/logger/` (5): ComponentInfo.h, IComponentMapping.h, ILoggerOutput.h, LevelInfo.h, LoggerBinding.h
- `libs/bsw/util/include/util/math/` (1): MovingAverage.h
- `libs/bsw/util/include/util/memory/` (2): Bit.h, BuddyMemoryManager.h
- `libs/bsw/util/include/util/meta/` (2): BinaryValue.h, Bitmask.h
- `libs/bsw/util/include/util/preprocessor/` (1): Macros.h
- `libs/bsw/util/include/util/spsc/` (2): Queue.h, ReadWrite.h
- `libs/bsw/util/include/util/stream/` (14): BspStubs.h, ByteBufferOutputStream.h, INonBlockingInputStream.h, IOutputStream.h, ISharedOutputStream.h, NormalizeLfOutputStream.h, NullOutputStream.h, SharedOutputStream.h, SharedOutputStreamResource.h, StdinStream.h, StdoutStream.h, TaggedOutputHelper.h, TaggedOutputStream.h, TaggedSharedOutputStream.h
- `libs/bsw/util/include/util/string/` (1): ConstString.h
- `libs/bsw/util/include/util/types/` (1): Enum.h
- `libs/bsw/util/mock/gmock/include/util/` (1): StdIoMock.h
- `libs/bsw/util/mock/gmock/include/util/estd/` (2): function_mock.h, gtest_extensions.h
- `libs/bsw/util/mock/gmock/include/util/logger/` (3): ComponentMappingMock.h, LoggerOutputMock.h, TestConsoleLogger.h
- `libs/bsw/util/mock/gmock/include/util/stream/` (2): OutputStreamMock.h, SharedOutputStreamMock.h
- `libs/bsw/util/mock/gmock/src/util/` (1): StdIoMock.cpp
- `libs/bsw/util/mock/gmock/src/util/logger/` (1): TestConsoleLogger.cpp
- `libs/bsw/util/src/util/command/` (5): CommandContext.cpp, GroupCommand.cpp, HelpCommand.cpp, ParentCommand.cpp, SimpleCommand.cpp
- `libs/bsw/util/src/util/crc/` (8): LookupTable_0x07.cpp, LookupTable_0x1021.cpp, LookupTable_0x1D.cpp, LookupTable_0x2F.cpp, LookupTable_0x31.cpp, LookupTable_0x4C11DB7.cpp, LookupTable_0xCF.cpp, LookupTable_0xF4ACFB13.cpp
- `libs/bsw/util/src/util/estd/` (1): assert.cpp
- `libs/bsw/util/src/util/format/` (7): AttributedString.cpp, PrintfArgumentReader.cpp, PrintfFormatScanner.cpp, PrintfFormatter.cpp, SharedStringWriter.cpp, StringWriter.cpp, Vt100AttributedStringFormatter.cpp
- `libs/bsw/util/src/util/logger/` (2): ComponentInfo.cpp, LevelInfo.cpp
- `libs/bsw/util/src/util/memory/` (1): BuddyMemoryManager.cpp
- `libs/bsw/util/src/util/stream/` (9): ByteBufferOutputStream.cpp, NormalizeLfOutputStream.cpp, NullOutputStream.cpp, SharedOutputStream.cpp, StdinStream.cpp, StdoutStream.cpp, TaggedOutputHelper.cpp, TaggedOutputStream.cpp, TaggedSharedOutputStream.cpp
- `libs/bsw/util/src/util/string/` (1): ConstString.cpp
- `libs/bsw/util/test/` (1): CMakeLists.txt
- `libs/bsw/util/test/include/fixtures/crc/` (1): CrcTestFixture.h
- `libs/bsw/util/test/src/util/` (1): IncludeTest.cpp
- `libs/bsw/util/test/src/util/command/` (6): CommandContextTest.cpp, GroupCommandTest.cpp, HelpCommandTest.cpp, ICommandTest.cpp, ParentCommandTest.cpp, SimpleCommandTest.cpp
- `libs/bsw/util/test/src/util/crc/` (8): Crc16Test.cpp, Crc32Test.cpp, Crc8Test.cpp, CrcRegisterTest.cpp, CrcTest.cpp, LookupTableTest.cpp, ReflectTest.cpp, XorTest.cpp
- `libs/bsw/util/test/src/util/defer/` (1): DeferTest.cpp
- `libs/bsw/util/test/src/util/format/` (7): AttributedStringTest.cpp, PrintfArgumentReaderTest.cpp, PrintfFormatScannerTest.cpp, PrintfFormatterTest.cpp, SharedStringWriterTest.cpp, StringWriterTest.cpp, Vt100AttributedStringFormatterTest.cpp
- `libs/bsw/util/test/src/util/logger/` (3): ComponentInfoTest.cpp, LevelInfoTest.cpp, LoggerTest.cpp
- `libs/bsw/util/test/src/util/math/` (1): MovingAverageTest.cpp
- `libs/bsw/util/test/src/util/memory/` (2): BitTest.cpp, BuddyMemoryManagerTest.cpp
- `libs/bsw/util/test/src/util/meta/` (2): BinaryValueTest.cpp, BitmaskTest.cpp
- `libs/bsw/util/test/src/util/spsc/` (1): QueueTest.cpp
- `libs/bsw/util/test/src/util/stream/` (10): ByteBufferOutputStreamTest.cpp, NormalizeLfOutputStreamTest.cpp, NullOutputStreamTest.cpp, SharedOutputStreamResourceTest.cpp, SharedOutputStreamTest.cpp, StdinStreamTest.cpp, StdoutStreamTest.cpp, TaggedOutputHelperTest.cpp, TaggedOutputStreamTest.cpp, TaggedSharedOutputStreamTest.cpp
- `libs/bsw/util/test/src/util/string/` (1): ConstStringTest.cpp
- `libs/bsw/util/test/src/util/types/` (1): EnumTest.cpp
- `libs/rust/console_out/` (1): Cargo.toml
- `libs/rust/panic_handler/` (1): Cargo.toml
- `libs/safety/safeMonitor/doc/` (1): index.rst
- `libs/safety/safeMonitor/include/safeMonitor/` (6): Register.h, Sequence.h, Trigger.h, Value.h, Watchdog.h, common.h
- `libs/safety/safeMonitor/mock/include/safeMonitor/` (5): RegisterMock.h, SequenceMock.h, TriggerMock.h, ValueMock.h, WatchdogMock.h
- `libs/safety/safeMonitor/test/include/` (1): common.h
- `libs/safety/safeMonitor/test/src/` (6): RegisterTest.cpp, SequenceTest.cpp, TriggerTest.cpp, ValueTest.cpp, WatchdogTest.cpp, common.cpp
- `libs/safety/safeUtils/doc/` (1): index.rst
- `libs/safety/safeUtils/include/safeUtils/` (1): SafetyLogger.h
- `libs/safety/safeUtils/test/src/safeUtils/` (1): include_test.cpp
- `platforms/posix/3rdparty/freeRtosPosix/` (1): BUILD.bazel
- `platforms/posix/3rdparty/threadx/` (1): BUILD.bazel
- `platforms/posix/bsp/bspEepromDriver/doc/` (1): index.rst
- `platforms/posix/bsp/bspEepromDriver/src/eeprom/` (1): EepromDriver.cpp
- `platforms/posix/bsp/bspEepromDriver/test/` (1): CMakeLists.txt
- `platforms/posix/bsp/bspEepromDriver/test/src/eeprom/` (1): EepromDriverTest.cpp
- `platforms/posix/bsp/bspInterruptsImpl/` (1): BUILD.bazel
- `platforms/posix/bsp/bspInterruptsImpl/doc/` (1): index.rst
- `platforms/posix/bsp/bspInterruptsImpl/freertos/include/interrupts/` (1): suspendResumeAllInterrupts.h
- `platforms/posix/bsp/bspInterruptsImpl/freertos/src/interrupts/` (1): suspendResumeAllInterrupts.cpp
- `platforms/posix/bsp/bspInterruptsImpl/threadx/include/interrupts/` (1): suspendResumeAllInterrupts.h
- `platforms/posix/bsp/bspInterruptsImpl/threadx/src/interrupts/` (1): suspendResumeAllInterrupts.cpp
- `platforms/posix/bsp/bspMcu/doc/` (1): index.rst
- `platforms/posix/bsp/bspMcu/include/mcu/` (1): mcu.h
- `platforms/posix/bsp/bspMcu/include/reset/` (1): softwareSystemReset.h
- `platforms/posix/bsp/bspMcu/src/reset/` (1): softwareSystemReset.cpp
- `platforms/posix/bsp/bspStdio/doc/` (1): index.rst
- `platforms/posix/bsp/bspSystemTime/doc/` (1): index.rst
- `platforms/posix/bsp/bspUart/include/bsp/` (1): Uart.h
- `platforms/posix/bsp/socketCanTransceiver/doc/` (1): index.rst
- `platforms/posix/bsp/socketCanTransceiver/include/can/` (1): SocketCanTransceiver.h
- `platforms/posix/bsp/socketCanTransceiver/test/src/can/` (2): IncludeTest.cpp, SocketCanTransceiverTest.cpp
- `platforms/posix/bsp/tapEthernetDriver/include/` (1): TapEthernetDriver.h
- `platforms/posix/etlImpl/` (1): BUILD.bazel
- `platforms/posix/etlImpl/src/` (2): clocks.cpp, print.cpp
- `platforms/posix/lwipSysArch/` (1): BUILD.bazel
- `platforms/posix/lwipSysArch/include/arch/` (2): cc.h, sys_arch.h
- `platforms/posix/lwipSysArch/src/arch/` (1): sys_arch.cpp
- `platforms/stm32/` (1): CMakeLists.txt
- `platforms/stm32/bsp/` (1): CMakeLists.txt
- `platforms/stm32/bsp/bspCan/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspCan/doc/` (1): index.rst
- `platforms/stm32/bsp/bspCan/test/` (1): CMakeLists.txt
- `platforms/stm32/bsp/bspClock/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspClock/doc/` (1): index.rst
- `platforms/stm32/bsp/bspInterruptsImpl/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspInterruptsImpl/doc/` (1): index.rst
- `platforms/stm32/bsp/bspIo/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspIo/doc/` (1): index.rst
- `platforms/stm32/bsp/bspMcu/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspMcu/doc/` (1): index.rst
- `platforms/stm32/bsp/bspMcu/include/3rdparty/st/stm32f4/` (1): .riminfo
- `platforms/stm32/bsp/bspMcu/include/3rdparty/st/stm32g4/` (1): .riminfo
- `platforms/stm32/bsp/bspTimer/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspTimer/doc/` (1): index.rst
- `platforms/stm32/bsp/bspUart/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bspUart/doc/` (1): index.rst
- `platforms/stm32/bsp/bxCanTransceiver/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/bxCanTransceiver/doc/` (1): index.rst
- `platforms/stm32/bsp/bxCanTransceiver/test/` (1): CMakeLists.txt
- `platforms/stm32/bsp/fdCanTransceiver/` (2): CMakeLists.txt, module.spec
- `platforms/stm32/bsp/fdCanTransceiver/doc/` (1): index.rst
- `platforms/stm32/bsp/fdCanTransceiver/test/` (1): CMakeLists.txt
- `platforms/stm32/cmake/` (2): stm32f413zh.cmake, stm32g474re.cmake
- `platforms/stm32/etlImpl/` (1): CMakeLists.txt
- `platforms/stm32/unitTest/` (1): CMakeLists.txt
- `test/pyTest/` (11): .pytest.ini, README.md, capture_serial.py, conftest.py, process_mgmt.py, pty_forwarder.py, serial_minilog.py, target_info.py, target_posix.toml, target_s32k148.toml, target_s32k148_with_hwtester.toml
- `test/pyTest/can/` (1): test_can.py
- `test/pyTest/console/` (9): test_adc.py, test_can_command.py, test_lc_reboot.py, test_lifecycle.py, test_logger.py, test_restart.py, test_stats_cpu.py, test_stats_stack.py, test_storage.py
- `test/pyTest/enet/` (1): test_ethernet.py
- `test/pyTest/helper/` (1): helper.py
- `test/pyTest/middleware/` (1): README.md
- `test/pyTest/uds/` (5): test_DiagSession.py, test_RDBI.py, test_TesterPresent.py, test_WDBI.py, test_udsToolRDBI.py
- `test/pyTest/uds/helpers/` (1): helper_functions.py
- `test/pyTest/unittests/` (2): test_capture_serial.py, test_process_mgmt.py

## Cross-references

- **U02** - import and differential-test UDS/DoCAN/DoIP vectors: bucket B UDS
  service/config paths and transport tester-address translation. Note: DoCAN
  itself has no upstream source drift (section 3), and DoIP drift is
  mechanical (section 4); U02 should therefore focus on the UDS 0x14/0x19
  vectors, UdsConfig parameterization, and TP-router translation tests.
  (`DoIpServerSystem.h` sizing was originally routed here; it was
  reclassified to bucket C on 2026-07-19 - see changelog.)
- **U03** - STM32/posix CAN: bucket B `platforms/stm32/bsp/bspCan`,
  `bxCanTransceiver`, `fdCanTransceiver` sources/tests and the
  `SocketCanTransceiver` extended-ID fix, compared against the native Rust
  bxCAN/FDCAN health, overflow and bus-off contracts.
- **U04** - middleware and reference-application composition: bucket B
  lifecycle/ethernet/stdio/UART/util paths; bucket C middleware-framework
  material feeds the U06 re-pin decision instead.
- **U05** - build, ETL, license and NOTICE review: the build/toolchain area
  (section 1) and the ETL version bumps; all classified no-behavioral-effect
  here, reviewed in U05 for dependency-policy impact only.

## Verification

- Bucket totals: 1525 + 55 + 274 = 1854 (exact).
- All 166 commits appear exactly once across the nine area sections.
- The pinned oracle `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` was not moved;
  this survey does not re-pin and does not restate release claims.

## Bucket B coverage map (2026-07-19 reconciliation)

This section reconciles every path originally classified bucket B (56 paths:
the 55 current bucket B paths plus `DoIpServerSystem.h`, reclassified to
bucket C - see changelog) against the drift-labeled tests and classification
records produced by U02-U05. Each row names the covering test(s) and/or the
divergence / re-pin record. Drift-labeled tests are NOT pinned-oracle
evidence.

Test-file keys:

| Key | File |
|---|---|
| T-UDS | `crates/bsw-uds/tests/drift_be0029b.rs` (13 tests) |
| T-DOIP | `crates/bsw-doip/tests/drift_be0029b.rs` (3 tests) |
| T-TP | `crates/bsw-transport/tests/drift_be0029b.rs` (3 tests, added 2026-07-19) |
| T-CAN | `crates/bsw-can/tests/drift_stm32_can_semantics.rs` (6 tests) |
| T-STM | `crates/bsw-bsp-stm32/tests/drift_stm32_can_semantics.rs` (5 tests) |
| T-APP | `crates/openbsw-reference-app/tests/reference_app_drift_2026_07_19.rs` (2 tests) |

Record keys: `U02-DIV-n` = numbered divergence n in
`drift-vectors-2026-07-19.md`; `U04-D-n` / `U04-RP-n` = divergence n /
re-pin item n in `composition-drift-review-2026-07-19.md`; `U03-GAP-nn` =
gap row nn in `stm32-can-drift-comparison-2026-07-19.md`; `CV-n` = coverage
record n defined below.

### Coverage records defined here

- **CV-1** (`07b75511`, `8f31f107` - `StringBufferOutputStream`
  finalization): post-drift upstream replaces the destructor try/catch with
  a shared non-throwing `finalizeBuffer()` and adds corner-size vectors.
  The port has no suffix-string stream type; per `util-parity.md` the
  `stream/*` family is assigned to the native bounded sinks (C14/C17/C18),
  whose contract (truncate at character boundaries, explicit truncated
  flag, no panic or abort at any buffer size) is pinned by the existing
  `bsw-logger` unit test `long_messages_truncate_at_character_boundaries`.
  The upstream hazard being fixed - an exception escaping a destructor - is
  unrepresentable in Rust (no exceptions; `Drop` cannot throw), matching
  the covered-by-construction pattern of U02-DIV-5. Classification:
  upstream-robustness-fix, port unaffected by construction; no new vector
  importable.
- **CV-2** (`bd0078d0`, `f381dd46` - POSIX UART/stdin read-error handling,
  `Uart.cpp`): upstream fixed both `fcntl` calls when configuring
  non-blocking console reads. The port's POSIX console input is a native
  replacement (`bsw.stdioConsoleInput` C15-C17): `bsw-console::posix`
  propagates read errors as typed `Result` values through `pump_once` and
  the fd-flag-manipulation defect class does not exist; robustness is
  pinned by the existing tests `io_writer_swallows_stream_errors` and
  `scripted_sessions_are_deterministic`. Classification:
  deliberate-native-difference, no new vector importable.
- **CV-3** (`39212d01` - ecureset configuration surface): the reset/
  shutdown constants moved from in-class constants to integrator-owned
  `uds/UdsConfig.h` with byte-identical values pin -> tip
  (`RESET_TIME_HARD`/`RESET_TIME_SOFT` = 1000 ms, `SHUTDOWN_TIME` = 10 ms;
  verified in the drift checkout). Wire behavior of 0x11 is unchanged and
  already pinned by the mandatory parity evidence (`bsw.uds` E14-E25);
  there is no new vector to import. Adopting the configuration surface
  itself is re-pin scope, same classification as the configuration-surface
  portion of U02-DIV-3.

### Coverage map (56 rows)

U02 path group (32 paths):

| Path | Coverage |
|---|---|
| `executables/referenceApp/application/include/systems/DoIpServerSystem.h` | reclassified to bucket C; U04-RP-5 (`330514aa`; no separate protocol-send-job pool exists in the Rust DoIP frontend to size) |
| `executables/referenceApp/application/include/systems/UdsSystem.h` | T-UDS `drift_9558c245_*`, `drift_f8132091_*` (service semantics) + U04-D-1 (composition registration) |
| `executables/referenceApp/application/src/systems/UdsSystem.cpp` | T-UDS `drift_9558c245_*`, `drift_f8132091_*` + U04-D-1 |
| `executables/referenceApp/transportConfiguration/include/transport/TransportConfiguration.h` | T-TP `drift_120f5688_tester_address_translation_is_expressible_as_integrator_config` + U04-D-5 |
| `executables/referenceApp/transportConfiguration/src/transport/TransportConfiguration.cpp` | T-TP (same test) + U04-D-5 |
| `executables/referenceApp/udsConfiguration/include/uds/UdsConfig.h` | T-UDS `drift_39212d01_*` + U02-DIV-3; ecureset portion CV-3 |
| `executables/unitTest/configuration/common/include/busid/BusId.h` | T-TP `drift_120f5688_router_boundary_round_trip_forwards_addresses_unchanged` (bus-id model) + U04-D-5 |
| `executables/unitTest/configuration/common/src/busid/BusId.cpp` | T-TP (same test) + U04-D-5 |
| `executables/unitTest/transportConfiguration/include/transport/TransportConfiguration.h` | T-TP (all 3 tests; UT vectors 0x0ECD/0x00F0/0x0006) + U04-D-5 |
| `executables/unitTest/udsConfiguration/include/uds/UdsConfig.h` | T-UDS `drift_39212d01_*` + U02-DIV-3 |
| `libs/bsw/transport/include/transport/LogicalAddress.h` | T-TP `drift_120f5688_tester_address_translation_is_expressible_as_integrator_config` |
| `libs/bsw/transport/src/LogicalAddress.cpp` | T-TP (same test) |
| `libs/bsw/transport/test/src/TesterAddressTest.cpp` | T-TP (same test; vector source) |
| `libs/bsw/transportRouterSimple/src/transport/routing/TransportRouterSimple.cpp` | T-TP `drift_120f5688_router_boundary_round_trip_forwards_addresses_unchanged`, `drift_120f5688_selfdiag_reply_without_declared_return_route_is_not_routed` + U04-D-5 |
| `libs/bsw/transportRouterSimple/test/src/TransportRouterSimpleTest.cpp` | T-TP (same 2 tests; vector source) + U04-D-5 |
| `libs/bsw/uds/include/uds/DiagCodes.h` | T-UDS `drift_39212d01_default_session_response_carries_configured_p2_values` + U02-DIV-3 |
| `libs/bsw/uds/include/uds/services/cleardiagnosticinformation/ClearDiagnosticInformation.h` | T-UDS `drift_9558c245_*` (5 tests) + U02-DIV-2 |
| `libs/bsw/uds/include/uds/services/ecureset/EnableRapidPowerShutdown.h` | CV-3 |
| `libs/bsw/uds/include/uds/services/ecureset/HardReset.h` | CV-3 |
| `libs/bsw/uds/include/uds/services/ecureset/SoftReset.h` | CV-3 |
| `libs/bsw/uds/include/uds/services/readdtcinformation/ReadDTCInformation.h` | T-UDS `drift_f8132091_*` (6 tests) + U02-DIV-1 |
| `libs/bsw/uds/include/uds/services/sessioncontrol/DiagnosticSessionControl.h` | T-UDS `drift_39212d01_*` + U02-DIV-3 |
| `libs/bsw/uds/src/uds/services/cleardiagnosticinformation/ClearDiagnosticInformation.cpp` | T-UDS `drift_9558c245_*` + U02-DIV-2 |
| `libs/bsw/uds/src/uds/services/ecureset/EnableRapidPowerShutdown.cpp` | CV-3 |
| `libs/bsw/uds/src/uds/services/ecureset/HardReset.cpp` | CV-3 |
| `libs/bsw/uds/src/uds/services/ecureset/SoftReset.cpp` | CV-3 |
| `libs/bsw/uds/src/uds/services/readdtcinformation/ReadDTCInformation.cpp` | T-UDS `drift_f8132091_*` + U02-DIV-1 |
| `libs/bsw/uds/src/uds/services/sessioncontrol/DiagnosticSessionControl.cpp` | T-UDS `drift_39212d01_*` + U02-DIV-3 |
| `libs/bsw/uds/test/mock/include/uds/UdsConfig.h` | T-UDS `drift_39212d01_*` + U02-DIV-3 |
| `libs/bsw/uds/test/src/uds/services/cleardiagnosticinformation/ClearDiagnosticInformationTest.cpp` | T-UDS `drift_9558c245_*` (vector source) + U02-DIV-2 |
| `libs/bsw/uds/test/src/uds/services/readdtcinformation/ReadDTCInformationTest.cpp` | T-UDS `drift_f8132091_*` (vector source) + U02-DIV-1 |
| `libs/bsw/uds/test/src/uds/services/sessioncontrol/DiagnosticSessionControlTest.cpp` | T-UDS `drift_39212d01_*` (vector source) + U02-DIV-3 |

U04 path group (8 paths):

| Path | Coverage |
|---|---|
| `executables/referenceApp/application/src/systems/EthernetSystem.cpp` | T-APP `drift_366d993e_network_echo_survives_reboot_and_full_restart` |
| `executables/referenceApp/platforms/posix/main/src/systems/TapEthernetSystem.cpp` | T-APP (same test, `b119bf9b`) |
| `libs/bsw/util/include/util/stream/StringBufferOutputStream.h` | CV-1 |
| `libs/bsw/util/src/util/stream/StringBufferOutputStream.cpp` | CV-1 |
| `libs/bsw/util/test/src/util/stream/StringBufferOutputStreamTest.cpp` | CV-1 (vector source) |
| `platforms/posix/bsp/bspStdio/src/bsp/stdIo/stdIo.cpp` | U04 section 2 class (a) record for `bd0078d0` (port test `f03_console_tree_and_log_level_are_scriptable`) |
| `platforms/posix/bsp/bspUart/src/bsp/Uart.cpp` | CV-2 |
| `platforms/posix/bsp/tapEthernetDriver/src/TapEthernetDriver.cpp` | T-APP `drift_366d993e_network_echo_survives_reboot_and_full_restart` (`b119bf9b`) |

U03 path group (16 paths):

| Path | Coverage |
|---|---|
| `platforms/posix/bsp/socketCanTransceiver/src/can/SocketCanTransceiver.cpp` | T-CAN `extended_id_raw_value_strips_qualifier_bits_for_display` (U03 row 9) |
| `platforms/stm32/bsp/bspCan/include/can/BxCanDevice.h` | T-CAN `lifecycle_guards_match_upstream_adapter_state_machine`, `classic_dlc_nine_to_fifteen_clamp_to_eight_bytes`, `bit_field_filter_domain_is_standard_ids_only`; T-STM `bxcan_status_decode_is_read_only`, `rx_queue_full_drops_newest_preserves_order_and_accounts`; U03-GAP-33/34/36 |
| `platforms/stm32/bsp/bspCan/src/can/BxCanDevice.cpp` | same as `BxCanDevice.h` |
| `platforms/stm32/bsp/bspCan/include/can/FdCanDevice.h` | T-STM `fdcan_ir_acknowledge_is_single_snapshot_write_back`, `bus_off_supervision_uses_exact_deadline_and_monotonic_overflow_accounting`; U03-GAP-35 |
| `platforms/stm32/bsp/bspCan/src/can/FdCanDevice.cpp` | same as `FdCanDevice.h` |
| `platforms/stm32/bsp/bspCan/test/include/mcu/mcu.h` | vector source for the T-CAN/T-STM register-seam tests (U03 rows 1-13) |
| `platforms/stm32/bsp/bspCan/test/src/can/BxCanDeviceTest.cpp` | vector source; `f2f01102` host-width/delegate repairs are stronger-by-construction in Rust (U03 rows 19-20) |
| `platforms/stm32/bsp/bspCan/test/src/can/FdCanDeviceTest.cpp` | vector source; U03 rows 19-20 |
| `platforms/stm32/bsp/bxCanTransceiver/include/can/transceiver/bxcan/BxCanTransceiver.h` | T-CAN `lifecycle_guards_match_upstream_adapter_state_machine`, `bus_off_is_latched_until_explicit_timed_recovery`, `tx_saturation_vocabulary_and_drop_accounting_exist`; T-STM `error_passive_flag_maps_to_passive_state`; U03-GAP-37 |
| `platforms/stm32/bsp/bxCanTransceiver/src/can/transceiver/bxcan/BxCanTransceiver.cpp` | same as `BxCanTransceiver.h` |
| `platforms/stm32/bsp/bxCanTransceiver/test/mock/include/can/BxCanDevice.h` | vector source for the same tests |
| `platforms/stm32/bsp/bxCanTransceiver/test/src/can/BxCanTransceiverTest.cpp` | vector source for the same tests |
| `platforms/stm32/bsp/fdCanTransceiver/include/can/transceiver/fdcan/FdCanTransceiver.h` | T-CAN `bus_off_is_latched_until_explicit_timed_recovery`; T-STM `bus_off_supervision_uses_exact_deadline_and_monotonic_overflow_accounting`; U03-GAP-37 |
| `platforms/stm32/bsp/fdCanTransceiver/src/can/transceiver/fdcan/FdCanTransceiver.cpp` | same as `FdCanTransceiver.h` |
| `platforms/stm32/bsp/fdCanTransceiver/test/mock/include/can/FdCanDevice.h` | vector source for the same tests |
| `platforms/stm32/bsp/fdCanTransceiver/test/src/can/FdCanTransceiverTest.cpp` | vector source for the same tests |

Coverage tally: 44 of the 56 paths are covered by drift-labeled tests
(most with an additional divergence/re-pin cross-reference), 11 by a
classification record only (CV-1: 3, CV-2: 1, CV-3: 6, U04 class (a): 1),
and 1 was reclassified to bucket C (`DoIpServerSystem.h`, U04-RP-5).

## Changelog

- 2026-07-19 (initial): survey created (U01).
- 2026-07-19 (reconciliation): `executables/referenceApp/application/
  include/systems/DoIpServerSystem.h` (`330514aa`) reclassified from
  bucket B (testable now, U02) to bucket C (requires re-pin). Justification:
  the constant corrected by `330514aa` sizes upstream's separate
  protocol-send-job pool inside the reference-app DoIP frontend template;
  the Rust port's `DoIpEntity` has a single const-generic `SEND_JOBS`
  diagnostic pool and no distinct protocol-send-job pool, so the corrected
  sizing is not demonstrable against the current API (matches U04's class
  (c) determination and re-pin item U04-RP-5). Bucket totals updated:
  B 56 -> 55, C 273 -> 274, total unchanged at 1,854.
- 2026-07-19 (reconciliation): the five `120f5688` transport/
  transportRouterSimple paths and the four `120f5688` configuration paths
  kept their bucket B classification: the upstream-demonstrated router
  round-trip and translation-table semantics were imported as
  `crates/bsw-transport/tests/drift_be0029b.rs` (3 tests; boundary
  behavior pinned at pinned-parity with the post-drift per-boundary
  rewrite recorded as upstream-behavior-change, U04-D-5). U02's original
  commit sweep had missed `120f5688` because the commit is routed to the
  reference-application area; `drift-vectors-2026-07-19.md` now carries
  the late determination.
- 2026-07-19 (reconciliation): bucket B coverage map appended (this
  section); coverage records CV-1..CV-3 defined for the paths whose drift
  content is covered by construction or by existing native-replacement
  evidence rather than by a new drift test.
