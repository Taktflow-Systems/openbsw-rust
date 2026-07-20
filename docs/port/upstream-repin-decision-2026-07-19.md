# Upstream re-pin decision memo (U06) - 2026-07-19

This is the U06 deliverable from `docs/port/upstream-drift-2026-07-18.md`:
the deliberate decision on whether, and under what scope, a future release
re-pins the behavioral oracle past the current baseline. It consumes the
closed follow-up packages U01-U05:

- U01 `upstream-drift-survey-2026-07-19.md` (module/file survey, 1,854
  mandatory-surface paths classified, bucket B coverage map)
- U02 `drift-vectors-2026-07-19.md` (UDS/DoCAN/DoIP determinations and
  drift-labeled vectors, incl. the 2026-07-19 `120f5688` reconciliation)
- U03 `stm32-can-drift-comparison-2026-07-19.md` (STM32/posix CAN
  semantics comparison)
- U04 `composition-drift-review-2026-07-19.md` (middleware and
  reference-app composition review)
- U05 `build-license-drift-review-2026-07-19.md` (build/toolchain and
  license/NOTICE delta review)

## 1. Decision

**Go-with-scope.** A future deliberate re-pin of the behavioral oracle to
the surveyed drift tip is approved in principle, but only as a dedicated,
separately-budgeted re-pin tranche that satisfies every criterion in
section 4, and only with the scope stated here. It is explicitly NOT
executed by this memo (section 5). No-go for any implicit or partial
re-pin (for example, adopting individual post-drift defaults outside a
governed re-pin change).

Evidence basis for "go":

- The drift window is 166 commits ahead, 0 behind
  (`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` -> `be0029bb`), 3,556
  changed paths, 1,854 on the mandatory surface; 1,525 of those have no
  behavioral effect (U01 buckets: 1,525 / 55 / 274).
- Protocol risk is low and fully characterized: the UDS/DoCAN/DoIP
  determination (U02) found only four behavioral protocol commits
  (`9558c245` UDS 0x14, `f8132091` UDS 0x19, `39212d01` UDS
  configuration, `1719a648` DoIP header graceful-failure), plus the
  robustness-only guard set `a510e3b6` and the late-determined TP-router
  translation `120f5688`; all are imported as drift-labeled tests or
  classified divergences. DoCAN has zero source drift (U01 section 3);
  the 18-commit DoIP series is a mechanical estd-to-ETL migration with
  no wire-behavior change found (U01 section 4, U02).
- All 36 drift-labeled tests pass against the current port (16 U02 +
  11 U03 + 6 U04 + 3 transport reconciliation), and every remaining
  difference is a recorded divergence or re-pin item, not an open
  question.

Evidence basis for "with-scope" (why not an unconditional go):

- 274 mandatory-surface paths carry behavior only meaningful after a
  re-pin, dominated by the new service-middleware framework (119 of the
  130 changed middleware paths are new at the tip), the new
  routing/blob subsystem (`RoutingSystem` at run level 6), the
  tester-address translation rewrite, UDS 0x14/0x19 composition
  routing, and the new STM32 platform tree.
- U03 recorded five register-level gaps in the port's STM32 CAN drivers
  relative to the new upstream adapters (U03 gaps 33-37), none
  host-testable without new production seams; a re-pin tranche must
  decide adopt/decline for each.
- U05 recorded oracle-build and licensing deltas that must move in the
  same governed change: ETL 20.46.2 -> 20.48.1, new ST STM32F4 2.6.11 /
  STM32G4 1.2.6 Apache-2.0 device headers, CMSIS path move, upstream
  minimum C++17 (`e45eee34`), upstream default Rust 1.96.0 (`8bcfb3dc`),
  and the Bazel-primary build layout plus middleware build-time code
  generation (`c4401a13`) that affect how a newer oracle is built.

## 2. Candidate commit

- Candidate re-pin target: `be0029bbb79fe901048a24c2665f2ba854328734`
  ("Add STM32 FDCAN transceiver adapter", authored 2026-06-02).
- Verification: as of the observation date **2026-07-19**,
  `refs/heads/main` of the read-only bare drift mirror
  (`target/drift/openbsw-drift.git`, a clean bare clone of the public
  upstream) resolves to exactly this commit; it is 166 commits ahead of
  and 0 behind the pinned oracle, and the pinned oracle is its merge
  base (no divergence).
- The candidate must be re-verified against the live public upstream at
  re-pin execution time (criterion 5 below); if upstream has advanced,
  the U01 survey method must be re-run over the extension window before
  the new tip may replace this candidate.

## 3. What a re-pin reopens

Row names from `docs/port/status.md`; package IDs from
`docs/port/parity-manifest.json` owner-package columns. "Reopen" means
the row's evidence must be re-verified (and possibly extended) against
the new oracle inside the re-pin tranche - it does not mean the row is
demoted today.

| Re-pin cluster (source record) | Rows reopened | Packages | Estimated verification surface |
|---|---|---|---|
| Service-middleware framework rework: proxy/skeleton/cluster model, queues, allocators, shm simulation, generated bindings, Foo demo (U04 D3, U04-RP-2; 119 new paths) | `bsw.middleware`; `app.referenceApp` for composition | E05-E08, F02-F10 | full workspace suite for `bsw-middleware`; contract cross-check of `ErrorState`/`MessageType`/`MAX_MESSAGE_SIZE` (drift test pins the tip values); decision whether a shm runtime enters scope |
| New routing/blob subsystem, `RoutingSystem` at run level 6 (U04 D2, U04-RP-1) | no existing row - the re-pin tranche must add new parity rows/packages for `libs/bsw/routing` and `libs/bsw/blob`; `app.referenceApp` composition | new package IDs to be allocated; F02-F10 | new row scoping, then full suite for the new crates plus reference-app workflow tests |
| UDS 0x14/0x19 services and composition routing; UdsConfig integrator surface incl. programming-session P2\*=5000 and ecureset constants (U02 DIV-1/2/3, CV-3; U04 D1, U04-RP-3) | `bsw.uds`; `app.referenceApp` | E14-E25, F02-F10 | UDS suites; oracle fixture regeneration for UDS differential pairs; decision on demo-service vs DEM-backed native 0x14/0x19 behavior (currently divergent by design) |
| Tester-address translation at TP-router boundaries (U02 DIV-6, U04 D5, U04-RP-4; `120f5688`) | `bsw.transport`, `bsw.transportRouterSimple`; `app.referenceApp` | E01-E02, E03-E04, F02-F10 | transport suites (drift tests already pin both models); router/composition differential traces if translation is adopted |
| DoIP estd-to-ETL migration (mechanical, but changes upstream internals used for fixture generation) and protocol-send-job pool sizing (U01 section 4, U04-RP-5, survey changelog `330514aa`) | `bsw.doip`; `app.referenceApp` DoIP frontend | E26-E31, F02-F10 | DoIP suites; oracle fixture regeneration for DoIP differential pairs; DoIP fuzz targets incl. `doip_entity`; frontend pool-sizing decision |
| STM32 CAN drivers/transceivers: five register-level gaps (U03 gaps 33-37) and design deltas | `bsw.bsp`, `bsw.cpp2can`, `extension.can-fd`, `bsp.*` G rows | D03, D12-D16, D14, G01-G20 | adopt/decline per gap; any adoption requires target/HIL re-verification (bus-off, overflow, recovery roles) on both boards |
| lwIP netif persistence, Tap lifecycle, ETH config registry re-plumbing (U04 tests, U02 re-pin-dependent `baa9589d`) | `bsw.lwipSocket`, `bsw.cpp2ethernet`, `platform.posix` | D20, D17-D21, F01 | POSIX network integration tests; reference-app echo workflows (drift test exists) |
| EEPROM-size configurability (`1578c18f`, U01 section 8) | `bsw.storage`, `platform.posix` | D06-D11, G11, F01 | storage suites; POSIX default-behavior check (default unchanged upstream) |
| estd/util removals in favor of ETL; StringBufferOutputStream finalization; TimestampProvider/SystemTimer rework (U01 section 9, survey CV-1, U04 `9d3e89a2`) | `bsw.estd`, `bsw.util`, `bsw.logger`, `bsw.timer`, `bsw.lifecycle` | C02-C04, C05-C06, C13-C14, C07-C08, G05, D01-D02 | native-replacement re-review only (no port code shared with upstream); confirm assigned-counterpart mapping in `util-parity.md` still holds at the tip |
| POSIX console stdin/UART robustness (`bd0078d0`, `f381dd46`; survey CV-2) | `bsw.stdioConsoleInput`, `bsw.asyncConsole` | C15-C17, C16-C18 | native-replacement re-review; existing console tests |
| Toolchain/build/licensing: ETL 20.48.1 NOTICE, ST F4/G4 headers, CMSIS path, C++17 minimum, Rust 1.96 default, Bazel layout, middleware codegen (U05 sections 1-5) | no parity row; release evidence and oracle tooling | release-gate scope | update `upstream-baseline.md` bundled-component table; rework `tools/port/openbsw_oracle.ps1` (pinned SHA, C++17/Bazel/codegen build path); re-run SBOM/dependency/license gates |
| DoCAN (zero source drift; U01 section 3, U02) | `bsw.docan` is NOT functionally reopened | E09-E13 | oracle fixture regeneration still re-executes the DoCAN differential pairs as a side effect of moving the oracle |

Aggregate verification surface for the tranche (from `test-guide.md` and
`final-acceptance-2026-07-18.md`): the full fail-closed workspace gate
(fmt, clippy, workspace tests, no_std, dependency/feature/privacy/SBOM/
evidence checks, cargo-deny), regeneration of all seven differential
oracle fixture pairs under `docs/port/oracle-fixtures/` from a rebuilt
oracle checkout at the new pin, the POSIX SocketCAN/vcan and DoIP
integration and differential traces, all 14 fuzz targets (100 runs each,
pinned container), the resource baselines
(`measure_resources -Check` against `resource-limits.json`), and the
authoritative never-skippable HIL physical matrix on both boards (smokes,
destructive storage, queue overflow, bus-off, ten resets, ten-minute
soak) via `run_physical_matrix.ps1` on immutable selected artifacts.

## 4. Criteria that must hold BEFORE the oracle moves

All of the following, in order; each is checkable:

1. U01-U05 documents (including the 2026-07-19 bucket B coverage map and
   reclassification changelog in the survey) are merged unchanged in the
   release history.
2. All 36 drift-labeled tests are green in the standard workspace gate
   (`bsw-uds` 13, `bsw-doip` 3, `bsw-transport` 3, `bsw-can` 6,
   `bsw-bsp-stm32` 5, `bsw-middleware` 3, `openbsw-reference-app` 2,
   `bsw-reference-core` 1).
3. A written oracle checkout and fixture-regeneration plan exists:
   `tools/port/openbsw_oracle.ps1` updated to the new pinned SHA and to
   upstream's C++17 minimum, Bazel-primary layout, and middleware
   build-time code generation (U05 item 3); the plan names which of the
   seven fixture pairs regenerate and how regressions are adjudicated.
4. The `upstream-baseline.md` bundled-component/NOTICE table update is
   prepared per U05 item 1 (ETL 20.48.1, CMSIS path move, ST STM32F4
   2.6.11 and STM32G4 1.2.6 Apache-2.0 rows) and ships in the same
   governed change as the pin move.
5. The upstream tip is re-verified against the live public repository at
   execution time; if it is no longer
   `be0029bbb79fe901048a24c2665f2ba854328734`, the U01 survey method is
   re-run over the extension window first.
6. An HIL bench with both reference boards is available for the full
   physical matrix, since STM32 CAN gap adoptions and any composition
   changes require target re-verification.
7. A dedicated re-pin tranche exists with its own package IDs, evidence
   budget, and row-reopening list (section 3), including the decision
   whether `libs/bsw/routing`/`libs/bsw/blob` enter the mandatory
   surface as new rows; no re-pin work is folded into unrelated
   packages.
8. Divergence dispositions are decided and recorded per item before the
   move: U02 DIV-1/2/3/6, U04 D1-D5, U03 gaps 33-37 (adopt upstream
   behavior, keep documented native difference, or defer with basis).

## 5. This memo moves nothing

- The behavioral oracle remains
  `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`. It was not moved by
  U01-U05 and is not moved by this memo.
- The mandatory parity ledger remains 37/37 rows closed and
  1,092/1,092 package-hours; `docs/port/parity-manifest.json`,
  `docs/port/status.md`, `docs/port/release-notes-2026-07-18.md`, and
  all release/evidence documents are untouched.
- `docs/port/oracle-fixtures/` is unchanged; the read-only oracle
  checkout location `target/oracle/openbsw` is absent and stays absent.
- All drift-labeled tests remain marked NOT pinned-oracle evidence; no
  post-drift default is adopted by this memo.
