# Drift-derived UDS/DoCAN/DoIP vectors - 2026-07-19 (U02)

Follow-up package U02 from `upstream-drift-2026-07-18.md`. The release
parity baseline remains the pinned upstream commit
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`. The drift tip surveyed here is
`be0029bbb79fe901048a24c2665f2ba854328734` (166 commits ahead of the pin).

All tests added by this package are drift-derived: they encode
post-drift upstream-demonstrated behavior for comparison. They are kept in
clearly separated files (`tests/drift_be0029b.rs` per crate), are NOT part
of the pinned-oracle evidence, and do not move the release baseline. Any
decision to adopt post-drift defaults belongs to the U06 re-pin decision.

## Commit determinations

The subject/area routing yields 25 unique commits touching UDS,
DoCAN/ISO-TP, or DoIP (the U02 brief's "5 UDS + 1 DoCAN + 18 DoIP" count
groups `57d27137` under build/toolchain and omits `4beb3f8b`; both are
included and determined here). Every diff was read from the verified bare
drift checkout.

| Commit | Subject | Area | Determination |
|---|---|---|---|
| `9558c245` | Add UDS service ClearDiagnosticInformation (0x14) | UDS | BEHAVIORAL - new service with demo group semantics and test vectors |
| `f8132091` | feat(uds): add ReadDTCInformation service (0x19) | UDS | BEHAVIORAL - new service with demo sub-function semantics and test vectors |
| `39212d01` | User specific configuration of module uds | UDS | BEHAVIORAL - timing constants moved to integrator-owned `uds/UdsConfig.h`; programming-session P2\*max response changed 3000 -> 5000 (10 ms units) |
| `4beb3f8b` | Enhance log when adding diagnostic jobs | UDS | mechanical - debug-log lines only in `AbstractDiagJob::addAbstractDiagJob`/`removeAbstractDiagJob`; logs are outside the parity contract (oracle protocol compares logs only under `compare_logs`) |
| `b3fb00f0` | Update run_process() implementation for udstool based tests | UDS | mechanical - Python pyTest harness (`test/pyTest`) process handling only; no library behavior |
| `57d27137` | Migrate transport, transportRouterSimple and uds to Bazel | UDS | mechanical - BUILD.bazel additions only |
| `c9ec30bc` | Migration of bsw libs docan, doip, logger_integration | DoCAN (+DoIP) | mechanical - BUILD.bazel additions only; the single DoCAN-subject drift commit carries no DoCAN behavior change |
| `8d18b1a1` | Change estd::slice to etl::span in DoIP | DoIP | mechanical - type migration; one internal null-slice construction replaced by a header-buffer subspan with identical observable behavior |
| `0dd375fd` | Switch bitset from estd to ETL in doip server | DoIP | mechanical - member type swap in one header |
| `118ea4f2` | Switch memory fns from estd to ETL in doip server | DoIP | mechanical - `estd::memory` -> `etl::mem_*` with added compile-time size assertions; byte-for-byte identical output |
| `bc20e058` | Switch object pool from estd to ETL in doip server | DoIP | mechanical - send-job/pool restructure to CRTP templates; identical wire behavior |
| `895adbe9` | Switch vector from estd to ETL in doip server | DoIP | mechanical - container type swap |
| `2cd7e37e` | Switch array from estd to ETL in doip server | DoIP | mechanical - test-code array type swap only |
| `76b31e1d` | Switch by_ref from estd to ETL in doip server | DoIP | mechanical - reference-wrapper swap |
| `a025a75c` | Switch optional from estd to ETL in doip server | DoIP | mechanical - optional type swap |
| `b99b1de6` | Switch lists from estd to ETL in doip server | DoIP | mechanical - intrusive list migration (swap -> splice), same ordering semantics |
| `f6049da5` | Switch variant from estd to ETL in doip server | DoIP | mechanical - variant API swap; the `needsDelay` rewrite (`holds_alternative` + `get`) preserves the pinned semantics of estd's typed variant comparison |
| `1719a648` | Switch assert from std to ETL in doip server | DoIP | BEHAVIORAL - `DoIpSendJobHelper::prepareHeaderBuffer` now returns an empty span for an undersized buffer instead of asserting; upstream test vector updated (7-byte buffer -> empty result) |
| `29ba5c06` | Switch be-helpers from estd to ETL in doip server | DoIP | mechanical - big-endian helper type swap; identical wire bytes |
| `721cbb6b` | Switch constructor from estd to ETL in doip server | DoIP | mechanical - constructor plumbing |
| `a510e3b6` | Switch function from estd to ETL in doip server | DoIP | BEHAVIORAL (robustness only) - unset VIN/GID/EID, payload-discarded, and detach callbacks are now guarded with `is_valid()` because default `etl::delegate` asserts where default `estd::function` no-opped |
| `a67de092` | Switch algorithm from std to ETL in doip server | DoIP | mechanical - `std::min`/`estd::min` -> `etl::min` |
| `acc49143` | Switch ordered_map from estd to ETL in doip server | DoIP | mechanical - OEM handler map type swap |
| `6c3dc32b` | Partially migrate Tests from estd::slice to etl::span | DoIP | mechanical - test-code type migration only |
| `baa9589d` | Switch remaining estd usages in DoIP/ETH to ETL | DoIP | mechanical for DoIP - signal/slot re-plumbing of the network-interface-config registry; the restored config-changed emission is in the lwIP/ETH layer, outside the DoIP port surface (see re-pin-dependent items) |

### Late-imported commit (2026-07-19 reconciliation)

The U01 survey routes the `libs/bsw/transport` and
`libs/bsw/transportRouterSimple` tester-address-translation paths to U02,
but the commit that changes them - `120f5688`, "Add tester address
translation to referenceApp TP-Router" - is assigned to the
reference-application area in `target/drift/analysis/area-map.tsv`, so the
UDS/DoCAN/DoIP subject sweep above missed it. Its diff was read in full
from the drift checkout and it is determined here:

| Commit | Subject | Area | Determination |
|---|---|---|---|
| `120f5688` | Add tester address translation to referenceApp TP-Router | transport/TP-router (reference-application area) | BEHAVIORAL - bounded 2-byte <-> 1-byte tester-address table (`0x0EF0..0x0EFB` <-> `0x00F0..0x00FB`) in integrator-owned transport configuration; the simple TP-router rewrites source/target addresses at 1-byte-bus boundaries; converter API renamed to 2-byte/1-byte terminology; new upstream router round-trip unit tests |

## Imported vectors

All imported tests pass against the current Rust port
(imported-and-passing) unless listed under Divergences.

### crates/bsw-uds/tests/drift_be0029b.rs (13 tests)

From `f8132091` (ReadDTCInformation 0x19):

| Test | Vector |
|---|---|
| `drift_f8132091_report_dtc_by_status_mask_returns_demo_golden_bytes` | `19 02 FF` -> `59 02 FF 12 34 56 09` (upstream demo DTC golden bytes) |
| `drift_f8132091_report_by_status_mask_without_mask_is_invalid_format` | `19 02` -> NRC 0x13 |
| `drift_f8132091_snapshot_subfunction_not_supported` | `19 04 FF` -> NRC 0x12 |
| `drift_f8132091_extended_data_subfunction_not_supported` | `19 06 FF` -> NRC 0x12 |
| `drift_f8132091_unknown_subfunction_not_supported` | `19 00 FF` -> NRC 0x12 |
| `drift_f8132091_supported_dtcs_subfunction_diverges_from_upstream_demo` | `19 0A FF` - divergence, see below |

From `9558c245` (ClearDiagnosticInformation 0x14):

| Test | Vector |
|---|---|
| `drift_9558c245_clear_all_group_returns_positive_response` | `14 FF FF FF` -> `54`, all DTCs cleared |
| `drift_9558c245_unknown_group_returns_request_out_of_range` | `14 12 34 56` (group not stored) -> NRC 0x31 |
| `drift_9558c245_powertrain_group_returns_request_out_of_range` | `14 00 00 00` -> NRC 0x31 |
| `drift_9558c245_short_request_is_invalid_format` | `14 FF FF` -> NRC 0x13 |
| `drift_9558c245_known_group_clears_diverging_from_upstream_demo` | `14 12 34 56` (group stored) - divergence, see below |

From `39212d01` (user-specific UDS configuration):

| Test | Vector |
|---|---|
| `drift_39212d01_default_session_response_carries_configured_p2_values` | `10 01` -> `50 01 00 32 01 F4` (P2 = 50 ms, P2\* = 500 x 10 ms; unchanged pin -> tip) |
| `drift_39212d01_programming_session_post_drift_pending_is_expressible` | `10 02` -> `50 02 00 32 13 88` with post-drift configuration - divergence context, see below |

### crates/bsw-doip/tests/drift_be0029b.rs (3 tests)

From `1719a648` (assert -> graceful failure in send-job header preparation):

| Test | Vector |
|---|---|
| `drift_1719a648_header_prepare_golden_bytes_for_diagnostic_ack` | header(version 0x03, type 0x8002, length 0x210) -> `03 FC 80 02 00 00 02 10` (upstream `DoIpSendJobTest.TestPrepareHeaderBuffer` success case) |
| `drift_1719a648_undersized_header_buffer_fails_gracefully` | 7-byte destination -> recoverable error, no abort (post-drift upstream returns an empty span; pinned upstream asserted) |
| `drift_1719a648_payload_overflow_fails_gracefully_and_exact_fit_encodes` | header-fits/payload-does-not -> recoverable error; exact fit encodes the full frame |

### crates/bsw-transport/tests/drift_be0029b.rs (3 tests, added 2026-07-19)

From `120f5688` (tester address translation in the TP-router; late-imported
determination above). The Rust `SimpleRouter` deliberately routes full
16-bit logical addresses without a per-boundary translation table, so these
tests pin the current pinned-parity behavior against the upstream
round-trip vectors and record the post-drift per-boundary rewrite as
upstream-behavior-change (divergence 6 below / U04 divergence D5):

| Test | Vector |
|---|---|
| `drift_120f5688_router_boundary_round_trip_forwards_addresses_unchanged` | upstream `requestFromETH_responseFromSelfDiag_roundTrip`: request 0x0ECD -> 0x0006 from ETH_0 forwarded to SELFDIAG, reply 0x0006 -> tester back to ETH_0; port delivers addresses unchanged at both boundaries (post-drift upstream delivers source 0x00F0 on SELFDIAG); end-to-end the tester receives the reply at its own 2-byte address on ETH_0, identical to the post-drift upstream round trip |
| `drift_120f5688_selfdiag_reply_without_declared_return_route_is_not_routed` | upstream `messageReceived_fromSelfDiag_noReplyTarget` (RECEIVED_ERROR): declarative analog - no declared SELFDIAG return route -> `NoRoute`, nothing delivered |
| `drift_120f5688_tester_address_translation_is_expressible_as_integrator_config` | upstream `TesterAddressTest.cpp` + referenceApp table: all 12 pairs `0x0EF0..0x0EFB` <-> `0x00F0..0x00FB` round-trip through prefix-guarded converters built on the public `LogicalAddress` API; passthrough for ECU 0x0006, functional 0x00DF, unmapped 0x0EE0/0x00FC; bounded-set membership replaces the pinned tester ranges (0x0EF5 in both models, 0x0EE5 only in the pinned ranges) |

### crates/bsw-docan

No test file added: the single DoCAN-subject drift commit (`c9ec30bc`) is
Bazel-only and demonstrates no transport vector.

## Divergences

None of the divergences below is a bug-in-port; no production code was
changed. In each case the pinned baseline `ddbcf88` either has no such
behavior at all or demonstrates the same behavior as the Rust port.

1. ReadDTCInformation 0x0A (reportSupportedDTCs)
   - Post-drift upstream demo: NRC 0x12 (recognized, not supported).
   - Pinned upstream: no 0x19 service exists in `libs/bsw/uds`.
   - Rust port: DEM-backed native service supports 0x0A and returns the
     supported-DTC list (documented native DEM extension,
     `uds-parity.md`).
   - Classification: upstream-behavior-change (new upstream demo service
     narrower than the port's native extension). Test asserts the current
     pinned-parity Rust behavior and records the post-drift difference in
     its comment.

2. ClearDiagnosticInformation with a stored non-all group
   - Post-drift upstream demo: NRC 0x31 for every group except 0xFFFFFF.
   - Pinned upstream: no 0x14 service exists in `libs/bsw/uds`.
   - Rust port: clears an exact stored 24-bit group/code and answers
     positively (documented native DEM extension).
   - Classification: upstream-behavior-change / deliberate-native-
     difference. Test asserts the current pinned-parity Rust behavior and
     records the post-drift difference in its comment.

3. Programming-session P2\*max in the DiagnosticSessionControl response
   - Pinned upstream: `EXTENDED_DIAG_RESPONSE_PENDING` = 3000 (wire bytes
     `0B B8`).
   - Post-drift upstream: `PROGRAMMING_DIAG_RESPONSE_PENDING` = 5000
     (wire bytes `13 88`), now integrator-owned configuration in
     `uds/UdsConfig.h`.
   - Rust port: P2/P2\* are already integrator configuration
     (`CoreServices::new` parameters) with no built-in per-session
     switch; the reference composition uses the default 50/500 pair. The
     per-session automatic switch is a deliberate-native-difference; the
     post-drift programming values are expressible through configuration
     (demonstrated by the test). Adopting 5000 as a shipped default is a
     U06 re-pin decision.

4. Undersized send-buffer handling in DoIP header preparation
   - Pinned upstream: `estd_assert` (abort) on a buffer smaller than the
     8-byte DoIP header.
   - Post-drift upstream: returns an empty span (graceful).
   - Rust port: `Packet::encode` returns `CodecError::OutputTooSmall`
     (graceful, recoverable). The port already matches the post-drift
     robustness contract; recorded as imported-and-passing, not a
     divergence requiring action.

5. Unset callback guards (`a510e3b6`)
   - Post-drift upstream guards optional VIN/GID/EID, payload-discarded,
     and detach delegates with `is_valid()` because default
     `etl::delegate` asserts.
   - Rust port: covered by construction - optional callbacks are typed
     `Option`/trait objects; an "invalid delegate" state is
     unrepresentable. No test needed; recorded here for completeness.

6. Tester-address translation at TP-router bus boundaries (`120f5688`;
   added 2026-07-19)
   - Post-drift upstream: the simple TP-router rewrites source/target
     addresses through the bounded 2-byte <-> 1-byte tester table at
     1-byte-bus boundaries.
   - Pinned upstream: no translation table; range-based tester checks
     only.
   - Rust port: `SimpleRouter` matches and forwards full 16-bit logical
     addresses; the end-to-end round-trip observable matches post-drift
     upstream (test above), and the table semantics are expressible as
     integrator configuration on the public `LogicalAddress` API.
   - Classification: upstream-behavior-change, re-pin-dependent for the
     per-boundary rewrite itself (same record as U04 divergence D5 in
     `composition-drift-review-2026-07-19.md`).

## Production fixes

None. No divergence was classified bug-in-port; the pinned baseline never
demonstrates a behavior the Rust port lacks among these 25 commits.

## Re-pin-dependent items (no test possible at the pinned scope)

- `baa9589d` restores a config-changed signal emission in the lwIP/ETH
  network-interface layer (`lwipSocket/netif/LwipNetworkInterface`) and
  reshapes the `NetworkInterfaceConfigRegistry` around bounded slot
  capacity. The Rust DoIP port does not model the lwIP netif config
  registry (POSIX/embedded discovery uses its own address configuration),
  so this is re-pin-dependent for any future ETH-layer parity work.
- `b3fb00f0` udstool pyTest harness changes concern the upstream
  container-based integration test runner, which the port replaces with
  its own oracle/differential harness; re-pin-dependent only if the
  upstream pyTest suite is ever adopted as an oracle source.

## Verification

- `cargo test -p bsw-uds -p bsw-doip --all-features --locked`: all
  suites pass (13 new drift tests in `bsw-uds`, 3 in `bsw-doip`; 0
  failed, 0 ignored).
- `cargo test -p bsw-transport --all-features --locked` (2026-07-19
  reconciliation): all suites pass, including the 3 new drift tests in
  `crates/bsw-transport/tests/drift_be0029b.rs`.
- New test files are rustfmt-clean.
- `python tools/port/check_privacy.py` passes.

These drift-derived tests are additional evidence only: the pinned-oracle
fixtures, the parity manifest, and the release baseline are unchanged.

## Changelog

- 2026-07-19 (initial): U02 package delivered (25-commit determination,
  16 drift tests, divergences 1-5).
- 2026-07-19 (reconciliation): late-imported determination for
  `120f5688` added (missed by the original subject sweep because the
  commit is routed to the reference-application area);
  `crates/bsw-transport/tests/drift_be0029b.rs` added (3 tests);
  divergence 6 recorded (cross-reference to U04 divergence D5). See the
  bucket B coverage map appended to
  `upstream-drift-survey-2026-07-19.md`.
