# UDS parity map - packages E14-E25

Pinned upstream: `be0029bbb79fe901048a24c2665f2ba854328734`, module
`libs/bsw/uds`.

| Behavior | Rust implementation | Evidence |
|---|---|---|
| SIDs, NRCs, sessions, bounded job matching | `service_id`, `nrc`, `session`, `dispatcher` | unit suites plus `complete_uds` job-tree scenarios |
| Physical/functional dispatch and transport registration | `Dispatcher` | suppress-response, unsupported-service, and unregister tests |
| Incoming/outgoing connection ownership | `ConnectionPool` | exhaustion, cancellation, processed metadata, stale-token tests |
| P2, P2-star, S3, and response-pending | `TimingState` with injected `Instant` | exact-boundary deterministic timing tests |
| Persistent session/authentication/security state | `DiagnosticState` | fake persistence, delay, attempt-limit, reset-policy tests |
| Core services and deferred reset | `CoreServices` | positive/negative service matrix and processed-response reset test |
| RDBI/WDBI | `DidRegistry` | multi-DID, permissions, security, bounds, application persistence tests |
| RoutineControl and IOControl | `RoutineRegistry`, `IoControlRegistry` | application-owned start/stop/results/control tests; no built-in routines |
| DEM, ReadDTC, ClearDTC | `DemManager`, `read_dtc_information`, `clear_diagnostic_information` | masks, groups, recording disable, and serialization recovery tests |
| Async/resumable/authentication-aware jobs | `AsyncExecutor` | pending/resume/cancel/exhaustion/token-reuse tests |
| Differential scenarios | A05/A07 normalized JSON protocol | `uds-openbsw.json` versus `uds-rust.json` |
| Malformed dispatch/connection fuzzing | `uds_dispatch`, `uds_connections` | mandatory fuzz smoke targets |

## Re-pin 2026-07-20 (`ddbcf88` -> `be0029b`)

The governed oracle re-pin (`docs/port/repin-2026-07-20.md`, executing
`docs/port/upstream-repin-decision-2026-07-19.md`) reopened the E14-E25
surface. Outcome:

- Re-verified rows: all existing E14-E25 evidence holds at the new pin. The
  only behavioral upstream deltas on this surface are the three feature
  commits `9558c245` (0x14), `f8132091` (0x19), and `39212d01` (UDS
  configuration); the remaining `libs/bsw/uds` drift is non-behavioral.
- Promoted tests: the 13 UDS drift tests from the 2026-07-19 tranche are
  now pinned-baseline parity evidence in
  `crates/bsw-uds/tests/baseline_be0029b.rs` (assertions unchanged).
- Adopted - composition routing: the reference composition
  (`bsw-reference-core` `DiagnosticCore`) now dispatches 0x14
  ClearDiagnosticInformation and 0x19 ReadDTCInformation to the DEM-backed
  `bsw-uds` services, matching the upstream tip `UdsSystem::addDiagJobs`
  registration. The composition DEM starts empty and is fed by the
  application (`dem_mut`); upstream's demo 0x19 service instead hard-codes
  DTC `0x123456` status `0x09`, so byte-for-byte differential vectors are
  taken with that DTC stored.
- Adopted - programming-session timing: `10 02` now answers
  `50 02 00 32 13 88` (P2\* = 5000 x 10 ms units), the upstream-tip
  `uds/UdsConfig.h` `PROGRAMMING_DIAG_RESPONSE_PENDING` value; default and
  extended sessions keep `01 f4` (500).
- Recorded native-difference decision 1 (0x19 0x0A): the port's DEM-backed
  service returns the supported-DTC list; the upstream demo service answers
  NRC 0x12. The existing DEM contract is broader than upstream's demo-grade
  service and must not be weakened; the divergence is deliberate and pinned
  by `baseline_f8132091_supported_dtcs_subfunction_diverges_from_upstream_demo`.
- Recorded native-difference decision 2 (0x14 exact group): the port clears
  an exact stored 24-bit group and answers `54`; the upstream demo answers
  NRC 0x31 for any group except 0xFFFFFF. Same rationale (DEM-backed native
  extension kept, upstream service is demo-grade); pinned by
  `baseline_9558c245_known_group_clears_diverging_from_upstream_demo`.
- Fixtures: `uds-openbsw.json`/`uds-rust.json` moved to the new pin and
  gained agreeing 0x14/0x19 differential records; the pre-re-pin pair is
  preserved under `oracle-fixtures/historical-ddbcf88/`.

The service and dispatcher vectors are grounded in the pinned upstream suites
under `libs/bsw/uds/test/src/uds`, notably the base/job-tree, connection,
session-control, tester-present, security-access, data-identifier,
routine-control, DTC, and async tests. Rust uses fixed pools and explicit
generation tokens instead of upstream intrusive ownership. The parity contract
is observable request/response data, dispatch ordering, ownership transitions,
timing boundaries, error codes, and recovery behavior.
