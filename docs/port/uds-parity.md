# UDS parity map - packages E14-E25

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`, module
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

The service and dispatcher vectors are grounded in the pinned upstream suites
under `libs/bsw/uds/test/src/uds`, notably the base/job-tree, connection,
session-control, tester-present, security-access, data-identifier,
routine-control, DTC, and async tests. Rust uses fixed pools and explicit
generation tokens instead of upstream intrusive ownership. The parity contract
is observable request/response data, dispatch ordering, ownership transitions,
timing boundaries, error codes, and recovery behavior.
