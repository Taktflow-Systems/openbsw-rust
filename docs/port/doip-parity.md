# DoIP parity map - packages E26-E31

Pinned upstream: `be0029bbb79fe901048a24c2665f2ba854328734`, module
`libs/bsw/doip`.

| Behavior | Rust implementation | Evidence |
|---|---|---|
| Generic header and inverse-version validation | `DoIpHeader`, `Packet` | header suite and malformed packet tests |
| Standard payload type codecs | `payload::Payload` | all-payload round-trip and upstream golden-byte tests |
| VIN/EID vehicle identification | `VehicleIdentification`, `DiscoveryEntity` | match/non-match request tests |
| Announcements and discovery timing | `AnnouncementSchedule`, `DiscoveryEntity::poll_announcement` | injected-instant exact-boundary tests |
| Entity status and diagnostic power mode | `EntityStatus`, discovery request handling | codec and service tests |
| POSIX UDP discovery | `discovery::posix::PosixDiscoveryService` | two-client loopback test over `bsw-ethernet` POSIX UDP |
| Routing activation, connection limits, alive checks, and inactivity teardown | `connection::ServerConnection`, `server::ServerTransportLayer` | injected-clock TCP server suite with upstream golden bytes |
| Diagnostic 0x8001 receive and 0x8002/0x8003 acknowledgement | `diagnostic::DiagnosticMessageHandler` | fragmented receive, source/target, size, pool-exhaustion, listener-error, and ACK-order tests |
| Fixed-pool transport receive bridge | `diagnostic::PoolDiagnosticGateway` over `bsw_transport::pool::MessagePool` | generation/release and complete UDS request tests |
| Bounded diagnostic transmit jobs | `diagnostic::DiagnosticSender` | route lookup, capacity, stale-token, and upstream wire-byte tests |
| Shared UDS over DoIP | `bsw-uds` `DiagRouter` behind the diagnostic gateway | `uds_over_doip_loopback_ack_precedes_shared_uds_response` |
| POSIX lifecycle composition | `entity::PosixDoIpEntity` | real UDP/TCP discovery, activation, shared-UDS diagnostics, malformed client, shutdown, and restart loopback scenario |
| Pinned upstream differential traces | routing activation, alive check, and diagnostic packet fixture pairs | `compare_oracle.py` normalizes and compares all three pairs in CI |
| Malformed input and state-machine fuzzing | `doip_header`, `doip_payload`, `doip_tcp`, `doip_diagnostic` | mandatory bounded fuzz smoke targets; TCP and diagnostic targets exercise fragmented and arbitrary packet sequences |

## Optional hardware-independent embedded tranche (post-release)

These rows extend the closed mandatory table above; they change no
mandatory totals. See `embedded-doip-2026-07-19.md` for the full
completed/not-completed boundary of this tranche.

| Behavior | Rust implementation | Evidence |
|---|---|---|
| Portable no_std live DoIP entity (protocol, discovery, announcements, admission, alive checks, timeouts, diagnostics, partial send, backpressure, link recovery, lifecycle) | `entity::DoIpEntity` over `bsw-ethernet` `SocketApi`/`DatagramApi`/`LinkState` | `embedded_entity.rs` deterministic suite (34 scenarios) |
| POSIX entity as adapter over the portable core | `entity::posix::PosixDoIpEntity` + `PosixSocketStack` | unchanged `posix_entity.rs` plus `entity_equivalence.rs` |
| Deterministic embedded composition with shared production UDS state and lifecycle control | `bsw-reference-core::DiagnosticCore` + `bsw-lifecycle` + `FakeStack` | `embedded_composition.rs` (DoCAN/DoIP shared state, restart, link loss) |
| Cross-cycle fragmented diagnostic receive | `diagnostic::DiagnosticReceiveState` suspend/resume | fragmented header/payload and pool-exhaustion scenarios |
| Host/embedded protocol equivalence | one portable core, two backends | `entity_equivalence.rs` transcript comparison |
| Entity-level robustness fuzzing | `doip_entity` bounded fuzz target | sanitizer-backed Linux fuzz smoke |

Known preserved limitation: a routing activation and a diagnostic message
arriving in the same receive chunk still validate the diagnostic source
against the chunk-start snapshot (parity with the pre-tranche POSIX
entity); activation and diagnostics in separate receive cycles are
unaffected.

The vectors are grounded in the pinned upstream suites under
`libs/bsw/doip/test/src/doip/common` and `libs/bsw/doip/test/src/doip/server`,
particularly header, payload send-job, UDP connection, vehicle-identification,
and entity-status tests. E26-E31 close every mandatory row above. The Rust
implementation deliberately shares the `bsw-uds` dispatcher with DoCAN rather
than maintaining transport-specific diagnostic state.

## Re-pin 2026-07-20: oracle baseline moved `ddbcf88` -> `be0029b`

Governed re-pin tranche (scope: `upstream-repin-decision-2026-07-19.md`;
tranche record: `repin-2026-07-20.md`). The pinned baseline for the E26-E31
surface (and the transport rows E01-E04, see the transport-boundary section
below) is now `be0029bbb79fe901048a24c2665f2ba854328734`. The 2026-07-19
drift tests were promoted to pinned-baseline parity evidence
(`crates/bsw-doip/tests/baseline_be0029b.rs`,
`crates/bsw-transport/tests/baseline_be0029b.rs`); no assertion changed. The
four differential fixture pairs (`docan`, `doip-alive`, `doip-diagnostic`,
`doip-routing`) were re-verified against the upstream checkout at `be0029b`
and re-pinned in place; every record byte is unchanged, and all four
`compare_oracle.py` runs pass. The pre-re-pin fixtures are preserved under
`oracle-fixtures/historical-ddbcf88/`.

Dispositions recorded by this re-pin:

1. **DoIP undersized-header graceful failure** (`1719a648`): at `be0029b`,
   `DoIpSendJobHelper::prepareHeaderBuffer` returns an empty span for an
   undersized destination instead of asserting. The port already conforms:
   `Packet::encode` reports `CodecError::OutputTooSmall` (recoverable, no
   panic). Recorded as conforming baseline behavior; evidence
   `baseline_1719a648_*` tests in
   `crates/bsw-doip/tests/baseline_be0029b.rs`, golden header bytes
   `03 FC 80 02 00 00 02 10` re-verified against the tip
   `DoIpSendJobHelperTest.cpp`.
2. **Unset-callback guards** (`a510e3b6`): baseline upstream guards optional
   VIN/GID/EID, payload-discarded, and detach delegates with `is_valid()`.
   Native difference by construction: the Rust design types optional
   callbacks as `Option`/trait objects, so an invalid-delegate state is
   unrepresentable; no test is possible or needed.
3. **DoIP send-job pool sizing** (`330514aa`, "Correct the number of
   protocol send jobs"): upstream corrected the reference-app frontend's
   `DoIpServerTransportConnectionPool` protocol-send-job pool from being
   mistakenly sized with `NUM_DIAGNOSTICSENDJOBS` (6) to
   `NUM_PROTOCOLSENDJOBS` (8) (`appConfig.h`: diagnostic 6, protocol 8;
   shared frontend-wide pools). Recorded as an explicit structural native
   difference, no production change: the Rust `DoIpEntity` has a single
   per-composition const-generic `SEND_JOBS` diagnostic send-job pool
   (default 6, matching upstream's diagnostic pool) and no separate
   protocol-send-job pool to size — protocol responses (routing activation,
   alive check, acks) are emitted through per-connection pending-send slots,
   so upstream's corrected constant has no structural counterpart (U01
   survey reclassification, U04-RP-5).
4. **estd-to-ETL DoIP migration** (18 commits, `6c3dc32b`..`baa9589d` plus
   `c21dcc3a`/`c9ec30bc`): mechanical container/utility migration with no
   wire-behavior change found (U01 section 4, U02). Affirmed here by fixture
   re-verification at `be0029b`: the routing-activation, alive-check, and
   diagnostic-ack byte layouts in the tip sources
   (`DoIpServerConnectionHandler.cpp`, `DoIpServerTransportMessageHandler.cpp`,
   `DoIpConstants.h`) still produce exactly the recorded fixture bytes, and
   all four differential pairs pass `compare_oracle.py` unchanged.

### Transport-boundary disposition (rows E01-E04, `bsw.transport` / `bsw.transportRouterSimple`)

These rows have no dedicated parity document (their evidence lives in
`parity-manifest.json` and the `bsw-transport` suites), so the re-pin
disposition for the transport boundary is recorded here:

5. **Tester-address translation** (`120f5688`; formerly divergence D5 /
   drift divergence 6): at `be0029b` the upstream simple TP-router rewrites
   source/target addresses through a bounded 2-byte <-> 1-byte
   tester-address table (`0x0EF0..0x0EFB` <-> `0x00F0..0x00FB`) at
   1-byte-bus boundaries. Decision: KEEP the port's full-16-bit
   logical-address forwarding as a recorded deliberate native difference.
   Rationale: the end-to-end round-trip observable (reply reaches the tester
   at its own 2-byte address) is identical to upstream, and the bounded
   translation table remains expressible as integrator configuration on the
   public `LogicalAddress` API — proven by the promoted test
   `baseline_120f5688_tester_address_translation_is_expressible_as_integrator_config`
   in `crates/bsw-transport/tests/baseline_be0029b.rs`. No production code
   change.
