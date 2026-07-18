# DoIP parity map - packages E26-E31

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`, module
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

The vectors are grounded in the pinned upstream suites under
`libs/bsw/doip/test/src/doip/common` and `libs/bsw/doip/test/src/doip/server`,
particularly header, payload send-job, UDP connection, vehicle-identification,
and entity-status tests. E26-E31 close every mandatory row above. The Rust
implementation deliberately shares the `bsw-uds` dispatcher with DoCAN rather
than maintaining transport-specific diagnostic state.
