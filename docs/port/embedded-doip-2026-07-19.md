# Hardware-independent embedded DoIP tranche - 2026-07-19

This is an optional post-release expansion. The mandatory release remains
1,092/1,092 package-hours and 37/37 parity rows; nothing in this tranche
changes those totals or the pinned upstream baseline
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`.

## Completed in this tranche (hardware-independent only)

- **Portable no_std live DoIP entity** — `bsw-doip::entity::DoIpEntity`
  owns discovery, scheduled announcements, admission, routing activation,
  alive-check arbitration, timeouts, diagnostic bridging, ordered
  partial-send/backpressure handling, link-loss recovery, and
  start/enable/poll/stop lifecycle. It compiles with
  `--no-default-features`, uses no `libc`, `std::net`, threads, heap
  collections, POSIX handles, or platform types, and passes the
  repository allocation detector (`tools/port/check_no_std.py`).
- **Generic bounded TCP/UDP boundary** — reuses `bsw-ethernet`
  `lwip::SocketApi` for TCP and adds only what production DoIP behavior
  demonstrated: `lwip::DatagramApi` (nonblocking UDP bind/close,
  unicast/broadcast transmission, receive with remote endpoint,
  stale-handle rejection, documented capacities) and `lwip::LinkState`
  (read-only link readiness). Ownership, callback, staleness, and
  capacity rules for future platform adapters are documented on the
  traits.
- **Deterministic embedded composition** — the portable entity over the
  deterministic `FakeStack` backend (now with a bounded UDP pool and link
  injection), an injected clock, `bsw-lifecycle` control, and the
  production `bsw-reference-core::DiagnosticCore`, so DoCAN and DoIP
  demonstrably share one UDS state and dispatcher. No second DoIP, UDS,
  or diagnostic implementation exists for tests.
- **POSIX preserved as an adapter** — `PosixDoIpEntity` keeps its
  package E30 public API and behavior but is now a thin adapter over the
  portable core through `bsw-ethernet::posix::PosixSocketStack`
  (`SocketApi + DatagramApi + LinkState` over the audited `std::net`
  adapters). The pre-existing POSIX integration test passes unmodified.
- **Cross-cycle fragmented receive** — `DiagnosticReceiveState` with
  handler `resume`/`suspend` keeps fragmented diagnostic messages intact
  across receive cycles and releases pool allocations on connection
  teardown (previously a latent per-read-handler limitation).
- **Deterministic behavioral evidence** — 34-scenario suite
  (`crates/bsw-doip/tests/embedded_entity.rs`) covering discovery,
  announcements at exact deadlines, entity status, power mode, accept,
  fragmentation, multi-frame receive, activation success/rejection,
  address validation, positive/negative acknowledgement, shared UDS,
  alive-check success/timeout, remote/local close, abort/reconnect, link
  down/up, stop/start, partial sends, receive pressure, malformed
  headers, unsupported payload types, oversized declared payloads,
  socket/connection/receive-message/send-job exhaustion, stale handles,
  exact timeout boundaries, timer wraparound, and two identical isolated
  runs; plus composition tests
  (`embedded_composition.rs`) and protocol-equivalence evidence
  (`entity_equivalence.rs`).
- **Explicit bounded capacities** — const-generic `SOCKETS`,
  `RX_MESSAGES`, `SEND_JOBS`, `PAYLOAD`; named constants `RECV_CHUNK`,
  `DISCOVERY_BUFFER`, `UDP_POLL_BUDGET`; backend constants
  (`UDP_SOCKETS`, `UDP_QUEUE`, `UDP_DATAGRAM_MAX`, wire/event queues);
  compile-time asserts on minimum capacities; test-enforced size bounds
  with representative sizes recorded in the tranche evidence.
- **Robustness** — new bounded `doip_entity` fuzz target (arbitrary
  interleavings of TCP bytes, UDP datagrams, time, link toggles,
  teardown, restarts, with pool-leak assertions) joins the existing
  `doip_header`, `doip_payload`, `doip_tcp`, `doip_diagnostic` targets in
  the sanitizer-backed Linux fuzz run (now 14 targets).

## NOT completed - platform-specific work this tranche does not claim

- MCU target and toolchain configuration for any Ethernet-capable device;
- startup code, vector table, linker script, and memory layout;
- clock, timer, and interrupt integration on a physical device;
- Ethernet MAC and DMA driver;
- MDIO management and PHY driver;
- cache, alignment, and coherency handling for descriptors/buffers;
- a compiled and tested lwIP (or other network-stack) backend — the
  lwIP-facing contract exists, but no real backend is claimed until one
  is built and tested against a pinned lwIP implementation;
- board pinmux, reset lines, reference clocks, and PHY addressing;
- watchdog, reset, fault, MPU, and ECC integration for the network path;
- physical Ethernet/DoIP hardware-in-the-loop testing;
- physical recovery, fault-injection, and soak campaigns;
- exact-artifact embedded resource measurements (flash/RAM of a real
  embedded DoIP image).

No physical embedded platform is proven by this tranche. The mandatory
release note that "embedded DoIP is excluded because these boards have no
supported Ethernet PHY path" still stands for the F413/G474 boards.

## Known preserved limitation

A routing activation and a diagnostic message arriving in one receive
chunk validate the diagnostic source against the chunk-start snapshot
(behavioral parity with the pre-tranche POSIX entity). Separate receive
cycles are unaffected. Recorded in `doip-parity.md`.

## Evidence

`docs/test-evidence/samples/embedded-doip-2026-07-19.json` records the
verification commands and totals for this tranche. Representative type
sizes (host x86-64 build, informative, test-enforced upper bounds 16 KiB
and 32 KiB): `DoIpEntity` with 6 connection slots, 6 receive messages, 6
send jobs, and 64-byte payloads is 8,880 bytes; `FakeStack<16, 512, 32>`
is 11,568 bytes. Exact-artifact embedded sizes remain future platform
work.
