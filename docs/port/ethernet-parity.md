# Ethernet (cpp2ethernet / lwipSocket) parity map — packages D17-D21

Pinned upstream: `be0029bbb79fe901048a24c2665f2ba854328734`, modules
`libs/bsw/cpp2ethernet` and `libs/bsw/lwipSocket`.
(Re-pinned 2026-07-20 from `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; see
the re-pin section below and docs/port/repin-2026-07-20.md.)

Every upstream public surface is assigned below.
POSIX adapters (D18/D19) and the lwIP socket
boundary with its fake backend (D20) live in `crates/bsw-ethernet`.

| Upstream API | Assignment | Rust location | Notes / evidence |
|---|---|---|---|
| `ip/IPAddress.h` | port | `bsw-ethernet::ip::IpAddress` | v4/v6, classification helpers. |
| `ip/IPEndpoint.h` | port | `bsw-ethernet::endpoint::IpEndpoint` | address+port with unset state. |
| `ip/NetworkInterfaceConfig.h` | port | `bsw-ethernet::network_config::NetworkConfig` | v4/v6 config with netmask/gateway/broadcast. |
| `ip/to_str.h` | native replacement | `core::fmt` implementations | Display formatting replaces the helper header. |
| `tcp/socket/AbstractSocket.h` | port | `bsw-ethernet::tcp::TcpSocket` trait | connect/bind/read/write/close semantics; close reasons in `ConnectionCloseReason`. |
| `tcp/socket/AbstractServerSocket.h` | port | `bsw-ethernet::tcp::TcpServerSocket` trait | accept/bind/close. |
| `tcp/IDataListener.h` | port | `bsw-ethernet::tcp::TcpDataListener` | data + connection-closed callbacks. |
| `tcp/IDataSendNotificationListener.h` | port | `bsw-ethernet::tcp::TcpSendListener` | send completion/queued notifications. |
| `tcp/socket/ISocketProvidingConnectionListener.h` | native replacement | accept loop returns owned sockets | Rust ownership replaces the socket-providing callback: `PosixTcpServerSocket::accept` yields a socket value. |
| `tcp/util/*` (iperf/loopback test servers) | excluded | — | test utilities tied to upstream benching; Rust integration tests cover the contracts instead. |
| `udp/DatagramPacket.h` | port | `bsw-ethernet::udp::DatagramPacket` | borrowed payload + endpoint. |
| `udp/socket/AbstractDatagramSocket.h` | port | `bsw-ethernet::udp::UdpSocket` trait | bind/join/send/receive/broadcast. |
| `udp/IDataListener.h` | port | `bsw-ethernet::udp::UdpDataListener` | datagram delivery callback. |
| `udp/IDataSentListener.h` | port | `bsw-ethernet::udp::UdpSendListener` | send completion callback. |
| `udp/util/*` (echo/iperf servers) | excluded | — | superseded by loopback conformance tests. |
| `EthernetLogger.h`, `tcp/TcpLogger.h`, `udp/UdpLogger.h` | native replacement | `bsw-logger` component ids | generic logging replaces per-module logger headers. |
| `cpp2ethernet/mock/*` | native replacement | `bsw-ethernet::lwip::fake` + std loopback tests | deterministic fake stack and real loopback sockets replace gmock doubles. |
| `lwipSocket/tcp/LwipSocket.h`, `LwipServerSocket.h` | port boundary | `bsw-ethernet::lwip::SocketApi` (TCP ops) | pure-Rust boundary; no C types cross into protocol crates. Fake backend proves the contract (D20). |
| `lwipSocket/udp/LwipDatagramSocket.h` | port boundary | `bsw-ethernet::lwip::SocketApi` (datagram ops) | same boundary trait family. |
| `lwipSocket/netif/LwipNetworkInterface.h` | port | `bsw-ethernet::network_interface::NetworkInterface` | lifecycle init/run/shutdown, bounded address-change publication, logging, clean reconfiguration, and restart recovery. |
| `lwipSocket/utils/LwipHelper.h`, `LwipLogger.h`, `TaskAssert.h` | excluded | — | C-side glue; not applicable to the Rust boundary. |

## Re-pin 2026-07-20: dispositions at be0029b (rows D17-D21)

Oracle re-pin ddbcf88a -> `be0029bbb79fe901048a24c2665f2ba854328734`
(be0029b), governed by docs/port/upstream-repin-decision-2026-07-19.md;
tranche record: docs/port/repin-2026-07-20.md. Relevant upstream changes on
the `bsw.lwipSocket`/`bsw.cpp2ethernet` surface (366d993e lwIP netif kept
persistent across lifecycle transitions, b119bf9b Tap restart fix on POSIX,
and the netif config registry surface touched by baa9589d - see the
upstream-drift-survey-2026-07-19.md cpp2ethernet/lwipSocket sections):

- **Observable contract - conforming at the tip.** Network echo (UDP and
  TCP) survives a console reboot and a full shutdown/start cycle. Pinned by
  the promoted baseline test
  `crates/openbsw-reference-app/tests/baseline_be0029b.rs`
  (`baseline_366d993e_network_echo_survives_reboot_and_full_restart`) and
  matching tip behavior.
- **NetworkInterfaceConfigRegistry - recorded native difference.** The
  upstream netif config registry
  (`cpp2ethernet/include/ip/INetworkInterfaceConfigRegistry.h`, plumbed at
  the tip from `EthernetSystem` into `DoIpServerSystem` as
  `netifConfigRegistry` at run level 6) is NOT modeled by the port. The
  Rust POSIX composition configures addresses directly: DoIP discovery uses
  its own address configuration, and the port-side
  `network_interface::NetworkInterface` keeps its own bounded
  `ConfigRegistry` (D21 below) rather than mirroring the upstream registry
  re-plumbing. The observable network behavior above is the parity key; the
  registry surface itself stays a native difference on the D17-D21 rows.

## POSIX adapters (D18/D19)

- `posix::udp::PosixUdpSocket` — std UDP with multicast/broadcast and
  listener integration; loopback + failure conformance in
  `crates/bsw-ethernet/tests/posix_udp.rs`.
- `posix::tcp::PosixTcpSocket` / `PosixTcpServerSocket` — client/server
  lifecycle, partial send with bounded pending buffer, backpressure,
  close reasons, concurrent clients; conformance in
  `crates/bsw-ethernet/tests/posix_tcp.rs`.
- Documented std limitations: bind-before-connect only for wildcard
  (std::net has no bind+connect socket builder), UDP disconnect keeps the
  OS-level peer filter, abort approximates RST via drop-with-unread-data.

## Fake lwIP backend (D20)

`lwip::fake::FakeStack` — allocation-free in-memory implementation of the
socket boundary with connect/accept pairing, bounded buffers with
backpressure, failure injection (refused connects, dropped data, injected
errors), and generation-checked handles; contract suite in
`crates/bsw-ethernet/tests/lwip_fake.rs`.

## Embedded datagram and link extensions (optional embedded-DoIP tranche)

Demonstrated by production DoIP behavior (discovery request/response with
remote-endpoint reporting; unicast/broadcast vehicle announcements; link
loss and recovery), the boundary gained two `no_std`, allocation-free
traits in `lwip.rs`:

- `lwip::DatagramApi` with its own `DatagramId` handle namespace —
  nonblocking UDP create/bind/close, unicast/broadcast/multicast
  transmission, receive with remote endpoint, truncation semantics,
  explicit backend-documented capacities, and stale-handle rejection;
- `lwip::LinkState` — minimal read-only link/interface readiness view
  (`NetworkInterface` remains the lifecycle owner).

`FakeStack` implements both deterministically with fixed capacities
(`UDP_SOCKETS`, `UDP_QUEUE`, `UDP_DATAGRAM_MAX`), observable receive-queue
loss (`dropped_datagrams`), and link-loss injection (`set_link_up`);
contract suite in `crates/bsw-ethernet/tests/datagram_link.rs`.

`posix::stack::PosixSocketStack` (std only) implements
`SocketApi + DatagramApi + LinkState` over the audited POSIX adapters so
host compositions drive the same portable protocol code as embedded
backends; fixed TCP/UDP slot pools, generation-checked handles, bounded
event queue, documented `std::net` limitations; conformance suite in
`crates/bsw-ethernet/tests/posix_stack.rs`. No real lwIP backend is
claimed: a platform adapter must compile and test against a pinned lwIP
implementation before any such claim is made.

## Network lifecycle integration (D21)

`network_interface::NetworkInterface` composes a backend, bounded
`ConfigRegistry`, and logger through `bsw-lifecycle::LifecycleComponent`.
Host tests cover restart after shutdown, live address reconfiguration, injected
startup failure and recovery, queue bounds, idempotence, and invalid
configuration rejection. Platform-independent state remains fixed-capacity and
`no_std`; only the POSIX adapter enables `std`.
