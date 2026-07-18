# Ethernet (cpp2ethernet / lwipSocket) parity map — packages D17-D21

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`, modules
`libs/bsw/cpp2ethernet` and `libs/bsw/lwipSocket`. Every upstream public
surface is assigned below. POSIX adapters (D18/D19) and the lwIP socket
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

## Network lifecycle integration (D21)

`network_interface::NetworkInterface` composes a backend, bounded
`ConfigRegistry`, and logger through `bsw-lifecycle::LifecycleComponent`.
Host tests cover restart after shutdown, live address reconfiguration, injected
startup failure and recovery, queue bounds, idempotence, and invalid
configuration rejection. Platform-independent state remains fixed-capacity and
`no_std`; only the POSIX adapter enables `std`.
