//! POSIX UDP adapter over [`std::net::UdpSocket`].
//!
//! [`PosixUdpSocket`] implements the crate's [`UdpSocket`] trait with
//! non-blocking polling semantics, following the state model of upstream
//! `udp::AbstractDatagramSocket`:
//!
//! * fresh / closed — not bound, [`is_closed`](UdpSocket::is_closed) is `true`
//! * bound — after a successful [`bind`](UdpSocket::bind)
//! * connected — after [`connect`](UdpSocket::connect) associated a remote
//!   endpoint (enables [`send`](UdpSocket::send))
//!
//! Receive operations never block: when no datagram is pending they return
//! immediately with a zero count.

use std::io;
use std::net::{SocketAddr, UdpSocket as StdUdpSocket};

use crate::endpoint::IpEndpoint;
use crate::ip::IpAddress;
use crate::udp::{DatagramPacket, UdpDataListener, UdpError, UdpSendListener, UdpSocket};

use super::{addr_or_unspecified, from_std_ip, to_std_ip};

/// UDP socket adapter backed by a non-blocking [`std::net::UdpSocket`].
///
/// See the [module documentation](self) for the state model. All I/O errors
/// from the operating system are mapped to [`UdpError::NotOk`]; the typed
/// entry points such as [`join_multicast_io`](Self::join_multicast_io) expose
/// the underlying [`std::io::Error`] where callers need to distinguish causes.
#[derive(Debug, Default)]
pub struct PosixUdpSocket {
    inner: Option<StdUdpSocket>,
    remote: Option<SocketAddr>,
}

impl PosixUdpSocket {
    /// Create a new, unbound socket.
    pub fn new() -> Self {
        Self::default()
    }

    /// Join an IPv4 (or, with the `ipv6` feature, IPv6) multicast group,
    /// returning the raw OS error on failure.
    ///
    /// For IPv4 the membership is added on the interface identified by
    /// `interface`. For IPv6 the group is joined on the default interface
    /// (index 0) because the address-based interface selection is an
    /// IPv4-only concept.
    ///
    /// The socket must be bound first.
    pub fn join_multicast_io(&mut self, group: IpAddress, interface: IpAddress) -> io::Result<()> {
        let socket = self
            .inner
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "socket not bound"))?;
        match (to_std_ip(group), to_std_ip(interface)) {
            (std::net::IpAddr::V4(g), std::net::IpAddr::V4(i)) => socket.join_multicast_v4(&g, &i),
            #[cfg(feature = "ipv6")]
            (std::net::IpAddr::V6(g), _) => socket.join_multicast_v6(&g, 0),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "mismatched address families for multicast join",
            )),
        }
    }

    /// Leave a multicast group previously joined with
    /// [`join_multicast`](Self::join_multicast), returning the raw OS error
    /// on failure.
    pub fn leave_multicast_io(&mut self, group: IpAddress, interface: IpAddress) -> io::Result<()> {
        let socket = self
            .inner
            .as_ref()
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotConnected, "socket not bound"))?;
        match (to_std_ip(group), to_std_ip(interface)) {
            (std::net::IpAddr::V4(g), std::net::IpAddr::V4(i)) => socket.leave_multicast_v4(&g, &i),
            #[cfg(feature = "ipv6")]
            (std::net::IpAddr::V6(g), _) => socket.leave_multicast_v6(&g, 0),
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "mismatched address families for multicast leave",
            )),
        }
    }

    /// Join a multicast group (upstream `AbstractDatagramSocket::join`).
    ///
    /// Returns [`UdpError::NotOk`] when the socket is unbound, the address
    /// families are mixed, or the OS refuses the membership.
    pub fn join_multicast(&mut self, group: IpAddress, interface: IpAddress) -> UdpError {
        match self.join_multicast_io(group, interface) {
            Ok(()) => UdpError::Ok,
            Err(_) => UdpError::NotOk,
        }
    }

    /// Leave a multicast group.
    ///
    /// Returns [`UdpError::NotOk`] when the socket is unbound, the address
    /// families are mixed, or the OS rejects the request.
    pub fn leave_multicast(&mut self, group: IpAddress, interface: IpAddress) -> UdpError {
        match self.leave_multicast_io(group, interface) {
            Ok(()) => UdpError::Ok,
            Err(_) => UdpError::NotOk,
        }
    }

    /// Enable or disable the `SO_BROADCAST` option.
    ///
    /// Returns [`UdpError::NotOk`] when the socket is unbound or the OS
    /// rejects the request.
    pub fn set_broadcast(&mut self, enable: bool) -> UdpError {
        match self.inner.as_ref() {
            Some(socket) => match socket.set_broadcast(enable) {
                Ok(()) => UdpError::Ok,
                Err(_) => UdpError::NotOk,
            },
            None => UdpError::NotOk,
        }
    }

    /// Current state of the `SO_BROADCAST` option, or `None` when the socket
    /// is unbound or the OS query fails.
    pub fn broadcast(&self) -> Option<bool> {
        self.inner.as_ref().and_then(|s| s.broadcast().ok())
    }

    /// Poll for one pending datagram.
    ///
    /// Returns `Ok(None)` when no datagram is currently available,
    /// `Ok(Some((length, source)))` when one was received into `buf`, and
    /// `Err(UdpError::NotOk)` when the socket is unbound or the OS reports a
    /// receive error. Never blocks.
    pub fn poll_recv_from(
        &mut self,
        buf: &mut [u8],
    ) -> Result<Option<(usize, IpEndpoint)>, UdpError> {
        let socket = self.inner.as_ref().ok_or(UdpError::NotOk)?;
        match socket.recv_from(buf) {
            Ok((length, source)) => {
                let address = from_std_ip(source.ip()).unwrap_or_default();
                Ok(Some((length, IpEndpoint::new(address, source.port()))))
            }
            Err(e) if is_transient(&e) => Ok(None),
            Err(_) => Err(UdpError::NotOk),
        }
    }

    /// Poll for one pending datagram and report it to a [`UdpDataListener`].
    ///
    /// The destination address reported to the listener is this socket's
    /// local address. Returns the number of bytes received (`0` when no
    /// datagram was pending). Never blocks.
    pub fn poll_with_listener(
        &mut self,
        buf: &mut [u8],
        listener: &mut dyn UdpDataListener,
    ) -> Result<usize, UdpError> {
        match self.poll_recv_from(buf)? {
            Some((length, source)) => {
                let dst = self.local_address();
                let src_port = source.port().unwrap_or(0);
                listener.on_datagram_received(
                    source.address(),
                    src_port,
                    &dst,
                    saturate_u16(length),
                );
                Ok(length)
            }
            None => Ok(0),
        }
    }

    /// Send to the connected remote endpoint and notify the listener on
    /// success (upstream `IDataSentListener` semantics).
    pub fn send_with_listener(
        &mut self,
        data: &[u8],
        listener: &mut dyn UdpSendListener,
    ) -> UdpError {
        let result = self.send(data);
        if result == UdpError::Ok {
            listener.on_datagram_sent(data);
        }
        result
    }

    /// Send a datagram to the endpoint in `packet` and notify the listener on
    /// success.
    pub fn send_to_with_listener(
        &mut self,
        packet: &DatagramPacket<'_>,
        listener: &mut dyn UdpSendListener,
    ) -> UdpError {
        let result = self.send_to(packet);
        if result == UdpError::Ok {
            listener.on_datagram_sent(packet.data());
        }
        result
    }

    /// Bind a fresh non-blocking OS socket. Fails if already bound.
    fn bind_inner(&mut self, target: SocketAddr) -> UdpError {
        if self.inner.is_some() {
            return UdpError::NotOk;
        }
        match StdUdpSocket::bind(target) {
            Ok(socket) => {
                if socket.set_nonblocking(true).is_err() {
                    return UdpError::NotOk;
                }
                self.inner = Some(socket);
                UdpError::Ok
            }
            Err(_) => UdpError::NotOk,
        }
    }
}

impl UdpSocket for PosixUdpSocket {
    /// Bind to a local address and port.
    ///
    /// Passing `None` for the address binds to the IPv4 wildcard `0.0.0.0`;
    /// to bind the IPv6 wildcard pass the explicit `::` address (requires the
    /// `ipv6` feature). Fails with [`UdpError::NotOk`] when already bound or
    /// when the OS rejects the bind.
    fn bind(&mut self, addr: Option<&IpAddress>, port: u16) -> UdpError {
        let ip = match addr {
            Some(a) => to_std_ip(*a),
            None => std::net::IpAddr::V4(std::net::Ipv4Addr::UNSPECIFIED),
        };
        self.bind_inner(SocketAddr::new(ip, port))
    }

    /// Close the socket. Idempotent; always returns [`UdpError::Ok`].
    fn close(&mut self) -> UdpError {
        self.inner = None;
        self.remote = None;
        UdpError::Ok
    }

    fn is_bound(&self) -> bool {
        self.inner.is_some()
    }

    fn is_closed(&self) -> bool {
        self.inner.is_none()
    }

    /// Send to the remote endpoint set via [`connect`](Self::connect).
    ///
    /// Returns [`UdpError::NotOk`] when unbound, not connected, when the OS
    /// rejects the datagram (e.g. oversized payload), or when the datagram
    /// was only partially accepted.
    fn send(&mut self, data: &[u8]) -> UdpError {
        let (Some(socket), Some(_)) = (self.inner.as_ref(), self.remote) else {
            return UdpError::NotOk;
        };
        match socket.send(data) {
            Ok(sent) if sent == data.len() => UdpError::Ok,
            Ok(_) | Err(_) => UdpError::NotOk,
        }
    }

    /// Send a datagram to the endpoint carried by `packet`.
    ///
    /// Returns [`UdpError::NotOk`] when unbound, when the packet has no port,
    /// or when the OS rejects the datagram (e.g. oversized payload).
    fn send_to(&mut self, packet: &DatagramPacket<'_>) -> UdpError {
        let Some(socket) = self.inner.as_ref() else {
            return UdpError::NotOk;
        };
        let Some(port) = packet.endpoint().port() else {
            return UdpError::NotOk;
        };
        let target = SocketAddr::new(to_std_ip(*packet.address()), port);
        match socket.send_to(packet.data(), target) {
            Ok(sent) if sent == packet.data().len() => UdpError::Ok,
            Ok(_) | Err(_) => UdpError::NotOk,
        }
    }

    /// Read the payload of one pending datagram without blocking.
    ///
    /// Returns `Ok(0)` when no datagram is pending. A datagram longer than
    /// `buf` is truncated on Unix; on Windows the OS reports an error, which
    /// maps to [`UdpError::NotOk`].
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, UdpError> {
        let socket = self.inner.as_ref().ok_or(UdpError::NotOk)?;
        match socket.recv(buf) {
            Ok(length) => Ok(length),
            Err(e) if is_transient(&e) => Ok(0),
            Err(_) => Err(UdpError::NotOk),
        }
    }

    /// Associate a remote endpoint, enabling [`send`](Self::send).
    ///
    /// If the socket is not yet bound it is bound implicitly to the wildcard
    /// address of the remote's family with an ephemeral port (mirroring the
    /// implicit-bind behaviour of upstream `connect`).
    fn connect(&mut self, addr: &IpAddress, port: u16) -> UdpError {
        let target = SocketAddr::new(to_std_ip(*addr), port);
        if self.inner.is_none() {
            let wildcard = match target {
                SocketAddr::V4(_) => std::net::IpAddr::V4(std::net::Ipv4Addr::UNSPECIFIED),
                SocketAddr::V6(_) => std::net::IpAddr::V6(std::net::Ipv6Addr::UNSPECIFIED),
            };
            if self.bind_inner(SocketAddr::new(wildcard, 0)) != UdpError::Ok {
                return UdpError::NotOk;
            }
        }
        let Some(socket) = self.inner.as_ref() else {
            return UdpError::NotOk;
        };
        match socket.connect(target) {
            Ok(()) => {
                self.remote = Some(target);
                UdpError::Ok
            }
            Err(_) => UdpError::NotOk,
        }
    }

    /// Drop the remote endpoint association.
    ///
    /// Limitation of the `std::net` backend: the OS-level peer filter
    /// installed by `connect` cannot be removed without re-creating the
    /// socket, so after `disconnect` the socket keeps receiving only from the
    /// previous peer. [`send`](Self::send) is disabled and
    /// [`is_connected`](Self::is_connected) reports `false` as required by
    /// the trait contract.
    fn disconnect(&mut self) -> UdpError {
        self.remote = None;
        UdpError::Ok
    }

    fn is_connected(&self) -> bool {
        self.remote.is_some()
    }

    /// Local address, or the unspecified address when unbound.
    fn local_address(&self) -> IpAddress {
        addr_or_unspecified(self.inner.as_ref().and_then(|s| s.local_addr().ok()))
    }

    /// Local port, or `0` when unbound.
    fn local_port(&self) -> u16 {
        self.inner
            .as_ref()
            .and_then(|s| s.local_addr().ok())
            .map_or(0, |sa| sa.port())
    }
}

/// `true` for I/O errors that mean "no data right now" rather than failure.
fn is_transient(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted | io::ErrorKind::TimedOut
    )
}

/// Clamp a byte count into `u16` (protocol listener callbacks carry `u16`).
fn saturate_u16(value: usize) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn loopback() -> IpAddress {
        IpAddress::ipv4(127, 0, 0, 1)
    }

    #[test]
    fn fresh_socket_state() {
        let socket = PosixUdpSocket::new();
        assert!(!socket.is_bound());
        assert!(socket.is_closed());
        assert!(!socket.is_connected());
        assert_eq!(socket.local_port(), 0);
        assert_eq!(socket.local_address(), IpAddress::unspecified());
        assert_eq!(socket.broadcast(), None);
    }

    #[test]
    fn send_unbound_fails() {
        let mut socket = PosixUdpSocket::new();
        assert_eq!(socket.send(&[1, 2, 3]), UdpError::NotOk);
    }

    #[test]
    fn send_to_unbound_fails() {
        let mut socket = PosixUdpSocket::new();
        let data = [1u8, 2];
        let packet = DatagramPacket::new(&data, loopback(), 4000);
        assert_eq!(socket.send_to(&packet), UdpError::NotOk);
    }

    #[test]
    fn read_unbound_fails() {
        let mut socket = PosixUdpSocket::new();
        let mut buf = [0u8; 8];
        assert_eq!(socket.read(&mut buf), Err(UdpError::NotOk));
    }

    #[test]
    fn bind_and_close_transitions() {
        let mut socket = PosixUdpSocket::new();
        let addr = loopback();
        assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
        assert!(socket.is_bound());
        assert!(!socket.is_closed());
        assert_ne!(socket.local_port(), 0);
        assert_eq!(socket.local_address(), addr);
        assert_eq!(socket.close(), UdpError::Ok);
        assert!(socket.is_closed());
        assert!(!socket.is_bound());
    }

    #[test]
    fn double_bind_fails() {
        let mut socket = PosixUdpSocket::new();
        let addr = loopback();
        assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
        assert_eq!(socket.bind(Some(&addr), 0), UdpError::NotOk);
    }

    #[test]
    fn send_to_without_port_fails() {
        let mut socket = PosixUdpSocket::new();
        let addr = loopback();
        assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
        let data = [1u8];
        let packet = DatagramPacket::with_endpoint(&data, IpEndpoint::unset());
        assert_eq!(socket.send_to(&packet), UdpError::NotOk);
    }

    #[test]
    fn send_unconnected_fails_even_when_bound() {
        let mut socket = PosixUdpSocket::new();
        let addr = loopback();
        assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
        assert_eq!(socket.send(&[1]), UdpError::NotOk);
    }

    #[test]
    fn connect_auto_binds() {
        let mut socket = PosixUdpSocket::new();
        assert_eq!(socket.connect(&loopback(), 12345), UdpError::Ok);
        assert!(socket.is_bound());
        assert!(socket.is_connected());
        assert_ne!(socket.local_port(), 0);
    }

    #[test]
    fn disconnect_clears_association() {
        let mut socket = PosixUdpSocket::new();
        assert_eq!(socket.connect(&loopback(), 12345), UdpError::Ok);
        assert_eq!(socket.disconnect(), UdpError::Ok);
        assert!(!socket.is_connected());
        assert_eq!(socket.send(&[1]), UdpError::NotOk);
    }

    #[test]
    fn multicast_unbound_fails() {
        let mut socket = PosixUdpSocket::new();
        let group = IpAddress::ipv4(224, 0, 0, 251);
        assert_eq!(socket.join_multicast(group, loopback()), UdpError::NotOk);
        assert_eq!(socket.leave_multicast(group, loopback()), UdpError::NotOk);
    }

    #[test]
    fn set_broadcast_unbound_fails() {
        let mut socket = PosixUdpSocket::new();
        assert_eq!(socket.set_broadcast(true), UdpError::NotOk);
    }

    #[test]
    fn saturate_u16_clamps() {
        assert_eq!(saturate_u16(0), 0);
        assert_eq!(saturate_u16(65_535), u16::MAX);
        assert_eq!(saturate_u16(1_000_000), u16::MAX);
    }
}
