//! Socket boundary for an lwIP-style embedded TCP stack.
//!
//! This module defines the pure-Rust contract that a port of upstream's
//! `lwipSocket` layer plugs into. No C types appear here: the boundary is
//! expressed with the crate's [`IpAddress`] / [`IpEndpoint`] types, `u16`
//! ports, and `&[u8]` buffers so that protocol crates stay free of FFI
//! details. It is fully `no_std` and allocation-free.
//!
//! # Model
//!
//! A backend implementing [`SocketApi`] owns a fixed pool of connection
//! resources identified by opaque [`SocketId`] handles. Operations are
//! non-blocking; asynchronous outcomes (data arrival, transmit completion,
//! connection teardown, injected faults) surface as [`SocketEvent`]s that the
//! application drains with [`SocketApi::poll_event`] or dispatches to a
//! [`SocketEventListener`] via [`SocketApi::dispatch_events`].
//!
//! # Backends
//!
//! * [`fake`] — a deterministic in-memory backend for host-side testing with
//!   bounded buffers, backpressure, and injectable failures.
//! * A real lwIP-backed implementation lives in the platform integration
//!   crate and must uphold the same contract.

pub mod fake;

use crate::endpoint::IpEndpoint;
use crate::ip::IpAddress;
use crate::tcp::ConnectionCloseReason;

/// Opaque handle identifying a socket inside a [`SocketApi`] backend.
///
/// Handles are backend-scoped: a handle obtained from one backend must not
/// be used with another. Backends invalidate handles on close so that stale
/// handles are rejected with [`SocketError::InvalidHandle`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SocketId(u16);

impl SocketId {
    /// Construct a handle from its raw backend-defined representation.
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    /// The raw backend-defined representation of this handle.
    #[inline]
    pub const fn raw(self) -> u16 {
        self.0
    }
}

/// Typed errors reported by [`SocketApi`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocketError {
    /// The handle does not refer to a live socket of this backend.
    InvalidHandle,
    /// No free socket slot / backlog entry is available.
    NoResources,
    /// The operation is not valid in the socket's current state.
    InvalidState,
    /// The requested local address/port is already in use.
    AddressInUse,
    /// No peer is listening at the requested endpoint, or the connection
    /// attempt was actively refused.
    ConnectionRefused,
    /// The peer's receive buffer cannot accept any byte right now
    /// (backpressure); retry after the peer consumed data.
    BufferFull,
    /// The socket is not connected.
    NotConnected,
    /// The connection has been closed.
    Closed,
}

/// Lifecycle state of a socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocketState {
    /// Freshly created, not yet bound.
    Created,
    /// Bound to a local endpoint.
    Bound,
    /// Listening for incoming connections.
    Listening,
    /// Connection requested, not yet accepted by the peer.
    Connecting,
    /// Connection established; data transfer possible.
    Established,
    /// Connection ended (locally or by the peer). Received data may still be
    /// drained with [`SocketApi::recv`].
    Closed,
}

/// Asynchronous notification produced by a [`SocketApi`] backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SocketEvent {
    /// A pending connect completed; the socket is now
    /// [`Established`](SocketState::Established).
    Connected,
    /// `length` bytes arrived and can be read with [`SocketApi::recv`]
    /// (saturated at `u16::MAX`).
    DataReceived {
        /// Number of newly arrived bytes.
        length: u16,
    },
    /// `length` bytes were handed to the stack for transmission
    /// (saturated at `u16::MAX`).
    DataSent {
        /// Number of transmitted bytes.
        length: u16,
    },
    /// The connection ended.
    ConnectionClosed {
        /// Why the connection ended.
        reason: ConnectionCloseReason,
    },
    /// An asynchronous error occurred on the socket.
    Error {
        /// The reported error.
        error: SocketError,
    },
}

/// Receiver of [`SocketEvent`]s, used with [`SocketApi::dispatch_events`].
pub trait SocketEventListener {
    /// Called once per pending event, in order of occurrence.
    fn on_socket_event(&mut self, socket: SocketId, event: SocketEvent);
}

/// Opaque handle identifying a datagram (UDP) socket inside a
/// [`DatagramApi`] backend.
///
/// Datagram handles live in their own namespace: a [`DatagramId`] is never
/// interchangeable with a [`SocketId`]. Backends invalidate handles on close
/// so stale handles are rejected with [`SocketError::InvalidHandle`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DatagramId(u16);

impl DatagramId {
    /// Construct a handle from its raw backend-defined representation.
    #[inline]
    pub const fn from_raw(raw: u16) -> Self {
        Self(raw)
    }

    /// The raw backend-defined representation of this handle.
    #[inline]
    pub const fn raw(self) -> u16 {
        self.0
    }
}

/// The datagram (UDP) boundary an embedded stack backend implements.
///
/// This is the connectionless companion of [`SocketApi`], demonstrated by
/// production `DoIP` behavior: vehicle-discovery request/response needs
/// nonblocking receive with the remote endpoint, and scheduled vehicle
/// announcements need unicast, broadcast, or multicast transmission. All
/// operations are non-blocking and heap-free; capacities (socket pool,
/// per-socket datagram queue, maximum datagram payload) are fixed by the
/// backend and must be documented by it.
///
/// # Ownership and stale handles
///
/// The backend owns every buffer; callers only pass borrowed slices whose
/// content is fully consumed (copied or delivered) before the call returns.
/// [`DatagramApi::close_datagram`] invalidates the handle; any later use
/// must fail with [`SocketError::InvalidHandle`]. Platform adapters must
/// uphold the same rules so protocol code cannot observe a freed resource.
pub trait DatagramApi {
    /// Allocate a new unbound datagram socket.
    ///
    /// Fails with [`SocketError::NoResources`] when the pool is exhausted.
    fn create_datagram(&mut self) -> Result<DatagramId, SocketError>;

    /// Bind the datagram socket to a local address and port (`0` selects an
    /// ephemeral port). Fails with [`SocketError::AddressInUse`] on
    /// conflicts and [`SocketError::InvalidState`] when already bound.
    fn bind_datagram(
        &mut self,
        socket: DatagramId,
        addr: IpAddress,
        port: u16,
    ) -> Result<(), SocketError>;

    /// Transmit one datagram to `target` without blocking.
    ///
    /// `target` may be a unicast, broadcast, or multicast endpoint; the
    /// backend performs any transport-specific setup itself. Fails with
    /// [`SocketError::InvalidState`] when unbound, with
    /// [`SocketError::NotConnected`] when the link is down, and with
    /// [`SocketError::BufferFull`] when transmit resources are exhausted
    /// (backpressure; the datagram was not sent).
    fn send_datagram(
        &mut self,
        socket: DatagramId,
        target: IpEndpoint,
        data: &[u8],
    ) -> Result<(), SocketError>;

    /// Receive one pending datagram without blocking.
    ///
    /// Returns `Ok(None)` when nothing is pending, otherwise the number of
    /// payload bytes copied into `buf` and the remote endpoint the datagram
    /// came from. A datagram longer than `buf` is truncated to `buf.len()`
    /// (the remainder is discarded, mirroring `recvfrom` semantics).
    fn recv_datagram(
        &mut self,
        socket: DatagramId,
        buf: &mut [u8],
    ) -> Result<Option<(usize, IpEndpoint)>, SocketError>;

    /// Local endpoint, or an unset endpoint before bind.
    fn datagram_local_endpoint(&self, socket: DatagramId) -> Result<IpEndpoint, SocketError>;

    /// Close the datagram socket and release its resources; the handle
    /// becomes invalid.
    fn close_datagram(&mut self, socket: DatagramId) -> Result<(), SocketError>;
}

/// Link/interface readiness observation for an embedded stack backend.
///
/// [`crate::network_interface::NetworkInterface`] owns interface lifecycle
/// and configuration; this trait is the minimal read-only view protocol
/// components need to react to link loss and recovery between polls. A
/// backend without link management reports `true` permanently.
pub trait LinkState {
    /// `true` while the underlying link/interface is usable.
    fn link_up(&self) -> bool {
        true
    }
}

/// The socket boundary an lwIP-style stack backend implements.
///
/// All operations are non-blocking and heap-free; capacities are fixed by
/// the backend. Unless documented otherwise, operations on an invalid
/// handle return [`SocketError::InvalidHandle`].
pub trait SocketApi {
    /// Allocate a new socket in state [`Created`](SocketState::Created).
    ///
    /// Fails with [`SocketError::NoResources`] when the pool is exhausted.
    fn create(&mut self) -> Result<SocketId, SocketError>;

    /// Bind the socket to a local address and port (`0` selects an
    /// ephemeral port). Valid in state [`Created`](SocketState::Created).
    fn bind(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError>;

    /// Start listening. Valid in state [`Bound`](SocketState::Bound).
    /// `backlog` limits pending, not-yet-accepted connections (the backend
    /// may clamp it to its fixed capacity).
    fn listen(&mut self, socket: SocketId, backlog: u8) -> Result<(), SocketError>;

    /// Request a connection to a remote endpoint. Valid in states
    /// [`Created`](SocketState::Created) (implicit ephemeral bind) and
    /// [`Bound`](SocketState::Bound). Completion is signalled by a
    /// [`SocketEvent::Connected`] event.
    fn connect(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError>;

    /// Accept one pending connection on a listening socket.
    ///
    /// Returns `Ok(None)` when no connection is pending (never blocks) and
    /// `Ok(Some(id))` with the handle of the new established connection
    /// socket otherwise.
    fn accept(&mut self, socket: SocketId) -> Result<Option<SocketId>, SocketError>;

    /// Queue up to `data.len()` bytes for transmission.
    ///
    /// Returns the number of bytes accepted, which may be less than
    /// `data.len()` when the peer's buffer is nearly full (partial write).
    /// Fails with [`SocketError::BufferFull`] when not a single byte can be
    /// accepted. Valid in state [`Established`](SocketState::Established).
    fn send(&mut self, socket: SocketId, data: &[u8]) -> Result<usize, SocketError>;

    /// Read up to `buf.len()` received bytes without blocking.
    ///
    /// Returns the number of bytes copied (`0` when nothing is pending).
    /// Also valid in state [`Closed`](SocketState::Closed) to drain data
    /// that arrived before the connection ended.
    fn recv(&mut self, socket: SocketId, buf: &mut [u8]) -> Result<usize, SocketError>;

    /// Close the socket gracefully and release its resources; the handle
    /// becomes invalid. A connected peer observes
    /// [`ConnectionCloseReason::ClosedByPeer`].
    fn close(&mut self, socket: SocketId) -> Result<(), SocketError>;

    /// Abort the socket; the handle becomes invalid. A connected peer
    /// observes [`ConnectionCloseReason::Reset`]. No-op on an invalid
    /// handle.
    fn abort(&mut self, socket: SocketId);

    /// Current lifecycle state of the socket.
    fn state(&self, socket: SocketId) -> Result<SocketState, SocketError>;

    /// Local endpoint, or an unset endpoint before bind.
    fn local_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError>;

    /// Remote endpoint, or an unset endpoint before connect/accept.
    fn remote_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError>;

    /// Take the oldest pending event, if any.
    fn poll_event(&mut self) -> Option<(SocketId, SocketEvent)>;

    /// Drain all pending events into `listener`; returns how many events
    /// were dispatched.
    fn dispatch_events(&mut self, listener: &mut dyn SocketEventListener) -> usize {
        let mut dispatched = 0usize;
        while let Some((socket, event)) = self.poll_event() {
            listener.on_socket_event(socket, event);
            dispatched += 1;
        }
        dispatched
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn socket_id_raw_roundtrip() {
        let id = SocketId::from_raw(0x1234);
        assert_eq!(id.raw(), 0x1234);
        assert_eq!(id, SocketId::from_raw(0x1234));
        assert_ne!(id, SocketId::from_raw(0x1235));
    }

    #[test]
    fn socket_error_equality() {
        assert_eq!(SocketError::BufferFull, SocketError::BufferFull);
        assert_ne!(SocketError::BufferFull, SocketError::InvalidHandle);
    }

    #[test]
    fn socket_state_equality() {
        assert_eq!(SocketState::Created, SocketState::Created);
        assert_ne!(SocketState::Created, SocketState::Closed);
    }

    #[test]
    fn socket_event_carries_payload() {
        let event = SocketEvent::DataReceived { length: 42 };
        assert_eq!(event, SocketEvent::DataReceived { length: 42 });
        assert_ne!(event, SocketEvent::DataReceived { length: 43 });
        let close = SocketEvent::ConnectionClosed {
            reason: ConnectionCloseReason::Reset,
        };
        assert_ne!(event, close);
    }
}
