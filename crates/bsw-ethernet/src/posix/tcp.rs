//! POSIX TCP adapter over [`std::net::TcpStream`] / [`std::net::TcpListener`].
//!
//! [`PosixTcpSocket`] implements the crate's [`TcpSocket`] trait and
//! [`PosixTcpServerSocket`] implements [`TcpServerSocket`], both with
//! non-blocking polling semantics mirroring upstream `tcp::AbstractSocket`
//! and `tcp::AbstractServerSocket`.
//!
//! # Send semantics and backpressure
//!
//! [`TcpSocket::send`] has full-write semantics: it either accepts the whole
//! payload or rejects it without consuming any byte. Bytes the OS cannot take
//! immediately (`EWOULDBLOCK`) are stored in an internal bounded pending
//! buffer and flushed by [`flush`](TcpSocket::flush),
//! [`poll_send`](PosixTcpSocket::poll_send), or the next `send`. When the
//! payload does not fit into the remaining pending capacity, `send` returns
//! [`TcpError::NoMoreBuffer`] and no data is written or queued, so no data is
//! ever lost.
//!
//! # Close reasons
//!
//! Remote-initiated closure is observed while polling and mapped to
//! [`ConnectionCloseReason`]: an orderly FIN maps to
//! [`ClosedByPeer`](ConnectionCloseReason::ClosedByPeer), a reset (`ECONNRESET`
//! and relatives) to [`Reset`](ConnectionCloseReason::Reset), and a timeout to
//! [`Timeout`](ConnectionCloseReason::Timeout). A local
//! [`close`](TcpSocket::close) records no reason, matching upstream where the
//! `connectionClosed` callback fires only for peer-initiated closure.

use std::collections::VecDeque;
use std::io::{self, Read as _, Write as _};
use std::net::{Shutdown, SocketAddr, TcpListener, TcpStream};
use std::time::Duration;

use crate::ip::IpAddress;
use crate::tcp::{
    ConnectionCloseReason, SendResult, TcpDataListener, TcpError, TcpSendListener, TcpServerSocket,
    TcpSocket,
};

use super::{addr_or_unspecified, to_std_ip};

/// Default capacity in bytes of the internal pending-send buffer.
pub const DEFAULT_PENDING_CAPACITY: usize = 64 * 1024;

/// Default timeout applied to [`TcpSocket::connect`].
pub const DEFAULT_CONNECT_TIMEOUT: Duration = Duration::from_secs(2);

/// Size of the stack probe used by `available` and `poll_receive`.
const PROBE_SIZE: usize = 4096;

/// Outcome of probing the receive side of a stream.
enum PeekOutcome {
    /// No data currently available.
    Idle,
    /// `n` bytes are ready to read (saturated at the probe size).
    Data(usize),
    /// The connection ended for the given reason.
    Closed(ConnectionCloseReason),
}

/// TCP client/connection socket backed by a non-blocking
/// [`std::net::TcpStream`].
///
/// Obtained either by [`connect`](TcpSocket::connect)-ing to a server, from
/// [`PosixTcpServerSocket::accept`], or by adopting an existing stream with
/// [`from_std_stream`](Self::from_std_stream).
#[derive(Debug)]
pub struct PosixTcpSocket {
    stream: Option<TcpStream>,
    established: bool,
    pending: VecDeque<u8>,
    pending_capacity: usize,
    connect_timeout: Duration,
    close_reason: Option<ConnectionCloseReason>,
    local: Option<SocketAddr>,
    remote: Option<SocketAddr>,
}

impl PosixTcpSocket {
    /// Create a new, unconnected socket with
    /// [`DEFAULT_PENDING_CAPACITY`] and [`DEFAULT_CONNECT_TIMEOUT`].
    pub fn new() -> Self {
        Self::with_pending_capacity(DEFAULT_PENDING_CAPACITY)
    }

    /// Create a new, unconnected socket with a custom pending-send buffer
    /// capacity in bytes.
    pub fn with_pending_capacity(capacity: usize) -> Self {
        Self {
            stream: None,
            established: false,
            pending: VecDeque::new(),
            pending_capacity: capacity,
            connect_timeout: DEFAULT_CONNECT_TIMEOUT,
            close_reason: None,
            local: None,
            remote: None,
        }
    }

    /// Adopt an already-connected [`TcpStream`] (e.g. from an external
    /// accept loop). The stream is switched to non-blocking mode.
    pub fn from_std_stream(stream: TcpStream) -> Result<Self, TcpError> {
        if stream.set_nonblocking(true).is_err() {
            return Err(TcpError::NotOk);
        }
        let local = stream.local_addr().ok();
        let remote = stream.peer_addr().ok();
        let mut socket = Self::new();
        socket.stream = Some(stream);
        socket.established = true;
        socket.local = local;
        socket.remote = remote;
        Ok(socket)
    }

    /// Set the timeout used by subsequent [`connect`](TcpSocket::connect)
    /// calls.
    pub fn set_connect_timeout(&mut self, timeout: Duration) {
        self.connect_timeout = timeout;
    }

    /// Reason the connection ended, when it was closed by the remote side.
    ///
    /// `None` while connected or after a purely local close.
    pub fn close_reason(&self) -> Option<ConnectionCloseReason> {
        self.close_reason
    }

    /// Number of bytes currently held in the pending-send buffer.
    pub fn pending_len(&self) -> usize {
        self.pending.len()
    }

    /// Enable or disable `TCP_NODELAY` (upstream `disableNagleAlgorithm`).
    ///
    /// Returns [`TcpError::NotOpen`] when not connected.
    pub fn set_nodelay(&mut self, nodelay: bool) -> TcpError {
        match self.stream.as_ref() {
            Some(stream) => match stream.set_nodelay(nodelay) {
                Ok(()) => TcpError::Ok,
                Err(_) => TcpError::NotOk,
            },
            None => TcpError::NotOpen,
        }
    }

    /// Poll the receive side once and notify the listener.
    ///
    /// Level-triggered: while unread data is buffered by the OS the listener's
    /// `on_data_received` is invoked with the currently available byte count
    /// (saturated at an internal probe size). When the peer ended the
    /// connection, the socket is closed, the reason recorded, and
    /// `on_connection_closed` invoked exactly once.
    pub fn poll_receive(&mut self, listener: &mut dyn TcpDataListener) {
        let outcome = match self.stream.as_ref() {
            Some(stream) => probe_stream(stream),
            None => return,
        };
        match outcome {
            PeekOutcome::Idle => {}
            PeekOutcome::Data(count) => listener.on_data_received(saturate_u16(count)),
            PeekOutcome::Closed(reason) => {
                self.mark_closed(reason);
                listener.on_connection_closed(reason);
            }
        }
    }

    /// Try to flush the pending-send buffer without blocking.
    ///
    /// When bytes were flushed and a listener is given, it is notified with
    /// [`SendResult::DataSent`]. Returns [`TcpError::NotOk`] when the
    /// connection failed while flushing (the close reason is recorded).
    pub fn poll_send(&mut self, listener: Option<&mut dyn TcpSendListener>) -> TcpError {
        match self.drain_pending() {
            Ok(flushed) => {
                if flushed > 0 {
                    if let Some(l) = listener {
                        l.on_data_sent(saturate_u16(flushed), SendResult::DataSent);
                    }
                }
                TcpError::Ok
            }
            Err(error) => error,
        }
    }

    /// Send with completion notification (upstream
    /// `IDataSendNotificationListener` semantics).
    ///
    /// On success the listener receives the accepted length together with
    /// [`SendResult::DataSent`] when everything reached the OS immediately, or
    /// [`SendResult::DataQueued`] when part of the payload went into the
    /// pending buffer.
    pub fn send_with_listener(
        &mut self,
        data: &[u8],
        listener: &mut dyn TcpSendListener,
    ) -> TcpError {
        match self.send_inner(data) {
            Ok(queued) => {
                if !data.is_empty() {
                    let result = if queued > 0 {
                        SendResult::DataQueued
                    } else {
                        SendResult::DataSent
                    };
                    listener.on_data_sent(saturate_u16(data.len()), result);
                }
                TcpError::Ok
            }
            Err(error) => error,
        }
    }

    /// Tear down the stream and record why the connection ended.
    fn mark_closed(&mut self, reason: ConnectionCloseReason) {
        self.stream = None;
        self.established = false;
        self.pending.clear();
        self.local = None;
        self.remote = None;
        self.close_reason = Some(reason);
    }

    /// Flush as much of the pending buffer as the OS accepts right now.
    ///
    /// Returns the number of flushed bytes, or an error after a fatal I/O
    /// failure (the socket is closed and the reason recorded).
    fn drain_pending(&mut self) -> Result<usize, TcpError> {
        let mut flushed = 0usize;
        let mut fatal: Option<io::ErrorKind> = None;
        if let Some(stream) = self.stream.as_mut() {
            while !self.pending.is_empty() {
                let write_result = {
                    let (front, _) = self.pending.as_slices();
                    write_nonblocking(stream, front)
                };
                match write_result {
                    Ok(0) => break,
                    Ok(n) => {
                        self.pending.drain(..n);
                        flushed += n;
                    }
                    Err(e) => {
                        fatal = Some(e.kind());
                        break;
                    }
                }
            }
        }
        if let Some(kind) = fatal {
            self.mark_closed(classify_close(kind));
            return Err(TcpError::NotOk);
        }
        Ok(flushed)
    }

    /// Core of `send`: full-write-or-reject.
    ///
    /// Returns the number of bytes that went into the pending buffer (`0`
    /// when everything was written immediately).
    fn send_inner(&mut self, data: &[u8]) -> Result<usize, TcpError> {
        if self.stream.is_none() {
            return Err(TcpError::NotOpen);
        }
        if data.is_empty() {
            return Ok(0);
        }
        if self.drain_pending().is_err() {
            return Err(TcpError::NotOk);
        }
        let free = self.pending_capacity.saturating_sub(self.pending.len());
        if data.len() > free {
            return Err(TcpError::NoMoreBuffer);
        }
        let mut written = 0usize;
        if self.pending.is_empty() {
            let mut fatal: Option<io::ErrorKind> = None;
            if let Some(stream) = self.stream.as_mut() {
                while written < data.len() {
                    match write_nonblocking(stream, &data[written..]) {
                        Ok(0) => break,
                        Ok(n) => written += n,
                        Err(e) => {
                            fatal = Some(e.kind());
                            break;
                        }
                    }
                }
            }
            if let Some(kind) = fatal {
                self.mark_closed(classify_close(kind));
                return Err(TcpError::NotOk);
            }
        }
        let queued = data.len() - written;
        self.pending.extend(data[written..].iter().copied());
        Ok(queued)
    }
}

impl Default for PosixTcpSocket {
    fn default() -> Self {
        Self::new()
    }
}

impl TcpSocket for PosixTcpSocket {
    /// Bind before connect.
    ///
    /// Limitation of the `std::net` backend: the standard library offers no
    /// bind-before-connect, so only the wildcard request (unspecified address
    /// and port `0` — the OS default) is accepted. Any specific local
    /// address or port returns [`TcpError::NotOk`].
    fn bind(&mut self, addr: &IpAddress, port: u16) -> TcpError {
        if self.stream.is_some() {
            return TcpError::NotOk;
        }
        if addr.is_unspecified() && port == 0 {
            TcpError::Ok
        } else {
            TcpError::NotOk
        }
    }

    /// Connect to a remote endpoint, bounded by the configured timeout
    /// (see [`set_connect_timeout`](Self::set_connect_timeout)).
    ///
    /// Returns [`TcpError::NotOk`] when already connected, when the peer
    /// refuses, or on timeout.
    fn connect(&mut self, addr: &IpAddress, port: u16) -> TcpError {
        if self.stream.is_some() {
            return TcpError::NotOk;
        }
        let target = SocketAddr::new(to_std_ip(*addr), port);
        match TcpStream::connect_timeout(&target, self.connect_timeout) {
            Ok(stream) => {
                if stream.set_nonblocking(true).is_err() {
                    return TcpError::NotOk;
                }
                self.local = stream.local_addr().ok();
                self.remote = stream.peer_addr().ok();
                self.stream = Some(stream);
                self.established = true;
                self.close_reason = None;
                TcpError::Ok
            }
            Err(_) => TcpError::NotOk,
        }
    }

    /// Close gracefully: flush what the OS accepts right now, then shut down
    /// both directions. Idempotent. A local close records no close reason.
    fn close(&mut self) -> TcpError {
        let _ = self.drain_pending();
        if let Some(stream) = self.stream.take() {
            let _ = stream.shutdown(Shutdown::Both);
            self.established = false;
            self.pending.clear();
            self.local = None;
            self.remote = None;
        }
        TcpError::Ok
    }

    /// Abort: drop the stream immediately without a graceful shutdown and
    /// discard any pending send data. If unread receive data exists, the OS
    /// answers the peer with a reset.
    fn abort(&mut self) {
        self.stream = None;
        self.established = false;
        self.pending.clear();
        self.local = None;
        self.remote = None;
    }

    /// Best-effort flush of the pending-send buffer (never blocks).
    fn flush(&mut self) {
        let _ = self.drain_pending();
    }

    /// Send with full-write semantics; see the
    /// [module documentation](self) for the backpressure contract.
    fn send(&mut self, data: &[u8]) -> TcpError {
        match self.send_inner(data) {
            Ok(_) => TcpError::Ok,
            Err(error) => error,
        }
    }

    /// Read without blocking.
    ///
    /// Returns `Ok(0)` when no data is currently available. When the peer
    /// ended the connection the socket is closed, the reason recorded (see
    /// [`close_reason`](Self::close_reason)), and an error returned:
    /// [`TcpError::NotOpen`] for an orderly FIN, [`TcpError::NotOk`] for a
    /// reset or another fatal error.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TcpError> {
        if buf.is_empty() {
            return Ok(0);
        }
        let Some(stream) = self.stream.as_mut() else {
            return Err(TcpError::NotOpen);
        };
        match stream.read(buf) {
            Ok(0) => {
                self.mark_closed(ConnectionCloseReason::ClosedByPeer);
                Err(TcpError::NotOpen)
            }
            Ok(n) => Ok(n),
            Err(e) if is_transient(&e) => Ok(0),
            Err(e) => {
                self.mark_closed(classify_close(e.kind()));
                Err(TcpError::NotOk)
            }
        }
    }

    /// Number of bytes ready to read, saturated at an internal probe size
    /// (4 KiB). `0` when not connected or no data is pending.
    fn available(&self) -> usize {
        let Some(stream) = self.stream.as_ref() else {
            return 0;
        };
        match probe_stream(stream) {
            PeekOutcome::Data(count) => count,
            PeekOutcome::Idle | PeekOutcome::Closed(_) => 0,
        }
    }

    fn is_closed(&self) -> bool {
        self.stream.is_none()
    }

    fn is_established(&self) -> bool {
        self.stream.is_some() && self.established
    }

    /// Remote address, or the unspecified address when not connected.
    fn remote_address(&self) -> IpAddress {
        addr_or_unspecified(self.remote)
    }

    /// Local address, or the unspecified address when not connected.
    fn local_address(&self) -> IpAddress {
        addr_or_unspecified(self.local)
    }

    /// Remote port, or `0` when not connected.
    fn remote_port(&self) -> u16 {
        self.remote.map_or(0, |sa| sa.port())
    }

    /// Local port, or `0` when not connected.
    fn local_port(&self) -> u16 {
        self.local.map_or(0, |sa| sa.port())
    }
}

/// TCP listening socket backed by a non-blocking
/// [`std::net::TcpListener`].
#[derive(Debug, Default)]
pub struct PosixTcpServerSocket {
    listener: Option<TcpListener>,
}

impl PosixTcpServerSocket {
    /// Create a new, unbound server socket.
    pub fn new() -> Self {
        Self::default()
    }

    /// Poll for one pending incoming connection.
    ///
    /// Returns `Ok(None)` when no connection is waiting (never blocks),
    /// `Ok(Some(socket))` with an established, non-blocking
    /// [`PosixTcpSocket`] on success, [`TcpError::NotOpen`] when the server
    /// is not bound, and [`TcpError::NotOk`] on other accept errors.
    pub fn accept(&mut self) -> Result<Option<PosixTcpSocket>, TcpError> {
        let Some(listener) = self.listener.as_ref() else {
            return Err(TcpError::NotOpen);
        };
        match listener.accept() {
            Ok((stream, _peer)) => PosixTcpSocket::from_std_stream(stream).map(Some),
            Err(e) if is_transient(&e) => Ok(None),
            Err(_) => Err(TcpError::NotOk),
        }
    }

    /// Local address the server is bound to, or the unspecified address.
    pub fn local_address(&self) -> IpAddress {
        addr_or_unspecified(self.listener.as_ref().and_then(|l| l.local_addr().ok()))
    }
}

impl TcpServerSocket for PosixTcpServerSocket {
    /// Bind and start listening on the given local address and port
    /// (`port` `0` selects an ephemeral port). Fails with
    /// [`TcpError::NotOk`] when already bound or when the OS rejects the
    /// bind.
    fn bind(&mut self, addr: &IpAddress, port: u16) -> TcpError {
        if self.listener.is_some() {
            return TcpError::NotOk;
        }
        let target = SocketAddr::new(to_std_ip(*addr), port);
        match TcpListener::bind(target) {
            Ok(listener) => {
                if listener.set_nonblocking(true).is_err() {
                    return TcpError::NotOk;
                }
                self.listener = Some(listener);
                TcpError::Ok
            }
            Err(_) => TcpError::NotOk,
        }
    }

    /// Stop listening. Idempotent; always returns [`TcpError::Ok`].
    fn close(&mut self) -> TcpError {
        self.listener = None;
        TcpError::Ok
    }

    fn is_closed(&self) -> bool {
        self.listener.is_none()
    }

    /// Port the server listens on, or `0` when unbound.
    fn local_port(&self) -> u16 {
        self.listener
            .as_ref()
            .and_then(|l| l.local_addr().ok())
            .map_or(0, |sa| sa.port())
    }
}

/// Non-blocking write helper: maps `EWOULDBLOCK` to `Ok(0)` and retries on
/// `EINTR`. `data` must not be empty (a zero return means "would block").
fn write_nonblocking(stream: &mut TcpStream, data: &[u8]) -> io::Result<usize> {
    loop {
        match stream.write(data) {
            Ok(n) => return Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(0),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e),
        }
    }
}

/// Probe the receive side of a stream without consuming data.
fn probe_stream(stream: &TcpStream) -> PeekOutcome {
    let mut probe = [0u8; PROBE_SIZE];
    match stream.peek(&mut probe) {
        Ok(0) => PeekOutcome::Closed(ConnectionCloseReason::ClosedByPeer),
        Ok(n) => PeekOutcome::Data(n),
        Err(e) if is_transient(&e) => PeekOutcome::Idle,
        Err(e) => PeekOutcome::Closed(classify_close(e.kind())),
    }
}

/// Map a fatal I/O error kind to a [`ConnectionCloseReason`].
fn classify_close(kind: io::ErrorKind) -> ConnectionCloseReason {
    match kind {
        io::ErrorKind::ConnectionReset
        | io::ErrorKind::ConnectionAborted
        | io::ErrorKind::BrokenPipe => ConnectionCloseReason::Reset,
        io::ErrorKind::TimedOut => ConnectionCloseReason::Timeout,
        _ => ConnectionCloseReason::Unknown,
    }
}

/// `true` for I/O errors that mean "try again later" rather than failure.
fn is_transient(error: &io::Error) -> bool {
    matches!(
        error.kind(),
        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
    )
}

/// Clamp a byte count into `u16` (listener callbacks carry `u16`).
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
        let socket = PosixTcpSocket::new();
        assert!(socket.is_closed());
        assert!(!socket.is_established());
        assert_eq!(socket.local_port(), 0);
        assert_eq!(socket.remote_port(), 0);
        assert_eq!(socket.local_address(), IpAddress::unspecified());
        assert_eq!(socket.remote_address(), IpAddress::unspecified());
        assert_eq!(socket.close_reason(), None);
        assert_eq!(socket.pending_len(), 0);
        assert_eq!(socket.available(), 0);
    }

    #[test]
    fn send_not_open() {
        let mut socket = PosixTcpSocket::new();
        assert_eq!(socket.send(&[1, 2, 3]), TcpError::NotOpen);
    }

    #[test]
    fn read_not_open() {
        let mut socket = PosixTcpSocket::new();
        let mut buf = [0u8; 8];
        assert_eq!(socket.read(&mut buf), Err(TcpError::NotOpen));
    }

    #[test]
    fn bind_wildcard_accepted() {
        let mut socket = PosixTcpSocket::new();
        assert_eq!(socket.bind(&IpAddress::unspecified(), 0), TcpError::Ok);
    }

    #[test]
    fn bind_specific_rejected() {
        let mut socket = PosixTcpSocket::new();
        assert_eq!(socket.bind(&loopback(), 0), TcpError::NotOk);
        assert_eq!(
            socket.bind(&IpAddress::unspecified(), 8080),
            TcpError::NotOk
        );
    }

    #[test]
    fn close_is_idempotent() {
        let mut socket = PosixTcpSocket::new();
        assert_eq!(socket.close(), TcpError::Ok);
        assert_eq!(socket.close(), TcpError::Ok);
        assert!(socket.is_closed());
    }

    #[test]
    fn abort_on_fresh_socket_is_noop() {
        let mut socket = PosixTcpSocket::new();
        socket.abort();
        assert!(socket.is_closed());
        assert_eq!(socket.close_reason(), None);
    }

    #[test]
    fn set_nodelay_not_open() {
        let mut socket = PosixTcpSocket::new();
        assert_eq!(socket.set_nodelay(true), TcpError::NotOpen);
    }

    #[test]
    fn server_fresh_state() {
        let server = PosixTcpServerSocket::new();
        assert!(server.is_closed());
        assert_eq!(server.local_port(), 0);
        assert_eq!(server.local_address(), IpAddress::unspecified());
    }

    #[test]
    fn server_accept_unbound_fails() {
        let mut server = PosixTcpServerSocket::new();
        assert!(matches!(server.accept(), Err(TcpError::NotOpen)));
    }

    #[test]
    fn server_bind_close_transitions() {
        let mut server = PosixTcpServerSocket::new();
        assert_eq!(server.bind(&loopback(), 0), TcpError::Ok);
        assert!(!server.is_closed());
        assert_ne!(server.local_port(), 0);
        assert_eq!(server.local_address(), loopback());
        // double bind rejected
        assert_eq!(server.bind(&loopback(), 0), TcpError::NotOk);
        assert_eq!(server.close(), TcpError::Ok);
        assert!(server.is_closed());
    }

    #[test]
    fn classify_close_mapping() {
        assert_eq!(
            classify_close(io::ErrorKind::ConnectionReset),
            ConnectionCloseReason::Reset
        );
        assert_eq!(
            classify_close(io::ErrorKind::ConnectionAborted),
            ConnectionCloseReason::Reset
        );
        assert_eq!(
            classify_close(io::ErrorKind::BrokenPipe),
            ConnectionCloseReason::Reset
        );
        assert_eq!(
            classify_close(io::ErrorKind::TimedOut),
            ConnectionCloseReason::Timeout
        );
        assert_eq!(
            classify_close(io::ErrorKind::Other),
            ConnectionCloseReason::Unknown
        );
    }

    #[test]
    fn saturate_u16_clamps() {
        assert_eq!(saturate_u16(0), 0);
        assert_eq!(saturate_u16(70_000), u16::MAX);
    }
}
