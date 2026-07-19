//! POSIX adapter for the portable stack boundary
//! ([`SocketApi`] / [`DatagramApi`] / [`LinkState`]).
//!
//! [`PosixSocketStack`] lets host compositions drive the same portable
//! protocol code that embedded backends drive, by implementing the generic
//! boundary over the audited `std::net` adapters ([`PosixTcpSocket`],
//! [`PosixTcpServerSocket`], [`PosixUdpSocket`]). Capacities stay fixed:
//! `TCP_SLOTS` bounds the TCP socket pool (listeners plus connections) and
//! `UDP_SLOTS` bounds the datagram pool; event delivery is bounded by
//! `EVENTS` with overflow counted, mirroring the deterministic fake.
//!
//! Documented `std::net` limitations apply unchanged: bind-before-connect
//! only for the wildcard request, `connect` blocks up to the configured
//! connect timeout, `listen` backlog is OS-managed (the argument is
//! accepted and ignored), and abort approximates RST via
//! drop-with-unread-data. The host link is reported permanently up.

use std::collections::VecDeque;

use crate::endpoint::IpEndpoint;
use crate::ip::IpAddress;
use crate::lwip::{
    DatagramApi, DatagramId, LinkState, SocketApi, SocketError, SocketEvent, SocketId, SocketState,
};
use crate::tcp::{ConnectionCloseReason, TcpError, TcpServerSocket, TcpSocket};
use crate::udp::{DatagramPacket, UdpError, UdpSocket};

use super::tcp::{PosixTcpServerSocket, PosixTcpSocket};
use super::udp::PosixUdpSocket;

/// Maximum number of buffered [`SocketEvent`]s.
pub const EVENTS: usize = 32;

/// Broadcast target recognised for automatic `SO_BROADCAST` setup.
fn is_broadcast(addr: &IpAddress) -> bool {
    addr.as_bytes() == [255, 255, 255, 255]
}

#[derive(Debug)]
enum TcpEntry {
    /// Created, with an optional stored bind request applied on `listen`.
    Fresh {
        bound: Option<(IpAddress, u16)>,
    },
    Listener(PosixTcpServerSocket),
    Stream {
        socket: PosixTcpSocket,
        /// Set once the peer ended the connection; the slot then reports
        /// [`SocketState::Closed`] until closed locally.
        closed: Option<ConnectionCloseReason>,
    },
}

#[derive(Debug, Default)]
struct TcpSlot {
    generation: u8,
    entry: Option<TcpEntry>,
}

#[derive(Debug, Default)]
struct UdpSlot {
    generation: u8,
    socket: Option<PosixUdpSocket>,
    broadcast_enabled: bool,
}

/// Fixed-capacity POSIX backend for the portable stack boundary.
#[derive(Debug)]
pub struct PosixSocketStack<const TCP_SLOTS: usize = 16, const UDP_SLOTS: usize = 4> {
    tcp: [TcpSlot; TCP_SLOTS],
    udp: [UdpSlot; UDP_SLOTS],
    events: VecDeque<(SocketId, SocketEvent)>,
    dropped_events: usize,
}

impl<const TCP_SLOTS: usize, const UDP_SLOTS: usize> Default
    for PosixSocketStack<TCP_SLOTS, UDP_SLOTS>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const TCP_SLOTS: usize, const UDP_SLOTS: usize> PosixSocketStack<TCP_SLOTS, UDP_SLOTS> {
    /// Create an empty stack with all slots free.
    pub fn new() -> Self {
        const {
            assert!(
                TCP_SLOTS >= 1 && TCP_SLOTS <= 256,
                "TCP_SLOTS must be 1..=256"
            );
        }
        const {
            assert!(
                UDP_SLOTS >= 1 && UDP_SLOTS <= 256,
                "UDP_SLOTS must be 1..=256"
            );
        }
        Self {
            tcp: std::array::from_fn(|_| TcpSlot::default()),
            udp: std::array::from_fn(|_| UdpSlot::default()),
            events: VecDeque::with_capacity(EVENTS),
            dropped_events: 0,
        }
    }

    /// Number of events discarded because the bounded queue was full.
    pub fn dropped_events(&self) -> usize {
        self.dropped_events
    }

    fn push_event(&mut self, socket: SocketId, event: SocketEvent) {
        if self.events.len() == EVENTS {
            self.dropped_events += 1;
            return;
        }
        self.events.push_back((socket, event));
    }

    fn resolve(&self, socket: SocketId) -> Result<usize, SocketError> {
        let index = usize::from(socket.raw() & 0xFF);
        let generation = (socket.raw() >> 8) as u8;
        if index >= TCP_SLOTS {
            return Err(SocketError::InvalidHandle);
        }
        let slot = &self.tcp[index];
        if slot.entry.is_some() && slot.generation == generation {
            Ok(index)
        } else {
            Err(SocketError::InvalidHandle)
        }
    }

    fn resolve_udp(&self, socket: DatagramId) -> Result<usize, SocketError> {
        let index = usize::from(socket.raw() & 0xFF);
        let generation = (socket.raw() >> 8) as u8;
        if index >= UDP_SLOTS {
            return Err(SocketError::InvalidHandle);
        }
        let slot = &self.udp[index];
        if slot.socket.is_some() && slot.generation == generation {
            Ok(index)
        } else {
            Err(SocketError::InvalidHandle)
        }
    }

    fn free_tcp_slot(&self) -> Option<usize> {
        (0..TCP_SLOTS).find(|&index| self.tcp[index].entry.is_none())
    }

    fn tcp_id(&self, index: usize) -> SocketId {
        SocketId::from_raw((u16::from(self.tcp[index].generation) << 8) | (index as u16))
    }

    fn udp_id(&self, index: usize) -> DatagramId {
        DatagramId::from_raw((u16::from(self.udp[index].generation) << 8) | (index as u16))
    }

    /// Release a TCP slot and invalidate its handles.
    fn release_tcp(&mut self, index: usize) {
        let slot = &mut self.tcp[index];
        slot.entry = None;
        slot.generation = slot.generation.wrapping_add(1);
    }
}

impl<const TCP_SLOTS: usize, const UDP_SLOTS: usize> SocketApi
    for PosixSocketStack<TCP_SLOTS, UDP_SLOTS>
{
    fn create(&mut self) -> Result<SocketId, SocketError> {
        let index = self.free_tcp_slot().ok_or(SocketError::NoResources)?;
        self.tcp[index].entry = Some(TcpEntry::Fresh { bound: None });
        Ok(self.tcp_id(index))
    }

    fn bind(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        match self.tcp[index].entry.as_mut() {
            Some(TcpEntry::Fresh {
                bound: bound @ None,
            }) => {
                *bound = Some((addr, port));
                Ok(())
            }
            _ => Err(SocketError::InvalidState),
        }
    }

    /// Start listening; the OS manages the actual backlog, so `_backlog`
    /// is accepted and ignored.
    fn listen(&mut self, socket: SocketId, _backlog: u8) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        let (addr, port) = match self.tcp[index].entry.as_ref() {
            Some(TcpEntry::Fresh { bound: Some(pair) }) => *pair,
            _ => return Err(SocketError::InvalidState),
        };
        let mut listener = PosixTcpServerSocket::new();
        match TcpServerSocket::bind(&mut listener, &addr, port) {
            TcpError::Ok => {
                self.tcp[index].entry = Some(TcpEntry::Listener(listener));
                Ok(())
            }
            _ => Err(SocketError::AddressInUse),
        }
    }

    /// Connect to a remote endpoint. Limitation of the `std::net` backend:
    /// the call blocks up to the configured connect timeout, and completion
    /// is reported synchronously via a queued [`SocketEvent::Connected`].
    fn connect(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        match self.tcp[index].entry.as_ref() {
            Some(TcpEntry::Fresh { .. }) => {}
            _ => return Err(SocketError::InvalidState),
        }
        let mut stream = PosixTcpSocket::new();
        match TcpSocket::connect(&mut stream, &addr, port) {
            TcpError::Ok => {
                let _ = stream.set_nodelay(true);
                self.tcp[index].entry = Some(TcpEntry::Stream {
                    socket: stream,
                    closed: None,
                });
                let id = self.tcp_id(index);
                self.push_event(id, SocketEvent::Connected);
                Ok(())
            }
            _ => Err(SocketError::ConnectionRefused),
        }
    }

    fn accept(&mut self, socket: SocketId) -> Result<Option<SocketId>, SocketError> {
        let index = self.resolve(socket)?;
        let Some(TcpEntry::Listener(listener)) = self.tcp[index].entry.as_mut() else {
            return Err(SocketError::InvalidState);
        };
        match listener.accept() {
            Ok(None) => Ok(None),
            Ok(Some(mut stream)) => {
                // Interactive protocol traffic (small request/response
                // frames) must not be Nagle-delayed.
                let _ = stream.set_nodelay(true);
                let Some(free) = self.free_tcp_slot() else {
                    // No slot for the accepted connection: refuse it hard so
                    // exhaustion stays observable, keep the listener alive.
                    let mut owned = stream;
                    owned.abort();
                    return Err(SocketError::NoResources);
                };
                self.tcp[free].entry = Some(TcpEntry::Stream {
                    socket: stream,
                    closed: None,
                });
                Ok(Some(self.tcp_id(free)))
            }
            Err(_) => Err(SocketError::InvalidState),
        }
    }

    fn send(&mut self, socket: SocketId, data: &[u8]) -> Result<usize, SocketError> {
        let index = self.resolve(socket)?;
        let Some(TcpEntry::Stream {
            socket: stream,
            closed,
        }) = self.tcp[index].entry.as_mut()
        else {
            return Err(SocketError::NotConnected);
        };
        if closed.is_some() {
            return Err(SocketError::Closed);
        }
        if data.is_empty() {
            return Ok(0);
        }
        match TcpSocket::send(stream, data) {
            TcpError::Ok => Ok(data.len()),
            TcpError::NoMoreBuffer | TcpError::Flush => Err(SocketError::BufferFull),
            TcpError::NotOpen => Err(SocketError::Closed),
            TcpError::NotOk => Err(SocketError::NotConnected),
        }
    }

    fn recv(&mut self, socket: SocketId, buf: &mut [u8]) -> Result<usize, SocketError> {
        let index = self.resolve(socket)?;
        let id = self.tcp_id(index);
        let Some(TcpEntry::Stream {
            socket: stream,
            closed,
        }) = self.tcp[index].entry.as_mut()
        else {
            return Err(SocketError::NotConnected);
        };
        if closed.is_some() {
            return Ok(0);
        }
        // Piggyback pending-send draining on the poll cycle.
        stream.flush();
        match stream.read(buf) {
            Ok(length) => Ok(length),
            Err(error) => {
                let reason = match error {
                    TcpError::NotOpen => ConnectionCloseReason::ClosedByPeer,
                    _ => stream
                        .close_reason()
                        .unwrap_or(ConnectionCloseReason::Reset),
                };
                *closed = Some(reason);
                self.push_event(id, SocketEvent::ConnectionClosed { reason });
                Ok(0)
            }
        }
    }

    fn close(&mut self, socket: SocketId) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        match self.tcp[index].entry.as_mut() {
            Some(TcpEntry::Stream { socket, .. }) => {
                let _ = socket.close();
            }
            Some(TcpEntry::Listener(listener)) => {
                let _ = listener.close();
            }
            Some(TcpEntry::Fresh { .. }) | None => {}
        }
        self.release_tcp(index);
        Ok(())
    }

    fn abort(&mut self, socket: SocketId) {
        let Ok(index) = self.resolve(socket) else {
            return;
        };
        if let Some(TcpEntry::Stream { socket, .. }) = self.tcp[index].entry.as_mut() {
            socket.abort();
        }
        if let Some(TcpEntry::Listener(listener)) = self.tcp[index].entry.as_mut() {
            let _ = listener.close();
        }
        self.release_tcp(index);
    }

    fn state(&self, socket: SocketId) -> Result<SocketState, SocketError> {
        let index = self.resolve(socket)?;
        Ok(match self.tcp[index].entry.as_ref() {
            Some(TcpEntry::Fresh { bound: None }) | None => SocketState::Created,
            Some(TcpEntry::Fresh { bound: Some(_) }) => SocketState::Bound,
            Some(TcpEntry::Listener(_)) => SocketState::Listening,
            Some(TcpEntry::Stream {
                closed: Some(_), ..
            }) => SocketState::Closed,
            Some(TcpEntry::Stream { socket, .. }) => {
                if socket.is_established() {
                    SocketState::Established
                } else {
                    SocketState::Closed
                }
            }
        })
    }

    fn local_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve(socket)?;
        Ok(match self.tcp[index].entry.as_ref() {
            Some(TcpEntry::Fresh {
                bound: Some((addr, port)),
            }) => IpEndpoint::new(*addr, *port),
            Some(TcpEntry::Listener(listener)) => {
                IpEndpoint::new(listener.local_address(), listener.local_port())
            }
            Some(TcpEntry::Stream { socket, .. }) => {
                IpEndpoint::new(socket.local_address(), socket.local_port())
            }
            Some(TcpEntry::Fresh { bound: None }) | None => IpEndpoint::unset(),
        })
    }

    fn remote_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve(socket)?;
        Ok(match self.tcp[index].entry.as_ref() {
            Some(TcpEntry::Stream { socket, .. }) => {
                IpEndpoint::new(socket.remote_address(), socket.remote_port())
            }
            _ => IpEndpoint::unset(),
        })
    }

    fn poll_event(&mut self) -> Option<(SocketId, SocketEvent)> {
        self.events.pop_front()
    }
}

impl<const TCP_SLOTS: usize, const UDP_SLOTS: usize> DatagramApi
    for PosixSocketStack<TCP_SLOTS, UDP_SLOTS>
{
    fn create_datagram(&mut self) -> Result<DatagramId, SocketError> {
        let index = (0..UDP_SLOTS)
            .find(|&index| self.udp[index].socket.is_none())
            .ok_or(SocketError::NoResources)?;
        self.udp[index].socket = Some(PosixUdpSocket::new());
        self.udp[index].broadcast_enabled = false;
        Ok(self.udp_id(index))
    }

    fn bind_datagram(
        &mut self,
        socket: DatagramId,
        addr: IpAddress,
        port: u16,
    ) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        let Some(slot) = self.udp[index].socket.as_mut() else {
            return Err(SocketError::InvalidHandle);
        };
        if slot.is_bound() {
            return Err(SocketError::InvalidState);
        }
        match slot.bind(Some(&addr), port) {
            UdpError::Ok => Ok(()),
            _ => Err(SocketError::AddressInUse),
        }
    }

    fn send_datagram(
        &mut self,
        socket: DatagramId,
        target: IpEndpoint,
        data: &[u8],
    ) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        if target.port().is_none() {
            return Err(SocketError::InvalidState);
        }
        let broadcast = is_broadcast(target.address());
        let enable_broadcast = broadcast && !self.udp[index].broadcast_enabled;
        let Some(slot) = self.udp[index].socket.as_mut() else {
            return Err(SocketError::InvalidHandle);
        };
        if !slot.is_bound() {
            // Implicit ephemeral bind, mirroring OS datagram semantics.
            if slot.bind(None, 0) != UdpError::Ok {
                return Err(SocketError::NoResources);
            }
        }
        if enable_broadcast {
            if slot.set_broadcast(true) != UdpError::Ok {
                return Err(SocketError::InvalidState);
            }
            self.udp[index].broadcast_enabled = true;
        }
        let Some(slot) = self.udp[index].socket.as_mut() else {
            return Err(SocketError::InvalidHandle);
        };
        let packet = DatagramPacket::with_endpoint(data, target);
        match slot.send_to(&packet) {
            UdpError::Ok => Ok(()),
            _ => Err(SocketError::BufferFull),
        }
    }

    fn recv_datagram(
        &mut self,
        socket: DatagramId,
        buf: &mut [u8],
    ) -> Result<Option<(usize, IpEndpoint)>, SocketError> {
        let index = self.resolve_udp(socket)?;
        let Some(slot) = self.udp[index].socket.as_mut() else {
            return Err(SocketError::InvalidHandle);
        };
        slot.poll_recv_from(buf)
            .map_err(|_| SocketError::NotConnected)
    }

    fn datagram_local_endpoint(&self, socket: DatagramId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve_udp(socket)?;
        let Some(slot) = self.udp[index].socket.as_ref() else {
            return Err(SocketError::InvalidHandle);
        };
        if slot.is_bound() {
            Ok(IpEndpoint::new(IpAddress::unspecified(), slot.local_port()))
        } else {
            Ok(IpEndpoint::unset())
        }
    }

    fn close_datagram(&mut self, socket: DatagramId) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        if let Some(mut owned) = self.udp[index].socket.take() {
            let _ = owned.close();
        }
        self.udp[index].generation = self.udp[index].generation.wrapping_add(1);
        self.udp[index].broadcast_enabled = false;
        Ok(())
    }
}

impl<const TCP_SLOTS: usize, const UDP_SLOTS: usize> LinkState
    for PosixSocketStack<TCP_SLOTS, UDP_SLOTS>
{
    /// The host link is reported permanently up.
    fn link_up(&self) -> bool {
        true
    }
}
