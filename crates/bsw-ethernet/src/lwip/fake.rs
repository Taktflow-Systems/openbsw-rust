//! Deterministic in-memory backend for the [`SocketApi`] boundary.
//!
//! [`FakeStack`] implements the full socket contract without any operating
//! system or C dependency: connect/accept pairing, bounded receive buffers
//! with backpressure, ordered event delivery, and injectable failures
//! (connect refusal, silent data drop, asynchronous error events). It is
//! `no_std` and allocation-free — all capacities are const-generic
//! parameters — so the same conformance tests can run on host and target.
//!
//! # Determinism
//!
//! Every operation completes synchronously and enqueues its events in a
//! fixed order, so a given call sequence always produces the same event
//! sequence. Delivery is instantaneous: bytes sent on one side are
//! immediately readable on the peer (up to the peer's buffer capacity).

use core::array;

use crate::endpoint::IpEndpoint;
use crate::ip::IpAddress;
use crate::tcp::ConnectionCloseReason;

use super::{
    DatagramApi, DatagramId, LinkState, SocketApi, SocketError, SocketEvent, SocketId, SocketState,
};

/// Maximum pending (not yet accepted) connections per listening socket.
pub const MAX_BACKLOG: usize = 4;

/// Fixed number of datagram (UDP) sockets of the fake stack.
pub const UDP_SOCKETS: usize = 4;

/// Fixed per-socket receive queue depth in datagrams.
pub const UDP_QUEUE: usize = 4;

/// Fixed maximum datagram payload held by the fake stack; longer payloads
/// are rejected by [`DatagramApi::send_datagram`] with
/// [`SocketError::BufferFull`].
pub const UDP_DATAGRAM_MAX: usize = 128;

/// First port used for ephemeral (implicit) binds.
const EPHEMERAL_START: u16 = 49152;

/// Fixed-capacity byte ring buffer.
#[derive(Debug, Clone, Copy)]
struct RingBuffer<const N: usize> {
    buf: [u8; N],
    head: usize,
    len: usize,
}

impl<const N: usize> RingBuffer<N> {
    const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            len: 0,
        }
    }

    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    const fn free(&self) -> usize {
        N - self.len
    }

    /// Copy as much of `data` as fits; returns the number of bytes accepted.
    fn push(&mut self, data: &[u8]) -> usize {
        let count = self.free().min(data.len());
        for (i, &byte) in data[..count].iter().enumerate() {
            self.buf[(self.head + self.len + i) % N] = byte;
        }
        self.len += count;
        count
    }

    /// Copy up to `out.len()` bytes out; returns the number of bytes copied.
    fn pop(&mut self, out: &mut [u8]) -> usize {
        let count = self.len.min(out.len());
        for (i, slot) in out[..count].iter_mut().enumerate() {
            *slot = self.buf[(self.head + i) % N];
        }
        self.head = (self.head + count) % N;
        self.len -= count;
        count
    }
}

/// Fixed-capacity FIFO of socket events.
#[derive(Debug, Clone, Copy)]
struct EventQueue<const N: usize> {
    items: [Option<(SocketId, SocketEvent)>; N],
    head: usize,
    len: usize,
    dropped: usize,
}

impl<const N: usize> EventQueue<N> {
    const fn new() -> Self {
        Self {
            items: [None; N],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    fn push(&mut self, socket: SocketId, event: SocketEvent) {
        if self.len == N {
            self.dropped += 1;
            return;
        }
        self.items[(self.head + self.len) % N] = Some((socket, event));
        self.len += 1;
    }

    fn pop(&mut self) -> Option<(SocketId, SocketEvent)> {
        if self.len == 0 {
            return None;
        }
        let item = self.items[self.head].take();
        self.head = (self.head + 1) % N;
        self.len -= 1;
        item
    }
}

/// One buffered datagram.
#[derive(Debug, Clone, Copy)]
struct Datagram {
    len: u16,
    addr: IpAddress,
    port: u16,
    bytes: [u8; UDP_DATAGRAM_MAX],
}

impl Datagram {
    const fn empty() -> Self {
        Self {
            len: 0,
            addr: IpAddress::unspecified(),
            port: 0,
            bytes: [0; UDP_DATAGRAM_MAX],
        }
    }
}

/// One datagram-socket slot of the fake pool.
#[derive(Debug, Clone, Copy)]
struct UdpSlot {
    used: bool,
    generation: u8,
    bound: bool,
    local_addr: IpAddress,
    local_port: u16,
    queue: [Option<Datagram>; UDP_QUEUE],
    queue_len: u8,
}

impl UdpSlot {
    const fn new() -> Self {
        Self {
            used: false,
            generation: 0,
            bound: false,
            local_addr: IpAddress::unspecified(),
            local_port: 0,
            queue: [None; UDP_QUEUE],
            queue_len: 0,
        }
    }

    fn reset_active(&mut self) {
        self.used = true;
        self.bound = false;
        self.local_addr = IpAddress::unspecified();
        self.local_port = 0;
        self.queue = [None; UDP_QUEUE];
        self.queue_len = 0;
    }

    /// Append one datagram; returns `false` when the queue is full.
    fn enqueue(&mut self, datagram: Datagram) -> bool {
        let len = usize::from(self.queue_len);
        if len == UDP_QUEUE {
            return false;
        }
        self.queue[len] = Some(datagram);
        self.queue_len += 1;
        true
    }

    /// Remove and return the oldest datagram.
    fn dequeue(&mut self) -> Option<Datagram> {
        if self.queue_len == 0 {
            return None;
        }
        let item = self.queue[0];
        for i in 1..usize::from(self.queue_len) {
            self.queue[i - 1] = self.queue[i];
        }
        self.queue[usize::from(self.queue_len) - 1] = None;
        self.queue_len -= 1;
        item
    }
}

/// One socket slot of the fake pool.
#[derive(Debug, Clone, Copy)]
struct Slot<const RX: usize> {
    used: bool,
    generation: u8,
    state: SocketState,
    local_addr: IpAddress,
    local_port: u16,
    remote_addr: IpAddress,
    remote_port: u16,
    peer: Option<SocketId>,
    rx: RingBuffer<RX>,
    backlog: [Option<SocketId>; MAX_BACKLOG],
    backlog_len: u8,
    backlog_cap: u8,
}

impl<const RX: usize> Slot<RX> {
    fn new() -> Self {
        Self {
            used: false,
            generation: 0,
            state: SocketState::Created,
            local_addr: IpAddress::unspecified(),
            local_port: 0,
            remote_addr: IpAddress::unspecified(),
            remote_port: 0,
            peer: None,
            rx: RingBuffer::new(),
            backlog: [None; MAX_BACKLOG],
            backlog_len: 0,
            backlog_cap: 0,
        }
    }

    /// Re-initialise the slot for a freshly allocated socket, keeping the
    /// generation counter.
    fn reset_active(&mut self) {
        self.used = true;
        self.state = SocketState::Created;
        self.local_addr = IpAddress::unspecified();
        self.local_port = 0;
        self.remote_addr = IpAddress::unspecified();
        self.remote_port = 0;
        self.peer = None;
        self.rx.clear();
        self.backlog = [None; MAX_BACKLOG];
        self.backlog_len = 0;
        self.backlog_cap = 0;
    }
}

/// Deterministic in-memory [`SocketApi`] backend.
///
/// * `SOCKETS` — size of the socket pool (1..=256)
/// * `RX_CAPACITY` — per-socket receive buffer size in bytes
/// * `EVENT_CAPACITY` — size of the shared event FIFO
///
/// See the [module documentation](self) for the behavioural model and
/// [`SocketApi`] for the contract.
#[derive(Debug)]
pub struct FakeStack<
    const SOCKETS: usize = 8,
    const RX_CAPACITY: usize = 256,
    const EVENT_CAPACITY: usize = 32,
> {
    slots: [Slot<RX_CAPACITY>; SOCKETS],
    udp_slots: [UdpSlot; UDP_SOCKETS],
    events: EventQueue<EVENT_CAPACITY>,
    refuse_connect: bool,
    drop_data: bool,
    link_up: bool,
    dropped_datagrams: usize,
    next_ephemeral: u16,
}

impl<const SOCKETS: usize, const RX_CAPACITY: usize, const EVENT_CAPACITY: usize>
    FakeStack<SOCKETS, RX_CAPACITY, EVENT_CAPACITY>
{
    /// Create an empty stack with all slots free.
    pub fn new() -> Self {
        const {
            assert!(SOCKETS >= 1 && SOCKETS <= 256, "SOCKETS must be 1..=256");
        }
        const {
            assert!(RX_CAPACITY >= 1, "RX_CAPACITY must be at least 1");
        }
        const {
            assert!(EVENT_CAPACITY >= 1, "EVENT_CAPACITY must be at least 1");
        }
        Self {
            slots: array::from_fn(|_| Slot::new()),
            udp_slots: [UdpSlot::new(); UDP_SOCKETS],
            events: EventQueue::new(),
            refuse_connect: false,
            drop_data: false,
            link_up: true,
            dropped_datagrams: 0,
            next_ephemeral: EPHEMERAL_START,
        }
    }

    /// Failure injection: make every subsequent [`SocketApi::connect`] fail
    /// with [`SocketError::ConnectionRefused`] until disabled again.
    pub fn set_refuse_connect(&mut self, refuse: bool) {
        self.refuse_connect = refuse;
    }

    /// Failure injection: silently drop the payload of every subsequent
    /// [`SocketApi::send`]. The sender still observes success and a
    /// [`SocketEvent::DataSent`] event, but nothing is delivered to the
    /// peer — modelling loss inside the stack.
    pub fn set_drop_data(&mut self, drop_data: bool) {
        self.drop_data = drop_data;
    }

    /// Failure injection: enqueue an asynchronous [`SocketEvent::Error`] for
    /// the given socket.
    pub fn inject_error(
        &mut self,
        socket: SocketId,
        error: SocketError,
    ) -> Result<(), SocketError> {
        self.resolve(socket)?;
        self.events.push(socket, SocketEvent::Error { error });
        Ok(())
    }

    /// Number of events discarded because the event queue was full.
    pub fn dropped_events(&self) -> usize {
        self.events.dropped
    }

    /// Failure injection: model link loss and recovery. While the link is
    /// down, [`SocketApi::connect`], [`SocketApi::send`], and
    /// [`DatagramApi::send_datagram`] fail with
    /// [`SocketError::NotConnected`]; already-delivered data can still be
    /// drained. The link starts up.
    pub fn set_link_up(&mut self, up: bool) {
        self.link_up = up;
    }

    /// Number of datagrams silently dropped because a receive queue was
    /// full (UDP loss semantics; exhaustion stays observable here).
    pub fn dropped_datagrams(&self) -> usize {
        self.dropped_datagrams
    }

    /// Validate a datagram handle and return its slot index.
    fn resolve_udp(&self, socket: DatagramId) -> Result<usize, SocketError> {
        let index = usize::from(socket.raw() & 0xFF);
        let generation = (socket.raw() >> 8) as u8;
        if index >= UDP_SOCKETS {
            return Err(SocketError::InvalidHandle);
        }
        let slot = &self.udp_slots[index];
        if slot.used && slot.generation == generation {
            Ok(index)
        } else {
            Err(SocketError::InvalidHandle)
        }
    }

    /// Validate a handle and return its slot index.
    fn resolve(&self, socket: SocketId) -> Result<usize, SocketError> {
        let index = usize::from(socket.raw() & 0xFF);
        let generation = (socket.raw() >> 8) as u8;
        if index >= SOCKETS {
            return Err(SocketError::InvalidHandle);
        }
        let slot = &self.slots[index];
        if slot.used && slot.generation == generation {
            Ok(index)
        } else {
            Err(SocketError::InvalidHandle)
        }
    }

    /// Allocate the next ephemeral port (wraps within the dynamic range).
    fn alloc_ephemeral(&mut self) -> u16 {
        let port = self.next_ephemeral;
        self.next_ephemeral = if port == u16::MAX {
            EPHEMERAL_START
        } else {
            port + 1
        };
        port
    }

    /// Remove and return the oldest backlog entry of a listener slot.
    fn pop_backlog(&mut self, index: usize) -> Option<SocketId> {
        let slot = &mut self.slots[index];
        if slot.backlog_len == 0 {
            return None;
        }
        let item = slot.backlog[0];
        for i in 1..usize::from(slot.backlog_len) {
            slot.backlog[i - 1] = slot.backlog[i];
        }
        slot.backlog[usize::from(slot.backlog_len) - 1] = None;
        slot.backlog_len -= 1;
        item
    }

    /// Put a backlog entry back at the front (used when accept cannot
    /// allocate a connection slot). Caller guarantees there is room.
    fn push_backlog_front(&mut self, index: usize, socket: SocketId) {
        let slot = &mut self.slots[index];
        let len = usize::from(slot.backlog_len);
        let mut i = len.min(MAX_BACKLOG - 1);
        while i > 0 {
            slot.backlog[i] = slot.backlog[i - 1];
            i -= 1;
        }
        slot.backlog[0] = Some(socket);
        if len < MAX_BACKLOG {
            slot.backlog_len += 1;
        }
    }

    /// Tear down a slot: notify the peer with `peer_reason`, refuse pending
    /// backlog clients, then free the slot and bump its generation.
    fn release(&mut self, index: usize, peer_reason: ConnectionCloseReason) {
        if let Some(peer_id) = self.slots[index].peer {
            if let Ok(peer_index) = self.resolve(peer_id) {
                let peer = &mut self.slots[peer_index];
                peer.state = SocketState::Closed;
                peer.peer = None;
                self.events.push(
                    peer_id,
                    SocketEvent::ConnectionClosed {
                        reason: peer_reason,
                    },
                );
            }
        }
        if self.slots[index].state == SocketState::Listening {
            while let Some(client_id) = self.pop_backlog(index) {
                if let Ok(client_index) = self.resolve(client_id) {
                    self.slots[client_index].state = SocketState::Closed;
                    self.events.push(
                        client_id,
                        SocketEvent::Error {
                            error: SocketError::ConnectionRefused,
                        },
                    );
                }
            }
        }
        let slot = &mut self.slots[index];
        slot.used = false;
        slot.generation = slot.generation.wrapping_add(1);
        slot.peer = None;
        slot.rx.clear();
        slot.backlog = [None; MAX_BACKLOG];
        slot.backlog_len = 0;
    }
}

impl<const SOCKETS: usize, const RX_CAPACITY: usize, const EVENT_CAPACITY: usize> Default
    for FakeStack<SOCKETS, RX_CAPACITY, EVENT_CAPACITY>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const SOCKETS: usize, const RX_CAPACITY: usize, const EVENT_CAPACITY: usize> SocketApi
    for FakeStack<SOCKETS, RX_CAPACITY, EVENT_CAPACITY>
{
    fn create(&mut self) -> Result<SocketId, SocketError> {
        for index in 0..SOCKETS {
            if !self.slots[index].used {
                self.slots[index].reset_active();
                return Ok(make_id(index, self.slots[index].generation));
            }
        }
        Err(SocketError::NoResources)
    }

    fn bind(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        if self.slots[index].state != SocketState::Created {
            return Err(SocketError::InvalidState);
        }
        if port != 0 {
            let conflict = self.slots.iter().enumerate().any(|(i, slot)| {
                i != index
                    && slot.used
                    && matches!(slot.state, SocketState::Bound | SocketState::Listening)
                    && slot.local_port == port
                    && (slot.local_addr == addr
                        || slot.local_addr.is_unspecified()
                        || addr.is_unspecified())
            });
            if conflict {
                return Err(SocketError::AddressInUse);
            }
        }
        let assigned = if port == 0 {
            self.alloc_ephemeral()
        } else {
            port
        };
        let slot = &mut self.slots[index];
        slot.local_addr = addr;
        slot.local_port = assigned;
        slot.state = SocketState::Bound;
        Ok(())
    }

    fn listen(&mut self, socket: SocketId, backlog: u8) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        let slot = &mut self.slots[index];
        if slot.state != SocketState::Bound {
            return Err(SocketError::InvalidState);
        }
        slot.state = SocketState::Listening;
        slot.backlog_cap = usize::from(backlog).clamp(1, MAX_BACKLOG) as u8;
        Ok(())
    }

    fn connect(&mut self, socket: SocketId, addr: IpAddress, port: u16) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        let state = self.slots[index].state;
        if !matches!(state, SocketState::Created | SocketState::Bound) {
            return Err(SocketError::InvalidState);
        }
        if !self.link_up {
            return Err(SocketError::NotConnected);
        }
        if self.refuse_connect {
            return Err(SocketError::ConnectionRefused);
        }
        let listener_index = self.slots.iter().position(|slot| {
            slot.used
                && slot.state == SocketState::Listening
                && slot.local_port == port
                && (slot.local_addr.is_unspecified() || slot.local_addr == addr)
        });
        let Some(listener_index) = listener_index else {
            return Err(SocketError::ConnectionRefused);
        };
        if self.slots[listener_index].backlog_len >= self.slots[listener_index].backlog_cap {
            return Err(SocketError::NoResources);
        }
        if state == SocketState::Created {
            let ephemeral = self.alloc_ephemeral();
            self.slots[index].local_port = ephemeral;
        }
        let client = &mut self.slots[index];
        client.remote_addr = addr;
        client.remote_port = port;
        client.state = SocketState::Connecting;
        let listener = &mut self.slots[listener_index];
        listener.backlog[usize::from(listener.backlog_len)] = Some(socket);
        listener.backlog_len += 1;
        Ok(())
    }

    fn accept(&mut self, socket: SocketId) -> Result<Option<SocketId>, SocketError> {
        let listener_index = self.resolve(socket)?;
        if self.slots[listener_index].state != SocketState::Listening {
            return Err(SocketError::InvalidState);
        }
        loop {
            let Some(client_id) = self.pop_backlog(listener_index) else {
                return Ok(None);
            };
            let Ok(client_index) = self.resolve(client_id) else {
                continue;
            };
            if self.slots[client_index].state != SocketState::Connecting {
                continue;
            }
            let Some(server_index) = self.slots.iter().position(|slot| !slot.used) else {
                self.push_backlog_front(listener_index, client_id);
                return Err(SocketError::NoResources);
            };
            let listener_local = (
                self.slots[listener_index].local_addr,
                self.slots[listener_index].local_port,
            );
            let client_local = (
                self.slots[client_index].local_addr,
                self.slots[client_index].local_port,
            );
            let server_id = {
                let server = &mut self.slots[server_index];
                server.reset_active();
                server.state = SocketState::Established;
                server.local_addr = listener_local.0;
                server.local_port = listener_local.1;
                server.remote_addr = client_local.0;
                server.remote_port = client_local.1;
                server.peer = Some(client_id);
                make_id(server_index, server.generation)
            };
            let client = &mut self.slots[client_index];
            client.state = SocketState::Established;
            client.peer = Some(server_id);
            self.events.push(client_id, SocketEvent::Connected);
            return Ok(Some(server_id));
        }
    }

    fn send(&mut self, socket: SocketId, data: &[u8]) -> Result<usize, SocketError> {
        let index = self.resolve(socket)?;
        match self.slots[index].state {
            SocketState::Established => {}
            SocketState::Closed => return Err(SocketError::Closed),
            _ => return Err(SocketError::NotConnected),
        }
        if !self.link_up {
            return Err(SocketError::NotConnected);
        }
        if data.is_empty() {
            return Ok(0);
        }
        if self.drop_data {
            self.events.push(
                socket,
                SocketEvent::DataSent {
                    length: saturate_u16(data.len()),
                },
            );
            return Ok(data.len());
        }
        let peer_id = self.slots[index].peer.ok_or(SocketError::NotConnected)?;
        let peer_index = self
            .resolve(peer_id)
            .map_err(|_| SocketError::NotConnected)?;
        let accepted = self.slots[peer_index].rx.push(data);
        if accepted == 0 {
            return Err(SocketError::BufferFull);
        }
        self.events.push(
            socket,
            SocketEvent::DataSent {
                length: saturate_u16(accepted),
            },
        );
        self.events.push(
            peer_id,
            SocketEvent::DataReceived {
                length: saturate_u16(accepted),
            },
        );
        Ok(accepted)
    }

    fn recv(&mut self, socket: SocketId, buf: &mut [u8]) -> Result<usize, SocketError> {
        let index = self.resolve(socket)?;
        match self.slots[index].state {
            SocketState::Established | SocketState::Closed => {}
            _ => return Err(SocketError::NotConnected),
        }
        Ok(self.slots[index].rx.pop(buf))
    }

    fn close(&mut self, socket: SocketId) -> Result<(), SocketError> {
        let index = self.resolve(socket)?;
        self.release(index, ConnectionCloseReason::ClosedByPeer);
        Ok(())
    }

    fn abort(&mut self, socket: SocketId) {
        if let Ok(index) = self.resolve(socket) {
            self.release(index, ConnectionCloseReason::Reset);
        }
    }

    fn state(&self, socket: SocketId) -> Result<SocketState, SocketError> {
        let index = self.resolve(socket)?;
        Ok(self.slots[index].state)
    }

    fn local_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve(socket)?;
        let slot = &self.slots[index];
        if slot.state == SocketState::Created {
            Ok(IpEndpoint::unset())
        } else {
            Ok(IpEndpoint::new(slot.local_addr, slot.local_port))
        }
    }

    fn remote_endpoint(&self, socket: SocketId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve(socket)?;
        let slot = &self.slots[index];
        match slot.state {
            SocketState::Created | SocketState::Bound | SocketState::Listening => {
                Ok(IpEndpoint::unset())
            }
            _ => Ok(IpEndpoint::new(slot.remote_addr, slot.remote_port)),
        }
    }

    fn poll_event(&mut self) -> Option<(SocketId, SocketEvent)> {
        self.events.pop()
    }
}

impl<const SOCKETS: usize, const RX_CAPACITY: usize, const EVENT_CAPACITY: usize> DatagramApi
    for FakeStack<SOCKETS, RX_CAPACITY, EVENT_CAPACITY>
{
    fn create_datagram(&mut self) -> Result<DatagramId, SocketError> {
        for index in 0..UDP_SOCKETS {
            if !self.udp_slots[index].used {
                self.udp_slots[index].reset_active();
                return Ok(make_udp_id(index, self.udp_slots[index].generation));
            }
        }
        Err(SocketError::NoResources)
    }

    fn bind_datagram(
        &mut self,
        socket: DatagramId,
        addr: IpAddress,
        port: u16,
    ) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        if self.udp_slots[index].bound {
            return Err(SocketError::InvalidState);
        }
        if port != 0 {
            let conflict = self.udp_slots.iter().enumerate().any(|(i, slot)| {
                i != index
                    && slot.used
                    && slot.bound
                    && slot.local_port == port
                    && (slot.local_addr == addr
                        || slot.local_addr.is_unspecified()
                        || addr.is_unspecified())
            });
            if conflict {
                return Err(SocketError::AddressInUse);
            }
        }
        let assigned = if port == 0 {
            self.alloc_ephemeral()
        } else {
            port
        };
        let slot = &mut self.udp_slots[index];
        slot.bound = true;
        slot.local_addr = addr;
        slot.local_port = assigned;
        Ok(())
    }

    fn send_datagram(
        &mut self,
        socket: DatagramId,
        target: IpEndpoint,
        data: &[u8],
    ) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        if !self.link_up {
            return Err(SocketError::NotConnected);
        }
        let Some(target_port) = target.port() else {
            return Err(SocketError::InvalidState);
        };
        if data.len() > UDP_DATAGRAM_MAX {
            return Err(SocketError::BufferFull);
        }
        if !self.udp_slots[index].bound {
            // Implicit ephemeral bind, mirroring OS datagram semantics.
            let ephemeral = self.alloc_ephemeral();
            let slot = &mut self.udp_slots[index];
            slot.bound = true;
            slot.local_port = ephemeral;
        }
        let mut datagram = Datagram::empty();
        datagram.len = saturate_u16(data.len());
        datagram.addr = self.udp_slots[index].local_addr;
        datagram.port = self.udp_slots[index].local_port;
        datagram.bytes[..data.len()].copy_from_slice(data);
        let target_addr = *target.address();
        let broadcast = is_fake_broadcast(&target_addr);
        for other in 0..UDP_SOCKETS {
            if other == index {
                continue;
            }
            let slot = &self.udp_slots[other];
            if !(slot.used && slot.bound && slot.local_port == target_port) {
                continue;
            }
            let matches = broadcast
                || slot.local_addr == target_addr
                || slot.local_addr.is_unspecified()
                || target_addr.is_unspecified();
            if !matches {
                continue;
            }
            if !self.udp_slots[other].enqueue(datagram) {
                // UDP loss semantics: drop silently but observably.
                self.dropped_datagrams += 1;
            }
            if !broadcast {
                break;
            }
        }
        Ok(())
    }

    fn recv_datagram(
        &mut self,
        socket: DatagramId,
        buf: &mut [u8],
    ) -> Result<Option<(usize, IpEndpoint)>, SocketError> {
        let index = self.resolve_udp(socket)?;
        let Some(datagram) = self.udp_slots[index].dequeue() else {
            return Ok(None);
        };
        let length = usize::from(datagram.len).min(buf.len());
        buf[..length].copy_from_slice(&datagram.bytes[..length]);
        Ok(Some((
            length,
            IpEndpoint::new(datagram.addr, datagram.port),
        )))
    }

    fn datagram_local_endpoint(&self, socket: DatagramId) -> Result<IpEndpoint, SocketError> {
        let index = self.resolve_udp(socket)?;
        let slot = &self.udp_slots[index];
        if slot.bound {
            Ok(IpEndpoint::new(slot.local_addr, slot.local_port))
        } else {
            Ok(IpEndpoint::unset())
        }
    }

    fn close_datagram(&mut self, socket: DatagramId) -> Result<(), SocketError> {
        let index = self.resolve_udp(socket)?;
        let slot = &mut self.udp_slots[index];
        slot.used = false;
        slot.generation = slot.generation.wrapping_add(1);
        slot.queue = [None; UDP_QUEUE];
        slot.queue_len = 0;
        Ok(())
    }
}

impl<const SOCKETS: usize, const RX_CAPACITY: usize, const EVENT_CAPACITY: usize> LinkState
    for FakeStack<SOCKETS, RX_CAPACITY, EVENT_CAPACITY>
{
    fn link_up(&self) -> bool {
        self.link_up
    }
}

/// Build a handle from slot index and generation (index fits in the low
/// byte because `SOCKETS <= 256`).
fn make_id(index: usize, generation: u8) -> SocketId {
    SocketId::from_raw((u16::from(generation) << 8) | (index as u16))
}

/// Build a datagram handle from slot index and generation.
fn make_udp_id(index: usize, generation: u8) -> DatagramId {
    DatagramId::from_raw((u16::from(generation) << 8) | (index as u16))
}

/// Limited-broadcast or multicast target of the fake stack.
fn is_fake_broadcast(addr: &IpAddress) -> bool {
    addr.is_multicast() || addr.as_bytes() == [255, 255, 255, 255]
}

/// Clamp a byte count into `u16` (event payloads carry `u16`).
fn saturate_u16(value: usize) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_buffer_push_pop_wraparound() {
        let mut ring: RingBuffer<4> = RingBuffer::new();
        assert_eq!(ring.push(&[1, 2, 3]), 3);
        let mut out = [0u8; 2];
        assert_eq!(ring.pop(&mut out), 2);
        assert_eq!(out, [1, 2]);
        // head has advanced; pushing 3 more wraps around the array boundary
        assert_eq!(ring.push(&[4, 5, 6]), 3);
        let mut all = [0u8; 4];
        assert_eq!(ring.pop(&mut all), 4);
        assert_eq!(all, [3, 4, 5, 6]);
        assert_eq!(ring.pop(&mut all), 0);
    }

    #[test]
    fn ring_buffer_backpressure() {
        let mut ring: RingBuffer<3> = RingBuffer::new();
        assert_eq!(ring.push(&[1, 2, 3, 4]), 3);
        assert_eq!(ring.free(), 0);
        assert_eq!(ring.push(&[9]), 0);
    }

    #[test]
    fn event_queue_order_and_overflow() {
        let mut queue: EventQueue<2> = EventQueue::new();
        let id = SocketId::from_raw(1);
        queue.push(id, SocketEvent::Connected);
        queue.push(id, SocketEvent::DataReceived { length: 1 });
        queue.push(id, SocketEvent::DataReceived { length: 2 });
        assert_eq!(queue.dropped, 1);
        assert_eq!(queue.pop(), Some((id, SocketEvent::Connected)));
        assert_eq!(
            queue.pop(),
            Some((id, SocketEvent::DataReceived { length: 1 }))
        );
        assert_eq!(queue.pop(), None);
    }

    #[test]
    fn stale_generation_is_invalid() {
        let mut stack: FakeStack<2, 16, 8> = FakeStack::new();
        let socket = stack.create().unwrap();
        stack.close(socket).unwrap();
        assert_eq!(stack.state(socket), Err(SocketError::InvalidHandle));
        // A new socket reuses the slot with a bumped generation.
        let reused = stack.create().unwrap();
        assert_ne!(socket, reused);
        assert_eq!(stack.state(reused), Ok(SocketState::Created));
        assert_eq!(stack.state(socket), Err(SocketError::InvalidHandle));
    }

    #[test]
    fn make_id_roundtrip() {
        let id = make_id(3, 7);
        assert_eq!(usize::from(id.raw() & 0xFF), 3);
        assert_eq!((id.raw() >> 8) as u8, 7);
    }

    #[test]
    fn saturate_u16_clamps() {
        assert_eq!(saturate_u16(10), 10);
        assert_eq!(saturate_u16(100_000), u16::MAX);
    }
}
