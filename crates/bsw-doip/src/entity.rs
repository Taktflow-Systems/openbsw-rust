//! Live `DoIP` entity: portable `no_std` core plus platform adapters.
//!
//! [`DoIpEntity`] owns every protocol decision of a live `DoIP` node —
//! discovery, announcements, admission, routing activation, alive checks,
//! timeouts, diagnostic bridging, backpressure, lifecycle, and teardown —
//! on top of the generic, allocation-free stack boundary of
//! `bsw-ethernet` ([`SocketApi`] / [`DatagramApi`] / [`LinkState`]).
//! It never touches an operating system, `libc`, `std::net`, threads, heap
//! collections, or platform socket types; a backend is borrowed per call,
//! and all capacities are const-generic or named constants.
//!
//! The POSIX adapter ([`posix::PosixDoIpEntity`], `std` only) composes the
//! same core with [`bsw_ethernet::posix::PosixSocketStack`] and keeps the
//! historical package E30 public API.
//!
//! # Backend contract
//!
//! One entity must be driven by exactly one backend instance between
//! [`DoIpEntity::start`] and [`DoIpEntity::stop`]; handles are
//! backend-scoped and generation-checked, so a stale or foreign handle is
//! rejected by the backend and treated as a connection loss by the entity.

#[cfg(feature = "std")]
pub mod posix;

use bsw_ethernet::{
    endpoint::IpEndpoint,
    ip::IpAddress,
    lwip::{DatagramApi, DatagramId, LinkState, SocketApi, SocketError, SocketId, SocketState},
};
use bsw_time::Instant;
use bsw_transport::pool::{AdmissionPolicy, AllocationRequest, MessageHandle, ProviderError};
use bsw_transport::TransportMessage;

use crate::diagnostic::{
    DiagnosticMessageHandler, DiagnosticMessageListener, DiagnosticReceiveState,
    DiagnosticSendToken, DiagnosticSender, PoolDiagnosticGateway,
};
use crate::server::ConnectionId;
use crate::{
    ActivationPolicy, CloseMode, DefaultActivationPolicy, DiscoveryEntity, Frame, ProtocolVersion,
    ServerTransportLayer, TransportParameters, WireAction,
};

/// Listener and protocol configuration shared by every backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityConfig {
    /// Address used for the TCP diagnostic listener.
    pub tcp_address: IpAddress,
    /// TCP port, where zero requests an ephemeral test port.
    pub tcp_port: u16,
    /// Address used for UDP discovery.
    pub udp_address: IpAddress,
    /// UDP port, where zero requests an ephemeral test port.
    pub udp_port: u16,
    /// Transport bus identifier written into received messages.
    pub source_bus: u8,
}

impl Default for EntityConfig {
    fn default() -> Self {
        Self {
            tcp_address: IpAddress::unspecified(),
            tcp_port: crate::TCP_DATA_PORT,
            udp_address: IpAddress::unspecified(),
            udp_port: crate::UDP_DISCOVERY_PORT,
            source_bus: 0,
        }
    }
}

/// Lifecycle state of a live `DoIP` entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityState {
    /// No sockets are owned.
    Stopped,
    /// UDP and TCP listeners are bound but polling is disabled.
    Initialized,
    /// The entity accepts and processes traffic.
    Running,
    /// The last transition or poll failed.
    Faulted(EntityError),
}

/// Observable entity failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityError {
    /// TCP listener bind or accept failed.
    Tcp,
    /// UDP discovery bind or receive failed.
    Udp,
    /// A socket slot and server-transport slot became inconsistent.
    Slot,
}

/// Application boundary shared by `DoIP` and other diagnostic transports.
pub trait DoIpApplication<const PAYLOAD: usize>: DiagnosticMessageListener<PAYLOAD> {
    /// Remove the next UDS response prepared while receiving a request.
    fn take_response(&mut self) -> Option<TransportMessage<PAYLOAD>>;
}

/// Admission policy accepting one bus and one `DoIP` entity address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityAdmissionPolicy {
    source_bus: u8,
    entity_address: u16,
}

impl EntityAdmissionPolicy {
    /// Create a receive-pool policy.
    pub const fn new(source_bus: u8, entity_address: u16) -> Self {
        Self {
            source_bus,
            entity_address,
        }
    }
}

impl AdmissionPolicy for EntityAdmissionPolicy {
    fn admit(&self, request: &AllocationRequest<'_>) -> Result<(), ProviderError> {
        if request.source_bus != self.source_bus {
            Err(ProviderError::NotResponsible)
        } else if request.source == u16::MAX {
            Err(ProviderError::InvalidSourceAddress)
        } else if request.target != self.entity_address {
            Err(ProviderError::InvalidTargetAddress)
        } else {
            Ok(())
        }
    }
}

/// Per-connection receive chunk copied out of the backend per poll cycle.
pub const RECV_CHUNK: usize = 256;

/// Staging buffer for one discovery request or response datagram.
pub const DISCOVERY_BUFFER: usize = 128;

/// Maximum discovery datagrams handled per poll cycle (receive pressure
/// stays bounded; the remainder is handled on the next cycle).
pub const UDP_POLL_BUDGET: usize = 8;

/// In-flight transmit progress of one wire action.
#[derive(Debug, Clone, Copy)]
enum PendingSend {
    /// A control frame, partially written.
    Control { frame: Frame, sent: usize },
    /// A diagnostic send job, partially written (offset spans the 12-byte
    /// prefix followed by the payload bytes).
    Diagnostic {
        token: DiagnosticSendToken,
        sent: usize,
    },
}

/// Progress of a nonblocking send attempt.
enum SendProgress {
    /// All bytes accepted by the backend.
    Done,
    /// Backpressure: retry on a later poll.
    Blocked,
    /// The socket is unusable.
    Failed,
}

#[derive(Debug, Clone, Copy)]
struct ConnSlot {
    id: ConnectionId,
    handle: SocketId,
    pending: Option<PendingSend>,
}

/// Restart-safe, allocation-free live `DoIP` entity over a generic stack.
///
/// `SOCKETS` is the connection-slot count, `RX_MESSAGES` the inbound
/// transport pool, `SEND_JOBS` the outbound job capacity, and `PAYLOAD` the
/// maximum application payload held by either path. Together with
/// [`RECV_CHUNK`], [`DISCOVERY_BUFFER`], [`UDP_POLL_BUDGET`], and the
/// wire/event queues of [`ServerTransportLayer`], every capacity of the
/// portable path is explicit and statically bounded.
pub struct DoIpEntity<
    A,
    P = DefaultActivationPolicy,
    const SOCKETS: usize = 6,
    const RX_MESSAGES: usize = 6,
    const SEND_JOBS: usize = 6,
    const PAYLOAD: usize = 4096,
> where
    A: DoIpApplication<PAYLOAD>,
    P: ActivationPolicy,
{
    config: EntityConfig,
    announcement_target: Option<IpEndpoint>,
    version: ProtocolVersion,
    entity_address: u16,
    parameters: TransportParameters,
    state: EntityState,
    listener: Option<SocketId>,
    discovery_socket: Option<DatagramId>,
    tcp_port: u16,
    udp_port: u16,
    discovery: DiscoveryEntity,
    sockets: [Option<ConnSlot>; SOCKETS],
    /// Per-slot in-flight diagnostic receive state, persisted across
    /// receive cycles so fragmented messages survive chunked reads.
    receive_states: [DiagnosticReceiveState<MessageHandle>; SOCKETS],
    transport: ServerTransportLayer<SOCKETS>,
    gateway: PoolDiagnosticGateway<RX_MESSAGES, PAYLOAD, EntityAdmissionPolicy, A>,
    sender: DiagnosticSender<SEND_JOBS, PAYLOAD>,
    activation_policy: P,
    accepted_connections: u32,
    malformed_or_closed: u32,
    backpressure_events: u32,
    resource_rejections: u32,
    link_up: bool,
}

impl<
        A,
        P,
        const SOCKETS: usize,
        const RX_MESSAGES: usize,
        const SEND_JOBS: usize,
        const PAYLOAD: usize,
    > DoIpEntity<A, P, SOCKETS, RX_MESSAGES, SEND_JOBS, PAYLOAD>
where
    A: DoIpApplication<PAYLOAD>,
    P: ActivationPolicy,
{
    /// Construct an entity without opening sockets.
    pub fn new(
        config: EntityConfig,
        version: ProtocolVersion,
        entity_address: u16,
        parameters: TransportParameters,
        discovery_entity: DiscoveryEntity,
        application: A,
        activation_policy: P,
    ) -> Self {
        const {
            assert!(SOCKETS >= 1, "at least one connection slot is required");
            assert!(RX_MESSAGES >= 1, "at least one receive message is required");
            assert!(SEND_JOBS >= 1, "at least one send job is required");
            assert!(
                PAYLOAD >= 5,
                "payload must hold a routing activation request body"
            );
        }
        Self {
            config,
            announcement_target: None,
            version,
            entity_address,
            parameters,
            state: EntityState::Stopped,
            listener: None,
            discovery_socket: None,
            tcp_port: 0,
            udp_port: 0,
            discovery: discovery_entity,
            sockets: [None; SOCKETS],
            receive_states: [DiagnosticReceiveState::new(); SOCKETS],
            transport: ServerTransportLayer::new(version, entity_address, parameters),
            gateway: PoolDiagnosticGateway::new(
                EntityAdmissionPolicy::new(config.source_bus, entity_address),
                application,
            ),
            sender: DiagnosticSender::new(version),
            activation_policy,
            accepted_connections: 0,
            malformed_or_closed: 0,
            backpressure_events: 0,
            resource_rejections: 0,
            link_up: true,
        }
    }

    /// Address unsolicited vehicle announcements to `target` (typically a
    /// broadcast endpoint). Announcements stay disabled while unset.
    pub fn set_announcement_target(&mut self, target: Option<IpEndpoint>) {
        self.announcement_target = target;
    }

    /// Current lifecycle state.
    pub const fn state(&self) -> EntityState {
        self.state
    }

    /// Bound TCP port, or zero while stopped.
    pub const fn tcp_port(&self) -> u16 {
        self.tcp_port
    }

    /// Bound UDP discovery port, or zero while stopped.
    pub const fn discovery_port(&self) -> u16 {
        self.udp_port
    }

    /// Number of successfully admitted TCP sockets since construction.
    pub const fn accepted_connections(&self) -> u32 {
        self.accepted_connections
    }

    /// Number of sockets closed because of malformed input or peer teardown.
    pub const fn malformed_or_closed(&self) -> u32 {
        self.malformed_or_closed
    }

    /// Number of transmit attempts deferred by socket backpressure.
    pub const fn backpressure_events(&self) -> u32 {
        self.backpressure_events
    }

    /// Number of connections or datagrams refused because a bounded
    /// resource (socket pool, connection slots, admission) was exhausted.
    pub const fn resource_rejections(&self) -> u32 {
        self.resource_rejections
    }

    /// Number of routing (`Active`) connections.
    pub fn routing_count(&self) -> usize {
        self.transport.routing_count()
    }

    /// Number of admitted connections (any non-released state).
    pub fn connection_count(&self) -> usize {
        self.transport.connection_count()
    }

    /// Receive-pool occupancy (bounded by `RX_MESSAGES`).
    pub fn receive_pool_in_use(&self) -> usize {
        self.gateway.in_use()
    }

    /// Send-job occupancy (bounded by `SEND_JOBS`).
    pub fn send_jobs_in_use(&self) -> usize {
        self.sender.in_use()
    }

    /// Shared application access for tests and composition diagnostics.
    pub const fn application(&self) -> &A {
        self.gateway.listener()
    }

    /// Mutable application access.
    pub fn application_mut(&mut self) -> &mut A {
        self.gateway.listener_mut()
    }

    /// Dynamic discovery configuration (power mode, announcements).
    pub fn discovery_mut(&mut self) -> &mut DiscoveryEntity {
        &mut self.discovery
    }

    /// Bind the discovery socket and TCP listener on `net`.
    ///
    /// Idempotent from [`EntityState::Initialized`]; any owned resource is
    /// released first, so a faulted or stopped entity can always be
    /// restarted deterministically.
    pub fn start<N>(&mut self, net: &mut N) -> Result<(), EntityError>
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        if self.state == EntityState::Initialized {
            return Ok(());
        }
        self.stop(net);
        let udp = net
            .create_datagram()
            .map_err(|_| self.fault(EntityError::Udp))?;
        self.discovery_socket = Some(udp);
        if net
            .bind_datagram(udp, self.config.udp_address, self.config.udp_port)
            .is_err()
        {
            self.stop(net);
            return Err(self.fault(EntityError::Udp));
        }
        self.udp_port = net
            .datagram_local_endpoint(udp)
            .ok()
            .and_then(|endpoint| endpoint.port())
            .unwrap_or(0);
        let Ok(listener) = net.create() else {
            self.stop(net);
            return Err(self.fault(EntityError::Tcp));
        };
        self.listener = Some(listener);
        let backlog = u8::try_from(SOCKETS).unwrap_or(u8::MAX);
        if net
            .bind(listener, self.config.tcp_address, self.config.tcp_port)
            .is_err()
            || net.listen(listener, backlog).is_err()
        {
            self.stop(net);
            return Err(self.fault(EntityError::Tcp));
        }
        self.tcp_port = net
            .local_endpoint(listener)
            .ok()
            .and_then(|endpoint| endpoint.port())
            .unwrap_or(0);
        self.state = EntityState::Initialized;
        Ok(())
    }

    /// Enable traffic processing. Announcement bursts start at `now` when a
    /// target is configured.
    pub fn enable(&mut self, now: Instant) -> Result<(), EntityError> {
        if self.state != EntityState::Initialized {
            return Err(EntityError::Slot);
        }
        self.state = EntityState::Running;
        self.link_up = true;
        if self.announcement_target.is_some() {
            self.discovery.start_announcements(now);
        }
        Ok(())
    }

    /// Poll UDP, accept TCP clients, process bytes, timers, and pending
    /// sends. All protocol deadlines use the caller-provided monotonic
    /// instant.
    pub fn poll<N>(&mut self, now: Instant, net: &mut N) -> Result<(), EntityError>
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        if self.state != EntityState::Running {
            return Ok(());
        }
        if !self.observe_link(now, net) {
            return Ok(());
        }
        self.poll_discovery(now, net)?;
        self.accept_clients(now, net)?;
        self.read_clients(now, net);
        self.transport.poll(now);
        self.queue_application_responses();
        self.drain_wire(net);
        self.update_discovery_occupancy();
        Ok(())
    }

    /// Release every owned socket and deterministically reinitialize all
    /// bounded resources. Safe from any state.
    pub fn stop<N>(&mut self, net: &mut N)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        // Give queued frames one orderly flush attempt, then close.
        self.transport.close_all(CloseMode::Close);
        self.transport.poll(Instant::from_nanos(0));
        self.drain_wire(net);
        for index in 0..SOCKETS {
            if let Some(owned) = self.sockets[index].take() {
                let _ = net.close(owned.handle);
            }
            self.receive_states[index].cancel(&mut self.gateway);
        }
        if let Some(listener) = self.listener.take() {
            let _ = net.close(listener);
        }
        if let Some(udp) = self.discovery_socket.take() {
            let _ = net.close_datagram(udp);
        }
        self.discovery.stop_announcements();
        self.transport =
            ServerTransportLayer::new(self.version, self.entity_address, self.parameters);
        self.sender = DiagnosticSender::new(self.version);
        self.tcp_port = 0;
        self.udp_port = 0;
        self.link_up = true;
        self.state = EntityState::Stopped;
    }

    // -- link ---------------------------------------------------------------

    /// Track link readiness; returns `false` while the link is down.
    fn observe_link<N>(&mut self, now: Instant, net: &mut N) -> bool
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        let up = net.link_up();
        if up == self.link_up {
            return up;
        }
        self.link_up = up;
        if up {
            // Recovery: restart the announcement burst like a fresh network
            // attach; listener and discovery socket are retained.
            if self.announcement_target.is_some() {
                self.discovery.start_announcements(now);
            }
            return true;
        }
        // Link loss: abort every connection and reinitialize the bounded
        // transmit state so recovery starts clean.
        for index in 0..SOCKETS {
            if let Some(owned) = self.sockets[index].take() {
                net.abort(owned.handle);
                self.receive_states[index].cancel(&mut self.gateway);
                self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
            }
        }
        self.transport =
            ServerTransportLayer::new(self.version, self.entity_address, self.parameters);
        self.sender = DiagnosticSender::new(self.version);
        self.discovery.stop_announcements();
        let _ = self.discovery.set_open_sockets(0);
        false
    }

    // -- discovery ----------------------------------------------------------

    fn poll_discovery<N>(&mut self, now: Instant, net: &mut N) -> Result<(), EntityError>
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        let Some(udp) = self.discovery_socket else {
            return Err(self.fault(EntityError::Udp));
        };
        let mut input = [0; DISCOVERY_BUFFER];
        let mut output = [0; DISCOVERY_BUFFER];
        for _ in 0..UDP_POLL_BUDGET {
            let received = match net.recv_datagram(udp, &mut input) {
                Ok(Some(received)) => received,
                Ok(None) => break,
                Err(_) => return Err(self.fault(EntityError::Udp)),
            };
            let (length, source) = received;
            let Ok(response_length) = self
                .discovery
                .handle_datagram(&input[..length], &mut output)
            else {
                // Malformed discovery datagrams are ignored (UDP has no
                // NACK path in the upstream discovery service).
                continue;
            };
            if response_length == 0 {
                continue;
            }
            match net.send_datagram(udp, source, &output[..response_length]) {
                Ok(()) => {}
                Err(SocketError::BufferFull) => {
                    self.backpressure_events = self.backpressure_events.saturating_add(1);
                }
                Err(_) => return Err(self.fault(EntityError::Udp)),
            }
        }
        if let Some(target) = self.announcement_target {
            loop {
                let Ok(length) = self.discovery.poll_announcement(now, &mut output) else {
                    break;
                };
                if length == 0 {
                    break;
                }
                match net.send_datagram(udp, target, &output[..length]) {
                    Ok(()) => {}
                    Err(SocketError::BufferFull) => {
                        self.backpressure_events = self.backpressure_events.saturating_add(1);
                    }
                    Err(_) => return Err(self.fault(EntityError::Udp)),
                }
            }
        }
        Ok(())
    }

    // -- admission ----------------------------------------------------------

    fn accept_clients<N>(&mut self, now: Instant, net: &mut N) -> Result<(), EntityError>
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        let Some(listener) = self.listener else {
            return Err(self.fault(EntityError::Tcp));
        };
        loop {
            let handle = match net.accept(listener) {
                Ok(Some(handle)) => handle,
                Ok(None) => break,
                Err(SocketError::NoResources) => {
                    // Backend socket pool exhausted; stays observable and
                    // is retried on the next cycle.
                    self.resource_rejections = self.resource_rejections.saturating_add(1);
                    break;
                }
                Err(_) => return Err(self.fault(EntityError::Tcp)),
            };
            match self.transport.accept(now, &mut self.activation_policy) {
                Ok(id) if id.index() < SOCKETS && self.sockets[id.index()].is_none() => {
                    self.sockets[id.index()] = Some(ConnSlot {
                        id,
                        handle,
                        pending: None,
                    });
                    self.accepted_connections = self.accepted_connections.saturating_add(1);
                }
                Ok(_) | Err(_) => {
                    self.resource_rejections = self.resource_rejections.saturating_add(1);
                    net.abort(handle);
                }
            }
        }
        Ok(())
    }

    // -- receive ------------------------------------------------------------

    fn read_clients<N>(&mut self, now: Instant, net: &mut N)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        let mut input = [0; RECV_CHUNK];
        for index in 0..SOCKETS {
            let Some(slot) = self.sockets[index] else {
                continue;
            };
            let id = slot.id;
            match net.recv(slot.handle, &mut input) {
                Ok(0) => {
                    let closed = !matches!(net.state(slot.handle), Ok(SocketState::Established));
                    if closed {
                        let _ = self.transport.mark_close(id, CloseMode::Close);
                        self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
                    }
                }
                Ok(length) => {
                    let source = self.transport.source_address(id);
                    let mut handler = DiagnosticMessageHandler::new(
                        self.version,
                        self.config.source_bus,
                        source,
                        self.entity_address,
                        self.parameters.max_payload_size as usize,
                        &mut self.gateway,
                    );
                    handler.resume(&mut self.receive_states[index]);
                    let _ = self.transport.handle_bytes(
                        now,
                        id,
                        &input[..length],
                        &mut self.activation_policy,
                        &mut handler,
                    );
                    handler.suspend(&mut self.receive_states[index]);
                    let _ = handler.apply_actions(&mut self.transport, id);
                }
                Err(_) => {
                    let _ = self.transport.mark_close(id, CloseMode::Close);
                    self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
                }
            }
        }
        self.queue_application_responses();
    }

    fn queue_application_responses(&mut self) {
        while let Some(response) = self.gateway.listener_mut().take_response() {
            let _ = self.sender.submit(&mut self.transport, &response);
        }
    }

    // -- transmit -----------------------------------------------------------

    fn drain_wire<N>(&mut self, net: &mut N)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        for index in 0..SOCKETS {
            self.drain_slot(net, index);
        }
    }

    /// Drain one connection's wire queue in order, honouring an in-flight
    /// partial send. Other connections are unaffected by this slot's
    /// backpressure.
    fn drain_slot<N>(&mut self, net: &mut N, index: usize)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        loop {
            let Some(slot) = self.sockets[index] else {
                return;
            };
            if let Some(pending) = slot.pending {
                match self.continue_send(net, slot.handle, pending) {
                    (SendProgress::Done, _) => {
                        if let Some(entry) = self.sockets[index].as_mut() {
                            entry.pending = None;
                        }
                        if let PendingSend::Diagnostic { token, .. } = pending {
                            let _ = self.sender.complete(token);
                        }
                    }
                    (SendProgress::Blocked, updated) => {
                        self.backpressure_events = self.backpressure_events.saturating_add(1);
                        if let Some(entry) = self.sockets[index].as_mut() {
                            entry.pending = Some(updated);
                        }
                        return;
                    }
                    (SendProgress::Failed, _) => {
                        if let PendingSend::Diagnostic { token, .. } = pending {
                            let _ = self.sender.complete(token);
                        }
                        if let Some(entry) = self.sockets[index].as_mut() {
                            entry.pending = None;
                        }
                        let _ = self.transport.mark_close(slot.id, CloseMode::Abort);
                        return;
                    }
                }
                continue;
            }
            match self.transport.take_action_for(slot.id) {
                None => return,
                Some(WireAction::Send(frame)) => {
                    if let Some(entry) = self.sockets[index].as_mut() {
                        entry.pending = Some(PendingSend::Control { frame, sent: 0 });
                    }
                }
                Some(WireAction::SendDiagnostic(token)) => {
                    if let Some(entry) = self.sockets[index].as_mut() {
                        entry.pending = Some(PendingSend::Diagnostic { token, sent: 0 });
                    }
                }
                Some(WireAction::Close) => {
                    self.finish_close(net, index, false);
                    return;
                }
                Some(WireAction::Abort) => {
                    self.finish_close(net, index, true);
                    return;
                }
            }
        }
    }

    /// Close or abort one socket and drop any queued follow-up actions,
    /// completing diagnostic tokens so send jobs never leak.
    fn finish_close<N>(&mut self, net: &mut N, index: usize, abort: bool)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        let Some(slot) = self.sockets[index].take() else {
            return;
        };
        if abort {
            net.abort(slot.handle);
        } else {
            let _ = net.close(slot.handle);
        }
        if let Some(PendingSend::Diagnostic { token, .. }) = slot.pending {
            let _ = self.sender.complete(token);
        }
        while let Some(action) = self.transport.take_action_for(slot.id) {
            if let WireAction::SendDiagnostic(token) = action {
                let _ = self.sender.complete(token);
            }
        }
        self.receive_states[index].cancel(&mut self.gateway);
        self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
    }

    /// Advance one pending send; returns the progress and updated state.
    fn continue_send<N>(
        &mut self,
        net: &mut N,
        handle: SocketId,
        pending: PendingSend,
    ) -> (SendProgress, PendingSend)
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        match pending {
            PendingSend::Control { frame, mut sent } => {
                let progress = Self::push_bytes(net, handle, frame.bytes(), &mut sent);
                (progress, PendingSend::Control { frame, sent })
            }
            PendingSend::Diagnostic { token, mut sent } => {
                let Ok(wire) = self.sender.wire_frame(token) else {
                    return (SendProgress::Failed, pending);
                };
                let prefix = wire.prefix();
                if sent < prefix.len() {
                    let mut offset = sent;
                    let progress = Self::push_bytes(net, handle, prefix, &mut offset);
                    sent = offset;
                    match progress {
                        SendProgress::Done => {}
                        other => return (other, PendingSend::Diagnostic { token, sent }),
                    }
                }
                let mut offset = sent - prefix.len();
                let progress = Self::push_bytes(net, handle, wire.data(), &mut offset);
                sent = prefix.len() + offset;
                (progress, PendingSend::Diagnostic { token, sent })
            }
        }
    }

    /// Push `bytes[*sent..]` without blocking, advancing `sent`.
    fn push_bytes<N>(net: &mut N, handle: SocketId, bytes: &[u8], sent: &mut usize) -> SendProgress
    where
        N: SocketApi + DatagramApi + LinkState,
    {
        while *sent < bytes.len() {
            match net.send(handle, &bytes[*sent..]) {
                Ok(0) | Err(SocketError::BufferFull) => return SendProgress::Blocked,
                Ok(accepted) => *sent += accepted,
                Err(_) => return SendProgress::Failed,
            }
        }
        SendProgress::Done
    }

    // -- shared helpers -----------------------------------------------------

    fn update_discovery_occupancy(&mut self) {
        let open = u8::try_from(self.transport.connection_count()).unwrap_or(u8::MAX);
        let _ = self.discovery.set_open_sockets(open);
    }

    fn fault(&mut self, error: EntityError) -> EntityError {
        self.state = EntityState::Faulted(error);
        error
    }
}
