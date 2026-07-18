//! Live POSIX DoIP entity composed under lifecycle control (package E30).

use std::vec::Vec;

use bsw_ethernet::{
    ip::IpAddress,
    posix::tcp::{PosixTcpServerSocket, PosixTcpSocket},
    tcp::{TcpError, TcpServerSocket, TcpSocket},
    udp::UdpError,
};
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::Instant;
use bsw_transport::pool::{AdmissionPolicy, AllocationRequest, ProviderError};
use bsw_transport::TransportMessage;

use crate::diagnostic::{
    DiagnosticMessageHandler, DiagnosticMessageListener, DiagnosticSender, PoolDiagnosticGateway,
};
use crate::discovery::posix::PosixDiscoveryService;
use crate::{
    ActivationPolicy, CloseMode, DefaultActivationPolicy, DiscoveryEntity, ProtocolVersion,
    ServerTransportLayer, TransportParameters, WireAction,
};

/// POSIX listener and protocol configuration.
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

/// Lifecycle state of a POSIX DoIP entity.
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

/// Application boundary shared by DoIP and other diagnostic transports.
pub trait DoIpApplication<const PAYLOAD: usize>: DiagnosticMessageListener<PAYLOAD> {
    /// Remove the next UDS response prepared while receiving a request.
    fn take_response(&mut self) -> Option<TransportMessage<PAYLOAD>>;
}

/// Admission policy accepting one bus and one DoIP entity address.
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

struct SocketSlot {
    id: crate::ConnectionId,
    socket: PosixTcpSocket,
}

/// Restart-safe live POSIX DoIP entity.
///
/// `SOCKETS` is the physical connection-slot count, `RX_MESSAGES` the inbound
/// transport pool, `SEND_JOBS` the outbound job capacity, and `PAYLOAD` the
/// maximum application payload held by either path.
pub struct PosixDoIpEntity<
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
    version: ProtocolVersion,
    entity_address: u16,
    parameters: TransportParameters,
    state: EntityState,
    listener: Option<PosixTcpServerSocket>,
    discovery: Option<PosixDiscoveryService>,
    discovery_entity: Option<DiscoveryEntity>,
    sockets: [Option<SocketSlot>; SOCKETS],
    transport: ServerTransportLayer<SOCKETS>,
    gateway: PoolDiagnosticGateway<RX_MESSAGES, PAYLOAD, EntityAdmissionPolicy, A>,
    sender: DiagnosticSender<SEND_JOBS, PAYLOAD>,
    activation_policy: P,
    accepted_connections: u32,
    malformed_or_closed: u32,
}

impl<
        A,
        P,
        const SOCKETS: usize,
        const RX_MESSAGES: usize,
        const SEND_JOBS: usize,
        const PAYLOAD: usize,
    > PosixDoIpEntity<A, P, SOCKETS, RX_MESSAGES, SEND_JOBS, PAYLOAD>
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
        Self {
            config,
            version,
            entity_address,
            parameters,
            state: EntityState::Stopped,
            listener: None,
            discovery: None,
            discovery_entity: Some(discovery_entity),
            sockets: core::array::from_fn(|_| None),
            transport: ServerTransportLayer::new(version, entity_address, parameters),
            gateway: PoolDiagnosticGateway::new(
                EntityAdmissionPolicy::new(config.source_bus, entity_address),
                application,
            ),
            sender: DiagnosticSender::new(version),
            activation_policy,
            accepted_connections: 0,
            malformed_or_closed: 0,
        }
    }

    /// Current lifecycle state.
    pub const fn state(&self) -> EntityState {
        self.state
    }

    /// Bound TCP port, or zero while stopped.
    pub fn tcp_port(&self) -> u16 {
        self.listener
            .as_ref()
            .map_or(0, TcpServerSocket::local_port)
    }

    /// Bound UDP discovery port, or zero while stopped.
    pub fn discovery_port(&self) -> u16 {
        self.discovery
            .as_ref()
            .map_or(0, PosixDiscoveryService::local_port)
    }

    /// Number of successfully admitted TCP sockets since construction.
    pub const fn accepted_connections(&self) -> u32 {
        self.accepted_connections
    }

    /// Number of sockets closed because of malformed input or peer teardown.
    pub const fn malformed_or_closed(&self) -> u32 {
        self.malformed_or_closed
    }

    /// Shared application access for tests and composition diagnostics.
    pub const fn application(&self) -> &A {
        self.gateway.listener()
    }

    /// Mutable application access.
    pub fn application_mut(&mut self) -> &mut A {
        self.gateway.listener_mut()
    }

    /// Poll UDP, accept TCP clients, process bytes, timers, and pending sends.
    /// All protocol deadlines use the caller-provided monotonic instant.
    pub fn poll(&mut self, now: Instant) -> Result<(), EntityError> {
        if self.state != EntityState::Running {
            return Ok(());
        }
        self.poll_discovery()?;
        self.accept_clients(now)?;
        self.read_clients(now);
        self.transport.poll(now);
        self.queue_application_responses();
        self.drain_wire();
        self.update_discovery_occupancy();
        Ok(())
    }

    fn poll_discovery(&mut self) -> Result<(), EntityError> {
        let Some(discovery) = self.discovery.as_mut() else {
            return Err(EntityError::Udp);
        };
        let mut input = [0; 64];
        let mut output = [0; 64];
        match discovery.poll_client(&mut input, &mut output) {
            Ok(_) => Ok(()),
            Err(UdpError::NotOk) => self.fail(EntityError::Udp),
            Err(_) => Ok(()),
        }
    }

    fn accept_clients(&mut self, now: Instant) -> Result<(), EntityError> {
        let Some(listener) = self.listener.as_mut() else {
            return Err(EntityError::Tcp);
        };
        loop {
            let Some(mut socket) = listener.accept().map_err(|_| EntityError::Tcp)? else {
                break;
            };
            match self.transport.accept(now, &mut self.activation_policy) {
                Ok(id) if id.index() < SOCKETS && self.sockets[id.index()].is_none() => {
                    let _ = socket.set_nodelay(true);
                    self.sockets[id.index()] = Some(SocketSlot { id, socket });
                    self.accepted_connections = self.accepted_connections.saturating_add(1);
                }
                _ => socket.abort(),
            }
        }
        Ok(())
    }

    fn read_clients(&mut self, now: Instant) {
        let mut input = [0; 4096];
        for index in 0..SOCKETS {
            let Some(slot) = self.sockets[index].as_mut() else {
                continue;
            };
            let id = slot.id;
            let read = slot.socket.read(&mut input);
            match read {
                Ok(0) => {}
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
                    let _ = self.transport.handle_bytes(
                        now,
                        id,
                        &input[..length],
                        &mut self.activation_policy,
                        &mut handler,
                    );
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

    fn drain_wire(&mut self) {
        while let Some((id, action)) = self.transport.take_action() {
            let Some(slot) = self.sockets.get_mut(id.index()).and_then(Option::as_mut) else {
                if let WireAction::SendDiagnostic(token) = action {
                    let _ = self.sender.complete(token);
                }
                continue;
            };
            match action {
                WireAction::Send(frame) => {
                    if slot.socket.send(frame.bytes()) != TcpError::Ok {
                        let _ = self.transport.mark_close(id, CloseMode::Abort);
                    }
                }
                WireAction::SendDiagnostic(token) => {
                    let result = self.sender.wire_frame(token).map(|wire| {
                        let mut bytes = Vec::with_capacity(wire.len());
                        bytes.extend_from_slice(wire.prefix());
                        bytes.extend_from_slice(wire.data());
                        slot.socket.send(&bytes)
                    });
                    if result != Ok(TcpError::Ok) {
                        let _ = self.transport.mark_close(id, CloseMode::Abort);
                    }
                    let _ = self.sender.complete(token);
                }
                WireAction::Close => {
                    let _ = slot.socket.close();
                    self.sockets[id.index()] = None;
                    self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
                }
                WireAction::Abort => {
                    slot.socket.abort();
                    self.sockets[id.index()] = None;
                    self.malformed_or_closed = self.malformed_or_closed.saturating_add(1);
                }
            }
        }
    }

    fn update_discovery_occupancy(&mut self) {
        if let Some(discovery) = self.discovery.as_mut() {
            let open = u8::try_from(self.transport.connection_count()).unwrap_or(u8::MAX);
            let _ = discovery.entity_mut().set_open_sockets(open);
        }
    }

    fn fail<T>(&mut self, error: EntityError) -> Result<T, EntityError> {
        self.state = EntityState::Faulted(error);
        Err(error)
    }

    fn stop(&mut self) {
        self.transport.close_all(CloseMode::Close);
        self.transport.poll(Instant::from_nanos(0));
        self.drain_wire();
        for slot in &mut self.sockets {
            if let Some(mut owned) = slot.take() {
                let _ = owned.socket.close();
            }
        }
        if let Some(mut listener) = self.listener.take() {
            let _ = listener.close();
        }
        if let Some(discovery) = self.discovery.take() {
            self.discovery_entity = Some(discovery.into_entity());
        }
        self.transport =
            ServerTransportLayer::new(self.version, self.entity_address, self.parameters);
        self.state = EntityState::Stopped;
    }
}

impl<
        A,
        P,
        const SOCKETS: usize,
        const RX_MESSAGES: usize,
        const SEND_JOBS: usize,
        const PAYLOAD: usize,
    > LifecycleComponent for PosixDoIpEntity<A, P, SOCKETS, RX_MESSAGES, SEND_JOBS, PAYLOAD>
where
    A: DoIpApplication<PAYLOAD>,
    P: ActivationPolicy,
{
    fn init(&mut self) -> TransitionResult {
        if self.state == EntityState::Initialized {
            return TransitionResult::Done;
        }
        if self.state == EntityState::Running {
            return TransitionResult::Error;
        }
        self.stop();
        let mut listener = PosixTcpServerSocket::new();
        if listener.bind(&self.config.tcp_address, self.config.tcp_port) != TcpError::Ok {
            self.state = EntityState::Faulted(EntityError::Tcp);
            return TransitionResult::Error;
        }
        let Some(discovery_entity) = self.discovery_entity.take() else {
            self.state = EntityState::Faulted(EntityError::Slot);
            return TransitionResult::Error;
        };
        let Ok(discovery) = PosixDiscoveryService::bind(
            self.config.udp_address,
            self.config.udp_port,
            discovery_entity,
        ) else {
            self.state = EntityState::Faulted(EntityError::Udp);
            return TransitionResult::Error;
        };
        self.listener = Some(listener);
        self.discovery = Some(discovery);
        self.state = EntityState::Initialized;
        TransitionResult::Done
    }

    fn run(&mut self) -> TransitionResult {
        if self.state != EntityState::Initialized {
            return TransitionResult::Error;
        }
        self.state = EntityState::Running;
        TransitionResult::Done
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.stop();
        TransitionResult::Done
    }

    fn name(&self) -> &str {
        "DoIpServerSystem"
    }
}
