//! Live POSIX `DoIP` entity composed under lifecycle control (package E30).
//!
//! Since the hardware-independent embedded `DoIP` tranche,
//! [`PosixDoIpEntity`] is a thin platform adapter: every protocol,
//! admission, timeout, diagnostic, and lifecycle decision lives in the
//! portable [`super::DoIpEntity`] core, driven here through
//! [`PosixSocketStack`] over the audited `std::net` adapters. The public
//! API of package E30 is unchanged.

use bsw_ethernet::posix::PosixSocketStack;
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::Instant;

use super::{DoIpApplication, DoIpEntity, EntityConfig, EntityError, EntityState};
use crate::{
    ActivationPolicy, DefaultActivationPolicy, DiscoveryEntity, ProtocolVersion,
    TransportParameters,
};

/// TCP socket-slot capacity of the POSIX stack: listener plus connection
/// slots plus in-flight accept headroom. [`PosixDoIpEntity`] therefore
/// supports at most `STACK_TCP_SLOTS - 2` connection slots.
pub const STACK_TCP_SLOTS: usize = 16;

/// Datagram socket-slot capacity of the POSIX stack (discovery only).
pub const STACK_UDP_SLOTS: usize = 1;

/// Restart-safe live POSIX `DoIP` entity.
///
/// `SOCKETS` is the physical connection-slot count (at most
/// `STACK_TCP_SLOTS - 2`), `RX_MESSAGES` the inbound transport pool,
/// `SEND_JOBS` the outbound job capacity, and `PAYLOAD` the maximum
/// application payload held by either path.
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
    core: DoIpEntity<A, P, SOCKETS, RX_MESSAGES, SEND_JOBS, PAYLOAD>,
    stack: PosixSocketStack<STACK_TCP_SLOTS, STACK_UDP_SLOTS>,
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
        const {
            assert!(
                SOCKETS + 2 <= STACK_TCP_SLOTS,
                "SOCKETS exceeds the POSIX stack socket pool"
            );
        }
        Self {
            core: DoIpEntity::new(
                config,
                version,
                entity_address,
                parameters,
                discovery_entity,
                application,
                activation_policy,
            ),
            stack: PosixSocketStack::new(),
        }
    }

    /// Current lifecycle state.
    pub const fn state(&self) -> EntityState {
        self.core.state()
    }

    /// Bound TCP port, or zero while stopped.
    pub const fn tcp_port(&self) -> u16 {
        self.core.tcp_port()
    }

    /// Bound UDP discovery port, or zero while stopped.
    pub const fn discovery_port(&self) -> u16 {
        self.core.discovery_port()
    }

    /// Number of successfully admitted TCP sockets since construction.
    pub const fn accepted_connections(&self) -> u32 {
        self.core.accepted_connections()
    }

    /// Number of sockets closed because of malformed input or peer teardown.
    pub const fn malformed_or_closed(&self) -> u32 {
        self.core.malformed_or_closed()
    }

    /// Shared application access for tests and composition diagnostics.
    pub const fn application(&self) -> &A {
        self.core.application()
    }

    /// Mutable application access.
    pub fn application_mut(&mut self) -> &mut A {
        self.core.application_mut()
    }

    /// Portable-core access for equivalence and diagnostics inspection.
    pub const fn core(&self) -> &DoIpEntity<A, P, SOCKETS, RX_MESSAGES, SEND_JOBS, PAYLOAD> {
        &self.core
    }

    /// Poll UDP, accept TCP clients, process bytes, timers, and pending
    /// sends. All protocol deadlines use the caller-provided monotonic
    /// instant.
    pub fn poll(&mut self, now: Instant) -> Result<(), EntityError> {
        self.core.poll(now, &mut self.stack)
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
        if self.core.state() == EntityState::Running {
            return TransitionResult::Error;
        }
        match self.core.start(&mut self.stack) {
            Ok(()) => TransitionResult::Done,
            Err(_) => TransitionResult::Error,
        }
    }

    fn run(&mut self) -> TransitionResult {
        match self.core.enable(Instant::from_nanos(0)) {
            Ok(()) => TransitionResult::Done,
            Err(_) => TransitionResult::Error,
        }
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.core.stop(&mut self.stack);
        TransitionResult::Done
    }

    fn name(&self) -> &str {
        "DoIpServerSystem"
    }
}
