//! `DoIP` TCP server transport layer — connection slots, admission control,
//! alive-check arbitration, and teardown (E28).
//!
//! Upstream: `libs/bsw/doip/src/doip/server/DoIpServerTransportLayer.cpp`.
//!
//! One [`ServerTransportLayer`] manages one socket group.  Upstream supports
//! several socket groups per layer via `declare::DoIpServerTransportLayer
//! <NUM_SOCKET_GROUPS>`; the Rust port instantiates one layer per group,
//! which reduces the `AliveCheckHelper` pool to a single optional record.
//!
//! Like the connections it owns, the layer is sans-IO: callers accept
//! sockets, feed received bytes per connection, drive time via
//! [`ServerTransportLayer::poll`], and drain wire actions
//! ([`WireAction`]) plus application notifications ([`ServerEvent`]).

use bsw_time::Instant;

use crate::connection::{
    Action, ActivationPolicy, CloseMode, ConnectionState, Frame, MessageHandler, RingQueue,
    ServerConnection,
};
use crate::diagnostic::DiagnosticSendToken;
use crate::{ProtocolVersion, RoutingActivationCode, TransportParameters};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Handle identifying one connection slot.
///
/// Slots are reused after a connection has been fully released; a handle is
/// only valid until the layer releases the connection on
/// [`ServerTransportLayer::poll`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionId(usize);

impl ConnectionId {
    /// Slot index of this connection.
    pub const fn index(self) -> usize {
        self.0
    }
}

/// Wire-level request the caller must apply to a connection's socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WireAction {
    /// Transmit these bytes.
    Send(Frame),
    /// Transmit a queued diagnostic message (E29).  Resolve the token against
    /// the [`DiagnosticSender`](crate::diagnostic::DiagnosticSender) that
    /// queued it — [`wire_frame`](crate::diagnostic::DiagnosticSender::wire_frame)
    /// yields the header and payload bytes.  Upstream:
    /// `DoIpTransportMessageSendJob` queued via `IDoIpServerConnection::sendMessage`.
    SendDiagnostic(DiagnosticSendToken),
    /// Close the socket with FIN.
    Close,
    /// Reset the socket (RST).
    Abort,
}

/// Application notification emitted by the layer
/// (upstream `IDoIpServerTransportLayerCallback`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServerEvent {
    /// Routing became active for a connection
    /// (upstream `routingActive`).
    RoutingActive {
        /// Connection that activated.
        id: ConnectionId,
        /// Tester source address.
        source_address: u16,
        /// Internal source address used for routing lookups.
        internal_source_address: u16,
    },
    /// A connection that is or was routing closed
    /// (upstream `connectionClosed`).
    ConnectionClosed {
        /// Connection that closed.
        id: ConnectionId,
        /// Tester source address that was bound to the connection.
        source_address: u16,
    },
    /// No remaining connection routes for this internal source address
    /// (upstream `routingInactive`, emitted on release).
    RoutingInactive {
        /// Internal source address that became unrouted.
        internal_source_address: u16,
    },
}

/// Admission failure for a new TCP connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcceptError {
    /// All connection slots are occupied.
    NoFreeSlot,
    /// Admission control refused the connection (upstream
    /// `filterConnection`: connection limit or application filter).
    Refused,
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

/// Per-slot wire-action queue capacity.  Callers are expected to drain
/// actions after every layer call; overflowing actions are counted and
/// dropped (upstream applies TCP backpressure instead).
const WIRE_QUEUE_CAPACITY: usize = 32;

/// Application-event queue capacity.
const EVENT_QUEUE_CAPACITY: usize = 16;

#[derive(Debug)]
struct Slot {
    connection: ServerConnection,
    wire: RingQueue<WireAction, WIRE_QUEUE_CAPACITY>,
    marked_close: Option<CloseMode>,
    /// Removed from the active list (upstream `_connectionsToRelease`);
    /// the slot is freed on `poll` once its wire queue is drained.
    released: bool,
}

/// Alive-check bookkeeping for the (single) socket group.
///
/// Upstream: `DoIpServerTransportLayer::AliveCheckHelper` — a pending-check
/// refcount plus the negative response code used when every checked
/// connection proves alive.
#[derive(Debug)]
struct AliveCheckState {
    /// Slot of the activating connection awaiting the arbitration result;
    /// `None` once released (activation already completed or connection
    /// closed).
    activating: Option<usize>,
    negative_code: RoutingActivationCode,
    pending: u8,
}

// ---------------------------------------------------------------------------
// ServerTransportLayer
// ---------------------------------------------------------------------------

/// Fixed-capacity `DoIP` server transport layer for one socket group.
///
/// `N` is the number of connection slots.  Admission deliberately allows one
/// connection more than [`TransportParameters::max_connection_count`]
/// (upstream `filterConnection` uses `count <= max`) so the alive-check
/// arbitration can displace a stale connection; size `N` accordingly
/// (default 6 slots for the default limit of 5).
#[derive(Debug)]
pub struct ServerTransportLayer<const N: usize = 6> {
    version: ProtocolVersion,
    entity_address: u16,
    parameters: TransportParameters,
    slots: [Option<Slot>; N],
    alive_check: Option<AliveCheckState>,
    events: RingQueue<ServerEvent, EVENT_QUEUE_CAPACITY>,
    dropped_wire_actions: u32,
}

impl<const N: usize> ServerTransportLayer<N> {
    /// Create an empty layer.
    pub fn new(
        version: ProtocolVersion,
        entity_address: u16,
        parameters: TransportParameters,
    ) -> Self {
        Self {
            version,
            entity_address,
            parameters,
            slots: core::array::from_fn(|_| None),
            alive_check: None,
            events: RingQueue::new(),
            dropped_wire_actions: 0,
        }
    }

    // -- admission -----------------------------------------------------------

    /// Accept a new TCP connection (upstream `filterConnection` +
    /// `connectionAccepted`): starts the connection and arms its
    /// initial-inactivity timer.
    pub fn accept<P: ActivationPolicy>(
        &mut self,
        now: Instant,
        policy: &mut P,
    ) -> Result<ConnectionId, AcceptError> {
        self.admit(now, policy, None)
    }

    /// Accept a resumed connection (upstream `resume` + `start`): the
    /// connection goes straight to `Active` with `source_address` bound and
    /// re-checks routing activation with `is_resuming == true`.
    pub fn accept_resumed<P: ActivationPolicy>(
        &mut self,
        now: Instant,
        policy: &mut P,
        source_address: u16,
    ) -> Result<ConnectionId, AcceptError> {
        self.admit(now, policy, Some(source_address))
    }

    fn admit<P: ActivationPolicy>(
        &mut self,
        now: Instant,
        policy: &mut P,
        resume_source: Option<u16>,
    ) -> Result<ConnectionId, AcceptError> {
        let max = usize::from(self.parameters.max_connection_count);
        // Upstream filterConnection: max > 0 && count <= max && app filter.
        // `<=` deliberately admits one extra connection for arbitration.
        if max == 0 || self.connection_count() > max || !policy.filter_connection() {
            return Err(AcceptError::Refused);
        }
        let index = self
            .slots
            .iter()
            .position(Option::is_none)
            .ok_or(AcceptError::NoFreeSlot)?;
        let mut connection =
            ServerConnection::new(self.version, self.entity_address, self.parameters);
        if let Some(source) = resume_source {
            let _ = connection.resume(source);
        }
        connection.start(now, policy);
        self.slots[index] = Some(Slot {
            connection,
            wire: RingQueue::new(),
            marked_close: None,
            released: false,
        });
        self.process(now);
        Ok(ConnectionId(index))
    }

    // -- receive / time ------------------------------------------------------

    /// Feed received TCP bytes for one connection.  Returns `false` when the
    /// handle no longer refers to an active connection.
    pub fn handle_bytes<P: ActivationPolicy, H: MessageHandler>(
        &mut self,
        now: Instant,
        id: ConnectionId,
        bytes: &[u8],
        policy: &mut P,
        handler: &mut H,
    ) -> bool {
        let Some(slot) = self.active_slot_mut(id.0) else {
            return false;
        };
        slot.connection.handle_bytes(now, bytes, policy, handler);
        self.process(now);
        true
    }

    /// Drive deferred closes, per-connection timers, and connection release.
    pub fn poll(&mut self, now: Instant) {
        // Deferred closes (upstream markForClose + executeEventClose).
        for slot in self.slots.iter_mut().flatten() {
            if !slot.released {
                if let Some(mode) = slot.marked_close.take() {
                    slot.connection.set_close_mode(mode);
                    slot.connection.close();
                }
                slot.connection.poll(now);
            }
        }
        self.process(now);
        self.release_connections();
    }

    // -- teardown ------------------------------------------------------------

    /// Mark the routing connection with this internal source address for
    /// close; executed on the next [`poll`](Self::poll)
    /// (upstream `closeConnection`).
    pub fn close_connection(&mut self, internal_source_address: u16, mode: CloseMode) -> bool {
        let Some(id) =
            self.find_routing_connection_by_internal_source_address(internal_source_address)
        else {
            return false;
        };
        if let Some(slot) = self.active_slot_mut(id.0) {
            slot.marked_close = Some(mode);
            return true;
        }
        false
    }

    /// Mark every connection for close; executed on the next
    /// [`poll`](Self::poll) (upstream `closeAllConnections`).
    pub fn close_all(&mut self, mode: CloseMode) {
        for slot in self.slots.iter_mut().flatten() {
            if !slot.released {
                slot.marked_close = Some(mode);
            }
        }
    }

    /// Mark one connection for close by handle; executed on the next
    /// [`poll`](Self::poll).  E29 uses this to apply a diagnostic handler's
    /// close-after-send request (upstream `IDoIpServerConnection::close`,
    /// invoked by `DoIpServerTransportMessageHandler::releaseSendJobAndClose`).
    pub fn mark_close(&mut self, id: ConnectionId, mode: CloseMode) -> bool {
        let Some(slot) = self.active_slot_mut(id.0) else {
            return false;
        };
        slot.marked_close = Some(mode);
        true
    }

    // -- diagnostic transmit path (E29) --------------------------------------

    /// Queue an already-encoded control frame (e.g. a diagnostic
    /// acknowledgement produced by
    /// [`DiagnosticMessageHandler`](crate::diagnostic::DiagnosticMessageHandler))
    /// on a connection's wire queue.  Returns `false` when the connection is
    /// gone or the queue is full.
    pub fn queue_frame(&mut self, id: ConnectionId, frame: Frame) -> bool {
        self.queue_wire(id, WireAction::Send(frame))
    }

    /// Queue a diagnostic message transmit job on a connection's wire queue
    /// (upstream `IDoIpServerConnection::sendMessage` with a
    /// `DoIpTransportMessageSendJob`).  Returns `false` when the connection is
    /// gone or the queue is full.
    pub fn queue_diagnostic_send(&mut self, id: ConnectionId, token: DiagnosticSendToken) -> bool {
        self.queue_wire(id, WireAction::SendDiagnostic(token))
    }

    fn queue_wire(&mut self, id: ConnectionId, action: WireAction) -> bool {
        let Some(slot) = self.active_slot_mut(id.0) else {
            return false;
        };
        if slot.wire.push(action) {
            true
        } else {
            self.dropped_wire_actions = self.dropped_wire_actions.saturating_add(1);
            false
        }
    }

    // -- draining ------------------------------------------------------------

    /// Remove and return the next wire action of any connection, scanning
    /// slots in ascending order.
    pub fn take_action(&mut self) -> Option<(ConnectionId, WireAction)> {
        for (index, slot) in self.slots.iter_mut().enumerate() {
            if let Some(slot) = slot.as_mut() {
                if let Some(action) = slot.wire.pop() {
                    return Some((ConnectionId(index), action));
                }
            }
        }
        None
    }

    /// Remove and return the next wire action of one connection.
    pub fn take_action_for(&mut self, id: ConnectionId) -> Option<WireAction> {
        self.slots.get_mut(id.0)?.as_mut()?.wire.pop()
    }

    /// Remove and return the next application event.
    pub fn take_event(&mut self) -> Option<ServerEvent> {
        self.events.pop()
    }

    /// Number of wire actions dropped because a queue overflowed (callers
    /// should drain actions after every layer call; zero in normal use).
    pub const fn dropped_wire_actions(&self) -> u32 {
        self.dropped_wire_actions
    }

    // -- queries -------------------------------------------------------------

    /// Number of connections in the active list (any state except released).
    pub fn connection_count(&self) -> usize {
        self.slots
            .iter()
            .flatten()
            .filter(|slot| !slot.released)
            .count()
    }

    /// Number of routing (`Active`) connections
    /// (upstream `getConnectionCount(active)`).
    pub fn routing_count(&self) -> usize {
        self.slots
            .iter()
            .flatten()
            .filter(|slot| !slot.released && slot.connection.is_routing())
            .count()
    }

    /// State of a connection, if the slot is occupied.
    pub fn connection_state(&self, id: ConnectionId) -> Option<ConnectionState> {
        Some(self.slot(id.0)?.connection.state())
    }

    /// Source address bound to a connection.
    pub fn source_address(&self, id: ConnectionId) -> Option<u16> {
        self.slot(id.0)?.connection.source_address()
    }

    /// Send-path lookup: routing connection bound to this internal source
    /// address (upstream `findRoutingConnectionByInternalSourceAddress`).
    pub fn find_routing_connection_by_internal_source_address(
        &self,
        internal_source_address: u16,
    ) -> Option<ConnectionId> {
        self.slots.iter().enumerate().find_map(|(index, slot)| {
            let slot = slot.as_ref()?;
            if !slot.released
                && slot.connection.is_routing()
                && slot.connection.internal_source_address() == Some(internal_source_address)
            {
                Some(ConnectionId(index))
            } else {
                None
            }
        })
    }

    // -- internal helpers ----------------------------------------------------

    fn slot(&self, index: usize) -> Option<&Slot> {
        self.slots.get(index)?.as_ref()
    }

    fn active_slot_mut(&mut self, index: usize) -> Option<&mut Slot> {
        let slot = self.slots.get_mut(index)?.as_mut()?;
        if slot.released {
            None
        } else {
            Some(slot)
        }
    }

    fn connection_mut(&mut self, index: usize) -> Option<&mut ServerConnection> {
        Some(&mut self.slots.get_mut(index)?.as_mut()?.connection)
    }

    /// Drain all connection action queues, handling layer notifications and
    /// forwarding wire actions, until no connection has pending actions.
    fn process(&mut self, now: Instant) {
        loop {
            let mut worked = false;
            for index in 0..N {
                while let Some(action) = self
                    .slots
                    .get_mut(index)
                    .and_then(|slot| slot.as_mut())
                    .and_then(|slot| slot.connection.take_action())
                {
                    worked = true;
                    self.dispatch(now, index, action);
                }
            }
            if !worked {
                break;
            }
        }
    }

    fn dispatch(&mut self, now: Instant, index: usize, action: Action) {
        match action {
            Action::Send(frame) => self.push_wire(index, WireAction::Send(frame)),
            Action::Close => self.push_wire(index, WireAction::Close),
            Action::Abort => self.push_wire(index, WireAction::Abort),
            Action::ActivationRequested => self.start_arbitration(now),
            Action::AliveCheckResult { alive } => {
                // Upstream aliveCheckResponseReceived: settle the helper,
                // then look for the next activating connection.
                self.end_alive_check(now, alive);
                self.start_arbitration(now);
            }
            Action::RoutingActive => {
                if let Some(slot) = self.slot(index) {
                    let source = slot.connection.source_address().unwrap_or(0);
                    let internal = slot.connection.internal_source_address().unwrap_or(source);
                    let _ = self.events.push(ServerEvent::RoutingActive {
                        id: ConnectionId(index),
                        source_address: source,
                        internal_source_address: internal,
                    });
                }
            }
            Action::Closed => self.handle_connection_closed(index),
        }
    }

    fn push_wire(&mut self, index: usize, action: WireAction) {
        if let Some(slot) = self.slots.get_mut(index).and_then(|slot| slot.as_mut()) {
            if !slot.wire.push(action) {
                self.dropped_wire_actions = self.dropped_wire_actions.saturating_add(1);
            }
        }
    }

    /// Upstream `connectionClosed`: release the alive-check reference if the
    /// closing connection is the activating one, notify the application, and
    /// move the connection to the deferred-release list.
    fn handle_connection_closed(&mut self, index: usize) {
        if let Some(helper) = self.alive_check.as_mut() {
            if helper.activating == Some(index) {
                helper.activating = None;
            }
        }
        let Some(slot) = self.slots.get_mut(index).and_then(|slot| slot.as_mut()) else {
            return;
        };
        if slot.connection.is_or_was_routing() {
            let source = slot.connection.source_address().unwrap_or(0);
            let _ = self.events.push(ServerEvent::ConnectionClosed {
                id: ConnectionId(index),
                source_address: source,
            });
        }
        slot.released = true;
    }

    /// Upstream `startAliveCheck` (no-argument overload): arbitrate pending
    /// activations while no alive check is in flight.
    fn start_arbitration(&mut self, now: Instant) {
        while self.alive_check.is_none() {
            let Some(activating) = self.find_activating() else {
                return;
            };
            let source = self
                .slot(activating)
                .and_then(|slot| slot.connection.source_address());
            let same_source = source.and_then(|sa| self.find_routing_by_source(sa));
            if let Some(check) = same_source {
                // An active connection already uses this source address:
                // alive-check the old connection; if it answers, the new one
                // is rejected with SourceAlreadyRegistered.
                self.alive_check = Some(AliveCheckState {
                    activating: Some(activating),
                    negative_code: RoutingActivationCode::SourceAlreadyRegistered,
                    pending: 1,
                });
                self.add_alive_check(now, check);
                self.end_alive_check(now, true);
            } else if self.routing_count() < usize::from(self.parameters.max_connection_count) {
                if let Some(connection) = self.connection_mut(activating) {
                    connection.routing_activation_completed(
                        now,
                        true,
                        RoutingActivationCode::Success,
                    );
                }
            } else {
                // Connection limit reached: alive-check every routing
                // connection in the socket group; if all answer, the new
                // connection is rejected with NoFreeSocket.
                self.alive_check = Some(AliveCheckState {
                    activating: Some(activating),
                    negative_code: RoutingActivationCode::NoFreeSocket,
                    pending: 1,
                });
                for index in 0..N {
                    let is_routing = self
                        .slot(index)
                        .is_some_and(|slot| !slot.released && slot.connection.is_routing());
                    if is_routing {
                        self.add_alive_check(now, index);
                    }
                }
                self.end_alive_check(now, true);
            }
        }
    }

    fn add_alive_check(&mut self, now: Instant, index: usize) {
        if let Some(helper) = self.alive_check.as_mut() {
            helper.pending = helper.pending.saturating_add(1);
        }
        if let Some(connection) = self.connection_mut(index) {
            connection.start_alive_check(now);
        }
    }

    /// Upstream `endAliveCheck`: the first failed check releases and
    /// activates the pending connection; once every check has settled, a
    /// still-pending activation is completed negatively.
    fn end_alive_check(&mut self, now: Instant, alive: bool) {
        let Some(helper) = self.alive_check.as_mut() else {
            return;
        };
        let mut activate: Option<usize> = None;
        if !alive {
            activate = helper.activating.take();
        }
        helper.pending = helper.pending.saturating_sub(1);
        let mut negative: Option<(usize, RoutingActivationCode)> = None;
        if helper.pending == 0 {
            if let Some(helper) = self.alive_check.take() {
                if let Some(index) = helper.activating {
                    negative = Some((index, helper.negative_code));
                }
            }
        }
        if let Some(index) = activate {
            if let Some(connection) = self.connection_mut(index) {
                connection.routing_activation_completed(now, true, RoutingActivationCode::Success);
            }
        }
        if let Some((index, code)) = negative {
            if let Some(connection) = self.connection_mut(index) {
                connection.routing_activation_completed(now, false, code);
            }
        }
    }

    /// Upstream `findActivatingConnection` (single socket group).
    fn find_activating(&self) -> Option<usize> {
        (0..N).find(|&index| {
            self.slot(index)
                .is_some_and(|slot| !slot.released && slot.connection.is_activating())
        })
    }

    /// Upstream `findRoutingConnectionBySourceAddress`.
    fn find_routing_by_source(&self, source_address: u16) -> Option<usize> {
        (0..N).find(|&index| {
            self.slot(index).is_some_and(|slot| {
                !slot.released
                    && slot.connection.is_routing()
                    && slot.connection.source_address() == Some(source_address)
            })
        })
    }

    /// Upstream `releaseConnections`: free released slots whose wire queues
    /// have been drained, reporting `RoutingInactive` when the last routing
    /// connection for an internal source address disappears.
    fn release_connections(&mut self) {
        for index in 0..N {
            let ready = self
                .slot(index)
                .is_some_and(|slot| slot.released && slot.wire.is_empty());
            if !ready {
                continue;
            }
            let Some(slot) = self.slots[index].take() else {
                continue;
            };
            if slot.connection.is_or_was_routing() {
                let internal = slot
                    .connection
                    .internal_source_address()
                    .or(slot.connection.source_address())
                    .unwrap_or(0);
                if self
                    .find_routing_connection_by_internal_source_address(internal)
                    .is_none()
                {
                    let _ = self.events.push(ServerEvent::RoutingInactive {
                        internal_source_address: internal,
                    });
                }
            }
        }
    }
}
