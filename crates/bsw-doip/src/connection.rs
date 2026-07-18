//! `DoIP` TCP server connection — byte framing plus routing-activation state
//! machine (E28).
//!
//! Sans-IO port of the upstream server connection handling:
//!
//! - Upstream: `libs/bsw/doip/src/doip/server/DoIpServerConnectionHandler.cpp`
//!   (state machine, routing activation, alive check, timers).
//! - Upstream: `libs/bsw/doip/src/doip/common/DoIpTcpConnection.cpp`
//!   (HEADER → PAYLOAD → DISCARD receive framing).
//! - Upstream: `libs/bsw/doip/include/doip/common/DoIpHeader.h`
//!   (`checkProtocolVersion`).
//!
//! The connection performs no I/O and owns no clock.  Callers feed received
//! byte chunks via [`ServerConnection::handle_bytes`], drive time via
//! [`ServerConnection::poll`] with an injected [`Instant`], and drain emitted
//! [`Action`]s (frames to send, close/abort requests, and layer
//! notifications).  All storage is fixed-capacity; nothing allocates.
//!
//! Payloads consumed by the connection itself (routing activation request,
//! alive check response) are at most 11 bytes and are assembled in an internal
//! buffer mirroring the upstream `_readBuffer[11]`.  Payloads of any other
//! type are offered to a [`MessageHandler`] (the E29 diagnostic-message hook);
//! unhandled payloads are answered with a generic NACK and consumed by the
//! DISCARD state, so oversized non-diagnostic payloads never occupy memory.

use bsw_time::{Duration, Instant};

use crate::{
    NackCode, Packet, Payload, PayloadType, ProtocolVersion, RoutingActivationCode,
    RoutingActivationRequest, RoutingActivationResponse, TransportParameters, HEADER_SIZE,
};

// ---------------------------------------------------------------------------
// Fixed-capacity ring queue (shared with the server transport layer)
// ---------------------------------------------------------------------------

/// Fixed-capacity FIFO used for pending actions.  Allocation-free.
#[derive(Debug)]
pub(crate) struct RingQueue<T: Copy, const CAP: usize> {
    items: [Option<T>; CAP],
    head: usize,
    len: usize,
}

impl<T: Copy, const CAP: usize> RingQueue<T, CAP> {
    /// Create an empty queue.
    pub(crate) const fn new() -> Self {
        Self {
            items: [None; CAP],
            head: 0,
            len: 0,
        }
    }

    /// Append an item.  Returns `false` (dropping the item) when full.
    pub(crate) fn push(&mut self, item: T) -> bool {
        if self.len == CAP {
            return false;
        }
        self.items[(self.head + self.len) % CAP] = Some(item);
        self.len += 1;
        true
    }

    /// Remove and return the oldest item.
    pub(crate) fn pop(&mut self) -> Option<T> {
        let item = self.items[self.head].take()?;
        self.head = (self.head + 1) % CAP;
        self.len -= 1;
        Some(item)
    }

    /// Number of free slots.
    pub(crate) const fn free(&self) -> usize {
        CAP - self.len
    }

    /// Mutable access to the most recently pushed item (E29 uses this to
    /// rewrite a queued positive diagnostic ACK into a negative ACK, mirroring
    /// upstream `job->setPayloadType(...)` in
    /// `libs/bsw/doip/src/doip/server/DoIpServerTransportMessageHandler.cpp`).
    pub(crate) fn back_mut(&mut self) -> Option<&mut T> {
        if self.len == 0 {
            return None;
        }
        self.items[(self.head + self.len - 1) % CAP].as_mut()
    }

    /// Whether the queue holds no items.
    pub(crate) const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<T: Copy, const CAP: usize> Default for RingQueue<T, CAP> {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Frames and actions
// ---------------------------------------------------------------------------

/// Largest control frame emitted by the connection or the diagnostic message
/// handler: 8-byte header plus the 10-byte diagnostic acknowledgement payload
/// (4 address bytes, 1 code byte, up to 5 echoed previous-message bytes —
/// upstream `STATIC_PAYLOAD_SENDJOB_SIZE` in
/// `libs/bsw/doip/include/doip/server/DoIpServerTransportMessageHandler.h`).
/// The 17-byte routing activation response also fits.
pub const CONTROL_FRAME_MAX: usize = 18;

/// One encoded `DoIP` control frame to transmit on the TCP socket.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    bytes: [u8; CONTROL_FRAME_MAX],
    len: u8,
}

impl Frame {
    /// Encoded frame bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }

    /// Encode a packet into a control frame.  Returns `None` when the encoded
    /// packet does not fit [`CONTROL_FRAME_MAX`].
    pub(crate) fn from_packet(packet: &Packet<'_>) -> Option<Self> {
        let mut frame = Self {
            bytes: [0; CONTROL_FRAME_MAX],
            len: 0,
        };
        let len = packet.encode(&mut frame.bytes).ok()?;
        frame.len = len as u8;
        Some(frame)
    }
}

/// How the TCP connection should be terminated.
///
/// Upstream: `IDoIpTcpConnection::CloseMode` — `CLOSE` performs an orderly
/// FIN close, `ABORT` resets the connection (RST).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CloseMode {
    /// Orderly close (FIN).
    Close,
    /// Abortive close (RST).
    Abort,
}

/// Action emitted by a [`ServerConnection`].
///
/// `Send`/`Close`/`Abort` are wire-level requests for the caller's socket.
/// The remaining variants are notifications the transport layer consumes
/// (upstream `IDoIpServerConnectionHandlerCallback`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Transmit these bytes on the socket.
    Send(Frame),
    /// Close the socket with FIN.
    Close,
    /// Reset the socket (RST).
    Abort,
    /// A routing activation request moved the connection to `Activating`;
    /// the transport layer must decide completion
    /// (upstream `handleRoutingActivationRequest`).
    ActivationRequested,
    /// Alive-check outcome (upstream `aliveCheckResponseReceived` callback):
    /// `alive == true` when the expected source address answered, `false`
    /// when the check failed (closed connection or non-active start).
    AliveCheckResult {
        /// Whether the connection proved alive.
        alive: bool,
    },
    /// Routing became active (upstream `routingActive` callback).
    RoutingActive,
    /// The connection reached `Shutdown`
    /// (upstream `connectionClosed` callback).
    Closed,
}

// ---------------------------------------------------------------------------
// Activation policy
// ---------------------------------------------------------------------------

/// Action requested by an [`ActivationPolicy`] check.
///
/// Upstream: `IDoIpServerConnectionFilter::Action`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyAction {
    /// Send the response code and close the connection.
    Reject,
    /// Send the response code but keep the connection open.
    Keep,
    /// Continue with regular routing activation handling.
    Continue,
}

/// Result of a routing-activation policy check.
///
/// Upstream: `IDoIpServerConnectionFilter::RoutingActivationCheckResult`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActivationDecision {
    /// Action to perform.
    pub action: PolicyAction,
    /// Response code sent with the routing activation response.
    pub response_code: RoutingActivationCode,
    /// Optional internal source address used for routing lookups; `None`
    /// resolves to the tester source address
    /// (upstream `resolveInternalSourceAddress`).
    pub internal_source_address: Option<u16>,
}

impl ActivationDecision {
    /// Continue with regular handling (upstream default-constructed result).
    pub const fn proceed() -> Self {
        Self {
            action: PolicyAction::Continue,
            response_code: RoutingActivationCode::Success,
            internal_source_address: None,
        }
    }

    /// Send `response_code` and keep the connection open.
    pub const fn keep(response_code: RoutingActivationCode) -> Self {
        Self {
            action: PolicyAction::Keep,
            response_code,
            internal_source_address: None,
        }
    }

    /// Send `response_code` and close the connection.
    pub const fn reject(response_code: RoutingActivationCode) -> Self {
        Self {
            action: PolicyAction::Reject,
            response_code,
            internal_source_address: None,
        }
    }
}

/// Project-supplied routing-activation policy (e.g. tester-address
/// validation returning [`RoutingActivationCode::UnknownSourceAddress`]).
///
/// Upstream: `IDoIpServerTransportLayerCallback::checkRoutingActivation` and
/// `IDoIpServerTransportLayerCallback::filterConnection`.
pub trait ActivationPolicy {
    /// Check a routing activation request.  `is_resuming` is `true` when a
    /// resumed connection re-checks its stored source address on start.
    fn check_routing_activation(
        &mut self,
        source_address: u16,
        activation_type: u8,
        is_resuming: bool,
    ) -> ActivationDecision;

    /// Application-level admission filter for new TCP connections.
    fn filter_connection(&mut self) -> bool {
        true
    }
}

/// Default policy: activation types above 0x01 are rejected with
/// [`RoutingActivationCode::UnsupportedActivationType`], everything else
/// continues.
///
/// Upstream: `libs/bsw/doip/src/doip/server/DoIpServerTransportLayer.cpp`
/// (`checkRoutingActivation` fallback, line 271) and
/// `executables/referenceApp/application/src/systems/DoIpServerSystem.cpp`
/// (line 149).
#[derive(Debug, Default, Clone, Copy)]
pub struct DefaultActivationPolicy;

impl ActivationPolicy for DefaultActivationPolicy {
    fn check_routing_activation(
        &mut self,
        _source_address: u16,
        activation_type: u8,
        _is_resuming: bool,
    ) -> ActivationDecision {
        if activation_type > 0x01 {
            ActivationDecision::reject(RoutingActivationCode::UnsupportedActivationType)
        } else {
            ActivationDecision::proceed()
        }
    }
}

// ---------------------------------------------------------------------------
// Message handler hook (E29 dispatch point)
// ---------------------------------------------------------------------------

/// Hook offered every payload type the connection does not consume itself
/// (everything except 0x0005, 0x0008 and 0x0000).
///
/// Upstream: `IDoIpServerMessageHandler::headerReceived` — the dispatch point
/// where E29 attaches diagnostic-message handling (0x8001/0x8002/0x8003).
pub trait MessageHandler {
    /// Offer a received header.  Return `true` to take over the payload; the
    /// connection then forwards payload bytes to
    /// [`payload_chunk`](Self::payload_chunk).  Return `false` to have the
    /// connection answer with a generic NACK 0x01 and discard the payload.
    fn header_received(&mut self, payload_type: u16, payload_length: u32) -> bool;

    /// Payload bytes for an accepted header, delivered in received order and
    /// possibly fragmented.
    fn payload_chunk(&mut self, bytes: &[u8]);
}

/// Message handler that accepts nothing; every non-connection payload type is
/// answered with a generic NACK 0x01.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoMessageHandler;

impl MessageHandler for NoMessageHandler {
    fn header_received(&mut self, _payload_type: u16, _payload_length: u32) -> bool {
        false
    }

    fn payload_chunk(&mut self, _bytes: &[u8]) {}
}

// ---------------------------------------------------------------------------
// Connection state machine
// ---------------------------------------------------------------------------

/// Connection lifecycle state.
///
/// Upstream: `DoIpServerConnectionHandler::State`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Started, waiting for a routing activation request.
    Inactive,
    /// Routing activation received, transport layer decides completion.
    Activating,
    /// Routing is active.
    Active,
    /// A close-after-send response is in flight.
    Closing,
    /// Connection is closed.
    Shutdown,
}

/// Internal read-framing state.
///
/// Upstream: `DoIpTcpConnection::ReadState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RxState {
    /// Assembling the 8-byte generic header.
    Header,
    /// Assembling a connection-consumed payload.
    Payload(PendingPayload),
    /// Consuming bytes of a discarded payload.
    Discard,
    /// Forwarding payload bytes to the message handler (E29 hook).
    HandlerPayload,
}

/// Payload the connection is currently assembling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PendingPayload {
    RoutingActivation,
    AliveCheck,
}

/// Internal read buffer size; the largest connection-consumed payload is the
/// 11-byte routing activation request (upstream `_readBuffer[11]`).
const READ_BUFFER_SIZE: usize = 11;

/// Action-queue capacity.  Mirrors the bounded upstream send-job pool: when
/// the queue cannot accept another frame the connection closes.
const ACTION_QUEUE_CAPACITY: usize = 16;

/// Slots kept free so a close sequence (`Close`/`Abort` + `Closed` +
/// `AliveCheckResult`) always fits.
const CLOSE_RESERVE: usize = 3;

const TYPE_NEGATIVE_ACK: u16 = PayloadType::NegativeAck.as_u16();
const TYPE_ROUTING_ACTIVATION_REQUEST: u16 = PayloadType::RoutingActivationRequest.as_u16();
const TYPE_ALIVE_CHECK_RESPONSE: u16 = PayloadType::AliveCheckResponse.as_u16();

/// Convert millisecond parameters to a [`Duration`] (cannot overflow for
/// `u32` inputs).
fn millis(value: u32) -> Duration {
    Duration::from_millis(u64::from(value)).unwrap_or(Duration::ZERO)
}

/// Sans-IO `DoIP` TCP server connection.
///
/// Upstream: `libs/bsw/doip/src/doip/server/DoIpServerConnectionHandler.cpp`.
#[derive(Debug)]
pub struct ServerConnection {
    version: ProtocolVersion,
    entity_address: u16,
    parameters: TransportParameters,
    state: ConnectionState,
    started: bool,
    close_mode: CloseMode,
    source_address: Option<u16>,
    internal_source_address: Option<u16>,
    /// Single multiplexed deadline (upstream `_timerTimeout`).
    deadline: Option<Instant>,
    alive_check_pending: bool,
    is_or_was_routing: bool,
    rx: RxState,
    header_buf: [u8; HEADER_SIZE],
    header_len: usize,
    payload_buf: [u8; READ_BUFFER_SIZE],
    payload_len: usize,
    payload_needed: usize,
    /// Remaining bytes in `Discard` / `HandlerPayload` states.
    remaining: u32,
    actions: RingQueue<Action, ACTION_QUEUE_CAPACITY>,
}

impl ServerConnection {
    /// Create an unstarted connection in `Inactive` state.
    pub const fn new(
        version: ProtocolVersion,
        entity_address: u16,
        parameters: TransportParameters,
    ) -> Self {
        Self {
            version,
            entity_address,
            parameters,
            state: ConnectionState::Inactive,
            started: false,
            close_mode: CloseMode::Close,
            source_address: None,
            internal_source_address: None,
            deadline: None,
            alive_check_pending: false,
            is_or_was_routing: false,
            rx: RxState::Header,
            header_buf: [0; HEADER_SIZE],
            header_len: 0,
            payload_buf: [0; READ_BUFFER_SIZE],
            payload_len: 0,
            payload_needed: 0,
            remaining: 0,
            actions: RingQueue::new(),
        }
    }

    // -- accessors ----------------------------------------------------------

    /// Current lifecycle state.
    pub const fn state(&self) -> ConnectionState {
        self.state
    }

    /// Whether the connection is in `Activating` state
    /// (upstream `isActivating`).
    pub fn is_activating(&self) -> bool {
        self.state == ConnectionState::Activating
    }

    /// Whether routing is active (upstream `isRouting`).
    pub fn is_routing(&self) -> bool {
        self.state == ConnectionState::Active
    }

    /// Sticky flag: whether routing is or ever was active
    /// (upstream `isOrWasRouting`).
    pub const fn is_or_was_routing(&self) -> bool {
        self.is_or_was_routing
    }

    /// Whether the connection reached `Shutdown` (upstream `isClosed`).
    pub fn is_closed(&self) -> bool {
        self.state == ConnectionState::Shutdown
    }

    /// Whether an alive check is awaiting its response.
    pub const fn alive_check_pending(&self) -> bool {
        self.alive_check_pending
    }

    /// Tester source address bound by routing activation.
    pub const fn source_address(&self) -> Option<u16> {
        self.source_address
    }

    /// Internal source address used for routing lookups.
    pub const fn internal_source_address(&self) -> Option<u16> {
        self.internal_source_address
    }

    /// Deadline of the armed timer, if any (for caller wait scheduling).
    pub const fn next_deadline(&self) -> Option<Instant> {
        self.deadline
    }

    /// Remove and return the oldest pending action.
    pub fn take_action(&mut self) -> Option<Action> {
        self.actions.pop()
    }

    /// Whether no actions are pending.
    pub fn actions_empty(&self) -> bool {
        self.actions.is_empty()
    }

    // -- lifecycle ----------------------------------------------------------

    /// Resume an established routing association.  Only allowed before
    /// [`start`](Self::start); the connection goes straight to `Active` with
    /// the sticky routing flag set (upstream `resume`).
    pub fn resume(&mut self, source_address: u16) -> bool {
        if self.started {
            return false;
        }
        self.source_address = Some(source_address);
        self.state = ConnectionState::Active;
        self.is_or_was_routing = true;
        true
    }

    /// Start the connection (upstream `start`).
    ///
    /// A fresh connection arms the initial-inactivity timer.  A resumed
    /// connection re-checks routing activation with `is_resuming == true`
    /// (only the internal source address of the decision is used, mirroring
    /// upstream), arms the general-inactivity timer, and reports routing
    /// active.
    pub fn start<P: ActivationPolicy>(&mut self, now: Instant, policy: &mut P) {
        if self.started {
            return;
        }
        self.started = true;
        if self.state == ConnectionState::Inactive {
            self.arm(now, self.parameters.inactivity_timeout_ms);
        } else {
            let source = self.source_address.unwrap_or(0);
            let decision = policy.check_routing_activation(source, 0, true);
            self.internal_source_address = Some(decision.internal_source_address.unwrap_or(source));
            self.arm(now, self.parameters.general_inactivity_timeout_ms);
            self.push_action(Action::RoutingActive);
        }
    }

    /// Select the close mode used by subsequent closes
    /// (upstream `setCloseMode` / `markForClose`).
    pub fn set_close_mode(&mut self, mode: CloseMode) {
        self.close_mode = mode;
    }

    /// Close the connection (upstream `close`).
    ///
    /// Emits the wire close, then `Closed`, then — if an alive check was
    /// pending — `AliveCheckResult { alive: false }`.  Upstream comment: the
    /// alive-check notification must follow the connection-closed handling so
    /// a new connection reusing the source address activates correctly.
    pub fn close(&mut self) {
        if self.state == ConnectionState::Shutdown {
            return;
        }
        self.state = ConnectionState::Shutdown;
        self.deadline = None;
        self.push_action(match self.close_mode {
            CloseMode::Close => Action::Close,
            CloseMode::Abort => Action::Abort,
        });
        self.push_action(Action::Closed);
        if self.alive_check_pending {
            self.alive_check_pending = false;
            self.push_action(Action::AliveCheckResult { alive: false });
        }
    }

    /// Report the transport layer's routing activation decision
    /// (upstream `routingActivationCompleted`).  Only valid in `Activating`.
    pub fn routing_activation_completed(
        &mut self,
        now: Instant,
        success: bool,
        response_code: RoutingActivationCode,
    ) {
        if self.state != ConnectionState::Activating {
            return;
        }
        let source = self.source_address.unwrap_or(0);
        self.send_routing_activation_response(source, response_code, !success);
        if success {
            self.state = ConnectionState::Active;
            self.is_or_was_routing = true;
            self.arm(now, self.parameters.general_inactivity_timeout_ms);
            self.push_action(Action::RoutingActive);
        }
    }

    /// Start an alive check (upstream `startAliveCheck`).
    ///
    /// Only meaningful in `Active`: marks the check pending, emits the
    /// alive-check request and arms the alive-check timer.  In any other
    /// started state the check fails immediately with
    /// `AliveCheckResult { alive: false }`.
    pub fn start_alive_check(&mut self, now: Instant) {
        if self.state == ConnectionState::Active {
            self.alive_check_pending = true;
            let frame = self.encode_frame(Payload::AliveCheckRequest);
            if self.push_send(frame) {
                self.arm(now, self.parameters.alive_check_timeout_ms);
            }
        } else if self.started {
            self.push_action(Action::AliveCheckResult { alive: false });
        }
    }

    // -- time ----------------------------------------------------------------

    /// Apply timer expiry (upstream `timerExpired`), driven by the injected
    /// clock.  Deadlines are inclusive: `now == deadline` expires.
    ///
    /// - `Inactive`: no routing request in time — close (FIN).
    /// - `Active` with alive check pending: no alive response — close (FIN).
    /// - `Active` otherwise: general inactivity — abort (RST).
    /// - Other states ignore expiry (upstream `default` branch).
    pub fn poll(&mut self, now: Instant) {
        let Some(deadline) = self.deadline else {
            return;
        };
        if !now.is_at_or_after(deadline) {
            return;
        }
        self.deadline = None;
        match self.state {
            ConnectionState::Inactive => self.close(),
            ConnectionState::Active => {
                if !self.alive_check_pending {
                    self.close_mode = CloseMode::Abort;
                }
                self.close();
            }
            ConnectionState::Activating | ConnectionState::Closing | ConnectionState::Shutdown => {}
        }
    }

    // -- receive path --------------------------------------------------------

    /// Feed a chunk of received TCP bytes (upstream `dataReceived` +
    /// `processNextReadChunk`).  Bytes arriving after shutdown, or before
    /// [`start`](Self::start), are ignored.
    pub fn handle_bytes<P: ActivationPolicy, H: MessageHandler>(
        &mut self,
        now: Instant,
        mut bytes: &[u8],
        policy: &mut P,
        handler: &mut H,
    ) {
        if !self.started {
            return;
        }
        while !bytes.is_empty() && self.state != ConnectionState::Shutdown {
            bytes = match self.rx {
                RxState::Header => self.rx_header(now, bytes, handler),
                RxState::Payload(kind) => self.rx_payload(bytes, kind, policy),
                RxState::Discard => self.rx_discard(bytes),
                RxState::HandlerPayload => self.rx_handler_payload(bytes, handler),
            };
        }
    }

    fn rx_header<'a, H: MessageHandler>(
        &mut self,
        now: Instant,
        bytes: &'a [u8],
        handler: &mut H,
    ) -> &'a [u8] {
        let take = (HEADER_SIZE - self.header_len).min(bytes.len());
        self.header_buf[self.header_len..self.header_len + take].copy_from_slice(&bytes[..take]);
        self.header_len += take;
        if self.header_len == HEADER_SIZE {
            self.header_len = 0;
            self.process_header(now, handler);
        }
        &bytes[take..]
    }

    fn rx_payload<'a, P: ActivationPolicy>(
        &mut self,
        bytes: &'a [u8],
        kind: PendingPayload,
        policy: &mut P,
    ) -> &'a [u8] {
        let take = (self.payload_needed - self.payload_len).min(bytes.len());
        self.payload_buf[self.payload_len..self.payload_len + take].copy_from_slice(&bytes[..take]);
        self.payload_len += take;
        if self.payload_len == self.payload_needed {
            self.rx = RxState::Header;
            match kind {
                PendingPayload::RoutingActivation => {
                    self.routing_activation_request_received(policy);
                }
                PendingPayload::AliveCheck => self.alive_check_response_received(),
            }
        }
        &bytes[take..]
    }

    fn rx_discard<'a>(&mut self, bytes: &'a [u8]) -> &'a [u8] {
        let take = bytes.len().min(self.remaining as usize);
        self.remaining -= take as u32;
        if self.remaining == 0 {
            self.rx = RxState::Header;
        }
        &bytes[take..]
    }

    fn rx_handler_payload<'a, H: MessageHandler>(
        &mut self,
        bytes: &'a [u8],
        handler: &mut H,
    ) -> &'a [u8] {
        let take = bytes.len().min(self.remaining as usize);
        handler.payload_chunk(&bytes[..take]);
        self.remaining -= take as u32;
        if self.remaining == 0 {
            self.rx = RxState::Header;
        }
        &bytes[take..]
    }

    /// Dispatch a completed 8-byte header (upstream `headerReceived`).
    fn process_header<H: MessageHandler>(&mut self, now: Instant, handler: &mut H) {
        let buf = self.header_buf;
        // Upstream DoIpHeader.h checkProtocolVersion: inverse byte must match
        // and the version must equal the configured protocol version.
        if buf[1] != !buf[0] || buf[0] != self.version.as_byte() {
            self.send_nack(NackCode::IncorrectPattern, true);
            return;
        }
        let payload_type = u16::from_be_bytes([buf[2], buf[3]]);
        let payload_length = u32::from_be_bytes([buf[4], buf[5], buf[6], buf[7]]);
        // Any received header re-arms the general-inactivity timer while
        // routing is active.
        if self.state == ConnectionState::Active {
            self.arm(now, self.parameters.general_inactivity_timeout_ms);
        }
        match payload_type {
            TYPE_ROUTING_ACTIVATION_REQUEST => {
                const RA_MIN: u32 = RoutingActivationRequest::MIN_PAYLOAD_SIZE as u32;
                const RA_MAX: u32 = RoutingActivationRequest::MAX_PAYLOAD_SIZE as u32;
                if payload_length == RA_MIN || payload_length == RA_MAX {
                    self.begin_payload(PendingPayload::RoutingActivation, payload_length);
                } else {
                    self.send_nack(NackCode::InvalidPayloadLength, true);
                }
            }
            TYPE_ALIVE_CHECK_RESPONSE => {
                if payload_length == 2 {
                    self.begin_payload(PendingPayload::AliveCheck, payload_length);
                } else {
                    self.send_nack(NackCode::InvalidPayloadLength, true);
                }
            }
            // Generic negative acknowledges are consumed without processing
            // (upstream headerReceivedNegativeAck).
            TYPE_NEGATIVE_ACK => self.begin_discard(payload_length),
            _ => {
                // Offer to the message handler hook (E29 dispatch point,
                // upstream headerReceivedDefault).
                if handler.header_received(payload_type, payload_length) {
                    if payload_length == 0 {
                        self.rx = RxState::Header;
                    } else {
                        self.rx = RxState::HandlerPayload;
                        self.remaining = payload_length;
                    }
                } else {
                    self.send_nack(NackCode::UnknownPayloadType, false);
                    self.begin_discard(payload_length);
                }
            }
        }
    }

    fn begin_payload(&mut self, kind: PendingPayload, payload_length: u32) {
        self.rx = RxState::Payload(kind);
        self.payload_needed = payload_length as usize;
        self.payload_len = 0;
    }

    fn begin_discard(&mut self, payload_length: u32) {
        if payload_length == 0 {
            self.rx = RxState::Header;
        } else {
            self.rx = RxState::Discard;
            self.remaining = payload_length;
        }
    }

    /// Handle a complete routing activation request payload
    /// (upstream `routingActivationRequestReceived`).
    fn routing_activation_request_received<P: ActivationPolicy>(&mut self, policy: &mut P) {
        let Ok(request) = RoutingActivationRequest::parse(&self.payload_buf[..self.payload_needed])
        else {
            // Unreachable: length was validated at header time.  Defensive
            // NACK instead of a panic.
            self.send_nack(NackCode::InvalidPayloadLength, true);
            return;
        };
        let source = request.source_address;
        let decision = policy.check_routing_activation(source, request.activation_type, false);
        match decision.action {
            PolicyAction::Reject => {
                self.send_routing_activation_response(source, decision.response_code, true);
            }
            PolicyAction::Keep => {
                self.send_routing_activation_response(source, decision.response_code, false);
            }
            PolicyAction::Continue => {
                if self.state == ConnectionState::Inactive {
                    self.source_address = Some(source);
                    self.internal_source_address =
                        Some(decision.internal_source_address.unwrap_or(source));
                    self.state = ConnectionState::Activating;
                    self.push_action(Action::ActivationRequested);
                } else if self.source_address == Some(source) {
                    self.send_routing_activation_response(
                        source,
                        RoutingActivationCode::Success,
                        false,
                    );
                } else {
                    self.send_routing_activation_response(
                        source,
                        RoutingActivationCode::WrongSourceAddress,
                        true,
                    );
                }
            }
        }
    }

    /// Handle a complete alive check response payload
    /// (upstream `aliveCheckResponseReceived`).
    fn alive_check_response_received(&mut self) {
        let source = u16::from_be_bytes([self.payload_buf[0], self.payload_buf[1]]);
        let expected = self.source_address == Some(source);
        if self.alive_check_pending {
            self.alive_check_pending = false;
            self.push_action(Action::AliveCheckResult { alive: expected });
        }
        if !expected {
            self.close();
        }
    }

    // -- transmit path -------------------------------------------------------

    fn send_nack(&mut self, code: NackCode, close_after_send: bool) {
        let frame = self.encode_frame(Payload::GenericNack(code));
        self.transmit(frame, close_after_send);
    }

    fn send_routing_activation_response(
        &mut self,
        tester_address: u16,
        response_code: RoutingActivationCode,
        close_after_send: bool,
    ) {
        let frame = self.encode_frame(Payload::RoutingActivationResponse(
            RoutingActivationResponse {
                tester_address,
                entity_address: self.entity_address,
                response_code,
                reserved: [0; 4],
                oem_specific: None,
            },
        ));
        self.transmit(frame, close_after_send);
    }

    fn encode_frame(&self, payload: Payload<'_>) -> Frame {
        let mut frame = Frame {
            bytes: [0; CONTROL_FRAME_MAX],
            len: 0,
        };
        let packet = Packet {
            version: self.version,
            payload,
        };
        // Cannot fail: every control payload fits CONTROL_FRAME_MAX.
        let len = packet.encode(&mut frame.bytes).unwrap_or(0);
        frame.len = len as u8;
        frame
    }

    /// Queue a frame; with `close_after_send` the connection enters `Closing`
    /// and closes once the frame is queued (upstream `allocateSendJob` with
    /// close callback + `releaseSendJobAndClose`).
    fn transmit(&mut self, frame: Frame, close_after_send: bool) {
        if self.state == ConnectionState::Shutdown {
            return;
        }
        if close_after_send {
            self.state = ConnectionState::Closing;
        }
        if !self.push_send(frame) {
            return;
        }
        if close_after_send {
            self.close();
        }
    }

    /// Queue a send.  When the reserve for the close sequence would be
    /// violated the connection closes instead — mirroring the upstream
    /// behaviour when the bounded send-job pool is exhausted.
    fn push_send(&mut self, frame: Frame) -> bool {
        if self.actions.free() <= CLOSE_RESERVE {
            self.close();
            return false;
        }
        let _ = self.actions.push(Action::Send(frame));
        true
    }

    fn push_action(&mut self, action: Action) {
        let _ = self.actions.push(action);
    }

    fn arm(&mut self, now: Instant, timeout_ms: u32) {
        self.deadline = Some(now.wrapping_add(millis(timeout_ms)));
    }
}
