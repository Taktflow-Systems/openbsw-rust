//! Diagnostic-message transport bridging for DoIP payloads 0x8001-0x8003.
//!
//! The receive path is allocation-free and uses the generation-checked
//! [`MessagePool`] from `bsw-transport`. The transmit path owns a bounded set
//! of send jobs whose tokens remain valid until the socket reports completion.

use bsw_transport::pool::{
    AdmissionPolicy, AllocationRequest, MessageHandle, MessagePool, ProviderError,
};
use bsw_transport::{TransportMessage, TransportResult};

use crate::connection::{CloseMode, Frame, MessageHandler, RingQueue};
use crate::payload::{DiagnosticAck, Packet, Payload};
use crate::server::{ConnectionId, ServerTransportLayer};
use crate::{DiagNackCode, DoIpHeader, NackCode, PayloadType, ProtocolVersion};

const ADDRESS_SIZE: usize = 4;
const ACK_MIN_SIZE: usize = 5;
const ACK_ECHO_SIZE: usize = 4;
const HANDLER_ACTION_CAPACITY: usize = 8;
const HANDLER_EVENT_CAPACITY: usize = 8;

/// Consumer of a fully assembled diagnostic transport message.
pub trait DiagnosticMessageListener<const PAYLOAD: usize> {
    /// Process one message synchronously. A non-`Ok` result is translated to
    /// diagnostic NACK 0x08 and the pool buffer is still released.
    fn message_received(&mut self, message: &TransportMessage<PAYLOAD>) -> TransportResult;
}

impl<const PAYLOAD: usize, F> DiagnosticMessageListener<PAYLOAD> for F
where
    F: FnMut(&TransportMessage<PAYLOAD>) -> TransportResult,
{
    fn message_received(&mut self, message: &TransportMessage<PAYLOAD>) -> TransportResult {
        self(message)
    }
}

/// Incremental receive boundary used by [`DiagnosticMessageHandler`].
pub trait DiagnosticGateway {
    /// Generation-checked ownership token.
    type Handle: Copy;

    /// Allocate a message for the application bytes after the four DoIP
    /// address bytes.
    fn allocate(
        &mut self,
        source_bus: u8,
        source: u16,
        target: u16,
        size: usize,
    ) -> Result<Self::Handle, ProviderError>;

    /// Append a received payload fragment.
    fn append(&mut self, handle: Self::Handle, bytes: &[u8]) -> Result<(), ProviderError>;

    /// Notify the listener and release the message regardless of its result.
    fn finish(&mut self, handle: Self::Handle) -> Result<TransportResult, ProviderError>;

    /// Release an incomplete message after an input error.
    fn cancel(&mut self, handle: Self::Handle);
}

/// Diagnostic gateway backed by a fixed `bsw-transport` message pool.
pub struct PoolDiagnosticGateway<const COUNT: usize, const PAYLOAD: usize, P, L> {
    pool: MessagePool<COUNT, PAYLOAD, P>,
    listener: L,
}

impl<const COUNT: usize, const PAYLOAD: usize, P, L> PoolDiagnosticGateway<COUNT, PAYLOAD, P, L>
where
    P: AdmissionPolicy,
    L: DiagnosticMessageListener<PAYLOAD>,
{
    /// Construct a gateway with an address/bus admission policy and listener.
    pub fn new(policy: P, listener: L) -> Self {
        Self {
            pool: MessagePool::new(policy),
            listener,
        }
    }

    /// Number of receive buffers currently checked out.
    pub fn in_use(&self) -> usize {
        self.pool.in_use()
    }

    /// Shared access to the listener for deterministic assertions.
    pub const fn listener(&self) -> &L {
        &self.listener
    }

    /// Mutable access to the listener.
    pub fn listener_mut(&mut self) -> &mut L {
        &mut self.listener
    }
}

impl<const COUNT: usize, const PAYLOAD: usize, P, L> DiagnosticGateway
    for PoolDiagnosticGateway<COUNT, PAYLOAD, P, L>
where
    P: AdmissionPolicy,
    L: DiagnosticMessageListener<PAYLOAD>,
{
    type Handle = MessageHandle;

    fn allocate(
        &mut self,
        source_bus: u8,
        source: u16,
        target: u16,
        size: usize,
    ) -> Result<Self::Handle, ProviderError> {
        self.pool.acquire(AllocationRequest {
            source_bus,
            source,
            target,
            size,
            peek: &[],
        })
    }

    fn append(&mut self, handle: Self::Handle, bytes: &[u8]) -> Result<(), ProviderError> {
        self.pool
            .get_mut(handle)?
            .append(bytes)
            .map_err(|_| ProviderError::SizeTooLarge)
    }

    fn finish(&mut self, handle: Self::Handle) -> Result<TransportResult, ProviderError> {
        let result = self.listener.message_received(self.pool.get(handle)?);
        self.pool.release(handle)?;
        Ok(result)
    }

    fn cancel(&mut self, handle: Self::Handle) {
        let _ = self.pool.release(handle);
    }
}

/// Deferred action emitted by a diagnostic handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticHandlerAction {
    /// Queue an encoded ACK/NACK control frame.
    Send(Frame),
    /// Close the connection after the queued frame has been sent.
    Close(CloseMode),
}

/// Parsed diagnostic acknowledgement received from the peer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReceivedDiagnosticAck {
    /// `true` for payload type 0x8002, `false` for 0x8003.
    pub positive: bool,
    /// ACK source address.
    pub source_address: u16,
    /// ACK target address.
    pub target_address: u16,
    /// ACK/NACK code.
    pub code: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReceiveKind {
    Diagnostic,
    Ack { positive: bool },
    Discard,
}

#[derive(Debug, Clone, Copy)]
struct ReceiveState<H: Copy> {
    kind: ReceiveKind,
    remaining: usize,
    total: usize,
    prefix: [u8; ACK_MIN_SIZE],
    prefix_len: usize,
    source: u16,
    target: u16,
    handle: Option<H>,
    echo: [u8; ACK_ECHO_SIZE],
    echo_len: usize,
    rejected: bool,
}

impl<H: Copy> ReceiveState<H> {
    const fn new(kind: ReceiveKind, total: usize) -> Self {
        Self {
            kind,
            remaining: total,
            total,
            prefix: [0; ACK_MIN_SIZE],
            prefix_len: 0,
            source: 0,
            target: 0,
            handle: None,
            echo: [0; ACK_ECHO_SIZE],
            echo_len: 0,
            rejected: false,
        }
    }
}

/// Stateful receiver for diagnostic messages and ACKs on one TCP connection.
///
/// Construct one handler per connection. After every call to
/// `ServerTransportLayer::handle_bytes`, call [`apply_actions`](Self::apply_actions)
/// before queuing application responses; this preserves upstream's
/// ACK-before-UDS-response ordering.
pub struct DiagnosticMessageHandler<'a, G: DiagnosticGateway> {
    version: ProtocolVersion,
    source_bus: u8,
    expected_source: Option<u16>,
    entity_address: u16,
    max_payload_length: usize,
    gateway: &'a mut G,
    receive: Option<ReceiveState<G::Handle>>,
    actions: RingQueue<DiagnosticHandlerAction, HANDLER_ACTION_CAPACITY>,
    events: RingQueue<ReceivedDiagnosticAck, HANDLER_EVENT_CAPACITY>,
    dropped_actions: u32,
}

impl<'a, G: DiagnosticGateway> DiagnosticMessageHandler<'a, G> {
    /// Create a handler. `expected_source` must be `Some` only while routing
    /// is active; `None` causes a diagnostic invalid-source NACK and close.
    pub fn new(
        version: ProtocolVersion,
        source_bus: u8,
        expected_source: Option<u16>,
        entity_address: u16,
        max_payload_length: usize,
        gateway: &'a mut G,
    ) -> Self {
        Self {
            version,
            source_bus,
            expected_source,
            entity_address,
            max_payload_length,
            gateway,
            receive: None,
            actions: RingQueue::new(),
            events: RingQueue::new(),
            dropped_actions: 0,
        }
    }

    /// Remove the next deferred wire/close action.
    pub fn take_action(&mut self) -> Option<DiagnosticHandlerAction> {
        self.actions.pop()
    }

    /// Apply all pending actions to the owning server connection in FIFO
    /// order. A close is deferred by the layer until its next poll.
    pub fn apply_actions<const N: usize>(
        &mut self,
        layer: &mut ServerTransportLayer<N>,
        connection: ConnectionId,
    ) -> bool {
        let mut accepted = true;
        while let Some(action) = self.take_action() {
            accepted &= match action {
                DiagnosticHandlerAction::Send(frame) => layer.queue_frame(connection, frame),
                DiagnosticHandlerAction::Close(mode) => layer.mark_close(connection, mode),
            };
        }
        accepted
    }

    /// Remove the next peer ACK notification.
    pub fn take_received_ack(&mut self) -> Option<ReceivedDiagnosticAck> {
        self.events.pop()
    }

    /// Number of handler actions dropped because the fixed queue filled.
    pub const fn dropped_actions(&self) -> u32 {
        self.dropped_actions
    }

    fn push_action(&mut self, action: DiagnosticHandlerAction) {
        if !self.actions.push(action) {
            self.dropped_actions = self.dropped_actions.saturating_add(1);
        }
    }

    fn generic_nack(&mut self, code: NackCode, close: bool) {
        let packet = Packet {
            version: self.version,
            payload: Payload::GenericNack(code),
        };
        if let Some(frame) = Frame::from_packet(&packet) {
            self.push_action(DiagnosticHandlerAction::Send(frame));
        }
        if close {
            self.push_action(DiagnosticHandlerAction::Close(CloseMode::Close));
        }
    }

    fn diagnostic_ack(&mut self, source: u16, target: u16, code: DiagNackCode, echo: &[u8]) {
        let ack = DiagnosticAck {
            source_address: target,
            target_address: source,
            code: code.as_byte(),
            previous_data: echo,
        };
        let payload = if code == DiagNackCode::Success {
            Payload::DiagnosticPositiveAck(ack)
        } else {
            Payload::DiagnosticNegativeAck(ack)
        };
        let packet = Packet {
            version: self.version,
            payload,
        };
        if let Some(frame) = Frame::from_packet(&packet) {
            self.push_action(DiagnosticHandlerAction::Send(frame));
        }
    }

    fn reject_diagnostic(
        &mut self,
        state: &mut ReceiveState<G::Handle>,
        code: DiagNackCode,
        close: bool,
    ) {
        if let Some(handle) = state.handle.take() {
            self.gateway.cancel(handle);
        }
        self.diagnostic_ack(
            state.source,
            state.target,
            code,
            &state.echo[..state.echo_len],
        );
        state.rejected = true;
        if close {
            self.push_action(DiagnosticHandlerAction::Close(CloseMode::Close));
        }
    }

    fn map_provider_error(error: ProviderError) -> (DiagNackCode, bool) {
        match error {
            ProviderError::InvalidSourceAddress => (DiagNackCode::InvalidSourceAddress, true),
            ProviderError::InvalidTargetAddress => (DiagNackCode::InvalidTargetAddress, false),
            ProviderError::SizeTooLarge => (DiagNackCode::DiagnosticMessageTooLarge, false),
            ProviderError::NoMessageAvailable => (DiagNackCode::OutOfMemory, false),
            ProviderError::NotResponsible | ProviderError::InvalidHandle => {
                (DiagNackCode::UnknownNetwork, false)
            }
        }
    }

    fn process_diagnostic(&mut self, state: &mut ReceiveState<G::Handle>, bytes: &[u8]) {
        let mut offset = 0;
        if state.prefix_len < ADDRESS_SIZE {
            let take = (ADDRESS_SIZE - state.prefix_len).min(bytes.len());
            state.prefix[state.prefix_len..state.prefix_len + take].copy_from_slice(&bytes[..take]);
            state.prefix_len += take;
            offset += take;
            if state.prefix_len == ADDRESS_SIZE {
                state.source = u16::from_be_bytes([state.prefix[0], state.prefix[1]]);
                state.target = u16::from_be_bytes([state.prefix[2], state.prefix[3]]);
                if self.expected_source != Some(state.source) {
                    self.reject_diagnostic(state, DiagNackCode::InvalidSourceAddress, true);
                } else if state.target != self.entity_address {
                    self.reject_diagnostic(state, DiagNackCode::InvalidTargetAddress, false);
                } else {
                    let size = state.total - ADDRESS_SIZE;
                    match self
                        .gateway
                        .allocate(self.source_bus, state.source, state.target, size)
                    {
                        Ok(handle) => state.handle = Some(handle),
                        Err(error) => {
                            let (code, close) = Self::map_provider_error(error);
                            self.reject_diagnostic(state, code, close);
                        }
                    }
                }
            }
        }

        let data = &bytes[offset..];
        let echo_take = (ACK_ECHO_SIZE - state.echo_len).min(data.len());
        state.echo[state.echo_len..state.echo_len + echo_take].copy_from_slice(&data[..echo_take]);
        state.echo_len += echo_take;
        if !state.rejected {
            if let Some(handle) = state.handle {
                if let Err(error) = self.gateway.append(handle, data) {
                    let (code, close) = Self::map_provider_error(error);
                    self.reject_diagnostic(state, code, close);
                }
            }
        }
    }

    fn finish_diagnostic(&mut self, state: &mut ReceiveState<G::Handle>) {
        if state.rejected {
            return;
        }
        let Some(handle) = state.handle.take() else {
            self.reject_diagnostic(state, DiagNackCode::UnknownNetwork, false);
            return;
        };

        // Queue the positive ACK before invoking UDS. If the listener rejects
        // the message, rewrite the just-queued ACK to NACK 0x08.
        self.diagnostic_ack(
            state.source,
            state.target,
            DiagNackCode::Success,
            &state.echo[..state.echo_len],
        );
        let accepted = matches!(self.gateway.finish(handle), Ok(TransportResult::Ok));
        if !accepted {
            let ack = DiagnosticAck {
                source_address: state.target,
                target_address: state.source,
                code: DiagNackCode::TransportProtocolError.as_byte(),
                previous_data: &state.echo[..state.echo_len],
            };
            let packet = Packet {
                version: self.version,
                payload: Payload::DiagnosticNegativeAck(ack),
            };
            if let Some(frame) = Frame::from_packet(&packet) {
                if let Some(DiagnosticHandlerAction::Send(last)) = self.actions.back_mut() {
                    *last = frame;
                }
            }
        }
    }

    fn finish_ack(&mut self, state: &ReceiveState<G::Handle>, positive: bool) {
        if state.prefix_len < ACK_MIN_SIZE {
            return;
        }
        let _ = self.events.push(ReceivedDiagnosticAck {
            positive,
            source_address: u16::from_be_bytes([state.prefix[0], state.prefix[1]]),
            target_address: u16::from_be_bytes([state.prefix[2], state.prefix[3]]),
            code: state.prefix[4],
        });
    }
}

impl<G: DiagnosticGateway> MessageHandler for DiagnosticMessageHandler<'_, G> {
    fn header_received(&mut self, payload_type: u16, payload_length: u32) -> bool {
        let Some(payload_type) = PayloadType::from_u16(payload_type) else {
            return false;
        };
        let Ok(length) = usize::try_from(payload_length) else {
            self.generic_nack(NackCode::MessageTooLarge, false);
            self.receive = Some(ReceiveState::new(ReceiveKind::Discard, 0));
            return true;
        };
        match payload_type {
            PayloadType::DiagnosticMessage => {
                if length < ADDRESS_SIZE {
                    self.generic_nack(NackCode::InvalidPayloadLength, true);
                    self.receive = Some(ReceiveState::new(ReceiveKind::Discard, length));
                } else if length > self.max_payload_length {
                    self.generic_nack(NackCode::MessageTooLarge, false);
                    self.receive = Some(ReceiveState::new(ReceiveKind::Discard, length));
                } else {
                    self.receive = Some(ReceiveState::new(ReceiveKind::Diagnostic, length));
                }
                true
            }
            PayloadType::DiagnosticMessagePositiveAck
            | PayloadType::DiagnosticMessageNegativeAck => {
                if length < ACK_MIN_SIZE {
                    self.generic_nack(NackCode::InvalidPayloadLength, true);
                    self.receive = Some(ReceiveState::new(ReceiveKind::Discard, length));
                } else if length > self.max_payload_length {
                    self.generic_nack(NackCode::MessageTooLarge, false);
                    self.receive = Some(ReceiveState::new(ReceiveKind::Discard, length));
                } else {
                    self.receive = Some(ReceiveState::new(
                        ReceiveKind::Ack {
                            positive: payload_type == PayloadType::DiagnosticMessagePositiveAck,
                        },
                        length,
                    ));
                }
                true
            }
            _ => false,
        }
    }

    fn payload_chunk(&mut self, bytes: &[u8]) {
        let Some(mut state) = self.receive.take() else {
            return;
        };
        match state.kind {
            ReceiveKind::Diagnostic => self.process_diagnostic(&mut state, bytes),
            ReceiveKind::Ack { .. } => {
                let take = (ACK_MIN_SIZE - state.prefix_len).min(bytes.len());
                state.prefix[state.prefix_len..state.prefix_len + take]
                    .copy_from_slice(&bytes[..take]);
                state.prefix_len += take;
            }
            ReceiveKind::Discard => {}
        }
        state.remaining = state.remaining.saturating_sub(bytes.len());
        if state.remaining == 0 {
            match state.kind {
                ReceiveKind::Diagnostic => self.finish_diagnostic(&mut state),
                ReceiveKind::Ack { positive } => self.finish_ack(&state, positive),
                ReceiveKind::Discard => {}
            }
        } else {
            self.receive = Some(state);
        }
    }
}

/// Generation-checked token for a bounded diagnostic transmit job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticSendToken {
    slot: u16,
    generation: u16,
}

/// Diagnostic send queue failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticSendError {
    /// No routing TCP connection matches the message target address.
    NoRoute,
    /// Every send-job slot is occupied.
    QueueFull,
    /// The connection wire queue could not accept the job.
    Backpressure,
    /// Token is stale or already completed.
    InvalidToken,
    /// Caller-provided output storage is too small.
    OutputTooSmall,
}

/// Scatter-friendly view of one encoded 0x8001 message.
pub struct DiagnosticWireFrame<'a> {
    prefix: [u8; 12],
    data: &'a [u8],
}

impl DiagnosticWireFrame<'_> {
    /// DoIP header plus source/target address bytes.
    pub const fn prefix(&self) -> &[u8; 12] {
        &self.prefix
    }

    /// Diagnostic application bytes.
    pub const fn data(&self) -> &[u8] {
        self.data
    }

    /// Total wire length.
    pub const fn len(&self) -> usize {
        self.prefix.len() + self.data.len()
    }

    /// Whether the frame contains no wire bytes (always false for DoIP).
    pub const fn is_empty(&self) -> bool {
        false
    }

    /// Encode the scatter view into contiguous caller-owned storage.
    pub fn encode_into(&self, output: &mut [u8]) -> Result<usize, DiagnosticSendError> {
        if output.len() < self.len() {
            return Err(DiagnosticSendError::OutputTooSmall);
        }
        output[..12].copy_from_slice(&self.prefix);
        output[12..self.len()].copy_from_slice(self.data);
        Ok(self.len())
    }
}

/// Fixed-capacity transmit jobs for diagnostic responses.
pub struct DiagnosticSender<const JOBS: usize, const PAYLOAD: usize> {
    version: ProtocolVersion,
    messages: [TransportMessage<PAYLOAD>; JOBS],
    in_use: [bool; JOBS],
    generations: [u16; JOBS],
    connections: [Option<ConnectionId>; JOBS],
}

impl<const JOBS: usize, const PAYLOAD: usize> DiagnosticSender<JOBS, PAYLOAD> {
    /// Create an empty sender.
    pub fn new(version: ProtocolVersion) -> Self {
        Self {
            version,
            messages: core::array::from_fn(|_| TransportMessage::new()),
            in_use: [false; JOBS],
            generations: [0; JOBS],
            connections: [None; JOBS],
        }
    }

    /// Copy and queue a transport message. The message target is the internal
    /// source address selected by routing activation.
    pub fn submit<const N: usize>(
        &mut self,
        layer: &mut ServerTransportLayer<N>,
        message: &TransportMessage<PAYLOAD>,
    ) -> Result<DiagnosticSendToken, DiagnosticSendError> {
        let connection = layer
            .find_routing_connection_by_internal_source_address(message.target_address())
            .ok_or(DiagnosticSendError::NoRoute)?;
        let slot = self
            .in_use
            .iter()
            .position(|used| !*used)
            .ok_or(DiagnosticSendError::QueueFull)?;
        self.in_use[slot] = true;
        self.messages[slot] = message.clone();
        self.connections[slot] = Some(connection);
        let token = DiagnosticSendToken {
            slot: slot as u16,
            generation: self.generations[slot],
        };
        if !layer.queue_diagnostic_send(connection, token) {
            self.release_slot(slot);
            return Err(DiagnosticSendError::Backpressure);
        }
        Ok(token)
    }

    /// Connection owning a live send job.
    pub fn connection(
        &self,
        token: DiagnosticSendToken,
    ) -> Result<ConnectionId, DiagnosticSendError> {
        let slot = self.validate(token)?;
        self.connections[slot].ok_or(DiagnosticSendError::InvalidToken)
    }

    /// Resolve a job into a scatter-friendly wire frame.
    pub fn wire_frame(
        &self,
        token: DiagnosticSendToken,
    ) -> Result<DiagnosticWireFrame<'_>, DiagnosticSendError> {
        let slot = self.validate(token)?;
        let message = &self.messages[slot];
        let payload_length = 4usize
            .checked_add(message.payload_len())
            .and_then(|length| u32::try_from(length).ok())
            .ok_or(DiagnosticSendError::OutputTooSmall)?;
        let header = DoIpHeader {
            version: self.version,
            payload_type: PayloadType::DiagnosticMessage,
            payload_length,
        }
        .encode();
        let mut prefix = [0; 12];
        prefix[..8].copy_from_slice(&header);
        prefix[8..10].copy_from_slice(&message.source_address().to_be_bytes());
        prefix[10..12].copy_from_slice(&message.target_address().to_be_bytes());
        Ok(DiagnosticWireFrame {
            prefix,
            data: message.payload(),
        })
    }

    /// Release a successfully sent or cancelled job.
    pub fn complete(&mut self, token: DiagnosticSendToken) -> Result<(), DiagnosticSendError> {
        let slot = self.validate(token)?;
        self.release_slot(slot);
        Ok(())
    }

    /// Number of jobs waiting for socket completion.
    pub fn in_use(&self) -> usize {
        self.in_use.iter().filter(|used| **used).count()
    }

    fn validate(&self, token: DiagnosticSendToken) -> Result<usize, DiagnosticSendError> {
        let slot = usize::from(token.slot);
        if slot >= JOBS || !self.in_use[slot] || self.generations[slot] != token.generation {
            Err(DiagnosticSendError::InvalidToken)
        } else {
            Ok(slot)
        }
    }

    fn release_slot(&mut self, slot: usize) {
        self.messages[slot].clear();
        self.in_use[slot] = false;
        self.connections[slot] = None;
        self.generations[slot] = self.generations[slot].wrapping_add(1);
    }
}

impl<const JOBS: usize, const PAYLOAD: usize> Default for DiagnosticSender<JOBS, PAYLOAD> {
    fn default() -> Self {
        Self::new(ProtocolVersion::Iso2012)
    }
}
