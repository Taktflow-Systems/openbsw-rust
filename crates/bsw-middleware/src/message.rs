//! Typed middleware message and transport boundary.

/// Middleware message category, matching upstream bit values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MessageKind {
    /// Request expecting a response.
    Request = 1,
    /// Request without response.
    FireAndForget = 2,
    /// Successful response.
    Response = 4,
    /// Published event.
    Event = 8,
    /// Error response.
    Error = 16,
}

/// Error payload values used by middleware responses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorState {
    /// No error.
    None = 0,
    /// Project-defined service error.
    UserDefined = 4,
    /// Service cannot accept another request.
    ServiceBusy = 5,
    /// Requested service does not exist.
    ServiceNotFound = 6,
    /// Request could not be serialized.
    Serialization = 7,
    /// Payload could not be decoded.
    Deserialization = 8,
    /// Target queue is full.
    QueueFull = 9,
}

/// Fixed message header shared by simulation and production transports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    /// Service identifier.
    pub service_id: u16,
    /// Method/event identifier.
    pub member_id: u16,
    /// Request correlation identifier.
    pub request_id: u16,
    /// Service instance identifier.
    pub instance_id: u16,
    /// Origin cluster.
    pub source_cluster: u8,
    /// Destination cluster.
    pub target_cluster: u8,
    /// Per-proxy address.
    pub address_id: u8,
    /// Message category.
    pub kind: MessageKind,
}

/// Message construction error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageError {
    /// Payload exceeds the inline fixed capacity.
    PayloadTooLarge,
    /// An external offset/size pair exceeds the configured shared region.
    ExternalPayloadOutOfRange,
}

/// Fixed inline message with an optional checked external payload handle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Message<const INLINE: usize> {
    header: Header,
    payload: [u8; INLINE],
    payload_len: usize,
    external: Option<(u32, u32)>,
    error: ErrorState,
}

impl<const INLINE: usize> Message<INLINE> {
    /// Create a message with empty inline payload.
    pub const fn new(header: Header) -> Self {
        Self {
            header,
            payload: [0; INLINE],
            payload_len: 0,
            external: None,
            error: ErrorState::None,
        }
    }

    /// Header fields.
    pub const fn header(&self) -> &Header {
        &self.header
    }

    /// Inline payload bytes, empty for an external/error payload.
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }

    /// Copy an inline payload transactionally.
    pub fn set_payload(&mut self, data: &[u8]) -> Result<(), MessageError> {
        if data.len() > INLINE {
            return Err(MessageError::PayloadTooLarge);
        }
        self.payload[..data.len()].copy_from_slice(data);
        self.payload_len = data.len();
        self.external = None;
        self.error = ErrorState::None;
        Ok(())
    }

    /// Set an external shared-region handle after bounds checking.
    pub fn set_external(
        &mut self,
        offset: u32,
        size: u32,
        region_size: u32,
    ) -> Result<(), MessageError> {
        if offset.checked_add(size).is_none_or(|end| end > region_size) {
            return Err(MessageError::ExternalPayloadOutOfRange);
        }
        self.payload_len = 0;
        self.external = Some((offset, size));
        self.error = ErrorState::None;
        Ok(())
    }

    /// Checked external payload offset and size.
    pub const fn external(&self) -> Option<(u32, u32)> {
        self.external
    }

    /// Convert the message to an error response.
    pub fn set_error(&mut self, error: ErrorState) {
        self.header.kind = MessageKind::Error;
        self.payload_len = 0;
        self.external = None;
        self.error = error;
    }

    /// Error response state, or [`ErrorState::None`].
    pub const fn error(&self) -> ErrorState {
        self.error
    }
}

/// Result returned by production and simulation middleware transports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MiddlewareResult {
    /// Message was accepted.
    Ok,
    /// Destination queue is full.
    QueueFull,
    /// Service/instance is not registered.
    ServiceNotFound,
    /// Target cluster does not match the connection.
    WrongTargetCluster,
    /// Payload failed validation.
    InvalidPayload,
    /// Requested capability is not implemented.
    NotImplemented,
}

/// One API shared by simulated and production middleware connections.
pub trait MiddlewareTransport<const INLINE: usize> {
    /// Source cluster owned by this connection.
    fn source_cluster(&self) -> u8;

    /// Send or enqueue one typed middleware message.
    fn send(&mut self, message: &Message<INLINE>) -> MiddlewareResult;
}
