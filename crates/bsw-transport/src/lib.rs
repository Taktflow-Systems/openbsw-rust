// Copyright 2024 Accenture / Taktflow Systems.
//
// Rust port of OpenBSW `transport`.
//
// Design:
//   - `#![no_std]` compatible — uses `core` only, zero heap allocation.
//   - `TransportMessage<N>` owns a fixed `[u8; N]` buffer.
//   - `LogicalAddress` wraps a `u16` matching DoIP-style 16-bit addressing.
//   - Traits (`TransportLayer`, `TransportMessageListener`,
//     `TransportMessageProvider`) are object-safe where possible so that
//     protocol stacks (ISO-TP, UDS, DoIP) can implement them without
//     coupling to a concrete buffer type.

#![cfg_attr(not(feature = "std"), no_std)]

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Error returned when a payload write exceeds the buffer capacity.
///
/// Returned by [`TransportMessage::append`] when the bytes to be written
/// would overflow the fixed `[u8; N]` buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferOverflow;

// ---------------------------------------------------------------------------
// TransportMessage
// ---------------------------------------------------------------------------

/// Transport message with fixed-capacity buffer.
///
/// Contains source/target addresses, a service identifier, and a payload
/// stored in a fixed-capacity byte buffer.  The payload occupies `0..payload_len`
/// of the inner buffer; the remainder is uninitialised/stale data and must not
/// be read directly — always use [`payload`](TransportMessage::payload).
///
/// # Invariant
///
/// `payload_len <= N` at all times.
pub struct TransportMessage<const N: usize> {
    buffer: [u8; N],
    payload_len: usize,
    source_address: u16,
    target_address: u16,
    service_id: u16,
}

// ---------------------------------------------------------------------------
// Constructors and field accessors
// ---------------------------------------------------------------------------

impl<const N: usize> TransportMessage<N> {
    /// Creates an empty `TransportMessage`.
    ///
    /// All address fields are zero and the payload length is zero.
    /// This function is `const` so it can initialise statics.
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; N],
            payload_len: 0,
            source_address: 0,
            target_address: 0,
            service_id: 0,
        }
    }

    /// Returns the fixed buffer capacity (`N`).
    pub const fn capacity() -> usize {
        N
    }

    // ---- Address fields ----------------------------------------------------

    /// Returns the source address.
    #[inline]
    pub const fn source_address(&self) -> u16 {
        self.source_address
    }

    /// Sets the source address.
    #[inline]
    pub fn set_source_address(&mut self, addr: u16) {
        self.source_address = addr;
    }

    /// Returns the target address.
    #[inline]
    pub const fn target_address(&self) -> u16 {
        self.target_address
    }

    /// Sets the target address.
    #[inline]
    pub fn set_target_address(&mut self, addr: u16) {
        self.target_address = addr;
    }

    /// Returns the service identifier.
    #[inline]
    pub const fn service_id(&self) -> u16 {
        self.service_id
    }

    /// Sets the service identifier.
    #[inline]
    pub fn set_service_id(&mut self, id: u16) {
        self.service_id = id;
    }

    // ---- Payload access ----------------------------------------------------

    /// Returns a shared slice over the current payload bytes.
    #[inline]
    pub fn payload(&self) -> &[u8] {
        &self.buffer[..self.payload_len]
    }

    /// Returns a mutable slice over the current payload bytes.
    ///
    /// This grants in-place mutation of existing payload data.  To extend the
    /// payload with new bytes use [`append`](Self::append), or call
    /// [`set_payload_len`](Self::set_payload_len) first to expand the window.
    #[inline]
    pub fn payload_mut(&mut self) -> &mut [u8] {
        &mut self.buffer[..self.payload_len]
    }

    /// Returns the number of bytes currently in the payload.
    #[inline]
    pub const fn payload_len(&self) -> usize {
        self.payload_len
    }

    // ---- Payload manipulation ----------------------------------------------

    /// Sets the payload length directly.
    ///
    /// The new length is clamped to [`capacity()`](Self::capacity) — it will
    /// never exceed `N`.  Bytes in `old_len..new_len` are whatever was
    /// previously stored in the buffer; callers must write them before reading.
    #[inline]
    pub fn set_payload_len(&mut self, len: usize) {
        self.payload_len = len.min(N);
    }

    /// Appends `data` to the end of the payload.
    ///
    /// Returns `Ok(())` on success, or `Err(`[`BufferOverflow`]`)` if there
    /// is insufficient space remaining in the buffer.  On error the message
    /// is unchanged.
    pub fn append(&mut self, data: &[u8]) -> Result<(), BufferOverflow> {
        let new_len = self
            .payload_len
            .checked_add(data.len())
            .ok_or(BufferOverflow)?;
        if new_len > N {
            return Err(BufferOverflow);
        }
        self.buffer[self.payload_len..new_len].copy_from_slice(data);
        self.payload_len = new_len;
        Ok(())
    }

    /// Clears the payload and resets all fields to zero.
    pub fn clear(&mut self) {
        self.payload_len = 0;
        self.source_address = 0;
        self.target_address = 0;
        self.service_id = 0;
    }

    // ---- Validation --------------------------------------------------------

    /// Returns `true` when the message is considered valid.
    ///
    /// A message is valid when its payload length does not exceed the buffer
    /// capacity (this invariant is maintained by all mutation methods, but this
    /// check acts as a defensive sanity gate for external callers).
    #[inline]
    pub const fn is_valid(&self) -> bool {
        self.payload_len <= N
    }
}

// ---------------------------------------------------------------------------
// Default — delegates to `new()`
// ---------------------------------------------------------------------------

impl<const N: usize> Default for TransportMessage<N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// LogicalAddress
// ---------------------------------------------------------------------------

/// Logical transport address (DoIP-style 16-bit or legacy 8-bit).
///
/// Wraps a `u16` address value.  Provides conversions to/from `u8` for
/// legacy protocols that use single-byte addressing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct LogicalAddress(u16);

impl LogicalAddress {
    /// Creates a `LogicalAddress` from a raw `u16`.
    #[inline]
    pub const fn new(addr: u16) -> Self {
        Self(addr)
    }

    /// Creates a `LogicalAddress` from a `u8`, zero-extending to 16 bits.
    #[inline]
    pub const fn from_u8(addr: u8) -> Self {
        Self(addr as u16)
    }

    /// Returns the address as a `u16`.
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self.0
    }

    /// Returns the address as a `u8` if it fits in one byte, otherwise `None`.
    #[inline]
    pub const fn as_u8(self) -> Option<u8> {
        if self.0 <= 0xFF {
            Some(self.0 as u8)
        } else {
            None
        }
    }

    /// Broadcast address (`0xFFFF`).
    pub const BROADCAST: Self = Self(0xFFFF);

    /// Invalid / unset address (`0x0000`).
    pub const INVALID: Self = Self(0x0000);
}

// ---------------------------------------------------------------------------
// TransportResult
// ---------------------------------------------------------------------------

/// Result of a transport message send or receive operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportResult {
    /// Operation completed successfully.
    Ok,
    /// The destination buffer is full; the message was not sent/queued.
    BufferFull,
    /// The supplied address is not routable or is otherwise invalid.
    InvalidAddress,
    /// The operation timed out before completion.
    Timeout,
    /// An unspecified transport-layer error occurred.
    Error,
}

// ---------------------------------------------------------------------------
// TransportMessageListener
// ---------------------------------------------------------------------------

/// Callback for incoming message processing completion.
///
/// Implementors receive a reference to the fully assembled inbound message and
/// return a [`TransportResult`] to signal acceptance or rejection.
pub trait TransportMessageListener {
    /// Called by the transport layer when a complete message has been received.
    fn message_received(&mut self, message: &TransportMessage<256>) -> TransportResult;
}

// ---------------------------------------------------------------------------
// TransportMessageProvider
// ---------------------------------------------------------------------------

/// Provider that manages a pool of message buffers.
///
/// Used by transport layers to allocate receive buffers without heap usage.
pub trait TransportMessageProvider {
    /// The concrete message type managed by this provider.
    type Msg;

    /// Obtains a free message buffer for receiving.
    ///
    /// Returns `None` when the pool is exhausted.
    fn get_message(&mut self) -> Option<&mut Self::Msg>;

    /// Returns a message buffer to the pool after processing is complete.
    fn release_message(&mut self, msg: &mut Self::Msg);
}

// ---------------------------------------------------------------------------
// TransportLayer
// ---------------------------------------------------------------------------

/// Abstract transport layer (CAN, Ethernet, etc.).
///
/// Protocol stacks (ISO-TP, UDS, DoIP) implement this trait to decouple
/// themselves from the physical medium.
pub trait TransportLayer {
    /// Initialises the transport layer.
    ///
    /// Must be called once before any other method.
    fn init(&mut self);

    /// Shuts down the transport layer and releases hardware resources.
    fn shutdown(&mut self);

    /// Queues a message for transmission.
    ///
    /// `data` is the serialised payload bytes.  `source` and `target` provide
    /// the logical routing information.
    fn send(
        &mut self,
        data: &[u8],
        source: LogicalAddress,
        target: LogicalAddress,
    ) -> TransportResult;

    /// Processes all pending operations.
    ///
    /// Call periodically (e.g. from a 1 ms task) to drive timers,
    /// retransmissions, and receive demultiplexing.
    fn cycle(&mut self);
}

// ---------------------------------------------------------------------------
// SendJob
// ---------------------------------------------------------------------------

/// A queued send job that owns the message and tracks completion state.
///
/// The `pending` flag starts `true` and is cleared by
/// [`complete`](SendJob::complete) once the underlying transport layer has
/// consumed the message.
pub struct SendJob<const N: usize> {
    message: TransportMessage<N>,
    pending: bool,
}

impl<const N: usize> SendJob<N> {
    /// Creates a new `SendJob` wrapping `message`.
    ///
    /// The job starts in the *pending* state.
    pub const fn new(message: TransportMessage<N>) -> Self {
        Self {
            message,
            pending: true,
        }
    }

    /// Returns `true` while the send operation has not yet completed.
    #[inline]
    pub const fn is_pending(&self) -> bool {
        self.pending
    }

    /// Returns a shared reference to the message being sent.
    #[inline]
    pub const fn message(&self) -> &TransportMessage<N> {
        &self.message
    }

    /// Returns a mutable reference to the message being sent.
    ///
    /// Use with caution — modifying the message after it has been submitted
    /// to a transport layer may result in inconsistent state.
    #[inline]
    pub fn message_mut(&mut self) -> &mut TransportMessage<N> {
        &mut self.message
    }

    /// Marks the send job as complete.
    ///
    /// After this call [`is_pending`](Self::is_pending) returns `false`.
    #[inline]
    pub fn complete(&mut self) {
        self.pending = false;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{LogicalAddress, SendJob, TransportMessage, TransportResult};

    // -----------------------------------------------------------------------
    // 1. TransportMessage: new is empty
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_new_is_empty() {
        let msg: TransportMessage<64> = TransportMessage::new();
        assert_eq!(msg.payload_len(), 0);
        assert_eq!(msg.payload(), &[] as &[u8]);
        assert_eq!(msg.source_address(), 0);
        assert_eq!(msg.target_address(), 0);
        assert_eq!(msg.service_id(), 0);
    }

    // -----------------------------------------------------------------------
    // 2. TransportMessage: set/get source address
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_source_address() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.set_source_address(0x0102);
        assert_eq!(msg.source_address(), 0x0102);
    }

    // -----------------------------------------------------------------------
    // 3. TransportMessage: set/get target address
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_target_address() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.set_target_address(0xFEDC);
        assert_eq!(msg.target_address(), 0xFEDC);
    }

    // -----------------------------------------------------------------------
    // 4. TransportMessage: set/get service id
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_service_id() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.set_service_id(0x0022);
        assert_eq!(msg.service_id(), 0x0022);
    }

    // -----------------------------------------------------------------------
    // 5. TransportMessage: append data to payload
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_append_data() {
        let mut msg: TransportMessage<16> = TransportMessage::new();
        assert!(msg.append(&[0x01, 0x02, 0x03]).is_ok());
        assert_eq!(msg.payload(), &[0x01, 0x02, 0x03]);
    }

    // -----------------------------------------------------------------------
    // 6. TransportMessage: append overflow returns Err
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_append_overflow_returns_err() {
        let mut msg: TransportMessage<4> = TransportMessage::new();
        assert!(msg.append(&[0xAA, 0xBB, 0xCC, 0xDD]).is_ok());
        // Buffer is now full; any further append must fail without mutation.
        let result = msg.append(&[0xEE]);
        assert!(result.is_err());
        assert_eq!(msg.payload_len(), 4);
    }

    // -----------------------------------------------------------------------
    // 7. TransportMessage: payload_len tracks correctly
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_payload_len_tracks() {
        let mut msg: TransportMessage<32> = TransportMessage::new();
        assert_eq!(msg.payload_len(), 0);
        msg.append(&[1, 2]).unwrap();
        assert_eq!(msg.payload_len(), 2);
        msg.append(&[3, 4, 5]).unwrap();
        assert_eq!(msg.payload_len(), 5);
    }

    // -----------------------------------------------------------------------
    // 8. TransportMessage: clear resets all fields
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_clear_resets_all() {
        let mut msg: TransportMessage<16> = TransportMessage::new();
        msg.set_source_address(0x1234);
        msg.set_target_address(0x5678);
        msg.set_service_id(0x22);
        msg.append(&[0xDE, 0xAD]).unwrap();

        msg.clear();

        assert_eq!(msg.payload_len(), 0);
        assert_eq!(msg.source_address(), 0);
        assert_eq!(msg.target_address(), 0);
        assert_eq!(msg.service_id(), 0);
        assert_eq!(msg.payload(), &[] as &[u8]);
    }

    // -----------------------------------------------------------------------
    // 9. TransportMessage: payload slice is correct length
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_payload_slice_correct_length() {
        let mut msg: TransportMessage<64> = TransportMessage::new();
        msg.append(&[0x10, 0x20, 0x30, 0x40, 0x50]).unwrap();
        let p = msg.payload();
        assert_eq!(p.len(), 5);
        assert_eq!(p[0], 0x10);
        assert_eq!(p[4], 0x50);
    }

    // -----------------------------------------------------------------------
    // 10. TransportMessage: is_valid for empty message
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_is_valid_empty() {
        let msg: TransportMessage<8> = TransportMessage::new();
        assert!(msg.is_valid());
    }

    #[test]
    fn transport_message_is_valid_after_append() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.append(&[1, 2, 3]).unwrap();
        assert!(msg.is_valid());
    }

    // -----------------------------------------------------------------------
    // 11. LogicalAddress: from u16 roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_from_u16_roundtrip() {
        let addr = LogicalAddress::new(0xABCD);
        assert_eq!(addr.as_u16(), 0xABCD);
    }

    // -----------------------------------------------------------------------
    // 12. LogicalAddress: from u8 promotion
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_from_u8_promotion() {
        let addr = LogicalAddress::from_u8(0xFE);
        assert_eq!(addr.as_u16(), 0x00FE);
    }

    // -----------------------------------------------------------------------
    // 13. LogicalAddress: as_u8 for small values
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_as_u8_small_value() {
        let addr = LogicalAddress::new(0x00FF);
        assert_eq!(addr.as_u8(), Some(0xFFu8));
    }

    // -----------------------------------------------------------------------
    // 14. LogicalAddress: as_u8 returns None for > 255
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_as_u8_too_large() {
        let addr = LogicalAddress::new(0x0100);
        assert_eq!(addr.as_u8(), None);

        let addr_max = LogicalAddress::new(0xFFFF);
        assert_eq!(addr_max.as_u8(), None);
    }

    // -----------------------------------------------------------------------
    // 15. LogicalAddress: BROADCAST and INVALID constants
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_constants() {
        assert_eq!(LogicalAddress::BROADCAST.as_u16(), 0xFFFF);
        assert_eq!(LogicalAddress::INVALID.as_u16(), 0x0000);
    }

    // -----------------------------------------------------------------------
    // 16. LogicalAddress: equality and copy
    // -----------------------------------------------------------------------

    #[test]
    fn logical_address_equality_and_copy() {
        let a = LogicalAddress::new(0x1234);
        let b = a; // Copy
        assert_eq!(a, b);

        let c = LogicalAddress::new(0x5678);
        assert_ne!(a, c);
    }

    // -----------------------------------------------------------------------
    // 17. SendJob: new is pending
    // -----------------------------------------------------------------------

    #[test]
    fn send_job_new_is_pending() {
        let msg: TransportMessage<8> = TransportMessage::new();
        let job = SendJob::new(msg);
        assert!(job.is_pending());
    }

    // -----------------------------------------------------------------------
    // 18. SendJob: complete marks not pending
    // -----------------------------------------------------------------------

    #[test]
    fn send_job_complete_marks_not_pending() {
        let msg: TransportMessage<8> = TransportMessage::new();
        let mut job = SendJob::new(msg);
        assert!(job.is_pending());
        job.complete();
        assert!(!job.is_pending());
    }

    // -----------------------------------------------------------------------
    // 19. TransportResult enum variants
    // -----------------------------------------------------------------------

    #[test]
    fn transport_result_variants_are_distinct() {
        assert_eq!(TransportResult::Ok, TransportResult::Ok);
        assert_ne!(TransportResult::Ok, TransportResult::BufferFull);
        assert_ne!(TransportResult::Ok, TransportResult::InvalidAddress);
        assert_ne!(TransportResult::Ok, TransportResult::Timeout);
        assert_ne!(TransportResult::Ok, TransportResult::Error);
        assert_ne!(TransportResult::BufferFull, TransportResult::Timeout);
    }

    #[test]
    fn transport_result_copy() {
        let r = TransportResult::BufferFull;
        let r2 = r; // Copy
        assert_eq!(r, r2);
    }

    // -----------------------------------------------------------------------
    // 20. Zero-capacity message
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_zero_capacity() {
        let mut msg: TransportMessage<0> = TransportMessage::new();
        assert_eq!(TransportMessage::<0>::capacity(), 0);
        assert_eq!(msg.payload_len(), 0);
        assert!(msg.is_valid());

        // Any append must fail immediately.
        assert!(msg.append(&[0x01]).is_err());
        assert!(msg.append(&[]).is_ok()); // empty append is always fine
        assert_eq!(msg.payload_len(), 0);
    }

    // -----------------------------------------------------------------------
    // Additional: set_payload_len clamping
    // -----------------------------------------------------------------------

    #[test]
    fn set_payload_len_clamped_to_capacity() {
        let mut msg: TransportMessage<4> = TransportMessage::new();
        msg.set_payload_len(100); // larger than capacity
        assert_eq!(msg.payload_len(), 4); // clamped
        assert!(msg.is_valid());
    }

    #[test]
    fn set_payload_len_within_capacity() {
        let mut msg: TransportMessage<16> = TransportMessage::new();
        msg.set_payload_len(7);
        assert_eq!(msg.payload_len(), 7);
    }

    // -----------------------------------------------------------------------
    // Additional: payload_mut allows in-place mutation
    // -----------------------------------------------------------------------

    #[test]
    fn payload_mut_in_place_mutation() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.append(&[0x00, 0x00, 0x00]).unwrap();
        msg.payload_mut().copy_from_slice(&[0xAA, 0xBB, 0xCC]);
        assert_eq!(msg.payload(), &[0xAA, 0xBB, 0xCC]);
    }

    // -----------------------------------------------------------------------
    // Additional: Default trait
    // -----------------------------------------------------------------------

    #[test]
    fn transport_message_default_is_empty() {
        let msg: TransportMessage<32> = TransportMessage::default();
        assert_eq!(msg.payload_len(), 0);
        assert_eq!(msg.source_address(), 0);
    }

    // -----------------------------------------------------------------------
    // Additional: SendJob message accessor
    // -----------------------------------------------------------------------

    #[test]
    fn send_job_message_accessor() {
        let mut msg: TransportMessage<8> = TransportMessage::new();
        msg.set_service_id(0x22);
        let job = SendJob::new(msg);
        assert_eq!(job.message().service_id(), 0x22);
    }

    #[test]
    fn send_job_message_mut_accessor() {
        let msg: TransportMessage<8> = TransportMessage::new();
        let mut job = SendJob::new(msg);
        job.message_mut().set_source_address(0xBEEF);
        assert_eq!(job.message().source_address(), 0xBEEF);
    }

    // -----------------------------------------------------------------------
    // Additional: multiple appends up to exact capacity
    // -----------------------------------------------------------------------

    #[test]
    fn append_exactly_fills_buffer() {
        let mut msg: TransportMessage<4> = TransportMessage::new();
        assert!(msg.append(&[0x01]).is_ok());
        assert!(msg.append(&[0x02]).is_ok());
        assert!(msg.append(&[0x03]).is_ok());
        assert!(msg.append(&[0x04]).is_ok());
        assert_eq!(msg.payload_len(), 4);
        assert_eq!(msg.payload(), &[0x01, 0x02, 0x03, 0x04]);
        // One more byte must fail.
        assert!(msg.append(&[0x05]).is_err());
        // State unchanged after failed append.
        assert_eq!(msg.payload_len(), 4);
    }
}
