//! Generation-checked incoming/outgoing diagnostic connection pools (E16).

use bsw_transport::TransportMessage;

use crate::dispatcher::RequestAddressing;

/// Direction of a diagnostic connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Remote tester initiated a request.
    Incoming,
    /// ECU initiated a request toward another diagnostic node.
    Outgoing,
}

/// Connection lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionState {
    /// Request is owned but not yet dispatched.
    Received,
    /// Job is executing.
    Processing,
    /// Response is owned by the transport.
    Sending,
    /// Cancellation/termination was requested.
    Terminated,
}

/// Generation-checked connection ownership token.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ConnectionHandle {
    slot: u16,
    generation: u16,
}

/// Connection pool error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionError {
    /// Pool is exhausted.
    Exhausted,
    /// Request exceeds fixed message storage.
    RequestTooLarge,
    /// Handle is stale or already released.
    InvalidHandle,
    /// Requested state transition is illegal.
    InvalidState,
}

/// Metadata and payload used to acquire a diagnostic connection.
#[derive(Debug, Clone, Copy)]
pub struct ConnectionRequest<'a> {
    /// Incoming/outgoing direction.
    pub direction: Direction,
    /// Physical or functional request.
    pub addressing: RequestAddressing,
    /// Owning transport identifier.
    pub source_bus: u8,
    /// Transport source address.
    pub source: u16,
    /// Transport target address.
    pub target: u16,
    /// Request payload copied into the pool.
    pub payload: &'a [u8],
    /// Whether processed notification is required on completion.
    pub notify_processed: bool,
}

/// One diagnostic transaction owned by a pool slot.
pub struct Connection<const REQUEST: usize, const RESPONSE: usize> {
    /// Incoming/outgoing direction.
    pub direction: Direction,
    /// Physical/functional request type.
    pub addressing: RequestAddressing,
    /// Source transport bus.
    pub source_bus: u8,
    /// Request message.
    pub request: TransportMessage<REQUEST>,
    /// Response message.
    pub response: TransportMessage<RESPONSE>,
    /// Current transaction state.
    pub state: ConnectionState,
    /// Whether upstream/provider expects a processed notification.
    pub notify_processed: bool,
}

/// Fixed incoming/outgoing connection pool.
pub struct ConnectionPool<const COUNT: usize, const REQUEST: usize, const RESPONSE: usize> {
    slots: [Option<Connection<REQUEST, RESPONSE>>; COUNT],
    generations: [u16; COUNT],
}

impl<const COUNT: usize, const REQUEST: usize, const RESPONSE: usize>
    ConnectionPool<COUNT, REQUEST, RESPONSE>
{
    /// Create an empty pool.
    pub fn new() -> Self {
        Self {
            slots: core::array::from_fn(|_| None),
            generations: [0; COUNT],
        }
    }

    /// Allocate and copy one transaction request.
    pub fn acquire(
        &mut self,
        allocation: ConnectionRequest<'_>,
    ) -> Result<ConnectionHandle, ConnectionError> {
        if allocation.payload.len() > REQUEST {
            return Err(ConnectionError::RequestTooLarge);
        }
        let Some(slot) = self.slots.iter().position(Option::is_none) else {
            return Err(ConnectionError::Exhausted);
        };
        let mut request_message = TransportMessage::new();
        request_message.set_source_address(allocation.source);
        request_message.set_target_address(allocation.target);
        request_message
            .append(allocation.payload)
            .map_err(|_| ConnectionError::RequestTooLarge)?;
        self.slots[slot] = Some(Connection {
            direction: allocation.direction,
            addressing: allocation.addressing,
            source_bus: allocation.source_bus,
            request: request_message,
            response: TransportMessage::new(),
            state: ConnectionState::Received,
            notify_processed: allocation.notify_processed,
        });
        Ok(ConnectionHandle {
            slot: slot as u16,
            generation: self.generations[slot],
        })
    }

    /// Borrow an owned connection.
    pub fn get(
        &self,
        handle: ConnectionHandle,
    ) -> Result<&Connection<REQUEST, RESPONSE>, ConnectionError> {
        let slot = self.validate(handle)?;
        self.slots[slot]
            .as_ref()
            .ok_or(ConnectionError::InvalidHandle)
    }

    /// Mutably borrow an owned connection.
    pub fn get_mut(
        &mut self,
        handle: ConnectionHandle,
    ) -> Result<&mut Connection<REQUEST, RESPONSE>, ConnectionError> {
        let slot = self.validate(handle)?;
        self.slots[slot]
            .as_mut()
            .ok_or(ConnectionError::InvalidHandle)
    }

    /// Transition Received -> Processing -> Sending.
    pub fn advance(
        &mut self,
        handle: ConnectionHandle,
    ) -> Result<ConnectionState, ConnectionError> {
        let connection = self.get_mut(handle)?;
        connection.state = match connection.state {
            ConnectionState::Received => ConnectionState::Processing,
            ConnectionState::Processing => ConnectionState::Sending,
            ConnectionState::Sending | ConnectionState::Terminated => {
                return Err(ConnectionError::InvalidState)
            }
        };
        Ok(connection.state)
    }

    /// Mark a connection terminated. Release remains explicit so processed
    /// notification can observe metadata first.
    pub fn cancel(&mut self, handle: ConnectionHandle) -> Result<(), ConnectionError> {
        self.get_mut(handle)?.state = ConnectionState::Terminated;
        Ok(())
    }

    /// Release request/response ownership and invalidate the handle.
    pub fn release(&mut self, handle: ConnectionHandle) -> Result<(), ConnectionError> {
        let slot = self.validate(handle)?;
        self.slots[slot] = None;
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        Ok(())
    }

    /// Occupied connection count.
    pub fn in_use(&self) -> usize {
        self.slots.iter().filter(|slot| slot.is_some()).count()
    }

    fn validate(&self, handle: ConnectionHandle) -> Result<usize, ConnectionError> {
        let slot = usize::from(handle.slot);
        if slot >= COUNT
            || self.generations[slot] != handle.generation
            || self.slots[slot].is_none()
        {
            Err(ConnectionError::InvalidHandle)
        } else {
            Ok(slot)
        }
    }
}

impl<const COUNT: usize, const REQUEST: usize, const RESPONSE: usize> Default
    for ConnectionPool<COUNT, REQUEST, RESPONSE>
{
    fn default() -> Self {
        Self::new()
    }
}
