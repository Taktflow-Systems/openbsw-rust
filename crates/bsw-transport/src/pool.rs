//! Generation-checked fixed transport-message pools (package E01).

use crate::TransportMessage;

/// Stable ownership token for one checked-out pool slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MessageHandle {
    slot: u16,
    generation: u16,
}

impl MessageHandle {
    /// Slot index, useful for deterministic tracing.
    pub const fn slot(self) -> u16 {
        self.slot
    }

    /// Reuse generation, useful for detecting stale ownership.
    pub const fn generation(self) -> u16 {
        self.generation
    }
}

/// Detailed provider result mirroring upstream `ITransportMessageProvider`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderError {
    /// Source address is not accepted by the provider.
    InvalidSourceAddress,
    /// Target address is not accepted by the provider.
    InvalidTargetAddress,
    /// Every matching message buffer is checked out.
    NoMessageAvailable,
    /// Requested payload cannot fit in a pool message.
    SizeTooLarge,
    /// This provider does not own the requested source bus.
    NotResponsible,
    /// The handle is stale, already released, or belongs to another slot.
    InvalidHandle,
}

/// Allocation request used by [`MessagePool::acquire`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AllocationRequest<'a> {
    /// Bus on which the message arrived.
    pub source_bus: u8,
    /// Logical source address.
    pub source: u16,
    /// Logical target address.
    pub target: u16,
    /// Expected total payload size.
    pub size: usize,
    /// Bytes already observed while selecting a provider.
    pub peek: &'a [u8],
}

/// Address/provider admission policy.
pub trait AdmissionPolicy {
    /// Validate and classify an allocation request.
    fn admit(&self, request: &AllocationRequest<'_>) -> Result<(), ProviderError>;
}

/// Policy accepting every address from one source bus.
#[derive(Debug, Clone, Copy)]
pub struct BusPolicy {
    /// Bus owned by this pool.
    pub source_bus: u8,
}

impl AdmissionPolicy for BusPolicy {
    fn admit(&self, request: &AllocationRequest<'_>) -> Result<(), ProviderError> {
        if request.source_bus != self.source_bus {
            Err(ProviderError::NotResponsible)
        } else if request.source == u16::MAX {
            Err(ProviderError::InvalidSourceAddress)
        } else if request.target == u16::MAX {
            Err(ProviderError::InvalidTargetAddress)
        } else {
            Ok(())
        }
    }
}

/// Heap-free pool of independently owned transport messages.
pub struct MessagePool<const COUNT: usize, const PAYLOAD: usize, P = BusPolicy> {
    messages: [TransportMessage<PAYLOAD>; COUNT],
    in_use: [bool; COUNT],
    generations: [u16; COUNT],
    policy: P,
}

impl<const COUNT: usize, const PAYLOAD: usize, P: AdmissionPolicy> MessagePool<COUNT, PAYLOAD, P> {
    /// Create an empty pool using `policy` for routing/admission checks.
    pub fn new(policy: P) -> Self {
        Self {
            messages: core::array::from_fn(|_| TransportMessage::new()),
            in_use: [false; COUNT],
            generations: [0; COUNT],
            policy,
        }
    }

    /// Acquire one message, initialize its metadata, and copy `peek` bytes.
    pub fn acquire(
        &mut self,
        request: AllocationRequest<'_>,
    ) -> Result<MessageHandle, ProviderError> {
        self.policy.admit(&request)?;
        if request.size > PAYLOAD || request.peek.len() > request.size {
            return Err(ProviderError::SizeTooLarge);
        }
        let Some(slot) = self.in_use.iter().position(|used| !*used) else {
            return Err(ProviderError::NoMessageAvailable);
        };
        self.in_use[slot] = true;
        let message = &mut self.messages[slot];
        message.clear();
        message.set_source_address(request.source);
        message.set_target_address(request.target);
        message
            .append(request.peek)
            .map_err(|_| ProviderError::SizeTooLarge)?;
        Ok(MessageHandle {
            slot: slot as u16,
            generation: self.generations[slot],
        })
    }

    /// Borrow an owned message.
    pub fn get(&self, handle: MessageHandle) -> Result<&TransportMessage<PAYLOAD>, ProviderError> {
        let slot = self.validate(handle)?;
        Ok(&self.messages[slot])
    }

    /// Mutably borrow an owned message.
    pub fn get_mut(
        &mut self,
        handle: MessageHandle,
    ) -> Result<&mut TransportMessage<PAYLOAD>, ProviderError> {
        let slot = self.validate(handle)?;
        Ok(&mut self.messages[slot])
    }

    /// Release ownership. Stale/double releases are rejected without effect.
    pub fn release(&mut self, handle: MessageHandle) -> Result<(), ProviderError> {
        let slot = self.validate(handle)?;
        self.messages[slot].clear();
        self.in_use[slot] = false;
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        Ok(())
    }

    /// Number of currently checked-out messages.
    pub fn in_use(&self) -> usize {
        self.in_use.iter().filter(|used| **used).count()
    }

    fn validate(&self, handle: MessageHandle) -> Result<usize, ProviderError> {
        let slot = usize::from(handle.slot);
        if slot >= COUNT || !self.in_use[slot] || self.generations[slot] != handle.generation {
            Err(ProviderError::InvalidHandle)
        } else {
            Ok(slot)
        }
    }
}
