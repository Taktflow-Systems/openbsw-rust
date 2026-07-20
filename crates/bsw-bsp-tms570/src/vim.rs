//! Bounded TMS570LC4357 VIM registration and dispatch model.
//!
//! VIM exposes 128 fixed request channels. Hardware index registers encode
//! no-pending as zero and a live channel as `channel + 1`; this module keeps
//! that conversion, registration ownership, IRQ/FIQ class, enable state, and
//! unexpected dispatch accounting testable without target hardware.

use crate::board;

/// Number of implemented LC4357 VIM request channels.
pub const CHANNEL_COUNT: usize = 128;
/// First application-registerable channel; zero and one are startup-owned.
pub const FIRST_REGISTERABLE_CHANNEL: u8 = 2;

/// A validated VIM request channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VimChannel(u8);

impl VimChannel {
    /// Validate an application-registerable channel number.
    pub const fn new(channel: u8) -> Result<Self, VimError> {
        if channel < FIRST_REGISTERABLE_CHANNEL {
            return Err(VimError::ReservedChannel);
        }
        if channel as usize >= CHANNEL_COUNT {
            return Err(VimError::ChannelOutOfRange);
        }
        Ok(Self(channel))
    }

    /// Raw zero-based hardware channel.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// One-based value reported by IRQINDEX/FIQINDEX.
    pub const fn index(self) -> u8 {
        self.0 + 1
    }

    /// VIM RAM word containing this channel's ISR address. Word zero is the
    /// phantom handler, so channel `n` occupies word `n + 1`.
    pub const fn vector_word(self) -> usize {
        self.0 as usize + 1
    }
}

/// CPU exception class selected for a VIM request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterruptClass {
    /// Normal interrupt request.
    Irq,
    /// Fast interrupt request.
    Fiq,
}

/// VIM registration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VimError {
    /// Channels zero and one remain startup-owned.
    ReservedChannel,
    /// The raw channel does not exist on this device.
    ChannelOutOfRange,
    /// A handler already owns the channel.
    AlreadyRegistered,
}

/// Result of decoding and dispatching one hardware index observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchOutcome {
    /// Hardware reported no active request.
    NoPending,
    /// A registered, enabled handler ran.
    Handled(VimChannel),
    /// A disabled, unregistered, or invalid index reached the default path.
    Defaulted { raw_index: u32 },
}

/// Plain A32 handler installed in VIM vector RAM.
pub type Handler = fn();

#[derive(Clone, Copy)]
struct Slot {
    handler: Option<Handler>,
    class: InterruptClass,
    enabled: bool,
}

const EMPTY_SLOT: Slot = Slot {
    handler: None,
    class: InterruptClass::Irq,
    enabled: false,
};

/// Fixed-capacity registration table and default-capture state.
pub struct VimTable {
    slots: [Slot; CHANNEL_COUNT],
    unexpected_count: u32,
    last_unexpected_index: u32,
}

impl VimTable {
    /// Create a table with every channel disabled and unregistered.
    pub const fn new() -> Self {
        Self {
            slots: [EMPTY_SLOT; CHANNEL_COUNT],
            unexpected_count: 0,
            last_unexpected_index: 0,
        }
    }

    /// Register but do not enable one channel.
    pub fn register(
        &mut self,
        channel: VimChannel,
        class: InterruptClass,
        handler: Handler,
    ) -> Result<(), VimError> {
        let slot = &mut self.slots[usize::from(channel.get())];
        if slot.handler.is_some() {
            return Err(VimError::AlreadyRegistered);
        }
        slot.handler = Some(handler);
        slot.class = class;
        Ok(())
    }

    /// Enable a previously registered channel; unregistered channels fail.
    pub fn enable(&mut self, channel: VimChannel) -> bool {
        let slot = &mut self.slots[usize::from(channel.get())];
        if slot.handler.is_none() {
            return false;
        }
        slot.enabled = true;
        true
    }

    /// Disable a channel without dropping its registration.
    pub fn disable(&mut self, channel: VimChannel) {
        self.slots[usize::from(channel.get())].enabled = false;
    }

    /// Selected exception class for a registered channel.
    pub fn class(&self, channel: VimChannel) -> Option<InterruptClass> {
        self.slots[usize::from(channel.get())]
            .handler
            .map(|_| self.slots[usize::from(channel.get())].class)
    }

    /// Decode IRQINDEX/FIQINDEX and invoke one bounded handler.
    pub fn dispatch_index(&mut self, raw_index: u32) -> DispatchOutcome {
        if raw_index == 0 {
            return DispatchOutcome::NoPending;
        }
        let zero_based = raw_index - 1;
        let Some(slot) = usize::try_from(zero_based)
            .ok()
            .and_then(|index| self.slots.get(index))
        else {
            return self.capture_default(raw_index);
        };
        let Some(handler) = slot.handler.filter(|_| slot.enabled) else {
            return self.capture_default(raw_index);
        };
        handler();
        DispatchOutcome::Handled(VimChannel(zero_based as u8))
    }

    /// Number of requests routed to the bounded default path.
    pub const fn unexpected_count(&self) -> u32 {
        self.unexpected_count
    }

    /// Most recent raw index routed to the default path.
    pub const fn last_unexpected_index(&self) -> u32 {
        self.last_unexpected_index
    }

    fn capture_default(&mut self, raw_index: u32) -> DispatchOutcome {
        self.unexpected_count = self.unexpected_count.saturating_add(1);
        self.last_unexpected_index = raw_index;
        DispatchOutcome::Defaulted { raw_index }
    }
}

impl Default for VimTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Single-owner VIM adapter. Hardware writes are added only while CPU
/// interrupts are masked by the caller's owned critical section.
pub struct VimController {
    _token: board::Vim,
    table: VimTable,
}

impl VimController {
    /// Bind the unique board token without enabling any request.
    pub const fn from_token(token: board::Vim) -> Self {
        Self {
            _token: token,
            table: VimTable::new(),
        }
    }

    /// Mutable access used by the startup/exception adapter while interrupts
    /// are masked. Applications receive narrower registration APIs later.
    pub fn table_mut(&mut self) -> &mut VimTable {
        &mut self.table
    }
}

const _: () = {
    assert!(CHANNEL_COUNT == 128);
    assert!(FIRST_REGISTERABLE_CHANNEL == 2);
};

#[cfg(test)]
mod tests {
    use core::sync::atomic::{AtomicU32, Ordering};

    use super::*;

    static CALLS: AtomicU32 = AtomicU32::new(0);

    fn handler() {
        CALLS.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn index_encoding_and_reserved_channels_are_exact() {
        assert_eq!(VimChannel::new(0), Err(VimError::ReservedChannel));
        assert_eq!(VimChannel::new(1), Err(VimError::ReservedChannel));
        let rti_compare0 = VimChannel::new(2).unwrap();
        assert_eq!(rti_compare0.get(), 2);
        assert_eq!(rti_compare0.index(), 3);
        assert_eq!(rti_compare0.vector_word(), 3);
        assert_eq!(VimChannel::new(127).unwrap().index(), 128);
    }

    #[test]
    fn registration_enable_dispatch_and_duplicate_rejection_are_bounded() {
        CALLS.store(0, Ordering::Relaxed);
        let channel = VimChannel::new(2).unwrap();
        let mut table = VimTable::new();
        table
            .register(channel, InterruptClass::Irq, handler)
            .unwrap();
        assert_eq!(table.class(channel), Some(InterruptClass::Irq));
        assert_eq!(
            table.dispatch_index(3),
            DispatchOutcome::Defaulted { raw_index: 3 }
        );
        assert!(table.enable(channel));
        assert_eq!(table.dispatch_index(3), DispatchOutcome::Handled(channel));
        assert_eq!(CALLS.load(Ordering::Relaxed), 1);
        assert_eq!(
            table.register(channel, InterruptClass::Fiq, handler),
            Err(VimError::AlreadyRegistered)
        );
    }

    #[test]
    fn no_pending_and_unexpected_indexes_remain_distinct() {
        let mut table = VimTable::new();
        assert_eq!(table.dispatch_index(0), DispatchOutcome::NoPending);
        assert_eq!(
            table.dispatch_index(129),
            DispatchOutcome::Defaulted { raw_index: 129 }
        );
        assert_eq!(table.unexpected_count(), 1);
        assert_eq!(table.last_unexpected_index(), 129);
    }
}
