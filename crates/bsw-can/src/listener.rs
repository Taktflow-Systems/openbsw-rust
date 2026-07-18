//! CAN frame listener management (package D13).
//!
//! Ports the upstream `cpp2can` listener model — `ICANFrameListener`,
//! `IFilteredCANFrameSentListener`, and `ICANTransceiverStateListener` —
//! with native Rust ownership:
//!
//! - upstream chains listeners through intrusive `etl` links; here a
//!   fixed-capacity [`FrameDispatcher`] owns `&mut` registrations, so a
//!   listener can never dangle or be registered twice;
//! - upstream listeners mutate the list during callbacks; here a callback
//!   returns [`ListenerAction`] to request its own removal, which the
//!   dispatcher applies after the call — reentrancy without aliasing.
//!
//! Dispatch order is registration order, matching the upstream intrusive
//! list append semantics.

use crate::frame::CanFrame;
use crate::transceiver::TransceiverState;

/// Non-blocking source of received CAN frames.
///
/// Hardware polling, interrupt-backed queues, SocketCAN, and virtual buses
/// implement this same contract; protocol adapters must not invent a
/// BSP-local receive trait.
pub trait FrameSource {
    /// Remove and return the oldest received frame, or `None` when empty.
    fn receive(&mut self) -> Option<CanFrame>;
}

/// Decision returned by a listener callback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ListenerAction {
    /// Stay registered.
    Keep,
    /// Deregister this listener; it will not be called again.
    Remove,
}

/// Receiver of frames accepted by the listener's own filter.
///
/// Mirror of upstream `ICANFrameListener` (receive path) and
/// `IFilteredCANFrameSentListener` (transmit-confirmation path); one trait
/// serves both directions because the shape is identical.
pub trait FrameListener {
    /// Whether this listener wants the frame. Upstream expresses this with
    /// an owned `IFilter`; a match predicate keeps the storage layout to
    /// the listener.
    fn matches(&self, raw_id: u32) -> bool {
        let _ = raw_id;
        true
    }

    /// Deliver one frame.
    fn on_frame(&mut self, frame: &CanFrame) -> ListenerAction;
}

/// Observer of transceiver error-state changes.
///
/// Mirror of upstream `ICANTransceiverStateListener`.
pub trait StateListener {
    /// The hardware error state changed.
    fn state_changed(&mut self, state: TransceiverState);

    /// A physical-layer error was detected.
    fn phy_error(&mut self);
}

/// Registration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ListenerError {
    /// No listener slot left; upstream `NoMoreListenersPossible`.
    Full,
}

/// Stable handle for one registration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ListenerHandle {
    slot: u16,
    generation: u16,
}

struct Slot<'a> {
    listener: &'a mut dyn FrameListener,
    generation: u16,
}

/// Fixed-capacity, order-preserving frame dispatcher.
pub struct FrameDispatcher<'a, const N: usize> {
    slots: [Option<Slot<'a>>; N],
    generations: [u16; N],
}

impl<'a, const N: usize> FrameDispatcher<'a, N> {
    /// Create an empty dispatcher.
    pub fn new() -> Self {
        Self {
            slots: [const { None }; N],
            generations: [0; N],
        }
    }

    /// Register a listener. Dispatch order is registration order.
    pub fn add(
        &mut self,
        listener: &'a mut dyn FrameListener,
    ) -> Result<ListenerHandle, ListenerError> {
        let slot = self
            .slots
            .iter()
            .position(Option::is_none)
            .ok_or(ListenerError::Full)?;
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        let generation = self.generations[slot];
        self.slots[slot] = Some(Slot {
            listener,
            generation,
        });
        Ok(ListenerHandle {
            slot: slot as u16,
            generation,
        })
    }

    /// Deregister a listener. Stale handles return `false`.
    pub fn remove(&mut self, handle: ListenerHandle) -> bool {
        let Some(slot) = self.slots.get_mut(usize::from(handle.slot)) else {
            return false;
        };
        if slot
            .as_ref()
            .is_some_and(|entry| entry.generation == handle.generation)
        {
            *slot = None;
            true
        } else {
            false
        }
    }

    /// Number of registered listeners.
    pub fn len(&self) -> usize {
        self.slots.iter().filter(|slot| slot.is_some()).count()
    }

    /// Whether no listener is registered.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Deliver `frame` to every matching listener in registration order.
    ///
    /// Returns the number of listeners that received the frame. Listeners
    /// returning [`ListenerAction::Remove`] are deregistered after their
    /// callback, so self-removal during dispatch is safe and later
    /// listeners still run.
    pub fn dispatch(&mut self, frame: &CanFrame) -> usize {
        let raw_id = frame.id().raw_id();
        let mut delivered = 0;
        for slot in &mut self.slots {
            let Some(entry) = slot.as_mut() else {
                continue;
            };
            if !entry.listener.matches(raw_id) {
                continue;
            }
            delivered += 1;
            if entry.listener.on_frame(frame) == ListenerAction::Remove {
                *slot = None;
            }
        }
        delivered
    }
}

impl<const N: usize> Default for FrameDispatcher<'_, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_id::CanId;
    use crate::filter::{Filter, IntervalFilter};

    struct Probe {
        accept_from: u32,
        accept_to: u32,
        received: u32,
        remove_after: Option<u32>,
    }

    impl Probe {
        fn open() -> Self {
            Self {
                accept_from: 0,
                accept_to: u32::MAX,
                received: 0,
                remove_after: None,
            }
        }
    }

    impl FrameListener for Probe {
        fn matches(&self, raw_id: u32) -> bool {
            // Reuse the crate's interval filter as upstream listeners do.
            IntervalFilter::with_range(self.accept_from, self.accept_to).matches(raw_id)
        }

        fn on_frame(&mut self, _frame: &CanFrame) -> ListenerAction {
            self.received += 1;
            if self
                .remove_after
                .is_some_and(|limit| self.received >= limit)
            {
                ListenerAction::Remove
            } else {
                ListenerAction::Keep
            }
        }
    }

    fn frame(raw_id: u16) -> CanFrame {
        CanFrame::with_id(CanId::base(raw_id))
    }

    #[test]
    fn dispatch_delivers_in_registration_order_with_filters() {
        let mut all = Probe::open();
        let mut narrow = Probe {
            accept_from: 0x100,
            accept_to: 0x1FF,
            ..Probe::open()
        };
        {
            let mut dispatcher: FrameDispatcher<'_, 4> = FrameDispatcher::new();
            dispatcher.add(&mut all).unwrap();
            dispatcher.add(&mut narrow).unwrap();
            assert_eq!(dispatcher.dispatch(&frame(0x100)), 2);
            assert_eq!(dispatcher.dispatch(&frame(0x300)), 1);
        }
        assert_eq!(all.received, 2);
        assert_eq!(narrow.received, 1);
    }

    #[test]
    fn capacity_is_reported_as_full() {
        let mut first = Probe::open();
        let mut second = Probe::open();
        let mut dispatcher: FrameDispatcher<'_, 1> = FrameDispatcher::new();
        dispatcher.add(&mut first).unwrap();
        assert_eq!(dispatcher.add(&mut second), Err(ListenerError::Full));
    }

    #[test]
    fn removal_by_handle_and_stale_handles() {
        let mut listener = Probe::open();
        let mut replacement = Probe::open();
        let mut dispatcher: FrameDispatcher<'_, 1> = FrameDispatcher::new();
        let handle = dispatcher.add(&mut listener).unwrap();
        assert!(dispatcher.remove(handle));
        assert!(!dispatcher.remove(handle), "stale handle rejected");
        let replacement_handle = dispatcher.add(&mut replacement).unwrap();
        assert!(
            !dispatcher.remove(handle),
            "old handle must not remove the new listener"
        );
        assert!(dispatcher.remove(replacement_handle));
    }

    #[test]
    fn reentrant_self_removal_keeps_later_listeners_running() {
        let mut once = Probe {
            remove_after: Some(1),
            ..Probe::open()
        };
        let mut steady = Probe::open();
        {
            let mut dispatcher: FrameDispatcher<'_, 2> = FrameDispatcher::new();
            dispatcher.add(&mut once).unwrap();
            dispatcher.add(&mut steady).unwrap();
            assert_eq!(dispatcher.dispatch(&frame(1)), 2);
            assert_eq!(dispatcher.len(), 1, "self-removed after first frame");
            assert_eq!(dispatcher.dispatch(&frame(2)), 1);
        }
        assert_eq!(once.received, 1);
        assert_eq!(steady.received, 2);
    }

    #[test]
    fn slots_are_reused_after_removal() {
        let mut a = Probe::open();
        let mut b = Probe::open();
        let mut dispatcher: FrameDispatcher<'_, 1> = FrameDispatcher::new();
        let handle = dispatcher.add(&mut a).unwrap();
        dispatcher.remove(handle);
        assert!(dispatcher.is_empty());
        dispatcher.add(&mut b).unwrap();
        assert_eq!(dispatcher.len(), 1);
    }
}
