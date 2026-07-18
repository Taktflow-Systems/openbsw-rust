//! Shared interrupt-to-task CAN queue used by both STM32 drivers (G03/G04).
//!
//! One ISR producer and one task/polling consumer share an atomically
//! published fixed ring. The same queue backs interrupt and polling modes;
//! hardware drivers only differ in how they obtain a frame.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use bsw_can::{CanFrame, FrameSource};

/// Fixed SPSC queue. One slot is reserved to distinguish full from empty.
pub struct InterruptQueue<const N: usize> {
    frames: UnsafeCell<[CanFrame; N]>,
    head: AtomicUsize,
    tail: AtomicUsize,
    dropped: AtomicU32,
}

// SAFETY: the contract permits exactly one producer and one consumer. Slot
// ownership is transferred by release/acquire cursor publication.
unsafe impl<const N: usize> Sync for InterruptQueue<N> {}

impl<const N: usize> InterruptQueue<N> {
    pub const fn new() -> Self {
        const EMPTY: CanFrame = CanFrame::new();
        assert!(N >= 2, "CAN interrupt queue needs at least two slots");
        Self {
            frames: UnsafeCell::new([EMPTY; N]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
            dropped: AtomicU32::new(0),
        }
    }

    /// Publish one frame from the sole ISR/polling producer.
    pub fn push(&self, frame: CanFrame) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        let next = (head + 1) % N;
        if next == self.tail.load(Ordering::Acquire) {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        // SAFETY: `head` belongs exclusively to the producer until the
        // release store below publishes it.
        unsafe { (*self.frames.get())[head] = frame };
        self.head.store(next, Ordering::Release);
        true
    }

    /// Remove one frame from the sole task consumer.
    pub fn pop(&self) -> Option<CanFrame> {
        let tail = self.tail.load(Ordering::Relaxed);
        if tail == self.head.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: `tail` belongs exclusively to the consumer until it is
        // released below; cloning leaves the reusable backing slot valid.
        let frame = unsafe { (*self.frames.get())[tail].clone() };
        self.tail.store((tail + 1) % N, Ordering::Release);
        Some(frame)
    }

    pub fn len(&self) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        (head + N - tail) % N
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn dropped(&self) -> u32 {
        self.dropped.load(Ordering::Relaxed)
    }
}

impl<const N: usize> Default for InterruptQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FrameSource for InterruptQueue<N> {
    fn receive(&mut self) -> Option<CanFrame> {
        self.pop()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_can::CanId;

    fn frame(id: u16) -> CanFrame {
        CanFrame::with_data(CanId::base(id), &[id as u8])
    }

    #[test]
    fn fifo_wrap_capacity_and_overflow_accounting() {
        let queue: InterruptQueue<4> = InterruptQueue::new();
        assert!(queue.push(frame(1)));
        assert!(queue.push(frame(2)));
        assert!(queue.push(frame(3)));
        assert!(!queue.push(frame(4)));
        assert_eq!(queue.dropped(), 1);
        assert_eq!(queue.pop().unwrap().id().raw_id(), 1);
        assert!(queue.push(frame(4)));
        assert_eq!(queue.pop().unwrap().id().raw_id(), 2);
        assert_eq!(queue.pop().unwrap().id().raw_id(), 3);
        assert_eq!(queue.pop().unwrap().id().raw_id(), 4);
        assert!(queue.is_empty());
    }

    #[test]
    fn common_frame_source_contract_drains_queue() {
        let mut queue: InterruptQueue<2> = InterruptQueue::new();
        queue.push(frame(7));
        assert_eq!(FrameSource::receive(&mut queue).unwrap().id().raw_id(), 7);
    }
}
