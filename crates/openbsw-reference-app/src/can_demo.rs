//! Injected-clock CAN counter and upstream-shaped echo listener.

use bsw_can::{CanFrame, CanId};
use bsw_time::{Duration, Instant};

const COUNTER_PERIOD: Duration = Duration::from_nanos(1_000_000_000);

pub struct CanDemo {
    next: Instant,
    counter: u32,
}

impl CanDemo {
    pub const fn new(start: Instant) -> Self {
        Self {
            next: start,
            counter: 0,
        }
    }

    /// Emit ID 0x558 on a drift-free one-second grid.
    pub fn poll_counter(&mut self, now: Instant) -> Option<CanFrame> {
        if !now.is_at_or_after(self.next) {
            return None;
        }
        let frame = CanFrame::with_data(CanId::base(0x558), &self.counter.to_be_bytes());
        self.counter = self.counter.wrapping_add(1);
        self.next = self.next.wrapping_add(COUNTER_PERIOD);
        while now.is_at_or_after(self.next) {
            self.next = self.next.wrapping_add(COUNTER_PERIOD);
        }
        Some(frame)
    }

    /// Echo the upstream demo's 0x123/0x124 inputs as ID+1.
    pub fn echo(frame: &CanFrame) -> Option<CanFrame> {
        let id = frame.id().raw_id();
        matches!(id, 0x123 | 0x124)
            .then(|| CanFrame::with_data(CanId::base((id + 1) as u16), frame.payload()))
    }
}
