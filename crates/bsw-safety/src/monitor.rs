//! Safe-monitor primitives grounded in upstream `libs/safety/safeMonitor`.

/// Receives a monitor event synchronously in the caller's context.
pub trait EventHandler<E> {
    fn handle(&mut self, event: E);
}

/// Explicit trigger with copied diagnostic context.
pub struct Trigger<E: Copy, C: Copy + Default> {
    event: E,
    context: C,
    latched: bool,
    count: u32,
}

impl<E: Copy, C: Copy + Default> Trigger<E, C> {
    pub fn new(event: E) -> Self {
        Self {
            event,
            context: C::default(),
            latched: false,
            count: 0,
        }
    }

    pub fn trigger<H: EventHandler<E>>(&mut self, handler: &mut H, context: C) {
        self.context = context;
        self.latched = true;
        self.count = self.count.saturating_add(1);
        handler.handle(self.event);
    }

    pub fn reset(&mut self) {
        self.latched = false;
    }
    pub const fn is_latched(&self) -> bool {
        self.latched
    }
    pub const fn trigger_count(&self) -> u32 {
        self.count
    }
    pub const fn context(&self) -> C {
        self.context
    }
}

/// Equality monitor. As upstream, every mismatch is reported.
pub struct ValueMonitor<E: Copy, T: PartialEq + Copy, C: Copy + Default> {
    trigger: Trigger<E, C>,
    expected: T,
}

impl<E: Copy, T: PartialEq + Copy, C: Copy + Default> ValueMonitor<E, T, C> {
    pub fn new(event: E, expected: T) -> Self {
        Self {
            trigger: Trigger::new(event),
            expected,
        }
    }
    pub fn check<H: EventHandler<E>>(&mut self, handler: &mut H, value: T, context: C) -> bool {
        let valid = value == self.expected;
        if valid {
            self.trigger.reset();
        } else {
            self.trigger.trigger(handler, context);
        }
        valid
    }
    pub const fn trigger(&self) -> &Trigger<E, C> {
        &self.trigger
    }
}

/// Ordered checkpoint monitor with explicit integer checkpoint values.
pub struct SequenceMonitor<E: Copy, C: Copy + Default> {
    trigger: Trigger<E, C>,
    first: u16,
    last: u16,
    expected: u16,
}

impl<E: Copy, C: Copy + Default> SequenceMonitor<E, C> {
    pub fn new(event: E, first: u16, last: u16) -> Option<Self> {
        (first <= last).then(|| Self {
            trigger: Trigger::new(event),
            first,
            last,
            expected: first,
        })
    }
    pub fn hit<H: EventHandler<E>>(
        &mut self,
        handler: &mut H,
        checkpoint: u16,
        context: C,
    ) -> bool {
        if checkpoint != self.expected {
            self.trigger.trigger(handler, context);
            return false;
        }
        self.trigger.reset();
        self.expected = if checkpoint == self.last {
            self.first
        } else {
            checkpoint + 1
        };
        true
    }
    pub const fn expected(&self) -> u16 {
        self.expected
    }
    pub const fn trigger(&self) -> &Trigger<E, C> {
        &self.trigger
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegisterEntry<T> {
    pub value: T,
    pub mask: T,
    pub expected: T,
}

/// Safe register-value monitor. MMIO reads stay in the platform driver; this
/// monitor accepts snapshots and therefore contains no raw-pointer access.
pub struct RegisterMonitor<'a, E: Copy, T: Copy, C: Copy + Default> {
    trigger: Trigger<E, C>,
    entries: &'a [RegisterEntry<T>],
    last_checked: Option<usize>,
}

impl<'a, E: Copy, T, C: Copy + Default> RegisterMonitor<'a, E, T, C>
where
    T: Copy + core::ops::BitAnd<Output = T> + PartialEq,
{
    pub fn new(event: E, entries: &'a [RegisterEntry<T>]) -> Self {
        Self {
            trigger: Trigger::new(event),
            entries,
            last_checked: None,
        }
    }
    pub fn check<H: EventHandler<E>>(&mut self, handler: &mut H, context: C) -> bool {
        for (index, entry) in self.entries.iter().enumerate() {
            self.last_checked = Some(index);
            if entry.value & entry.mask != entry.expected {
                self.trigger.trigger(handler, context);
                return false;
            }
        }
        self.trigger.reset();
        true
    }
    pub const fn last_checked(&self) -> Option<usize> {
        self.last_checked
    }
}

/// Cyclic software watchdog. Once zero is reached the upstream behavior emits
/// an event on every service call until kicked.
pub struct WatchdogMonitor<E: Copy, C: Copy + Default> {
    trigger: Trigger<E, C>,
    timeout: u32,
    counter: u32,
}

impl<E: Copy, C: Copy + Default> WatchdogMonitor<E, C> {
    pub fn new(event: E, timeout: u32) -> Option<Self> {
        (timeout > 0).then(|| Self {
            trigger: Trigger::new(event),
            timeout,
            counter: timeout,
        })
    }
    pub fn kick(&mut self, context: C) {
        self.counter = self.timeout;
        self.trigger.context = context;
        self.trigger.reset();
    }
    pub fn service<H: EventHandler<E>>(&mut self, handler: &mut H) -> bool {
        self.counter = self.counter.saturating_sub(1);
        if self.counter == 0 {
            self.trigger.trigger(handler, self.trigger.context);
            false
        } else {
            true
        }
    }
    pub const fn counter(&self) -> u32 {
        self.counter
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Default)]
    struct Handler {
        count: u32,
        last: u8,
    }
    impl EventHandler<u8> for Handler {
        fn handle(&mut self, event: u8) {
            self.count += 1;
            self.last = event;
        }
    }

    #[test]
    fn trigger_latches_resets_and_repeats() {
        let mut h = Handler::default();
        let mut t = Trigger::<_, u16>::new(7);
        t.trigger(&mut h, 11);
        t.trigger(&mut h, 12);
        assert_eq!(
            (h.count, h.last, t.trigger_count(), t.context()),
            (2, 7, 2, 12)
        );
        assert!(t.is_latched());
        t.reset();
        assert!(!t.is_latched());
    }
    #[test]
    fn value_and_sequence_match_upstream_vectors() {
        let mut h = Handler::default();
        let mut v = ValueMonitor::<_, _, u8>::new(1, 42);
        assert!(v.check(&mut h, 42, 1));
        assert!(!v.check(&mut h, 41, 2));
        assert_eq!(h.count, 1);
        let mut s = SequenceMonitor::<_, u8>::new(2, 3, 5).unwrap();
        assert!(s.hit(&mut h, 3, 0));
        assert!(!s.hit(&mut h, 5, 0));
        assert_eq!(s.expected(), 4);
        assert!(s.hit(&mut h, 4, 0));
        assert!(s.hit(&mut h, 5, 0));
        assert_eq!(s.expected(), 3);
    }
    #[test]
    fn register_stops_at_first_mismatch() {
        let entries = [
            RegisterEntry {
                value: 0x12u8,
                mask: 0xf0,
                expected: 0x10,
            },
            RegisterEntry {
                value: 3,
                mask: 3,
                expected: 2,
            },
            RegisterEntry {
                value: 0,
                mask: 0,
                expected: 0,
            },
        ];
        let mut m = RegisterMonitor::<_, _, u8>::new(9, &entries);
        let mut h = Handler::default();
        assert!(!m.check(&mut h, 4));
        assert_eq!(m.last_checked(), Some(1));
        assert_eq!(h.count, 1);
    }
    #[test]
    fn watchdog_repeats_at_zero_until_kicked() {
        let mut h = Handler::default();
        let mut w = WatchdogMonitor::<_, u8>::new(4, 2).unwrap();
        assert!(w.service(&mut h));
        assert!(!w.service(&mut h));
        assert!(!w.service(&mut h));
        assert_eq!(h.count, 2);
        w.kick(8);
        assert_eq!(w.counter(), 2);
        assert!(w.service(&mut h));
    }
}
