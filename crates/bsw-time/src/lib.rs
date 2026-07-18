//! Monotonic time values, clocks, and a fixed-capacity timer queue.

#![cfg_attr(not(feature = "std"), no_std)]

/// Unsigned duration represented as integer nanoseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Duration(u64);

impl Duration {
    /// Zero duration.
    pub const ZERO: Self = Self(0);

    /// Construct from nanoseconds.
    pub const fn from_nanos(value: u64) -> Self {
        Self(value)
    }

    /// Construct from microseconds, returning `None` on overflow.
    pub const fn from_micros(value: u64) -> Option<Self> {
        match value.checked_mul(1_000) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Construct from milliseconds, returning `None` on overflow.
    pub const fn from_millis(value: u64) -> Option<Self> {
        match value.checked_mul(1_000_000) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Construct from seconds, returning `None` on overflow.
    pub const fn from_secs(value: u64) -> Option<Self> {
        match value.checked_mul(1_000_000_000) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Return nanoseconds.
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// Return whole microseconds.
    pub const fn as_micros(self) -> u64 {
        self.0 / 1_000
    }

    /// Return whole milliseconds.
    pub const fn as_millis(self) -> u64 {
        self.0 / 1_000_000
    }
}

/// Wrapping monotonic timestamp represented as nanoseconds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Instant(u64);

impl Instant {
    /// Construct from the raw wrapping tick value.
    pub const fn from_nanos(value: u64) -> Self {
        Self(value)
    }

    /// Return the raw tick value.
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// Add a duration using wrapping monotonic arithmetic.
    #[must_use]
    pub const fn wrapping_add(self, duration: Duration) -> Self {
        Self(self.0.wrapping_add(duration.0))
    }

    /// Elapsed duration from `earlier` to `self` using wrapping arithmetic.
    pub const fn duration_since(self, earlier: Self) -> Duration {
        Duration(self.0.wrapping_sub(earlier.0))
    }

    /// Return true when `self` is at or after `deadline` within half the clock range.
    pub const fn is_at_or_after(self, deadline: Self) -> bool {
        self.0.wrapping_sub(deadline.0) < (1_u64 << 63)
    }
}

/// Source of monotonic time.
pub trait Clock {
    /// Current wrapping monotonic instant.
    fn now(&self) -> Instant;
}

impl<C: Clock + ?Sized> Clock for &C {
    fn now(&self) -> Instant {
        (**self).now()
    }
}

/// Deterministic clock for tests and simulations.
#[derive(Debug, Clone, Copy, Default)]
pub struct FakeClock {
    now: Instant,
}

impl FakeClock {
    /// Create a clock at a specified instant.
    pub const fn new(now: Instant) -> Self {
        Self { now }
    }

    /// Advance the clock using wrapping arithmetic.
    pub fn advance(&mut self, duration: Duration) {
        self.now = self.now.wrapping_add(duration);
    }

    /// Set the current time explicitly.
    pub fn set(&mut self, now: Instant) {
        self.now = now;
    }
}

impl Clock for FakeClock {
    fn now(&self) -> Instant {
        self.now
    }
}

/// Monotonic wall-process clock backed by [`std::time::Instant`].
///
/// The reported [`Instant`] is the elapsed time since this clock was
/// constructed, so independent clocks are not comparable with each other.
#[cfg(feature = "std")]
#[derive(Debug, Clone)]
pub struct StdClock {
    origin: std::time::Instant,
}

#[cfg(feature = "std")]
impl StdClock {
    /// Create a clock whose zero instant is the moment of construction.
    #[must_use]
    pub fn new() -> Self {
        Self {
            origin: std::time::Instant::now(),
        }
    }
}

#[cfg(feature = "std")]
impl Default for StdClock {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "std")]
impl Clock for StdClock {
    fn now(&self) -> Instant {
        let nanos = self.origin.elapsed().as_nanos();
        Instant::from_nanos(nanos as u64)
    }
}

/// Stable handle identifying one timer registration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerHandle {
    slot: u16,
    generation: u16,
}

/// Timer registration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerError {
    /// Queue has no free slot.
    Full,
    /// A periodic timer was requested with a zero period.
    ZeroPeriod,
    /// Capacity cannot be represented by a timer handle.
    CapacityTooLarge,
}

/// Event returned for an expired timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerEvent {
    /// Application-defined timer identifier.
    pub id: u16,
    /// Scheduled deadline that caused this event.
    pub deadline: Instant,
}

#[derive(Debug, Clone, Copy)]
struct Timer {
    id: u16,
    deadline: Instant,
    period: Option<Duration>,
    generation: u16,
    sequence: u64,
}

/// Fixed-capacity timer queue with deterministic deadline/registration ordering.
pub struct TimerQueue<const N: usize> {
    timers: [Option<Timer>; N],
    generations: [u16; N],
    next_sequence: u64,
}

impl<const N: usize> TimerQueue<N> {
    /// Create an empty queue.
    pub const fn new() -> Self {
        Self {
            timers: [None; N],
            generations: [0; N],
            next_sequence: 0,
        }
    }

    /// Register a one-shot timer.
    pub fn register(&mut self, id: u16, deadline: Instant) -> Result<TimerHandle, TimerError> {
        self.register_inner(id, deadline, None)
    }

    /// Register a periodic timer.
    pub fn register_periodic(
        &mut self,
        id: u16,
        deadline: Instant,
        period: Duration,
    ) -> Result<TimerHandle, TimerError> {
        if period == Duration::ZERO {
            return Err(TimerError::ZeroPeriod);
        }
        self.register_inner(id, deadline, Some(period))
    }

    fn register_inner(
        &mut self,
        id: u16,
        deadline: Instant,
        period: Option<Duration>,
    ) -> Result<TimerHandle, TimerError> {
        let slot = self
            .timers
            .iter()
            .position(Option::is_none)
            .ok_or(TimerError::Full)?;
        let slot_u16 = u16::try_from(slot).map_err(|_| TimerError::CapacityTooLarge)?;
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        let generation = self.generations[slot];
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        self.timers[slot] = Some(Timer {
            id,
            deadline,
            period,
            generation,
            sequence,
        });
        Ok(TimerHandle {
            slot: slot_u16,
            generation,
        })
    }

    /// Cancel a live registration. Stale handles return `false`.
    pub fn cancel(&mut self, handle: TimerHandle) -> bool {
        let Some(slot) = self.timers.get_mut(usize::from(handle.slot)) else {
            return false;
        };
        if slot
            .as_ref()
            .is_some_and(|timer| timer.generation == handle.generation)
        {
            *slot = None;
            true
        } else {
            false
        }
    }

    /// Return the next due event, ordered by deadline then registration order.
    pub fn poll(&mut self, now: Instant) -> Option<TimerEvent> {
        let selected = self
            .timers
            .iter()
            .enumerate()
            .filter_map(|(index, timer)| timer.map(|timer| (index, timer)))
            .filter(|(_, timer)| now.is_at_or_after(timer.deadline))
            .max_by_key(|(_, timer)| {
                (
                    now.duration_since(timer.deadline).as_nanos(),
                    u64::MAX - timer.sequence,
                )
            });
        let (index, timer) = selected?;
        if let Some(period) = timer.period {
            let mut next = timer.deadline.wrapping_add(period);
            while now.is_at_or_after(next) {
                next = next.wrapping_add(period);
            }
            self.timers[index].as_mut().unwrap().deadline = next;
        } else {
            self.timers[index] = None;
        }
        Some(TimerEvent {
            id: timer.id,
            deadline: timer.deadline,
        })
    }

    /// Return the deadline of the next timer relative to `now`.
    ///
    /// A timer that is already due reports `now` itself, so callers can use
    /// the result directly as a wait bound without re-checking due-ness.
    pub fn next_deadline(&self, now: Instant) -> Option<Instant> {
        let mut best: Option<(u64, Instant)> = None;
        for timer in self.timers.iter().flatten() {
            if now.is_at_or_after(timer.deadline) {
                return Some(now);
            }
            let wait = timer.deadline.duration_since(now).as_nanos();
            if best.is_none_or(|(current, _)| wait < current) {
                best = Some((wait, timer.deadline));
            }
        }
        best.map(|(_, deadline)| deadline)
    }

    /// Number of active registrations.
    pub fn len(&self) -> usize {
        self.timers.iter().filter(|timer| timer.is_some()).count()
    }

    /// Whether the queue has no active registrations.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Default for TimerQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: Duration = Duration::from_nanos(1_000_000);

    #[test]
    fn checked_integer_conversions() {
        assert_eq!(Duration::from_millis(7).unwrap().as_micros(), 7_000);
        assert_eq!(Duration::from_secs(u64::MAX), None);
    }

    #[test]
    fn wraparound_deadline_is_ordered() {
        let start = Instant::from_nanos(u64::MAX - 3);
        let deadline = start.wrapping_add(Duration::from_nanos(8));
        assert!(!start.is_at_or_after(deadline));
        assert!(Instant::from_nanos(5).is_at_or_after(deadline));
    }

    #[test]
    fn queue_orders_deadline_then_registration() {
        let mut queue: TimerQueue<4> = TimerQueue::new();
        queue.register(2, Instant::from_nanos(20)).unwrap();
        queue.register(1, Instant::from_nanos(10)).unwrap();
        queue.register(3, Instant::from_nanos(10)).unwrap();
        let now = Instant::from_nanos(20);
        assert_eq!(queue.poll(now).unwrap().id, 1);
        assert_eq!(queue.poll(now).unwrap().id, 3);
        assert_eq!(queue.poll(now).unwrap().id, 2);
    }

    #[test]
    fn cancellation_rejects_stale_handle() {
        let mut queue: TimerQueue<1> = TimerQueue::new();
        let old = queue.register(1, Instant::from_nanos(1)).unwrap();
        assert!(queue.cancel(old));
        let current = queue.register(2, Instant::from_nanos(2)).unwrap();
        assert!(!queue.cancel(old));
        assert!(queue.cancel(current));
    }

    #[test]
    fn periodic_timer_skips_missed_periods() {
        let mut queue: TimerQueue<1> = TimerQueue::new();
        queue
            .register_periodic(7, Instant::from_nanos(10), MS)
            .unwrap();
        let now = Instant::from_nanos(3_000_010);
        assert_eq!(queue.poll(now).unwrap().deadline, Instant::from_nanos(10));
        assert!(queue.poll(now).is_none());
        assert_eq!(queue.poll(Instant::from_nanos(4_000_010)).unwrap().id, 7);
    }

    #[test]
    fn next_deadline_reports_due_and_future_timers() {
        let mut queue: TimerQueue<3> = TimerQueue::new();
        assert_eq!(queue.next_deadline(Instant::from_nanos(0)), None);
        queue.register(1, Instant::from_nanos(50)).unwrap();
        queue.register(2, Instant::from_nanos(30)).unwrap();
        assert_eq!(
            queue.next_deadline(Instant::from_nanos(10)),
            Some(Instant::from_nanos(30))
        );
        assert_eq!(
            queue.next_deadline(Instant::from_nanos(30)),
            Some(Instant::from_nanos(30))
        );
        assert_eq!(
            queue.next_deadline(Instant::from_nanos(40)),
            Some(Instant::from_nanos(40))
        );
    }

    #[test]
    fn next_deadline_orders_across_wraparound() {
        let mut queue: TimerQueue<2> = TimerQueue::new();
        let now = Instant::from_nanos(u64::MAX - 5);
        queue.register(1, Instant::from_nanos(3)).unwrap();
        queue.register(2, Instant::from_nanos(10)).unwrap();
        assert_eq!(queue.next_deadline(now), Some(Instant::from_nanos(3)));
    }

    #[cfg(feature = "std")]
    #[test]
    fn std_clock_is_monotonic() {
        let clock = StdClock::new();
        let first = clock.now();
        let second = clock.now();
        assert!(second.is_at_or_after(first));
    }

    #[test]
    fn capacity_and_zero_period_are_reported() {
        let mut queue: TimerQueue<1> = TimerQueue::new();
        assert_eq!(
            queue.register_periodic(1, Instant::from_nanos(0), Duration::ZERO),
            Err(TimerError::ZeroPeriod)
        );
        queue.register(1, Instant::from_nanos(0)).unwrap();
        assert_eq!(
            queue.register(2, Instant::from_nanos(0)),
            Err(TimerError::Full)
        );
    }
}
