//! Lock-free single-producer single-consumer (SPSC) queue.
//!
//! This is a `no_std`-compatible ring buffer port of the C++ `util/spsc/Queue`
//! from OpenBSW.  It uses atomics for thread-safety without any locks and
//! avoids heap allocation entirely.
//!
//! # Ring buffer semantics
//!
//! The backing buffer has `N` slots but only `N - 1` are usable.  The spare
//! slot lets the implementation distinguish "full" (`(head + 1) % N == tail`)
//! from "empty" (`head == tail`) without a separate counter.
//!
//! # Atomic ordering
//!
//! | Operation | Ordering |
//! |-----------|----------|
//! | Producer stores `head` | `Release` — makes written data visible to consumer |
//! | Consumer loads `head`  | `Acquire` — sees producer's data |
//! | Consumer stores `tail` | `Release` — makes freed slot visible to producer |
//! | Producer loads `tail`  | `Acquire` — sees consumer's freed slots |
//! | `len` / `is_empty` / `is_full` queries | `Relaxed` — best-effort snapshot |

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicUsize, Ordering};

// ── Queue ──────────────────────────────────────────────────────────────────

/// A fixed-capacity lock-free ring buffer for SPSC use.
///
/// `N` is the *backing-buffer* size; the usable capacity is `N - 1`.
/// Use [`Queue::split`] to obtain a [`Producer`] / [`Consumer`] pair.
pub struct Queue<T, const N: usize> {
    buffer: [UnsafeCell<MaybeUninit<T>>; N],
    /// Write cursor — only the producer advances this.
    head: AtomicUsize,
    /// Read cursor — only the consumer advances this.
    tail: AtomicUsize,
}

// SAFETY: `Queue` is designed for single-producer single-consumer access.
// The producer only writes `head`; the consumer only writes `tail`.  Both
// may read either index.  Atomic ordering (Release/Acquire pairs) ensures
// all memory accesses are properly visible across threads.  `T: Send` is
// required because values are transferred between threads.
unsafe impl<T: Send, const N: usize> Send for Queue<T, N> {}
// SAFETY: Same reasoning as `Send`.  Shared references are never handed out
// to the underlying buffer slots; only the producer/consumer handles expose
// operations, and those are `'_`-lifetime borrows from `&mut self` (split),
// preventing misuse at the type level.
unsafe impl<T: Send, const N: usize> Sync for Queue<T, N> {}

impl<T, const N: usize> Queue<T, N> {
    /// Create a new, empty queue.
    ///
    /// `N` must be `>= 2` (usable capacity = `N - 1`).  A compile-time
    /// assertion enforces this.
    #[must_use]
    pub const fn new() -> Self {
        // Compile-time assertion: N must be at least 2.
        const { assert!(N >= 2, "Queue<T, N>: N must be >= 2 (capacity = N - 1)") }

        // SAFETY: `MaybeUninit<T>` does not require initialisation; this is
        // the canonical way to create an array of `UnsafeCell<MaybeUninit<T>>`
        // without requiring `T: Copy` or allocating on the heap.
        // `UnsafeCell` is `repr(transparent)` so the bit-pattern of an
        // uninitialised `MaybeUninit<T>` is a valid `UnsafeCell<MaybeUninit<T>>`.
        let buffer = unsafe {
            MaybeUninit::<[UnsafeCell<MaybeUninit<T>>; N]>::uninit().assume_init()
        };

        Self {
            buffer,
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Maximum number of elements the queue can hold (`N - 1`).
    #[inline]
    pub const fn capacity() -> usize {
        N - 1
    }

    /// Current number of elements in the queue (best-effort snapshot).
    #[inline]
    pub fn len(&self) -> usize {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        head.wrapping_sub(tail).wrapping_add(N) % N
    }

    /// Whether the queue contains no elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.head.load(Ordering::Relaxed) == self.tail.load(Ordering::Relaxed)
    }

    /// Whether the queue is at capacity and no more elements can be pushed.
    #[inline]
    pub fn is_full(&self) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Relaxed);
        (head + 1) % N == tail
    }

    /// Split the queue into a [`Producer`] and a [`Consumer`] handle.
    ///
    /// Both handles borrow `self` mutably (via `&mut self`) ensuring that at
    /// most one producer and one consumer exist at any time — statically
    /// enforced by the Rust borrow checker through the `'_` lifetime.
    #[inline]
    pub fn split(&mut self) -> (Producer<'_, T, N>, Consumer<'_, T, N>) {
        (Producer { queue: self }, Consumer { queue: self })
    }
}

impl<T, const N: usize> Default for Queue<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> Drop for Queue<T, N> {
    fn drop(&mut self) {
        // Drop every element that has been pushed but not yet popped.
        let mut tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Relaxed);
        while tail != head {
            // SAFETY: The slot at `tail` contains a validly initialised `T`
            // because head has moved past it (producer wrote it) but tail has
            // not yet consumed it.  We are in `drop`, so there can be no
            // concurrent access.
            unsafe {
                (*self.buffer[tail].get()).assume_init_drop();
            }
            tail = (tail + 1) % N;
        }
    }
}

// ── Producer ──────────────────────────────────────────────────────────────

/// The write-end of a [`Queue`].
///
/// Only one `Producer` may exist at a time; this is enforced by the lifetime
/// tied back to the `&mut Queue` from which it was created via [`Queue::split`].
pub struct Producer<'a, T, const N: usize> {
    queue: &'a Queue<T, N>,
}

impl<T, const N: usize> Producer<'_, T, N> {
    /// Attempt to push `value` onto the queue.
    ///
    /// Returns `Err(value)` if the queue is full (non-blocking).
    pub fn try_push(&self, value: T) -> Result<(), T> {
        // Load head with Relaxed — we are the only writer of head, so the
        // value we last stored is still the current value.
        let head = self.queue.head.load(Ordering::Relaxed);
        let next_head = (head + 1) % N;
        // Acquire so we see any tail-advancement the consumer has published.
        let tail = self.queue.tail.load(Ordering::Acquire);
        if next_head == tail {
            return Err(value); // full
        }
        // SAFETY: We are the only producer.  `head` points to an unoccupied
        // slot (the consumer never touches slots in the range tail..head, and
        // head != tail because next_head != tail).  We write before advancing
        // head so the consumer cannot observe this slot until after the
        // Release store below.
        unsafe {
            (*self.queue.buffer[head].get()).write(value);
        }
        // Release: makes the written value and the buffer write above visible
        // to the consumer when it loads head with Acquire.
        self.queue.head.store(next_head, Ordering::Release);
        Ok(())
    }

    /// Returns `true` if the queue is currently full.
    #[inline]
    pub fn is_full(&self) -> bool {
        let head = self.queue.head.load(Ordering::Relaxed);
        let tail = self.queue.tail.load(Ordering::Acquire);
        (head + 1) % N == tail
    }
}

// ── Consumer ──────────────────────────────────────────────────────────────

/// The read-end of a [`Queue`].
///
/// Only one `Consumer` may exist at a time; this is enforced by the lifetime
/// tied back to the `&mut Queue` from which it was created via [`Queue::split`].
pub struct Consumer<'a, T, const N: usize> {
    queue: &'a Queue<T, N>,
}

impl<T, const N: usize> Consumer<'_, T, N> {
    /// Attempt to pop a value from the front of the queue.
    ///
    /// Returns `None` if the queue is empty (non-blocking).
    pub fn try_pop(&self) -> Option<T> {
        // Load tail with Relaxed — we are the only writer of tail.
        let tail = self.queue.tail.load(Ordering::Relaxed);
        // Acquire so we see any data the producer has published.
        let head = self.queue.head.load(Ordering::Acquire);
        if tail == head {
            return None; // empty
        }
        // SAFETY: We are the only consumer.  `tail` points to a slot that has
        // been fully written by the producer (producer stores head with Release
        // after writing; we loaded head with Acquire, establishing the
        // happens-before relationship).  `assume_init_read` moves the value
        // out; we then advance tail so the producer can reuse the slot.
        let value = unsafe { (*self.queue.buffer[tail].get()).assume_init_read() };
        // Release: makes the freed slot visible to the producer.
        self.queue.tail.store((tail + 1) % N, Ordering::Release);
        Some(value)
    }

    /// Peek at the front value without removing it.
    ///
    /// Returns `None` if the queue is empty.
    pub fn peek(&self) -> Option<&T> {
        let tail = self.queue.tail.load(Ordering::Relaxed);
        let head = self.queue.head.load(Ordering::Acquire);
        if tail == head {
            return None; // empty
        }
        // SAFETY: Same as `try_pop`: the slot at `tail` contains a validly
        // initialised `T`.  We return a shared reference tied to `&self` so
        // the caller cannot pop while holding the reference (both require
        // `&self` on the same `Consumer`, and `&T` borrows `self`).
        Some(unsafe { (*self.queue.buffer[tail].get()).assume_init_ref() })
    }

    /// Returns `true` if the queue is currently empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.queue.tail.load(Ordering::Relaxed) == self.queue.head.load(Ordering::Acquire)
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::Queue;
    use core::sync::atomic::{AtomicUsize, Ordering};

    // ── 1. New queue is empty ──────────────────────────────────────────────
    #[test]
    fn new_queue_is_empty() {
        let q: Queue<u32, 4> = Queue::new();
        assert!(q.is_empty());
        assert!(!q.is_full());
        assert_eq!(q.len(), 0);
    }

    // ── 2. Push one, pop one ───────────────────────────────────────────────
    #[test]
    fn push_one_pop_one() {
        let mut q: Queue<u32, 4> = Queue::new();
        {
            let (prod, cons) = q.split();
            assert!(prod.try_push(42).is_ok());
            assert_eq!(cons.try_pop(), Some(42));
        }
        assert!(q.is_empty());
    }

    // ── 3. Push until full returns Err ────────────────────────────────────
    #[test]
    fn push_until_full_returns_err() {
        // N=4 → capacity 3
        let mut q: Queue<u32, 4> = Queue::new();
        let (prod, _cons) = q.split();
        assert!(prod.try_push(1).is_ok());
        assert!(prod.try_push(2).is_ok());
        assert!(prod.try_push(3).is_ok());
        // Queue now full
        assert!(prod.is_full());
        let result = prod.try_push(99);
        assert_eq!(result, Err(99));
    }

    // ── 4. Pop from empty returns None ────────────────────────────────────
    #[test]
    fn pop_from_empty_returns_none() {
        let mut q: Queue<u32, 4> = Queue::new();
        let (_prod, cons) = q.split();
        assert_eq!(cons.try_pop(), None);
        assert!(cons.is_empty());
    }

    // ── 5. FIFO order preserved ───────────────────────────────────────────
    #[test]
    fn fifo_order() {
        let mut q: Queue<u32, 8> = Queue::new();
        let (prod, cons) = q.split();
        for i in 0..7_u32 {
            assert!(prod.try_push(i).is_ok());
        }
        for i in 0..7_u32 {
            assert_eq!(cons.try_pop(), Some(i));
        }
    }

    // ── 6. Fill and drain multiple cycles (wrap-around) ───────────────────
    #[test]
    fn wrap_around_multiple_cycles() {
        // N=4 → capacity 3; cycle 10 times, pushing/popping 3 items each
        let mut q: Queue<u32, 4> = Queue::new();
        let (prod, cons) = q.split();
        let mut counter = 0_u32;
        for _ in 0..10 {
            // fill
            assert!(prod.try_push(counter).is_ok());
            assert!(prod.try_push(counter + 1).is_ok());
            assert!(prod.try_push(counter + 2).is_ok());
            assert!(prod.is_full());
            // drain
            assert_eq!(cons.try_pop(), Some(counter));
            assert_eq!(cons.try_pop(), Some(counter + 1));
            assert_eq!(cons.try_pop(), Some(counter + 2));
            assert!(cons.is_empty());
            counter += 3;
        }
    }

    // ── 7. Peek returns front without removing ────────────────────────────
    #[test]
    fn peek_does_not_remove() {
        let mut q: Queue<u32, 4> = Queue::new();
        {
            let (prod, cons) = q.split();
            prod.try_push(10).ok();
            prod.try_push(20).ok();

            assert_eq!(cons.peek(), Some(&10));
            assert_eq!(cons.peek(), Some(&10)); // still there

            // Verify len while handles are live (via the handles' queue ref)
            assert_eq!(cons.queue.len(), 2);

            // Pop both
            assert_eq!(cons.try_pop(), Some(10));
            assert_eq!(cons.try_pop(), Some(20));
        }
        assert!(q.is_empty());
    }

    // ── 8. len() tracks correctly ─────────────────────────────────────────
    #[test]
    fn len_tracks_correctly() {
        let mut q: Queue<u32, 5> = Queue::new(); // capacity 4

        assert_eq!(q.len(), 0);

        {
            let (prod, cons) = q.split();

            prod.try_push(1).ok();
            assert_eq!(prod.queue.len(), 1);

            prod.try_push(2).ok();
            assert_eq!(prod.queue.len(), 2);

            cons.try_pop();
            assert_eq!(cons.queue.len(), 1);

            cons.try_pop();
            assert_eq!(cons.queue.len(), 0);
        }

        assert_eq!(q.len(), 0);
    }

    // ── 9. capacity() returns N-1 ─────────────────────────────────────────
    #[test]
    fn capacity_is_n_minus_one() {
        assert_eq!(Queue::<u32, 4>::capacity(), 3);
        assert_eq!(Queue::<u8, 2>::capacity(), 1);
        assert_eq!(Queue::<u64, 16>::capacity(), 15);
    }

    // ── 10. Producer is_full / Consumer is_empty ──────────────────────────
    #[test]
    fn producer_is_full_consumer_is_empty() {
        let mut q: Queue<u32, 3> = Queue::new(); // capacity 2
        let (prod, cons) = q.split();

        assert!(cons.is_empty());
        assert!(!prod.is_full());

        prod.try_push(1).ok();
        assert!(!cons.is_empty());
        assert!(!prod.is_full());

        prod.try_push(2).ok();
        assert!(!cons.is_empty());
        assert!(prod.is_full());

        cons.try_pop();
        assert!(!prod.is_full());
    }

    // ── 11. Drop cleans up remaining elements ─────────────────────────────

    /// A value that increments a shared counter when dropped.
    struct DropCounter<'a> {
        count: &'a AtomicUsize,
    }

    impl Drop for DropCounter<'_> {
        fn drop(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[test]
    fn drop_cleans_remaining_elements() {
        let drops = AtomicUsize::new(0);
        {
            let mut q: Queue<DropCounter<'_>, 5> = Queue::new();
            let (prod, cons) = q.split();

            // push_ok is a helper closure that aborts on full rather than unwrap(T: Debug)
            assert!(prod.try_push(DropCounter { count: &drops }).is_ok());
            assert!(prod.try_push(DropCounter { count: &drops }).is_ok());
            assert!(prod.try_push(DropCounter { count: &drops }).is_ok());

            // Pop one — its drop runs here
            let popped = cons.try_pop();
            drop(popped);
            assert_eq!(drops.load(Ordering::Relaxed), 1);

            // End handles' lifetimes so q can be dropped at the end of the block.
            // `let _ = ...` is used because Producer/Consumer have no Drop impl.
            let _ = (prod, cons);
            // q goes out of scope here; its Drop impl must drop the 2 remaining
        }
        assert_eq!(drops.load(Ordering::Relaxed), 3);
    }

    // ── 12. split produces valid producer/consumer ────────────────────────
    #[test]
    fn split_produces_valid_handles() {
        let mut q: Queue<i32, 4> = Queue::new();
        {
            let (prod, cons) = q.split();
            assert!(prod.try_push(7).is_ok());
            assert!(prod.try_push(8).is_ok());
            assert_eq!(cons.try_pop(), Some(7));
            assert_eq!(cons.try_pop(), Some(8));
        }
        // After handles are dropped, queue is accessible again
        assert!(q.is_empty());
    }

    // ── 13. N=2 (minimum, capacity=1) works ──────────────────────────────
    #[test]
    fn minimum_capacity_n2() {
        let mut q: Queue<u32, 2> = Queue::new();
        assert_eq!(Queue::<u32, 2>::capacity(), 1);

        let (prod, cons) = q.split();
        assert!(prod.try_push(99).is_ok());
        assert!(prod.is_full());
        assert_eq!(prod.try_push(100), Err(100));
        assert_eq!(cons.try_pop(), Some(99));
        assert!(cons.is_empty());
    }

    // ── 14. Interleaved push/pop ──────────────────────────────────────────
    #[test]
    fn interleaved_push_pop() {
        // Producer pushes 2, consumer pops 1, repeated 20 times.
        // Net accumulation = 20 items; N=32 (capacity 31) is sufficient.
        let mut q: Queue<u32, 32> = Queue::new();
        let (prod, cons) = q.split();
        let mut pushed = 0_u32;
        let mut popped = 0_u32;
        for _ in 0..20 {
            assert!(prod.try_push(pushed).is_ok());
            pushed += 1;
            assert!(prod.try_push(pushed).is_ok());
            pushed += 1;
            let val = cons.try_pop().expect("queue should not be empty");
            assert_eq!(val, popped);
            popped += 1;
        }
        // Drain remaining
        while let Some(val) = cons.try_pop() {
            assert_eq!(val, popped);
            popped += 1;
        }
        assert_eq!(popped, pushed);
    }
}
