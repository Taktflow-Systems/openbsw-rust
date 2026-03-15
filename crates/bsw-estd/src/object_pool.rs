// Copyright 2024 Accenture.
// SPDX-License-Identifier: Apache-2.0

//! Fixed-capacity pool allocator with inline storage and bitfield occupancy tracking.
//!
//! This is a `#![no_std]`-compatible port of OpenBSW's `estd::declare::object_pool<T, N>`.
//!
//! # Design
//!
//! - Objects live in `[MaybeUninit<T>; N]` — no heap allocation.
//! - A packed `[u8; B]` bitfield tracks which slots are occupied (bit = 1 means used).
//!   `B` must equal `(N + 7) / 8`; this invariant is checked with a compile-time assertion.
//! - An `available` counter mirrors the number of free slots for O(1) `size()` / `is_empty()`.
//!
//! # Stable-Rust note on const generics
//!
//! `generic_const_exprs` (allowing expressions like `[u8; (N+7)/8]` in struct definitions) is
//! still nightly-only as of Rust 1.94.  The struct therefore takes a second const generic `B`
//! for the bitfield byte-count.  Use the [`object_pool!`] convenience macro (or the type alias
//! [`PoolN`]) to avoid spelling it out manually.
//!
//! # Naming note (matches C++ convention)
//!
//! `size()` returns the number of **available** (free) slots, *not* the number of acquired ones.
//! This matches the C++ `estd::object_pool::size()` API.  Use `acquired_count()` for the inverse.
//!
//! # Example
//!
//! ```rust
//! # use bsw_estd::object_pool::ObjectPool;
//! // ObjectPool<T, N, B> where B = (N + 7) / 8.
//! let mut pool: ObjectPool<u32, 4, 1> = ObjectPool::new();
//! assert!(pool.is_full());
//!
//! let a_ptr = {
//!     let a = pool.acquire_with(1).unwrap();
//!     a as *const u32
//! };
//! let _b = pool.acquire_with(2).unwrap();
//! assert_eq!(pool.size(), 2);          // 2 free slots remain
//! assert_eq!(pool.acquired_count(), 2);
//!
//! // SAFETY: a_ptr was obtained from this pool and the slot is still acquired.
//! pool.release(unsafe { &*a_ptr });
//! assert_eq!(pool.size(), 3);
//! pool.clear();
//! assert!(pool.is_full());
//! ```

use core::fmt;
use core::mem::MaybeUninit;
use core::ptr;

// ---------------------------------------------------------------------------
// Helper: checked bitfield byte-count.
// ---------------------------------------------------------------------------

/// Returns the number of bytes needed to hold `n` bits, rounded up.
pub const fn bitfield_bytes(n: usize) -> usize {
    n.div_ceil(8)
}

// ---------------------------------------------------------------------------
// ObjectPool
// ---------------------------------------------------------------------------

/// Fixed-capacity pool allocator with inline storage.
///
/// - `T` — element type.
/// - `N` — maximum number of objects.
/// - `B` — bitfield byte-count; **must equal `(N + 7) / 8`**.  A compile-time
///   assertion enforces this.  You can use `bitfield_bytes(N)` in the type.
///
/// ```rust
/// # use bsw_estd::object_pool::{ObjectPool, bitfield_bytes};
/// let mut pool: ObjectPool<i32, 8, { bitfield_bytes(8) }> = ObjectPool::new();
/// ```
pub struct ObjectPool<T, const N: usize, const B: usize> {
    /// Raw storage for up to N objects.
    data: [MaybeUninit<T>; N],
    /// Occupancy bitfield — bit `i` is 1 when slot `i` is acquired.
    used: [u8; B],
    /// Number of **free** slots (C++ convention: "size" = available capacity).
    available: usize,
}

// SAFETY: ObjectPool owns its T values.  Sharing across threads requires T: Send / T: Sync just
// as Vec<T> does.  We spell it out explicitly to be self-documenting.
unsafe impl<T: Send, const N: usize, const B: usize> Send for ObjectPool<T, N, B> {}
unsafe impl<T: Sync, const N: usize, const B: usize> Sync for ObjectPool<T, N, B> {}

impl<T, const N: usize, const B: usize> ObjectPool<T, N, B> {
    // Compile-time check: B must be exactly ceil(N/8).
    const CHECK_B: () = {
        assert!(
            B == bitfield_bytes(N),
            "ObjectPool: B must equal (N + 7) / 8"
        );
    };

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /// Creates a new, fully available pool.
    ///
    /// All slots are free; no objects have been constructed yet.
    ///
    /// # Panics (compile-time)
    ///
    /// Panics at compile time if `B != (N + 7) / 8`.
    pub const fn new() -> Self {
        // Trigger the compile-time check.
        let () = Self::CHECK_B;

        Self {
            // SAFETY: MaybeUninit arrays can be initialised via uninit().assume_init().
            // The actual T values are only read after a slot is marked as used.
            data: unsafe { MaybeUninit::uninit().assume_init() },
            used: [0u8; B],
            available: N,
        }
    }

    // -----------------------------------------------------------------------
    // Capacity queries
    // -----------------------------------------------------------------------

    /// Returns the number of **free** (not-yet-acquired) slots.
    ///
    /// Matches the C++ `estd::object_pool::size()` convention where "size"
    /// means remaining capacity, not the number of acquired objects.
    #[inline]
    pub fn size(&self) -> usize {
        self.available
    }

    /// Returns the maximum number of objects the pool can hold (`== N`).
    #[inline]
    pub const fn max_size(&self) -> usize {
        N
    }

    /// Returns `true` when no free slots remain (pool is depleted).
    ///
    /// Mirrors C++ `empty()` — the pool is "empty" of available capacity.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.available == 0
    }

    /// Returns `true` when all slots are free (nothing has been acquired).
    ///
    /// Mirrors C++ `full()`.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.available == N
    }

    /// Returns the number of currently acquired (live) objects (`== N - size()`).
    #[inline]
    pub fn acquired_count(&self) -> usize {
        N - self.available
    }

    // -----------------------------------------------------------------------
    // Acquire
    // -----------------------------------------------------------------------

    /// Acquires a slot and constructs the object using [`Default`].
    ///
    /// Returns `None` if the pool is depleted.
    pub fn acquire(&mut self) -> Option<&mut T>
    where
        T: Default,
    {
        self.acquire_with(T::default())
    }

    /// Acquires a slot and writes `value` into it.
    ///
    /// Returns `None` if the pool is depleted.
    pub fn acquire_with(&mut self, value: T) -> Option<&mut T> {
        if self.available == 0 {
            return None;
        }
        for i in 0..N {
            if !self.is_used(i) {
                self.mark_used(i);
                self.available -= 1;
                // SAFETY: slot i was free, so data[i] is uninitialized (or previously destructed).
                // We write a valid T, mark it used, and return a &mut tied to self's lifetime.
                // No aliasing can occur because `&mut self` is exclusive.
                unsafe {
                    self.data[i].write(value);
                    return Some(self.data[i].assume_init_mut());
                }
            }
        }
        // available > 0 but no free bit found: available counter is inconsistent with bitfield.
        unreachable!("ObjectPool: available counter inconsistent with bitfield")
    }

    // -----------------------------------------------------------------------
    // Release
    // -----------------------------------------------------------------------

    /// Releases an acquired object back to the pool, running its destructor.
    ///
    /// # Panics
    ///
    /// Panics if `object` does not belong to this pool's storage.
    ///
    /// # No-op on already-free slot
    ///
    /// If the slot is already free (double-release), the method silently does
    /// nothing — matching the C++ behaviour where `isUsed` is checked first.
    pub fn release(&mut self, object: &T) {
        assert!(
            self.contains(object),
            "ObjectPool::release: object does not belong to this pool"
        );
        let pos = self.slot_index(object);
        if self.is_used(pos) {
            // SAFETY: slot is marked used, so it contains a valid T.  We drop it in place and
            // then mark the slot free.  No other reference to this slot can exist because we hold
            // `&mut self`.
            unsafe {
                ptr::drop_in_place(self.data[pos].as_mut_ptr());
            }
            self.mark_free(pos);
            self.available += 1;
        }
    }

    // -----------------------------------------------------------------------
    // Contains / ownership check
    // -----------------------------------------------------------------------

    /// Returns `true` if `object` falls within this pool's internal storage range.
    ///
    /// This does **not** check whether the slot is currently acquired.
    pub fn contains(&self, object: &T) -> bool {
        if N == 0 {
            return false;
        }
        let obj_addr = object as *const T as usize;
        // SAFETY: we only read pointer values, never dereference through them here.
        let base = self.data[0].as_ptr() as usize;
        let last = self.data[N - 1].as_ptr() as usize;
        obj_addr >= base && obj_addr <= last
    }

    // -----------------------------------------------------------------------
    // Clear
    // -----------------------------------------------------------------------

    /// Destructs all currently acquired objects and marks every slot free.
    ///
    /// After this call the pool is fully available (`is_full() == true`).
    pub fn clear(&mut self) {
        for i in 0..N {
            if self.is_used(i) {
                // SAFETY: slot is marked used, so it contains a valid, fully initialised T.
                unsafe {
                    ptr::drop_in_place(self.data[i].as_mut_ptr());
                }
                self.mark_free(i);
            }
        }
        self.available = N;
    }

    // -----------------------------------------------------------------------
    // Iterators
    // -----------------------------------------------------------------------

    /// Returns an iterator over all **acquired** (live) objects, in slot order.
    pub fn iter(&self) -> Iter<'_, T, N, B> {
        Iter { pool: self, pos: 0 }
    }

    /// Returns a mutable iterator over all **acquired** (live) objects, in slot order.
    pub fn iter_mut(&mut self) -> IterMut<'_, T, N, B> {
        IterMut { pool: self, pos: 0 }
    }

    // -----------------------------------------------------------------------
    // Internal bitfield helpers
    // -----------------------------------------------------------------------

    #[inline]
    fn is_used(&self, pos: usize) -> bool {
        let byte = pos / 8;
        let bit = pos % 8;
        (self.used[byte] & (1u8 << bit)) != 0
    }

    #[inline]
    fn mark_used(&mut self, pos: usize) {
        let byte = pos / 8;
        let bit = pos % 8;
        self.used[byte] |= 1u8 << bit;
    }

    #[inline]
    fn mark_free(&mut self, pos: usize) {
        let byte = pos / 8;
        let bit = pos % 8;
        self.used[byte] &= !(1u8 << bit);
    }

    // -----------------------------------------------------------------------
    // Internal: compute slot index from a reference (pointer arithmetic)
    // -----------------------------------------------------------------------

    /// Returns the slot index for `object`, which must be inside this pool.
    ///
    /// Caller must verify `contains(object)` first.
    fn slot_index(&self, object: &T) -> usize {
        let obj_ptr = object as *const T;
        let base_ptr = self.data[0].as_ptr();
        // SAFETY: obj_ptr is within the same allocated object (the inline array) as base_ptr,
        // so offset_from is well-defined and yields a non-negative, in-bounds value.
        unsafe { obj_ptr.offset_from(base_ptr) as usize }
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<T, const N: usize, const B: usize> Default for ObjectPool<T, N, B> {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Drop — destruct all live objects
// ---------------------------------------------------------------------------

impl<T, const N: usize, const B: usize> Drop for ObjectPool<T, N, B> {
    fn drop(&mut self) {
        self.clear();
    }
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

impl<T: fmt::Debug, const N: usize, const B: usize> fmt::Debug for ObjectPool<T, N, B> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut list = f.debug_list();
        for item in self.iter() {
            list.entry(item);
        }
        list.finish()
    }
}

// ---------------------------------------------------------------------------
// IntoIterator adapters
// ---------------------------------------------------------------------------

impl<'a, T, const N: usize, const B: usize> IntoIterator for &'a ObjectPool<T, N, B> {
    type Item = &'a T;
    type IntoIter = Iter<'a, T, N, B>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, T, const N: usize, const B: usize> IntoIterator for &'a mut ObjectPool<T, N, B> {
    type Item = &'a mut T;
    type IntoIter = IterMut<'a, T, N, B>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

// ---------------------------------------------------------------------------
// Iter — shared (immutable) iterator
// ---------------------------------------------------------------------------

/// Iterator over the acquired (live) objects of an [`ObjectPool`].
///
/// Yields `&T` for every slot that is currently marked as used, in ascending
/// slot-index order.
pub struct Iter<'a, T, const N: usize, const B: usize> {
    pool: &'a ObjectPool<T, N, B>,
    pos: usize,
}

impl<'a, T, const N: usize, const B: usize> Iterator for Iter<'a, T, N, B> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        while self.pos < N {
            let i = self.pos;
            self.pos += 1;
            if self.pool.is_used(i) {
                // SAFETY: slot i is marked used, so data[i] contains a fully initialised T.
                // The lifetime is bound to the pool's shared borrow 'a.
                return Some(unsafe { self.pool.data[i].assume_init_ref() });
            }
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = N.saturating_sub(self.pos);
        (0, Some(remaining))
    }
}

// ---------------------------------------------------------------------------
// IterMut — mutable iterator
// ---------------------------------------------------------------------------

/// Mutable iterator over the acquired (live) objects of an [`ObjectPool`].
///
/// Yields `&mut T` for every slot that is currently marked as used, in
/// ascending slot-index order.
pub struct IterMut<'a, T, const N: usize, const B: usize> {
    pool: &'a mut ObjectPool<T, N, B>,
    pos: usize,
}

impl<'a, T, const N: usize, const B: usize> Iterator for IterMut<'a, T, N, B> {
    type Item = &'a mut T;

    fn next(&mut self) -> Option<&'a mut T> {
        while self.pos < N {
            let i = self.pos;
            self.pos += 1;
            if self.pool.is_used(i) {
                // SAFETY: slot i is marked used, so data[i] contains a fully initialised T.
                // We extend the lifetime from the pool's mutable borrow to 'a.  This is sound
                // because:
                //   1. We advance `pos` before returning so the same index is never yielded twice.
                //   2. The IterMut holds the only &mut to the pool, preventing concurrent access.
                return Some(unsafe { &mut *self.pool.data[i].as_mut_ptr() });
            }
        }
        None
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = N.saturating_sub(self.pos);
        (0, Some(remaining))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    // Shorthand type aliases used throughout the tests.
    // B = (N + 7) / 8 spelled out literally for each N we use.
    type Pool4<T> = ObjectPool<T, 4, 1>; //  4 slots ->  1 byte bitfield
    type Pool6<T> = ObjectPool<T, 6, 1>; //  6 slots ->  1 byte bitfield
    type Pool8<T> = ObjectPool<T, 8, 1>; //  8 slots ->  1 byte bitfield
    type Pool9<T> = ObjectPool<T, 9, 2>; //  9 slots ->  2 byte bitfield
    type Pool16<T> = ObjectPool<T, 16, 2>; // 16 slots ->  2 byte bitfield
    type Pool64<T> = ObjectPool<T, 64, 8>; // 64 slots ->  8 byte bitfield
    type Pool0<T> = ObjectPool<T, 0, 0>; //  0 slots ->  0 byte bitfield
    type Pool2<T> = ObjectPool<T, 2, 1>; //  2 slots ->  1 byte bitfield
    type Pool3<T> = ObjectPool<T, 3, 1>; //  3 slots ->  1 byte bitfield

    // -----------------------------------------------------------------------
    // Test helper: counts Drop invocations via a shared Cell.
    // -----------------------------------------------------------------------

    struct DropCounter<'a> {
        drops: &'a Cell<usize>,
        _value: i32,
    }

    impl<'a> DropCounter<'a> {
        fn new(drops: &'a Cell<usize>, value: i32) -> Self {
            DropCounter { drops, _value: value }
        }
    }

    impl Drop for DropCounter<'_> {
        fn drop(&mut self) {
            self.drops.set(self.drops.get() + 1);
        }
    }

    // -----------------------------------------------------------------------
    // 1. New pool invariants
    // -----------------------------------------------------------------------

    #[test]
    fn new_pool_has_full_availability() {
        let pool: Pool8<i32> = ObjectPool::new();
        assert_eq!(pool.size(), 8, "size() should equal N for a fresh pool");
        assert!(pool.is_full(), "is_full() should be true for a fresh pool");
        assert!(!pool.is_empty(), "is_empty() should be false for a fresh pool");
        assert_eq!(pool.max_size(), 8);
        assert_eq!(pool.acquired_count(), 0);
    }

    // -----------------------------------------------------------------------
    // 2. Acquire one object
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_one_decrements_available() {
        let mut pool: Pool8<i32> = ObjectPool::new();
        let r = pool.acquire().unwrap();
        *r = 42;
        assert_eq!(pool.size(), 7, "size() should decrease by 1 after one acquire");
        assert_eq!(pool.acquired_count(), 1);
        assert!(!pool.is_full());
        assert!(!pool.is_empty());
    }

    // -----------------------------------------------------------------------
    // 3. Acquire until depleted
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_until_depleted() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        for i in 0..4_i32 {
            let r = pool.acquire_with(i);
            assert!(r.is_some(), "acquire should succeed for slot {i}");
        }
        assert!(pool.is_empty(), "pool should be empty after N acquires");
        assert_eq!(pool.size(), 0);
        assert!(pool.acquire().is_none(), "acquire on depleted pool must return None");
        assert!(pool.acquire_with(99).is_none());
    }

    // -----------------------------------------------------------------------
    // 4. Release returns slot to pool
    // -----------------------------------------------------------------------

    #[test]
    fn release_increments_available() {
        let mut pool: Pool4<i32> = ObjectPool::new();

        // Acquire all four slots, capturing raw pointers so we can release later
        // without holding live references into the pool simultaneously.
        let mut ptrs = [core::ptr::null::<i32>(); 4];
        for (idx, slot) in ptrs.iter_mut().enumerate() {
            let r = pool.acquire_with(idx as i32).unwrap();
            *slot = r as *const i32;
        }
        assert_eq!(pool.size(), 0);

        // Release the first slot via its raw pointer.
        // SAFETY: ptrs[0] was obtained from this pool and the slot is still acquired.
        pool.release(unsafe { &*ptrs[0] });
        assert_eq!(pool.size(), 1);
        assert_eq!(pool.acquired_count(), 3);
    }

    // -----------------------------------------------------------------------
    // 5. Release object not in pool — should panic
    // -----------------------------------------------------------------------

    #[test]
    #[should_panic(expected = "ObjectPool::release: object does not belong to this pool")]
    fn release_foreign_object_panics() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        let x: i32 = 99; // stack variable, not inside the pool
        pool.release(&x);
    }

    // -----------------------------------------------------------------------
    // 6. contains() — true for pool objects, false for stack objects
    // -----------------------------------------------------------------------

    #[test]
    fn contains_true_for_acquired_false_for_stack() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        // Capture the pointer, drop the &mut borrow, then call the shared &self method.
        let r_ptr: *const i32 = pool.acquire_with(1).unwrap();
        // SAFETY: r_ptr is a valid acquired slot; we take a shared ref for the duration of the
        // assert only, after the &mut from acquire_with is no longer live.
        assert!(pool.contains(unsafe { &*r_ptr }), "contains must be true for pool slot");

        let stack_val: i32 = 0;
        assert!(!pool.contains(&stack_val), "contains must be false for stack variable");
    }

    #[test]
    fn contains_covers_every_slot() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        let mut ptrs = [core::ptr::null::<i32>(); 4];
        for (idx, slot) in ptrs.iter_mut().enumerate() {
            *slot = pool.acquire_with(idx as i32).unwrap() as *const i32;
        }
        for p in &ptrs {
            // SAFETY: each pointer is from the pool and the slot is still acquired.
            assert!(pool.contains(unsafe { &**p }));
        }
    }

    // -----------------------------------------------------------------------
    // 7. clear() destructs all acquired objects, pool returns to full
    // -----------------------------------------------------------------------

    #[test]
    fn clear_resets_pool_to_full_and_runs_destructors() {
        let drops = Cell::new(0usize);
        {
            let mut pool: Pool4<DropCounter<'_>> = ObjectPool::new();
            pool.acquire_with(DropCounter::new(&drops, 1)).unwrap();
            pool.acquire_with(DropCounter::new(&drops, 2)).unwrap();
            pool.acquire_with(DropCounter::new(&drops, 3)).unwrap();
            assert_eq!(drops.get(), 0, "no drops before clear");

            pool.clear();
            assert_eq!(drops.get(), 3, "clear must destruct all 3 acquired objects");
            assert!(pool.is_full());
            assert_eq!(pool.size(), 4);
            assert_eq!(pool.acquired_count(), 0);
        }
        // Pool drops here; it is already cleared so no extra drops occur.
        assert_eq!(drops.get(), 3);
    }

    // -----------------------------------------------------------------------
    // 8. Iterator yields only acquired objects in slot order
    // -----------------------------------------------------------------------

    #[test]
    fn iter_yields_acquired_in_slot_order() {
        let mut pool: Pool6<i32> = ObjectPool::new();
        pool.acquire_with(10).unwrap();
        pool.acquire_with(20).unwrap();
        pool.acquire_with(30).unwrap();

        let mut vals = [0i32; 6];
        let mut count = 0usize;
        for v in pool.iter() {
            vals[count] = *v;
            count += 1;
        }
        assert_eq!(count, 3);
        assert_eq!(&vals[..3], &[10i32, 20, 30]);
    }

    // -----------------------------------------------------------------------
    // 9. Iterator skips released slots
    // -----------------------------------------------------------------------

    #[test]
    fn iter_skips_released_slots() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        let mut ptrs = [core::ptr::null::<i32>(); 4];
        for (idx, slot) in ptrs.iter_mut().enumerate() {
            *slot = pool.acquire_with((idx as i32) * 10).unwrap() as *const i32;
        }

        // Release slot 1 (value 10) — the second slot.
        // SAFETY: ptrs[1] is a valid acquired slot in this pool.
        pool.release(unsafe { &*ptrs[1] });

        let mut vals = [0i32; 4];
        let mut count = 0usize;
        for v in pool.iter() {
            vals[count] = *v;
            count += 1;
        }
        // Slots 0, 2, 3 are live: values 0, 20, 30.
        assert_eq!(count, 3);
        assert_eq!(&vals[..3], &[0i32, 20, 30]);
    }

    // -----------------------------------------------------------------------
    // 10. Drop counter: acquire_with calls no extra drops; release calls exactly 1
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_does_not_drop_value_release_drops_exactly_once() {
        let drops = Cell::new(0usize);
        let mut pool: Pool4<DropCounter<'_>> = ObjectPool::new();

        let r_ptr: *mut DropCounter<'_> =
            pool.acquire_with(DropCounter::new(&drops, 7)).unwrap() as *mut DropCounter<'_>;
        assert_eq!(drops.get(), 0, "acquire_with must not drop the supplied value");

        // SAFETY: r_ptr is a valid acquired slot still belonging to this pool.
        pool.release(unsafe { &*r_ptr });
        assert_eq!(drops.get(), 1, "release must drop the object exactly once");
        assert!(pool.is_full());
    }

    // -----------------------------------------------------------------------
    // 11. Drop of pool destructs all acquired objects
    // -----------------------------------------------------------------------

    #[test]
    fn pool_drop_destructs_all_acquired() {
        let drops = Cell::new(0usize);
        {
            let mut pool: Pool3<DropCounter<'_>> = ObjectPool::new();
            pool.acquire_with(DropCounter::new(&drops, 1)).unwrap();
            pool.acquire_with(DropCounter::new(&drops, 2)).unwrap();
            // Slot 2 left free intentionally — only 2 drops should occur.
            assert_eq!(drops.get(), 0);
        } // pool drops here
        assert_eq!(drops.get(), 2, "pool Drop must destruct exactly the 2 acquired objects");
    }

    // -----------------------------------------------------------------------
    // 12. Acquire-release-acquire reuses the freed slot
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_release_acquire_reuses_freed_slot() {
        let mut pool: Pool2<i32> = ObjectPool::new();

        let p0 = pool.acquire_with(0).unwrap() as *const i32;
        let p1 = pool.acquire_with(1).unwrap() as *const i32;
        assert!(pool.is_empty());

        // Release slot 0.
        // SAFETY: p0 is an acquired slot in this pool.
        pool.release(unsafe { &*p0 });
        assert_eq!(pool.size(), 1);

        // Next acquire should reuse slot 0 (first-free scan returns index 0).
        let p2 = pool.acquire_with(42).unwrap() as *const i32;
        assert_eq!(p2, p0, "re-acquired pointer must point to the freed slot");
        assert!(pool.is_empty());

        // Clean up both remaining live slots.
        // SAFETY: p1 and p2 are both valid acquired slots in this pool.
        pool.release(unsafe { &*p1 });
        pool.release(unsafe { &*p2 });
    }

    // -----------------------------------------------------------------------
    // 13. Zero-capacity pool
    // -----------------------------------------------------------------------

    #[test]
    fn zero_capacity_pool_behaviour() {
        let mut pool: Pool0<i32> = ObjectPool::new();
        assert_eq!(pool.size(), 0);
        assert_eq!(pool.max_size(), 0);
        // C++ full() means all N slots are available.  For N=0: 0 == 0 -> is_full is true.
        assert!(pool.is_full(), "zero-cap pool: 0/0 slots available => is_full");
        // 0 available slots => is_empty is also true.
        assert!(pool.is_empty(), "zero-cap pool: 0 available slots => is_empty");
        assert!(pool.acquire().is_none());
        assert!(pool.acquire_with(1).is_none());
        assert_eq!(pool.acquired_count(), 0);

        // Iterators must yield nothing.
        assert!(pool.iter().next().is_none());
        assert!(pool.iter_mut().next().is_none());

        // contains must return false for any external reference.
        let x: i32 = 0;
        assert!(!pool.contains(&x));
    }

    // -----------------------------------------------------------------------
    // 14. Mutable iteration and modification
    // -----------------------------------------------------------------------

    #[test]
    fn iter_mut_allows_in_place_modification() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        pool.acquire_with(1).unwrap();
        pool.acquire_with(2).unwrap();
        pool.acquire_with(3).unwrap();

        // Double every value in place.
        for v in pool.iter_mut() {
            *v *= 2;
        }

        let mut vals = [0i32; 4];
        let mut count = 0usize;
        for v in pool.iter() {
            vals[count] = *v;
            count += 1;
        }
        assert_eq!(count, 3);
        assert_eq!(&vals[..3], &[2i32, 4, 6]);
    }

    // -----------------------------------------------------------------------
    // Extra: bitfield spans multiple bytes (N = 9 requires 2 bytes)
    // -----------------------------------------------------------------------

    #[test]
    fn bitfield_spans_multiple_bytes() {
        let mut pool: Pool9<i32> = ObjectPool::new();
        let mut ptrs = [core::ptr::null::<i32>(); 9];
        for (idx, slot) in ptrs.iter_mut().enumerate() {
            *slot = pool.acquire_with(idx as i32).unwrap() as *const i32;
        }
        assert!(pool.is_empty());
        assert_eq!(pool.acquired_count(), 9);

        // Release slot 8 — bit 0 of the second byte.
        // SAFETY: ptrs[8] is a valid acquired slot in this pool.
        pool.release(unsafe { &*ptrs[8] });
        assert_eq!(pool.size(), 1);

        // Iterator should yield slots 0-7 only.
        let count = pool.iter().count();
        assert_eq!(count, 8);
    }

    // -----------------------------------------------------------------------
    // Extra: double-release is silently ignored (no double-drop)
    // -----------------------------------------------------------------------

    #[test]
    fn double_release_is_no_op() {
        let drops = Cell::new(0usize);
        let mut pool: Pool4<DropCounter<'_>> = ObjectPool::new();

        let r_ptr: *const DropCounter<'_> =
            pool.acquire_with(DropCounter::new(&drops, 1)).unwrap() as *const DropCounter<'_>;

        // First release: destructor fires once.
        // SAFETY: r_ptr is a valid acquired slot in this pool.
        pool.release(unsafe { &*r_ptr });
        assert_eq!(drops.get(), 1);
        assert_eq!(pool.size(), 4);

        // Second release of the same address: slot is now free, destructor branch skipped.
        // SAFETY: the address is still within the pool's memory range; accessing a free slot's
        // address to pass to contains() is safe because we only compare addresses, not values.
        pool.release(unsafe { &*r_ptr });
        assert_eq!(drops.get(), 1, "drop count must not increase on double-release");
        assert_eq!(pool.size(), 4);
    }

    // -----------------------------------------------------------------------
    // Extra: Debug output contains acquired values (std feature only)
    // -----------------------------------------------------------------------

    #[cfg(feature = "std")]
    #[test]
    fn debug_shows_acquired_values() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        pool.acquire_with(10).unwrap();
        pool.acquire_with(20).unwrap();
        let s = std::format!("{:?}", pool);
        assert!(s.contains("10"), "debug output should contain 10, got: {s}");
        assert!(s.contains("20"), "debug output should contain 20, got: {s}");
    }

    // -----------------------------------------------------------------------
    // Extra: IntoIterator for &pool
    // -----------------------------------------------------------------------

    #[test]
    fn into_iter_ref_works() {
        let mut pool: Pool4<i32> = ObjectPool::new();
        pool.acquire_with(5).unwrap();
        pool.acquire_with(6).unwrap();

        let mut count = 0usize;
        for _ in &pool {
            count += 1;
        }
        assert_eq!(count, 2);
    }

    // -----------------------------------------------------------------------
    // Extra: large pool N=64, verify correct slot index arithmetic across all 8 bytes
    // -----------------------------------------------------------------------

    #[test]
    fn large_pool_n64_full_acquire_and_sum() {
        let mut pool: Pool64<u64> = ObjectPool::new();
        assert_eq!(pool.size(), 64);
        for i in 0..64u64 {
            pool.acquire_with(i).unwrap();
        }
        assert!(pool.is_empty());
        assert_eq!(pool.acquired_count(), 64);

        let sum: u64 = pool.iter().copied().sum();
        assert_eq!(sum, 63 * 64 / 2); // 0 + 1 + ... + 63
    }

    // -----------------------------------------------------------------------
    // Extra: acquire_with preserves the exact value written
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_with_preserves_value() {
        #[derive(Debug, PartialEq)]
        struct Pair(i32, i32);

        let mut pool: ObjectPool<Pair, 2, 1> = ObjectPool::new();
        let r_ptr: *const Pair = pool.acquire_with(Pair(3, 7)).unwrap() as *const Pair;
        // SAFETY: r_ptr is a valid acquired slot in this pool.
        assert_eq!(unsafe { &*r_ptr }, &Pair(3, 7));
    }

    // -----------------------------------------------------------------------
    // Extra: Default trait constructs a fresh pool
    // -----------------------------------------------------------------------

    #[test]
    fn default_trait_produces_full_pool() {
        let pool: Pool16<u8> = ObjectPool::default();
        assert!(pool.is_full());
        assert_eq!(pool.size(), 16);
    }

    // -----------------------------------------------------------------------
    // Extra: Iterator size_hint upper bound decreases as pos advances
    // -----------------------------------------------------------------------

    #[test]
    fn iter_size_hint_upper_bound_decreases() {
        let mut pool: Pool8<i32> = ObjectPool::new();
        for i in 0..5_i32 {
            pool.acquire_with(i).unwrap();
        }
        let mut iter = pool.iter();
        let (lo, hi) = iter.size_hint();
        assert_eq!(lo, 0, "lower bound is always 0 (some slots may be free)");
        assert!(hi.unwrap() <= 8, "upper bound must not exceed N");

        // Advance twice; upper bound should decrease.
        iter.next();
        iter.next();
        let (_, hi2) = iter.size_hint();
        assert!(hi2.unwrap() <= 6, "upper bound should decrease as pos advances");
    }

    // -----------------------------------------------------------------------
    // Extra: compile-time assertion fires when B is wrong (verified manually)
    // The following would fail to compile:
    //     let _: ObjectPool<i32, 4, 2> = ObjectPool::new(); // B=2 != (4+7)/8=1
    // We can't test a compile_fail in a no_std lib without a ui test, so we
    // just document it here and verify the correct B works.
    // -----------------------------------------------------------------------

    #[test]
    fn correct_b_compiles_and_works() {
        // B = (4 + 7) / 8 = 1.
        let pool: ObjectPool<i32, 4, 1> = ObjectPool::new();
        assert_eq!(pool.max_size(), 4);
        assert!(pool.is_full());
    }

    // -----------------------------------------------------------------------
    // Extra: bitfield_bytes helper function
    // -----------------------------------------------------------------------

    #[test]
    fn bitfield_bytes_helper() {
        assert_eq!(bitfield_bytes(0), 0);
        assert_eq!(bitfield_bytes(1), 1);
        assert_eq!(bitfield_bytes(7), 1);
        assert_eq!(bitfield_bytes(8), 1);
        assert_eq!(bitfield_bytes(9), 2);
        assert_eq!(bitfield_bytes(16), 2);
        assert_eq!(bitfield_bytes(17), 3);
        assert_eq!(bitfield_bytes(64), 8);
    }
}
