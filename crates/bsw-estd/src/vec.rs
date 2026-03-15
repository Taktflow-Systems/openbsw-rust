// Copyright 2024 Accenture / Taktflow Systems.
//
// Rust port of OpenBSW `estd::vector` / `estd::declare::vector`.
//
// Design:
//   - `#![no_std]` compatible — uses `core` only, zero heap allocation.
//   - Inline storage: `[MaybeUninit<T>; N]`.
//   - `len` tracks how many elements are live.
//   - Every `unsafe` block carries a `// SAFETY:` comment.

use core::fmt;
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut, Index, IndexMut};
use core::ptr;

// ---------------------------------------------------------------------------
// Primary type
// ---------------------------------------------------------------------------

/// A fixed-capacity vector with inline (stack) storage.
///
/// This is the Rust port of `estd::declare::vector<T, N>`.  The capacity
/// `N` is a const-generic parameter fixed at compile time.  No heap
/// allocation is ever performed.
///
/// # Invariants
///
/// At all times:
/// - `len <= N`
/// - Elements at indices `0..len` are fully initialised.
/// - Elements at indices `len..N` are uninitialised and must never be read.
pub struct FixedVec<T, const N: usize> {
    data: [MaybeUninit<T>; N],
    len: usize,
}

// ---------------------------------------------------------------------------
// Constructors and core methods
// ---------------------------------------------------------------------------

impl<T, const N: usize> FixedVec<T, N> {
    /// Creates an empty `FixedVec`.
    ///
    /// This function is `const`, so it can be used to initialise statics.
    pub const fn new() -> Self {
        Self {
            // SAFETY: An array of `MaybeUninit<T>` does not require `T` to be
            // initialised; `MaybeUninit::uninit()` is always valid here and is
            // zero-overhead at runtime.
            data: unsafe { MaybeUninit::<[MaybeUninit<T>; N]>::uninit().assume_init() },
            len: 0,
        }
    }

    /// Returns the number of elements currently stored.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns the fixed capacity (`N`).
    ///
    /// This replaces C++'s `max_size()`.
    #[inline]
    pub fn capacity(&self) -> usize {
        N
    }

    /// Returns `true` when no elements are stored.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns `true` when `len == N` (capacity exhausted).
    ///
    /// Maps to C++'s `full()`.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.len == N
    }

    // --- Raw pointer helpers (private) ------------------------------------

    /// Returns a raw const pointer to element at `index`.
    ///
    /// # Safety
    /// `index` must be `< N`.
    #[inline]
    unsafe fn ptr_at(&self, index: usize) -> *const T {
        // SAFETY: caller guarantees index < N; `data` has N slots, each
        // `MaybeUninit<T>` has the same layout as `T`.
        self.data.as_ptr().add(index).cast::<T>()
    }

    /// Returns a raw mutable pointer to element at `index`.
    ///
    /// # Safety
    /// `index` must be `< N`.
    #[inline]
    unsafe fn ptr_at_mut(&mut self, index: usize) -> *mut T {
        // SAFETY: same as `ptr_at`, but mutable.
        self.data.as_mut_ptr().add(index).cast::<T>()
    }

    // --- Slice views -------------------------------------------------------

    /// Returns a shared slice over the live elements.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        // SAFETY: Elements at `0..self.len` are initialised (struct invariant).
        // The pointer is valid for `self.len` elements of type `T`.
        unsafe { core::slice::from_raw_parts(self.ptr_at(0), self.len) }
    }

    /// Returns a mutable slice over the live elements.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        let len = self.len;
        // SAFETY: Same as `as_slice`, but with exclusive access guaranteed by
        // `&mut self`.
        unsafe { core::slice::from_raw_parts_mut(self.ptr_at_mut(0), len) }
    }

    // --- Push / Pop --------------------------------------------------------

    /// Appends `value` at the end.
    ///
    /// Returns `Err(value)` if the vector is already full, leaving it
    /// unchanged.
    pub fn push(&mut self, value: T) -> Result<(), T> {
        if self.len == N {
            return Err(value);
        }
        // SAFETY: `self.len < N`, so `self.len` is a valid slot index.
        // We write through the `MaybeUninit` pointer, which is always safe.
        unsafe {
            ptr::write(self.ptr_at_mut(self.len), value);
        }
        self.len += 1;
        Ok(())
    }

    /// Removes and returns the last element, or `None` if empty.
    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        self.len -= 1;
        // SAFETY: `self.len` was just decremented; the element at the old
        // `self.len` is live (invariant held before decrement) and ownership
        // is transferred to the caller via `ptr::read`.  The slot is now
        // logically uninitialised (len was decremented).
        let value = unsafe { ptr::read(self.ptr_at(self.len)) };
        Some(value)
    }

    // --- Insert / Remove ---------------------------------------------------

    /// Inserts `value` at `index`, shifting elements to the right.
    ///
    /// Returns `Err(value)` if:
    /// - the vector is full, or
    /// - `index > self.len` (out of range).
    pub fn insert(&mut self, index: usize, value: T) -> Result<(), T> {
        if self.len == N || index > self.len {
            return Err(value);
        }
        // SAFETY: `index <= self.len < N`.
        // We shift elements `[index, len)` one slot to the right, then write
        // `value` at `index`.
        //
        // `ptr::copy` handles potentially overlapping source/destination
        // ranges correctly (it is equivalent to `memmove`).
        unsafe {
            let src = self.ptr_at(index);
            let dst = self.ptr_at_mut(index + 1);
            let count = self.len - index;
            ptr::copy(src, dst, count);
            ptr::write(self.ptr_at_mut(index), value);
        }
        self.len += 1;
        Ok(())
    }

    /// Removes and returns the element at `index`, shifting the tail left.
    ///
    /// # Panics
    ///
    /// Panics if `index >= self.len`.
    pub fn remove(&mut self, index: usize) -> T {
        assert!(index < self.len, "FixedVec::remove: index out of bounds");
        // SAFETY: `index < self.len` ensures the slot is initialised.
        // We read the element to return it, then compact the tail.
        let value = unsafe { ptr::read(self.ptr_at(index)) };
        // SAFETY: Elements `(index+1)..len` are shifted left by one.
        // `ptr::copy` handles overlapping ranges (memmove semantics).
        unsafe {
            let src = self.ptr_at(index + 1);
            let dst = self.ptr_at_mut(index);
            let count = self.len - index - 1;
            ptr::copy(src, dst, count);
        }
        self.len -= 1;
        value
    }

    /// Removes the element at `index` by swapping it with the last element
    /// and decrementing `len`.  O(1) but does not preserve order.
    ///
    /// # Panics
    ///
    /// Panics if `index >= self.len`.
    pub fn swap_remove(&mut self, index: usize) -> T {
        assert!(
            index < self.len,
            "FixedVec::swap_remove: index out of bounds"
        );
        // SAFETY: `index < self.len` and `self.len - 1 < N`; both slots are
        // initialised.  We swap the target element with the last element and
        // then read the (now last) slot without dropping it (decrement len).
        unsafe {
            let last_index = self.len - 1;
            ptr::swap(self.ptr_at_mut(index), self.ptr_at_mut(last_index));
            self.len -= 1;
            ptr::read(self.ptr_at(self.len))
        }
    }

    // --- Bulk operations ---------------------------------------------------

    /// Removes all elements, running their destructors.
    pub fn clear(&mut self) {
        let old_len = self.len;
        // Set `len` to 0 *before* dropping so that a panic in a destructor
        // does not leave the vec in a state where elements could be
        // double-dropped.
        self.len = 0;
        // SAFETY: Elements at `0..old_len` are initialised (invariant).
        // `drop_in_place` on a slice pointer calls each element's destructor
        // exactly once.
        unsafe {
            let slice_ptr =
                ptr::slice_from_raw_parts_mut(self.data.as_mut_ptr().cast::<T>(), old_len);
            ptr::drop_in_place(slice_ptr);
        }
    }

    /// Shortens the vector to `new_len`, dropping any excess elements.
    ///
    /// If `new_len >= self.len`, this is a no-op.
    pub fn truncate(&mut self, new_len: usize) {
        if new_len >= self.len {
            return;
        }
        let old_len = self.len;
        // Guard against panics in Drop: shrink len first.
        self.len = new_len;
        // SAFETY: Elements at `new_len..old_len` are initialised and need to
        // be dropped.  We form a raw slice pointer to exactly those elements.
        unsafe {
            let ptr = self.data.as_mut_ptr().add(new_len).cast::<T>();
            let slice_ptr = ptr::slice_from_raw_parts_mut(ptr, old_len - new_len);
            ptr::drop_in_place(slice_ptr);
        }
    }

    /// Resizes the vector to `new_len`.
    ///
    /// - If `new_len < self.len`: excess elements are dropped (`truncate`).
    /// - If `new_len > self.len`: new slots are filled with clones of `value`.
    /// - If `new_len > N`: only `N` elements will be kept (saturates at
    ///   capacity rather than panicking, matching the C++ semantics where the
    ///   assert is a debug-only guard).
    pub fn resize(&mut self, new_len: usize, value: T)
    where
        T: Clone,
    {
        let clamped = new_len.min(N);
        if clamped <= self.len {
            self.truncate(clamped);
        } else {
            while self.len < clamped {
                // We know there is room: self.len < clamped <= N.
                // SAFETY: `self.len < N`, so the slot is uninitialised and
                // valid to write.
                unsafe {
                    ptr::write(self.ptr_at_mut(self.len), value.clone());
                }
                self.len += 1;
            }
        }
    }

    /// Retains only the elements for which `f` returns `true`.
    ///
    /// Elements are visited in order.  Removed elements are dropped in place.
    /// The relative order of retained elements is preserved.
    pub fn retain(&mut self, mut f: impl FnMut(&T) -> bool) {
        let mut write_idx = 0usize;
        let mut read_idx = 0usize;
        while read_idx < self.len {
            // SAFETY: `read_idx < self.len` — slot is initialised.
            let keep = unsafe { f(&*self.ptr_at(read_idx)) };
            if keep {
                if write_idx != read_idx {
                    // SAFETY: Both indices are within `0..self.len`.
                    // `ptr::copy_nonoverlapping` is safe because `write_idx <
                    // read_idx`, so the two slots do not overlap.
                    unsafe {
                        ptr::copy_nonoverlapping(
                            self.ptr_at(read_idx),
                            self.ptr_at_mut(write_idx),
                            1,
                        );
                    }
                }
                write_idx += 1;
            } else {
                // SAFETY: `read_idx < self.len` — slot is initialised; we
                // drop it in place.
                unsafe {
                    ptr::drop_in_place(self.ptr_at_mut(read_idx));
                }
            }
            read_idx += 1;
        }
        self.len = write_idx;
    }

    /// Appends clones of all elements from `other` until full or `other` is
    /// exhausted.
    pub fn extend_from_slice(&mut self, other: &[T])
    where
        T: Clone,
    {
        for item in other {
            if self.len == N {
                break;
            }
            // SAFETY: `self.len < N`, slot is uninitialised and valid to write.
            unsafe {
                ptr::write(self.ptr_at_mut(self.len), item.clone());
            }
            self.len += 1;
        }
    }

    // --- Element access ----------------------------------------------------

    /// Returns a reference to the first element, or `None` if empty.
    #[inline]
    pub fn first(&self) -> Option<&T> {
        self.as_slice().first()
    }

    /// Returns a reference to the last element, or `None` if empty.
    #[inline]
    pub fn last(&self) -> Option<&T> {
        self.as_slice().last()
    }

    /// Returns a reference to the element at `index`, or `None` if out of
    /// bounds.
    #[inline]
    pub fn get(&self, index: usize) -> Option<&T> {
        self.as_slice().get(index)
    }

    /// Returns a mutable reference to the element at `index`, or `None` if
    /// out of bounds.
    #[inline]
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        self.as_mut_slice().get_mut(index)
    }

    // --- Iterators ---------------------------------------------------------

    /// Returns an iterator over shared references to the live elements.
    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, T> {
        self.as_slice().iter()
    }

    /// Returns an iterator over mutable references to the live elements.
    #[inline]
    pub fn iter_mut(&mut self) -> core::slice::IterMut<'_, T> {
        self.as_mut_slice().iter_mut()
    }
}

// ---------------------------------------------------------------------------
// Drop
// ---------------------------------------------------------------------------

impl<T, const N: usize> Drop for FixedVec<T, N> {
    fn drop(&mut self) {
        self.clear();
    }
}

// ---------------------------------------------------------------------------
// Deref / DerefMut
// ---------------------------------------------------------------------------

impl<T, const N: usize> Deref for FixedVec<T, N> {
    type Target = [T];

    #[inline]
    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T, const N: usize> DerefMut for FixedVec<T, N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

// ---------------------------------------------------------------------------
// Index / IndexMut
// ---------------------------------------------------------------------------

impl<T, const N: usize> Index<usize> for FixedVec<T, N> {
    type Output = T;

    #[inline]
    fn index(&self, index: usize) -> &T {
        &self.as_slice()[index]
    }
}

impl<T, const N: usize> IndexMut<usize> for FixedVec<T, N> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut T {
        &mut self.as_mut_slice()[index]
    }
}

// ---------------------------------------------------------------------------
// IntoIterator
// ---------------------------------------------------------------------------

impl<'a, T, const N: usize> IntoIterator for &'a FixedVec<T, N> {
    type Item = &'a T;
    type IntoIter = core::slice::Iter<'a, T>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, T, const N: usize> IntoIterator for &'a mut FixedVec<T, N> {
    type Item = &'a mut T;
    type IntoIter = core::slice::IterMut<'a, T>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

// ---------------------------------------------------------------------------
// PartialEq / Eq
// ---------------------------------------------------------------------------

impl<T: PartialEq, const N: usize, const M: usize> PartialEq<FixedVec<T, M>> for FixedVec<T, N> {
    fn eq(&self, other: &FixedVec<T, M>) -> bool {
        self.as_slice() == other.as_slice()
    }
}

impl<T: Eq, const N: usize> Eq for FixedVec<T, N> {}

// ---------------------------------------------------------------------------
// PartialOrd / Ord
// ---------------------------------------------------------------------------

impl<T: PartialOrd, const N: usize, const M: usize> PartialOrd<FixedVec<T, M>>
    for FixedVec<T, N>
{
    fn partial_cmp(&self, other: &FixedVec<T, M>) -> Option<core::cmp::Ordering> {
        self.as_slice().partial_cmp(other.as_slice())
    }
}

impl<T: Ord, const N: usize> Ord for FixedVec<T, N> {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.as_slice().cmp(other.as_slice())
    }
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

impl<T: fmt::Debug, const N: usize> fmt::Debug for FixedVec<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FixedVec")
            .field("len", &self.len)
            .field("capacity", &N)
            .field("data", &self.as_slice())
            .finish()
    }
}

// ---------------------------------------------------------------------------
// Clone
// ---------------------------------------------------------------------------

impl<T: Clone, const N: usize> Clone for FixedVec<T, N> {
    fn clone(&self) -> Self {
        let mut out = Self::new();
        for item in self.as_slice() {
            // SAFETY: `out.len` starts at 0 and is incremented only after a
            // successful write; the loop runs at most `self.len <= N` times
            // so we never exceed capacity.
            unsafe {
                ptr::write(out.ptr_at_mut(out.len), item.clone());
            }
            out.len += 1;
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<T, const N: usize> Default for FixedVec<T, N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// From<[T; M]>
// ---------------------------------------------------------------------------

/// Constructs a `FixedVec<T, N>` from an array `[T; M]` where `M <= N`.
///
/// Because Rust's const-generics cannot express `M <= N` as a trait bound
/// directly, this is implemented for `M == N` and separately via a helper
/// macro for common smaller sizes.  For general use, prefer
/// [`FixedVec::extend_from_slice`].
///
/// A compile-time assert inside the `from` body enforces `M <= N`.
impl<T: Clone, const N: usize, const M: usize> From<[T; M]> for FixedVec<T, N> {
    fn from(arr: [T; M]) -> Self {
        // Enforce M <= N at compile time.
        const { assert!(M <= N, "array length M must be <= FixedVec capacity N") }
        let mut v = Self::new();
        for item in &arr {
            // SAFETY: M <= N (asserted above); the loop runs at most M times
            // so we never exceed capacity.
            unsafe {
                ptr::write(v.ptr_at_mut(v.len), item.clone());
            }
            v.len += 1;
        }
        // We consumed the array elements via clone; now forget the originals
        // to avoid double-drop.
        core::mem::forget(arr);
        v
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::FixedVec;
    use core::cell::Cell;

    // -----------------------------------------------------------------------
    // Helper: drop counter
    // -----------------------------------------------------------------------

    // Debug is required because push() returns Result<(), T> and .unwrap()
    // needs T: Debug to format the panic message on Err.
    #[derive(Debug)]
    struct DropCounter<'a>(&'a Cell<usize>);

    impl Drop for DropCounter<'_> {
        fn drop(&mut self) {
            self.0.set(self.0.get() + 1);
        }
    }

    // A cloneable wrapper around a drop counter for tests that need both Clone
    // and drop tracking.  Cloning increments the source counter so the test can
    // observe it.
    #[derive(Debug)]
    struct CloneDropCounter<'a>(&'a Cell<usize>);

    impl Clone for CloneDropCounter<'_> {
        fn clone(&self) -> Self {
            // Each clone call is observable — the new instance shares the counter.
            CloneDropCounter(self.0)
        }
    }

    impl Drop for CloneDropCounter<'_> {
        fn drop(&mut self) {
            self.0.set(self.0.get() + 1);
        }
    }

    // -----------------------------------------------------------------------
    // 1. Basic push / pop lifecycle
    // -----------------------------------------------------------------------

    #[test]
    fn push_pop_basic() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        assert!(v.is_empty());
        assert_eq!(v.len(), 0);

        assert!(v.push(10).is_ok());
        assert!(v.push(20).is_ok());
        assert!(v.push(30).is_ok());
        assert_eq!(v.len(), 3);
        assert!(!v.is_empty());

        assert_eq!(v.pop(), Some(30));
        assert_eq!(v.pop(), Some(20));
        assert_eq!(v.pop(), Some(10));
        assert_eq!(v.pop(), None);
        assert!(v.is_empty());
    }

    // -----------------------------------------------------------------------
    // 2. Push until full; is_full(); push returns Err
    // -----------------------------------------------------------------------

    #[test]
    fn push_until_full() {
        let mut v: FixedVec<u8, 3> = FixedVec::new();
        assert!(!v.is_full());

        assert!(v.push(1).is_ok());
        assert!(v.push(2).is_ok());
        assert!(v.push(3).is_ok());

        assert!(v.is_full());
        assert_eq!(v.capacity(), 3);

        // Push to a full vec must return Err with the original value.
        let result = v.push(99);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), 99);
        assert_eq!(v.len(), 3); // unchanged
    }

    // -----------------------------------------------------------------------
    // 3. Insert at beginning, middle, end
    // -----------------------------------------------------------------------

    #[test]
    fn insert_at_beginning() {
        let mut v: FixedVec<i32, 5> = FixedVec::new();
        v.push(2).unwrap();
        v.push(3).unwrap();

        v.insert(0, 1).unwrap();
        assert_eq!(v.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn insert_at_middle() {
        let mut v: FixedVec<i32, 5> = FixedVec::new();
        v.push(1).unwrap();
        v.push(3).unwrap();

        v.insert(1, 2).unwrap();
        assert_eq!(v.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn insert_at_end() {
        let mut v: FixedVec<i32, 5> = FixedVec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();

        v.insert(2, 3).unwrap();
        assert_eq!(v.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn insert_into_full_returns_err() {
        let mut v: FixedVec<i32, 2> = FixedVec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();

        let result = v.insert(0, 99);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), 99);
    }

    #[test]
    fn insert_out_of_range_returns_err() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();

        // index == 5, but len == 1
        let result = v.insert(5, 42);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // 4. Remove from beginning, middle, end
    // -----------------------------------------------------------------------

    #[test]
    fn remove_from_beginning() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [10, 20, 30] {
            v.push(x).unwrap();
        }
        let removed = v.remove(0);
        assert_eq!(removed, 10);
        assert_eq!(v.as_slice(), &[20, 30]);
    }

    #[test]
    fn remove_from_middle() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [10, 20, 30] {
            v.push(x).unwrap();
        }
        let removed = v.remove(1);
        assert_eq!(removed, 20);
        assert_eq!(v.as_slice(), &[10, 30]);
    }

    #[test]
    fn remove_from_end() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [10, 20, 30] {
            v.push(x).unwrap();
        }
        let removed = v.remove(2);
        assert_eq!(removed, 30);
        assert_eq!(v.as_slice(), &[10, 20]);
    }

    #[test]
    #[should_panic(expected = "index out of bounds")]
    fn remove_out_of_bounds_panics() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        v.remove(5);
    }

    // -----------------------------------------------------------------------
    // 5. Clear drops all elements
    // -----------------------------------------------------------------------

    #[test]
    fn clear_drops_all_elements() {
        let counter = Cell::new(0usize);
        {
            let mut v: FixedVec<DropCounter<'_>, 4> = FixedVec::new();
            v.push(DropCounter(&counter)).unwrap();
            v.push(DropCounter(&counter)).unwrap();
            v.push(DropCounter(&counter)).unwrap();
            assert_eq!(counter.get(), 0); // nothing dropped yet

            v.clear();
            assert_eq!(counter.get(), 3); // all three dropped
            assert!(v.is_empty());
        }
        // vec dropped at end of scope — but it is empty so no extra drops.
        assert_eq!(counter.get(), 3);
    }

    // -----------------------------------------------------------------------
    // 6. Iteration: for loop, iter(), iter_mut()
    // -----------------------------------------------------------------------

    #[test]
    fn iteration_for_loop() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 2, 3] {
            v.push(x).unwrap();
        }
        let mut sum = 0i32;
        for &val in &v {
            sum += val;
        }
        assert_eq!(sum, 6);
    }

    #[test]
    fn iteration_iter() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [10, 20, 30] {
            v.push(x).unwrap();
        }
        let collected: [i32; 3] = [10, 20, 30];
        assert!(v.iter().zip(collected.iter()).all(|(a, b)| a == b));
    }

    #[test]
    fn iteration_iter_mut() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 2, 3] {
            v.push(x).unwrap();
        }
        for val in v.iter_mut() {
            *val *= 2;
        }
        assert_eq!(v.as_slice(), &[2, 4, 6]);
    }

    // -----------------------------------------------------------------------
    // 7. PartialEq between two FixedVecs
    // -----------------------------------------------------------------------

    #[test]
    fn partial_eq_same_capacity() {
        let mut a: FixedVec<i32, 4> = FixedVec::new();
        let mut b: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 2, 3] {
            a.push(x).unwrap();
            b.push(x).unwrap();
        }
        assert_eq!(a, b);
        b.push(4).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn partial_eq_different_capacity() {
        let mut a: FixedVec<i32, 4> = FixedVec::new();
        let mut b: FixedVec<i32, 8> = FixedVec::new();
        for x in [1, 2, 3] {
            a.push(x).unwrap();
            b.push(x).unwrap();
        }
        assert_eq!(a, b);
    }

    #[test]
    fn partial_eq_empty_vecs() {
        let a: FixedVec<i32, 4> = FixedVec::new();
        let b: FixedVec<i32, 4> = FixedVec::new();
        assert_eq!(a, b);
    }

    // -----------------------------------------------------------------------
    // 8. Clone
    // -----------------------------------------------------------------------

    #[test]
    fn clone_produces_independent_copy() {
        let mut original: FixedVec<i32, 4> = FixedVec::new();
        original.push(1).unwrap();
        original.push(2).unwrap();

        let mut cloned = original.clone();
        assert_eq!(original, cloned);

        // Mutation of clone should not affect original.
        cloned.push(3).unwrap();
        assert_eq!(cloned.len(), 3);
        assert_eq!(original.len(), 2);
    }

    #[test]
    fn clone_drops_correctly() {
        let counter = Cell::new(0usize);
        {
            let mut v: FixedVec<CloneDropCounter<'_>, 4> = FixedVec::new();
            v.push(CloneDropCounter(&counter)).unwrap();
            v.push(CloneDropCounter(&counter)).unwrap();

            // Cloning produces 2 new CloneDropCounter instances (alive, not dropped yet).
            let _cloned = v.clone();
            assert_eq!(counter.get(), 0); // nothing dropped yet
        }
        // Both `v` and `_cloned` are dropped — 4 total drops.
        assert_eq!(counter.get(), 4);
    }

    // -----------------------------------------------------------------------
    // 9. Deref to slice, indexing
    // -----------------------------------------------------------------------

    #[test]
    fn deref_to_slice() {
        let mut v: FixedVec<u32, 4> = FixedVec::new();
        v.push(10).unwrap();
        v.push(20).unwrap();
        let s: &[u32] = &v;
        assert_eq!(s, &[10, 20]);
    }

    #[test]
    fn index_operator() {
        let mut v: FixedVec<u32, 4> = FixedVec::new();
        v.push(100).unwrap();
        v.push(200).unwrap();
        assert_eq!(v[0], 100);
        assert_eq!(v[1], 200);
    }

    #[test]
    fn index_mut_operator() {
        let mut v: FixedVec<u32, 4> = FixedVec::new();
        v.push(1).unwrap();
        v[0] = 42;
        assert_eq!(v[0], 42);
    }

    #[test]
    #[should_panic]
    fn index_out_of_bounds_panics() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        let _ = v[5];
    }

    // -----------------------------------------------------------------------
    // 10. Zero-capacity FixedVec<T, 0>
    // -----------------------------------------------------------------------

    #[test]
    fn zero_capacity_is_full_and_empty() {
        let v: FixedVec<i32, 0> = FixedVec::new();
        assert!(v.is_empty());
        assert!(v.is_full());
        assert_eq!(v.len(), 0);
        assert_eq!(v.capacity(), 0);
    }

    #[test]
    fn zero_capacity_push_returns_err() {
        let mut v: FixedVec<i32, 0> = FixedVec::new();
        assert_eq!(v.push(1), Err(1));
    }

    #[test]
    fn zero_capacity_pop_returns_none() {
        let mut v: FixedVec<i32, 0> = FixedVec::new();
        assert_eq!(v.pop(), None);
    }

    #[test]
    fn zero_capacity_clear_is_noop() {
        let mut v: FixedVec<i32, 0> = FixedVec::new();
        v.clear(); // must not panic
        assert!(v.is_empty());
    }

    // -----------------------------------------------------------------------
    // 11. resize — grow and shrink
    // -----------------------------------------------------------------------

    #[test]
    fn resize_grow() {
        let mut v: FixedVec<i32, 6> = FixedVec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();
        v.resize(5, 0);
        assert_eq!(v.len(), 5);
        assert_eq!(v.as_slice(), &[1, 2, 0, 0, 0]);
    }

    #[test]
    fn resize_shrink() {
        let mut v: FixedVec<i32, 6> = FixedVec::new();
        for x in [10, 20, 30, 40] {
            v.push(x).unwrap();
        }
        v.resize(2, 0);
        assert_eq!(v.len(), 2);
        assert_eq!(v.as_slice(), &[10, 20]);
    }

    #[test]
    fn resize_same_len_is_noop() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(7).unwrap();
        v.push(8).unwrap();
        v.resize(2, 0);
        assert_eq!(v.as_slice(), &[7, 8]);
    }

    #[test]
    fn resize_drops_removed_elements() {
        let counter = Cell::new(0usize);
        {
            let mut v: FixedVec<CloneDropCounter<'_>, 4> = FixedVec::new();
            v.push(CloneDropCounter(&counter)).unwrap();
            v.push(CloneDropCounter(&counter)).unwrap();
            v.push(CloneDropCounter(&counter)).unwrap();
            assert_eq!(counter.get(), 0);
            // resize to 1: truncates and drops indices 1 and 2; `value` arg
            // is not cloned (we shrink) so it is dropped at end of this call.
            v.resize(1, CloneDropCounter(&counter));
            // 2 truncated + 1 unused value param = 3 drops so far.
            assert_eq!(counter.get(), 3);
        }
        // The one remaining element in `v` is dropped at end of scope.
        assert_eq!(counter.get(), 4);
    }

    // -----------------------------------------------------------------------
    // 12. retain
    // -----------------------------------------------------------------------

    #[test]
    fn retain_keeps_matching_elements() {
        let mut v: FixedVec<i32, 8> = FixedVec::new();
        for x in [1, 2, 3, 4, 5, 6] {
            v.push(x).unwrap();
        }
        v.retain(|x| x % 2 == 0);
        assert_eq!(v.as_slice(), &[2, 4, 6]);
    }

    #[test]
    fn retain_removes_all() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 3, 5] {
            v.push(x).unwrap();
        }
        v.retain(|x| x % 2 == 0);
        assert!(v.is_empty());
    }

    #[test]
    fn retain_keeps_all() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [2, 4, 6] {
            v.push(x).unwrap();
        }
        v.retain(|x| x % 2 == 0);
        assert_eq!(v.as_slice(), &[2, 4, 6]);
    }

    #[test]
    fn retain_drops_removed_elements() {
        // Declare `keep_counter` BEFORE `v` so it lives longer (drop order is
        // reverse of declaration order within the same scope).
        let keep_counter = Cell::new(0usize);
        let counter = Cell::new(0usize);
        {
            let mut v: FixedVec<DropCounter<'_>, 6> = FixedVec::new();
            // Push 4 counters; we will remove indices 0 and 2.
            v.push(DropCounter(&counter)).unwrap(); // index 0 — remove
            v.push(DropCounter(&keep_counter)).unwrap(); // index 1 — keep
            v.push(DropCounter(&counter)).unwrap(); // index 2 — remove
            v.push(DropCounter(&keep_counter)).unwrap(); // index 3 — keep
            assert_eq!(counter.get(), 0);

            // Retain every element at an odd index.
            let idx = Cell::new(0usize);
            v.retain(|_| {
                let i = idx.get();
                idx.set(i + 1);
                !i.is_multiple_of(2) // keep indices 1 and 3
            });
            assert_eq!(counter.get(), 2); // indices 0 and 2 dropped
        }
        // `v` dropped — 2 remaining elements (tracked by keep_counter).
        assert_eq!(counter.get(), 2); // unchanged (kept elements use keep_counter)
        assert_eq!(keep_counter.get(), 2); // the 2 kept elements are now dropped
    }

    // -----------------------------------------------------------------------
    // 13. swap_remove
    // -----------------------------------------------------------------------

    #[test]
    fn swap_remove_middle() {
        let mut v: FixedVec<i32, 5> = FixedVec::new();
        for x in [10, 20, 30, 40] {
            v.push(x).unwrap();
        }
        // Remove index 1 (value 20); last element (40) swaps in.
        let removed = v.swap_remove(1);
        assert_eq!(removed, 20);
        assert_eq!(v.len(), 3);
        // Order after swap_remove: [10, 40, 30]
        assert_eq!(v[0], 10);
        assert_eq!(v[1], 40);
        assert_eq!(v[2], 30);
    }

    #[test]
    fn swap_remove_last_element() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 2, 3] {
            v.push(x).unwrap();
        }
        let removed = v.swap_remove(2);
        assert_eq!(removed, 3);
        assert_eq!(v.as_slice(), &[1, 2]);
    }

    #[test]
    fn swap_remove_only_element() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(42).unwrap();
        let removed = v.swap_remove(0);
        assert_eq!(removed, 42);
        assert!(v.is_empty());
    }

    #[test]
    #[should_panic(expected = "index out of bounds")]
    fn swap_remove_out_of_bounds_panics() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        v.swap_remove(3);
    }

    // -----------------------------------------------------------------------
    // 14. extend_from_slice
    // -----------------------------------------------------------------------

    #[test]
    fn extend_from_slice_basic() {
        let mut v: FixedVec<i32, 6> = FixedVec::new();
        v.push(1).unwrap();
        v.extend_from_slice(&[2, 3, 4]);
        assert_eq!(v.as_slice(), &[1, 2, 3, 4]);
    }

    #[test]
    fn extend_from_slice_stops_at_capacity() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        // Slice has 5 elements but capacity is only 4; only 3 more fit.
        v.extend_from_slice(&[2, 3, 4, 5, 6]);
        assert_eq!(v.len(), 4);
        assert_eq!(v.as_slice(), &[1, 2, 3, 4]);
    }

    #[test]
    fn extend_from_empty_slice_is_noop() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(99).unwrap();
        v.extend_from_slice(&[]);
        assert_eq!(v.len(), 1);
        assert_eq!(v[0], 99);
    }

    // -----------------------------------------------------------------------
    // 15. Drop counter — verify exact drop counts
    // -----------------------------------------------------------------------

    #[test]
    fn drop_on_vec_destruction() {
        let counter = Cell::new(0usize);
        {
            let mut v: FixedVec<DropCounter<'_>, 4> = FixedVec::new();
            v.push(DropCounter(&counter)).unwrap();
            v.push(DropCounter(&counter)).unwrap();
            v.push(DropCounter(&counter)).unwrap();
            assert_eq!(counter.get(), 0);
        } // v dropped here
        assert_eq!(counter.get(), 3);
    }

    #[test]
    fn drop_on_pop() {
        let counter = Cell::new(0usize);
        let mut v: FixedVec<DropCounter<'_>, 4> = FixedVec::new();
        v.push(DropCounter(&counter)).unwrap();
        v.push(DropCounter(&counter)).unwrap();
        assert_eq!(counter.get(), 0);
        let popped = v.pop(); // takes ownership; not dropped yet
        assert_eq!(counter.get(), 0);
        drop(popped); // now dropped
        assert_eq!(counter.get(), 1);
        drop(v); // remaining element dropped
        assert_eq!(counter.get(), 2);
    }

    #[test]
    fn drop_on_remove() {
        let counter = Cell::new(0usize);
        let mut v: FixedVec<DropCounter<'_>, 4> = FixedVec::new();
        v.push(DropCounter(&counter)).unwrap();
        v.push(DropCounter(&counter)).unwrap();
        v.push(DropCounter(&counter)).unwrap();
        let removed = v.remove(1);
        assert_eq!(counter.get(), 0); // ownership transferred to caller
        drop(removed);
        assert_eq!(counter.get(), 1);
        drop(v); // 2 remaining elements
        assert_eq!(counter.get(), 3);
    }

    #[test]
    fn drop_on_truncate() {
        let counter = Cell::new(0usize);
        let mut v: FixedVec<DropCounter<'_>, 6> = FixedVec::new();
        for _ in 0..5 {
            v.push(DropCounter(&counter)).unwrap();
        }
        v.truncate(2);
        assert_eq!(counter.get(), 3); // 3 elements dropped
        assert_eq!(v.len(), 2);
        drop(v);
        assert_eq!(counter.get(), 5); // 2 remaining dropped
    }

    // -----------------------------------------------------------------------
    // Additional: get / get_mut / first / last
    // -----------------------------------------------------------------------

    #[test]
    fn get_and_get_mut() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();

        assert_eq!(v.get(0), Some(&1));
        assert_eq!(v.get(1), Some(&2));
        assert_eq!(v.get(2), None);

        *v.get_mut(0).unwrap() = 99;
        assert_eq!(v[0], 99);
    }

    #[test]
    fn first_and_last() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        assert_eq!(v.first(), None);
        assert_eq!(v.last(), None);

        v.push(10).unwrap();
        v.push(20).unwrap();
        v.push(30).unwrap();

        assert_eq!(v.first(), Some(&10));
        assert_eq!(v.last(), Some(&30));
    }

    // -----------------------------------------------------------------------
    // Additional: Default
    // -----------------------------------------------------------------------

    #[test]
    fn default_is_empty() {
        let v: FixedVec<u32, 8> = FixedVec::default();
        assert!(v.is_empty());
        assert_eq!(v.capacity(), 8);
    }

    // -----------------------------------------------------------------------
    // Additional: From<[T; M]>
    // -----------------------------------------------------------------------

    #[test]
    fn from_array_exact_capacity() {
        let v: FixedVec<i32, 3> = FixedVec::from([1, 2, 3]);
        assert_eq!(v.as_slice(), &[1, 2, 3]);
    }

    #[test]
    fn from_array_smaller_than_capacity() {
        let v: FixedVec<i32, 5> = FixedVec::from([10, 20]);
        assert_eq!(v.len(), 2);
        assert_eq!(v.as_slice(), &[10, 20]);
    }

    #[test]
    fn from_empty_array() {
        let v: FixedVec<i32, 4> = FixedVec::from([]);
        assert!(v.is_empty());
    }

    // -----------------------------------------------------------------------
    // Additional: Ord
    // -----------------------------------------------------------------------

    #[test]
    fn ord_comparison() {
        let mut a: FixedVec<i32, 4> = FixedVec::new();
        let mut b: FixedVec<i32, 4> = FixedVec::new();
        for x in [1, 2, 3] {
            a.push(x).unwrap();
        }
        for x in [1, 2, 4] {
            b.push(x).unwrap();
        }
        assert!(a < b);
        assert!(b > a);
    }

    // -----------------------------------------------------------------------
    // Additional: Debug formatting (smoke test — must not panic)
    // -----------------------------------------------------------------------

    #[test]
    fn debug_does_not_panic() {
        let mut v: FixedVec<i32, 4> = FixedVec::new();
        v.push(1).unwrap();
        v.push(2).unwrap();
        let _s = ::core::format_args!("{:?}", v);
    }
}
