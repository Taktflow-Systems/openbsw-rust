// Copyright 2024 Accenture.
// SPDX-License-Identifier: Apache-2.0

//! Binary-tree buddy memory allocator — `#![no_std]`-compatible port of
//! OpenBSW `util/memory/BuddyMemoryManager`.
//!
//! # Design
//!
//! A buddy allocator divides a fixed pool into power-of-2 blocks and tracks
//! their state in a complete binary tree stored in a flat array.  Buddies
//! (sibling blocks at the same level) are automatically coalesced when both
//! become free.
//!
//! ## Const generics
//!
//! Because `generic_const_exprs` is still nightly-only, the struct takes three
//! explicit const parameters rather than computing them from each other:
//!
//! - `N` — number of minimum-sized (leaf) blocks; **must be a power of 2**.
//! - `T` — tree node count; **must equal `2*N - 1`**.  Use [`buddy_tree_nodes`].
//! - `POOL_SIZE` — total pool size in bytes; **must equal `N * min_block_size`**,
//!   i.e. must be a multiple of `N` and must be `>= N`.
//!
//! Minimum block size is `POOL_SIZE / N` bytes.
//!
//! ## Example
//!
//! ```rust
//! use bsw_util::buddy::{BuddyAllocator, buddy_tree_nodes};
//!
//! // 16 leaves x 64 bytes = 1 KiB pool, 31 tree nodes.
//! const N: usize = 16;
//! const T: usize = buddy_tree_nodes(N);
//! const POOL: usize = 1024;
//!
//! let mut alloc: BuddyAllocator<N, T, POOL> = BuddyAllocator::new();
//! assert_eq!(BuddyAllocator::<N, T, POOL>::min_block_size(), 64);
//!
//! let block = alloc.acquire(100).unwrap(); // rounds up to 128 bytes
//! assert_eq!(block.len(), 128);
//! let ptr = block.as_ptr(); // capture raw pointer; mutable borrow ends here
//!
//! let offset = alloc.block_offset(ptr);
//! alloc.release_by_offset(offset);
//!
//! assert_eq!(alloc.largest_available(), 1024);
//! ```

// ---------------------------------------------------------------------------
// NodeState
// ---------------------------------------------------------------------------

/// State of a single node in the buddy tree.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum NodeState {
    /// The block represented by this node is entirely free.
    Free = 0,
    /// The block is partially allocated — at least one child is used.
    Split = 1,
    /// The block has been directly allocated to a caller.
    Allocated = 2,
}

// ---------------------------------------------------------------------------
// Public helper: tree node count
// ---------------------------------------------------------------------------

/// Returns the number of tree nodes required for `n` leaf blocks.
///
/// Use this in type annotations to avoid spelling out `2*N - 1` manually:
///
/// ```rust
/// use bsw_util::buddy::{BuddyAllocator, buddy_tree_nodes};
/// const N: usize = 8;
/// let _: BuddyAllocator<N, { buddy_tree_nodes(N) }, 256> = BuddyAllocator::new();
/// ```
///
/// # Panics (compile time)
///
/// Panics if `n == 0`.
pub const fn buddy_tree_nodes(n: usize) -> usize {
    2 * n - 1
}

// ---------------------------------------------------------------------------
// BuddyAllocator
// ---------------------------------------------------------------------------

/// Fixed-size buddy memory allocator with inline pool and tree storage.
///
/// - `N` — number of minimum-sized (leaf) blocks; must be a power of 2.
/// - `T` — tree node count; must equal `2*N - 1` (use [`buddy_tree_nodes`]).
/// - `POOL_SIZE` — total pool size in bytes; must be a multiple of `N` and
///   must be `>= N`.
///
/// All invariants are checked with compile-time assertions inside [`new`].
///
/// [`new`]: BuddyAllocator::new
pub struct BuddyAllocator<const N: usize, const T: usize, const POOL_SIZE: usize> {
    /// Inline byte pool.
    pool: [u8; POOL_SIZE],
    /// Flat binary tree tracking block states (root = index 0).
    tree: [NodeState; T],
}

// SAFETY: BuddyAllocator owns all of its memory inline.  It contains no raw
// pointers to external data, so Send + Sync are trivially satisfied as long as
// the pool bytes are treated as plain data (which they are — no interior
// mutability, no thread-local state).
unsafe impl<const N: usize, const T: usize, const POOL_SIZE: usize> Send
    for BuddyAllocator<N, T, POOL_SIZE>
{
}
unsafe impl<const N: usize, const T: usize, const POOL_SIZE: usize> Sync
    for BuddyAllocator<N, T, POOL_SIZE>
{
}

impl<const N: usize, const T: usize, const POOL_SIZE: usize> BuddyAllocator<N, T, POOL_SIZE> {
    // -----------------------------------------------------------------------
    // Compile-time invariant checks
    // -----------------------------------------------------------------------

    /// Evaluated at compile time by [`new`]; asserts:
    /// - `N` is a power of 2 and `> 0`
    /// - `T == 2*N - 1`
    /// - `POOL_SIZE >= N`
    /// - `POOL_SIZE % N == 0`
    const CHECK: () = {
        assert!(N > 0, "BuddyAllocator: N must be > 0");
        assert!(N.is_power_of_two(), "BuddyAllocator: N must be a power of 2");
        assert!(T == 2 * N - 1, "BuddyAllocator: T must equal 2*N - 1");
        assert!(POOL_SIZE >= N, "BuddyAllocator: POOL_SIZE must be >= N");
        assert!(POOL_SIZE.is_multiple_of(N), "BuddyAllocator: POOL_SIZE must be divisible by N");
    };

    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /// Creates a new, fully-free allocator with the pool zeroed.
    ///
    /// # Panics (compile time)
    ///
    /// Panics at compile time if any const-generic invariant is violated.
    pub const fn new() -> Self {
        // Trigger compile-time checks.
        let () = Self::CHECK;
        Self {
            pool: [0u8; POOL_SIZE],
            tree: [NodeState::Free; T],
        }
    }

    // -----------------------------------------------------------------------
    // Const queries
    // -----------------------------------------------------------------------

    /// Size of the smallest allocatable block in bytes (`POOL_SIZE / N`).
    pub const fn min_block_size() -> usize {
        POOL_SIZE / N
    }

    /// Total pool size in bytes.
    pub const fn pool_size() -> usize {
        POOL_SIZE
    }

    // -----------------------------------------------------------------------
    // Acquire
    // -----------------------------------------------------------------------

    /// Allocates at least `size` bytes and returns a mutable slice into the
    /// pool.
    ///
    /// The actual allocated length is rounded up to the next power of 2,
    /// clamped to at least [`min_block_size`] and at most [`pool_size`].
    ///
    /// Returns `None` if no contiguous block large enough is available.
    ///
    /// [`min_block_size`]: BuddyAllocator::min_block_size
    /// [`pool_size`]: BuddyAllocator::pool_size
    pub fn acquire(&mut self, size: usize) -> Option<&mut [u8]> {
        let target = Self::required_block_size(size);
        let block_offset = self.find_and_allocate(0, POOL_SIZE, 0, target)?;
        // SAFETY: `block_offset` and `block_offset + target` are within
        // `self.pool` because `find_and_allocate` only returns offsets in
        // `[0, POOL_SIZE)` with `block_offset + target <= POOL_SIZE`.  The
        // returned slice covers exactly the allocated node's region and no
        // other live `&mut [u8]` can alias this range while `&mut self` is
        // held.
        Some(unsafe {
            core::slice::from_raw_parts_mut(self.pool.as_mut_ptr().add(block_offset), target)
        })
    }

    // -----------------------------------------------------------------------
    // Release
    // -----------------------------------------------------------------------

    /// Releases a block previously returned by [`acquire`].
    ///
    /// # Safety
    ///
    /// `ptr` must be the exact pointer that was returned as the start of a
    /// slice by a prior call to [`acquire`] on *this* allocator, and the block
    /// must not have been released already.
    ///
    /// [`acquire`]: BuddyAllocator::acquire
    pub unsafe fn release(&mut self, ptr: *mut u8) {
        // SAFETY: caller guarantees ptr came from this allocator's pool.
        let pool_base = self.pool.as_ptr() as usize;
        let ptr_addr = ptr as usize;
        debug_assert!(
            ptr_addr >= pool_base && ptr_addr < pool_base + POOL_SIZE,
            "BuddyAllocator::release: pointer outside pool"
        );
        let offset = ptr_addr - pool_base;
        self.release_by_offset(offset);
    }

    /// Releases a block identified by its byte offset from the start of the
    /// pool.
    ///
    /// The offset must have been obtained from the slice returned by [`acquire`]
    /// via [`block_offset`].
    ///
    /// Silently does nothing if no allocated node matches `offset`.
    ///
    /// [`acquire`]: BuddyAllocator::acquire
    /// [`block_offset`]: BuddyAllocator::block_offset
    pub fn release_by_offset(&mut self, offset: usize) {
        self.find_and_free(0, POOL_SIZE, 0, offset);
    }

    // -----------------------------------------------------------------------
    // Queries
    // -----------------------------------------------------------------------

    /// Returns the byte offset of a block within the pool.
    ///
    /// Pass the raw pointer obtained from `block.as_ptr()` immediately after
    /// calling [`acquire`].  Using a raw pointer avoids holding the mutable
    /// borrow of `self` while querying the offset.
    ///
    /// # Safety
    ///
    /// `ptr` must point into this allocator's pool (i.e. it must have been
    /// obtained from a slice returned by [`acquire`] on *this* allocator).
    ///
    /// [`acquire`]: BuddyAllocator::acquire
    /// [`release_by_offset`]: BuddyAllocator::release_by_offset
    pub fn block_offset(&self, ptr: *const u8) -> usize {
        let pool_base = self.pool.as_ptr() as usize;
        let block_ptr = ptr as usize;
        debug_assert!(
            block_ptr >= pool_base && block_ptr < pool_base + POOL_SIZE,
            "BuddyAllocator::block_offset: pointer outside pool"
        );
        block_ptr - pool_base
    }

    /// Returns the size of the largest contiguous free block currently
    /// available, in bytes.  Returns 0 when the pool is fully allocated.
    pub fn largest_available(&self) -> usize {
        self.largest_free(0, POOL_SIZE)
    }

    // -----------------------------------------------------------------------
    // Internal: required block size helper
    // -----------------------------------------------------------------------

    /// Rounds `size` up to the next power-of-2 block, clamped to
    /// `[min_block_size, POOL_SIZE]`.
    #[inline]
    const fn required_block_size(size: usize) -> usize {
        let min = Self::min_block_size();
        if size <= min {
            return min;
        }
        if size >= POOL_SIZE {
            return POOL_SIZE;
        }
        size.next_power_of_two()
    }

    // -----------------------------------------------------------------------
    // Internal: recursive allocate
    // -----------------------------------------------------------------------

    /// Searches the subtree rooted at `node` for a free block of `target`
    /// bytes, marks it `Allocated`, and returns its pool byte offset on
    /// success.
    ///
    /// - `node`         — current tree node index (0-based).
    /// - `block_size`   — size in bytes of the block this node represents.
    /// - `block_offset` — byte offset of this node's block from pool start.
    /// - `target`       — desired allocation size (power-of-2 already).
    fn find_and_allocate(
        &mut self,
        node: usize,
        block_size: usize,
        block_offset: usize,
        target: usize,
    ) -> Option<usize> {
        if block_size < target {
            return None;
        }
        match self.tree[node] {
            NodeState::Allocated => None,
            NodeState::Free => {
                if block_size == target {
                    self.tree[node] = NodeState::Allocated;
                    return Some(block_offset);
                }
                // block_size > target: split and recurse.
                self.tree[node] = NodeState::Split;
                let half = block_size / 2;
                let left = 2 * node + 1;
                let right = 2 * node + 2;
                if let Some(off) = self.find_and_allocate(left, half, block_offset, target) {
                    return Some(off);
                }
                if let Some(off) =
                    self.find_and_allocate(right, half, block_offset + half, target)
                {
                    return Some(off);
                }
                // Neither child could satisfy — undo the split.
                self.tree[node] = NodeState::Free;
                None
            }
            NodeState::Split => {
                let half = block_size / 2;
                let left = 2 * node + 1;
                let right = 2 * node + 2;
                if let Some(off) = self.find_and_allocate(left, half, block_offset, target) {
                    return Some(off);
                }
                self.find_and_allocate(right, half, block_offset + half, target)
            }
        }
    }

    // -----------------------------------------------------------------------
    // Internal: recursive free + coalesce
    // -----------------------------------------------------------------------

    /// Finds the `Allocated` node whose block starts at `target_offset`,
    /// marks it `Free`, and coalesces buddies on the way back up.
    ///
    /// Returns `true` when a node was freed (used to trigger coalescing in the
    /// parent call frame).
    fn find_and_free(
        &mut self,
        node: usize,
        block_size: usize,
        block_offset: usize,
        target_offset: usize,
    ) -> bool {
        // Prune: target_offset must lie within this node's range.
        if target_offset < block_offset || target_offset >= block_offset + block_size {
            return false;
        }
        match self.tree[node] {
            NodeState::Free => false,
            NodeState::Allocated => {
                self.tree[node] = NodeState::Free;
                true
            }
            NodeState::Split => {
                let half = block_size / 2;
                let left = 2 * node + 1;
                let right = 2 * node + 2;
                let freed = if target_offset < block_offset + half {
                    self.find_and_free(left, half, block_offset, target_offset)
                } else {
                    self.find_and_free(right, half, block_offset + half, target_offset)
                };
                if freed
                    && self.tree[left] == NodeState::Free
                    && self.tree[right] == NodeState::Free
                {
                    self.tree[node] = NodeState::Free;
                }
                freed
            }
        }
    }

    // -----------------------------------------------------------------------
    // Internal: largest free block query
    // -----------------------------------------------------------------------

    fn largest_free(&self, node: usize, block_size: usize) -> usize {
        match self.tree[node] {
            NodeState::Free => block_size,
            NodeState::Allocated => 0,
            NodeState::Split => {
                let half = block_size / 2;
                let left = 2 * node + 1;
                let right = 2 * node + 2;
                self.largest_free(left, half).max(self.largest_free(right, half))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<const N: usize, const T: usize, const POOL_SIZE: usize> Default
    for BuddyAllocator<N, T, POOL_SIZE>
{
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Convenient type aliases.

    /// 1 leaf x 64 bytes = 64 B pool.  min_block = 64 B.
    type Alloc1 = BuddyAllocator<1, { buddy_tree_nodes(1) }, 64>;

    /// 4 leaves x 16 bytes = 64 B pool.  min_block = 16 B.
    type Alloc4 = BuddyAllocator<4, { buddy_tree_nodes(4) }, 64>;

    /// 8 leaves x 8 bytes = 64 B pool.  min_block = 8 B.
    type Alloc8 = BuddyAllocator<8, { buddy_tree_nodes(8) }, 64>;

    /// 16 leaves x 64 bytes = 1 KiB pool.  min_block = 64 B.
    type Alloc16 = BuddyAllocator<16, { buddy_tree_nodes(16) }, 1024>;

    // -----------------------------------------------------------------------
    // 1. New allocator — full pool available
    // -----------------------------------------------------------------------

    #[test]
    fn new_allocator_full_pool_available() {
        let alloc = Alloc16::new();
        assert_eq!(
            alloc.largest_available(),
            1024,
            "fresh allocator should offer the full pool"
        );
    }

    // -----------------------------------------------------------------------
    // 2. Acquire min block succeeds
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_min_block_succeeds() {
        let mut alloc = Alloc16::new();
        let block = alloc.acquire(1).unwrap();
        assert_eq!(
            block.len(),
            64,
            "single-byte request rounds up to min_block (64)"
        );
    }

    // -----------------------------------------------------------------------
    // 3. Acquire full pool size succeeds
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_full_pool_size_succeeds() {
        let mut alloc = Alloc16::new();
        let block = alloc.acquire(1024).unwrap();
        assert_eq!(block.len(), 1024);
    }

    // -----------------------------------------------------------------------
    // 4. Acquire 0 returns min block
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_zero_returns_min_block() {
        let mut alloc = Alloc16::new();
        let block = alloc.acquire(0).unwrap();
        assert_eq!(block.len(), Alloc16::min_block_size());
    }

    // -----------------------------------------------------------------------
    // 5. Acquire when full returns None
    // -----------------------------------------------------------------------

    #[test]
    fn acquire_when_full_returns_none() {
        let mut alloc = Alloc4::new();
        for _ in 0..4 {
            assert!(alloc.acquire(1).is_some());
        }
        assert_eq!(alloc.largest_available(), 0);
        assert!(alloc.acquire(1).is_none());
    }

    // -----------------------------------------------------------------------
    // 6. Release and re-acquire works
    // -----------------------------------------------------------------------

    #[test]
    fn release_and_re_acquire_works() {
        let mut alloc = Alloc4::new();
        let b0 = alloc.acquire(1).unwrap();
        let ptr0 = b0.as_ptr();
        let _ = b0;
        let offset = alloc.block_offset(ptr0);
        alloc.release_by_offset(offset);
        assert!(alloc.acquire(1).is_some());
    }

    // -----------------------------------------------------------------------
    // 7. Buddy coalescing — both halves freed, full block restored
    // -----------------------------------------------------------------------

    #[test]
    fn buddy_coalescing_restores_full_block() {
        type A = BuddyAllocator<2, { buddy_tree_nodes(2) }, 64>;
        let mut alloc = A::new();

        let b0 = alloc.acquire(32).unwrap();
        let ptr0 = b0.as_ptr();
        let _ = b0;
        let off0 = alloc.block_offset(ptr0);

        let b1 = alloc.acquire(32).unwrap();
        let ptr1 = b1.as_ptr();
        let _ = b1;
        let off1 = alloc.block_offset(ptr1);

        assert_eq!(alloc.largest_available(), 0);

        alloc.release_by_offset(off0);
        assert_eq!(alloc.largest_available(), 32);

        alloc.release_by_offset(off1);
        assert_eq!(alloc.largest_available(), 64, "buddies must coalesce to full 64 B");
    }

    // -----------------------------------------------------------------------
    // 8. Multiple small allocations fill the pool
    // -----------------------------------------------------------------------

    #[test]
    fn multiple_small_allocations_fill_pool() {
        let mut alloc = Alloc8::new();
        let mut offsets = [0usize; 8];
        for off in &mut offsets {
            let b = alloc.acquire(1).unwrap();
            let ptr = b.as_ptr();
            let _ = b;
            *off = alloc.block_offset(ptr);
        }
        assert_eq!(alloc.largest_available(), 0);
        for off in offsets {
            alloc.release_by_offset(off);
        }
        assert_eq!(alloc.largest_available(), 64);
    }

    // -----------------------------------------------------------------------
    // 9. Largest available decreases / increases with allocations
    // -----------------------------------------------------------------------

    #[test]
    fn largest_available_tracks_allocations() {
        let mut alloc = Alloc16::new();
        assert_eq!(alloc.largest_available(), 1024);

        let b = alloc.acquire(512).unwrap();
        let ptr = b.as_ptr();
        let _ = b;
        let off = alloc.block_offset(ptr);
        assert_eq!(alloc.largest_available(), 512);

        let b2 = alloc.acquire(512).unwrap();
        let ptr2 = b2.as_ptr();
        let _ = b2;
        let off2 = alloc.block_offset(ptr2);
        assert_eq!(alloc.largest_available(), 0);

        alloc.release_by_offset(off);
        alloc.release_by_offset(off2);
        assert_eq!(alloc.largest_available(), 1024);
    }

    // -----------------------------------------------------------------------
    // 10. Release by offset correct
    // -----------------------------------------------------------------------

    #[test]
    fn release_by_offset_correct() {
        let mut alloc = Alloc4::new();
        let b = alloc.acquire(16).unwrap();
        let ptr = b.as_ptr();
        let _ = b;
        let offset = alloc.block_offset(ptr);
        assert!(offset < 64);
        assert_eq!(offset % 16, 0);
        alloc.release_by_offset(offset);
        assert_eq!(alloc.largest_available(), 64);
    }

    // -----------------------------------------------------------------------
    // 11. Fragmentation scenario
    // -----------------------------------------------------------------------

    #[test]
    fn fragmentation_scenario() {
        type A = BuddyAllocator<4, { buddy_tree_nodes(4) }, 32>;
        let mut alloc = A::new();

        let b0 = alloc.acquire(8).unwrap();
        let ptr0 = b0.as_ptr();
        let _ = b0;
        let off0 = alloc.block_offset(ptr0);

        let b1 = alloc.acquire(8).unwrap();
        let ptr1 = b1.as_ptr();
        let _ = b1;
        let off1 = alloc.block_offset(ptr1);

        let b2 = alloc.acquire(8).unwrap();
        let _ = b2; // offset not needed

        let b3 = alloc.acquire(8).unwrap();
        let ptr3 = b3.as_ptr();
        let _ = b3;
        let off3 = alloc.block_offset(ptr3);

        assert_eq!(alloc.largest_available(), 0);

        // Free the two left-hand leaves; they are buddies and should coalesce.
        alloc.release_by_offset(off0);
        alloc.release_by_offset(off1);
        assert_eq!(alloc.largest_available(), 16);

        alloc.release_by_offset(off3);
        // Still fragmented on the right side; largest = 16.
        assert_eq!(alloc.largest_available(), 16);

        // Allocate the coalesced 16-byte block.
        let big = alloc.acquire(16).unwrap();
        assert_eq!(big.len(), 16);
    }

    // -----------------------------------------------------------------------
    // 12. Power-of-2 rounding
    // -----------------------------------------------------------------------

    #[test]
    fn power_of_two_rounding() {
        let mut alloc = Alloc8::new();
        // 5 rounds up to 8 (= min_block).
        let b = alloc.acquire(5).unwrap();
        assert_eq!(b.len(), 8);
        // 9 rounds up to 16.
        let b2 = alloc.acquire(9).unwrap();
        assert_eq!(b2.len(), 16);
    }

    // -----------------------------------------------------------------------
    // 13. Allocate-release-allocate cycle — no corruption
    // -----------------------------------------------------------------------

    #[test]
    fn allocate_release_allocate_no_corruption() {
        let mut alloc = Alloc8::new();
        let b = alloc.acquire(8).unwrap();
        b.iter_mut().enumerate().for_each(|(i, byte)| *byte = i as u8);
        let ptr = b.as_ptr();
        let _ = b;
        let offset = alloc.block_offset(ptr);
        alloc.release_by_offset(offset);

        let b2 = alloc.acquire(8).unwrap();
        b2.fill(0xAB);
        assert!(b2.iter().all(|&x| x == 0xAB));
    }

    // -----------------------------------------------------------------------
    // 14. All leaves allocated — further acquire returns None
    // -----------------------------------------------------------------------

    #[test]
    fn all_leaves_allocated_pool_full() {
        let mut alloc = Alloc4::new();
        let mut offsets = [0usize; 4];
        for off in &mut offsets {
            let b = alloc.acquire(1).unwrap();
            let ptr = b.as_ptr();
            let _ = b;
            *off = alloc.block_offset(ptr);
        }
        assert!(alloc.acquire(1).is_none());
        for off in offsets {
            alloc.release_by_offset(off);
        }
    }

    // -----------------------------------------------------------------------
    // 15. N=1 single-block allocator
    // -----------------------------------------------------------------------

    #[test]
    fn single_block_allocator() {
        let mut alloc = Alloc1::new();
        assert_eq!(Alloc1::min_block_size(), 64);
        assert_eq!(alloc.largest_available(), 64);

        let b = alloc.acquire(1).unwrap();
        assert_eq!(b.len(), 64);
        let ptr = b.as_ptr();
        let _ = b;
        assert_eq!(alloc.largest_available(), 0);

        let offset = alloc.block_offset(ptr);
        alloc.release_by_offset(offset);
        assert_eq!(alloc.largest_available(), 64);

        let b2 = alloc.acquire(64).unwrap();
        assert_eq!(b2.len(), 64);
    }

    // -----------------------------------------------------------------------
    // 16. unsafe release via raw pointer
    // -----------------------------------------------------------------------

    #[test]
    fn unsafe_release_via_raw_pointer() {
        let mut alloc = Alloc4::new();
        let raw_ptr: *mut u8 = {
            let b = alloc.acquire(1).unwrap();
            b.as_mut_ptr()
        };
        // SAFETY: raw_ptr was returned by this allocator's acquire and has not
        // been released yet.
        unsafe { alloc.release(raw_ptr) };
        assert_eq!(alloc.largest_available(), 64);
    }

    // -----------------------------------------------------------------------
    // 17. Default trait produces a fresh allocator
    // -----------------------------------------------------------------------

    #[test]
    fn default_trait_produces_fresh_allocator() {
        let alloc: Alloc16 = BuddyAllocator::default();
        assert_eq!(alloc.largest_available(), 1024);
    }

    // -----------------------------------------------------------------------
    // 18. buddy_tree_nodes helper values
    // -----------------------------------------------------------------------

    #[test]
    fn buddy_tree_nodes_helper_values() {
        assert_eq!(buddy_tree_nodes(1), 1);
        assert_eq!(buddy_tree_nodes(2), 3);
        assert_eq!(buddy_tree_nodes(4), 7);
        assert_eq!(buddy_tree_nodes(8), 15);
        assert_eq!(buddy_tree_nodes(16), 31);
        assert_eq!(buddy_tree_nodes(64), 127);
    }

    // -----------------------------------------------------------------------
    // 19. Const queries
    // -----------------------------------------------------------------------

    #[test]
    fn const_queries_correct() {
        assert_eq!(Alloc16::min_block_size(), 64);
        assert_eq!(Alloc16::pool_size(), 1024);
        assert_eq!(Alloc4::min_block_size(), 16);
        assert_eq!(Alloc4::pool_size(), 64);
    }

    // -----------------------------------------------------------------------
    // 20. Release of unknown offset is silently ignored
    // -----------------------------------------------------------------------

    #[test]
    fn release_unknown_offset_is_no_op() {
        let mut alloc = Alloc4::new();
        // No allocation made; releasing offset 0 must not panic or corrupt.
        alloc.release_by_offset(0);
        assert_eq!(alloc.largest_available(), 64);
    }

    // -----------------------------------------------------------------------
    // Extra: block_offset is pool-relative and block-aligned
    // -----------------------------------------------------------------------

    #[test]
    fn block_offset_is_pool_relative() {
        let mut alloc = Alloc4::new();
        let b = alloc.acquire(16).unwrap();
        let ptr = b.as_ptr();
        let _ = b;
        let offset = alloc.block_offset(ptr);
        assert!(offset < 64);
        assert_eq!(offset % 16, 0);
    }
}
