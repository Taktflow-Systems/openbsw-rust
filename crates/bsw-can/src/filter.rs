// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! CAN receive filters — Rust port of `OpenBSW` `BitFieldFilter` and
//! `IntervalFilter`.
//!
//! Two complementary filter strategies are provided:
//!
//! | Type              | Matching strategy           | ID range  |
//! |-------------------|-----------------------------|-----------|
//! | [`BitFieldFilter`]| 256-byte bitmask            | 0x000–0x7FF (11-bit base only) |
//! | [`IntervalFilter`]| inclusive `[from, to]` range| any `u32` |
//!
//! Both implement the [`Filter`] trait.  An [`IntervalFilter`] can be merged
//! into a [`BitFieldFilter`] via [`Filter::merge_into`], which sets every
//! bit in the mask that falls within the interval's `[from, to]` range.

// ---------------------------------------------------------------------------
// Filter trait
// ---------------------------------------------------------------------------

/// Behaviour shared by all CAN receive filter types.
///
/// Each implementation maintains its own internal state describing which CAN
/// IDs should be accepted.
pub trait Filter {
    /// Adds a single ID to the accept set.
    fn add(&mut self, filter_id: u32);

    /// Adds an inclusive range of IDs to the accept set.
    fn add_range(&mut self, from: u32, to: u32);

    /// Returns `true` if `filter_id` is in the current accept set.
    fn matches(&self, filter_id: u32) -> bool;

    /// Resets the filter to the reject-all state.
    fn clear(&mut self);

    /// Opens the filter to accept every possible ID.
    fn open(&mut self);

    /// Merges this filter's accepted IDs into a [`BitFieldFilter`].
    ///
    /// All IDs accepted by `self` that are within [`BitFieldFilter::MAX_ID`]
    /// will be set in `target`.
    fn merge_into(&self, target: &mut BitFieldFilter);
}

// ---------------------------------------------------------------------------
// BitFieldFilter
// ---------------------------------------------------------------------------

/// A receive filter backed by a 256-byte bitmask covering all 2048 possible
/// 11-bit base CAN IDs (0x000–0x7FF).
///
/// Each ID maps to a single bit: byte index `id / 8`, bit position `id % 8`.
/// IDs outside the 11-bit range (`> 0x7FF`) are **silently ignored** on all
/// mutating operations; [`Filter::matches`] returns `false` for them.
#[derive(Clone, Debug)]
pub struct BitFieldFilter {
    mask: [u8; Self::MASK_SIZE],
}

impl BitFieldFilter {
    /// Maximum ID supported by this filter (11-bit: `0x7FF`).
    pub const MAX_ID: u16 = 0x7FF;

    /// Number of bits in the bitmask: `MAX_ID + 1 = 2048`.
    pub const NUMBER_OF_BITS: u16 = Self::MAX_ID + 1;

    /// Size of the backing byte array: `NUMBER_OF_BITS / 8 = 256`.
    pub const MASK_SIZE: usize = (Self::NUMBER_OF_BITS / 8) as usize;

    /// Creates a new filter in the reject-all state (all mask bytes zero).
    #[inline]
    pub const fn new() -> Self {
        Self {
            mask: [0u8; Self::MASK_SIZE],
        }
    }

    /// Returns a reference to the raw bitmask byte array.
    #[inline]
    pub const fn raw_bit_field(&self) -> &[u8; Self::MASK_SIZE] {
        &self.mask
    }

    // -- Internal bit helpers ------------------------------------------------

    #[inline]
    fn set_bit(&mut self, id: u16) {
        let byte_idx = (id / 8) as usize;
        let bit_pos = id % 8;
        self.mask[byte_idx] |= 1 << bit_pos;
    }

    #[inline]
    fn test_bit(&self, id: u16) -> bool {
        let byte_idx = (id / 8) as usize;
        let bit_pos = id % 8;
        (self.mask[byte_idx] >> bit_pos) & 1 != 0
    }

    // -- Merge helpers (called by IntervalFilter::merge_into) ----------------

    /// ORs another `BitFieldFilter`'s mask into `self`.
    pub fn merge_with_bit_field(&mut self, other: &BitFieldFilter) {
        for (dst, src) in self.mask.iter_mut().zip(other.mask.iter()) {
            *dst |= src;
        }
    }

    /// Sets all bits in `self` that fall within `other`'s `[from, to]` range.
    pub fn merge_with_interval(&mut self, other: &IntervalFilter) {
        if other.from > other.to {
            return; // empty interval — nothing to set
        }
        // `.min(MAX_ID)` guarantees the value fits in u16.
        #[allow(clippy::cast_possible_truncation)]
        let lo = other.from.min(u32::from(Self::MAX_ID)) as u16;
        #[allow(clippy::cast_possible_truncation)]
        let hi = other.to.min(u32::from(Self::MAX_ID)) as u16;
        for id in lo..=hi {
            self.set_bit(id);
        }
    }
}

impl Filter for BitFieldFilter {
    fn add(&mut self, filter_id: u32) {
        if filter_id > u32::from(Self::MAX_ID) {
            return; // out of range — silently ignored
        }
        // Guarded by the range check above; value fits in u16.
        #[allow(clippy::cast_possible_truncation)]
        self.set_bit(filter_id as u16);
    }

    fn add_range(&mut self, from: u32, to: u32) {
        // `.min(MAX_ID)` ensures value <= 0x7FF which fits in u16.
        #[allow(clippy::cast_possible_truncation)]
        let lo = from.min(u32::from(Self::MAX_ID)) as u16;
        #[allow(clippy::cast_possible_truncation)]
        let hi = to.min(u32::from(Self::MAX_ID)) as u16;
        if lo > hi {
            return;
        }
        for id in lo..=hi {
            self.set_bit(id);
        }
    }

    fn matches(&self, filter_id: u32) -> bool {
        if filter_id > u32::from(Self::MAX_ID) {
            return false;
        }
        // Guarded by the range check above; value fits in u16.
        #[allow(clippy::cast_possible_truncation)]
        self.test_bit(filter_id as u16)
    }

    fn clear(&mut self) {
        self.mask = [0u8; Self::MASK_SIZE];
    }

    fn open(&mut self) {
        self.mask = [0xFF_u8; Self::MASK_SIZE];
    }

    fn merge_into(&self, target: &mut BitFieldFilter) {
        target.merge_with_bit_field(self);
    }
}

impl PartialEq for BitFieldFilter {
    fn eq(&self, other: &Self) -> bool {
        self.mask == other.mask
    }
}

impl Eq for BitFieldFilter {}

impl Default for BitFieldFilter {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// IntervalFilter
// ---------------------------------------------------------------------------

/// A receive filter that accepts all IDs in an inclusive `[from, to]` range.
///
/// The empty state is represented by `from = u32::MAX` and `to = 0`, which
/// produces no matches because `from > to`.
///
/// Adding a single ID via [`Filter::add`] extends the range: if the ID is
/// outside the current `[from, to]` interval, `from` is lowered or `to` is
/// raised accordingly.
#[derive(Clone, Debug)]
pub struct IntervalFilter {
    from: u32,
    to: u32,
}

impl IntervalFilter {
    /// Creates a new, empty filter (`from = u32::MAX`, `to = 0`).
    #[inline]
    pub const fn new() -> Self {
        Self {
            from: u32::MAX,
            to: 0,
        }
    }

    /// Creates a filter that accepts the inclusive range `[from, to]`.
    #[inline]
    pub const fn with_range(from: u32, to: u32) -> Self {
        Self { from, to }
    }

    /// Returns the lower bound of the accepted range.
    #[inline]
    pub const fn lower_bound(&self) -> u32 {
        self.from
    }

    /// Returns the upper bound of the accepted range.
    #[inline]
    pub const fn upper_bound(&self) -> u32 {
        self.to
    }
}

impl Filter for IntervalFilter {
    fn add(&mut self, filter_id: u32) {
        if self.from > self.to {
            // Empty: initialise range to this single ID.
            self.from = filter_id;
            self.to = filter_id;
        } else {
            if filter_id < self.from {
                self.from = filter_id;
            }
            if filter_id > self.to {
                self.to = filter_id;
            }
        }
    }

    fn add_range(&mut self, from: u32, to: u32) {
        if from > to {
            return; // invalid range — ignore
        }
        if self.from > self.to {
            // Currently empty: set directly.
            self.from = from;
            self.to = to;
        } else {
            if from < self.from {
                self.from = from;
            }
            if to > self.to {
                self.to = to;
            }
        }
    }

    fn matches(&self, filter_id: u32) -> bool {
        self.from <= filter_id && filter_id <= self.to
    }

    fn clear(&mut self) {
        self.from = u32::MAX;
        self.to = 0;
    }

    fn open(&mut self) {
        self.from = 0;
        self.to = u32::MAX;
    }

    fn merge_into(&self, target: &mut BitFieldFilter) {
        target.merge_with_interval(self);
    }
}

impl Default for IntervalFilter {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for IntervalFilter {
    fn eq(&self, other: &Self) -> bool {
        self.from == other.from && self.to == other.to
    }
}

impl Eq for IntervalFilter {}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{BitFieldFilter, Filter, IntervalFilter};

    // -----------------------------------------------------------------------
    // BitFieldFilter
    // -----------------------------------------------------------------------

    // 1 — new filter rejects everything
    #[test]
    fn bitfield_new_rejects_all() {
        let f = BitFieldFilter::new();
        for id in [0u32, 1, 0x100, 0x7FF] {
            assert!(!f.matches(id), "id {id:#x} should be rejected");
        }
    }

    // 2 — add a single ID; only that ID matches
    #[test]
    fn bitfield_add_single_id() {
        let mut f = BitFieldFilter::new();
        f.add(0x123);
        assert!(f.matches(0x123));
        assert!(!f.matches(0x122));
        assert!(!f.matches(0x124));
    }

    // 3 — add_range sets every ID in [from, to]
    #[test]
    fn bitfield_add_range() {
        let mut f = BitFieldFilter::new();
        f.add_range(0x10, 0x13);
        assert!(f.matches(0x10));
        assert!(f.matches(0x11));
        assert!(f.matches(0x12));
        assert!(f.matches(0x13));
        assert!(!f.matches(0x0F));
        assert!(!f.matches(0x14));
    }

    // 4 — matches after add
    #[test]
    fn bitfield_match_after_add() {
        let mut f = BitFieldFilter::new();
        f.add(0x7FF);
        assert!(f.matches(0x7FF));
    }

    // 5 — clear resets to reject-all
    #[test]
    fn bitfield_clear_resets() {
        let mut f = BitFieldFilter::new();
        f.add(0x100);
        assert!(f.matches(0x100));
        f.clear();
        assert!(!f.matches(0x100));
    }

    // 6 — open makes every valid ID match
    #[test]
    fn bitfield_open_accepts_all() {
        let mut f = BitFieldFilter::new();
        f.open();
        for id in [0u32, 1, 0x3FF, 0x7FF] {
            assert!(f.matches(id), "id {id:#x} should match after open");
        }
    }

    // 7 — IDs > 0x7FF are silently ignored
    #[test]
    fn bitfield_out_of_range_ignored() {
        let mut f = BitFieldFilter::new();
        f.add(0x800); // one past MAX
        f.add(0xFFFF_FFFF);
        assert!(!f.matches(0x800));
        assert!(!f.matches(0xFFFF_FFFF));
    }

    // 8 — PartialEq compares byte-by-byte
    #[test]
    fn bitfield_equality() {
        let mut a = BitFieldFilter::new();
        let mut b = BitFieldFilter::new();
        a.add(0x55);
        b.add(0x55);
        assert_eq!(a, b);
        b.add(0x56);
        assert_ne!(a, b);
    }

    // 9 — merge_with_bit_field ORs masks together
    #[test]
    fn bitfield_merge_with_bitfield() {
        let mut src = BitFieldFilter::new();
        src.add(0x200);
        src.add(0x300);

        let mut dst = BitFieldFilter::new();
        dst.add(0x100);
        dst.merge_with_bit_field(&src);

        assert!(dst.matches(0x100));
        assert!(dst.matches(0x200));
        assert!(dst.matches(0x300));
    }

    // 10 — merge_with_interval sets bits for each ID in range
    #[test]
    fn bitfield_merge_with_interval() {
        let interval = IntervalFilter::with_range(0x40, 0x42);
        let mut bf = BitFieldFilter::new();
        bf.merge_with_interval(&interval);

        assert!(bf.matches(0x40));
        assert!(bf.matches(0x41));
        assert!(bf.matches(0x42));
        assert!(!bf.matches(0x3F));
        assert!(!bf.matches(0x43));
    }

    // -----------------------------------------------------------------------
    // IntervalFilter
    // -----------------------------------------------------------------------

    // 11 — new is empty: matches nothing
    #[test]
    fn interval_new_is_empty() {
        let f = IntervalFilter::new();
        assert!(!f.matches(0));
        assert!(!f.matches(u32::MAX));
        assert!(f.lower_bound() > f.upper_bound()); // empty sentinel
    }

    // 12 — with_range constructs correct bounds
    #[test]
    fn interval_with_range() {
        let f = IntervalFilter::with_range(10, 20);
        assert_eq!(f.lower_bound(), 10);
        assert_eq!(f.upper_bound(), 20);
    }

    // 13 — matches is inclusive on both ends
    #[test]
    fn interval_match_inclusive() {
        let f = IntervalFilter::with_range(5, 10);
        assert!(f.matches(5));
        assert!(f.matches(7));
        assert!(f.matches(10));
        assert!(!f.matches(4));
        assert!(!f.matches(11));
    }

    // 14 — add extends range when ID is outside current bounds
    #[test]
    fn interval_add_extends_range() {
        let mut f = IntervalFilter::with_range(10, 20);
        f.add(5);  // extend lower
        f.add(25); // extend upper
        assert_eq!(f.lower_bound(), 5);
        assert_eq!(f.upper_bound(), 25);
    }

    // 15 — add_range extends range
    #[test]
    fn interval_add_range_extends() {
        let mut f = IntervalFilter::with_range(10, 20);
        f.add_range(1, 30);
        assert_eq!(f.lower_bound(), 1);
        assert_eq!(f.upper_bound(), 30);
    }

    // 16 — clear makes filter empty
    #[test]
    fn interval_clear_empties() {
        let mut f = IntervalFilter::with_range(0, 100);
        f.clear();
        assert!(!f.matches(50));
        assert!(f.lower_bound() > f.upper_bound());
    }

    // 17 — open accepts every u32
    #[test]
    fn interval_open_accepts_all() {
        let mut f = IntervalFilter::new();
        f.open();
        assert!(f.matches(0));
        assert!(f.matches(0x7FF));
        assert!(f.matches(u32::MAX));
    }

    // 18 — lower_bound / upper_bound accessors
    #[test]
    fn interval_lower_upper_bounds() {
        let f = IntervalFilter::with_range(100, 200);
        assert_eq!(f.lower_bound(), 100);
        assert_eq!(f.upper_bound(), 200);
    }

    // 19 — empty filter (from > to) matches nothing
    #[test]
    fn interval_empty_filter_matches_nothing() {
        let f = IntervalFilter::new();
        for id in [0u32, 1, 100, u32::MAX] {
            assert!(!f.matches(id));
        }
    }

    // -----------------------------------------------------------------------
    // Cross-filter: merge_into
    // -----------------------------------------------------------------------

    // 20 — IntervalFilter::merge_into sets bits in BitFieldFilter
    #[test]
    fn interval_merge_into_bitfield() {
        let interval = IntervalFilter::with_range(0x010, 0x012);
        let mut bf = BitFieldFilter::new();
        interval.merge_into(&mut bf);

        assert!(bf.matches(0x010));
        assert!(bf.matches(0x011));
        assert!(bf.matches(0x012));
        assert!(!bf.matches(0x00F));
        assert!(!bf.matches(0x013));
    }
}
