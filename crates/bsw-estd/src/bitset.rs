// Copyright 2024 Accenture.
// SPDX-License-Identifier: Apache-2.0

//! Fixed-size bit set with inline storage — a `no_std` port of OpenBSW's `estd::bitset`.
//!
//! # Design
//!
//! Storage is `[u32; M]` where `M = (N + 31) / 32`.  On stable Rust,
//! `generic_const_exprs` is not yet stable, so the struct takes two const
//! generics: `N` (the number of bits) and `M` (the word count).  A
//! compile-time assertion in [`Bitset::new`] verifies that `M == (N + 31) / 32`.
//!
//! Use [`bitset_words`] to compute `M` at the call site:
//!
//! ```rust
//! # use bsw_estd::bitset::{Bitset, bitset_words};
//! let mut bs: Bitset<64, { bitset_words(64) }> = Bitset::new();
//! bs.set(0);
//! assert!(bs.test(0));
//! ```
//!
//! # Key invariant
//!
//! Bits at positions `>= N` are **always zero**.  Every operation that could
//! set out-of-range bits calls [`Bitset::mask_unused_bits`] before returning.
//!
//! # C++ compatibility notes
//!
//! - `size()` returns `N`, matching `std::bitset::size()`.
//! - `from_bytes` follows big-endian byte order matching the C++ `from_bytes` helper.
//! - Shift semantics: `<<` shifts toward *higher* bit indices (logical left shift),
//!   `>>` shifts toward *lower* bit indices (logical right shift).

use core::fmt;
use core::ops::{
    BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not, Shl, ShlAssign, Shr,
    ShrAssign,
};

// ---------------------------------------------------------------------------
// Helper: compute word count at compile time
// ---------------------------------------------------------------------------

/// Returns the number of 32-bit words needed to hold `n` bits, rounded up.
///
/// Use this to compute the `M` const generic for [`Bitset`]:
///
/// ```rust
/// # use bsw_estd::bitset::{Bitset, bitset_words};
/// let _: Bitset<100, { bitset_words(100) }> = Bitset::new();
/// ```
#[inline]
pub const fn bitset_words(n: usize) -> usize {
    n.div_ceil(32)
}

// ---------------------------------------------------------------------------
// Bitset
// ---------------------------------------------------------------------------

/// Fixed-size bit set with `N` bits stored in `M` 32-bit words.
///
/// - `N` — number of bits.
/// - `M` — word count; **must equal `(N + 31) / 32`**.  A compile-time
///   assertion in [`new`](Bitset::new) enforces this.  Compute `M` with
///   [`bitset_words(N)`](bitset_words).
///
/// All bits beyond position `N - 1` are guaranteed to be zero at all times.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Bitset<const N: usize, const M: usize> {
    words: [u32; M],
}

impl<const N: usize, const M: usize> Bitset<N, M> {
    // Compile-time check: M must be exactly ceil(N / 32).
    //
    // For N=0 the word count must be 0.  div_ceil(0, 32) = 0.
    const CHECK_M: () = {
        assert!(M == N.div_ceil(32), "Bitset: M must equal (N + 31) / 32");
    };

    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------

    /// Creates a new `Bitset` with all bits set to zero.
    ///
    /// # Panics (compile-time)
    ///
    /// Panics at compile time if `M != (N + 31) / 32`.
    pub const fn new() -> Self {
        let () = Self::CHECK_M;
        Self { words: [0u32; M] }
    }

    /// Creates a `Bitset` from a `u64`, using its low `N` bits.
    ///
    /// # Panics
    ///
    /// Panics if `N < 64` and `value` has bits set above position `N - 1`
    /// (i.e., the value doesn't fit in `N` bits).
    pub fn from_u64(value: u64) -> Self {
        let () = Self::CHECK_M;
        let mut bs = Self::new();
        if N == 0 {
            assert!(value == 0, "Bitset: N=0 but value is non-zero");
            return bs;
        }
        // Reject values that exceed N bits.
        if N < 64 {
            assert!(
                value < (1u64 << N),
                "Bitset::from_u64: value does not fit in N bits"
            );
        }
        // Low 32 bits → word 0.
        if M > 0 {
            bs.words[0] = value as u32;
        }
        // High 32 bits → word 1 (only present when N > 32).
        if M > 1 {
            bs.words[1] = (value >> 32) as u32;
        }
        bs
    }

    /// Creates a `Bitset` from a byte slice (big-endian / network order).
    ///
    /// The first byte in `bytes` provides the most-significant bits.
    /// The byte slice must contain exactly `(N + 7) / 8` bytes.
    ///
    /// # Panics
    ///
    /// Panics if `bytes.len() != (N + 7) / 8`.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let () = Self::CHECK_M;
        let required = N.div_ceil(8);
        assert!(
            bytes.len() == required,
            "Bitset::from_bytes: expected {required} bytes, got {}",
            bytes.len()
        );
        let mut bs = Self::new();
        // Iterate from the last byte (LSB side) to the first byte (MSB side).
        // byte index `i` from the end maps to bit position `i * 8`.
        for (i, &byte) in bytes.iter().rev().enumerate() {
            let bit_pos = i * 8;
            if bit_pos >= N {
                break;
            }
            let bits_in_this_byte = (N - bit_pos).min(8);
            let mask = if bits_in_this_byte == 8 {
                0xFF_u8
            } else {
                (1u8 << bits_in_this_byte) - 1
            };
            let effective_byte = byte & mask;
            let word = bit_pos / 32;
            let offset = bit_pos % 32;
            bs.words[word] |= u32::from(effective_byte) << offset;
            // Handle the case where the byte straddles a word boundary.
            if offset > 24 && word + 1 < M {
                bs.words[word + 1] |= u32::from(effective_byte) >> (32 - offset);
            }
        }
        bs
    }

    // -----------------------------------------------------------------------
    // Capacity
    // -----------------------------------------------------------------------

    /// Returns the number of bits in the set (compile-time constant `N`).
    #[inline]
    pub const fn size(&self) -> usize {
        N
    }

    // -----------------------------------------------------------------------
    // Single-bit operations
    // -----------------------------------------------------------------------

    /// Sets all `N` bits to 1.
    #[inline]
    pub fn set_all(&mut self) {
        for w in &mut self.words {
            *w = u32::MAX;
        }
        self.mask_unused_bits();
    }

    /// Sets bit at position `pos` to 1.
    ///
    /// # Panics
    ///
    /// Panics if `pos >= N`.
    #[inline]
    pub fn set(&mut self, pos: usize) {
        assert!(pos < N, "Bitset::set: pos {pos} >= N {N}");
        self.words[Self::word_index(pos)] |= 1u32 << Self::bit_index(pos);
    }

    /// Sets bit at position `pos` to `val`.
    ///
    /// # Panics
    ///
    /// Panics if `pos >= N`.
    #[inline]
    pub fn set_val(&mut self, pos: usize, val: bool) {
        if val {
            self.set(pos);
        } else {
            self.reset(pos);
        }
    }

    /// Clears all bits to 0.
    #[inline]
    pub fn reset_all(&mut self) {
        for w in &mut self.words {
            *w = 0;
        }
    }

    /// Clears bit at position `pos` to 0.
    ///
    /// # Panics
    ///
    /// Panics if `pos >= N`.
    #[inline]
    pub fn reset(&mut self, pos: usize) {
        assert!(pos < N, "Bitset::reset: pos {pos} >= N {N}");
        self.words[Self::word_index(pos)] &= !(1u32 << Self::bit_index(pos));
    }

    /// Returns the value of bit at position `pos`.
    ///
    /// # Panics
    ///
    /// Panics if `pos >= N`.
    #[inline]
    pub fn test(&self, pos: usize) -> bool {
        assert!(pos < N, "Bitset::test: pos {pos} >= N {N}");
        (self.words[Self::word_index(pos)] >> Self::bit_index(pos)) & 1 == 1
    }

    /// Inverts all `N` bits.
    #[inline]
    pub fn flip_all(&mut self) {
        for w in &mut self.words {
            *w = !*w;
        }
        self.mask_unused_bits();
    }

    /// Inverts bit at position `pos`.
    ///
    /// # Panics
    ///
    /// Panics if `pos >= N`.
    #[inline]
    pub fn flip(&mut self, pos: usize) {
        assert!(pos < N, "Bitset::flip: pos {pos} >= N {N}");
        self.words[Self::word_index(pos)] ^= 1u32 << Self::bit_index(pos);
    }

    // -----------------------------------------------------------------------
    // Aggregate queries
    // -----------------------------------------------------------------------

    /// Returns `true` if all `N` bits are set.
    pub fn all(&self) -> bool {
        if N == 0 {
            return true;
        }
        // All words except the last must be fully set.
        for i in 0..M.saturating_sub(1) {
            if self.words[i] != u32::MAX {
                return false;
            }
        }
        // Last word: only the bits < N within that word should be set.
        let remainder = N % 32;
        let last_mask = if remainder == 0 {
            u32::MAX
        } else {
            (1u32 << remainder) - 1
        };
        self.words[M - 1] == last_mask
    }

    /// Returns `true` if at least one bit is set.
    pub fn any(&self) -> bool {
        self.words.iter().any(|&w| w != 0)
    }

    /// Returns `true` if no bits are set.
    pub fn none(&self) -> bool {
        !self.any()
    }

    /// Returns the number of bits that are set (population count).
    pub fn count(&self) -> usize {
        self.words.iter().map(|w| w.count_ones() as usize).sum()
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Zeroes out bits at positions `>= N` in the last word.
    ///
    /// This must be called after any operation that could set out-of-range bits
    /// (e.g., `set_all`, `flip_all`, bitwise NOT, shifts).
    #[inline]
    pub fn mask_unused_bits(&mut self) {
        if M == 0 {
            return;
        }
        let remainder = N % 32;
        if remainder != 0 {
            let mask = (1u32 << remainder) - 1;
            self.words[M - 1] &= mask;
        }
        // If remainder == 0 the last word uses all 32 bits; nothing to mask.
    }

    /// Returns the word index for bit position `pos`.
    #[inline]
    const fn word_index(pos: usize) -> usize {
        pos / 32
    }

    /// Returns the bit index within a word for bit position `pos`.
    #[inline]
    const fn bit_index(pos: usize) -> usize {
        pos % 32
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<const N: usize, const M: usize> Default for Bitset<N, M> {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Debug — show as "Bitset<N>{ 0b... }" with bit 0 at the right
// ---------------------------------------------------------------------------

impl<const N: usize, const M: usize> fmt::Debug for Bitset<N, M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Bitset<{N}>{{")?;
        fmt::Binary::fmt(self, f)?;
        write!(f, "}}")
    }
}

// ---------------------------------------------------------------------------
// Binary — output N binary digits, bit N-1 first (MSB on the left)
// ---------------------------------------------------------------------------

impl<const N: usize, const M: usize> fmt::Binary for Bitset<N, M> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for pos in (0..N).rev() {
            let bit = if self.test(pos) { '1' } else { '0' };
            f.write_fmt(format_args!("{bit}"))?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Bitwise operators
// ---------------------------------------------------------------------------

impl<const N: usize, const M: usize> BitAnd for Bitset<N, M> {
    type Output = Self;
    fn bitand(mut self, rhs: Self) -> Self {
        self &= rhs;
        self
    }
}

impl<const N: usize, const M: usize> BitAndAssign for Bitset<N, M> {
    fn bitand_assign(&mut self, rhs: Self) {
        for (lw, rw) in self.words.iter_mut().zip(rhs.words.iter()) {
            *lw &= rw;
        }
        // AND can only clear bits, so no unused bits can be set — no masking needed.
    }
}

impl<const N: usize, const M: usize> BitOr for Bitset<N, M> {
    type Output = Self;
    fn bitor(mut self, rhs: Self) -> Self {
        self |= rhs;
        self
    }
}

impl<const N: usize, const M: usize> BitOrAssign for Bitset<N, M> {
    fn bitor_assign(&mut self, rhs: Self) {
        for (lw, rw) in self.words.iter_mut().zip(rhs.words.iter()) {
            *lw |= rw;
        }
        // Both operands already satisfy the invariant, so OR cannot set unused bits.
    }
}

impl<const N: usize, const M: usize> BitXor for Bitset<N, M> {
    type Output = Self;
    fn bitxor(mut self, rhs: Self) -> Self {
        self ^= rhs;
        self
    }
}

impl<const N: usize, const M: usize> BitXorAssign for Bitset<N, M> {
    fn bitxor_assign(&mut self, rhs: Self) {
        for (lw, rw) in self.words.iter_mut().zip(rhs.words.iter()) {
            *lw ^= rw;
        }
        // XOR of two values that each have unused bits = 0 keeps them 0.
    }
}

impl<const N: usize, const M: usize> Not for Bitset<N, M> {
    type Output = Self;
    fn not(mut self) -> Self {
        for w in &mut self.words {
            *w = !*w;
        }
        self.mask_unused_bits();
        self
    }
}

// ---------------------------------------------------------------------------
// Shift operators — multi-word aware
//
// `<< shift` moves bits toward higher indices (bit 0 → bit `shift`).
// `>> shift` moves bits toward lower indices (bit `shift` → bit 0).
// ---------------------------------------------------------------------------

impl<const N: usize, const M: usize> Shl<usize> for Bitset<N, M> {
    type Output = Self;
    fn shl(mut self, rhs: usize) -> Self {
        self <<= rhs;
        self
    }
}

impl<const N: usize, const M: usize> ShlAssign<usize> for Bitset<N, M> {
    fn shl_assign(&mut self, shift: usize) {
        if shift >= N || M == 0 {
            self.reset_all();
            return;
        }
        let word_shift = shift / 32;
        let bit_shift = shift % 32;

        if bit_shift == 0 {
            // Pure word-aligned shift: copy words upward.
            for i in (0..M).rev() {
                self.words[i] = if i >= word_shift {
                    self.words[i - word_shift]
                } else {
                    0
                };
            }
        } else {
            for i in (0..M).rev() {
                let src = if i >= word_shift { self.words[i - word_shift] } else { 0 };
                let src_lo = if i > word_shift {
                    self.words[i - word_shift - 1]
                } else {
                    0
                };
                self.words[i] = (src << bit_shift) | (src_lo >> (32 - bit_shift));
            }
        }
        self.mask_unused_bits();
    }
}

impl<const N: usize, const M: usize> Shr<usize> for Bitset<N, M> {
    type Output = Self;
    fn shr(mut self, rhs: usize) -> Self {
        self >>= rhs;
        self
    }
}

impl<const N: usize, const M: usize> ShrAssign<usize> for Bitset<N, M> {
    fn shr_assign(&mut self, shift: usize) {
        if shift >= N || M == 0 {
            self.reset_all();
            return;
        }
        let word_shift = shift / 32;
        let bit_shift = shift % 32;

        if bit_shift == 0 {
            // Pure word-aligned shift: copy words downward.
            for i in 0..M {
                self.words[i] = if i + word_shift < M {
                    self.words[i + word_shift]
                } else {
                    0
                };
            }
        } else {
            for i in 0..M {
                let src = if i + word_shift < M {
                    self.words[i + word_shift]
                } else {
                    0
                };
                let src_hi = if i + word_shift + 1 < M {
                    self.words[i + word_shift + 1]
                } else {
                    0
                };
                self.words[i] = (src >> bit_shift) | (src_hi << (32 - bit_shift));
            }
        }
        // Right shift cannot set unused bits (shifts toward 0) — no masking needed.
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Convenience type aliases.
    type Bs0 = Bitset<0, { bitset_words(0) }>;
    type Bs7 = Bitset<7, { bitset_words(7) }>;
    type Bs8 = Bitset<8, { bitset_words(8) }>;
    type Bs32 = Bitset<32, { bitset_words(32) }>;
    type Bs33 = Bitset<33, { bitset_words(33) }>;
    type Bs64 = Bitset<64, { bitset_words(64) }>;
    type Bs65 = Bitset<65, { bitset_words(65) }>;
    type Bs128 = Bitset<128, { bitset_words(128) }>;
    type Bs256 = Bitset<256, { bitset_words(256) }>;

    // -----------------------------------------------------------------------
    // 1. New bitset is all zeros
    // -----------------------------------------------------------------------

    #[test]
    fn new_is_all_zeros() {
        let bs: Bs64 = Bitset::new();
        assert!(bs.none(), "new bitset must have all bits clear");
        assert_eq!(bs.count(), 0);
        for i in 0..64 {
            assert!(!bs.test(i), "bit {i} must be 0 in new bitset");
        }
    }

    // -----------------------------------------------------------------------
    // 2. set / test single bits
    // -----------------------------------------------------------------------

    #[test]
    fn set_and_test_single_bits() {
        let mut bs: Bs8 = Bitset::new();
        for pos in 0..8usize {
            bs.set(pos);
            assert!(bs.test(pos), "bit {pos} must be set after set()");
        }
        assert_eq!(bs.count(), 8);
    }

    #[test]
    fn set_does_not_affect_other_bits() {
        let mut bs: Bs32 = Bitset::new();
        bs.set(15);
        for i in 0..32usize {
            assert_eq!(bs.test(i), i == 15, "only bit 15 should be set");
        }
    }

    // -----------------------------------------------------------------------
    // 3. set_all / reset_all
    // -----------------------------------------------------------------------

    #[test]
    fn set_all_and_reset_all() {
        let mut bs: Bs33 = Bitset::new();
        bs.set_all();
        assert!(bs.all(), "all() must be true after set_all()");
        assert_eq!(bs.count(), 33);

        bs.reset_all();
        assert!(bs.none(), "none() must be true after reset_all()");
        assert_eq!(bs.count(), 0);
    }

    // -----------------------------------------------------------------------
    // 4. flip individual and flip_all
    // -----------------------------------------------------------------------

    #[test]
    fn flip_single_bit() {
        let mut bs: Bs8 = Bitset::new();
        bs.flip(3);
        assert!(bs.test(3));
        bs.flip(3);
        assert!(!bs.test(3));
    }

    #[test]
    fn flip_all_inverts_all_bits() {
        let mut bs: Bs33 = Bitset::new();
        bs.flip_all();
        assert!(bs.all(), "flip_all on zero must yield all-ones");
        assert_eq!(bs.count(), 33);

        bs.flip_all();
        assert!(bs.none(), "double flip_all must yield all-zeros");
        assert_eq!(bs.count(), 0);
    }

    // -----------------------------------------------------------------------
    // 5. count() popcount correctness
    // -----------------------------------------------------------------------

    #[test]
    fn count_popcount_correctness() {
        let mut bs: Bs64 = Bitset::new();
        for i in (0..64usize).step_by(2) {
            bs.set(i);
        }
        assert_eq!(bs.count(), 32, "every other bit set in 64-bit set = 32 bits");
    }

    #[test]
    fn count_large_bitset() {
        let mut bs: Bs256 = Bitset::new();
        bs.set_all();
        assert_eq!(bs.count(), 256);
        bs.reset_all();
        assert_eq!(bs.count(), 0);
    }

    // -----------------------------------------------------------------------
    // 6. all() / any() / none() predicates
    // -----------------------------------------------------------------------

    #[test]
    fn all_any_none_predicates() {
        let mut bs: Bs8 = Bitset::new();

        assert!(bs.none(), "fresh bitset: none()");
        assert!(!bs.any(), "fresh bitset: !any()");
        assert!(!bs.all(), "fresh bitset: !all()");

        bs.set(4);
        assert!(!bs.none(), "one bit set: !none()");
        assert!(bs.any(), "one bit set: any()");
        assert!(!bs.all(), "one bit set: !all()");

        bs.set_all();
        assert!(!bs.none(), "all set: !none()");
        assert!(bs.any(), "all set: any()");
        assert!(bs.all(), "all set: all()");
    }

    #[test]
    fn all_on_non_power_of_two_size() {
        let mut bs: Bs7 = Bitset::new();
        bs.set_all();
        assert!(bs.all(), "7-bit set_all must satisfy all()");
        assert_eq!(bs.count(), 7);
    }

    // -----------------------------------------------------------------------
    // 7. Bitwise AND / OR / XOR / NOT
    // -----------------------------------------------------------------------

    #[test]
    fn bitwise_and() {
        let mut a: Bs8 = Bitset::new();
        let mut b: Bs8 = Bitset::new();
        a.set(0);
        a.set(1);
        b.set(1);
        b.set(2);
        let c = a & b;
        assert!(c.test(1), "AND: bit 1 should be set");
        assert!(!c.test(0), "AND: bit 0 should be clear");
        assert!(!c.test(2), "AND: bit 2 should be clear");
    }

    #[test]
    fn bitwise_or() {
        let mut a: Bs8 = Bitset::new();
        let mut b: Bs8 = Bitset::new();
        a.set(0);
        b.set(1);
        let c = a | b;
        assert!(c.test(0));
        assert!(c.test(1));
        assert!(!c.test(2));
    }

    #[test]
    fn bitwise_xor() {
        let mut a: Bs8 = Bitset::new();
        let mut b: Bs8 = Bitset::new();
        a.set(0);
        a.set(1);
        b.set(1);
        b.set(2);
        let c = a ^ b;
        assert!(c.test(0), "XOR: bit 0 set only in a");
        assert!(!c.test(1), "XOR: bit 1 in both, should cancel");
        assert!(c.test(2), "XOR: bit 2 set only in b");
    }

    #[test]
    fn bitwise_not() {
        let mut bs: Bs8 = Bitset::new();
        bs.set(0);
        bs.set(7);
        let inv = !bs;
        assert!(!inv.test(0));
        assert!(!inv.test(7));
        for i in 1..7usize {
            assert!(inv.test(i), "bit {i} should be set after NOT");
        }
        assert_eq!(inv.count(), 6);
    }

    #[test]
    fn bitwise_not_on_non_multiple_of_32() {
        let mut bs: Bs33 = Bitset::new();
        bs.set_all();
        let inv = !bs;
        assert!(inv.none(), "NOT of all-ones is all-zeros");
        assert_eq!(inv.count(), 0);
    }

    #[test]
    fn bitwise_and_assign() {
        let mut a: Bs8 = Bitset::new();
        a.set(0);
        a.set(1);
        let mut b: Bs8 = Bitset::new();
        b.set(1);
        a &= b;
        assert!(!a.test(0));
        assert!(a.test(1));
    }

    #[test]
    fn bitwise_or_assign() {
        let mut a: Bs8 = Bitset::new();
        a.set(0);
        let mut b: Bs8 = Bitset::new();
        b.set(1);
        a |= b;
        assert!(a.test(0));
        assert!(a.test(1));
    }

    #[test]
    fn bitwise_xor_assign() {
        let mut a: Bs8 = Bitset::new();
        a.set(0);
        a.set(1);
        let mut b: Bs8 = Bitset::new();
        b.set(1);
        a ^= b;
        assert!(a.test(0));
        assert!(!a.test(1));
    }

    // -----------------------------------------------------------------------
    // 8. Left shift / right shift (including multi-word shifts)
    // -----------------------------------------------------------------------

    #[test]
    fn shl_basic() {
        let mut bs: Bs8 = Bitset::new();
        bs.set(0);
        let shifted = bs << 3;
        assert!(!shifted.test(0));
        assert!(shifted.test(3), "bit 0 shifted left by 3 → bit 3");
    }

    #[test]
    fn shr_basic() {
        let mut bs: Bs8 = Bitset::new();
        bs.set(7);
        let shifted = bs >> 3;
        assert!(!shifted.test(7));
        assert!(shifted.test(4), "bit 7 shifted right by 3 → bit 4");
    }

    #[test]
    fn shl_multi_word() {
        // 64-bit set, shift by 32 (exactly one word).
        let mut bs: Bs64 = Bitset::new();
        bs.set(0); // lowest bit
        let shifted = bs << 32;
        assert!(!shifted.test(0));
        assert!(shifted.test(32), "bit 0 shl 32 → bit 32");
    }

    #[test]
    fn shr_multi_word() {
        let mut bs: Bs64 = Bitset::new();
        bs.set(32); // first bit of second word
        let shifted = bs >> 32;
        assert!(!shifted.test(32));
        assert!(shifted.test(0), "bit 32 shr 32 → bit 0");
    }

    #[test]
    fn shl_across_word_boundary() {
        // N=64. Set bit 16 (in word 0), shift left by 20 → should land at bit 36 (in word 1).
        let mut bs: Bs64 = Bitset::new();
        bs.set(16);
        let shifted = bs << 20;
        assert!(shifted.test(36), "bit 16 shl 20 → bit 36");
        assert!(!shifted.test(16));
    }

    #[test]
    fn shr_across_word_boundary() {
        let mut bs: Bs64 = Bitset::new();
        bs.set(36); // word 1
        let shifted = bs >> 20;
        assert!(shifted.test(16), "bit 36 shr 20 → bit 16");
        assert!(!shifted.test(36));
    }

    #[test]
    fn shl_128_multi_word() {
        let mut bs: Bs128 = Bitset::new();
        bs.set(0);
        let shifted = bs << 65;
        assert!(shifted.test(65), "bit 0 shl 65 → bit 65");
        assert!(!shifted.test(0));
    }

    #[test]
    fn shr_128_multi_word() {
        let mut bs: Bs128 = Bitset::new();
        bs.set(65);
        let shifted = bs >> 65;
        assert!(shifted.test(0), "bit 65 shr 65 → bit 0");
        assert!(!shifted.test(65));
    }

    // -----------------------------------------------------------------------
    // 9. Shift by 0, by word boundary (32), by more than N
    // -----------------------------------------------------------------------

    #[test]
    fn shl_by_zero_is_identity() {
        let mut bs: Bs64 = Bitset::new();
        bs.set(5);
        bs.set(42);
        let orig = bs;
        let shifted = bs << 0;
        assert_eq!(shifted, orig, "shift left by 0 must be identity");
    }

    #[test]
    fn shr_by_zero_is_identity() {
        let mut bs: Bs64 = Bitset::new();
        bs.set(5);
        bs.set(42);
        let orig = bs;
        let shifted = bs >> 0;
        assert_eq!(shifted, orig, "shift right by 0 must be identity");
    }

    #[test]
    fn shl_by_n_clears_all() {
        let mut bs: Bs64 = Bitset::new();
        bs.set_all();
        let shifted = bs << 64;
        assert!(shifted.none(), "shift left by N must produce zero bitset");
    }

    #[test]
    fn shr_by_n_clears_all() {
        let mut bs: Bs64 = Bitset::new();
        bs.set_all();
        let shifted = bs >> 64;
        assert!(shifted.none(), "shift right by N must produce zero bitset");
    }

    #[test]
    fn shl_by_more_than_n_clears_all() {
        let mut bs: Bs32 = Bitset::new();
        bs.set_all();
        let shifted = bs << 100;
        assert!(shifted.none(), "shift left by > N must produce zero bitset");
    }

    #[test]
    fn shr_by_more_than_n_clears_all() {
        let mut bs: Bs32 = Bitset::new();
        bs.set_all();
        let shifted = bs >> 100;
        assert!(shifted.none(), "shift right by > N must produce zero bitset");
    }

    // -----------------------------------------------------------------------
    // 10. from_u64 roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn from_u64_roundtrip_small() {
        let bs: Bs8 = Bitset::from_u64(0b1010_1010);
        assert!(bs.test(1));
        assert!(!bs.test(0));
        assert!(bs.test(3));
        assert!(!bs.test(2));
        assert!(bs.test(5));
        assert!(bs.test(7));
        assert_eq!(bs.count(), 4);
    }

    #[test]
    fn from_u64_full_64_bits() {
        let value = 0xDEAD_BEEF_CAFE_BABEu64;
        let bs: Bs64 = Bitset::from_u64(value);
        for i in 0..64usize {
            let expected = (value >> i) & 1 == 1;
            assert_eq!(
                bs.test(i),
                expected,
                "bit {i} mismatch for value {value:#x}"
            );
        }
    }

    #[test]
    fn from_u64_zero() {
        let bs: Bs64 = Bitset::from_u64(0);
        assert!(bs.none());
    }

    #[test]
    fn from_u64_max_for_32_bit() {
        let bs: Bs32 = Bitset::from_u64(u64::from(u32::MAX));
        assert!(bs.all());
    }

    #[test]
    #[should_panic(expected = "value does not fit in N bits")]
    fn from_u64_overflow_panics() {
        // N=8, value has bit 8 set — should panic.
        let _: Bs8 = Bitset::from_u64(256);
    }

    // -----------------------------------------------------------------------
    // 11. from_bytes roundtrip
    // -----------------------------------------------------------------------

    #[test]
    fn from_bytes_8bit() {
        // big-endian: byte[0] = MSB.
        let bs: Bs8 = Bitset::from_bytes(&[0b1010_1010]);
        // bits: 7=1, 6=0, 5=1, 4=0, 3=1, 2=0, 1=1, 0=0
        assert!(!bs.test(0));
        assert!(bs.test(1));
        assert!(!bs.test(2));
        assert!(bs.test(3));
        assert!(!bs.test(4));
        assert!(bs.test(5));
        assert!(!bs.test(6));
        assert!(bs.test(7));
    }

    #[test]
    fn from_bytes_16bit() {
        // big-endian: [high_byte, low_byte]
        let bs: Bitset<16, { bitset_words(16) }> = Bitset::from_bytes(&[0x01, 0x00]);
        // 0x0100 = bit 8 set
        assert!(bs.test(8), "bit 8 should be set");
        assert!(!bs.test(0), "bit 0 should be clear");
    }

    #[test]
    fn from_bytes_roundtrip_with_from_u64() {
        let value = 0xAB_CDu64;
        let bs_u64: Bitset<16, { bitset_words(16) }> = Bitset::from_u64(value);
        let bs_bytes: Bitset<16, { bitset_words(16) }> = Bitset::from_bytes(&[0xAB, 0xCD]);
        assert_eq!(bs_u64, bs_bytes, "from_u64 and from_bytes must agree");
    }

    #[test]
    #[should_panic(expected = "expected 1 bytes, got 2")]
    fn from_bytes_wrong_length_panics() {
        let _: Bs8 = Bitset::from_bytes(&[0x00, 0x00]);
    }

    // -----------------------------------------------------------------------
    // 12. Equality comparison
    // -----------------------------------------------------------------------

    #[test]
    fn equality_comparison() {
        let mut a: Bs64 = Bitset::new();
        let mut b: Bs64 = Bitset::new();
        assert_eq!(a, b, "two fresh bitsets must be equal");

        a.set(10);
        assert_ne!(a, b);

        b.set(10);
        assert_eq!(a, b);
    }

    // -----------------------------------------------------------------------
    // 13. Unused bits are always zero after any operation
    // -----------------------------------------------------------------------

    #[test]
    fn unused_bits_always_zero_after_set_all() {
        let mut bs: Bs33 = Bitset::new();
        bs.set_all();
        // The last word (word index 1) must only have bit 0 set (bit 32 of the bitset).
        assert_eq!(bs.words[1], 1, "only bit 32 (in word 1) should be set");
    }

    #[test]
    fn unused_bits_always_zero_after_flip_all() {
        let mut bs: Bs33 = Bitset::new();
        bs.flip_all();
        assert_eq!(bs.words[1], 1, "flip_all: only bit 32 should be set in last word");
    }

    #[test]
    fn unused_bits_always_zero_after_not() {
        let bs: Bs33 = Bitset::new();
        let inv = !bs;
        assert_eq!(inv.words[1], 1, "NOT: only bit 32 should be set in last word");
    }

    #[test]
    fn unused_bits_always_zero_after_shl() {
        let mut bs: Bs33 = Bitset::new();
        bs.set_all();
        let shifted = bs << 1;
        // After shift: bit 33 would be bit 34, but N=33 so it's masked.
        assert_eq!(shifted.words[1] & !1u32, 0, "upper word high bits must be zero after shl");
    }

    #[test]
    fn unused_bits_always_zero_after_all_ops() {
        // For Bs65 (N=65, M=3): last word is words[2], only bit 0 should be set when full.
        let mut bs: Bs65 = Bitset::new();
        bs.set_all();
        assert_eq!(bs.words[2], 1, "only bit 64 should be in words[2]");
        assert_eq!(bs.count(), 65);

        let inv = !bs;
        assert!(inv.none(), "NOT of all-ones in Bs65 must be all-zeros");

        bs.flip_all();
        assert!(bs.none());
    }

    // -----------------------------------------------------------------------
    // 14. Zero-size bitset (N=0)
    // -----------------------------------------------------------------------

    #[test]
    fn zero_size_bitset() {
        let bs: Bs0 = Bitset::new();
        assert_eq!(bs.size(), 0);
        assert!(bs.none(), "N=0: none()");
        assert!(!bs.any(), "N=0: !any()");
        assert!(bs.all(), "N=0: all() vacuously true");
        assert_eq!(bs.count(), 0);

        let mut bs2: Bs0 = Bitset::new();
        bs2.set_all();
        assert!(bs2.none());

        bs2.flip_all();
        assert!(bs2.none());

        let inv = !bs;
        assert!(inv.none());

        let a = bs;
        let b = bs;
        assert_eq!(a & b, bs);
        assert_eq!(a | b, bs);
        assert_eq!(a ^ b, bs);
    }

    // -----------------------------------------------------------------------
    // 15. Exactly 32-bit boundary (N=32)
    // -----------------------------------------------------------------------

    #[test]
    fn exactly_32_bits() {
        let mut bs: Bs32 = Bitset::new();
        bs.set_all();
        assert!(bs.all());
        assert_eq!(bs.count(), 32);
        assert_eq!(bs.words[0], u32::MAX);

        let inv = !bs;
        assert!(inv.none());
    }

    // -----------------------------------------------------------------------
    // 16. Exactly 64-bit boundary (N=64)
    // -----------------------------------------------------------------------

    #[test]
    fn exactly_64_bits() {
        let mut bs: Bs64 = Bitset::new();
        bs.set_all();
        assert!(bs.all());
        assert_eq!(bs.count(), 64);
        assert_eq!(bs.words[0], u32::MAX);
        assert_eq!(bs.words[1], u32::MAX);

        let inv = !bs;
        assert!(inv.none());
    }

    // -----------------------------------------------------------------------
    // 17. Large bitsets (N=128, N=256)
    // -----------------------------------------------------------------------

    #[test]
    fn large_128_bit_operations() {
        let mut bs: Bs128 = Bitset::new();
        bs.set_all();
        assert_eq!(bs.count(), 128);
        assert!(bs.all());

        let half = bs >> 64;
        assert_eq!(half.count(), 64, "shr 64 on all-ones 128-bit set should yield 64 set bits");
        for i in 0..64usize {
            assert!(half.test(i));
        }
        for i in 64..128usize {
            assert!(!half.test(i));
        }
    }

    #[test]
    fn large_256_bit_set_all_and_flip() {
        let mut bs: Bs256 = Bitset::new();
        bs.set_all();
        assert_eq!(bs.count(), 256);
        bs.flip_all();
        assert_eq!(bs.count(), 0);
        assert!(bs.none());
    }

    // -----------------------------------------------------------------------
    // 18. Out-of-bounds panics for set / test / reset / flip
    // -----------------------------------------------------------------------

    #[test]
    #[should_panic(expected = "pos 8 >= N 8")]
    fn set_out_of_bounds_panics() {
        let mut bs: Bs8 = Bitset::new();
        bs.set(8);
    }

    #[test]
    #[should_panic(expected = "pos 8 >= N 8")]
    fn test_out_of_bounds_panics() {
        let bs: Bs8 = Bitset::new();
        bs.test(8);
    }

    #[test]
    #[should_panic(expected = "pos 8 >= N 8")]
    fn reset_out_of_bounds_panics() {
        let mut bs: Bs8 = Bitset::new();
        bs.reset(8);
    }

    #[test]
    #[should_panic(expected = "pos 8 >= N 8")]
    fn flip_out_of_bounds_panics() {
        let mut bs: Bs8 = Bitset::new();
        bs.flip(8);
    }

    // -----------------------------------------------------------------------
    // Extra: set_val
    // -----------------------------------------------------------------------

    #[test]
    fn set_val_sets_and_clears() {
        let mut bs: Bs8 = Bitset::new();
        bs.set_val(3, true);
        assert!(bs.test(3));
        bs.set_val(3, false);
        assert!(!bs.test(3));
    }

    // -----------------------------------------------------------------------
    // Extra: Debug / Binary format (std only — needs format! macro)
    // -----------------------------------------------------------------------

    #[cfg(feature = "std")]
    #[test]
    fn debug_format_contains_binary_digits() {
        let mut bs: Bs8 = Bitset::new();
        bs.set(0);
        bs.set(7);
        let s = std::format!("{bs:?}");
        // Debug shows binary digits; bit 7 should be leftmost, bit 0 rightmost.
        assert!(s.contains('1'), "debug output must contain '1'");
        assert!(s.contains("Bitset"), "debug output must contain 'Bitset'");
    }

    #[cfg(feature = "std")]
    #[test]
    fn binary_format_msb_first() {
        let bs: Bs8 = Bitset::from_u64(0b1000_0001);
        let s = std::format!("{bs:b}");
        assert_eq!(s, "10000001", "binary format: MSB first, 8 digits, got: {s}");
    }

    // -----------------------------------------------------------------------
    // Extra: bitset_words helper
    // -----------------------------------------------------------------------

    #[test]
    fn bitset_words_helper() {
        assert_eq!(bitset_words(0), 0);
        assert_eq!(bitset_words(1), 1);
        assert_eq!(bitset_words(31), 1);
        assert_eq!(bitset_words(32), 1);
        assert_eq!(bitset_words(33), 2);
        assert_eq!(bitset_words(64), 2);
        assert_eq!(bitset_words(65), 3);
        assert_eq!(bitset_words(128), 4);
        assert_eq!(bitset_words(256), 8);
    }

    // -----------------------------------------------------------------------
    // Extra: Clone / Copy
    // -----------------------------------------------------------------------

    #[test]
    fn clone_and_copy_work() {
        let mut bs: Bs32 = Bitset::new();
        bs.set(5);
        let cloned = bs;
        assert_eq!(bs, cloned, "copy must produce identical bitset");
        bs.set(10);
        assert!(!cloned.test(10), "copy must be independent");
    }

    // -----------------------------------------------------------------------
    // Extra: shift-assign operators
    // -----------------------------------------------------------------------

    #[test]
    fn shl_assign_and_shr_assign() {
        let mut bs: Bs64 = Bitset::new();
        bs.set(0);
        bs <<= 10;
        assert!(bs.test(10));
        bs >>= 10;
        assert!(bs.test(0));
    }
}
