// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! CAN identifier encoding and decoding — Rust port of `OpenBSW` `CanId`.
//!
//! # Bit layout (32-bit encoded value)
//!
//! ```text
//! Bit 31 (X): Extended qualifier — 1 = extended frame, 0 = base frame
//! Bit 30 (I): Invalid qualifier  — 1 = invalid
//! Bit 29 (F): Force non-FD qualifier
//! Bits 0–28:  Raw arbitration ID (29-bit field; for base frames only
//!             bits 0–10 are meaningful)
//! ```
//!
//! The `INVALID_ID` sentinel `0xFFFF_FFFF` is used to represent an absent or
//! uninitialised identifier.

// ---------------------------------------------------------------------------
// Qualifier bit constants
// ---------------------------------------------------------------------------

/// Bit 31 — set for extended (29-bit) CAN IDs.
pub const EXTENDED_QUALIFIER_BIT: u32 = 0x8000_0000;

/// Bit 30 — set to mark an ID as invalid / absent.
pub const INVALID_QUALIFIER_BIT: u32 = 0x4000_0000;

/// Bit 29 — set to force classic (non-FD) transmission even on an FD bus.
pub const FORCE_NON_FD_QUALIFIER_BIT: u32 = 0x2000_0000;

/// Sentinel value used when no valid CAN ID is present.
pub const INVALID_ID: u32 = 0xFFFF_FFFF;

/// Maximum raw value for an 11-bit base CAN ID.
pub const MAX_RAW_BASE_ID: u16 = 0x7FF;

/// Maximum raw value for a 29-bit extended CAN ID.
pub const MAX_RAW_EXTENDED_ID: u32 = 0x1FFF_FFFF;

/// Mask covering the 29 raw arbitration bits (bits 0–28).
const RAW_ID_MASK: u32 = MAX_RAW_EXTENDED_ID;

// ---------------------------------------------------------------------------
// CanId
// ---------------------------------------------------------------------------

/// An encoded CAN arbitration identifier.
///
/// The qualifier bits are stored together with the raw ID in a single `u32`
/// word, exactly mirroring the layout used by the C++ `CanId` class.
///
/// # Construction
///
/// ```
/// # use bsw_can::can_id::{CanId, MAX_RAW_BASE_ID};
/// let base     = CanId::base(0x123);
/// let extended = CanId::extended(0x1234_567);
/// let also_ext = CanId::id(0x1234_567, true);
/// ```
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
pub struct CanId(u32);

impl CanId {
    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------

    /// Creates a base-frame (11-bit) CAN ID.
    ///
    /// The EXTENDED bit is **not** set.  `base_id` is masked to 11 bits; any
    /// higher bits are silently truncated.
    #[inline]
    pub const fn base(base_id: u16) -> Self {
        // Widening u16 → u32: lossless, but `u32::from` is not const-stable yet.
        #[allow(clippy::cast_lossless)]
        { Self(base_id as u32 & MAX_RAW_BASE_ID as u32) }
    }

    /// Creates an extended-frame (29-bit) CAN ID.
    ///
    /// Sets the [`EXTENDED_QUALIFIER_BIT`].  `ext_id` is masked to 29 bits.
    #[inline]
    pub const fn extended(ext_id: u32) -> Self {
        Self(EXTENDED_QUALIFIER_BIT | (ext_id & RAW_ID_MASK))
    }

    /// Creates a CAN ID from a raw value and an extended flag.
    ///
    /// Equivalent to calling [`base`](CanId::base) or [`extended`](CanId::extended)
    /// based on `is_extended`.
    #[inline]
    pub const fn id(value: u32, is_extended: bool) -> Self {
        if is_extended {
            Self::extended(value)
        } else {
            // Truncation is intentional: caller supplies a base-frame value
            // that fits in 11 bits; base() masks off any stray high bits.
            #[allow(clippy::cast_possible_truncation)]
            Self::base(value as u16)
        }
    }

    /// Creates a CAN ID with full control over both the extended and
    /// force-no-FD qualifier bits.
    #[inline]
    pub const fn id_with_flags(value: u32, is_extended: bool, force_no_fd: bool) -> Self {
        let this = Self::id(value, is_extended);
        if force_no_fd {
            this.force_no_fd()
        } else {
            this
        }
    }

    /// Returns a copy of `self` with the [`FORCE_NON_FD_QUALIFIER_BIT`] set.
    #[must_use]
    #[inline]
    pub const fn force_no_fd(self) -> Self {
        Self(self.0 | FORCE_NON_FD_QUALIFIER_BIT)
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /// Returns the raw arbitration ID (bits 0–28) without any qualifier bits.
    #[inline]
    pub const fn raw_id(self) -> u32 {
        self.0 & RAW_ID_MASK
    }

    /// Returns the full encoded `u32` including all qualifier bits.
    #[inline]
    pub const fn value(self) -> u32 {
        self.0
    }

    /// Returns `true` if this is a **base** (11-bit) frame identifier
    /// (i.e. the extended bit is **not** set).
    #[inline]
    pub const fn is_base(self) -> bool {
        (self.0 & EXTENDED_QUALIFIER_BIT) == 0
    }

    /// Returns `true` if this is an **extended** (29-bit) frame identifier.
    #[inline]
    pub const fn is_extended(self) -> bool {
        (self.0 & EXTENDED_QUALIFIER_BIT) != 0
    }

    /// Returns `true` if the force-non-FD bit is set.
    #[inline]
    pub const fn is_force_no_fd(self) -> bool {
        (self.0 & FORCE_NON_FD_QUALIFIER_BIT) != 0
    }

    /// Returns `true` if this ID is **valid** (the invalid qualifier bit is
    /// **not** set).
    ///
    /// The [`INVALID_ID`] sentinel (`0xFFFF_FFFF`) and any value with bit 30
    /// set will return `false`.
    #[inline]
    pub const fn is_valid(self) -> bool {
        (self.0 & INVALID_QUALIFIER_BIT) == 0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{
        CanId, EXTENDED_QUALIFIER_BIT, FORCE_NON_FD_QUALIFIER_BIT, INVALID_ID,
        INVALID_QUALIFIER_BIT, MAX_RAW_BASE_ID, MAX_RAW_EXTENDED_ID,
    };

    // 1 — base ID creation stores correct raw value, no qualifier bits set
    #[test]
    fn base_id_creation() {
        let id = CanId::base(0x123);
        assert_eq!(id.raw_id(), 0x123);
        assert!(id.is_base());
        assert!(!id.is_extended());
        assert!(id.is_valid());
        assert_eq!(id.value(), 0x123);
    }

    // 2 — extended ID creation stores raw value + EXTENDED bit
    #[test]
    fn extended_id_creation() {
        let id = CanId::extended(0x0123_4567);
        assert_eq!(id.raw_id(), 0x0123_4567);
        assert!(id.is_extended());
        assert!(!id.is_base());
        assert!(id.is_valid());
        assert_eq!(id.value(), EXTENDED_QUALIFIER_BIT | 0x0123_4567);
    }

    // 3 — force_no_fd flag is set; other bits unchanged
    #[test]
    fn force_no_fd_flag() {
        let id = CanId::base(0x100).force_no_fd();
        assert!(id.is_force_no_fd());
        assert_eq!(id.raw_id(), 0x100);
        assert!(id.is_base());
        assert_eq!(id.value(), FORCE_NON_FD_QUALIFIER_BIT | 0x100);
    }

    // 4 — raw_id_extraction strips all qualifier bits
    #[test]
    fn raw_id_extraction() {
        let id = CanId::extended(0x555).force_no_fd();
        assert_eq!(id.raw_id(), 0x555);
        // value should have both EXTENDED and FORCE_NON_FD bits
        assert_eq!(
            id.value(),
            EXTENDED_QUALIFIER_BIT | FORCE_NON_FD_QUALIFIER_BIT | 0x555
        );
    }

    // 5 — is_base returns true only when EXTENDED bit is absent
    #[test]
    fn is_base_checks() {
        assert!(CanId::base(0).is_base());
        assert!(CanId::base(0x7FF).is_base());
        assert!(!CanId::extended(0).is_base());
    }

    // 6 — is_extended returns true only when EXTENDED bit is set
    #[test]
    fn is_extended_checks() {
        assert!(!CanId::base(0x100).is_extended());
        assert!(CanId::extended(0x100).is_extended());
    }

    // 7 — INVALID_ID value should report as invalid
    #[test]
    fn invalid_id() {
        let id = CanId(INVALID_ID);
        assert!(!id.is_valid());
        // INVALID_ID has the INVALID_QUALIFIER_BIT set
        assert_ne!(id.value() & INVALID_QUALIFIER_BIT, 0);
    }

    // 8 — CanId::id with is_extended=true behaves like CanId::extended
    #[test]
    fn id_with_extended_flag() {
        let a = CanId::id(0x00AB_CDEF, true);
        let b = CanId::extended(0x00AB_CDEF);
        assert_eq!(a, b);
    }

    // 9 — CanId::id_with_flags sets both bits when requested
    #[test]
    fn id_with_both_flags() {
        let id = CanId::id_with_flags(0x7FF, true, true);
        assert!(id.is_extended());
        assert!(id.is_force_no_fd());
        assert_eq!(id.raw_id(), 0x7FF);
    }

    // 10 — base ID at maximum 11-bit value
    #[test]
    fn base_id_max_value() {
        let id = CanId::base(MAX_RAW_BASE_ID);
        assert_eq!(id.raw_id(), u32::from(MAX_RAW_BASE_ID));
        assert!(id.is_base());
    }

    // 11 — extended ID at maximum 29-bit value
    #[test]
    fn extended_id_max_value() {
        let id = CanId::extended(MAX_RAW_EXTENDED_ID);
        assert_eq!(id.raw_id(), MAX_RAW_EXTENDED_ID);
        assert!(id.is_extended());
    }

    // 12 — Default yields a zero-valued (base, raw_id=0) identifier
    #[test]
    fn default_is_zero() {
        let id = CanId::default();
        assert_eq!(id.value(), 0);
        assert_eq!(id.raw_id(), 0);
        assert!(id.is_base());
        assert!(id.is_valid());
    }

    // 13 — PartialEq and Ord work correctly
    #[test]
    fn equality_and_ordering() {
        let a = CanId::base(0x100);
        let b = CanId::base(0x100);
        let c = CanId::base(0x200);
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert!(a < c);
        assert!(c > a);
    }

    // 14 — const fn base can be used in a const context
    #[test]
    fn compile_time_base() {
        const ID: CanId = CanId::base(0x42);
        assert_eq!(ID.raw_id(), 0x42);
        assert!(ID.is_base());
    }

    // 15 — const fn extended can be used in a const context
    #[test]
    fn compile_time_extended() {
        const ID: CanId = CanId::extended(0x1_2345);
        assert_eq!(ID.raw_id(), 0x1_2345);
        assert!(ID.is_extended());
    }
}
