// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! CAN frame type — Rust port of `OpenBSW` `CANFrame`.
//!
//! A [`CanFrame`] carries a [`CanId`], an optional timestamp, and up to
//! [`MAX_FRAME_LENGTH`] bytes of payload.  The payload buffer is always
//! stack-allocated; no heap allocation is ever performed.
//!
//! # Feature flags
//!
//! | Feature  | `MAX_FRAME_LENGTH` |
//! |----------|--------------------|
//! | *(none)* | 8 (classic CAN)    |
//! | `can-fd` | 64 (CAN-FD)        |

use core::ops::{Index, IndexMut};
use core::{error::Error, fmt};

use crate::can_id::CanId;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum payload length in bytes.
///
/// Classic CAN: 8 bytes.  Enabled with the `can-fd` feature: 64 bytes.
#[cfg(not(feature = "can-fd"))]
pub const MAX_FRAME_LENGTH: usize = 8;

/// Maximum payload length in bytes (CAN-FD: 64).
#[cfg(feature = "can-fd")]
pub const MAX_FRAME_LENGTH: usize = 64;

/// Number of overhead bits in a classic CAN frame (for bus load calculations).
pub const CAN_OVERHEAD_BITS: u8 = 47;

/// Error returned when a frame payload cannot fit the selected CAN mode.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FrameError {
    /// The requested payload is longer than [`MAX_FRAME_LENGTH`].
    PayloadTooLong {
        /// Requested byte length.
        length: usize,
        /// Maximum byte length for this build.
        maximum: usize,
    },
}

impl fmt::Display for FrameError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLong { length, maximum } => {
                write!(
                    formatter,
                    "payload length {length} exceeds maximum {maximum}"
                )
            }
        }
    }
}

impl Error for FrameError {}

// ---------------------------------------------------------------------------
// CanFrame
// ---------------------------------------------------------------------------

/// A CAN bus frame.
///
/// Stores a [`CanId`], a reception timestamp in microseconds (not compared
/// in [`PartialEq`] — mirrors C++ semantics), and a fixed-size payload buffer.
///
/// # Equality
///
/// Two frames are equal when their IDs, payload lengths, and payload bytes
/// match.  The timestamp is **not** considered.
#[derive(Clone, Debug)]
pub struct CanFrame {
    id: CanId,
    timestamp: u32,
    payload: [u8; MAX_FRAME_LENGTH],
    payload_length: u8,
}

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

impl CanFrame {
    /// Creates an empty frame: `id = 0`, `payload_length = 0`, `timestamp = 0`.
    #[inline]
    pub const fn new() -> Self {
        Self {
            id: CanId::base(0),
            timestamp: 0,
            payload: [0u8; MAX_FRAME_LENGTH],
            payload_length: 0,
        }
    }

    /// Creates a frame with the given `id` and an empty payload.
    #[inline]
    pub const fn with_id(id: CanId) -> Self {
        Self {
            id,
            timestamp: 0,
            payload: [0u8; MAX_FRAME_LENGTH],
            payload_length: 0,
        }
    }

    /// Creates a frame with the given `id` and `data` slice as payload.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() > MAX_FRAME_LENGTH`.
    pub fn with_data(id: CanId, data: &[u8]) -> Self {
        Self::try_with_data(id, data).unwrap_or_else(|error| panic!("CanFrame::with_data: {error}"))
    }

    /// Tries to create a frame with the given payload.
    pub fn try_with_data(id: CanId, data: &[u8]) -> Result<Self, FrameError> {
        if data.len() > MAX_FRAME_LENGTH {
            return Err(FrameError::PayloadTooLong {
                length: data.len(),
                maximum: MAX_FRAME_LENGTH,
            });
        }
        let mut frame = Self::with_id(id);
        frame.payload[..data.len()].copy_from_slice(data);
        // MAX_FRAME_LENGTH is at most 64 and therefore fits in u8.
        #[allow(clippy::cast_possible_truncation)]
        {
            frame.payload_length = data.len() as u8;
        }
        Ok(frame)
    }

    /// Creates a frame from a raw 32-bit ID value, a data slice, and an
    /// extended-frame flag.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() > MAX_FRAME_LENGTH`.
    pub fn with_raw_id(raw_id: u32, data: &[u8], is_extended: bool) -> Self {
        Self::with_data(CanId::id(raw_id, is_extended), data)
    }

    /// Tries to create a frame from a raw ID and payload.
    pub fn try_with_raw_id(
        raw_id: u32,
        data: &[u8],
        is_extended: bool,
    ) -> Result<Self, FrameError> {
        Self::try_with_data(CanId::id(raw_id, is_extended), data)
    }
}

// ---------------------------------------------------------------------------
// Getters / setters
// ---------------------------------------------------------------------------

impl CanFrame {
    /// Returns the frame's [`CanId`].
    #[inline]
    pub const fn id(&self) -> CanId {
        self.id
    }

    /// Replaces the frame's [`CanId`].
    #[inline]
    pub fn set_id(&mut self, id: CanId) {
        self.id = id;
    }

    /// Returns a slice of the **live** payload bytes (`0..payload_length`).
    #[inline]
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_length as usize]
    }

    /// Returns a mutable slice of the **live** payload bytes.
    #[inline]
    pub fn payload_mut(&mut self) -> &mut [u8] {
        let len = self.payload_length as usize;
        &mut self.payload[..len]
    }

    /// Copies `data` into the payload buffer and sets `payload_length`.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() > MAX_FRAME_LENGTH`.
    pub fn set_payload(&mut self, data: &[u8]) {
        self.try_set_payload(data)
            .unwrap_or_else(|error| panic!("CanFrame::set_payload: {error}"));
    }

    /// Tries to replace the payload without panicking on external input.
    pub fn try_set_payload(&mut self, data: &[u8]) -> Result<(), FrameError> {
        if data.len() > MAX_FRAME_LENGTH {
            return Err(FrameError::PayloadTooLong {
                length: data.len(),
                maximum: MAX_FRAME_LENGTH,
            });
        }
        self.payload[..data.len()].copy_from_slice(data);
        // MAX_FRAME_LENGTH is at most 64 and therefore fits in u8.
        #[allow(clippy::cast_possible_truncation)]
        {
            self.payload_length = data.len() as u8;
        }
        Ok(())
    }

    /// Returns the current payload length in bytes.
    #[inline]
    pub const fn payload_length(&self) -> u8 {
        self.payload_length
    }

    /// Sets the payload length.
    ///
    /// # Panics
    ///
    /// Panics if `len as usize > MAX_FRAME_LENGTH`.
    #[inline]
    pub fn set_payload_length(&mut self, len: u8) {
        self.try_set_payload_length(len)
            .unwrap_or_else(|error| panic!("CanFrame::set_payload_length: {error}"));
    }

    /// Tries to set the live payload length.
    #[inline]
    pub fn try_set_payload_length(&mut self, len: u8) -> Result<(), FrameError> {
        if usize::from(len) > MAX_FRAME_LENGTH {
            return Err(FrameError::PayloadTooLong {
                length: usize::from(len),
                maximum: MAX_FRAME_LENGTH,
            });
        }
        self.payload_length = len;
        Ok(())
    }

    /// Returns the maximum supported payload length (compile-time constant).
    #[inline]
    pub const fn max_payload_length() -> u8 {
        // MAX_FRAME_LENGTH is 8 or 64, both well within u8 range.
        #[allow(clippy::cast_possible_truncation)]
        {
            MAX_FRAME_LENGTH as u8
        }
    }

    /// Returns the reception timestamp in microseconds.
    #[inline]
    pub const fn timestamp(&self) -> u32 {
        self.timestamp
    }

    /// Sets the reception timestamp.
    #[inline]
    pub fn set_timestamp(&mut self, ts: u32) {
        self.timestamp = ts;
    }
}

// ---------------------------------------------------------------------------
// PartialEq — timestamp intentionally excluded (matches C++ behaviour)
// ---------------------------------------------------------------------------

impl PartialEq for CanFrame {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
            && self.payload_length == other.payload_length
            && self.payload[..self.payload_length as usize]
                == other.payload[..other.payload_length as usize]
    }
}

impl Eq for CanFrame {}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl Default for CanFrame {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Index / IndexMut over u8 index (byte offset into payload)
// ---------------------------------------------------------------------------

impl Index<u8> for CanFrame {
    type Output = u8;

    #[inline]
    fn index(&self, index: u8) -> &u8 {
        &self.payload[index as usize]
    }
}

impl IndexMut<u8> for CanFrame {
    #[inline]
    fn index_mut(&mut self, index: u8) -> &mut u8 {
        &mut self.payload[index as usize]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{CanFrame, FrameError, MAX_FRAME_LENGTH};
    use crate::can_id::CanId;

    // 1 — default / new frame has zeroed fields
    #[test]
    fn default_frame() {
        let f = CanFrame::new();
        assert_eq!(f.id(), CanId::base(0));
        assert_eq!(f.payload_length(), 0);
        assert_eq!(f.timestamp(), 0);
        assert!(f.payload().is_empty());
    }

    // 2 — with_id sets id, leaves payload empty
    #[test]
    fn frame_with_id() {
        let id = CanId::base(0x321);
        let f = CanFrame::with_id(id);
        assert_eq!(f.id(), id);
        assert_eq!(f.payload_length(), 0);
    }

    // 3 — with_data stores payload correctly
    #[test]
    fn frame_with_data() {
        let id = CanId::base(0x100);
        let data = [0xDE, 0xAD, 0xBE, 0xEF];
        let f = CanFrame::with_data(id, &data);
        assert_eq!(f.payload_length(), 4);
        assert_eq!(f.payload(), &data[..]);
    }

    // 4 — with_raw_id using extended flag
    #[test]
    fn frame_with_raw_extended_id() {
        let f = CanFrame::with_raw_id(0x0123_4567, &[1, 2, 3], true);
        assert!(f.id().is_extended());
        assert_eq!(f.id().raw_id(), 0x0123_4567);
        assert_eq!(f.payload_length(), 3);
    }

    // 5 — payload() slice length matches payload_length
    #[test]
    fn payload_slice_returns_correct_length() {
        let f = CanFrame::with_data(CanId::base(0x10), &[1, 2, 3, 4, 5]);
        assert_eq!(f.payload().len(), 5);
    }

    // 6 — set_payload copies data and updates length
    #[test]
    fn set_payload_copies_data() {
        let mut f = CanFrame::new();
        f.set_payload(&[0xAA, 0xBB]);
        assert_eq!(f.payload_length(), 2);
        assert_eq!(f.payload(), &[0xAA, 0xBB]);
    }

    // 7 — set_payload_length changes reported length
    #[test]
    fn set_payload_length() {
        let mut f = CanFrame::with_data(CanId::base(0), &[1, 2, 3, 4]);
        assert_eq!(f.payload_length(), 4);
        f.set_payload_length(2);
        assert_eq!(f.payload_length(), 2);
        assert_eq!(f.payload(), &[1, 2]);
    }

    // 8 — timestamp is NOT compared in PartialEq
    #[test]
    fn timestamp_not_compared_in_eq() {
        let mut a = CanFrame::with_data(CanId::base(0x10), &[1, 2]);
        let mut b = CanFrame::with_data(CanId::base(0x10), &[1, 2]);
        a.set_timestamp(100);
        b.set_timestamp(999);
        assert_eq!(a, b); // must still be equal
    }

    // 9 — Index<u8> gives direct byte access
    #[test]
    fn index_access() {
        let f = CanFrame::with_data(CanId::base(0), &[10, 20, 30]);
        assert_eq!(f[0], 10);
        assert_eq!(f[1], 20);
        assert_eq!(f[2], 30);
    }

    // 10 — two identical frames are equal
    #[test]
    fn equality_same_frames() {
        let a = CanFrame::with_data(CanId::base(0x77), &[0xDE, 0xAD]);
        let b = CanFrame::with_data(CanId::base(0x77), &[0xDE, 0xAD]);
        assert_eq!(a, b);
    }

    // 11 — different IDs → not equal
    #[test]
    fn equality_different_ids() {
        let a = CanFrame::with_data(CanId::base(0x10), &[1]);
        let b = CanFrame::with_data(CanId::base(0x20), &[1]);
        assert_ne!(a, b);
    }

    // 12 — different payload bytes → not equal
    #[test]
    fn equality_different_payload() {
        let a = CanFrame::with_data(CanId::base(0x10), &[1, 2, 3]);
        let b = CanFrame::with_data(CanId::base(0x10), &[1, 2, 4]);
        assert_ne!(a, b);
    }

    // 13 — different payload lengths → not equal
    #[test]
    fn equality_different_length() {
        let a = CanFrame::with_data(CanId::base(0x10), &[1, 2, 3]);
        let b = CanFrame::with_data(CanId::base(0x10), &[1, 2]);
        assert_ne!(a, b);
    }

    // 14 — max_payload_length matches compile-time constant
    #[test]
    fn max_payload_length() {
        assert_eq!(CanFrame::max_payload_length() as usize, MAX_FRAME_LENGTH);
    }

    // 15 — set_payload panics when data is too long
    #[test]
    #[should_panic(expected = "exceeds maximum")]
    fn set_payload_panics_on_overflow() {
        let mut f = CanFrame::new();
        let too_long = [0u8; MAX_FRAME_LENGTH + 1];
        f.set_payload(&too_long);
    }

    #[test]
    fn fallible_payload_apis_report_capacity() {
        let too_long = [0u8; MAX_FRAME_LENGTH + 1];
        let expected = FrameError::PayloadTooLong {
            length: MAX_FRAME_LENGTH + 1,
            maximum: MAX_FRAME_LENGTH,
        };
        assert_eq!(
            CanFrame::try_with_data(CanId::base(0), &too_long),
            Err(expected)
        );

        let mut frame = CanFrame::new();
        assert_eq!(frame.try_set_payload(&too_long), Err(expected));
        assert_eq!(
            frame.try_set_payload_length((MAX_FRAME_LENGTH + 1) as u8),
            Err(expected)
        );
    }
}
