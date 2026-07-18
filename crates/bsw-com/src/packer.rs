// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Signal packing and unpacking into/from CAN PDU byte buffers.
//!
//! Part of the recorded `extension.com` **project extension** — upstream
//! Eclipse OpenBSW has no general COM stack, so this module follows the
//! de-facto DBC signal-layout conventions (validated by hand-computed
//! reference vectors and property tests) rather than a C++ oracle.
//!
//! # Scope
//!
//! - Arbitrary (non-byte-aligned) bit positions; numeric widths 1..=32.
//! - Frames up to 64 bytes (CAN FD, [`FD_MAX_LENGTH`]).
//! - Intel (little-endian) and Motorola (big-endian, DBC sawtooth) layouts.
//! - Two's-complement signed signals at any width, e.g. a 12-bit signal
//!   carried in `Sint16` (range −2048..=2047).
//! - Byte arrays (`Uint8N`) of 1..=[`MAX_BYTE_ARRAY_LEN`] bytes,
//!   byte-aligned only.  The bound comes from the `u32` carrier in
//!   [`SignalValue`]; longer arrays are rejected with
//!   [`PackError::ByteArrayTooLong`] — never silently truncated.
//!
//! All entry points return typed [`PackError`]s on malformed descriptors
//! instead of panicking (the byte-aligned v1 packer used `assert!`).
//!
//! # Bit numbering convention
//!
//! Bits inside a byte use LSB0 numbering: bit 0 is the least significant
//! bit of a byte, bit 7 the most significant.  A descriptor's
//! `bit_position` is `byte_index * 8 + bit_in_byte`.
//!
//! ## Intel (`ByteOrder::LittleEndian`)
//!
//! `bit_position` addresses the **LSBit** of the signal.  Value bits fill
//! toward increasing bit positions, crossing into the next-higher byte
//! after bit 7.
//!
//! A 12-bit Intel signal at `bit_position = 4` with raw value `0xABC`:
//!
//! ```text
//! byte 0 = 0xC0   (value bits 3..0 in byte 0, bits 7..4)
//! byte 1 = 0xAB   (value bits 11..4)
//! ```
//!
//! ## Motorola (`ByteOrder::BigEndian`)
//!
//! `bit_position` addresses the **MSBit** of the signal (the DBC start
//! bit).  Value bits fill toward decreasing bit numbers within a byte;
//! after bit 0 the layout continues at bit 7 of the *next-higher* byte
//! index — the DBC "sawtooth".
//!
//! The classic example — a 12-bit Motorola signal at start bit 7 with raw
//! value `0xABC`:
//!
//! ```text
//! byte 0 = 0xAB   (value bits 11..4 in byte 0, bits 7..0)
//! byte 1 = 0xC0   (value bits 3..0  in byte 1, bits 7..4)
//! ```
//!
//! Byte-aligned Motorola signals therefore use `bit_position =
//! first_byte * 8 + 7` (the MSBit of the first byte) — **not**
//! `first_byte * 8` as accepted by the byte-aligned v1 packer.
//!
//! # Error precedence
//!
//! Structural descriptor errors (zero width, width vs. type, byte-array
//! alignment) are reported first, then the 64-byte CAN FD structural cap
//! ([`PackError::BitRangeOutOfBounds`]), then the concrete buffer bound
//! ([`PackError::BufferTooSmall`]).

use crate::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
use bsw_can::dlc::FD_MAX_LENGTH;

// ---------------------------------------------------------------------------
// PackError
// ---------------------------------------------------------------------------

/// Maximum supported `Uint8N` byte-array length.
///
/// Bounded by the `u32` carrier in [`SignalValue`]; longer arrays are
/// rejected with [`PackError::ByteArrayTooLong`] rather than silently
/// truncated.
pub const MAX_BYTE_ARRAY_LEN: usize = 4;

/// Typed packing / descriptor-validation errors.
///
/// Replaces the `assert!` panics of the byte-aligned v1 packer: every
/// packer entry point returns `Err` on malformed input instead of
/// panicking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PackError {
    /// `bit_size` is zero (or the descriptor is `Uint8N(0)`).
    ZeroWidth,
    /// `bit_size` exceeds the capacity of `signal_type`
    /// (e.g. `Uint8` with `bit_size > 8`, `Sint16` with `bit_size > 16`).
    WidthExceedsType,
    /// The field extends beyond the PDU length passed to
    /// [`SignalDescriptor::validate`], or beyond the 64-byte CAN FD
    /// maximum.
    BitRangeOutOfBounds,
    /// The field extends beyond the buffer passed to pack/unpack.
    BufferTooSmall,
    /// `Uint8N` byte arrays must start on a byte boundary
    /// (`bit_position % 8 == 0`).
    ByteArrayNotByteAligned,
    /// `Uint8N` length exceeds [`MAX_BYTE_ARRAY_LEN`].
    ByteArrayTooLong,
    /// `Uint8N(n)` requires `bit_size == 8 * n`.
    ByteArrayWidthMismatch,
    /// PDU length passed to [`SignalDescriptor::validate`] exceeds the
    /// 64-byte CAN FD maximum.
    InvalidPduLength,
    /// Scaling has a zero numerator or denominator.
    InvalidScaling,
    /// Intermediate scaling arithmetic overflowed `i64`.
    ScalingOverflow,
    /// Physical value maps to a raw value outside the field's range.
    ValueOutOfRange,
    /// The operation does not support this signal type (the
    /// physical-value helpers reject `Uint8N`).
    UnsupportedSignalType,
}

// ---------------------------------------------------------------------------
// Descriptor geometry helpers
// ---------------------------------------------------------------------------

/// Maximum representable width in bits for a numeric signal type.
const fn numeric_max_bits(signal_type: SignalType) -> u8 {
    match signal_type {
        SignalType::Uint8 | SignalType::Sint8 => 8,
        SignalType::Uint16 | SignalType::Sint16 => 16,
        // Boolean fields may be declared at any supported width; any
        // non-zero raw pattern reads back as `true`.
        SignalType::Boolean | SignalType::Uint32 | SignalType::Sint32 | SignalType::Uint8N(_) => 32,
    }
}

const fn is_signed(signal_type: SignalType) -> bool {
    matches!(
        signal_type,
        SignalType::Sint8 | SignalType::Sint16 | SignalType::Sint32
    )
}

/// Bit mask covering the low `width` bits (total for all `u8` inputs).
const fn width_mask(width: u8) -> u32 {
    if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    }
}

/// Number of buffer bytes (counted from index 0) the descriptor's field
/// requires, after structural validation of the descriptor itself.
fn required_bytes(descriptor: &SignalDescriptor) -> Result<usize, PackError> {
    let bit_position = descriptor.bit_position as usize;
    match descriptor.signal_type {
        SignalType::Uint8N(n) => {
            if n == 0 {
                return Err(PackError::ZeroWidth);
            }
            if n > MAX_BYTE_ARRAY_LEN {
                return Err(PackError::ByteArrayTooLong);
            }
            if !bit_position.is_multiple_of(8) {
                return Err(PackError::ByteArrayNotByteAligned);
            }
            if descriptor.bit_size as usize != n * 8 {
                return Err(PackError::ByteArrayWidthMismatch);
            }
            Ok(bit_position / 8 + n)
        }
        signal_type => {
            let width = descriptor.bit_size;
            if width == 0 {
                return Err(PackError::ZeroWidth);
            }
            if width > numeric_max_bits(signal_type) {
                return Err(PackError::WidthExceedsType);
            }
            // Linear start index in the byte-order's fill direction:
            // Intel counts LSB0 positions upward from `bit_position`;
            // Motorola consumes MSB-first positions starting at
            // `byte * 8 + (7 - bit_in_byte)`.
            let start = match descriptor.byte_order {
                ByteOrder::LittleEndian => bit_position,
                ByteOrder::BigEndian => (bit_position / 8) * 8 + (7 - bit_position % 8),
            };
            Ok((start + width as usize).div_ceil(8))
        }
    }
}

/// Full descriptor check against an available byte count.
///
/// `bound_error` selects the error reported when the (structurally valid)
/// field does not fit into `available_bytes` — `BitRangeOutOfBounds` for
/// PDU-length validation, `BufferTooSmall` for concrete buffers.
fn check_fits(
    descriptor: &SignalDescriptor,
    available_bytes: usize,
    bound_error: PackError,
) -> Result<(), PackError> {
    let needed = required_bytes(descriptor)?;
    if needed > FD_MAX_LENGTH {
        return Err(PackError::BitRangeOutOfBounds);
    }
    if needed > available_bytes {
        return Err(bound_error);
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// SignalDescriptor::validate
// ---------------------------------------------------------------------------

impl SignalDescriptor {
    /// Validate this descriptor against a PDU payload length in bytes.
    ///
    /// Checks, in order: PDU length (≤ 64 bytes), structural descriptor
    /// consistency (width, type capacity, byte-array rules), the field's
    /// bit range against the PDU length, and — if present — the scaling
    /// metadata.
    ///
    /// # Errors
    ///
    /// Any [`PackError`] variant except [`PackError::BufferTooSmall`],
    /// [`PackError::ScalingOverflow`], [`PackError::ValueOutOfRange`] and
    /// [`PackError::UnsupportedSignalType`].
    pub fn validate(&self, pdu_length_bytes: usize) -> Result<(), PackError> {
        if pdu_length_bytes > FD_MAX_LENGTH {
            return Err(PackError::InvalidPduLength);
        }
        check_fits(self, pdu_length_bytes, PackError::BitRangeOutOfBounds)?;
        if let Some(scaling) = self.scaling {
            scaling.validate()?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// pack_signal
// ---------------------------------------------------------------------------

#[inline]
fn set_bit(buf: &mut [u8], byte: usize, shift: usize, bit: u8) {
    buf[byte] = (buf[byte] & !(1u8 << shift)) | (bit << shift);
}

/// Pack `value` into `buf` at the position described by `descriptor`.
///
/// Values wider than `bit_size` are truncated to the low `bit_size` bits
/// (two's-complement truncation for signed values) — this is the
/// documented DBC packing semantic, mirrored by [`unpack_signal`]'s sign
/// extension.  Bits of `buf` outside the field are left untouched.
///
/// # Errors
///
/// Returns a [`PackError`] instead of panicking when the descriptor is
/// malformed or the field does not fit into `buf`.
pub fn pack_signal(
    buf: &mut [u8],
    descriptor: &SignalDescriptor,
    value: SignalValue,
) -> Result<(), PackError> {
    check_fits(descriptor, buf.len(), PackError::BufferTooSmall)?;

    let bit_position = descriptor.bit_position as usize;

    // Byte arrays: copied in ascending byte order regardless of the
    // descriptor's `byte_order` (LSByte of the u32 carrier first).
    if let SignalType::Uint8N(n) = descriptor.signal_type {
        let start = bit_position / 8;
        let raw = value.as_u32();
        for (i, byte) in buf[start..start + n].iter_mut().enumerate() {
            *byte = (raw >> (i * 8)) as u8;
        }
        return Ok(());
    }

    let width = u32::from(descriptor.bit_size);
    let raw = value.as_u32() & width_mask(descriptor.bit_size);

    match descriptor.byte_order {
        ByteOrder::LittleEndian => {
            // LSBit of the value at `bit_position`, filling upward.
            for i in 0..width {
                let pos = bit_position + i as usize;
                let bit = ((raw >> i) & 1) as u8;
                set_bit(buf, pos / 8, pos % 8, bit);
            }
        }
        ByteOrder::BigEndian => {
            // MSBit of the value at `bit_position`, filling down the
            // sawtooth (bit_in_byte 0 wraps to next byte's bit 7).
            let mut byte = bit_position / 8;
            let mut shift = bit_position % 8;
            let mut i = width;
            while i > 0 {
                i -= 1;
                let bit = ((raw >> i) & 1) as u8;
                set_bit(buf, byte, shift, bit);
                if shift == 0 {
                    byte += 1;
                    shift = 7;
                } else {
                    shift -= 1;
                }
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// unpack_signal
// ---------------------------------------------------------------------------

/// Unpack a signal value from `buf` at the position described by
/// `descriptor`.
///
/// Signed signals are sign-extended from `bit_size` bits (e.g. a 12-bit
/// `Sint16` field reads back −2048..=2047).  Boolean signals read `true`
/// for any non-zero field content.
///
/// # Errors
///
/// Returns a [`PackError`] instead of panicking when the descriptor is
/// malformed or the field does not fit into `buf`.
pub fn unpack_signal(buf: &[u8], descriptor: &SignalDescriptor) -> Result<SignalValue, PackError> {
    check_fits(descriptor, buf.len(), PackError::BufferTooSmall)?;

    let bit_position = descriptor.bit_position as usize;

    if let SignalType::Uint8N(n) = descriptor.signal_type {
        let start = bit_position / 8;
        let mut raw = 0u32;
        for (i, byte) in buf[start..start + n].iter().enumerate() {
            raw |= u32::from(*byte) << (i * 8);
        }
        return Ok(SignalValue::U32(raw));
    }

    let width = u32::from(descriptor.bit_size);
    let mask = width_mask(descriptor.bit_size);
    let mut raw = 0u32;

    match descriptor.byte_order {
        ByteOrder::LittleEndian => {
            for i in 0..width {
                let pos = bit_position + i as usize;
                let bit = u32::from((buf[pos / 8] >> (pos % 8)) & 1);
                raw |= bit << i;
            }
        }
        ByteOrder::BigEndian => {
            let mut byte = bit_position / 8;
            let mut shift = bit_position % 8;
            for _ in 0..width {
                raw = (raw << 1) | u32::from((buf[byte] >> shift) & 1);
                if shift == 0 {
                    byte += 1;
                    shift = 7;
                } else {
                    shift -= 1;
                }
            }
        }
    }

    if matches!(descriptor.signal_type, SignalType::Boolean) {
        return Ok(SignalValue::Bool(raw != 0));
    }

    if is_signed(descriptor.signal_type) && width < 32 && (raw >> (width - 1)) & 1 == 1 {
        raw |= !mask;
    }

    Ok(SignalValue::from_u32(raw, descriptor.signal_type))
}

// ---------------------------------------------------------------------------
// Physical-value helpers (integer linear scaling, project extension)
// ---------------------------------------------------------------------------

/// Signed 64-bit view of a raw signal value (sign-extended for signed
/// types, zero-extended otherwise).
fn raw_to_i64(value: SignalValue) -> i64 {
    match value {
        SignalValue::Bool(b) => i64::from(b),
        SignalValue::U8(v) => i64::from(v),
        SignalValue::U16(v) => i64::from(v),
        SignalValue::U32(v) => i64::from(v),
        SignalValue::I8(v) => i64::from(v),
        SignalValue::I16(v) => i64::from(v),
        SignalValue::I32(v) => i64::from(v),
    }
}

/// Inclusive raw-value range representable by a field of `width` bits.
///
/// `width` must already be validated to 1..=32.
fn raw_range(signal_type: SignalType, width: u8) -> (i64, i64) {
    if matches!(signal_type, SignalType::Boolean) {
        return (0, 1);
    }
    if is_signed(signal_type) {
        let half = 1i64 << (width - 1);
        (-half, half - 1)
    } else {
        (0, i64::from(width_mask(width)))
    }
}

/// Unpack a signal and convert it to its physical value.
///
/// Without scaling metadata this is the raw value as `i64` (identity
/// path).  With scaling it applies
/// [`Scaling::raw_to_physical`](crate::signal::Scaling::raw_to_physical).
///
/// # Errors
///
/// [`PackError::UnsupportedSignalType`] for `Uint8N` descriptors, plus
/// any error of [`unpack_signal`] or the scaling conversion.
pub fn unpack_physical(buf: &[u8], descriptor: &SignalDescriptor) -> Result<i64, PackError> {
    if matches!(descriptor.signal_type, SignalType::Uint8N(_)) {
        return Err(PackError::UnsupportedSignalType);
    }
    let value = unpack_signal(buf, descriptor)?;
    let raw = raw_to_i64(value);
    match descriptor.scaling {
        None => Ok(raw),
        Some(scaling) => scaling.raw_to_physical(raw),
    }
}

/// Convert a physical value to raw and pack it.
///
/// Without scaling metadata `physical` is taken as the raw value
/// directly (identity path).  The resulting raw value is range-checked
/// against the field's width and signedness before packing — out-of-range
/// values are rejected, never truncated.
///
/// # Errors
///
/// [`PackError::UnsupportedSignalType`] for `Uint8N` descriptors,
/// [`PackError::ValueOutOfRange`] when the raw value does not fit the
/// field, plus any error of the scaling conversion or [`pack_signal`].
pub fn pack_physical(
    buf: &mut [u8],
    descriptor: &SignalDescriptor,
    physical: i64,
) -> Result<(), PackError> {
    if matches!(descriptor.signal_type, SignalType::Uint8N(_)) {
        return Err(PackError::UnsupportedSignalType);
    }
    check_fits(descriptor, buf.len(), PackError::BufferTooSmall)?;

    let raw = match descriptor.scaling {
        None => physical,
        Some(scaling) => scaling.physical_to_raw(physical)?,
    };

    let (min, max) = raw_range(descriptor.signal_type, descriptor.bit_size);
    if raw < min || raw > max {
        return Err(PackError::ValueOutOfRange);
    }

    pack_signal(
        buf,
        descriptor,
        SignalValue::from_u32(raw as u32, descriptor.signal_type),
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{pack_physical, pack_signal, unpack_physical, unpack_signal, PackError};
    use crate::signal::{ByteOrder, Scaling, SignalDescriptor, SignalType, SignalValue};

    fn make_desc(
        id: u16,
        bit_position: u16,
        bit_size: u8,
        sig_type: SignalType,
        byte_order: ByteOrder,
        init_value: u32,
    ) -> SignalDescriptor {
        SignalDescriptor::new(id, bit_position, bit_size, sig_type, byte_order, init_value)
    }

    // 1 — pack/unpack u8 at byte boundary (LE)
    #[test]
    fn u8_at_byte_boundary_le() {
        let desc = make_desc(1, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0xAB)).unwrap();
        assert_eq!(buf[1], 0xAB);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U8(0xAB)));
    }

    // 2 — pack/unpack u16 little-endian
    #[test]
    fn u16_little_endian() {
        let desc = make_desc(2, 0, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0x1234)).unwrap();
        // LE: LSByte at buf[0], MSByte at buf[1]
        assert_eq!(buf[0], 0x34);
        assert_eq!(buf[1], 0x12);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0x1234)));
    }

    // 3 — pack/unpack u16 big-endian (DBC start bit 7 = MSBit of byte 0)
    #[test]
    fn u16_big_endian() {
        let desc = make_desc(3, 7, 16, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0x1234)).unwrap();
        // BE: MSByte at buf[0], LSByte at buf[1]
        assert_eq!(buf[0], 0x12);
        assert_eq!(buf[1], 0x34);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0x1234)));
    }

    // 4 — pack/unpack u32 little-endian
    #[test]
    fn u32_little_endian() {
        let desc = make_desc(4, 0, 32, SignalType::Uint32, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0xDEAD_BEEF)).unwrap();
        assert_eq!(&buf[0..4], &[0xEF, 0xBE, 0xAD, 0xDE]);
        assert_eq!(
            unpack_signal(&buf, &desc),
            Ok(SignalValue::U32(0xDEAD_BEEF))
        );
    }

    // 5 — pack/unpack boolean (8-bit field, byte-aligned)
    #[test]
    fn boolean_pack_unpack() {
        let desc = make_desc(5, 8, 8, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::Bool(true)).unwrap();
        assert_eq!(buf[1], 1);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::Bool(true)));

        pack_signal(&mut buf, &desc, SignalValue::Bool(false)).unwrap();
        assert_eq!(buf[1], 0);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::Bool(false)));
    }

    // 6 — multiple signals in one PDU do not overlap
    #[test]
    fn multiple_signals_no_overlap() {
        let desc_a = make_desc(10, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let desc_b = make_desc(11, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let desc_c = make_desc(12, 16, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];

        pack_signal(&mut buf, &desc_a, SignalValue::U8(0x11)).unwrap();
        pack_signal(&mut buf, &desc_b, SignalValue::U8(0x22)).unwrap();
        pack_signal(&mut buf, &desc_c, SignalValue::U16(0x3344)).unwrap();

        assert_eq!(unpack_signal(&buf, &desc_a), Ok(SignalValue::U8(0x11)));
        assert_eq!(unpack_signal(&buf, &desc_b), Ok(SignalValue::U8(0x22)));
        assert_eq!(unpack_signal(&buf, &desc_c), Ok(SignalValue::U16(0x3344)));
    }

    // 7 — signed i8 negative round-trip
    #[test]
    fn i8_negative_pack_unpack() {
        let desc = make_desc(20, 0, 8, SignalType::Sint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::I8(-42)).unwrap();
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::I8(-42)));
    }

    // 8 — signed i16 negative round-trip
    #[test]
    fn i16_negative_pack_unpack() {
        let desc = make_desc(21, 0, 16, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::I16(-1000)).unwrap();
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::I16(-1000)));
    }

    // 9 — pack/unpack at byte 7 (last byte of 8-byte PDU)
    #[test]
    fn u8_at_last_byte() {
        let desc = make_desc(30, 56, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0xFF)).unwrap();
        assert_eq!(buf[7], 0xFF);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U8(0xFF)));
    }

    // 10 — pack does not corrupt adjacent bytes
    #[test]
    fn pack_does_not_corrupt_adjacent() {
        let desc = make_desc(40, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0xAAu8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0x55)).unwrap();
        assert_eq!(buf[0], 0xAA); // byte 0 untouched
        assert_eq!(buf[1], 0x55); // written
        assert_eq!(buf[2], 0xAA); // byte 2 untouched
    }

    // 11 — boolean non-zero raw → true
    #[test]
    fn boolean_nonzero_is_true() {
        let desc = make_desc(50, 0, 8, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let buf = [0x05, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::Bool(true)));
    }

    // 12 — u32 big-endian round-trip (DBC start bit 7)
    #[test]
    fn u32_big_endian() {
        let desc = make_desc(60, 7, 32, SignalType::Uint32, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0x0102_0304)).unwrap();
        assert_eq!(&buf[0..4], &[0x01, 0x02, 0x03, 0x04]);
        assert_eq!(
            unpack_signal(&buf, &desc),
            Ok(SignalValue::U32(0x0102_0304))
        );
    }

    // 13 — u16 BE at byte offset 4 (DBC start bit 39 = byte 4, bit 7)
    #[test]
    fn u16_be_offset() {
        let desc = make_desc(70, 39, 16, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABCD)).unwrap();
        assert_eq!(buf[4], 0xAB);
        assert_eq!(buf[5], 0xCD);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0xABCD)));
    }

    // -----------------------------------------------------------------------
    // Hand-computed Motorola (DBC sawtooth) reference vectors
    // -----------------------------------------------------------------------

    // Vector 1 (classic): 12-bit BE at start bit 7, value 0xABC.
    // Byte 0 bits 7..0 <- value bits 11..4 (0xAB);
    // byte 1 bits 7..4 <- value bits 3..0 (0xC).
    #[test]
    fn motorola_vector_12bit_start_bit_7() {
        let desc = make_desc(100, 7, 12, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABC)).unwrap();
        assert_eq!(&buf[0..2], &[0xAB, 0xC0]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0xABC)));
    }

    // Vector 2: 12-bit BE at start bit 3 (byte 0, bit 3), value 0xABC.
    // Byte 0 bits 3..0 <- value bits 11..8 (0xA);
    // byte 1 bits 7..0 <- value bits 7..0 (0xBC).
    #[test]
    fn motorola_vector_12bit_start_bit_3() {
        let desc = make_desc(101, 3, 12, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABC)).unwrap();
        assert_eq!(&buf[0..2], &[0x0A, 0xBC]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0xABC)));
    }

    // Vector 3: 16-bit BE at start bit 5 (byte 0, bit 5), value 0x1234.
    // 0x1234 = 0b0001_0010_0011_0100.
    // Byte 0 bits 5..0 <- value bits 15..10 (0b000100 = 0x04);
    // byte 1 bits 7..0 <- value bits 9..2  (0b1000_1101 = 0x8D);
    // byte 2 bits 7..6 <- value bits 1..0  (0b00) -> 0x00.
    #[test]
    fn motorola_vector_16bit_start_bit_5() {
        let desc = make_desc(102, 5, 16, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0x1234)).unwrap();
        assert_eq!(&buf[0..3], &[0x04, 0x8D, 0x00]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0x1234)));
    }

    // Vector 4: 8-bit BE at start bit 0 (byte 0, bit 0) — sawtooth wrap.
    // Value 0xA5 = 0b1010_0101.
    // Byte 0 bit 0    <- value bit 7 (1) -> 0x01;
    // byte 1 bits 7..1 <- value bits 6..0 (0b0100101) -> 0x4A.
    #[test]
    fn motorola_vector_wraps_from_bit_0() {
        let desc = make_desc(103, 0, 8, SignalType::Uint8, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0xA5)).unwrap();
        assert_eq!(&buf[0..2], &[0x01, 0x4A]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U8(0xA5)));
    }

    // -----------------------------------------------------------------------
    // Intel cross-byte vectors
    // -----------------------------------------------------------------------

    // 12-bit LE at bit_position 4, value 0xABC:
    // byte 0 bits 7..4 <- value bits 3..0 (0xC) -> 0xC0;
    // byte 1 bits 7..0 <- value bits 11..4 (0xAB).
    #[test]
    fn intel_vector_12bit_bit_position_4() {
        let desc = make_desc(110, 4, 12, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABC)).unwrap();
        assert_eq!(&buf[0..2], &[0xC0, 0xAB]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0xABC)));
    }

    // 5-bit LE at bit_position 6, value 0x16 = 0b10110:
    // byte 0 bits 7..6 <- value bits 1..0 (0b10) -> 0x80;
    // byte 1 bits 2..0 <- value bits 4..2 (0b101) -> 0x05.
    #[test]
    fn intel_vector_5bit_cross_byte() {
        let desc = make_desc(111, 6, 5, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0x16)).unwrap();
        assert_eq!(&buf[0..2], &[0x80, 0x05]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U8(0x16)));
    }

    // -----------------------------------------------------------------------
    // Signed boundaries and truncation
    // -----------------------------------------------------------------------

    // 12-bit signed in Sint16, BE at start bit 7 — byte images at the
    // boundaries: -2048 -> 0x800, 2047 -> 0x7FF, -1 -> 0xFFF.
    #[test]
    fn signed_12bit_boundaries_be_byte_images() {
        let desc = make_desc(120, 7, 12, SignalType::Sint16, ByteOrder::BigEndian, 0);
        let cases = [
            (-2048i16, [0x80u8, 0x00]),
            (2047, [0x7F, 0xF0]),
            (-1, [0xFF, 0xF0]),
        ];
        for (value, image) in cases {
            let mut buf = [0u8; 8];
            pack_signal(&mut buf, &desc, SignalValue::I16(value)).unwrap();
            assert_eq!(&buf[0..2], &image, "value {value}");
            assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::I16(value)));
        }
    }

    // 12-bit signed in Sint16, LE at an unaligned position.
    #[test]
    fn signed_12bit_le_unaligned_boundaries() {
        let desc = make_desc(121, 5, 12, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        for value in [-2048i16, -1, 0, 1, 2047] {
            let mut buf = [0xFFu8; 8];
            pack_signal(&mut buf, &desc, SignalValue::I16(value)).unwrap();
            assert_eq!(
                unpack_signal(&buf, &desc),
                Ok(SignalValue::I16(value)),
                "value {value}"
            );
        }
    }

    // Packing a value that does not fit the field width truncates to the
    // low bits (documented two's-complement semantics, no panic).
    #[test]
    fn signed_truncation_on_pack_is_twos_complement() {
        let desc = make_desc(122, 0, 12, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        // -3000 as u32 = 0xFFFF_F448; low 12 bits = 0x448 = +1096.
        pack_signal(&mut buf, &desc, SignalValue::I16(-3000)).unwrap();
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::I16(1096)));
    }

    // Boolean at a single arbitrary bit position.
    #[test]
    fn boolean_single_bit_unaligned() {
        let desc = make_desc(123, 5, 1, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::Bool(true)).unwrap();
        assert_eq!(buf[0], 0x20);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::Bool(true)));
        pack_signal(&mut buf, &desc, SignalValue::Bool(false)).unwrap();
        assert_eq!(buf[0], 0x00);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::Bool(false)));
    }

    // -----------------------------------------------------------------------
    // CAN FD payloads
    // -----------------------------------------------------------------------

    #[test]
    fn fd_frame_le_u32_at_byte_60() {
        let desc = make_desc(130, 480, 32, SignalType::Uint32, ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(64), Ok(()));
        assert_eq!(desc.validate(32), Err(PackError::BitRangeOutOfBounds));
        let mut buf = [0u8; 64];
        pack_signal(&mut buf, &desc, SignalValue::U32(0xCAFE_F00D)).unwrap();
        assert_eq!(&buf[60..64], &[0x0D, 0xF0, 0xFE, 0xCA]);
        assert_eq!(
            unpack_signal(&buf, &desc),
            Ok(SignalValue::U32(0xCAFE_F00D))
        );
    }

    #[test]
    fn fd_frame_be_u16_across_bytes_62_63() {
        // Start bit: byte 62, bit 3 -> 12-bit signal ending at byte 63 bit 0.
        let desc = make_desc(
            131,
            62 * 8 + 3,
            12,
            SignalType::Uint16,
            ByteOrder::BigEndian,
            0,
        );
        assert_eq!(desc.validate(64), Ok(()));
        let mut buf = [0u8; 64];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABC)).unwrap();
        assert_eq!(&buf[62..64], &[0x0A, 0xBC]);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(0xABC)));
    }

    // -----------------------------------------------------------------------
    // Byte arrays (Uint8N)
    // -----------------------------------------------------------------------

    #[test]
    fn uint8n_round_trip_four_bytes() {
        let desc = make_desc(
            140,
            16,
            32,
            SignalType::Uint8N(4),
            ByteOrder::LittleEndian,
            0,
        );
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0xAABB_CCDD)).unwrap();
        assert_eq!(&buf[2..6], &[0xDD, 0xCC, 0xBB, 0xAA]);
        assert_eq!(
            unpack_signal(&buf, &desc),
            Ok(SignalValue::U32(0xAABB_CCDD))
        );
    }

    #[test]
    fn uint8n_two_bytes_leaves_neighbors_untouched() {
        let desc = make_desc(141, 8, 16, SignalType::Uint8N(2), ByteOrder::BigEndian, 0);
        let mut buf = [0x55u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0x1234)).unwrap();
        assert_eq!(buf[0], 0x55);
        assert_eq!(&buf[1..3], &[0x34, 0x12]);
        assert_eq!(buf[3], 0x55);
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U32(0x1234)));
    }

    // -----------------------------------------------------------------------
    // Invalid descriptors — every error variant reachable, no panics
    // -----------------------------------------------------------------------

    #[test]
    fn error_zero_width() {
        let desc = make_desc(150, 0, 0, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        assert_eq!(
            pack_signal(&mut buf, &desc, SignalValue::U8(1)),
            Err(PackError::ZeroWidth)
        );
        assert_eq!(unpack_signal(&buf, &desc), Err(PackError::ZeroWidth));
        assert_eq!(desc.validate(8), Err(PackError::ZeroWidth));

        let desc = make_desc(151, 0, 0, SignalType::Uint8N(0), ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(8), Err(PackError::ZeroWidth));
    }

    #[test]
    fn error_width_exceeds_type() {
        let mut buf = [0u8; 8];
        let desc = make_desc(152, 0, 9, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        assert_eq!(
            pack_signal(&mut buf, &desc, SignalValue::U8(1)),
            Err(PackError::WidthExceedsType)
        );
        let desc = make_desc(153, 0, 17, SignalType::Sint16, ByteOrder::BigEndian, 0);
        assert_eq!(desc.validate(8), Err(PackError::WidthExceedsType));
        let desc = make_desc(154, 0, 33, SignalType::Uint32, ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(8), Err(PackError::WidthExceedsType));
    }

    #[test]
    fn error_bit_range_out_of_bounds() {
        // Fits in 8 bytes structurally, but not in a 4-byte PDU.
        let desc = make_desc(155, 40, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(4), Err(PackError::BitRangeOutOfBounds));
        // Beyond the 64-byte FD cap: structural error even with a larger buffer.
        let desc = make_desc(
            156,
            65 * 8,
            8,
            SignalType::Uint8,
            ByteOrder::LittleEndian,
            0,
        );
        let mut buf = [0u8; 128];
        assert_eq!(
            pack_signal(&mut buf, &desc, SignalValue::U8(1)),
            Err(PackError::BitRangeOutOfBounds)
        );
    }

    #[test]
    fn error_buffer_too_small() {
        let desc = make_desc(157, 8, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 2];
        assert_eq!(
            pack_signal(&mut buf, &desc, SignalValue::U16(1)),
            Err(PackError::BufferTooSmall)
        );
        assert_eq!(unpack_signal(&buf, &desc), Err(PackError::BufferTooSmall));
    }

    #[test]
    fn error_byte_array_variants() {
        let mut buf = [0u8; 8];
        // Not byte-aligned.
        let desc = make_desc(
            158,
            4,
            16,
            SignalType::Uint8N(2),
            ByteOrder::LittleEndian,
            0,
        );
        assert_eq!(
            pack_signal(&mut buf, &desc, SignalValue::U32(1)),
            Err(PackError::ByteArrayNotByteAligned)
        );
        // Longer than the u32 carrier.
        let desc = make_desc(
            159,
            0,
            40,
            SignalType::Uint8N(5),
            ByteOrder::LittleEndian,
            0,
        );
        assert_eq!(desc.validate(8), Err(PackError::ByteArrayTooLong));
        // bit_size inconsistent with n.
        let desc = make_desc(160, 0, 8, SignalType::Uint8N(2), ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(8), Err(PackError::ByteArrayWidthMismatch));
    }

    #[test]
    fn error_invalid_pdu_length() {
        let desc = make_desc(161, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        assert_eq!(desc.validate(65), Err(PackError::InvalidPduLength));
        assert_eq!(desc.validate(64), Ok(()));
    }

    #[test]
    fn error_invalid_scaling_via_validate() {
        let desc = make_desc(162, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0)
            .with_scaling(Scaling::new(1, 0, 0));
        assert_eq!(desc.validate(8), Err(PackError::InvalidScaling));
        let desc = make_desc(163, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0)
            .with_scaling(Scaling::new(0, 1, 0));
        assert_eq!(desc.validate(8), Err(PackError::InvalidScaling));
    }

    // -----------------------------------------------------------------------
    // Physical-value helpers
    // -----------------------------------------------------------------------

    #[test]
    fn physical_identity_without_scaling() {
        let desc = make_desc(170, 3, 12, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_physical(&mut buf, &desc, -2048).unwrap();
        assert_eq!(unpack_physical(&buf, &desc), Ok(-2048));
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::I16(-2048)));
    }

    #[test]
    fn physical_with_scaling_round_trip() {
        // Temperature-style scaling: physical = raw / 2 - 40.
        let desc = make_desc(171, 7, 12, SignalType::Uint16, ByteOrder::BigEndian, 0)
            .with_scaling(Scaling::new(1, 2, -40));
        let mut buf = [0u8; 8];
        pack_physical(&mut buf, &desc, 10).unwrap();
        // raw = (10 + 40) * 2 = 100
        assert_eq!(unpack_signal(&buf, &desc), Ok(SignalValue::U16(100)));
        assert_eq!(unpack_physical(&buf, &desc), Ok(10));
    }

    #[test]
    fn physical_value_out_of_range() {
        let mut buf = [0u8; 8];
        // 8-bit unsigned raw range is 0..=255.
        let desc = make_desc(172, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        assert_eq!(
            pack_physical(&mut buf, &desc, 256),
            Err(PackError::ValueOutOfRange)
        );
        assert_eq!(
            pack_physical(&mut buf, &desc, -1),
            Err(PackError::ValueOutOfRange)
        );
        // 12-bit signed raw range is -2048..=2047.
        let desc = make_desc(173, 0, 12, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        assert_eq!(
            pack_physical(&mut buf, &desc, -2049),
            Err(PackError::ValueOutOfRange)
        );
        assert_eq!(
            pack_physical(&mut buf, &desc, 2048),
            Err(PackError::ValueOutOfRange)
        );
        assert_eq!(pack_physical(&mut buf, &desc, 2047), Ok(()));
    }

    #[test]
    fn physical_rejects_byte_arrays() {
        let desc = make_desc(
            174,
            0,
            16,
            SignalType::Uint8N(2),
            ByteOrder::LittleEndian,
            0,
        );
        let mut buf = [0u8; 8];
        assert_eq!(
            pack_physical(&mut buf, &desc, 1),
            Err(PackError::UnsupportedSignalType)
        );
        assert_eq!(
            unpack_physical(&buf, &desc),
            Err(PackError::UnsupportedSignalType)
        );
    }

    #[test]
    fn physical_boolean_range_is_zero_or_one() {
        let desc = make_desc(175, 4, 1, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_physical(&mut buf, &desc, 1).unwrap();
        assert_eq!(unpack_physical(&buf, &desc), Ok(1));
        assert_eq!(
            pack_physical(&mut buf, &desc, 2),
            Err(PackError::ValueOutOfRange)
        );
    }
}
