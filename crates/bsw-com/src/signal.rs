// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Signal types — AUTOSAR `ComSignalType` mapping and runtime signal values.
//!
//! A [`SignalDescriptor`] is a compile-time-constant struct that describes
//! where a signal lives inside a PDU (bit offset, width, byte order) and,
//! optionally, how its raw value maps to a physical value ([`Scaling`]).
//! A [`SignalValue`] is the runtime value that flows between the application
//! and the COM layer at run-time.

use crate::packer::PackError;

// ---------------------------------------------------------------------------
// SignalType
// ---------------------------------------------------------------------------

/// Signal data type — matches AUTOSAR `ComSignalType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalType {
    /// 1-bit boolean.
    Boolean,
    /// 8-bit unsigned integer.
    Uint8,
    /// 16-bit unsigned integer.
    Uint16,
    /// 32-bit unsigned integer.
    Uint32,
    /// 8-bit signed integer.
    Sint8,
    /// 16-bit signed integer.
    Sint16,
    /// 32-bit signed integer.
    Sint32,
    /// Byte array of `N` bytes (AUTOSAR `UINT8_N`).
    Uint8N(usize),
}

// ---------------------------------------------------------------------------
// ByteOrder
// ---------------------------------------------------------------------------

/// Signal byte order inside a PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteOrder {
    /// Intel / little-endian: LSBit at `bit_position`.
    LittleEndian,
    /// Motorola / big-endian: MSBit at `bit_position` (DBC convention).
    BigEndian,
}

// ---------------------------------------------------------------------------
// TransferProperty
// ---------------------------------------------------------------------------

/// Per-signal transfer property (AUTOSAR `ComTransferProperty`-style).
///
/// Controls whether writing the signal requests an event transmission of its
/// PDU.  Only effective when the owning TX PDU's
/// [`TxMode`](crate::pdu::TxMode) accepts events (`EventOnly` / `Mixed`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferProperty {
    /// Writing the signal only updates the shadow buffer; transmission
    /// happens on the cyclic grid or an explicit
    /// [`trigger`](crate::com::ComManager::trigger).
    Pending,
    /// Writing the signal additionally requests an event transmission of
    /// its PDU on the next tick.
    Triggered,
}

// ---------------------------------------------------------------------------
// Scaling
// ---------------------------------------------------------------------------

/// Integer-only linear scaling metadata (project extension).
///
/// Maps a raw wire value to a physical value without floating point:
///
/// ```text
/// physical = round(raw * factor_num / factor_den) + offset
/// raw      = round((physical - offset) * factor_den / factor_num)
/// ```
///
/// All intermediate math is checked `i64`; division rounds to the nearest
/// integer with ties away from zero.  Both factors must be non-zero so the
/// mapping is invertible.
///
/// Upstream Eclipse OpenBSW has no general COM stack; this type belongs to
/// the recorded `extension.com` project extension and follows DBC-style
/// factor/offset semantics restricted to rational integer factors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Scaling {
    /// Numerator of the scale factor.  Must be non-zero.
    pub factor_num: i32,
    /// Denominator of the scale factor.  Must be non-zero.
    pub factor_den: i32,
    /// Physical-unit offset added after scaling.
    pub offset: i32,
}

impl Scaling {
    /// Create a new scaling descriptor.
    #[must_use]
    pub const fn new(factor_num: i32, factor_den: i32, offset: i32) -> Self {
        Self {
            factor_num,
            factor_den,
            offset,
        }
    }

    /// Check the scaling parameters.
    ///
    /// # Errors
    ///
    /// [`PackError::InvalidScaling`] if either factor is zero.
    pub const fn validate(&self) -> Result<(), PackError> {
        if self.factor_num == 0 || self.factor_den == 0 {
            return Err(PackError::InvalidScaling);
        }
        Ok(())
    }

    /// Convert a raw wire value to its physical representation:
    /// `physical = round(raw * factor_num / factor_den) + offset`.
    ///
    /// Division rounds to the nearest integer, ties away from zero.
    ///
    /// # Errors
    ///
    /// - [`PackError::InvalidScaling`] if either factor is zero.
    /// - [`PackError::ScalingOverflow`] if an intermediate overflows `i64`.
    pub fn raw_to_physical(&self, raw: i64) -> Result<i64, PackError> {
        self.validate()?;
        let scaled = raw
            .checked_mul(i64::from(self.factor_num))
            .ok_or(PackError::ScalingOverflow)?;
        let quotient = div_round_nearest(scaled, i64::from(self.factor_den))
            .ok_or(PackError::ScalingOverflow)?;
        quotient
            .checked_add(i64::from(self.offset))
            .ok_or(PackError::ScalingOverflow)
    }

    /// Convert a physical value back to its raw wire representation:
    /// `raw = round((physical - offset) * factor_den / factor_num)`.
    ///
    /// Division rounds to the nearest integer, ties away from zero.
    ///
    /// # Errors
    ///
    /// - [`PackError::InvalidScaling`] if either factor is zero.
    /// - [`PackError::ScalingOverflow`] if an intermediate overflows `i64`.
    pub fn physical_to_raw(&self, physical: i64) -> Result<i64, PackError> {
        self.validate()?;
        let shifted = physical
            .checked_sub(i64::from(self.offset))
            .ok_or(PackError::ScalingOverflow)?;
        let scaled = shifted
            .checked_mul(i64::from(self.factor_den))
            .ok_or(PackError::ScalingOverflow)?;
        div_round_nearest(scaled, i64::from(self.factor_num)).ok_or(PackError::ScalingOverflow)
    }
}

/// Signed division rounding to the nearest integer, ties away from zero.
///
/// Returns `None` for division by zero and for the `i64::MIN / -1`
/// overflow case.
fn div_round_nearest(numerator: i64, denominator: i64) -> Option<i64> {
    let quotient = numerator.checked_div(denominator)?;
    let remainder = numerator.checked_rem(denominator)?;
    if remainder == 0 {
        return Some(quotient);
    }
    if remainder.unsigned_abs() * 2 >= denominator.unsigned_abs() {
        if (numerator < 0) == (denominator < 0) {
            quotient.checked_add(1)
        } else {
            quotient.checked_sub(1)
        }
    } else {
        Some(quotient)
    }
}

// ---------------------------------------------------------------------------
// SignalDescriptor
// ---------------------------------------------------------------------------

/// Static signal descriptor — defines where a signal lives in a PDU.
///
/// All fields are `pub` to allow `const` table initialisation in application
/// code; [`SignalDescriptor::new`] is the backward-compatible `const`
/// constructor (no scaling), [`SignalDescriptor::with_scaling`] attaches
/// scaling metadata builder-style.
#[derive(Debug, Clone, Copy)]
pub struct SignalDescriptor {
    /// Signal ID — unique within the system.
    pub id: u16,
    /// Bit position in the PDU (LSBit for LE, MSBit for BE — see the
    /// [`crate::packer`] module docs for the numbering convention).
    pub bit_position: u16,
    /// Width of the signal in bits.
    pub bit_size: u8,
    /// Data type.
    pub signal_type: SignalType,
    /// Byte order.
    pub byte_order: ByteOrder,
    /// Initial / default value packed as `u32`.
    pub init_value: u32,
    /// Optional integer linear scaling (project extension).  `None` keeps
    /// the raw packing path unchanged.
    pub scaling: Option<Scaling>,
    /// Transfer property — whether writing this signal triggers an event
    /// transmission of its PDU (package E33).  Defaults to
    /// [`TransferProperty::Pending`].
    pub transfer: TransferProperty,
}

impl SignalDescriptor {
    /// Create a descriptor without scaling metadata and with the
    /// [`TransferProperty::Pending`] transfer property.
    #[must_use]
    pub const fn new(
        id: u16,
        bit_position: u16,
        bit_size: u8,
        signal_type: SignalType,
        byte_order: ByteOrder,
        init_value: u32,
    ) -> Self {
        Self {
            id,
            bit_position,
            bit_size,
            signal_type,
            byte_order,
            init_value,
            scaling: None,
            transfer: TransferProperty::Pending,
        }
    }

    /// Attach linear scaling metadata (builder style).
    #[must_use]
    pub const fn with_scaling(mut self, scaling: Scaling) -> Self {
        self.scaling = Some(scaling);
        self
    }

    /// Set the transfer property (builder style).
    #[must_use]
    pub const fn with_transfer_property(mut self, transfer: TransferProperty) -> Self {
        self.transfer = transfer;
        self
    }
}

// ---------------------------------------------------------------------------
// SignalValue
// ---------------------------------------------------------------------------

/// Runtime signal value — typed union of all supported primitive types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SignalValue {
    /// Boolean signal.
    Bool(bool),
    /// 8-bit unsigned.
    U8(u8),
    /// 16-bit unsigned.
    U16(u16),
    /// 32-bit unsigned.
    U32(u32),
    /// 8-bit signed.
    I8(i8),
    /// 16-bit signed.
    I16(i16),
    /// 32-bit signed.
    I32(i32),
}

impl SignalValue {
    /// Losslessly encode any variant as a `u32` bit pattern.
    ///
    /// Signed values are zero-extended after casting to the corresponding
    /// unsigned type (same as how they are packed into the PDU buffer).
    #[must_use]
    pub fn as_u32(&self) -> u32 {
        match *self {
            Self::Bool(b) => u32::from(b),
            Self::U8(v) => u32::from(v),
            Self::U16(v) => u32::from(v),
            Self::U32(v) => v,
            Self::I8(v) => u32::from(v as u8),
            Self::I16(v) => u32::from(v as u16),
            Self::I32(v) => v as u32,
        }
    }

    /// Reconstruct a [`SignalValue`] from a `u32` bit pattern, guided by
    /// `sig_type`.
    ///
    /// For `Uint8N` arrays this returns `U32` with the raw bytes — callers
    /// that need the full array should handle the byte-array path separately.
    #[must_use]
    pub fn from_u32(val: u32, sig_type: SignalType) -> Self {
        match sig_type {
            SignalType::Boolean => Self::Bool(val != 0),
            SignalType::Uint8 => Self::U8(val as u8),
            SignalType::Uint16 => Self::U16(val as u16),
            SignalType::Uint32 => Self::U32(val),
            SignalType::Sint8 => Self::I8(val as i8),
            SignalType::Sint16 => Self::I16(val as i16),
            SignalType::Sint32 => Self::I32(val as i32),
            // For byte arrays, return the raw u32 — packer handles full copy.
            SignalType::Uint8N(_) => Self::U32(val),
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{Scaling, SignalType, SignalValue};
    use crate::packer::PackError;

    #[test]
    fn bool_round_trip() {
        assert_eq!(SignalValue::Bool(true).as_u32(), 1);
        assert_eq!(SignalValue::Bool(false).as_u32(), 0);
        assert_eq!(
            SignalValue::from_u32(1, SignalType::Boolean),
            SignalValue::Bool(true)
        );
        assert_eq!(
            SignalValue::from_u32(0, SignalType::Boolean),
            SignalValue::Bool(false)
        );
    }

    #[test]
    fn u8_round_trip() {
        assert_eq!(SignalValue::U8(0xAB).as_u32(), 0xAB);
        assert_eq!(
            SignalValue::from_u32(0xAB, SignalType::Uint8),
            SignalValue::U8(0xAB)
        );
    }

    #[test]
    fn u16_round_trip() {
        assert_eq!(SignalValue::U16(0xBEEF).as_u32(), 0xBEEF);
        assert_eq!(
            SignalValue::from_u32(0xBEEF, SignalType::Uint16),
            SignalValue::U16(0xBEEF)
        );
    }

    #[test]
    fn u32_round_trip() {
        assert_eq!(SignalValue::U32(0xDEAD_BEEF).as_u32(), 0xDEAD_BEEF);
        assert_eq!(
            SignalValue::from_u32(0xDEAD_BEEF, SignalType::Uint32),
            SignalValue::U32(0xDEAD_BEEF)
        );
    }

    #[test]
    fn i8_negative_round_trip() {
        let v = SignalValue::I8(-1_i8);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFF);
        assert_eq!(
            SignalValue::from_u32(raw, SignalType::Sint8),
            SignalValue::I8(-1)
        );
    }

    #[test]
    fn i16_negative_round_trip() {
        let v = SignalValue::I16(-256_i16);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFF00);
        assert_eq!(
            SignalValue::from_u32(raw, SignalType::Sint16),
            SignalValue::I16(-256)
        );
    }

    #[test]
    fn i32_negative_round_trip() {
        let v = SignalValue::I32(-1_i32);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFFFF_FFFF);
        assert_eq!(
            SignalValue::from_u32(raw, SignalType::Sint32),
            SignalValue::I32(-1)
        );
    }

    // -----------------------------------------------------------------------
    // Scaling
    // -----------------------------------------------------------------------

    #[test]
    fn scaling_round_trip_exact() {
        // physical = raw / 2 - 40 (temperature-style).
        let s = Scaling::new(1, 2, -40);
        assert_eq!(s.raw_to_physical(100), Ok(10));
        assert_eq!(s.physical_to_raw(10), Ok(100));
        // Negative factor.
        let s = Scaling::new(-3, 1, 5);
        assert_eq!(s.raw_to_physical(4), Ok(-7));
        assert_eq!(s.physical_to_raw(-7), Ok(4));
    }

    #[test]
    fn scaling_division_rounds_to_nearest_ties_away_from_zero() {
        let half = Scaling::new(1, 2, 0);
        assert_eq!(half.raw_to_physical(3), Ok(2)); // 1.5 -> 2
        assert_eq!(half.raw_to_physical(-3), Ok(-2)); // -1.5 -> -2
        let third = Scaling::new(1, 3, 0);
        assert_eq!(third.raw_to_physical(1), Ok(0)); // 0.33 -> 0
        assert_eq!(third.raw_to_physical(2), Ok(1)); // 0.67 -> 1
        assert_eq!(third.raw_to_physical(-1), Ok(0)); // -0.33 -> 0
        assert_eq!(third.raw_to_physical(-2), Ok(-1)); // -0.67 -> -1
                                                       // Inverse direction divides by factor_num.
        let by_seven = Scaling::new(7, 1, 0);
        assert_eq!(by_seven.physical_to_raw(10), Ok(1)); // 1.43 -> 1
        assert_eq!(by_seven.physical_to_raw(11), Ok(2)); // 1.57 -> 2
    }

    #[test]
    fn scaling_zero_factors_rejected() {
        assert_eq!(
            Scaling::new(0, 1, 0).raw_to_physical(1),
            Err(PackError::InvalidScaling)
        );
        assert_eq!(
            Scaling::new(1, 0, 0).raw_to_physical(1),
            Err(PackError::InvalidScaling)
        );
        assert_eq!(
            Scaling::new(1, 0, 0).physical_to_raw(1),
            Err(PackError::InvalidScaling)
        );
        assert_eq!(Scaling::new(1, 1, 0).validate(), Ok(()));
    }

    #[test]
    fn scaling_overflow_rejected() {
        let s = Scaling::new(i32::MAX, 1, 0);
        assert_eq!(s.raw_to_physical(i64::MAX), Err(PackError::ScalingOverflow));
        let s = Scaling::new(1, 1, -1);
        assert_eq!(s.physical_to_raw(i64::MAX), Err(PackError::ScalingOverflow));
        let s = Scaling::new(1, i32::MAX, 0);
        assert_eq!(
            s.physical_to_raw(i64::MAX / 2),
            Err(PackError::ScalingOverflow)
        );
    }
}
