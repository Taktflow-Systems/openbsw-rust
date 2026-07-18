// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Deterministic property tests for bit-level signal packing (package E32).
//!
//! The workspace deliberately has no property-testing dependency, so a
//! fixed-seed SplitMix64 PRNG generates thousands of descriptor/value
//! pairs across both byte orders, widths 1..=32, bit positions across
//! 0..512, and all CAN FD frame sizes.  Every case asserts a pack→unpack
//! round-trip and that no buffer bits outside the field are modified.

use bsw_com::{
    pack_physical, pack_signal, unpack_physical, unpack_signal, ByteOrder, PackError, Scaling,
    SignalDescriptor, SignalType, SignalValue,
};

// ---------------------------------------------------------------------------
// Deterministic PRNG (SplitMix64)
// ---------------------------------------------------------------------------

struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }
}

// ---------------------------------------------------------------------------
// Reference helpers (independent re-derivations of the packer conventions)
// ---------------------------------------------------------------------------

/// Valid CAN / CAN FD payload lengths (DLC step table).
const FRAME_LENGTHS: [usize; 15] = [1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

fn mask(width: u8) -> u32 {
    if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    }
}

fn sign_extend(raw: u32, width: u8) -> u32 {
    let m = mask(width);
    let r = raw & m;
    if width < 32 && (r >> (width - 1)) & 1 == 1 {
        r | !m
    } else {
        r
    }
}

/// DBC start bit (MSBit position) for a Motorola signal whose MSB-first
/// linear index is `linear`.
fn be_bit_position(linear: usize) -> u16 {
    ((linear / 8) * 8 + 7 - linear % 8) as u16
}

/// Reference per-byte mask of the bits a numeric descriptor's field
/// occupies — an independent re-walk of the Intel/Motorola layouts.
fn touched_bytes(descriptor: &SignalDescriptor) -> [u8; 64] {
    let mut touched = [0u8; 64];
    let width = usize::from(descriptor.bit_size);
    let bit_position = usize::from(descriptor.bit_position);
    match descriptor.byte_order {
        ByteOrder::LittleEndian => {
            for k in 0..width {
                let pos = bit_position + k;
                touched[pos / 8] |= 1 << (pos % 8);
            }
        }
        ByteOrder::BigEndian => {
            let mut byte = bit_position / 8;
            let mut shift = bit_position % 8;
            for _ in 0..width {
                touched[byte] |= 1 << shift;
                if shift == 0 {
                    byte += 1;
                    shift = 7;
                } else {
                    shift -= 1;
                }
            }
        }
    }
    touched
}

fn unsigned_types_for(width: u8) -> &'static [SignalType] {
    if width <= 8 {
        &[SignalType::Uint8, SignalType::Uint16, SignalType::Uint32]
    } else if width <= 16 {
        &[SignalType::Uint16, SignalType::Uint32]
    } else {
        &[SignalType::Uint32]
    }
}

fn signed_types_for(width: u8) -> &'static [SignalType] {
    if width <= 8 {
        &[SignalType::Sint8, SignalType::Sint16, SignalType::Sint32]
    } else if width <= 16 {
        &[SignalType::Sint16, SignalType::Sint32]
    } else {
        &[SignalType::Sint32]
    }
}

fn signed_value(signal_type: SignalType, value: i64) -> SignalValue {
    match signal_type {
        SignalType::Sint8 => SignalValue::I8(value as i8),
        SignalType::Sint16 => SignalValue::I16(value as i16),
        SignalType::Sint32 => SignalValue::I32(value as i32),
        _ => unreachable!("signed_value called with unsigned type"),
    }
}

// ---------------------------------------------------------------------------
// Property: pack→unpack round-trip, field isolation (numeric signals)
// ---------------------------------------------------------------------------

#[test]
fn property_numeric_round_trip_and_field_isolation() {
    let mut rng = SplitMix64(0xE320_0001);
    for iteration in 0..30_000u32 {
        let width = (rng.below(32) + 1) as u8;
        let len = loop {
            let candidate = FRAME_LENGTHS[rng.below(FRAME_LENGTHS.len() as u64) as usize];
            if candidate * 8 >= usize::from(width) {
                break candidate;
            }
        };
        let linear = rng.below((len * 8 - usize::from(width) + 1) as u64) as usize;
        let byte_order = if rng.next() & 1 == 0 {
            ByteOrder::LittleEndian
        } else {
            ByteOrder::BigEndian
        };
        let bit_position = match byte_order {
            ByteOrder::LittleEndian => linear as u16,
            ByteOrder::BigEndian => be_bit_position(linear),
        };
        let raw_bits = (rng.next() as u32) & mask(width);
        let (signal_type, value) = match rng.below(8) {
            0 => (SignalType::Boolean, SignalValue::Bool(raw_bits & 1 == 1)),
            1..=4 => {
                let types = unsigned_types_for(width);
                let t = types[rng.below(types.len() as u64) as usize];
                (t, SignalValue::from_u32(raw_bits, t))
            }
            _ => {
                let types = signed_types_for(width);
                let t = types[rng.below(types.len() as u64) as usize];
                (t, SignalValue::from_u32(sign_extend(raw_bits, width), t))
            }
        };
        let descriptor = SignalDescriptor::new(1, bit_position, width, signal_type, byte_order, 0);
        assert_eq!(descriptor.validate(len), Ok(()), "iteration {iteration}");

        let mut frame = [0u8; 64];
        for byte in frame.iter_mut().take(len) {
            *byte = rng.next() as u8;
        }
        let original = frame;

        pack_signal(&mut frame[..len], &descriptor, value)
            .unwrap_or_else(|e| panic!("iteration {iteration}: pack failed {e:?}"));
        let unpacked = unpack_signal(&frame[..len], &descriptor)
            .unwrap_or_else(|e| panic!("iteration {iteration}: unpack failed {e:?}"));
        assert_eq!(
            unpacked, value,
            "iteration {iteration}: round-trip mismatch for {descriptor:?}"
        );

        let touched = touched_bytes(&descriptor);
        for i in 0..len {
            assert_eq!(
                frame[i] & !touched[i],
                original[i] & !touched[i],
                "iteration {iteration}: untouched bits modified at byte {i} ({descriptor:?})"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Property: byte arrays (Uint8N) round-trip and neighbor isolation
// ---------------------------------------------------------------------------

#[test]
fn property_byte_array_round_trip() {
    let mut rng = SplitMix64(0xE320_0002);
    for iteration in 0..2_000u32 {
        let n = (rng.below(4) + 1) as usize;
        let len = loop {
            let candidate = FRAME_LENGTHS[rng.below(FRAME_LENGTHS.len() as u64) as usize];
            if candidate >= n {
                break candidate;
            }
        };
        let start_byte = rng.below((len - n + 1) as u64) as usize;
        let byte_order = if rng.next() & 1 == 0 {
            ByteOrder::LittleEndian
        } else {
            ByteOrder::BigEndian
        };
        let raw = (rng.next() as u32)
            & if n >= 4 {
                u32::MAX
            } else {
                (1u32 << (8 * n)) - 1
            };
        let descriptor = SignalDescriptor::new(
            2,
            (start_byte * 8) as u16,
            (8 * n) as u8,
            SignalType::Uint8N(n),
            byte_order,
            0,
        );
        assert_eq!(descriptor.validate(len), Ok(()), "iteration {iteration}");

        let mut frame = [0u8; 64];
        for byte in frame.iter_mut().take(len) {
            *byte = rng.next() as u8;
        }
        let original = frame;

        pack_signal(&mut frame[..len], &descriptor, SignalValue::U32(raw)).unwrap();
        assert_eq!(
            unpack_signal(&frame[..len], &descriptor),
            Ok(SignalValue::U32(raw)),
            "iteration {iteration}"
        );
        for i in (0..len).filter(|i| *i < start_byte || *i >= start_byte + n) {
            assert_eq!(
                frame[i], original[i],
                "iteration {iteration}: neighbor byte {i} modified"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Property: arbitrary (mostly invalid) descriptors never panic; validate
// is sound w.r.t. pack
// ---------------------------------------------------------------------------

#[test]
fn property_arbitrary_descriptors_no_panic_validate_sound() {
    let mut rng = SplitMix64(0xE320_0003);
    for iteration in 0..30_000u32 {
        let bit_position = (rng.next() & 0x03FF) as u16; // 0..=1023
        let bit_size = rng.below(40) as u8; // 0..=39, including invalid
        let byte_order = if rng.next() & 1 == 0 {
            ByteOrder::LittleEndian
        } else {
            ByteOrder::BigEndian
        };
        let signal_type = match rng.below(8) {
            0 => SignalType::Boolean,
            1 => SignalType::Uint8,
            2 => SignalType::Uint16,
            3 => SignalType::Uint32,
            4 => SignalType::Sint8,
            5 => SignalType::Sint16,
            6 => SignalType::Sint32,
            _ => SignalType::Uint8N(rng.below(7) as usize),
        };
        let len = rng.below(70) as usize; // 0..=69, including invalid > 64
        let descriptor =
            SignalDescriptor::new(3, bit_position, bit_size, signal_type, byte_order, 0);
        let value = SignalValue::from_u32(rng.next() as u32, signal_type);
        let validity = descriptor.validate(len);

        if len <= 64 {
            let mut frame = [0u8; 64];
            let packed = pack_signal(&mut frame[..len], &descriptor, value);
            if validity.is_ok() {
                assert!(
                    packed.is_ok(),
                    "iteration {iteration}: validate({len}) passed but pack failed \
                     ({descriptor:?}, {:?})",
                    packed.unwrap_err()
                );
            }
            match packed {
                Ok(()) => {
                    unpack_signal(&frame[..len], &descriptor)
                        .unwrap_or_else(|e| panic!("iteration {iteration}: unpack failed {e:?}"));
                }
                Err(_) => {
                    assert!(unpack_signal(&frame[..len], &descriptor).is_err());
                }
            }
        } else {
            assert_eq!(validity, Err(PackError::InvalidPduLength));
            let mut frame = [0u8; 64];
            let _ = pack_signal(&mut frame, &descriptor, value);
            let _ = unpack_signal(&frame, &descriptor);
        }
    }
}

// ---------------------------------------------------------------------------
// Exhaustive boundaries: every width 1..=32 at 0/1/max and signed min/max
// ---------------------------------------------------------------------------

#[test]
fn exhaustive_boundaries_every_width() {
    for width in 1u8..=32 {
        let positions = [0usize, 3, 29, 512 - usize::from(width)];
        for byte_order in [ByteOrder::LittleEndian, ByteOrder::BigEndian] {
            for &linear in &positions {
                let bit_position = match byte_order {
                    ByteOrder::LittleEndian => linear as u16,
                    ByteOrder::BigEndian => be_bit_position(linear),
                };
                // Unsigned: 0, 1, max — in every carrier type that fits.
                for &signal_type in unsigned_types_for(width) {
                    for raw in [0u32, 1, mask(width)] {
                        let descriptor = SignalDescriptor::new(
                            4,
                            bit_position,
                            width,
                            signal_type,
                            byte_order,
                            0,
                        );
                        let value = SignalValue::from_u32(raw, signal_type);
                        let mut frame = [0xA5u8; 64];
                        pack_signal(&mut frame, &descriptor, value).unwrap();
                        assert_eq!(
                            unpack_signal(&frame, &descriptor),
                            Ok(value),
                            "width {width} linear {linear} raw {raw:#x} {byte_order:?}"
                        );
                    }
                }
                // Signed: min, -1, 0, max — e.g. width 12 covers -2048/2047.
                let half = 1i64 << (width - 1);
                for &signal_type in signed_types_for(width) {
                    for value_i in [-half, -1, 0, half - 1] {
                        let descriptor = SignalDescriptor::new(
                            5,
                            bit_position,
                            width,
                            signal_type,
                            byte_order,
                            0,
                        );
                        let value = signed_value(signal_type, value_i);
                        let mut frame = [0x5Au8; 64];
                        pack_signal(&mut frame, &descriptor, value).unwrap();
                        assert_eq!(
                            unpack_signal(&frame, &descriptor),
                            Ok(value),
                            "width {width} linear {linear} value {value_i} {byte_order:?}"
                        );
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Property: physical round-trip through integer scaling (exact family)
// ---------------------------------------------------------------------------

#[test]
fn property_scaling_physical_round_trip() {
    let mut rng = SplitMix64(0xE320_0004);
    for iteration in 0..5_000u32 {
        let width = (rng.below(31) + 2) as u8; // 2..=32
        let factor_num = loop {
            let v = (rng.next() as i32) % 1_000;
            if v != 0 {
                break v;
            }
        };
        let offset = (rng.next() as i32) % 10_000;
        let scaling = Scaling::new(factor_num, 1, offset);
        let half = 1i64 << (width - 1);
        let raw = (rng.next() as i64) % half; // representable in `width` bits
        let physical = raw * i64::from(factor_num) + i64::from(offset);

        let descriptor =
            SignalDescriptor::new(6, 0, width, SignalType::Sint32, ByteOrder::LittleEndian, 0)
                .with_scaling(scaling);
        assert_eq!(descriptor.validate(8), Ok(()));

        let mut frame = [0u8; 8];
        pack_physical(&mut frame, &descriptor, physical)
            .unwrap_or_else(|e| panic!("iteration {iteration}: pack_physical failed {e:?}"));
        assert_eq!(
            unpack_signal(&frame, &descriptor),
            Ok(SignalValue::I32(raw as i32)),
            "iteration {iteration}: raw on the wire"
        );
        assert_eq!(
            unpack_physical(&frame, &descriptor),
            Ok(physical),
            "iteration {iteration}: physical round-trip"
        );
    }
}
