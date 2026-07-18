#![no_main]

//! Fuzz target for bsw-com bit-level signal packing (package E32).
//!
//! Exercises arbitrary bit positions, widths, signedness, byte orders and
//! frame sizes up to 64 bytes (CAN FD).  Invariants:
//!
//! - no call may panic, whatever the descriptor;
//! - if `validate(frame_len)` accepts the descriptor, `pack_signal` into a
//!   `frame_len`-byte buffer must succeed;
//! - whenever packing succeeds, unpacking returns the value truncated to
//!   the field width (two's-complement for signed types).

use bsw_com::packer::{pack_signal, unpack_signal};
use bsw_com::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
use libfuzzer_sys::fuzz_target;

/// Valid CAN / CAN FD payload lengths (DLC step table).
const FRAME_LENGTHS: [usize; 15] = [1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

fn mask(width: u8) -> u32 {
    if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    }
}

/// Expected value after the documented truncation to `bit_size` bits
/// (two's-complement for signed types) — mirrors the packer semantics.
fn expected(raw: u32, descriptor: &SignalDescriptor) -> SignalValue {
    let width = descriptor.bit_size;
    match descriptor.signal_type {
        SignalType::Boolean => SignalValue::Bool(raw & mask(width) != 0),
        SignalType::Uint8N(n) => {
            let m = if n >= 4 {
                u32::MAX
            } else {
                (1u32 << (8 * n)) - 1
            };
            SignalValue::U32(raw & m)
        }
        t @ (SignalType::Sint8 | SignalType::Sint16 | SignalType::Sint32) => {
            let m = mask(width);
            let r = raw & m;
            let s = if width < 32 && (r >> (width - 1)) & 1 == 1 {
                r | !m
            } else {
                r
            };
            SignalValue::from_u32(s, t)
        }
        t => SignalValue::from_u32(raw & mask(width), t),
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 11 {
        return;
    }
    let frame_len = FRAME_LENGTHS[usize::from(data[0]) % FRAME_LENGTHS.len()];
    let byte_order = if data[1] & 1 == 0 {
        ByteOrder::LittleEndian
    } else {
        ByteOrder::BigEndian
    };
    let signal_type = match data[2] % 8 {
        0 => SignalType::Boolean,
        1 => SignalType::Uint8,
        2 => SignalType::Uint16,
        3 => SignalType::Uint32,
        4 => SignalType::Sint8,
        5 => SignalType::Sint16,
        6 => SignalType::Sint32,
        // n in 0..=5: covers ZeroWidth and ByteArrayTooLong as well.
        _ => SignalType::Uint8N(usize::from(data[3] % 6)),
    };
    // Arbitrary width (0..=255) and bit position (0..=519, beyond the
    // 512-bit FD maximum): the packer must reject invalid combinations
    // with a typed error, never panic.
    let bit_size = data[4];
    let bit_position = u16::from_le_bytes([data[5], data[6]]) % 520;
    let raw = u32::from_le_bytes(data[7..11].try_into().unwrap());

    let descriptor = SignalDescriptor::new(0, bit_position, bit_size, signal_type, byte_order, 0);
    let value = SignalValue::from_u32(raw, signal_type);
    let valid = descriptor.validate(frame_len).is_ok();

    let mut buffer = [0u8; 64];
    let frame = &mut buffer[..frame_len];
    match pack_signal(frame, &descriptor, value) {
        Ok(()) => {
            let unpacked = unpack_signal(frame, &descriptor).expect("unpack after successful pack");
            assert_eq!(unpacked, expected(raw, &descriptor));
        }
        Err(_) => assert!(!valid, "descriptor validated Ok but pack failed"),
    }
});
