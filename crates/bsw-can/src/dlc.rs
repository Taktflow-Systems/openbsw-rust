//! CAN / CAN FD DLC and payload-length mapping (package D14).
//!
//! The wire DLC field is four bits; CAN FD reuses codes 9..=15 for the
//! stepped payload sizes 12..=64. All mappings are total and checked — no
//! panic on malformed hardware values.

/// Maximum classic CAN payload length.
pub const CLASSIC_MAX_LENGTH: usize = 8;

/// Maximum CAN FD payload length.
pub const FD_MAX_LENGTH: usize = 64;

/// Payload length encoded by `dlc`.
///
/// In classic mode, DLC codes 9..=15 are transmitted on the bus but always
/// mean 8 data bytes; they are reported as 8, mirroring hardware behavior.
/// Codes above 15 are invalid in either mode.
pub const fn dlc_to_length(dlc: u8, fd: bool) -> Option<usize> {
    if dlc > 15 {
        return None;
    }
    if dlc <= 8 {
        return Some(dlc as usize);
    }
    if !fd {
        return Some(CLASSIC_MAX_LENGTH);
    }
    Some(match dlc {
        9 => 12,
        10 => 16,
        11 => 20,
        12 => 24,
        13 => 32,
        14 => 48,
        _ => 64,
    })
}

/// DLC encoding exactly matching `length`, if one exists.
pub const fn length_to_dlc(length: usize, fd: bool) -> Option<u8> {
    if length <= 8 {
        #[allow(clippy::cast_possible_truncation)]
        return Some(length as u8);
    }
    if !fd {
        return None;
    }
    Some(match length {
        12 => 9,
        16 => 10,
        20 => 11,
        24 => 12,
        32 => 13,
        48 => 14,
        64 => 15,
        _ => return None,
    })
}

/// Smallest transmittable payload length that can hold `length` bytes.
///
/// CAN FD payloads between the stepped sizes are padded up by the sender;
/// classic payloads are never padded.
pub const fn padded_length(length: usize, fd: bool) -> Option<usize> {
    if length <= 8 {
        return Some(length);
    }
    if !fd {
        return None;
    }
    Some(match length {
        9..=12 => 12,
        13..=16 => 16,
        17..=20 => 20,
        21..=24 => 24,
        25..=32 => 32,
        33..=48 => 48,
        49..=64 => 64,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_dlc_roundtrips_in_fd_mode() {
        for dlc in 0..=15u8 {
            let length = dlc_to_length(dlc, true).unwrap();
            assert_eq!(length_to_dlc(length, true), Some(dlc));
        }
        assert_eq!(dlc_to_length(16, true), None);
        assert_eq!(dlc_to_length(16, false), None);
    }

    #[test]
    fn classic_mode_caps_high_dlc_codes_at_eight() {
        for dlc in 9..=15u8 {
            assert_eq!(dlc_to_length(dlc, false), Some(8));
        }
        assert_eq!(length_to_dlc(12, false), None);
    }

    #[test]
    fn fd_lengths_between_steps_have_no_exact_dlc() {
        assert_eq!(length_to_dlc(13, true), None);
        assert_eq!(length_to_dlc(65, true), None);
    }

    #[test]
    fn padding_rounds_up_to_the_next_step() {
        assert_eq!(padded_length(5, false), Some(5));
        assert_eq!(padded_length(9, false), None);
        assert_eq!(padded_length(9, true), Some(12));
        assert_eq!(padded_length(33, true), Some(48));
        assert_eq!(padded_length(64, true), Some(64));
        assert_eq!(padded_length(65, true), None);
    }
}
