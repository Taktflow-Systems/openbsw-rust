// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! E2E (End-to-End) protection — simplified AUTOSAR E2E Profile 1.
//!
//! Provides CRC-8 + alive-counter stamping on transmit and verification on
//! receive.  The algorithm is a subset of AUTOSAR SWS_E2E_00323 (Profile 1)
//! adapted for direct embedding into a CAN frame payload.
//!
//! # Frame layout
//!
//! ```text
//! byte[0]  = CRC-8 over bytes [1 .. len-1]  (uses CRC-8/SAE-J1850)
//! byte[1]  = alive counter (low nibble [3:0], wrapping 0–15)
//!            upper nibble [7:4] is application data — not touched by E2E.
//! bytes[2..len-1] = application payload
//! ```
//!
//! # Example
//!
//! ```rust
//! use bsw_util::e2e::{E2eProfile, E2eProtector, E2eChecker, E2eResult};
//!
//! let profile = E2eProfile::default();
//! let mut protector = E2eProtector::new(profile);
//! let mut checker   = E2eChecker::new(E2eProfile::default());
//!
//! let mut buf = [0u8; 8];
//! buf[2] = 0xAB; // application data at byte 2+
//!
//! // TX side — stamp CRC + counter.
//! protector.protect(&mut buf, 8);
//!
//! // RX side — verify.
//! assert_eq!(checker.check(&buf, 8), E2eResult::Initial);  // first reception
//! assert_eq!(checker.check(&buf, 8), E2eResult::Repeated); // same counter
//!
//! // Increment counter by protecting again, then check.
//! protector.protect(&mut buf, 8);
//! assert_eq!(checker.check(&buf, 8), E2eResult::Ok);
//! ```
//!
//! # Algorithm notes
//!
//! - CRC covers `data[1..len]` (everything except the CRC byte itself).
//! - Alive counter occupies `data[1] & 0x0F` (low nibble of byte 1).
//! - Counter wraps at 16 (modulo-16 arithmetic).
//! - `max_delta` is the maximum acceptable counter distance in the forward
//!   direction.  A delta of 0 means the counter did not advance (repeated
//!   or replayed message).  A delta > `max_delta` means messages were lost.
//!
//! # Safety
//!
//! All operations are `no_std`-compatible and perform no heap allocation.

use crate::crc::{Crc8, CRC8_SAE_J1850};

// ---------------------------------------------------------------------------
// E2eProfile
// ---------------------------------------------------------------------------

/// Configuration for the E2E protection algorithm.
///
/// Controls which CRC variant is used, how the alive counter increments, and
/// how many skipped messages are tolerated before declaring a loss.
///
/// The `crc` field is `pub` so callers may substitute a different `Crc8`
/// algorithm (e.g. `CRC8_H2F` for AUTOSAR-strict deployments).
pub struct E2eProfile {
    /// CRC-8 algorithm used for frame protection.
    ///
    /// Default: [`CRC8_SAE_J1850`] (poly=0x1D, init=0xFF, XOR=0xFF).
    pub crc: Crc8,
    /// Amount by which the alive counter is incremented on each transmit.
    ///
    /// Typical value: 1.  Must not be 0.
    pub counter_increment: u8,
    /// Maximum acceptable forward counter delta before declaring a loss.
    ///
    /// A delta of 1 means no gaps allowed.  A value of 15 (maximum for a
    /// 4-bit counter) is permissive but still detects replays (delta=0).
    pub max_delta: u8,
}

impl E2eProfile {
    /// Create a profile with explicit CRC algorithm and counter parameters.
    pub const fn new(crc: Crc8, counter_increment: u8, max_delta: u8) -> Self {
        Self { crc, counter_increment, max_delta }
    }
}

impl Default for E2eProfile {
    /// Default profile: CRC-8/SAE-J1850, increment=1, max_delta=1.
    ///
    /// `max_delta=1` means exactly one message is expected between each
    /// reception (no gaps, no replays).
    fn default() -> Self {
        Self {
            crc: CRC8_SAE_J1850,
            counter_increment: 1,
            max_delta: 1,
        }
    }
}

// ---------------------------------------------------------------------------
// E2eResult
// ---------------------------------------------------------------------------

/// Outcome of an [`E2eChecker::check`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum E2eResult {
    /// CRC matches and counter advanced by the expected delta.
    Ok,
    /// CRC mismatch — frame may be corrupted or from a different sender.
    WrongCrc,
    /// Counter delta exceeded `max_delta` — one or more messages were lost.
    WrongCounter,
    /// Counter did not change — the same message was received again.
    Repeated,
    /// First reception; no reference counter available yet.
    ///
    /// The CRC is still verified.  If it passes, the counter is recorded as
    /// the new reference and subsequent checks compare against it.
    Initial,
}

// ---------------------------------------------------------------------------
// E2eProtector
// ---------------------------------------------------------------------------

/// Stamps CRC-8 and an alive counter onto outgoing frame payloads.
///
/// Intended for use on the transmitting side.  Each call to [`protect`]
/// advances the alive counter by `profile.counter_increment` (modulo 16)
/// and writes the CRC into `data[0]`.
///
/// # Thread safety
///
/// Not thread-safe.  Use from a single task / ISR context only.
pub struct E2eProtector {
    profile: E2eProfile,
    /// Current alive counter (0..=15).
    tx_counter: u8,
}

impl E2eProtector {
    /// Create a new protector with the given profile.
    ///
    /// The alive counter starts at 0.
    pub const fn new(profile: E2eProfile) -> Self {
        Self { profile, tx_counter: 0 }
    }

    /// Stamp CRC-8 and the alive counter onto `data[0..len]`.
    ///
    /// Layout written:
    /// - `data[0]` = CRC-8 of `data[1..len]`
    /// - `data[1] & 0x0F` = alive counter (low nibble; upper nibble preserved)
    ///
    /// Then the alive counter is incremented modulo 16 for the next call.
    ///
    /// # Panics
    ///
    /// Panics if `len < 2` (need at least one CRC byte and one counter byte)
    /// or `len > data.len()`.
    pub fn protect(&mut self, data: &mut [u8], len: usize) {
        assert!(len >= 2, "E2eProtector::protect: len must be >= 2 (CRC byte + counter byte)");
        assert!(
            len <= data.len(),
            "E2eProtector::protect: len {} > data.len() {}",
            len,
            data.len()
        );

        // Write alive counter into the low nibble of byte[1].
        data[1] = (data[1] & 0xF0) | (self.tx_counter & 0x0F);

        // Compute CRC over bytes[1..len] (everything except the CRC slot).
        let crc = self.profile.crc.checksum(&data[1..len]);
        data[0] = crc;

        // Advance counter (wraps at 16, not 256, because only 4 bits are used).
        self.tx_counter = self.tx_counter
            .wrapping_add(self.profile.counter_increment)
            & 0x0F;
    }

    /// Returns the current alive counter value (before the next protect call).
    #[inline]
    pub fn counter(&self) -> u8 {
        self.tx_counter
    }
}

// ---------------------------------------------------------------------------
// E2eChecker
// ---------------------------------------------------------------------------

/// Verifies CRC-8 and alive counter on incoming frame payloads.
///
/// Intended for use on the receiving side.  Each call to [`check`] verifies
/// the CRC and evaluates the counter delta relative to the last accepted
/// counter value.
///
/// # Thread safety
///
/// Not thread-safe.  Use from a single task context only.
pub struct E2eChecker {
    profile: E2eProfile,
    /// Last accepted counter value.  `None` on first reception.
    last_counter: Option<u8>,
}

impl E2eChecker {
    /// Create a new checker with the given profile.
    ///
    /// Initial state: no reference counter (`last_counter = None`).
    pub const fn new(profile: E2eProfile) -> Self {
        Self { profile, last_counter: None }
    }

    /// Verify the E2E protection on a received frame payload.
    ///
    /// Checks `data[0..len]` using the frame layout described in the module doc.
    ///
    /// Returns:
    /// - [`E2eResult::WrongCrc`] if the CRC does not match.
    /// - [`E2eResult::Initial`] if this is the first reception (CRC passed).
    ///   The counter is recorded as the new reference.
    /// - [`E2eResult::Repeated`] if the counter has not changed.
    /// - [`E2eResult::WrongCounter`] if the counter delta exceeds `max_delta`.
    /// - [`E2eResult::Ok`] if CRC and counter are both valid.
    ///
    /// On [`E2eResult::Ok`] or [`E2eResult::Initial`] the internal reference
    /// counter is updated.  On all other outcomes it is left unchanged.
    ///
    /// # Panics
    ///
    /// Panics if `len < 2` or `len > data.len()`.
    pub fn check(&mut self, data: &[u8], len: usize) -> E2eResult {
        assert!(len >= 2, "E2eChecker::check: len must be >= 2");
        assert!(
            len <= data.len(),
            "E2eChecker::check: len {} > data.len() {}",
            len,
            data.len()
        );

        // --- CRC check ---
        let received_crc = data[0];
        let computed_crc = self.profile.crc.checksum(&data[1..len]);
        if received_crc != computed_crc {
            return E2eResult::WrongCrc;
        }

        // --- Counter check ---
        let received_counter = data[1] & 0x0F;

        match self.last_counter {
            None => {
                // First reception — accept unconditionally (CRC already passed).
                self.last_counter = Some(received_counter);
                E2eResult::Initial
            }
            Some(last) => {
                // Compute forward distance in modulo-16 arithmetic.
                let delta = received_counter.wrapping_sub(last) & 0x0F;

                if delta == 0 {
                    // Counter did not advance — replay or retransmission.
                    E2eResult::Repeated
                } else if delta > self.profile.max_delta {
                    // Too many messages skipped.
                    E2eResult::WrongCounter
                } else {
                    // Valid advancement.
                    self.last_counter = Some(received_counter);
                    E2eResult::Ok
                }
            }
        }
    }

    /// Returns the last accepted counter value, or `None` before the first
    /// successful reception.
    #[inline]
    pub fn last_counter(&self) -> Option<u8> {
        self.last_counter
    }

    /// Reset the checker to its initial state (clears the reference counter).
    #[inline]
    pub fn reset(&mut self) {
        self.last_counter = None;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_protector() -> E2eProtector {
        E2eProtector::new(E2eProfile::default())
    }

    fn make_checker() -> E2eChecker {
        E2eChecker::new(E2eProfile::default())
    }

    // ── CRC correctness ────────────────────────────────────────────────────

    // 1 — CRC-8/SAE-J1850 check value for b"123456789" is 0x4B
    #[test]
    fn crc8_sae_j1850_check_value() {
        assert_eq!(CRC8_SAE_J1850.checksum(b"123456789"), 0x4B);
    }

    // 2 — protect then check on fresh frame passes with Initial result
    #[test]
    fn protect_then_check_initial() {
        let mut buf = [0u8; 8];
        buf[2] = 0xDE; // application data
        let mut p = make_protector();
        let mut c = make_checker();
        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Initial);
    }

    // 3 — second reception of same frame is Repeated
    #[test]
    fn same_frame_twice_is_repeated() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();
        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial
        assert_eq!(c.check(&buf, 8), E2eResult::Repeated);
    }

    // 4 — sequential protect/check produces Ok after Initial
    #[test]
    fn sequential_protect_check_ok() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();

        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Initial);

        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Ok);

        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Ok);
    }

    // 5 — CRC corruption is detected
    #[test]
    fn crc_corruption_detected() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();
        p.protect(&mut buf, 8);
        buf[0] ^= 0xFF; // flip all CRC bits
        assert_eq!(c.check(&buf, 8), E2eResult::WrongCrc);
    }

    // 6 — data byte corruption changes CRC and is caught
    #[test]
    fn data_corruption_caught_by_crc() {
        let mut buf = [0u8; 8];
        buf[3] = 0xAA;
        let mut p = make_protector();
        let mut c = make_checker();
        p.protect(&mut buf, 8);
        buf[3] ^= 0x01; // flip one bit in payload
        assert_eq!(c.check(&buf, 8), E2eResult::WrongCrc);
    }

    // 7 — counter skip beyond max_delta gives WrongCounter
    #[test]
    fn counter_gap_beyond_max_delta() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        // default max_delta = 1
        let mut c = make_checker();

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial, counter = 0

        // Advance protector twice without checking — delta will be 2.
        p.protect(&mut buf, 8); // counter = 1
        p.protect(&mut buf, 8); // counter = 2
        assert_eq!(c.check(&buf, 8), E2eResult::WrongCounter);
    }

    // 8 — permissive max_delta allows gaps
    #[test]
    fn permissive_max_delta_allows_gaps() {
        let profile = E2eProfile::new(CRC8_SAE_J1850, 1, 5);
        let mut p = E2eProtector::new(E2eProfile::new(CRC8_SAE_J1850, 1, 5));
        let mut c = E2eChecker::new(profile);
        let mut buf = [0u8; 8];

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial

        // Skip 4 messages (advance protector 5× without checking 4 of them)
        for _ in 0..4 {
            p.protect(&mut buf, 8);
        }
        p.protect(&mut buf, 8); // 5th — delta = 5 from last accepted
        assert_eq!(c.check(&buf, 8), E2eResult::Ok);
    }

    // 9 — alive counter wraps at 16
    #[test]
    fn counter_wraps_at_16() {
        let mut p = make_protector();
        let mut buf = [0u8; 8];
        // Advance 15 times → counter goes 0,1,2,...,15
        for _ in 0..15 {
            p.protect(&mut buf, 8);
        }
        assert_eq!(p.counter(), 15);
        p.protect(&mut buf, 8); // wraps to 0
        assert_eq!(p.counter(), 0);
    }

    // 10 — counter wrap-around check: 15→0 is delta=1 with max_delta=1
    #[test]
    fn counter_wrap_ok() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = E2eChecker::new(E2eProfile::new(CRC8_SAE_J1850, 1, 1));

        // Establish reference at counter=14 (15 protect+check cycles: Initial + 14× Ok).
        for _ in 0..15 {
            p.protect(&mut buf, 8);
            c.check(&buf, 8);
        }
        // last_counter == 14.  Next protect writes counter=15, delta=1 ✓
        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Ok);
        // Next protect writes counter=0 (wraps), delta from 15→0 = 1 modulo 16 ✓
        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::Ok);
    }

    // 11 — upper nibble of counter byte is preserved
    #[test]
    fn upper_nibble_preserved() {
        let mut buf = [0u8; 8];
        buf[1] = 0xF0; // upper nibble set by application
        let mut p = make_protector();
        p.protect(&mut buf, 8);
        // Upper nibble must still be 0xF.
        assert_eq!(buf[1] & 0xF0, 0xF0);
    }

    // 12 — protect is deterministic for same input
    #[test]
    fn protect_is_deterministic() {
        let original = [0xAB, 0x00, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC];
        let mut buf_a = original;
        let mut buf_b = original;

        // Reset counter via fresh protector each time.
        E2eProtector::new(E2eProfile::default()).protect(&mut buf_a, 8);
        E2eProtector::new(E2eProfile::default()).protect(&mut buf_b, 8);

        assert_eq!(buf_a, buf_b);
    }

    // 13 — checker reset clears reference counter
    #[test]
    fn checker_reset_clears_reference() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Ok

        c.reset();
        assert_eq!(c.last_counter(), None);

        // After reset the next check is Initial again.
        assert_eq!(c.check(&buf, 8), E2eResult::Initial);
    }

    // 14 — minimum frame length (len=2): only CRC + counter bytes
    #[test]
    fn minimum_frame_length() {
        let mut buf = [0u8; 2];
        let mut p = make_protector();
        let mut c = make_checker();
        p.protect(&mut buf, 2);
        assert_eq!(c.check(&buf, 2), E2eResult::Initial);
        p.protect(&mut buf, 2);
        assert_eq!(c.check(&buf, 2), E2eResult::Ok);
    }

    // 15 — last_counter accessor returns the last seen counter
    #[test]
    fn last_counter_accessor() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();
        assert_eq!(c.last_counter(), None);

        p.protect(&mut buf, 8);  // counter=0 stamped
        c.check(&buf, 8);
        assert_eq!(c.last_counter(), Some(0));

        p.protect(&mut buf, 8);  // counter=1 stamped
        c.check(&buf, 8);
        assert_eq!(c.last_counter(), Some(1));
    }

    // 16 — wrong CRC does not update reference counter
    #[test]
    fn wrong_crc_does_not_update_counter() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial, ref=0

        p.protect(&mut buf, 8); // counter=1
        let saved = c.last_counter();
        buf[0] ^= 0xFF; // corrupt CRC
        assert_eq!(c.check(&buf, 8), E2eResult::WrongCrc);
        assert_eq!(c.last_counter(), saved); // reference must not change
    }

    // 17 — wrong counter does not update reference counter
    #[test]
    fn wrong_counter_does_not_update_reference() {
        let mut buf = [0u8; 8];
        let mut p = make_protector();
        let mut c = make_checker();

        p.protect(&mut buf, 8);
        c.check(&buf, 8); // Initial, ref=0

        // Skip two frames so delta = 2 > max_delta 1.
        p.protect(&mut buf, 8);
        p.protect(&mut buf, 8);
        assert_eq!(c.check(&buf, 8), E2eResult::WrongCounter);
        // Reference must remain at 0.
        assert_eq!(c.last_counter(), Some(0));
    }
}
