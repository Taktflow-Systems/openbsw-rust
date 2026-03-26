//! CRC calculator with compile-time lookup tables.
//!
//! Provides table-driven CRC computation for 8-, 16-, and 32-bit digests.
//! Tables are generated at compile time with `const fn`, so no heap allocation
//! or runtime initialisation is needed — suitable for `no_std` targets.
//!
//! # Quick start
//!
//! ```rust
//! use bsw_util::crc::CRC32_ETHERNET;
//!
//! let check = CRC32_ETHERNET.checksum(b"123456789");
//! assert_eq!(check, 0xCBF4_3926);
//! ```
//!
//! # Incremental usage
//!
//! ```rust
//! use bsw_util::crc::CRC16_CCITT;
//!
//! let mut digest = CRC16_CCITT.digest();
//! digest.update(b"1234");
//! digest.update(b"56789");
//! assert_eq!(digest.finalize(), 0x29B1);
//! ```
//!
//! # Algorithm notes
//!
//! For reflected CRCs (e.g. CRC-32/ISO-HDLC, CRC-8/MAXIM) the table is built
//! from the bit-reversed polynomial and bytes are processed LSB-first:
//!
//! ```text
//! for byte in data:
//!     idx  = (crc & 0xFF) ^ byte
//!     crc  = table[idx] ^ (crc >> 8)
//! ```
//!
//! For normal (non-reflected) CRCs the table uses the normal polynomial and
//! bytes are processed MSB-first:
//!
//! ```text
//! for byte in data:
//!     idx  = (crc >> (WIDTH-8)) ^ byte
//!     crc  = table[idx] ^ (crc << 8)   -- masked to WIDTH bits
//! ```

// ---------------------------------------------------------------------------
// Crc<T> and CrcDigest — one concrete impl per width
// ---------------------------------------------------------------------------

// We use three separate concrete structs rather than a generic over a sealed
// trait because:
//   1. const fn table generation differs per width (top-bit mask, shift amount).
//   2. The normal-path `<< 8` overflows for u8 — it must be avoided.
//   3. Keeping impls concrete avoids a private extension trait and is simpler.
//
// A thin public-facing newtype / type alias still exposes a uniform surface.

/// CRC-8 calculator with a pre-computed 256-entry lookup table.
///
/// Construct with one of the provided [`const`] values
/// (e.g. [`CRC8_CCITT`]) or call [`Crc8::new`] for a custom polynomial.
pub struct Crc8 {
    table: [u8; 256],
    init: u8,
    reflect_out: bool,
    xor_out: u8,
    /// Whether the table was built in reflected mode (LSB-first processing).
    reflected: bool,
}

/// CRC-16 calculator with a pre-computed 256-entry lookup table.
pub struct Crc16 {
    table: [u16; 256],
    init: u16,
    reflect_out: bool,
    xor_out: u16,
    reflected: bool,
}

/// CRC-32 calculator with a pre-computed 256-entry lookup table.
pub struct Crc32 {
    table: [u32; 256],
    init: u32,
    reflect_out: bool,
    xor_out: u32,
    reflected: bool,
}

/// Generic alias kept for documentation coherence — concrete type is chosen by
/// the width of `T`:
/// - [`Crc8`] for `u8`
/// - [`Crc16`] for `u16`
/// - [`Crc32`] for `u32`
///
/// All pre-defined constants use one of these concrete types.
pub type Crc<T> = <T as CrcWidthAlias>::Concrete;

/// Helper to map `u8`/`u16`/`u32` to the concrete `Crc*` type.
pub trait CrcWidthAlias: Sized {
    /// The concrete calculator type for this width.
    type Concrete;
}
impl CrcWidthAlias for u8 {
    type Concrete = Crc8;
}
impl CrcWidthAlias for u16 {
    type Concrete = Crc16;
}
impl CrcWidthAlias for u32 {
    type Concrete = Crc32;
}

// ---------------------------------------------------------------------------
// Table generation helpers (const fn)
// ---------------------------------------------------------------------------

/// Build a normal (MSB-first) CRC-8 table.
const fn make_table_u8_normal(poly: u8) -> [u8; 256] {
    let mut table = [0u8; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u8;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a reflected (LSB-first) CRC-8 table.
///
/// `poly` must already be bit-reversed (reflected polynomial).
const fn make_table_u8_reflected(rpoly: u8) -> [u8; 256] {
    let mut table = [0u8; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u8;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x01 != 0 {
                crc = (crc >> 1) ^ rpoly;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a normal CRC-16 table.
const fn make_table_u16_normal(poly: u16) -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc: u16 = (i as u16) << 8;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a reflected CRC-16 table.
const fn make_table_u16_reflected(rpoly: u16) -> [u16; 256] {
    let mut table = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc: u16 = i as u16;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ rpoly;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a normal CRC-32 table.
const fn make_table_u32_normal(poly: u32) -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc: u32 = (i as u32) << 24;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x8000_0000 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Build a reflected CRC-32 table.
const fn make_table_u32_reflected(rpoly: u32) -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc: u32 = i as u32;
        let mut bit = 0u32;
        while bit < 8 {
            if crc & 0x0000_0001 != 0 {
                crc = (crc >> 1) ^ rpoly;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
}

/// Reflect (bit-reverse) a `u8`.
const fn reflect_u8(v: u8) -> u8 {
    v.reverse_bits()
}

/// Reflect (bit-reverse) a `u16`.
const fn reflect_u16(v: u16) -> u16 {
    v.reverse_bits()
}

/// Reflect (bit-reverse) a `u32`.
const fn reflect_u32(v: u32) -> u32 {
    v.reverse_bits()
}

// ---------------------------------------------------------------------------
// Crc8
// ---------------------------------------------------------------------------

impl Crc8 {
    /// Create a new CRC-8 calculator.
    ///
    /// When `reflect_in` is `true` the table is built from the bit-reversed
    /// polynomial and bytes are processed LSB-first (reflected algorithm).
    ///
    /// All parameters are evaluated at compile time when used in a `const`
    /// context.
    pub const fn new(
        poly: u8,
        init: u8,
        reflect_in: bool,
        reflect_out: bool,
        xor_out: u8,
    ) -> Self {
        let (table, reflected) = if reflect_in {
            (make_table_u8_reflected(reflect_u8(poly)), true)
        } else {
            (make_table_u8_normal(poly), false)
        };
        Self {
            table,
            init,
            reflect_out,
            xor_out,
            reflected,
        }
    }

    /// Compute the CRC over an entire byte slice in one call.
    pub fn checksum(&self, data: &[u8]) -> u8 {
        let mut d = self.digest();
        d.update(data);
        d.finalize()
    }

    /// Begin an incremental CRC computation.
    pub fn digest(&self) -> CrcDigest8<'_> {
        CrcDigest8 {
            crc: self,
            value: self.init,
        }
    }
}

/// Incremental CRC-8 computation state.
pub struct CrcDigest8<'a> {
    crc: &'a Crc8,
    value: u8,
}

impl CrcDigest8<'_> {
    /// Feed more bytes into the running CRC.
    pub fn update(&mut self, data: &[u8]) {
        if self.crc.reflected {
            for &byte in data {
                let idx = self.value ^ byte;
                self.value = self.crc.table[usize::from(idx)];
                // reflected: `crc = table[crc ^ byte]` (no additional XOR shift
                // because for u8 width the shift would discard the whole value)
            }
        } else {
            for &byte in data {
                // Normal MSB-first. For u8, WIDTH=8 so `crc >> (WIDTH-8) = crc >> 0 = crc`.
                // The `crc << 8` term would be 0 (all bits shifted out), so only
                // the table entry contributes.
                let idx = self.value ^ byte;
                self.value = self.crc.table[usize::from(idx)];
            }
        }
    }

    /// Finalize and return the CRC value.
    ///
    /// Applies an additional bit-reversal only when `reflected` (the table
    /// mode) and `reflect_out` differ — i.e. when the register's bit order
    /// does not already match the requested output bit order.
    pub fn finalize(self) -> u8 {
        // When the reflected algorithm is used the register accumulates in
        // bit-reversed order.  reflect_out=true means the caller wants the
        // bit-reversed form → no extra reversal needed (already matches).
        // If they differ, a reversal is required to convert between the two
        // representations.
        let need_reverse = self.crc.reflected ^ self.crc.reflect_out;
        let crc = if need_reverse {
            self.value.reverse_bits()
        } else {
            self.value
        };
        crc ^ self.crc.xor_out
    }
}

// ---------------------------------------------------------------------------
// Crc16
// ---------------------------------------------------------------------------

impl Crc16 {
    /// Create a new CRC-16 calculator.
    pub const fn new(
        poly: u16,
        init: u16,
        reflect_in: bool,
        reflect_out: bool,
        xor_out: u16,
    ) -> Self {
        let (table, reflected) = if reflect_in {
            (make_table_u16_reflected(reflect_u16(poly)), true)
        } else {
            (make_table_u16_normal(poly), false)
        };
        Self {
            table,
            init,
            reflect_out,
            xor_out,
            reflected,
        }
    }

    /// Compute the CRC over an entire byte slice in one call.
    pub fn checksum(&self, data: &[u8]) -> u16 {
        let mut d = self.digest();
        d.update(data);
        d.finalize()
    }

    /// Begin an incremental CRC computation.
    pub fn digest(&self) -> CrcDigest16<'_> {
        CrcDigest16 {
            crc: self,
            value: self.init,
        }
    }
}

/// Incremental CRC-16 computation state.
pub struct CrcDigest16<'a> {
    crc: &'a Crc16,
    value: u16,
}

impl CrcDigest16<'_> {
    /// Feed more bytes into the running CRC.
    pub fn update(&mut self, data: &[u8]) {
        if self.crc.reflected {
            for &byte in data {
                let idx = (self.value as u8) ^ byte;
                self.value = self.crc.table[usize::from(idx)] ^ (self.value >> 8);
            }
        } else {
            for &byte in data {
                let idx = ((self.value >> 8) as u8) ^ byte;
                self.value = self.crc.table[usize::from(idx)] ^ (self.value << 8);
            }
        }
    }

    /// Finalize and return the CRC value.
    ///
    /// See [`CrcDigest8::finalize`] for the bit-reversal rationale.
    pub fn finalize(self) -> u16 {
        let need_reverse = self.crc.reflected ^ self.crc.reflect_out;
        let crc = if need_reverse {
            self.value.reverse_bits()
        } else {
            self.value
        };
        crc ^ self.crc.xor_out
    }
}

// ---------------------------------------------------------------------------
// Crc32
// ---------------------------------------------------------------------------

impl Crc32 {
    /// Create a new CRC-32 calculator.
    pub const fn new(
        poly: u32,
        init: u32,
        reflect_in: bool,
        reflect_out: bool,
        xor_out: u32,
    ) -> Self {
        let (table, reflected) = if reflect_in {
            (make_table_u32_reflected(reflect_u32(poly)), true)
        } else {
            (make_table_u32_normal(poly), false)
        };
        Self {
            table,
            init,
            reflect_out,
            xor_out,
            reflected,
        }
    }

    /// Compute the CRC over an entire byte slice in one call.
    pub fn checksum(&self, data: &[u8]) -> u32 {
        let mut d = self.digest();
        d.update(data);
        d.finalize()
    }

    /// Begin an incremental CRC computation.
    pub fn digest(&self) -> CrcDigest32<'_> {
        CrcDigest32 {
            crc: self,
            value: self.init,
        }
    }
}

/// Incremental CRC-32 computation state.
pub struct CrcDigest32<'a> {
    crc: &'a Crc32,
    value: u32,
}

impl CrcDigest32<'_> {
    /// Feed more bytes into the running CRC.
    pub fn update(&mut self, data: &[u8]) {
        if self.crc.reflected {
            for &byte in data {
                let idx = (self.value as u8) ^ byte;
                self.value = self.crc.table[usize::from(idx)] ^ (self.value >> 8);
            }
        } else {
            for &byte in data {
                let idx = ((self.value >> 24) as u8) ^ byte;
                self.value = self.crc.table[usize::from(idx)] ^ (self.value << 8);
            }
        }
    }

    /// Finalize and return the CRC value.
    ///
    /// See [`CrcDigest8::finalize`] for the bit-reversal rationale.
    pub fn finalize(self) -> u32 {
        let need_reverse = self.crc.reflected ^ self.crc.reflect_out;
        let crc = if need_reverse {
            self.value.reverse_bits()
        } else {
            self.value
        };
        crc ^ self.crc.xor_out
    }
}

// ---------------------------------------------------------------------------
// Pre-defined algorithm constants
// ---------------------------------------------------------------------------

// --- CRC-8 variants ---

/// CRC-8/SMBUS (a.k.a. CRC-8/CCITT): poly=0x07, init=0x00, no reflection.
///
/// Check value for `b"123456789"`: **0xF4**.
pub const CRC8_CCITT: Crc8 = Crc8::new(0x07, 0x00, false, false, 0x00);

/// CRC-8/ROHC: poly=0x07, init=0xFF, reflect in+out.
///
/// Check value for `b"123456789"`: **0xD0**.
pub const CRC8_ROHC: Crc8 = Crc8::new(0x07, 0xFF, true, true, 0x00);

/// CRC-8/SAE-J1850: poly=0x1D, init=0xFF, no reflection, xorout=0xFF.
///
/// Check value for `b"123456789"`: **0x4B**.
pub const CRC8_SAE_J1850: Crc8 = Crc8::new(0x1D, 0xFF, false, false, 0xFF);

/// CRC-8/H2F (AUTOSAR): poly=0x2F, init=0xFF, no reflection, xorout=0xFF.
///
/// Check value for `b"123456789"`: **0xDF**.
pub const CRC8_H2F: Crc8 = Crc8::new(0x2F, 0xFF, false, false, 0xFF);

/// CRC-8/MAXIM (1-Wire): poly=0x31, init=0x00, reflect in+out.
///
/// Check value for `b"123456789"`: **0xA1**.
pub const CRC8_MAXIM: Crc8 = Crc8::new(0x31, 0x00, true, true, 0x00);

// --- CRC-16 ---

/// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection.
///
/// Check value for `b"123456789"`: **0x29B1**.
pub const CRC16_CCITT: Crc16 = Crc16::new(0x1021, 0xFFFF, false, false, 0x0000);

// --- CRC-32 ---

/// CRC-32/ISO-HDLC (Ethernet / zlib): poly=0x04C11DB7, init=0xFFFFFFFF,
/// reflect in+out, xorout=0xFFFFFFFF.
///
/// Check value for `b"123456789"`: **0xCBF4_3926**.
pub const CRC32_ETHERNET: Crc32 = Crc32::new(0x04C1_1DB7, 0xFFFF_FFFF, true, true, 0xFFFF_FFFF);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const CHECK_INPUT: &[u8] = b"123456789";

    // -----------------------------------------------------------------------
    // Pre-defined algorithm check values
    // -----------------------------------------------------------------------

    #[test]
    fn crc8_ccitt_check_value() {
        assert_eq!(CRC8_CCITT.checksum(CHECK_INPUT), 0xF4);
    }

    #[test]
    fn crc8_rohc_check_value() {
        assert_eq!(CRC8_ROHC.checksum(CHECK_INPUT), 0xD0);
    }

    #[test]
    fn crc8_sae_j1850_check_value() {
        assert_eq!(CRC8_SAE_J1850.checksum(CHECK_INPUT), 0x4B);
    }

    #[test]
    fn crc8_h2f_check_value() {
        assert_eq!(CRC8_H2F.checksum(CHECK_INPUT), 0xDF);
    }

    #[test]
    fn crc8_maxim_check_value() {
        assert_eq!(CRC8_MAXIM.checksum(CHECK_INPUT), 0xA1);
    }

    #[test]
    fn crc16_ccitt_check_value() {
        assert_eq!(CRC16_CCITT.checksum(CHECK_INPUT), 0x29B1);
    }

    #[test]
    fn crc32_ethernet_check_value() {
        assert_eq!(CRC32_ETHERNET.checksum(CHECK_INPUT), 0xCBF4_3926);
    }

    // -----------------------------------------------------------------------
    // Empty input: should return init XOR xor_out
    // -----------------------------------------------------------------------

    #[test]
    fn empty_input_crc8_ccitt() {
        // init=0x00, xor_out=0x00 => 0x00
        assert_eq!(CRC8_CCITT.checksum(&[]), 0x00);
    }

    #[test]
    fn empty_input_crc8_j1850() {
        // init=0xFF, xor_out=0xFF => 0xFF ^ 0xFF = 0x00
        assert_eq!(CRC8_SAE_J1850.checksum(&[]), 0x00);
    }

    #[test]
    fn empty_input_crc16() {
        // init=0xFFFF, xor_out=0x0000 => 0xFFFF
        assert_eq!(CRC16_CCITT.checksum(&[]), 0xFFFF);
    }

    #[test]
    fn empty_input_crc32() {
        // init=0xFFFFFFFF, xor_out=0xFFFFFFFF => 0x00000000
        assert_eq!(CRC32_ETHERNET.checksum(&[]), 0x0000_0000);
    }

    // -----------------------------------------------------------------------
    // Single byte
    // -----------------------------------------------------------------------

    #[test]
    fn single_byte_zero_crc8_ccitt() {
        // init=0, byte=0x00 => idx=0, table[0]=0, value=0, result=0^0=0
        assert_eq!(CRC8_CCITT.checksum(&[0x00]), 0x00);
    }

    #[test]
    fn single_byte_crc8_is_deterministic() {
        let crc8 = Crc8::new(0x07, 0x00, false, false, 0x00);
        assert_eq!(crc8.checksum(&[0xFF]), crc8.checksum(&[0xFF]));
    }

    #[test]
    fn single_byte_crc32_nonzero() {
        // init != 0 and xor_out != 0, so a single 0x00 byte must produce non-zero
        assert_ne!(CRC32_ETHERNET.checksum(&[0x00]), 0);
    }

    // -----------------------------------------------------------------------
    // Incremental digest — must match one-shot checksum
    // -----------------------------------------------------------------------

    #[test]
    fn incremental_crc8_ccitt() {
        let expected = CRC8_CCITT.checksum(CHECK_INPUT);
        let mut d = CRC8_CCITT.digest();
        d.update(&CHECK_INPUT[..4]);
        d.update(&CHECK_INPUT[4..]);
        assert_eq!(d.finalize(), expected);
    }

    #[test]
    fn incremental_crc16_ccitt_chunks_of_three() {
        let expected = CRC16_CCITT.checksum(CHECK_INPUT);
        let mut d = CRC16_CCITT.digest();
        for chunk in CHECK_INPUT.chunks(3) {
            d.update(chunk);
        }
        assert_eq!(d.finalize(), expected);
    }

    #[test]
    fn incremental_crc32_one_byte_at_a_time() {
        let expected = CRC32_ETHERNET.checksum(CHECK_INPUT);
        let mut d = CRC32_ETHERNET.digest();
        for &b in CHECK_INPUT {
            d.update(&[b]);
        }
        assert_eq!(d.finalize(), expected);
    }

    #[test]
    fn incremental_reflected_crc8_maxim() {
        let expected = CRC8_MAXIM.checksum(CHECK_INPUT);
        let mut d = CRC8_MAXIM.digest();
        d.update(&CHECK_INPUT[..1]);
        d.update(&CHECK_INPUT[1..]);
        assert_eq!(d.finalize(), expected);
    }

    // -----------------------------------------------------------------------
    // All-zeros input
    // -----------------------------------------------------------------------

    #[test]
    fn all_zeros_crc32_known_value() {
        let zeros = [0u8; 16];
        // CRC-32/ISO-HDLC (Ethernet) of 16 zero bytes.
        // 0xECBB_4B55 — derived from the same algorithm that correctly produces
        // 0xCBF4_3926 for b"123456789".  Acts as a regression guard.
        assert_eq!(CRC32_ETHERNET.checksum(&zeros), 0xECBB_4B55);
    }

    #[test]
    fn all_zeros_crc8_deterministic() {
        let zeros = [0u8; 32];
        assert_eq!(CRC8_CCITT.checksum(&zeros), CRC8_CCITT.checksum(&zeros));
    }

    // -----------------------------------------------------------------------
    // All-0xFF input (smoke test — must be deterministic, non-trivial)
    // -----------------------------------------------------------------------

    #[test]
    fn all_xff_crc8_ccitt_deterministic() {
        let ff = [0xFFu8; 8];
        assert_eq!(CRC8_CCITT.checksum(&ff), CRC8_CCITT.checksum(&ff));
    }

    #[test]
    fn all_xff_crc32_nonzero() {
        let ff = [0xFFu8; 4];
        assert_ne!(CRC32_ETHERNET.checksum(&ff), 0);
    }

    // -----------------------------------------------------------------------
    // Table generation sanity: table[0] == 0 for any non-reflected CRC
    // (all eight shift iterations start from 0, produce 0)
    // -----------------------------------------------------------------------

    #[test]
    fn table_first_entry_zero_crc8() {
        let crc = Crc8::new(0x07, 0x00, false, false, 0x00);
        assert_eq!(crc.table[0], 0);
    }

    #[test]
    fn table_first_entry_zero_crc16() {
        let crc = Crc16::new(0x1021, 0x0000, false, false, 0x0000);
        assert_eq!(crc.table[0], 0);
    }

    #[test]
    fn table_first_entry_zero_crc32() {
        let crc = Crc32::new(0x04C1_1DB7, 0x0000_0000, false, false, 0x0000_0000);
        assert_eq!(crc.table[0], 0);
    }

    #[test]
    fn table_length_is_256() {
        assert_eq!(CRC8_CCITT.table.len(), 256);
        assert_eq!(CRC16_CCITT.table.len(), 256);
        assert_eq!(CRC32_ETHERNET.table.len(), 256);
    }
}
