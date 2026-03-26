//! Big-endian serialization types for protocol frames.
//!
//! Ports the C++ `estd::big_endian` family to Rust with `no_std` support.
//!
//! ## Unsigned standard-width types
//! - [`BeU16`], [`BeU32`], [`BeU64`] — 16 / 32 / 64-bit big-endian values
//!
//! ## Unsigned sub-width types
//! - [`BeU24`] — 24-bit value stored in 3 bytes, native type `u32`
//! - [`BeU48`] — 48-bit value stored in 6 bytes, native type `u64`
//!
//! ## Signed variants
//! - [`BeI16`], [`BeI32`], [`BeI64`]
//!
//! ## Free byte-level functions
//! [`read_be_u16`], [`read_be_u32`], [`read_be_u64`], [`read_be_u24`], [`read_be_u48`],
//! [`write_be_u16`], [`write_be_u32`], [`write_be_u64`], [`write_be_u24`], [`write_be_u48`]
//!
//! ## Free bit-level functions (CAN signal extraction)
//! [`read_be_bits`], [`write_be_bits`] — operate on arbitrary bit fields, MSB-first.

use core::fmt;

// ---------------------------------------------------------------------------
// Sealed trait — BeInteger
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
    impl Sealed for u8 {}
    impl Sealed for u16 {}
    impl Sealed for u32 {}
    impl Sealed for u64 {}
}

/// Trait for integer types that can be used with [`read_be_bits`] / [`write_be_bits`].
///
/// Sealed — cannot be implemented outside this crate.
pub trait BeInteger: Copy + sealed::Sealed {
    /// Construct from a `u64`, truncating to `Self::bit_width()` bits.
    fn from_u64(v: u64) -> Self;
    /// Widen to `u64`.
    fn to_u64(self) -> u64;
    /// Number of bits in this integer type.
    fn bit_width() -> usize;
}

impl BeInteger for u8 {
    #[inline]
    fn from_u64(v: u64) -> Self {
        v as u8
    }
    #[inline]
    fn to_u64(self) -> u64 {
        u64::from(self)
    }
    #[inline]
    fn bit_width() -> usize {
        8
    }
}

impl BeInteger for u16 {
    #[inline]
    fn from_u64(v: u64) -> Self {
        v as u16
    }
    #[inline]
    fn to_u64(self) -> u64 {
        u64::from(self)
    }
    #[inline]
    fn bit_width() -> usize {
        16
    }
}

impl BeInteger for u32 {
    #[inline]
    fn from_u64(v: u64) -> Self {
        v as u32
    }
    #[inline]
    fn to_u64(self) -> u64 {
        u64::from(self)
    }
    #[inline]
    fn bit_width() -> usize {
        32
    }
}

impl BeInteger for u64 {
    #[inline]
    fn from_u64(v: u64) -> Self {
        v
    }
    #[inline]
    fn to_u64(self) -> u64 {
        self
    }
    #[inline]
    fn bit_width() -> usize {
        64
    }
}

// ---------------------------------------------------------------------------
// BeU16
// ---------------------------------------------------------------------------

/// 16-bit unsigned integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeU16([u8; 2]);

impl BeU16 {
    /// Create from a native `u16`.
    #[inline]
    pub fn new(value: u16) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `u16`.
    #[inline]
    pub fn get(self) -> u16 {
        u16::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 2`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 2, "BeU16::from_bytes requires exactly 2 bytes");
        let mut arr = [0u8; 2];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeU16 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 2])
    }
}

impl From<u16> for BeU16 {
    #[inline]
    fn from(v: u16) -> Self {
        Self::new(v)
    }
}

impl From<BeU16> for u16 {
    #[inline]
    fn from(b: BeU16) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeU16 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeU16({:#010x})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeU32
// ---------------------------------------------------------------------------

/// 32-bit unsigned integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeU32([u8; 4]);

impl BeU32 {
    /// Create from a native `u32`.
    #[inline]
    pub fn new(value: u32) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `u32`.
    #[inline]
    pub fn get(self) -> u32 {
        u32::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 4`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 4, "BeU32::from_bytes requires exactly 4 bytes");
        let mut arr = [0u8; 4];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeU32 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 4])
    }
}

impl From<u32> for BeU32 {
    #[inline]
    fn from(v: u32) -> Self {
        Self::new(v)
    }
}

impl From<BeU32> for u32 {
    #[inline]
    fn from(b: BeU32) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeU32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeU32({:#010x})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeU64
// ---------------------------------------------------------------------------

/// 64-bit unsigned integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeU64([u8; 8]);

impl BeU64 {
    /// Create from a native `u64`.
    #[inline]
    pub fn new(value: u64) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `u64`.
    #[inline]
    pub fn get(self) -> u64 {
        u64::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 8`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 8, "BeU64::from_bytes requires exactly 8 bytes");
        let mut arr = [0u8; 8];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeU64 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 8])
    }
}

impl From<u64> for BeU64 {
    #[inline]
    fn from(v: u64) -> Self {
        Self::new(v)
    }
}

impl From<BeU64> for u64 {
    #[inline]
    fn from(b: BeU64) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeU64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeU64({:#018x})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeU24  (3 bytes → u32)
// ---------------------------------------------------------------------------

/// 24-bit unsigned integer stored in big-endian byte order.
///
/// The native representation is `u32`; only the low 24 bits are used.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeU24([u8; 3]);

impl BeU24 {
    /// Create from the low 24 bits of a native `u32`.
    #[inline]
    pub fn new(value: u32) -> Self {
        let b = value.to_be_bytes();
        // b = [b3, b2, b1, b0]; drop the leading (most-significant) byte
        Self([b[1], b[2], b[3]])
    }

    /// Return the value as a native `u32`.
    #[inline]
    pub fn get(self) -> u32 {
        u32::from_be_bytes([0, self.0[0], self.0[1], self.0[2]])
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 3`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 3, "BeU24::from_bytes requires exactly 3 bytes");
        let mut arr = [0u8; 3];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeU24 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 3])
    }
}

impl From<u32> for BeU24 {
    #[inline]
    fn from(v: u32) -> Self {
        Self::new(v)
    }
}

impl From<BeU24> for u32 {
    #[inline]
    fn from(b: BeU24) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeU24 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeU24({:#08x})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeU48  (6 bytes → u64)
// ---------------------------------------------------------------------------

/// 48-bit unsigned integer stored in big-endian byte order.
///
/// The native representation is `u64`; only the low 48 bits are used.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeU48([u8; 6]);

impl BeU48 {
    /// Create from the low 48 bits of a native `u64`.
    #[inline]
    pub fn new(value: u64) -> Self {
        let b = value.to_be_bytes();
        // b = [b7..b0]; keep bytes 2..8 (low 6 bytes)
        Self([b[2], b[3], b[4], b[5], b[6], b[7]])
    }

    /// Return the value as a native `u64`.
    #[inline]
    pub fn get(self) -> u64 {
        u64::from_be_bytes([
            0, 0, self.0[0], self.0[1], self.0[2], self.0[3], self.0[4], self.0[5],
        ])
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 6`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 6, "BeU48::from_bytes requires exactly 6 bytes");
        let mut arr = [0u8; 6];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeU48 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 6])
    }
}

impl From<u64> for BeU48 {
    #[inline]
    fn from(v: u64) -> Self {
        Self::new(v)
    }
}

impl From<BeU48> for u64 {
    #[inline]
    fn from(b: BeU48) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeU48 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeU48({:#014x})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeI16
// ---------------------------------------------------------------------------

/// 16-bit signed integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeI16([u8; 2]);

impl BeI16 {
    /// Create from a native `i16`.
    #[inline]
    pub fn new(value: i16) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `i16`.
    #[inline]
    pub fn get(self) -> i16 {
        i16::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 2`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 2, "BeI16::from_bytes requires exactly 2 bytes");
        let mut arr = [0u8; 2];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeI16 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 2])
    }
}

impl From<i16> for BeI16 {
    #[inline]
    fn from(v: i16) -> Self {
        Self::new(v)
    }
}

impl From<BeI16> for i16 {
    #[inline]
    fn from(b: BeI16) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeI16 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeI16({})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeI32
// ---------------------------------------------------------------------------

/// 32-bit signed integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeI32([u8; 4]);

impl BeI32 {
    /// Create from a native `i32`.
    #[inline]
    pub fn new(value: i32) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `i32`.
    #[inline]
    pub fn get(self) -> i32 {
        i32::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 4`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 4, "BeI32::from_bytes requires exactly 4 bytes");
        let mut arr = [0u8; 4];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeI32 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 4])
    }
}

impl From<i32> for BeI32 {
    #[inline]
    fn from(v: i32) -> Self {
        Self::new(v)
    }
}

impl From<BeI32> for i32 {
    #[inline]
    fn from(b: BeI32) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeI32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeI32({})", self.get())
    }
}

// ---------------------------------------------------------------------------
// BeI64
// ---------------------------------------------------------------------------

/// 64-bit signed integer stored in big-endian byte order.
#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BeI64([u8; 8]);

impl BeI64 {
    /// Create from a native `i64`.
    #[inline]
    pub fn new(value: i64) -> Self {
        Self(value.to_be_bytes())
    }

    /// Return the value as a native `i64`.
    #[inline]
    pub fn get(self) -> i64 {
        i64::from_be_bytes(self.0)
    }

    /// View the raw big-endian bytes.
    #[inline]
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    /// Construct from a byte slice.
    ///
    /// # Panics
    /// Panics if `bytes.len() != 8`.
    #[inline]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 8, "BeI64::from_bytes requires exactly 8 bytes");
        let mut arr = [0u8; 8];
        arr.copy_from_slice(bytes);
        Self(arr)
    }
}

impl Default for BeI64 {
    #[inline]
    fn default() -> Self {
        Self([0u8; 8])
    }
}

impl From<i64> for BeI64 {
    #[inline]
    fn from(v: i64) -> Self {
        Self::new(v)
    }
}

impl From<BeI64> for i64 {
    #[inline]
    fn from(b: BeI64) -> Self {
        b.get()
    }
}

impl fmt::Debug for BeI64 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "BeI64({})", self.get())
    }
}

// ---------------------------------------------------------------------------
// Free functions — byte-level reads
// ---------------------------------------------------------------------------

/// Read a big-endian `u16` from the first 2 bytes of `src`.
///
/// # Panics
/// Panics if `src.len() < 2`.
#[inline]
pub fn read_be_u16(src: &[u8]) -> u16 {
    u16::from_be_bytes([src[0], src[1]])
}

/// Read a big-endian `u32` from the first 4 bytes of `src`.
///
/// # Panics
/// Panics if `src.len() < 4`.
#[inline]
pub fn read_be_u32(src: &[u8]) -> u32 {
    u32::from_be_bytes([src[0], src[1], src[2], src[3]])
}

/// Read a big-endian `u64` from the first 8 bytes of `src`.
///
/// # Panics
/// Panics if `src.len() < 8`.
#[inline]
pub fn read_be_u64(src: &[u8]) -> u64 {
    u64::from_be_bytes([
        src[0], src[1], src[2], src[3], src[4], src[5], src[6], src[7],
    ])
}

/// Read a big-endian 24-bit unsigned integer from the first 3 bytes of `src`,
/// returning it as a `u32`.
///
/// # Panics
/// Panics if `src.len() < 3`.
#[inline]
pub fn read_be_u24(src: &[u8]) -> u32 {
    u32::from_be_bytes([0, src[0], src[1], src[2]])
}

/// Read a big-endian 48-bit unsigned integer from the first 6 bytes of `src`,
/// returning it as a `u64`.
///
/// # Panics
/// Panics if `src.len() < 6`.
#[inline]
pub fn read_be_u48(src: &[u8]) -> u64 {
    u64::from_be_bytes([0, 0, src[0], src[1], src[2], src[3], src[4], src[5]])
}

// ---------------------------------------------------------------------------
// Free functions — byte-level writes
// ---------------------------------------------------------------------------

/// Write `value` as a big-endian `u16` into the first 2 bytes of `dst`.
///
/// # Panics
/// Panics if `dst.len() < 2`.
#[inline]
pub fn write_be_u16(dst: &mut [u8], value: u16) {
    dst[..2].copy_from_slice(&value.to_be_bytes());
}

/// Write `value` as a big-endian `u32` into the first 4 bytes of `dst`.
///
/// # Panics
/// Panics if `dst.len() < 4`.
#[inline]
pub fn write_be_u32(dst: &mut [u8], value: u32) {
    dst[..4].copy_from_slice(&value.to_be_bytes());
}

/// Write `value` as a big-endian `u64` into the first 8 bytes of `dst`.
///
/// # Panics
/// Panics if `dst.len() < 8`.
#[inline]
pub fn write_be_u64(dst: &mut [u8], value: u64) {
    dst[..8].copy_from_slice(&value.to_be_bytes());
}

/// Write the low 24 bits of `value` as a big-endian 24-bit integer into the
/// first 3 bytes of `dst`.
///
/// # Panics
/// Panics if `dst.len() < 3`.
#[inline]
pub fn write_be_u24(dst: &mut [u8], value: u32) {
    let b = value.to_be_bytes();
    dst[..3].copy_from_slice(&b[1..]);
}

/// Write the low 48 bits of `value` as a big-endian 48-bit integer into the
/// first 6 bytes of `dst`.
///
/// # Panics
/// Panics if `dst.len() < 6`.
#[inline]
pub fn write_be_u48(dst: &mut [u8], value: u64) {
    let b = value.to_be_bytes();
    dst[..6].copy_from_slice(&b[2..]);
}

// ---------------------------------------------------------------------------
// Free functions — bit-level
// ---------------------------------------------------------------------------

/// Read `length` bits starting at bit `offset` from `src`, big-endian bit
/// order (bit 0 = MSB of byte 0), and return as `T`.
///
/// # Panics
/// Panics if the bit range `offset..offset+length` extends beyond `src`.
/// Panics if `length > T::bit_width()`.
pub fn read_be_bits<T: BeInteger>(src: &[u8], offset: usize, length: usize) -> T {
    assert!(length <= T::bit_width(), "length exceeds target type width");
    assert!(
        offset + length <= src.len() * 8,
        "bit range out of bounds"
    );

    if length == 0 {
        return T::from_u64(0);
    }

    let mut result: u64 = 0;
    let mut bits_remaining = length;
    let mut bit_pos = offset;

    while bits_remaining > 0 {
        let byte_idx = bit_pos / 8;
        let bit_in_byte = bit_pos % 8; // 0 = MSB of this byte
        let bits_available = 8 - bit_in_byte;
        let bits_to_take = bits_available.min(bits_remaining);

        // Extract `bits_to_take` bits starting at MSB position `bit_in_byte`
        // of the current byte.
        let shift = bits_available - bits_to_take; // right-shift to align to LSB
        // Use u16 for the mask to avoid overflow when bits_to_take == 8.
        let mask = ((1u16 << bits_to_take) - 1) as u8;
        let extracted = u64::from((src[byte_idx] >> shift) & mask);

        result = (result << bits_to_take) | extracted;
        bit_pos += bits_to_take;
        bits_remaining -= bits_to_take;
    }

    T::from_u64(result)
}

/// Write the low `length` bits of `value` starting at bit `offset` in `dst`,
/// big-endian bit order (bit 0 = MSB of byte 0).
///
/// Bits outside the specified range in `dst` are left unchanged.
///
/// # Panics
/// Panics if the bit range `offset..offset+length` extends beyond `dst`.
/// Panics if `length > T::bit_width()`.
pub fn write_be_bits<T: BeInteger>(dst: &mut [u8], value: T, offset: usize, length: usize) {
    assert!(length <= T::bit_width(), "length exceeds source type width");
    assert!(
        offset + length <= dst.len() * 8,
        "bit range out of bounds"
    );

    if length == 0 {
        return;
    }

    // Shift value so bit `length-1` is at position 0.
    // We will place bits from MSB to LSB into the destination.
    let mut value_bits = value.to_u64();
    // Mask to `length` bits to drop any high bits the caller may have set.
    if length < 64 {
        value_bits &= (1u64 << length) - 1;
    }

    let mut bits_remaining = length;
    let mut bit_pos = offset;

    while bits_remaining > 0 {
        let byte_idx = bit_pos / 8;
        let bit_in_byte = bit_pos % 8; // 0 = MSB of this byte
        let bits_available = 8 - bit_in_byte;
        let bits_to_write = bits_available.min(bits_remaining);

        // The slice of value_bits we want is bits [bits_remaining-1 .. bits_remaining-bits_to_write]
        let shift = bits_remaining - bits_to_write;
        // Use u16 for masks to avoid overflow when bits_to_write == 8.
        let chunk = ((value_bits >> shift) as u8) & (((1u16 << bits_to_write) - 1) as u8);

        // Place into the byte: shift left by (bits_available - bits_to_write)
        let byte_shift = bits_available - bits_to_write;
        let byte_mask = (((1u16 << bits_to_write) - 1) as u8) << byte_shift;

        dst[byte_idx] = (dst[byte_idx] & !byte_mask) | (chunk << byte_shift);

        bit_pos += bits_to_write;
        bits_remaining -= bits_to_write;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- BeU16 ---

    #[test]
    fn beu16_roundtrip() {
        assert_eq!(BeU16::from(0x1234u16).get(), 0x1234u16);
    }

    #[test]
    fn beu16_bytes_are_big_endian() {
        assert_eq!(BeU16::from(0x0102u16).as_bytes(), &[0x01, 0x02]);
    }

    #[test]
    fn beu16_zero_roundtrip() {
        assert_eq!(BeU16::from(0u16).get(), 0u16);
    }

    #[test]
    fn beu16_max_roundtrip() {
        assert_eq!(BeU16::from(u16::MAX).get(), u16::MAX);
    }

    #[test]
    fn beu16_default_is_zero() {
        assert_eq!(BeU16::default().get(), 0u16);
    }

    #[test]
    fn beu16_debug_format() {
        let s = format!("{:?}", BeU16::from(0x1234u16));
        assert!(s.contains("0x00001234") || s.contains("1234"), "got: {s}");
    }

    #[test]
    fn beu16_from_bytes() {
        assert_eq!(BeU16::from_bytes(&[0xAB, 0xCD]).get(), 0xABCDu16);
    }

    // --- BeU32 ---

    #[test]
    fn beu32_roundtrip() {
        assert_eq!(BeU32::from(0x1234_5678u32).get(), 0x1234_5678u32);
    }

    #[test]
    fn beu32_bytes_are_big_endian() {
        assert_eq!(
            BeU32::from(0x0102_0304u32).as_bytes(),
            &[0x01, 0x02, 0x03, 0x04]
        );
    }

    #[test]
    fn beu32_zero_roundtrip() {
        assert_eq!(BeU32::from(0u32).get(), 0u32);
    }

    #[test]
    fn beu32_max_roundtrip() {
        assert_eq!(BeU32::from(u32::MAX).get(), u32::MAX);
    }

    #[test]
    fn beu32_default_is_zero() {
        assert_eq!(BeU32::default().get(), 0u32);
    }

    #[test]
    fn beu32_debug_format() {
        let s = format!("{:?}", BeU32::from(0x1234_5678u32));
        assert!(s.contains("12345678"), "got: {s}");
    }

    // --- BeU64 ---

    #[test]
    fn beu64_roundtrip() {
        assert_eq!(
            BeU64::from(0x0102_0304_0506_0708u64).get(),
            0x0102_0304_0506_0708u64
        );
    }

    #[test]
    fn beu64_bytes_are_big_endian() {
        assert_eq!(
            BeU64::from(0x0102_0304_0506_0708u64).as_bytes(),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
        );
    }

    #[test]
    fn beu64_zero_roundtrip() {
        assert_eq!(BeU64::from(0u64).get(), 0u64);
    }

    #[test]
    fn beu64_max_roundtrip() {
        assert_eq!(BeU64::from(u64::MAX).get(), u64::MAX);
    }

    #[test]
    fn beu64_default_is_zero() {
        assert_eq!(BeU64::default().get(), 0u64);
    }

    // --- BeU24 ---

    #[test]
    fn beu24_roundtrip() {
        assert_eq!(BeU24::new(0x12_3456).get(), 0x12_3456u32);
    }

    #[test]
    fn beu24_bytes_are_big_endian() {
        assert_eq!(BeU24::new(0x01_0203).as_bytes(), &[0x01, 0x02, 0x03]);
    }

    #[test]
    fn beu24_zero_roundtrip() {
        assert_eq!(BeU24::new(0).get(), 0u32);
    }

    #[test]
    fn beu24_max_roundtrip() {
        assert_eq!(BeU24::new(0x00FF_FFFF).get(), 0x00FF_FFFFu32);
    }

    #[test]
    fn beu24_default_is_zero() {
        assert_eq!(BeU24::default().get(), 0u32);
    }

    #[test]
    fn beu24_from_bytes() {
        assert_eq!(BeU24::from_bytes(&[0x01, 0x02, 0x03]).get(), 0x01_0203u32);
    }

    // --- BeU48 ---

    #[test]
    fn beu48_roundtrip() {
        assert_eq!(BeU48::new(0x0001_0203_0405u64).get(), 0x0001_0203_0405u64);
    }

    #[test]
    fn beu48_bytes_are_big_endian() {
        assert_eq!(
            BeU48::new(0x0102_0304_0506u64).as_bytes(),
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
        );
    }

    #[test]
    fn beu48_zero_roundtrip() {
        assert_eq!(BeU48::new(0).get(), 0u64);
    }

    #[test]
    fn beu48_max_roundtrip() {
        assert_eq!(
            BeU48::new(0x0000_FFFF_FFFF_FFFFu64).get(),
            0x0000_FFFF_FFFF_FFFFu64
        );
    }

    #[test]
    fn beu48_default_is_zero() {
        assert_eq!(BeU48::default().get(), 0u64);
    }

    // --- Signed types ---

    #[test]
    fn bei16_negative_roundtrip() {
        assert_eq!(BeI16::from(-1_i16).get(), -1_i16);
        assert_eq!(BeI16::from(-32768_i16).get(), -32768_i16);
        assert_eq!(BeI16::from(32767_i16).get(), 32767_i16);
    }

    #[test]
    fn bei16_default_is_zero() {
        assert_eq!(BeI16::default().get(), 0_i16);
    }

    #[test]
    fn bei16_debug_format() {
        let s = format!("{:?}", BeI16::from(-1_i16));
        assert!(s.contains("-1"), "got: {s}");
    }

    #[test]
    fn bei32_negative_roundtrip() {
        assert_eq!(BeI32::from(-1_i32).get(), -1_i32);
        assert_eq!(BeI32::from(i32::MIN).get(), i32::MIN);
        assert_eq!(BeI32::from(i32::MAX).get(), i32::MAX);
    }

    #[test]
    fn bei32_default_is_zero() {
        assert_eq!(BeI32::default().get(), 0_i32);
    }

    #[test]
    fn bei64_negative_roundtrip() {
        assert_eq!(BeI64::from(-1_i64).get(), -1_i64);
        assert_eq!(BeI64::from(i64::MIN).get(), i64::MIN);
        assert_eq!(BeI64::from(i64::MAX).get(), i64::MAX);
    }

    #[test]
    fn bei64_default_is_zero() {
        assert_eq!(BeI64::default().get(), 0_i64);
    }

    // --- Free byte-level read functions ---

    #[test]
    fn read_beu16_correct() {
        assert_eq!(read_be_u16(&[0x12, 0x34]), 0x1234u16);
    }

    #[test]
    fn read_beu32_correct() {
        assert_eq!(read_be_u32(&[0x12, 0x34, 0x56, 0x78]), 0x1234_5678u32);
    }

    #[test]
    fn read_beu64_correct() {
        assert_eq!(
            read_be_u64(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]),
            0x0102_0304_0506_0708u64
        );
    }

    #[test]
    fn read_beu24_correct() {
        assert_eq!(read_be_u24(&[0x01, 0x02, 0x03]), 0x01_0203u32);
    }

    #[test]
    fn read_beu48_correct() {
        assert_eq!(
            read_be_u48(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]),
            0x0102_0304_0506u64
        );
    }

    // --- Free byte-level write functions ---

    #[test]
    fn write_beu16_correct() {
        let mut buf = [0u8; 2];
        write_be_u16(&mut buf, 0xABCDu16);
        assert_eq!(buf, [0xAB, 0xCD]);
    }

    #[test]
    fn write_beu32_correct() {
        let mut buf = [0u8; 4];
        write_be_u32(&mut buf, 0x1234_5678u32);
        assert_eq!(buf, [0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn write_beu64_correct() {
        let mut buf = [0u8; 8];
        write_be_u64(&mut buf, 0x0102_0304_0506_0708u64);
        assert_eq!(buf, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
    }

    #[test]
    fn write_beu24_correct() {
        let mut buf = [0u8; 3];
        write_be_u24(&mut buf, 0x01_0203u32);
        assert_eq!(buf, [0x01, 0x02, 0x03]);
    }

    #[test]
    fn write_beu48_correct() {
        let mut buf = [0u8; 6];
        write_be_u48(&mut buf, 0x0102_0304_0506u64);
        assert_eq!(buf, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
    }

    // --- Bit-level read ---

    #[test]
    fn read_be_bits_full_byte_at_offset_0() {
        // Read 8 bits at offset 0 → should equal first byte
        let buf = [0xABu8, 0xCD];
        assert_eq!(read_be_bits::<u8>(&buf, 0, 8), 0xABu8);
    }

    #[test]
    fn read_be_bits_second_byte() {
        let buf = [0xABu8, 0xCD];
        assert_eq!(read_be_bits::<u8>(&buf, 8, 8), 0xCDu8);
    }

    #[test]
    fn read_be_bits_nibble_at_offset_4() {
        // High nibble of second byte via offset 8+4=12? No: offset 4 = low nibble of first byte
        // Bits 4..8 of [0xAB] = 0xB
        let buf = [0xABu8];
        assert_eq!(read_be_bits::<u8>(&buf, 4, 4), 0x0Bu8);
    }

    #[test]
    fn read_be_bits_12_spanning_2_bytes() {
        // [0xAB, 0xCD]: bits 4..16 = low nibble of 0xAB (0xB) + full 0xCD = 0xBCD
        let buf = [0xABu8, 0xCD];
        assert_eq!(read_be_bits::<u16>(&buf, 4, 12), 0x0BCDu16);
    }

    #[test]
    fn read_be_bits_16_at_offset_0() {
        let buf = [0x12u8, 0x34];
        assert_eq!(read_be_bits::<u16>(&buf, 0, 16), 0x1234u16);
    }

    #[test]
    fn read_be_bits_zero_length() {
        let buf = [0xFFu8];
        assert_eq!(read_be_bits::<u8>(&buf, 0, 0), 0u8);
    }

    #[test]
    fn read_be_bits_single_bit() {
        // MSB of 0x80 is 1
        let buf = [0x80u8];
        assert_eq!(read_be_bits::<u8>(&buf, 0, 1), 1u8);
        // MSB of 0x7F is 0
        let buf2 = [0x7Fu8];
        assert_eq!(read_be_bits::<u8>(&buf2, 0, 1), 0u8);
    }

    // --- Bit-level write ---

    #[test]
    fn write_be_bits_roundtrip() {
        let mut buf = [0u8; 2];
        write_be_bits::<u16>(&mut buf, 0x1234u16, 0, 16);
        assert_eq!(read_be_bits::<u16>(&buf, 0, 16), 0x1234u16);
    }

    #[test]
    fn write_be_bits_partial_byte_no_corruption() {
        // Start with all-ones buffer; write 0x0 into bits 4..8; bits 0..4 should remain 0xF
        let mut buf = [0xFFu8];
        write_be_bits::<u8>(&mut buf, 0x0u8, 4, 4);
        // Low nibble zeroed, high nibble untouched
        assert_eq!(buf[0], 0xF0u8);
    }

    #[test]
    fn write_be_bits_partial_high_nibble_no_corruption() {
        // Start with 0x00; write 0xA into high nibble (bits 0..4); low nibble should stay 0
        let mut buf = [0x00u8];
        write_be_bits::<u8>(&mut buf, 0xAu8, 0, 4);
        assert_eq!(buf[0], 0xA0u8);
    }

    #[test]
    fn write_be_bits_spanning_bytes() {
        let mut buf = [0x00u8; 2];
        // Write 0xBCD into bits 4..16 (12 bits)
        write_be_bits::<u16>(&mut buf, 0x0BCDu16, 4, 12);
        // Byte 0 bits 4..8 should be 0xB, byte 1 should be 0xCD
        assert_eq!(buf[0], 0x0Bu8);
        assert_eq!(buf[1], 0xCDu8);
    }

    #[test]
    fn write_be_bits_zero_length_noop() {
        let mut buf = [0xFFu8];
        write_be_bits::<u8>(&mut buf, 0x00u8, 0, 0);
        assert_eq!(buf[0], 0xFF);
    }

    // --- Debug format ---

    #[test]
    fn beu32_debug_shows_hex() {
        let s = format!("{:?}", BeU32::new(0x1234_5678u32));
        // Must contain the hex digits
        assert!(s.contains("12345678"), "got: {s}");
    }

    #[test]
    fn bei16_debug_shows_decimal() {
        let s = format!("{:?}", BeI16::new(-100_i16));
        assert!(s.contains("-100"), "got: {s}");
    }
}
