//! Fixed-capacity, inline UTF-8 string.

use core::fmt;

/// Error returned when an append would exceed string capacity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityError;

/// UTF-8 string backed by an inline byte array.
#[derive(Clone, PartialEq, Eq)]
pub struct BoundedString<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> BoundedString<N> {
    /// Create an empty string.
    pub const fn new() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }

    /// Create a string from a slice, failing atomically if it does not fit.
    pub fn try_from_str(value: &str) -> Result<Self, CapacityError> {
        let mut result = Self::new();
        result.try_push_str(value)?;
        Ok(result)
    }

    /// Return the current string.
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len])
            .expect("BoundedString only accepts complete UTF-8 strings")
    }

    /// Current length in bytes.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Inline capacity in bytes.
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Whether the string is empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Remaining capacity in bytes.
    pub const fn remaining_capacity(&self) -> usize {
        N - self.len
    }

    /// Append a string, leaving `self` unchanged on insufficient capacity.
    pub fn try_push_str(&mut self, value: &str) -> Result<(), CapacityError> {
        if value.len() > self.remaining_capacity() {
            return Err(CapacityError);
        }
        let end = self.len + value.len();
        self.bytes[self.len..end].copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }

    /// Append one Unicode scalar value.
    pub fn try_push(&mut self, value: char) -> Result<(), CapacityError> {
        let mut encoded = [0_u8; 4];
        self.try_push_str(value.encode_utf8(&mut encoded))
    }

    /// Remove and return the final character.
    pub fn pop(&mut self) -> Option<char> {
        let value = self.as_str().chars().next_back()?;
        self.len -= value.len_utf8();
        Some(value)
    }

    /// Empty the string without changing capacity.
    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for BoundedString<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> fmt::Display for BoundedString<N> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl<const N: usize> fmt::Debug for BoundedString<N> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.as_str().fmt(formatter)
    }
}

impl<const N: usize> fmt::Write for BoundedString<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        self.try_push_str(value).map_err(|_| fmt::Error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write;

    #[test]
    fn exact_capacity_and_atomic_overflow() {
        let mut value: BoundedString<4> = BoundedString::new();
        assert_eq!(value.try_push_str("rust"), Ok(()));
        assert_eq!(value.try_push('!'), Err(CapacityError));
        assert_eq!(value.as_str(), "rust");
    }

    #[test]
    fn utf8_boundaries_are_preserved() {
        let mut value: BoundedString<5> = BoundedString::new();
        value.try_push('\u{00e9}').unwrap();
        value.try_push('\u{20ac}').unwrap();
        assert_eq!(value.len(), 5);
        assert_eq!(value.pop(), Some('\u{20ac}'));
        assert_eq!(value.as_str(), "\u{00e9}");
    }

    #[test]
    fn zero_capacity_accepts_only_empty_string() {
        let mut value: BoundedString<0> = BoundedString::new();
        assert_eq!(value.try_push_str(""), Ok(()));
        assert_eq!(value.try_push_str("x"), Err(CapacityError));
    }

    #[test]
    fn formatting_respects_capacity() {
        let mut value: BoundedString<8> = BoundedString::new();
        write!(&mut value, "{}:{}", 3, 7).unwrap();
        assert_eq!(value.as_str(), "3:7");
    }
}
