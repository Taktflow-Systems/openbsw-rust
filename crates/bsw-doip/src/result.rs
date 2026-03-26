//! `DoIP` operation result type — a value paired with an optional NACK code.
//!
//! Unlike `std::result::Result`, `DoIpResult` always carries a value even when
//! a NACK is present.  This matches the C++ `OpenBSW` pattern where a response is
//! always sent but may also carry an error code.

use crate::constants::NackCode;

/// A `DoIP` operation result: a `value` plus an optional [`NackCode`].
///
/// `T` must be `Copy` so the struct is always trivially copyable without heap
/// allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpResult<T: Copy> {
    /// The primary result value.
    pub value: T,
    /// Optional NACK code; `None` means the operation succeeded cleanly.
    pub nack: Option<NackCode>,
}

impl<T: Copy> DoIpResult<T> {
    /// Create a successful result (no NACK).
    pub const fn ok(value: T) -> Self {
        Self { value, nack: None }
    }

    /// Create a result that carries a NACK code alongside the value.
    pub const fn with_nack(value: T, nack: NackCode) -> Self {
        Self {
            value,
            nack: Some(nack),
        }
    }

    /// Returns `true` when no NACK is present.
    pub const fn is_ok(&self) -> bool {
        self.nack.is_none()
    }

    /// Returns `true` when a NACK code is present.
    pub const fn has_nack(&self) -> bool {
        self.nack.is_some()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::NackCode;

    #[test]
    fn result_ok() {
        let r = DoIpResult::ok(42u32);
        assert_eq!(r.value, 42);
        assert_eq!(r.nack, None);
    }

    #[test]
    fn result_with_nack() {
        let r = DoIpResult::with_nack(0u8, NackCode::OutOfMemory);
        assert_eq!(r.value, 0);
        assert_eq!(r.nack, Some(NackCode::OutOfMemory));
    }

    #[test]
    fn is_ok_true() {
        let r = DoIpResult::ok(());
        assert!(r.is_ok());
        assert!(!r.has_nack());
    }

    #[test]
    fn has_nack_true() {
        let r = DoIpResult::with_nack(false, NackCode::MessageTooLarge);
        assert!(r.has_nack());
        assert!(!r.is_ok());
    }

    #[test]
    fn result_debug_format() {
        let r = DoIpResult::ok(99u32);
        let s = format!("{r:?}");
        assert!(s.contains("99"));
        assert!(s.contains("None"));
    }
}
