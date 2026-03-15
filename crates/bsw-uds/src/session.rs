//! Diagnostic session management (ISO 14229 §9.2).

/// Diagnostic session types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum DiagSession {
    Default = 0x01,
    Programming = 0x02,
    Extended = 0x03,
}

/// Bitmask for session-based access control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SessionMask(u8);

impl SessionMask {
    pub const NONE: Self = Self(0);
    pub const DEFAULT: Self = Self(1 << 0);
    pub const PROGRAMMING: Self = Self(1 << 1);
    pub const EXTENDED: Self = Self(1 << 2);
    pub const ALL: Self = Self(0x07);
    pub const DEFAULT_AND_EXTENDED: Self = Self(Self::DEFAULT.0 | Self::EXTENDED.0);

    /// Create a mask from raw bits.
    pub const fn new(bits: u8) -> Self {
        Self(bits)
    }

    /// Returns `true` if the given session is included in this mask.
    pub const fn contains(self, session: DiagSession) -> bool {
        let bit = session.to_mask().0;
        self.0 & bit != 0
    }

    /// Bitwise OR of two masks.
    #[must_use]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Bitwise AND of two masks.
    #[must_use]
    pub const fn intersection(self, other: Self) -> Self {
        Self(self.0 & other.0)
    }

    /// Returns `true` if no sessions are set.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

impl DiagSession {
    /// Convert from raw byte. Returns `None` for unrecognized values.
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x01 => Some(Self::Default),
            0x02 => Some(Self::Programming),
            0x03 => Some(Self::Extended),
            _ => None,
        }
    }

    /// Convert to raw byte.
    pub const fn as_byte(self) -> u8 {
        self as u8
    }

    /// Return the single-bit `SessionMask` for this session.
    pub const fn to_mask(self) -> SessionMask {
        match self {
            Self::Default => SessionMask::DEFAULT,
            Self::Programming => SessionMask::PROGRAMMING,
            Self::Extended => SessionMask::EXTENDED,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_session() {
        assert_eq!(DiagSession::Default.as_byte(), 0x01);
    }

    #[test]
    fn session_from_byte() {
        assert_eq!(DiagSession::from_byte(0x01), Some(DiagSession::Default));
        assert_eq!(DiagSession::from_byte(0x02), Some(DiagSession::Programming));
        assert_eq!(DiagSession::from_byte(0x03), Some(DiagSession::Extended));
        assert_eq!(DiagSession::from_byte(0x00), None);
        assert_eq!(DiagSession::from_byte(0x04), None);
        assert_eq!(DiagSession::from_byte(0xFF), None);
    }

    #[test]
    fn session_mask_contains() {
        assert!(SessionMask::ALL.contains(DiagSession::Default));
        assert!(SessionMask::ALL.contains(DiagSession::Programming));
        assert!(SessionMask::ALL.contains(DiagSession::Extended));
        assert!(!SessionMask::NONE.contains(DiagSession::Default));
        assert!(SessionMask::DEFAULT.contains(DiagSession::Default));
        assert!(!SessionMask::DEFAULT.contains(DiagSession::Programming));
    }

    #[test]
    fn session_mask_all() {
        assert!(SessionMask::ALL.contains(DiagSession::Default));
        assert!(SessionMask::ALL.contains(DiagSession::Programming));
        assert!(SessionMask::ALL.contains(DiagSession::Extended));
        assert!(!SessionMask::ALL.is_empty());
    }

    #[test]
    fn session_mask_union() {
        let mask = SessionMask::DEFAULT.union(SessionMask::PROGRAMMING);
        assert!(mask.contains(DiagSession::Default));
        assert!(mask.contains(DiagSession::Programming));
        assert!(!mask.contains(DiagSession::Extended));
    }

    #[test]
    fn session_mask_intersection() {
        let a = SessionMask::DEFAULT.union(SessionMask::EXTENDED);
        let b = SessionMask::DEFAULT.union(SessionMask::PROGRAMMING);
        let result = a.intersection(b);
        assert!(result.contains(DiagSession::Default));
        assert!(!result.contains(DiagSession::Programming));
        assert!(!result.contains(DiagSession::Extended));
    }

    #[test]
    fn session_mask_none_is_empty() {
        assert!(SessionMask::NONE.is_empty());
        assert!(!SessionMask::DEFAULT.is_empty());
        assert!(!SessionMask::ALL.is_empty());
    }

    #[test]
    fn default_and_extended() {
        assert!(SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Default));
        assert!(SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Extended));
        assert!(!SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Programming));
    }

    #[test]
    fn programming_not_in_default_extended() {
        assert!(!SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Programming));
    }

    #[test]
    fn to_mask_roundtrip() {
        for session in [DiagSession::Default, DiagSession::Programming, DiagSession::Extended] {
            let mask = session.to_mask();
            assert!(mask.contains(session));
            // Each mask has exactly one bit set — no other session is contained.
            for other in [DiagSession::Default, DiagSession::Programming, DiagSession::Extended] {
                if other == session {
                    assert!(mask.contains(other));
                } else {
                    assert!(!mask.contains(other));
                }
            }
        }
    }

    #[test]
    fn session_byte_values() {
        assert_eq!(DiagSession::Default.as_byte(), 0x01);
        assert_eq!(DiagSession::Programming.as_byte(), 0x02);
        assert_eq!(DiagSession::Extended.as_byte(), 0x03);
    }

    #[test]
    fn mask_bitwise_operations() {
        let empty = SessionMask::NONE;
        let all = SessionMask::ALL;
        assert_eq!(empty.union(all), all);
        assert_eq!(all.intersection(empty), empty);
        assert_eq!(all.union(empty), all);
        let custom = SessionMask::new(0b101);
        assert!(custom.contains(DiagSession::Default));
        assert!(!custom.contains(DiagSession::Programming));
        assert!(custom.contains(DiagSession::Extended));
    }
}
