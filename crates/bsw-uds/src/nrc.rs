//! UDS Negative Response Codes (ISO 14229-1 §A.1).

/// UDS Negative Response Codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Nrc {
    GeneralReject = 0x10,
    ServiceNotSupported = 0x11,
    SubFunctionNotSupported = 0x12,
    IncorrectMessageLengthOrInvalidFormat = 0x13,
    ResponseTooLong = 0x14,
    BusyRepeatRequest = 0x21,
    ConditionsNotCorrect = 0x22,
    RequestSequenceError = 0x24,
    NoResponseFromSubnetComponent = 0x25,
    FailurePreventsExecutionOfRequestedAction = 0x26,
    RequestOutOfRange = 0x31,
    SecurityAccessDenied = 0x33,
    AuthenticationRequired = 0x34,
    InvalidKey = 0x35,
    ExceededNumberOfAttempts = 0x36,
    RequiredTimeDelayNotExpired = 0x37,
    SecureDataTransmissionRequired = 0x38,
    SecureDataTransmissionNotAllowed = 0x39,
    SecureDataVerificationFailed = 0x3A,
    UploadDownloadNotAccepted = 0x70,
    TransferDataSuspended = 0x71,
    GeneralProgrammingFailure = 0x72,
    WrongBlockSequenceCounter = 0x73,
    RequestCorrectlyReceivedResponsePending = 0x78,
    SubFunctionNotSupportedInActiveSession = 0x7E,
    ServiceNotSupportedInActiveSession = 0x7F,
}

impl Nrc {
    /// Convert from raw byte. Returns `None` for unrecognized values.
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x10 => Some(Self::GeneralReject),
            0x11 => Some(Self::ServiceNotSupported),
            0x12 => Some(Self::SubFunctionNotSupported),
            0x13 => Some(Self::IncorrectMessageLengthOrInvalidFormat),
            0x14 => Some(Self::ResponseTooLong),
            0x21 => Some(Self::BusyRepeatRequest),
            0x22 => Some(Self::ConditionsNotCorrect),
            0x24 => Some(Self::RequestSequenceError),
            0x25 => Some(Self::NoResponseFromSubnetComponent),
            0x26 => Some(Self::FailurePreventsExecutionOfRequestedAction),
            0x31 => Some(Self::RequestOutOfRange),
            0x33 => Some(Self::SecurityAccessDenied),
            0x34 => Some(Self::AuthenticationRequired),
            0x35 => Some(Self::InvalidKey),
            0x36 => Some(Self::ExceededNumberOfAttempts),
            0x37 => Some(Self::RequiredTimeDelayNotExpired),
            0x38 => Some(Self::SecureDataTransmissionRequired),
            0x39 => Some(Self::SecureDataTransmissionNotAllowed),
            0x3A => Some(Self::SecureDataVerificationFailed),
            0x70 => Some(Self::UploadDownloadNotAccepted),
            0x71 => Some(Self::TransferDataSuspended),
            0x72 => Some(Self::GeneralProgrammingFailure),
            0x73 => Some(Self::WrongBlockSequenceCounter),
            0x78 => Some(Self::RequestCorrectlyReceivedResponsePending),
            0x7E => Some(Self::SubFunctionNotSupportedInActiveSession),
            0x7F => Some(Self::ServiceNotSupportedInActiveSession),
            _ => None,
        }
    }

    /// Convert to raw byte.
    pub const fn as_byte(self) -> u8 {
        self as u8
    }

    /// Whether this NRC indicates the server is busy (client should retry).
    pub const fn is_busy(self) -> bool {
        matches!(
            self,
            Self::BusyRepeatRequest | Self::RequestCorrectlyReceivedResponsePending
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::hash::{Hash, Hasher};

    // Simple hasher for testing Hash impl.
    struct SimpleHasher(u64);

    impl Hasher for SimpleHasher {
        fn finish(&self) -> u64 {
            self.0
        }
        fn write(&mut self, bytes: &[u8]) {
            for &b in bytes {
                self.0 = self.0.wrapping_mul(31).wrapping_add(u64::from(b));
            }
        }
    }

    fn hash_nrc(nrc: Nrc) -> u64 {
        let mut h = SimpleHasher(0);
        nrc.hash(&mut h);
        h.finish()
    }

    #[test]
    fn from_byte_valid() {
        assert_eq!(Nrc::from_byte(0x10), Some(Nrc::GeneralReject));
        assert_eq!(Nrc::from_byte(0x7F), Some(Nrc::ServiceNotSupportedInActiveSession));
        assert_eq!(Nrc::from_byte(0x78), Some(Nrc::RequestCorrectlyReceivedResponsePending));
    }

    #[test]
    fn from_byte_invalid() {
        assert_eq!(Nrc::from_byte(0x00), None);
        assert_eq!(Nrc::from_byte(0x01), None);
        assert_eq!(Nrc::from_byte(0xFF), None);
        assert_eq!(Nrc::from_byte(0x20), None);
        assert_eq!(Nrc::from_byte(0x23), None);
    }

    #[test]
    fn as_byte_roundtrip() {
        let nrc = Nrc::SecurityAccessDenied;
        assert_eq!(Nrc::from_byte(nrc.as_byte()), Some(nrc));
    }

    #[test]
    fn is_busy() {
        assert!(Nrc::BusyRepeatRequest.is_busy());
        assert!(Nrc::RequestCorrectlyReceivedResponsePending.is_busy());
        assert!(!Nrc::GeneralReject.is_busy());
        assert!(!Nrc::ServiceNotSupported.is_busy());
        assert!(!Nrc::ServiceNotSupportedInActiveSession.is_busy());
    }

    #[test]
    fn general_reject_value() {
        assert_eq!(Nrc::GeneralReject.as_byte(), 0x10);
    }

    #[test]
    fn service_not_supported_value() {
        assert_eq!(Nrc::ServiceNotSupported.as_byte(), 0x11);
    }

    #[test]
    fn all_defined_nrcs_roundtrip() {
        let all: &[Nrc] = &[
            Nrc::GeneralReject,
            Nrc::ServiceNotSupported,
            Nrc::SubFunctionNotSupported,
            Nrc::IncorrectMessageLengthOrInvalidFormat,
            Nrc::ResponseTooLong,
            Nrc::BusyRepeatRequest,
            Nrc::ConditionsNotCorrect,
            Nrc::RequestSequenceError,
            Nrc::NoResponseFromSubnetComponent,
            Nrc::FailurePreventsExecutionOfRequestedAction,
            Nrc::RequestOutOfRange,
            Nrc::SecurityAccessDenied,
            Nrc::AuthenticationRequired,
            Nrc::InvalidKey,
            Nrc::ExceededNumberOfAttempts,
            Nrc::RequiredTimeDelayNotExpired,
            Nrc::SecureDataTransmissionRequired,
            Nrc::SecureDataTransmissionNotAllowed,
            Nrc::SecureDataVerificationFailed,
            Nrc::UploadDownloadNotAccepted,
            Nrc::TransferDataSuspended,
            Nrc::GeneralProgrammingFailure,
            Nrc::WrongBlockSequenceCounter,
            Nrc::RequestCorrectlyReceivedResponsePending,
            Nrc::SubFunctionNotSupportedInActiveSession,
            Nrc::ServiceNotSupportedInActiveSession,
        ];
        for &nrc in all {
            assert_eq!(Nrc::from_byte(nrc.as_byte()), Some(nrc));
        }
    }

    #[test]
    fn debug_format() {
        let s = format!("{:?}", Nrc::GeneralReject);
        assert!(s.contains("GeneralReject"));
    }

    #[test]
    fn equality() {
        assert_eq!(Nrc::InvalidKey, Nrc::InvalidKey);
        assert_ne!(Nrc::InvalidKey, Nrc::SecurityAccessDenied);
    }

    #[test]
    fn hash_works() {
        let h1 = hash_nrc(Nrc::GeneralReject);
        let h2 = hash_nrc(Nrc::GeneralReject);
        let h3 = hash_nrc(Nrc::ServiceNotSupported);
        assert_eq!(h1, h2);
        assert_ne!(h1, h3);
    }
}
