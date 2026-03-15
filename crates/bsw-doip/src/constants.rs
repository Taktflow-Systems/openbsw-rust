//! `DoIP` protocol constants — versions, payload types, NACK codes, ports, and field sizes.
//!
//! All values are taken from ISO 13400-2.

// ---------------------------------------------------------------------------
// Protocol Version
// ---------------------------------------------------------------------------

/// `DoIP` protocol version byte (octet 1 of every `DoIP` message).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProtocolVersion {
    /// ISO 13400-2:2012 (version 0x02).
    Iso2012 = 0x02,
    /// ISO 13400-2:2019 (version 0x03).
    Iso2019 = 0x03,
}

impl ProtocolVersion {
    /// Decode a version byte.  Returns `None` for unknown values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x02 => Some(Self::Iso2012),
            0x03 => Some(Self::Iso2019),
            _ => None,
        }
    }

    /// Encode the version as its byte representation.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }

    /// Bitwise inverse of the version byte (used in `DoIP` header validation).
    #[inline]
    pub const fn inverted(self) -> u8 {
        !self.as_byte()
    }
}

// ---------------------------------------------------------------------------
// Payload Type
// ---------------------------------------------------------------------------

/// `DoIP` payload type codes (16-bit big-endian, octets 3–4 of header).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u16)]
pub enum PayloadType {
    /// Generic `DoIP` header negative acknowledge (0x0000).
    NegativeAck = 0x0000,
    /// Vehicle identification request (0x0001).
    VehicleIdentificationRequest = 0x0001,
    /// Vehicle identification request by EID (0x0002).
    VehicleIdentificationRequestByEid = 0x0002,
    /// Vehicle identification request by VIN (0x0003).
    VehicleIdentificationRequestByVin = 0x0003,
    /// Vehicle announcement / identification response (0x0004).
    VehicleAnnouncementMessage = 0x0004,
    /// Routing activation request (0x0005).
    RoutingActivationRequest = 0x0005,
    /// Routing activation response (0x0006).
    RoutingActivationResponse = 0x0006,
    /// Alive check request (0x0007).
    AliveCheckRequest = 0x0007,
    /// Alive check response (0x0008).
    AliveCheckResponse = 0x0008,
    /// `DoIP` entity status request (0x4001).
    EntityStatusRequest = 0x4001,
    /// `DoIP` entity status response (0x4002).
    EntityStatusResponse = 0x4002,
    /// Diagnostic power mode information request (0x4003).
    DiagnosticPowerModeInfoRequest = 0x4003,
    /// Diagnostic power mode information response (0x4004).
    DiagnosticPowerModeInfoResponse = 0x4004,
    /// Diagnostic message (0x8001).
    DiagnosticMessage = 0x8001,
    /// Diagnostic message positive acknowledge (0x8002).
    DiagnosticMessagePositiveAck = 0x8002,
    /// Diagnostic message negative acknowledge (0x8003).
    DiagnosticMessageNegativeAck = 0x8003,
}

impl PayloadType {
    /// Decode a 16-bit value.  Returns `None` for reserved / unknown codes.
    #[inline]
    pub const fn from_u16(value: u16) -> Option<Self> {
        match value {
            0x0000 => Some(Self::NegativeAck),
            0x0001 => Some(Self::VehicleIdentificationRequest),
            0x0002 => Some(Self::VehicleIdentificationRequestByEid),
            0x0003 => Some(Self::VehicleIdentificationRequestByVin),
            0x0004 => Some(Self::VehicleAnnouncementMessage),
            0x0005 => Some(Self::RoutingActivationRequest),
            0x0006 => Some(Self::RoutingActivationResponse),
            0x0007 => Some(Self::AliveCheckRequest),
            0x0008 => Some(Self::AliveCheckResponse),
            0x4001 => Some(Self::EntityStatusRequest),
            0x4002 => Some(Self::EntityStatusResponse),
            0x4003 => Some(Self::DiagnosticPowerModeInfoRequest),
            0x4004 => Some(Self::DiagnosticPowerModeInfoResponse),
            0x8001 => Some(Self::DiagnosticMessage),
            0x8002 => Some(Self::DiagnosticMessagePositiveAck),
            0x8003 => Some(Self::DiagnosticMessageNegativeAck),
            _ => None,
        }
    }

    /// Encode as a 16-bit value.
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self as u16
    }
}

// ---------------------------------------------------------------------------
// Generic NACK Codes
// ---------------------------------------------------------------------------

/// Generic `DoIP` header NACK codes (payload type 0x0000).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NackCode {
    /// Incorrect pattern format (sync byte mismatch) (0x00).
    IncorrectPattern = 0x00,
    /// Unknown payload type (0x01).
    UnknownPayloadType = 0x01,
    /// Message too large (0x02).
    MessageTooLarge = 0x02,
    /// Out of memory (0x03).
    OutOfMemory = 0x03,
    /// Invalid payload length (0x04).
    InvalidPayloadLength = 0x04,
}

impl NackCode {
    /// Decode from a byte.  Returns `None` for unknown values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::IncorrectPattern),
            0x01 => Some(Self::UnknownPayloadType),
            0x02 => Some(Self::MessageTooLarge),
            0x03 => Some(Self::OutOfMemory),
            0x04 => Some(Self::InvalidPayloadLength),
            _ => None,
        }
    }

    /// Encode as a byte.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Diagnostic Message NACK Codes
// ---------------------------------------------------------------------------

/// Diagnostic message NACK codes (payload type 0x8003).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DiagNackCode {
    /// Routing successful (not an error; 0x00).
    Success = 0x00,
    // 0x01 reserved
    /// Invalid source address (0x02).
    InvalidSourceAddress = 0x02,
    /// Invalid target address (0x03).
    InvalidTargetAddress = 0x03,
    /// Diagnostic message too large (0x04).
    DiagnosticMessageTooLarge = 0x04,
    /// Out of memory (0x05).
    OutOfMemory = 0x05,
    /// Target unreachable (0x06).
    TargetUnreachable = 0x06,
    /// Unknown network (0x07).
    UnknownNetwork = 0x07,
    /// Transport protocol error (0x08).
    TransportProtocolError = 0x08,
}

impl DiagNackCode {
    /// Decode from a byte.  Returns `None` for unknown / reserved values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::Success),
            0x02 => Some(Self::InvalidSourceAddress),
            0x03 => Some(Self::InvalidTargetAddress),
            0x04 => Some(Self::DiagnosticMessageTooLarge),
            0x05 => Some(Self::OutOfMemory),
            0x06 => Some(Self::TargetUnreachable),
            0x07 => Some(Self::UnknownNetwork),
            0x08 => Some(Self::TransportProtocolError),
            _ => None,
        }
    }

    /// Encode as a byte.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Routing Activation Codes
// ---------------------------------------------------------------------------

/// Routing activation response codes (Table 23, ISO 13400-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RoutingActivationCode {
    /// Source address unknown (0x00).
    UnknownSourceAddress = 0x00,
    /// No free socket available (0x01).
    NoFreeSocket = 0x01,
    /// Different source address than expected (0x02).
    WrongSourceAddress = 0x02,
    /// Source address already registered (0x03).
    SourceAlreadyRegistered = 0x03,
    /// Missing authentication (0x04).
    MissingAuthentication = 0x04,
    /// Rejected confirmation (0x05).
    RejectedConfirmation = 0x05,
    /// Unsupported activation type (0x06).
    UnsupportedActivationType = 0x06,
    /// Routing successfully activated (0x10).
    Success = 0x10,
    /// Routing activated, confirmation required (0x11).
    ConfirmationRequired = 0x11,
}

impl RoutingActivationCode {
    /// Decode from a byte.  Returns `None` for reserved / unknown values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::UnknownSourceAddress),
            0x01 => Some(Self::NoFreeSocket),
            0x02 => Some(Self::WrongSourceAddress),
            0x03 => Some(Self::SourceAlreadyRegistered),
            0x04 => Some(Self::MissingAuthentication),
            0x05 => Some(Self::RejectedConfirmation),
            0x06 => Some(Self::UnsupportedActivationType),
            0x10 => Some(Self::Success),
            0x11 => Some(Self::ConfirmationRequired),
            _ => None,
        }
    }

    /// Encode as a byte.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Activation Types
// ---------------------------------------------------------------------------

/// Routing activation type codes (Table 22, ISO 13400-2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ActivationType {
    /// Default activation (0x00).
    Default = 0x00,
    /// WWH-OBD activation (0x01).
    WwhObd = 0x01,
    /// Central security activation (0xE0).
    CentralSecurity = 0xE0,
}

impl ActivationType {
    /// Decode from a byte.  Returns `None` for reserved / unknown values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::Default),
            0x01 => Some(Self::WwhObd),
            0xE0 => Some(Self::CentralSecurity),
            _ => None,
        }
    }

    /// Encode as a byte.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Diagnostic Power Mode
// ---------------------------------------------------------------------------

/// Diagnostic power mode codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DiagnosticPowerMode {
    /// Not ready (0x00).
    NotReady = 0x00,
    /// Ready (0x01).
    Ready = 0x01,
    /// Not supported (0x02).
    NotSupported = 0x02,
}

impl DiagnosticPowerMode {
    /// Decode from a byte.  Returns `None` for unknown values.
    #[inline]
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x00 => Some(Self::NotReady),
            0x01 => Some(Self::Ready),
            0x02 => Some(Self::NotSupported),
            _ => None,
        }
    }

    /// Encode as a byte.
    #[inline]
    pub const fn as_byte(self) -> u8 {
        self as u8
    }
}

// ---------------------------------------------------------------------------
// Well-Known Ports
// ---------------------------------------------------------------------------

/// UDP vehicle discovery port (13400).
pub const UDP_DISCOVERY_PORT: u16 = 13400;

/// TCP data channel port (13400).
pub const TCP_DATA_PORT: u16 = 13400;

/// TCP TLS data channel port (3496).
pub const TCP_TLS_PORT: u16 = 3496;

// ---------------------------------------------------------------------------
// Field Sizes
// ---------------------------------------------------------------------------

/// Length of the VIN field in bytes (17 ASCII characters).
pub const VIN_LENGTH: usize = 17;

/// Length of the EID (Entity Identifier / MAC) field in bytes.
pub const EID_LENGTH: usize = 6;

/// Length of the GID (Group Identifier) field in bytes.
pub const GID_LENGTH: usize = 6;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn protocol_version_values() {
        assert_eq!(ProtocolVersion::Iso2012.as_byte(), 0x02);
        assert_eq!(ProtocolVersion::Iso2019.as_byte(), 0x03);
    }

    #[test]
    fn payload_type_from_u16() {
        assert_eq!(PayloadType::from_u16(0x0001), Some(PayloadType::VehicleIdentificationRequest));
        assert_eq!(PayloadType::from_u16(0x8001), Some(PayloadType::DiagnosticMessage));
        assert_eq!(PayloadType::from_u16(0xFFFF), None);
    }

    #[test]
    fn nack_code_values() {
        assert_eq!(NackCode::IncorrectPattern.as_byte(), 0x00);
        assert_eq!(NackCode::InvalidPayloadLength.as_byte(), 0x04);
        assert_eq!(NackCode::from_byte(0x02), Some(NackCode::MessageTooLarge));
        assert_eq!(NackCode::from_byte(0xFF), None);
    }

    #[test]
    fn diag_nack_values() {
        assert_eq!(DiagNackCode::Success.as_byte(), 0x00);
        assert_eq!(DiagNackCode::TransportProtocolError.as_byte(), 0x08);
        assert_eq!(DiagNackCode::from_byte(0x06), Some(DiagNackCode::TargetUnreachable));
        assert_eq!(DiagNackCode::from_byte(0x01), None); // reserved
    }

    #[test]
    fn routing_activation_codes() {
        assert_eq!(RoutingActivationCode::UnknownSourceAddress.as_byte(), 0x00);
        assert_eq!(RoutingActivationCode::ConfirmationRequired.as_byte(), 0x11);
        assert_eq!(
            RoutingActivationCode::from_byte(0x03),
            Some(RoutingActivationCode::SourceAlreadyRegistered)
        );
        assert_eq!(RoutingActivationCode::from_byte(0x07), None); // reserved
    }

    #[test]
    fn activation_types() {
        assert_eq!(ActivationType::Default.as_byte(), 0x00);
        assert_eq!(ActivationType::WwhObd.as_byte(), 0x01);
        assert_eq!(ActivationType::CentralSecurity.as_byte(), 0xE0);
        assert_eq!(ActivationType::from_byte(0x02), None); // reserved
    }

    #[test]
    fn power_mode_values() {
        assert_eq!(DiagnosticPowerMode::NotReady.as_byte(), 0x00);
        assert_eq!(DiagnosticPowerMode::Ready.as_byte(), 0x01);
        assert_eq!(DiagnosticPowerMode::NotSupported.as_byte(), 0x02);
        assert_eq!(DiagnosticPowerMode::from_byte(0x03), None);
    }

    #[test]
    fn port_constants() {
        assert_eq!(UDP_DISCOVERY_PORT, 13400);
        assert_eq!(TCP_DATA_PORT, 13400);
        assert_eq!(TCP_TLS_PORT, 3496);
    }

    #[test]
    fn field_size_constants() {
        assert_eq!(VIN_LENGTH, 17);
        assert_eq!(EID_LENGTH, 6);
        assert_eq!(GID_LENGTH, 6);
    }

    #[test]
    fn protocol_version_inverted() {
        assert_eq!(ProtocolVersion::Iso2012.inverted(), !0x02u8);
        assert_eq!(ProtocolVersion::Iso2019.inverted(), !0x03u8);
    }

    #[test]
    fn payload_type_roundtrip() {
        for (val, expected) in [
            (0x0000u16, PayloadType::NegativeAck),
            (0x0005, PayloadType::RoutingActivationRequest),
            (0x8001, PayloadType::DiagnosticMessage),
            (0x4004, PayloadType::DiagnosticPowerModeInfoResponse),
        ] {
            let pt = PayloadType::from_u16(val).unwrap();
            assert_eq!(pt, expected);
            assert_eq!(pt.as_u16(), val);
        }
    }

    #[test]
    fn all_payload_types_unique() {
        let types = [
            PayloadType::NegativeAck,
            PayloadType::VehicleIdentificationRequest,
            PayloadType::VehicleIdentificationRequestByEid,
            PayloadType::VehicleIdentificationRequestByVin,
            PayloadType::VehicleAnnouncementMessage,
            PayloadType::RoutingActivationRequest,
            PayloadType::RoutingActivationResponse,
            PayloadType::AliveCheckRequest,
            PayloadType::AliveCheckResponse,
            PayloadType::EntityStatusRequest,
            PayloadType::EntityStatusResponse,
            PayloadType::DiagnosticPowerModeInfoRequest,
            PayloadType::DiagnosticPowerModeInfoResponse,
            PayloadType::DiagnosticMessage,
            PayloadType::DiagnosticMessagePositiveAck,
            PayloadType::DiagnosticMessageNegativeAck,
        ];
        // Verify all discriminants are distinct by checking round-trip for each.
        for (i, &a) in types.iter().enumerate() {
            for (j, &b) in types.iter().enumerate() {
                if i != j {
                    assert_ne!(a.as_u16(), b.as_u16(), "duplicate discriminant at indices {i} and {j}");
                }
            }
        }
    }

    #[test]
    fn routing_code_success_is_0x10() {
        assert_eq!(RoutingActivationCode::Success.as_byte(), 0x10);
    }

    #[test]
    fn nack_unknown_type_is_0x01() {
        assert_eq!(NackCode::UnknownPayloadType.as_byte(), 0x01);
    }

    #[test]
    fn diag_nack_success_is_0x00() {
        assert_eq!(DiagNackCode::Success.as_byte(), 0x00);
    }
}
