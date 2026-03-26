//! `DoIP` message header — 8-byte framing header that precedes every `DoIP` payload.
//!
//! Wire format (Table 2, ISO 13400-2):
//! ```text
//! Octet 1 : Protocol version
//! Octet 2 : Inverse protocol version (~version)
//! Octet 3–4: Payload type (big-endian u16)
//! Octet 5–8: Payload length (big-endian u32)
//! ```

use crate::constants::{PayloadType, ProtocolVersion};

// ---------------------------------------------------------------------------
// Header size constant
// ---------------------------------------------------------------------------

/// Size of the `DoIP` generic header in bytes.
pub const HEADER_SIZE: usize = 8;

// ---------------------------------------------------------------------------
// DoIpHeader
// ---------------------------------------------------------------------------

/// Parsed `DoIP` generic header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoIpHeader {
    /// Protocol version extracted from octet 1.
    pub version: ProtocolVersion,
    /// Payload type extracted from octets 3–4.
    pub payload_type: PayloadType,
    /// Payload length in bytes extracted from octets 5–8.
    pub payload_length: u32,
}

impl DoIpHeader {
    /// Size of the header on the wire.
    pub const SIZE: usize = HEADER_SIZE;

    /// Parse a `DoIP` header from exactly 8 bytes.
    ///
    /// # Errors
    /// Returns `HeaderError` if the version byte is unknown, the inverse-version
    /// byte does not match, or the payload type is unrecognised.
    pub fn parse(data: &[u8; 8]) -> Result<Self, HeaderError> {
        // Octet 1: version
        let version = ProtocolVersion::from_byte(data[0])
            .ok_or(HeaderError::InvalidVersion(data[0]))?;

        // Octet 2: ~version
        if data[1] != version.inverted() {
            return Err(HeaderError::VersionMismatch {
                version: data[0],
                inverted: data[1],
            });
        }

        // Octets 3–4: payload type (big-endian)
        let payload_type_raw = u16::from_be_bytes([data[2], data[3]]);
        let payload_type = PayloadType::from_u16(payload_type_raw)
            .ok_or(HeaderError::UnknownPayloadType(payload_type_raw))?;

        // Octets 5–8: payload length (big-endian)
        let payload_length = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);

        Ok(Self {
            version,
            payload_type,
            payload_length,
        })
    }

    /// Encode the header into 8 bytes.
    pub fn encode(&self) -> [u8; 8] {
        let pt = self.payload_type.as_u16().to_be_bytes();
        let pl = self.payload_length.to_be_bytes();
        [
            self.version.as_byte(),
            self.version.inverted(),
            pt[0],
            pt[1],
            pl[0],
            pl[1],
            pl[2],
            pl[3],
        ]
    }

    /// Total message size on the wire: header bytes plus payload bytes.
    ///
    /// Returned as `u64` to avoid overflow when `payload_length` is near `u32::MAX`.
    pub const fn total_size(&self) -> u64 {
        Self::SIZE as u64 + self.payload_length as u64
    }
}

// ---------------------------------------------------------------------------
// HeaderError
// ---------------------------------------------------------------------------

/// Errors that can occur while parsing a `DoIP` header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeaderError {
    /// Octet 1 is not a known `ProtocolVersion` value.
    InvalidVersion(u8),
    /// Octet 2 is not the bitwise inverse of octet 1.
    VersionMismatch {
        /// Raw version byte (octet 1).
        version: u8,
        /// Raw inverted byte (octet 2) as received.
        inverted: u8,
    },
    /// Octets 3–4 do not match any defined `PayloadType`.
    UnknownPayloadType(u16),
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{PayloadType, ProtocolVersion};

    /// Build a valid 8-byte header array from components.
    fn make_header(version: u8, pt: u16, len: u32) -> [u8; 8] {
        let pt_bytes = pt.to_be_bytes();
        let len_bytes = len.to_be_bytes();
        [
            version,
            !version,
            pt_bytes[0],
            pt_bytes[1],
            len_bytes[0],
            len_bytes[1],
            len_bytes[2],
            len_bytes[3],
        ]
    }

    #[test]
    fn parse_valid_header() {
        let raw = make_header(0x02, 0x0005, 7);
        let hdr = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(hdr.version, ProtocolVersion::Iso2012);
        assert_eq!(hdr.payload_type, PayloadType::RoutingActivationRequest);
        assert_eq!(hdr.payload_length, 7);
    }

    #[test]
    fn parse_invalid_version() {
        let raw = make_header(0xFF, 0x0005, 7);
        assert_eq!(DoIpHeader::parse(&raw), Err(HeaderError::InvalidVersion(0xFF)));
    }

    #[test]
    fn parse_version_mismatch() {
        // Octet 2 should be !0x02 = 0xFD, but we put 0x00 instead.
        let raw = [0x02u8, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x07];
        assert_eq!(
            DoIpHeader::parse(&raw),
            Err(HeaderError::VersionMismatch { version: 0x02, inverted: 0x00 })
        );
    }

    #[test]
    fn parse_unknown_payload_type() {
        let raw = make_header(0x02, 0x0099, 0);
        assert_eq!(DoIpHeader::parse(&raw), Err(HeaderError::UnknownPayloadType(0x0099)));
    }

    #[test]
    fn encode_roundtrip() {
        let hdr = DoIpHeader {
            version: ProtocolVersion::Iso2019,
            payload_type: PayloadType::DiagnosticMessage,
            payload_length: 1234,
        };
        let encoded = hdr.encode();
        let decoded = DoIpHeader::parse(&encoded).unwrap();
        assert_eq!(hdr, decoded);
    }

    #[test]
    fn encode_ack_header() {
        let hdr = DoIpHeader {
            version: ProtocolVersion::Iso2012,
            payload_type: PayloadType::NegativeAck,
            payload_length: 1,
        };
        let encoded = hdr.encode();
        assert_eq!(encoded[0], 0x02);
        assert_eq!(encoded[1], !0x02u8);
        assert_eq!(encoded[2], 0x00);
        assert_eq!(encoded[3], 0x00);
        assert_eq!(&encoded[4..], &[0x00, 0x00, 0x00, 0x01]);
    }

    #[test]
    fn encode_diagnostic_message() {
        let hdr = DoIpHeader {
            version: ProtocolVersion::Iso2019,
            payload_type: PayloadType::DiagnosticMessage,
            payload_length: 0x0000_0100,
        };
        let encoded = hdr.encode();
        // payload type 0x8001
        assert_eq!(encoded[2], 0x80);
        assert_eq!(encoded[3], 0x01);
        // payload length big-endian
        assert_eq!(&encoded[4..], &[0x00, 0x00, 0x01, 0x00]);
    }

    #[test]
    fn total_size_calculation() {
        let hdr = DoIpHeader {
            version: ProtocolVersion::Iso2012,
            payload_type: PayloadType::AliveCheckRequest,
            payload_length: 0,
        };
        assert_eq!(hdr.total_size(), 8);

        let hdr2 = DoIpHeader {
            version: ProtocolVersion::Iso2012,
            payload_type: PayloadType::DiagnosticMessage,
            payload_length: 100,
        };
        assert_eq!(hdr2.total_size(), 108);
    }

    #[test]
    fn header_size_is_8() {
        assert_eq!(HEADER_SIZE, 8);
        assert_eq!(DoIpHeader::SIZE, 8);
    }

    #[test]
    fn parse_vehicle_id_request() {
        let raw = make_header(0x02, 0x0001, 0);
        let hdr = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(hdr.payload_type, PayloadType::VehicleIdentificationRequest);
        assert_eq!(hdr.payload_length, 0);
    }

    #[test]
    fn parse_routing_activation() {
        let raw = make_header(0x03, 0x0005, 11);
        let hdr = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(hdr.version, ProtocolVersion::Iso2019);
        assert_eq!(hdr.payload_type, PayloadType::RoutingActivationRequest);
        assert_eq!(hdr.payload_length, 11);
    }

    #[test]
    fn parse_alive_check() {
        let raw = make_header(0x02, 0x0007, 0);
        let hdr = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(hdr.payload_type, PayloadType::AliveCheckRequest);
    }

    #[test]
    fn encode_negative_ack() {
        let hdr = DoIpHeader {
            version: ProtocolVersion::Iso2012,
            payload_type: PayloadType::NegativeAck,
            payload_length: 1,
        };
        let raw = hdr.encode();
        let back = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(back.payload_type, PayloadType::NegativeAck);
    }

    #[test]
    fn parse_all_payload_types_roundtrip() {
        let types = [
            0x0000u16, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x4001,
            0x4002, 0x4003, 0x4004, 0x8001, 0x8002, 0x8003,
        ];
        for &pt_val in &types {
            let raw = make_header(0x02, pt_val, 0);
            let hdr = DoIpHeader::parse(&raw).unwrap();
            assert_eq!(hdr.payload_type.as_u16(), pt_val);
        }
    }

    #[test]
    fn parse_zero_length_payload() {
        let raw = make_header(0x02, 0x0007, 0);
        let hdr = DoIpHeader::parse(&raw).unwrap();
        assert_eq!(hdr.payload_length, 0);
        assert_eq!(hdr.total_size(), 8);
    }
}
