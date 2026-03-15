//! `DoIP` routing / vehicle-announcement message types — parse and encode helpers.
//!
//! All multi-byte fields are big-endian (network byte order) as required by
//! ISO 13400-2.

use crate::constants::{EID_LENGTH, GID_LENGTH, RoutingActivationCode, VIN_LENGTH};

// ---------------------------------------------------------------------------
// ParseError
// ---------------------------------------------------------------------------

/// Errors returned by message parsers in this module.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// The input slice is shorter than required.
    TooShort {
        /// Minimum number of bytes needed.
        expected: usize,
        /// Number of bytes actually available.
        actual: usize,
    },
    /// The activation-type byte is not a known [`ActivationType`] value.
    InvalidActivationType(u8),
    /// The response-code byte is not a known [`RoutingActivationCode`] value.
    InvalidResponseCode(u8),
}

// ---------------------------------------------------------------------------
// RoutingActivationRequest
// ---------------------------------------------------------------------------

/// Parsed routing activation request payload (follows the 8-byte `DoIP` header).
///
/// Wire layout:
/// ```text
/// Bytes 0–1 : Source address (big-endian u16)
/// Byte  2   : Activation type
/// Bytes 3–6 : Reserved / ISO (4 bytes, must be 0x00000000)
/// Bytes 7–10: OEM-specific (optional, 4 bytes) — present when total length == 11
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutingActivationRequest {
    /// Tester logical address (source).
    pub source_address: u16,
    /// Activation type raw byte (see [`ActivationType`]).
    pub activation_type: u8,
    /// Reserved 4-byte field; `None` indicates the field was absent (should
    /// not occur in normal frames — kept optional for fuzz tolerance).
    pub reserved: Option<[u8; 4]>,
    /// OEM-specific 4-byte field; present only in the 11-byte variant.
    pub oem_specific: Option<[u8; 4]>,
}

impl RoutingActivationRequest {
    /// Minimum payload length (source + type + 4 reserved bytes).
    pub const MIN_PAYLOAD_SIZE: usize = 7;
    /// Maximum payload length (minimum + 4 OEM bytes).
    pub const MAX_PAYLOAD_SIZE: usize = 11;

    /// Parse from a payload byte slice.
    ///
    /// # Errors
    /// Returns [`ParseError::TooShort`] if `data.len() < MIN_PAYLOAD_SIZE`.
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(ParseError::TooShort {
                expected: Self::MIN_PAYLOAD_SIZE,
                actual: data.len(),
            });
        }

        let source_address = u16::from_be_bytes([data[0], data[1]]);
        let activation_type = data[2];
        let reserved = Some([data[3], data[4], data[5], data[6]]);

        let oem_specific = if data.len() >= Self::MAX_PAYLOAD_SIZE {
            Some([data[7], data[8], data[9], data[10]])
        } else {
            None
        };

        Ok(Self {
            source_address,
            activation_type,
            reserved,
            oem_specific,
        })
    }

    /// Encode into `buf`.  Returns the number of bytes written.
    ///
    /// Writes 11 bytes if `oem_specific` is `Some`, otherwise 7 bytes.
    /// The caller must ensure `buf` is large enough.
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let sa = self.source_address.to_be_bytes();
        buf[0] = sa[0];
        buf[1] = sa[1];
        buf[2] = self.activation_type;

        let res = self.reserved.unwrap_or([0u8; 4]);
        buf[3] = res[0];
        buf[4] = res[1];
        buf[5] = res[2];
        buf[6] = res[3];

        if let Some(oem) = self.oem_specific {
            buf[7] = oem[0];
            buf[8] = oem[1];
            buf[9] = oem[2];
            buf[10] = oem[3];
            11
        } else {
            7
        }
    }
}

// ---------------------------------------------------------------------------
// RoutingActivationResponse
// ---------------------------------------------------------------------------

/// Parsed routing activation response payload.
///
/// Wire layout:
/// ```text
/// Bytes 0–1 : Tester logical address echoed (big-endian u16)
/// Bytes 2–3 : Entity logical address (big-endian u16)
/// Byte  4   : Response code
/// Bytes 5–8 : Reserved / ISO (4 bytes)
/// Bytes 9–12: OEM-specific (optional, 4 bytes)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoutingActivationResponse {
    /// Tester logical address echoed from the request.
    pub tester_address: u16,
    /// Entity (server) logical address.
    pub entity_address: u16,
    /// Response code.
    pub response_code: RoutingActivationCode,
    /// Reserved 4-byte field (should be 0x00000000).
    pub reserved: [u8; 4],
    /// OEM-specific 4-byte field; present only in the 13-byte variant.
    pub oem_specific: Option<[u8; 4]>,
}

impl RoutingActivationResponse {
    /// Minimum payload length.
    pub const MIN_PAYLOAD_SIZE: usize = 9;
    /// Maximum payload length (with OEM field).
    pub const MAX_PAYLOAD_SIZE: usize = 13;

    /// Parse from a payload byte slice.
    ///
    /// # Errors
    /// Returns [`ParseError::TooShort`] if `data.len() < MIN_PAYLOAD_SIZE`, or
    /// [`ParseError::InvalidResponseCode`] for an unrecognised code byte.
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(ParseError::TooShort {
                expected: Self::MIN_PAYLOAD_SIZE,
                actual: data.len(),
            });
        }

        let tester_address = u16::from_be_bytes([data[0], data[1]]);
        let entity_address = u16::from_be_bytes([data[2], data[3]]);
        let response_code = RoutingActivationCode::from_byte(data[4])
            .ok_or(ParseError::InvalidResponseCode(data[4]))?;
        let reserved = [data[5], data[6], data[7], data[8]];

        let oem_specific = if data.len() >= Self::MAX_PAYLOAD_SIZE {
            Some([data[9], data[10], data[11], data[12]])
        } else {
            None
        };

        Ok(Self {
            tester_address,
            entity_address,
            response_code,
            reserved,
            oem_specific,
        })
    }

    /// Encode into `buf`.  Returns the number of bytes written (9 or 13).
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let ta = self.tester_address.to_be_bytes();
        let ea = self.entity_address.to_be_bytes();
        buf[0] = ta[0];
        buf[1] = ta[1];
        buf[2] = ea[0];
        buf[3] = ea[1];
        buf[4] = self.response_code.as_byte();
        buf[5] = self.reserved[0];
        buf[6] = self.reserved[1];
        buf[7] = self.reserved[2];
        buf[8] = self.reserved[3];

        if let Some(oem) = self.oem_specific {
            buf[9] = oem[0];
            buf[10] = oem[1];
            buf[11] = oem[2];
            buf[12] = oem[3];
            13
        } else {
            9
        }
    }
}

// ---------------------------------------------------------------------------
// VehicleAnnouncement
// ---------------------------------------------------------------------------

/// Vehicle announcement / identification response payload (payload type 0x0004).
///
/// Wire layout:
/// ```text
/// Bytes  0–16 : VIN (17 ASCII bytes)
/// Bytes 17–18 : Logical address (big-endian u16)
/// Bytes 19–24 : EID — Entity Identifier / MAC (6 bytes)
/// Bytes 25–30 : GID — Group Identifier (6 bytes)
/// Byte  31    : Further action required
/// Byte  32    : VIN/GID sync status (optional)
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VehicleAnnouncement {
    /// Vehicle Identification Number (17 ASCII bytes).
    pub vin: [u8; VIN_LENGTH],
    /// `DoIP` entity logical address.
    pub logical_address: u16,
    /// Entity Identifier (typically the MAC address, 6 bytes).
    pub eid: [u8; EID_LENGTH],
    /// Group Identifier (6 bytes).
    pub gid: [u8; GID_LENGTH],
    /// Further action required byte (0x00 = none).
    pub further_action_required: u8,
    /// Optional VIN/GID sync status byte.
    pub sync_status: Option<u8>,
}

impl VehicleAnnouncement {
    /// Minimum payload length (without sync status).
    pub const MIN_PAYLOAD_SIZE: usize = 32; // 17 + 2 + 6 + 6 + 1
    /// Maximum payload length (with sync status).
    pub const MAX_PAYLOAD_SIZE: usize = 33;

    /// Parse from a payload byte slice.
    ///
    /// # Errors
    /// Returns [`ParseError::TooShort`] if `data.len() < MIN_PAYLOAD_SIZE`.
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < Self::MIN_PAYLOAD_SIZE {
            return Err(ParseError::TooShort {
                expected: Self::MIN_PAYLOAD_SIZE,
                actual: data.len(),
            });
        }

        let mut vin = [0u8; VIN_LENGTH];
        vin.copy_from_slice(&data[0..VIN_LENGTH]);

        let logical_address = u16::from_be_bytes([data[17], data[18]]);

        let mut eid = [0u8; EID_LENGTH];
        eid.copy_from_slice(&data[19..25]);

        let mut gid = [0u8; GID_LENGTH];
        gid.copy_from_slice(&data[25..31]);

        let further_action_required = data[31];

        let sync_status = if data.len() >= Self::MAX_PAYLOAD_SIZE {
            Some(data[32])
        } else {
            None
        };

        Ok(Self {
            vin,
            logical_address,
            eid,
            gid,
            further_action_required,
            sync_status,
        })
    }

    /// Encode into `buf`.  Returns the number of bytes written (32 or 33).
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        buf[0..VIN_LENGTH].copy_from_slice(&self.vin);
        let la = self.logical_address.to_be_bytes();
        buf[17] = la[0];
        buf[18] = la[1];
        buf[19..25].copy_from_slice(&self.eid);
        buf[25..31].copy_from_slice(&self.gid);
        buf[31] = self.further_action_required;

        if let Some(ss) = self.sync_status {
            buf[32] = ss;
            33
        } else {
            32
        }
    }
}

// ---------------------------------------------------------------------------
// DiagnosticMessageHeader
// ---------------------------------------------------------------------------

/// First 4 bytes of a diagnostic message payload: source and target addresses.
///
/// Wire layout:
/// ```text
/// Bytes 0–1 : Source address (big-endian u16)
/// Bytes 2–3 : Target address (big-endian u16)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticMessageHeader {
    /// Tester (source) logical address.
    pub source_address: u16,
    /// ECU (target) logical address.
    pub target_address: u16,
}

impl DiagnosticMessageHeader {
    /// Size of the header on the wire (4 bytes).
    pub const SIZE: usize = 4;

    /// Parse from the first 4 bytes of a diagnostic message payload.
    ///
    /// # Errors
    /// Returns [`ParseError::TooShort`] if `data.len() < 4`.
    pub fn parse(data: &[u8]) -> Result<Self, ParseError> {
        if data.len() < Self::SIZE {
            return Err(ParseError::TooShort {
                expected: Self::SIZE,
                actual: data.len(),
            });
        }
        Ok(Self {
            source_address: u16::from_be_bytes([data[0], data[1]]),
            target_address: u16::from_be_bytes([data[2], data[3]]),
        })
    }

    /// Encode into a fixed 4-byte array.
    pub fn encode(&self) -> [u8; 4] {
        let sa = self.source_address.to_be_bytes();
        let ta = self.target_address.to_be_bytes();
        [sa[0], sa[1], ta[0], ta[1]]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{ActivationType, RoutingActivationCode};

    // ------------------------------------------------------------------
    // RoutingActivationRequest
    // ------------------------------------------------------------------

    #[test]
    fn routing_activation_request_parse_min() {
        // 7 bytes: SA=0x0E00, type=Default, reserved=00000000
        let data: [u8; 7] = [0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let req = RoutingActivationRequest::parse(&data).unwrap();
        assert_eq!(req.source_address, 0x0E00);
        assert_eq!(req.activation_type, ActivationType::Default.as_byte());
        assert_eq!(req.reserved, Some([0u8; 4]));
        assert_eq!(req.oem_specific, None);
    }

    #[test]
    fn routing_activation_request_parse_with_oem() {
        let data: [u8; 11] = [
            0x0E, 0x01, // source = 0x0E01
            0x00,       // Default activation
            0x00, 0x00, 0x00, 0x00, // reserved
            0xDE, 0xAD, 0xBE, 0xEF, // OEM
        ];
        let req = RoutingActivationRequest::parse(&data).unwrap();
        assert_eq!(req.source_address, 0x0E01);
        assert_eq!(req.oem_specific, Some([0xDE, 0xAD, 0xBE, 0xEF]));
    }

    #[test]
    fn routing_activation_request_parse_too_short() {
        let data: [u8; 3] = [0x0E, 0x00, 0x00];
        assert_eq!(
            RoutingActivationRequest::parse(&data),
            Err(ParseError::TooShort { expected: 7, actual: 3 })
        );
    }

    #[test]
    fn routing_activation_request_encode_roundtrip() {
        let req = RoutingActivationRequest {
            source_address: 0x1234,
            activation_type: 0x00,
            reserved: Some([0x00; 4]),
            oem_specific: Some([0x01, 0x02, 0x03, 0x04]),
        };
        let mut buf = [0u8; 11];
        let n = req.encode(&mut buf);
        assert_eq!(n, 11);
        let back = RoutingActivationRequest::parse(&buf[..n]).unwrap();
        assert_eq!(back, req);
    }

    #[test]
    fn routing_activation_request_encode_min_size() {
        let req = RoutingActivationRequest {
            source_address: 0xABCD,
            activation_type: 0x00,
            reserved: None,
            oem_specific: None,
        };
        let mut buf = [0u8; 11];
        let n = req.encode(&mut buf);
        assert_eq!(n, 7);
        // source address big-endian
        assert_eq!(buf[0], 0xAB);
        assert_eq!(buf[1], 0xCD);
    }

    // ------------------------------------------------------------------
    // RoutingActivationResponse
    // ------------------------------------------------------------------

    #[test]
    fn routing_activation_response_parse_success() {
        let data: [u8; 9] = [
            0x0E, 0x00, // tester address
            0xE0, 0x00, // entity address
            0x10,       // Success
            0x00, 0x00, 0x00, 0x00, // reserved
        ];
        let resp = RoutingActivationResponse::parse(&data).unwrap();
        assert_eq!(resp.tester_address, 0x0E00);
        assert_eq!(resp.entity_address, 0xE000);
        assert_eq!(resp.response_code, RoutingActivationCode::Success);
        assert_eq!(resp.oem_specific, None);
    }

    #[test]
    fn routing_activation_response_parse_with_oem() {
        let data: [u8; 13] = [
            0x0E, 0x00, 0xE0, 0x00,
            0x10,
            0x00, 0x00, 0x00, 0x00,
            0xCA, 0xFE, 0xBA, 0xBE,
        ];
        let resp = RoutingActivationResponse::parse(&data).unwrap();
        assert_eq!(resp.oem_specific, Some([0xCA, 0xFE, 0xBA, 0xBE]));
    }

    #[test]
    fn routing_activation_response_encode_roundtrip() {
        let resp = RoutingActivationResponse {
            tester_address: 0x1000,
            entity_address: 0x2000,
            response_code: RoutingActivationCode::Success,
            reserved: [0u8; 4],
            oem_specific: None,
        };
        let mut buf = [0u8; 13];
        let n = resp.encode(&mut buf);
        assert_eq!(n, 9);
        let back = RoutingActivationResponse::parse(&buf[..n]).unwrap();
        assert_eq!(back.tester_address, resp.tester_address);
        assert_eq!(back.response_code, resp.response_code);
    }

    // ------------------------------------------------------------------
    // VehicleAnnouncement
    // ------------------------------------------------------------------

    fn make_vehicle_announcement_bytes(with_sync: bool) -> Vec<u8> {
        let mut v = Vec::new();
        // VIN: 17 bytes 'A'
        v.extend_from_slice(&[b'A'; 17]);
        // logical address 0x0E80
        v.push(0x0E);
        v.push(0x80);
        // EID: 6 bytes
        v.extend_from_slice(&[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        // GID: 6 bytes
        v.extend_from_slice(&[0x10, 0x20, 0x30, 0x40, 0x50, 0x60]);
        // further action
        v.push(0x00);
        if with_sync {
            v.push(0x10);
        }
        v
    }

    #[test]
    fn vehicle_announcement_parse_min() {
        let data = make_vehicle_announcement_bytes(false);
        let va = VehicleAnnouncement::parse(&data).unwrap();
        assert_eq!(va.vin, [b'A'; 17]);
        assert_eq!(va.logical_address, 0x0E80);
        assert_eq!(va.eid, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(va.gid, [0x10, 0x20, 0x30, 0x40, 0x50, 0x60]);
        assert_eq!(va.further_action_required, 0x00);
        assert_eq!(va.sync_status, None);
    }

    #[test]
    fn vehicle_announcement_parse_with_sync() {
        let data = make_vehicle_announcement_bytes(true);
        let va = VehicleAnnouncement::parse(&data).unwrap();
        assert_eq!(va.sync_status, Some(0x10));
    }

    #[test]
    fn vehicle_announcement_parse_too_short() {
        let short = [0u8; 10];
        assert_eq!(
            VehicleAnnouncement::parse(&short),
            Err(ParseError::TooShort { expected: 32, actual: 10 })
        );
    }

    #[test]
    fn vehicle_announcement_encode_roundtrip() {
        let data = make_vehicle_announcement_bytes(true);
        let va = VehicleAnnouncement::parse(&data).unwrap();
        let mut buf = [0u8; 33];
        let n = va.encode(&mut buf);
        assert_eq!(n, 33);
        let back = VehicleAnnouncement::parse(&buf[..n]).unwrap();
        assert_eq!(va, back);
    }

    #[test]
    fn vin_length_17() {
        assert_eq!(VIN_LENGTH, 17);
        let data = make_vehicle_announcement_bytes(false);
        let va = VehicleAnnouncement::parse(&data).unwrap();
        assert_eq!(va.vin.len(), 17);
    }

    // ------------------------------------------------------------------
    // DiagnosticMessageHeader
    // ------------------------------------------------------------------

    #[test]
    fn diagnostic_message_header_parse_valid() {
        let data = [0x0E, 0x00, 0x10, 0x01];
        let hdr = DiagnosticMessageHeader::parse(&data).unwrap();
        assert_eq!(hdr.source_address, 0x0E00);
        assert_eq!(hdr.target_address, 0x1001);
    }

    #[test]
    fn diagnostic_message_header_parse_too_short() {
        let data = [0x0E, 0x00];
        assert_eq!(
            DiagnosticMessageHeader::parse(&data),
            Err(ParseError::TooShort { expected: 4, actual: 2 })
        );
    }

    #[test]
    fn diagnostic_message_header_encode_roundtrip() {
        let hdr = DiagnosticMessageHeader {
            source_address: 0xABCD,
            target_address: 0x1234,
        };
        let encoded = hdr.encode();
        assert_eq!(encoded, [0xAB, 0xCD, 0x12, 0x34]);
        let back = DiagnosticMessageHeader::parse(&encoded).unwrap();
        assert_eq!(back, hdr);
    }

    #[test]
    fn diagnostic_message_header_size_is_4() {
        assert_eq!(DiagnosticMessageHeader::SIZE, 4);
    }

    // ------------------------------------------------------------------
    // ParseError
    // ------------------------------------------------------------------

    #[test]
    fn too_short_error() {
        let err = ParseError::TooShort { expected: 7, actual: 3 };
        match err {
            ParseError::TooShort { expected, actual } => {
                assert_eq!(expected, 7);
                assert_eq!(actual, 3);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn parse_error_debug_format() {
        let err = ParseError::InvalidActivationType(0xFF);
        let s = format!("{err:?}");
        assert!(s.contains("InvalidActivationType"));
        assert!(s.contains("255"));
    }
}
