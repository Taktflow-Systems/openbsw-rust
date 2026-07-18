//! Bounded, allocation-free codec for the standard DoIP payload set (E26).

use crate::{
    DiagnosticPowerMode, DoIpHeader, HeaderError, NackCode, PayloadType, ProtocolVersion,
    RoutingActivationRequest, RoutingActivationResponse, VehicleAnnouncement, EID_LENGTH,
    HEADER_SIZE, VIN_LENGTH,
};

/// Vehicle-identification request selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleIdentification {
    /// Identify every entity.
    All,
    /// Identify one entity identifier.
    Eid([u8; EID_LENGTH]),
    /// Identify one VIN.
    Vin([u8; VIN_LENGTH]),
}

/// Entity-status response fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntityStatus {
    /// 0 gateway, 1 node.
    pub node_type: u8,
    /// Maximum concurrent TCP data sockets.
    pub max_sockets: u8,
    /// Currently open TCP data sockets.
    pub open_sockets: u8,
    /// Optional maximum diagnostic payload size.
    pub max_data_size: Option<u32>,
}

/// Diagnostic payload view shared by messages and acknowledgements.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticPayload<'a> {
    /// Source logical address.
    pub source_address: u16,
    /// Target logical address.
    pub target_address: u16,
    /// Diagnostic application bytes.
    pub data: &'a [u8],
}

/// Diagnostic acknowledgement payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticAck<'a> {
    /// Source logical address.
    pub source_address: u16,
    /// Target logical address.
    pub target_address: u16,
    /// Positive ACK or diagnostic NACK code.
    pub code: u8,
    /// Optional previous diagnostic-message bytes.
    pub previous_data: &'a [u8],
}

/// Typed DoIP payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Payload<'a> {
    GenericNack(NackCode),
    VehicleIdentification(VehicleIdentification),
    VehicleAnnouncement(VehicleAnnouncement),
    RoutingActivationRequest(RoutingActivationRequest),
    RoutingActivationResponse(RoutingActivationResponse),
    AliveCheckRequest,
    AliveCheckResponse(u16),
    EntityStatusRequest,
    EntityStatusResponse(EntityStatus),
    PowerModeRequest,
    PowerModeResponse(DiagnosticPowerMode),
    DiagnosticMessage(DiagnosticPayload<'a>),
    DiagnosticPositiveAck(DiagnosticAck<'a>),
    DiagnosticNegativeAck(DiagnosticAck<'a>),
}

/// Parsed packet including its protocol version.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet<'a> {
    pub version: ProtocolVersion,
    pub payload: Payload<'a>,
}

/// Packet codec failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecError {
    Header(HeaderError),
    TooShort,
    LengthMismatch,
    InvalidPayload,
    OutputTooSmall,
}

impl From<HeaderError> for CodecError {
    fn from(value: HeaderError) -> Self {
        Self::Header(value)
    }
}

impl<'a> Packet<'a> {
    /// Parse a complete datagram/frame and reject trailing or truncated data.
    pub fn parse(data: &'a [u8]) -> Result<Self, CodecError> {
        let header_bytes: &[u8; HEADER_SIZE] = data
            .get(..HEADER_SIZE)
            .ok_or(CodecError::TooShort)?
            .try_into()
            .map_err(|_| CodecError::TooShort)?;
        let header = DoIpHeader::parse(header_bytes)?;
        let payload_length =
            usize::try_from(header.payload_length).map_err(|_| CodecError::LengthMismatch)?;
        if data.len()
            != HEADER_SIZE
                .checked_add(payload_length)
                .ok_or(CodecError::LengthMismatch)?
        {
            return Err(CodecError::LengthMismatch);
        }
        let bytes = &data[HEADER_SIZE..];
        let payload = parse_payload(header.payload_type, bytes)?;
        Ok(Self {
            version: header.version,
            payload,
        })
    }

    /// Encode a complete packet into caller-owned storage.
    pub fn encode(&self, output: &mut [u8]) -> Result<usize, CodecError> {
        if output.len() < HEADER_SIZE {
            return Err(CodecError::OutputTooSmall);
        }
        let (payload_type, payload_length) = payload_metadata(&self.payload);
        let total = HEADER_SIZE
            .checked_add(payload_length)
            .ok_or(CodecError::OutputTooSmall)?;
        if output.len() < total {
            return Err(CodecError::OutputTooSmall);
        }
        let header = DoIpHeader {
            version: self.version,
            payload_type,
            payload_length: u32::try_from(payload_length)
                .map_err(|_| CodecError::OutputTooSmall)?,
        };
        output[..HEADER_SIZE].copy_from_slice(&header.encode());
        encode_payload(&self.payload, &mut output[HEADER_SIZE..total]);
        Ok(total)
    }
}

fn parse_payload(payload_type: PayloadType, data: &[u8]) -> Result<Payload<'_>, CodecError> {
    match payload_type {
        PayloadType::NegativeAck => exact(data, 1).and_then(|()| {
            NackCode::from_byte(data[0])
                .map(Payload::GenericNack)
                .ok_or(CodecError::InvalidPayload)
        }),
        PayloadType::VehicleIdentificationRequest => {
            exact(data, 0).map(|()| Payload::VehicleIdentification(VehicleIdentification::All))
        }
        PayloadType::VehicleIdentificationRequestByEid => {
            exact(data, EID_LENGTH)?;
            let mut eid = [0; EID_LENGTH];
            eid.copy_from_slice(data);
            Ok(Payload::VehicleIdentification(VehicleIdentification::Eid(
                eid,
            )))
        }
        PayloadType::VehicleIdentificationRequestByVin => {
            exact(data, VIN_LENGTH)?;
            let mut vin = [0; VIN_LENGTH];
            vin.copy_from_slice(data);
            Ok(Payload::VehicleIdentification(VehicleIdentification::Vin(
                vin,
            )))
        }
        PayloadType::VehicleAnnouncementMessage => {
            if !matches!(data.len(), 32 | 33) {
                return Err(CodecError::InvalidPayload);
            }
            VehicleAnnouncement::parse(data)
                .map(Payload::VehicleAnnouncement)
                .map_err(|_| CodecError::InvalidPayload)
        }
        PayloadType::RoutingActivationRequest => {
            if !matches!(data.len(), 7 | 11) {
                return Err(CodecError::InvalidPayload);
            }
            RoutingActivationRequest::parse(data)
                .map(Payload::RoutingActivationRequest)
                .map_err(|_| CodecError::InvalidPayload)
        }
        PayloadType::RoutingActivationResponse => {
            if !matches!(data.len(), 9 | 13) {
                return Err(CodecError::InvalidPayload);
            }
            RoutingActivationResponse::parse(data)
                .map(Payload::RoutingActivationResponse)
                .map_err(|_| CodecError::InvalidPayload)
        }
        PayloadType::AliveCheckRequest => exact(data, 0).map(|()| Payload::AliveCheckRequest),
        PayloadType::AliveCheckResponse => exact(data, 2)
            .map(|()| Payload::AliveCheckResponse(u16::from_be_bytes([data[0], data[1]]))),
        PayloadType::EntityStatusRequest => exact(data, 0).map(|()| Payload::EntityStatusRequest),
        PayloadType::EntityStatusResponse => {
            if !matches!(data.len(), 3 | 7) || data[0] > 1 || data[2] > data[1] {
                return Err(CodecError::InvalidPayload);
            }
            Ok(Payload::EntityStatusResponse(EntityStatus {
                node_type: data[0],
                max_sockets: data[1],
                open_sockets: data[2],
                max_data_size: (data.len() == 7)
                    .then(|| u32::from_be_bytes([data[3], data[4], data[5], data[6]])),
            }))
        }
        PayloadType::DiagnosticPowerModeInfoRequest => {
            exact(data, 0).map(|()| Payload::PowerModeRequest)
        }
        PayloadType::DiagnosticPowerModeInfoResponse => exact(data, 1).and_then(|()| {
            DiagnosticPowerMode::from_byte(data[0])
                .map(Payload::PowerModeResponse)
                .ok_or(CodecError::InvalidPayload)
        }),
        PayloadType::DiagnosticMessage => parse_diagnostic(data).map(Payload::DiagnosticMessage),
        PayloadType::DiagnosticMessagePositiveAck => {
            parse_ack(data).map(Payload::DiagnosticPositiveAck)
        }
        PayloadType::DiagnosticMessageNegativeAck => {
            parse_ack(data).map(Payload::DiagnosticNegativeAck)
        }
    }
}

fn parse_diagnostic(data: &[u8]) -> Result<DiagnosticPayload<'_>, CodecError> {
    if data.len() < 4 {
        return Err(CodecError::InvalidPayload);
    }
    Ok(DiagnosticPayload {
        source_address: u16::from_be_bytes([data[0], data[1]]),
        target_address: u16::from_be_bytes([data[2], data[3]]),
        data: &data[4..],
    })
}

fn parse_ack(data: &[u8]) -> Result<DiagnosticAck<'_>, CodecError> {
    if data.len() < 5 {
        return Err(CodecError::InvalidPayload);
    }
    Ok(DiagnosticAck {
        source_address: u16::from_be_bytes([data[0], data[1]]),
        target_address: u16::from_be_bytes([data[2], data[3]]),
        code: data[4],
        previous_data: &data[5..],
    })
}

fn exact(data: &[u8], length: usize) -> Result<(), CodecError> {
    if data.len() == length {
        Ok(())
    } else {
        Err(CodecError::InvalidPayload)
    }
}

fn payload_metadata(payload: &Payload<'_>) -> (PayloadType, usize) {
    match payload {
        Payload::GenericNack(_) => (PayloadType::NegativeAck, 1),
        Payload::VehicleIdentification(VehicleIdentification::All) => {
            (PayloadType::VehicleIdentificationRequest, 0)
        }
        Payload::VehicleIdentification(VehicleIdentification::Eid(_)) => {
            (PayloadType::VehicleIdentificationRequestByEid, EID_LENGTH)
        }
        Payload::VehicleIdentification(VehicleIdentification::Vin(_)) => {
            (PayloadType::VehicleIdentificationRequestByVin, VIN_LENGTH)
        }
        Payload::VehicleAnnouncement(value) => (
            PayloadType::VehicleAnnouncementMessage,
            if value.sync_status.is_some() { 33 } else { 32 },
        ),
        Payload::RoutingActivationRequest(value) => (
            PayloadType::RoutingActivationRequest,
            if value.oem_specific.is_some() { 11 } else { 7 },
        ),
        Payload::RoutingActivationResponse(value) => (
            PayloadType::RoutingActivationResponse,
            if value.oem_specific.is_some() { 13 } else { 9 },
        ),
        Payload::AliveCheckRequest => (PayloadType::AliveCheckRequest, 0),
        Payload::AliveCheckResponse(_) => (PayloadType::AliveCheckResponse, 2),
        Payload::EntityStatusRequest => (PayloadType::EntityStatusRequest, 0),
        Payload::EntityStatusResponse(value) => (
            PayloadType::EntityStatusResponse,
            if value.max_data_size.is_some() { 7 } else { 3 },
        ),
        Payload::PowerModeRequest => (PayloadType::DiagnosticPowerModeInfoRequest, 0),
        Payload::PowerModeResponse(_) => (PayloadType::DiagnosticPowerModeInfoResponse, 1),
        Payload::DiagnosticMessage(value) => (PayloadType::DiagnosticMessage, 4 + value.data.len()),
        Payload::DiagnosticPositiveAck(value) => (
            PayloadType::DiagnosticMessagePositiveAck,
            5 + value.previous_data.len(),
        ),
        Payload::DiagnosticNegativeAck(value) => (
            PayloadType::DiagnosticMessageNegativeAck,
            5 + value.previous_data.len(),
        ),
    }
}

fn encode_payload(payload: &Payload<'_>, output: &mut [u8]) {
    match payload {
        Payload::GenericNack(value) => output[0] = value.as_byte(),
        Payload::VehicleIdentification(VehicleIdentification::All)
        | Payload::AliveCheckRequest
        | Payload::EntityStatusRequest
        | Payload::PowerModeRequest => {}
        Payload::VehicleIdentification(VehicleIdentification::Eid(value)) => {
            output.copy_from_slice(value);
        }
        Payload::VehicleIdentification(VehicleIdentification::Vin(value)) => {
            output.copy_from_slice(value);
        }
        Payload::VehicleAnnouncement(value) => {
            value.encode(output);
        }
        Payload::RoutingActivationRequest(value) => {
            value.encode(output);
        }
        Payload::RoutingActivationResponse(value) => {
            value.encode(output);
        }
        Payload::AliveCheckResponse(value) => output.copy_from_slice(&value.to_be_bytes()),
        Payload::EntityStatusResponse(value) => {
            output[..3].copy_from_slice(&[value.node_type, value.max_sockets, value.open_sockets]);
            if let Some(size) = value.max_data_size {
                output[3..7].copy_from_slice(&size.to_be_bytes());
            }
        }
        Payload::PowerModeResponse(value) => output[0] = value.as_byte(),
        Payload::DiagnosticMessage(value) => encode_diagnostic(
            value.source_address,
            value.target_address,
            value.data,
            output,
        ),
        Payload::DiagnosticPositiveAck(value) | Payload::DiagnosticNegativeAck(value) => {
            output[..2].copy_from_slice(&value.source_address.to_be_bytes());
            output[2..4].copy_from_slice(&value.target_address.to_be_bytes());
            output[4] = value.code;
            output[5..].copy_from_slice(value.previous_data);
        }
    }
}

fn encode_diagnostic(source: u16, target: u16, data: &[u8], output: &mut [u8]) {
    output[..2].copy_from_slice(&source.to_be_bytes());
    output[2..4].copy_from_slice(&target.to_be_bytes());
    output[4..].copy_from_slice(data);
}
