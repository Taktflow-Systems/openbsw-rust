//! UDS Service Identifiers (ISO 14229-1 §9).

/// UDS Service Identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ServiceId {
    DiagnosticSessionControl = 0x10,
    EcuReset = 0x11,
    ClearDiagnosticInformation = 0x14,
    ReadDataByIdentifier = 0x22,
    ReadMemoryByAddress = 0x23,
    ReadScalingDataByIdentifier = 0x24,
    SecurityAccess = 0x27,
    CommunicationControl = 0x28,
    Authentication = 0x29,
    ReadDataByPeriodicIdentifier = 0x2A,
    DynamicallyDefineDataIdentifier = 0x2C,
    WriteDataByIdentifier = 0x2E,
    InputOutputControlByIdentifier = 0x2F,
    RoutineControl = 0x31,
    RequestDownload = 0x34,
    RequestUpload = 0x35,
    TransferData = 0x36,
    RequestTransferExit = 0x37,
    RequestFileTransfer = 0x38,
    WriteMemoryByAddress = 0x3D,
    TesterPresent = 0x3E,
    ReadDtcInformation = 0x19,
    AccessTimingParameter = 0x83,
    SecuredDataTransmission = 0x84,
    ControlDtcSetting = 0x85,
    ResponseOnEvent = 0x86,
    LinkControl = 0x87,
    /// Negative response service ID (not a real service).
    NegativeResponse = 0x7F,
}

impl ServiceId {
    /// Convert from raw byte. Returns `None` for unrecognized values.
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x10 => Some(Self::DiagnosticSessionControl),
            0x11 => Some(Self::EcuReset),
            0x14 => Some(Self::ClearDiagnosticInformation),
            0x19 => Some(Self::ReadDtcInformation),
            0x22 => Some(Self::ReadDataByIdentifier),
            0x23 => Some(Self::ReadMemoryByAddress),
            0x24 => Some(Self::ReadScalingDataByIdentifier),
            0x27 => Some(Self::SecurityAccess),
            0x28 => Some(Self::CommunicationControl),
            0x29 => Some(Self::Authentication),
            0x2A => Some(Self::ReadDataByPeriodicIdentifier),
            0x2C => Some(Self::DynamicallyDefineDataIdentifier),
            0x2E => Some(Self::WriteDataByIdentifier),
            0x2F => Some(Self::InputOutputControlByIdentifier),
            0x31 => Some(Self::RoutineControl),
            0x34 => Some(Self::RequestDownload),
            0x35 => Some(Self::RequestUpload),
            0x36 => Some(Self::TransferData),
            0x37 => Some(Self::RequestTransferExit),
            0x38 => Some(Self::RequestFileTransfer),
            0x3D => Some(Self::WriteMemoryByAddress),
            0x3E => Some(Self::TesterPresent),
            0x7F => Some(Self::NegativeResponse),
            0x83 => Some(Self::AccessTimingParameter),
            0x84 => Some(Self::SecuredDataTransmission),
            0x85 => Some(Self::ControlDtcSetting),
            0x86 => Some(Self::ResponseOnEvent),
            0x87 => Some(Self::LinkControl),
            _ => None,
        }
    }

    /// Convert to raw byte.
    pub const fn as_byte(self) -> u8 {
        self as u8
    }

    /// Positive response SID = request SID + 0x40.
    pub const fn positive_response_id(self) -> u8 {
        self.as_byte().wrapping_add(0x40)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_byte_valid() {
        assert_eq!(ServiceId::from_byte(0x10), Some(ServiceId::DiagnosticSessionControl));
        assert_eq!(ServiceId::from_byte(0x22), Some(ServiceId::ReadDataByIdentifier));
        assert_eq!(ServiceId::from_byte(0x7F), Some(ServiceId::NegativeResponse));
        assert_eq!(ServiceId::from_byte(0x3E), Some(ServiceId::TesterPresent));
    }

    #[test]
    fn from_byte_invalid() {
        assert_eq!(ServiceId::from_byte(0x00), None);
        assert_eq!(ServiceId::from_byte(0x01), None);
        assert_eq!(ServiceId::from_byte(0xFF), None);
        assert_eq!(ServiceId::from_byte(0x40), None);
    }

    #[test]
    fn positive_response_ids() {
        assert_eq!(ServiceId::DiagnosticSessionControl.positive_response_id(), 0x50);
        assert_eq!(ServiceId::EcuReset.positive_response_id(), 0x51);
        assert_eq!(ServiceId::TesterPresent.positive_response_id(), 0x7E);
        assert_eq!(ServiceId::ReadDataByIdentifier.positive_response_id(), 0x62);
    }

    #[test]
    fn as_byte_roundtrip() {
        let sid = ServiceId::SecurityAccess;
        assert_eq!(ServiceId::from_byte(sid.as_byte()), Some(sid));
    }

    #[test]
    fn all_services_have_unique_ids() {
        let all: &[ServiceId] = &[
            ServiceId::DiagnosticSessionControl,
            ServiceId::EcuReset,
            ServiceId::SecurityAccess,
            ServiceId::CommunicationControl,
            ServiceId::Authentication,
            ServiceId::TesterPresent,
            ServiceId::AccessTimingParameter,
            ServiceId::SecuredDataTransmission,
            ServiceId::ControlDtcSetting,
            ServiceId::ResponseOnEvent,
            ServiceId::LinkControl,
            ServiceId::ReadDataByIdentifier,
            ServiceId::ReadMemoryByAddress,
            ServiceId::ReadScalingDataByIdentifier,
            ServiceId::ReadDataByPeriodicIdentifier,
            ServiceId::DynamicallyDefineDataIdentifier,
            ServiceId::WriteDataByIdentifier,
            ServiceId::WriteMemoryByAddress,
            ServiceId::ClearDiagnosticInformation,
            ServiceId::ReadDtcInformation,
            ServiceId::InputOutputControlByIdentifier,
            ServiceId::RoutineControl,
            ServiceId::RequestDownload,
            ServiceId::RequestUpload,
            ServiceId::TransferData,
            ServiceId::RequestTransferExit,
            ServiceId::RequestFileTransfer,
            ServiceId::NegativeResponse,
        ];
        // Check all bytes are distinct.
        for i in 0..all.len() {
            for j in (i + 1)..all.len() {
                assert_ne!(all[i].as_byte(), all[j].as_byte(), "{:?} and {:?} share byte", all[i], all[j]);
            }
        }
    }

    #[test]
    fn session_control_is_0x10() {
        assert_eq!(ServiceId::DiagnosticSessionControl.as_byte(), 0x10);
    }

    #[test]
    fn negative_response_is_0x7f() {
        assert_eq!(ServiceId::NegativeResponse.as_byte(), 0x7F);
    }

    #[test]
    fn read_data_is_0x22() {
        assert_eq!(ServiceId::ReadDataByIdentifier.as_byte(), 0x22);
    }
}
