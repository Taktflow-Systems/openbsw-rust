//! Standard UDS service implementations.

use crate::diag_job::DiagJob;
use crate::nrc::Nrc;
use crate::session::{DiagSession, SessionMask};

/// `TesterPresent` service (0x3E).
pub struct TesterPresent {
    pub session_mask: SessionMask,
}

impl DiagJob for TesterPresent {
    fn implemented_request(&self) -> &[u8] {
        &[0x3E]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F; // mask off suppress-positive-response bit
        if sub_fn != 0x00 {
            return Err(Nrc::SubFunctionNotSupported);
        }
        response[0] = 0x7E; // 0x3E + 0x40
        response[1] = 0x00;
        Ok(2)
    }
}

/// `EcuReset` service (0x11).
pub struct EcuReset {
    pub session_mask: SessionMask,
}

/// Reset types for `EcuReset`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ResetType {
    HardReset = 0x01,
    KeyOffOnReset = 0x02,
    SoftReset = 0x03,
}

impl ResetType {
    /// Convert from raw byte. Returns `None` for unrecognized values.
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x01 => Some(Self::HardReset),
            0x02 => Some(Self::KeyOffOnReset),
            0x03 => Some(Self::SoftReset),
            _ => None,
        }
    }
}

impl DiagJob for EcuReset {
    fn implemented_request(&self) -> &[u8] {
        &[0x11]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F;
        if ResetType::from_byte(sub_fn).is_none() {
            return Err(Nrc::SubFunctionNotSupported);
        }
        response[0] = 0x51; // 0x11 + 0x40
        response[1] = sub_fn;
        Ok(2)
    }
}

/// `DiagnosticSessionControl` service (0x10).
pub struct DiagnosticSessionControl {
    pub current_session: DiagSession,
}

impl DiagJob for DiagnosticSessionControl {
    fn implemented_request(&self) -> &[u8] {
        &[0x10]
    }
    fn session_mask(&self) -> SessionMask {
        SessionMask::ALL
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F;
        if DiagSession::from_byte(sub_fn).is_none() {
            return Err(Nrc::SubFunctionNotSupported);
        }
        response[0] = 0x50; // 0x10 + 0x40
        response[1] = sub_fn;
        // Default timing parameters (P2 server = 50 ms, P2* = 5000 ms)
        response[2] = 0x00;
        response[3] = 0x32; // P2 = 50 (big-endian u16)
        response[4] = 0x01;
        response[5] = 0xF4; // P2* = 500 (×10 ms = 5000 ms, big-endian u16)
        Ok(6)
    }
}

/// `ControlDtcSetting` service (0x85).
pub struct ControlDtcSetting {
    pub session_mask: SessionMask,
}

/// DTC setting types for `ControlDtcSetting`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DtcSettingType {
    On = 0x01,
    Off = 0x02,
}

impl DtcSettingType {
    /// Convert from raw byte. Returns `None` for unrecognized values.
    pub const fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0x01 => Some(Self::On),
            0x02 => Some(Self::Off),
            _ => None,
        }
    }
}

impl DiagJob for ControlDtcSetting {
    fn implemented_request(&self) -> &[u8] {
        &[0x85]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F;
        if DtcSettingType::from_byte(sub_fn).is_none() {
            return Err(Nrc::SubFunctionNotSupported);
        }
        response[0] = 0xC5; // 0x85 + 0x40
        response[1] = sub_fn;
        Ok(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- TesterPresent ---

    #[test]
    fn tester_present_ok() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x3E, 0x00], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0x7E);
        assert_eq!(buf[1], 0x00);
    }

    #[test]
    fn tester_present_bad_subfn() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // sub-function 0x01 is not supported
        let result = svc.process(&[0x3E, 0x01], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn tester_present_too_short() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x3E], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    #[test]
    fn tester_present_suppress_bit_masked() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // 0x80 = suppress bit set over sub-function 0x00 — still valid
        let result = svc.process(&[0x3E, 0x80], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0x7E);
        assert_eq!(buf[1], 0x00);
    }

    // --- EcuReset ---

    #[test]
    fn ecu_reset_hard() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x11, 0x01], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0x51);
        assert_eq!(buf[1], 0x01);
    }

    #[test]
    fn ecu_reset_soft() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x11, 0x03], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0x51);
        assert_eq!(buf[1], 0x03);
    }

    #[test]
    fn ecu_reset_bad_type() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x11, 0x04], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn ecu_reset_too_short() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x11], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    // --- DiagnosticSessionControl ---

    #[test]
    fn session_control_default() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x10, 0x01], &mut buf);
        assert_eq!(result, Ok(6));
        assert_eq!(buf[0], 0x50);
        assert_eq!(buf[1], 0x01);
    }

    #[test]
    fn session_control_extended() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x10, 0x03], &mut buf);
        assert_eq!(result, Ok(6));
        assert_eq!(buf[0], 0x50);
        assert_eq!(buf[1], 0x03);
    }

    #[test]
    fn session_control_bad_session() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        // 0x04 is ISO reserved — not a valid standard session.
        let result = svc.process(&[0x10, 0x04], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn session_control_response_format() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        svc.process(&[0x10, 0x02], &mut buf).unwrap();
        // P2 server = 50 ms big-endian u16
        assert_eq!(u16::from_be_bytes([buf[2], buf[3]]), 50);
        // P2* = 500 × 10 ms = 5000 ms big-endian u16
        assert_eq!(u16::from_be_bytes([buf[4], buf[5]]), 500);
    }

    // --- ControlDtcSetting ---

    #[test]
    fn dtc_on() {
        let svc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x85, 0x01], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0xC5);
        assert_eq!(buf[1], 0x01);
    }

    #[test]
    fn dtc_off() {
        let svc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x85, 0x02], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0xC5);
        assert_eq!(buf[1], 0x02);
    }

    #[test]
    fn dtc_bad_type() {
        let svc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x85, 0x03], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn dtc_too_short() {
        let svc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x85], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }
}
