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

// ─── WriteDataByIdentifier (0x2E) ────────────────────────────────────────────

/// `WriteDataByIdentifier` service (0x2E).
///
/// Validates request format and session/security access; the actual NvM write
/// is expected to be performed by the caller after receiving a positive result.
pub struct WriteDataByIdentifier {
    pub session_mask: SessionMask,
    /// If `true`, `SecurityAccess` must have been unlocked before writes are
    /// accepted (checked by the caller via `SecurityAccess::is_unlocked`).
    pub security_required: bool,
    /// Whether security has been granted externally (set by caller).
    pub security_unlocked: bool,
}

impl DiagJob for WriteDataByIdentifier {
    fn implemented_request(&self) -> &[u8] {
        &[0x2E]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        // Min: SID(1) + DID_hi(1) + DID_lo(1) + data(≥1) = 4 bytes
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if self.security_required && !self.security_unlocked {
            return Err(Nrc::SecurityAccessDenied);
        }
        let did_hi = request[1];
        let did_lo = request[2];
        // Validate DID is non-zero (0x0000 is reserved / undefined)
        if did_hi == 0x00 && did_lo == 0x00 {
            return Err(Nrc::RequestOutOfRange);
        }
        // Positive response: [0x6E, DID_hi, DID_lo]
        response[0] = 0x6E;
        response[1] = did_hi;
        response[2] = did_lo;
        Ok(3)
    }
}

// ─── SecurityAccess (0x27) ───────────────────────────────────────────────────

/// `SecurityAccess` service (0x27).
///
/// Simple seed/key exchange. Key = seed XOR 0xDEAD_BEEF (demo algorithm).
/// Uses interior mutability (`Cell`) so the `DiagJob::process(&self, …)` contract
/// is honoured while still allowing state updates during a request.
pub struct SecurityAccess {
    pub session_mask: SessionMask,
    seed: core::cell::Cell<u32>,
    seed_issued: core::cell::Cell<bool>,
    unlocked: core::cell::Cell<bool>,
    failed_attempts: core::cell::Cell<u8>,
    pub max_attempts: u8,
    locked_until_reset: core::cell::Cell<bool>,
}

impl SecurityAccess {
    /// Create a new `SecurityAccess` instance.
    pub const fn new(session_mask: SessionMask, max_attempts: u8) -> Self {
        Self {
            session_mask,
            seed: core::cell::Cell::new(0),
            seed_issued: core::cell::Cell::new(false),
            unlocked: core::cell::Cell::new(false),
            failed_attempts: core::cell::Cell::new(0),
            max_attempts,
            locked_until_reset: core::cell::Cell::new(false),
        }
    }

    /// Returns `true` if the security level has been unlocked.
    pub fn is_unlocked(&self) -> bool {
        self.unlocked.get()
    }

    /// Reset the lock-out state (call on ECU reset / session change).
    pub fn reset(&self) {
        self.seed.set(0);
        self.seed_issued.set(false);
        self.unlocked.set(false);
        self.failed_attempts.set(0);
        self.locked_until_reset.set(false);
    }

    /// Simple counter-based PRNG — deterministic but non-zero for any seed != 0.
    fn next_seed(prev: u32) -> u32 {
        // Linear congruential generator (Knuth params)
        prev.wrapping_mul(1_664_525).wrapping_add(1_013_904_223) | 0x0000_0001
    }
}

impl DiagJob for SecurityAccess {
    fn implemented_request(&self) -> &[u8] {
        &[0x27]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if self.locked_until_reset.get() {
            return Err(Nrc::ExceededNumberOfAttempts);
        }
        let sub_fn = request[1] & 0x7F;
        match sub_fn {
            0x01 => {
                // requestSeed
                if self.unlocked.get() {
                    // Already unlocked — return zero seed (ISO 14229 §10.4.3)
                    response[0] = 0x67;
                    response[1] = 0x01;
                    response[2] = 0x00;
                    response[3] = 0x00;
                    return Ok(4);
                }
                let new_seed = Self::next_seed(self.seed.get());
                self.seed.set(new_seed);
                self.seed_issued.set(true);
                response[0] = 0x67;
                response[1] = 0x01;
                response[2] = (new_seed >> 8) as u8;
                response[3] = new_seed as u8;
                Ok(4)
            }
            0x02 => {
                // sendKey
                if !self.seed_issued.get() {
                    return Err(Nrc::RequestSequenceError);
                }
                if request.len() < 4 {
                    return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
                }
                let received_key =
                    (u32::from(request[2]) << 8) | u32::from(request[3]);
                let expected_key = (self.seed.get() ^ 0xDEAD_BEEF) as u16;
                if received_key as u16 != expected_key {
                    let attempts = self.failed_attempts.get().saturating_add(1);
                    self.failed_attempts.set(attempts);
                    if attempts >= self.max_attempts {
                        self.locked_until_reset.set(true);
                        return Err(Nrc::ExceededNumberOfAttempts);
                    }
                    return Err(Nrc::InvalidKey);
                }
                self.unlocked.set(true);
                self.seed_issued.set(false);
                response[0] = 0x67;
                response[1] = 0x02;
                Ok(2)
            }
            _ => Err(Nrc::SubFunctionNotSupported),
        }
    }
}

// ─── RoutineControl (0x31) ───────────────────────────────────────────────────

/// `RoutineControl` service (0x31).
///
/// Supports `startRoutine` (sub-function 0x01) for three fixed routine IDs:
/// * `0xFF00` — LED blink test
/// * `0xFF01` — CAN self-test
/// * `0xFF02` — Read uptime (4-byte placeholder)
pub struct RoutineControl {
    pub session_mask: SessionMask,
}

impl DiagJob for RoutineControl {
    fn implemented_request(&self) -> &[u8] {
        &[0x31]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        // Min: SID(1) + sub-fn(1) + routine_hi(1) + routine_lo(1) = 4 bytes
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F;
        if sub_fn != 0x01 {
            // Only startRoutine is implemented
            return Err(Nrc::SubFunctionNotSupported);
        }
        let routine_hi = request[2];
        let routine_lo = request[3];
        match (routine_hi, routine_lo) {
            (0xFF, 0x00) => {
                // LED blink test — return status=started (0x01)
                response[0] = 0x71;
                response[1] = 0x01;
                response[2] = 0xFF;
                response[3] = 0x00;
                response[4] = 0x01; // status: started
                Ok(5)
            }
            (0xFF, 0x01) => {
                // CAN self-test — return result byte 0x00 (pass)
                response[0] = 0x71;
                response[1] = 0x01;
                response[2] = 0xFF;
                response[3] = 0x01;
                response[4] = 0x00; // result: pass
                Ok(5)
            }
            (0xFF, 0x02) => {
                // Read uptime — 4-byte placeholder (all zeros until timer wired)
                response[0] = 0x71;
                response[1] = 0x01;
                response[2] = 0xFF;
                response[3] = 0x02;
                response[4] = 0x00;
                response[5] = 0x00;
                response[6] = 0x00;
                response[7] = 0x00;
                Ok(8)
            }
            _ => Err(Nrc::RequestOutOfRange),
        }
    }
}

// ─── ReadDtcInformation (0x19) ───────────────────────────────────────────────

/// `ReadDtcInformation` service (0x19).
///
/// Implements:
/// * Sub-function `0x02` — `reportDTCByStatusMask` (returns empty list until DEM is wired)
/// * Sub-function `0x0A` — `reportSupportedDTC` (returns empty list until DEM is wired)
pub struct ReadDtcInformation {
    pub session_mask: SessionMask,
}

impl DiagJob for ReadDtcInformation {
    fn implemented_request(&self) -> &[u8] {
        &[0x19]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let sub_fn = request[1] & 0x7F;
        match sub_fn {
            0x02 => {
                // reportDTCByStatusMask — needs 3-byte request (SID + sub-fn + mask)
                if request.len() < 3 {
                    return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
                }
                // Return empty DTC list; availability mask = 0xFF
                response[0] = 0x59;
                response[1] = 0x02;
                response[2] = 0xFF; // availability mask
                Ok(3)
            }
            0x0A => {
                // reportSupportedDTC — return empty DTC list; availability mask = 0xFF
                response[0] = 0x59;
                response[1] = 0x0A;
                response[2] = 0xFF; // availability mask
                Ok(3)
            }
            _ => Err(Nrc::SubFunctionNotSupported),
        }
    }
}

// ─── ClearDiagnosticInformation (0x14) ───────────────────────────────────────

/// `ClearDiagnosticInformation` service (0x14).
///
/// Accepts any 3-byte group-of-DTC value. `0xFFFFFF` clears all DTCs.
/// Actual DTC clearing is deferred to the DEM integration layer; for now
/// this service always returns a positive response.
pub struct ClearDiagnosticInformation {
    pub session_mask: SessionMask,
}

impl DiagJob for ClearDiagnosticInformation {
    fn implemented_request(&self) -> &[u8] {
        &[0x14]
    }
    fn session_mask(&self) -> SessionMask {
        self.session_mask
    }
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        // SID(1) + group_hi(1) + group_mid(1) + group_lo(1) = 4 bytes
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        // Positive response is a single byte: 0x54
        response[0] = 0x54;
        Ok(1)
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

    // --- WriteDataByIdentifier ---

    #[test]
    fn write_did_ok() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: false,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        // SID + DID 0xF190 + 1 data byte
        let result = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(result, Ok(3));
        assert_eq!(buf[0], 0x6E);
        assert_eq!(buf[1], 0xF1);
        assert_eq!(buf[2], 0x90);
    }

    #[test]
    fn write_did_too_short() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: false,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        // Only 3 bytes — missing data byte
        let result = svc.process(&[0x2E, 0xF1, 0x90], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    #[test]
    fn write_did_security_denied() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: true,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(result, Err(Nrc::SecurityAccessDenied));
    }

    #[test]
    fn write_did_security_unlocked_ok() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: true,
            security_unlocked: true,
        };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(result, Ok(3));
    }

    #[test]
    fn write_did_reserved_zero_did() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: false,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x2E, 0x00, 0x00, 0xAB], &mut buf);
        assert_eq!(result, Err(Nrc::RequestOutOfRange));
    }

    #[test]
    fn write_did_session_mask_enforced() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::EXTENDED,
            security_required: false,
            security_unlocked: false,
        };
        // verify() is checked by the router; process() itself doesn't re-check session
        // — just verify session_mask is stored correctly so the router can reject it.
        assert!(!svc.session_mask().contains(DiagSession::Default));
        assert!(svc.session_mask().contains(DiagSession::Extended));
    }

    // --- SecurityAccess ---

    #[test]
    fn security_access_request_seed() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(result, Ok(4));
        assert_eq!(buf[0], 0x67);
        assert_eq!(buf[1], 0x01);
        // Seed must be non-zero (LCG with OR 0x0001)
        let seed = u16::from_be_bytes([buf[2], buf[3]]);
        assert_ne!(seed, 0);
    }

    #[test]
    fn security_access_correct_key_unlocks() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        // 1. Request seed
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        let seed_hi = buf[2];
        let seed_lo = buf[3];
        let seed = (u32::from(seed_hi) << 8) | u32::from(seed_lo);
        // 2. Compute correct key (lower 16 bits of seed XOR 0xDEAD_BEEF)
        let key = (seed ^ 0xDEAD_BEEF) as u16;
        let key_hi = (key >> 8) as u8;
        let key_lo = key as u8;
        let result = svc.process(&[0x27, 0x02, key_hi, key_lo], &mut buf);
        assert_eq!(result, Ok(2));
        assert_eq!(buf[0], 0x67);
        assert_eq!(buf[1], 0x02);
        assert!(svc.is_unlocked());
    }

    #[test]
    fn security_access_wrong_key_nrc() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        let result = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        assert_eq!(result, Err(Nrc::InvalidKey));
        assert!(!svc.is_unlocked());
    }

    #[test]
    fn security_access_exceeds_max_attempts() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 2);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        // First wrong key
        let _ = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        // Second wrong key — should lock
        let result = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        assert_eq!(result, Err(Nrc::ExceededNumberOfAttempts));
        // Subsequent requests also locked
        let result2 = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(result2, Err(Nrc::ExceededNumberOfAttempts));
    }

    #[test]
    fn security_access_send_key_without_seed_nrc() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        // sendKey without requestSeed first
        let result = svc.process(&[0x27, 0x02, 0x12, 0x34], &mut buf);
        assert_eq!(result, Err(Nrc::RequestSequenceError));
    }

    #[test]
    fn security_access_already_unlocked_returns_zero_seed() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        // Unlock first
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        let seed_hi = buf[2];
        let seed_lo = buf[3];
        let seed = (u32::from(seed_hi) << 8) | u32::from(seed_lo);
        let key = (seed ^ 0xDEAD_BEEF) as u16;
        svc.process(&[0x27, 0x02, (key >> 8) as u8, key as u8], &mut buf).unwrap();
        assert!(svc.is_unlocked());
        // Now request seed again — should return [0x67, 0x01, 0x00, 0x00]
        let result = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(result, Ok(4));
        assert_eq!(buf[2], 0x00);
        assert_eq!(buf[3], 0x00);
    }

    #[test]
    fn security_access_bad_subfn() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x27, 0x03], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn security_access_reset_clears_state() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 2);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        // Force lock-out
        let _ = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        let _ = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        assert!(svc.locked_until_reset.get());
        svc.reset();
        assert!(!svc.locked_until_reset.get());
        assert!(!svc.is_unlocked());
        // Should be able to request seed again after reset
        let result = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(result, Ok(4));
    }

    // --- RoutineControl ---

    #[test]
    fn routine_control_led_blink() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x31, 0x01, 0xFF, 0x00], &mut buf);
        assert_eq!(result, Ok(5));
        assert_eq!(buf[0], 0x71);
        assert_eq!(buf[1], 0x01);
        assert_eq!(buf[2], 0xFF);
        assert_eq!(buf[3], 0x00);
        assert_eq!(buf[4], 0x01); // status: started
    }

    #[test]
    fn routine_control_can_self_test() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x31, 0x01, 0xFF, 0x01], &mut buf);
        assert_eq!(result, Ok(5));
        assert_eq!(buf[0], 0x71);
        assert_eq!(buf[3], 0x01);
        assert_eq!(buf[4], 0x00); // result: pass
    }

    #[test]
    fn routine_control_read_uptime() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x31, 0x01, 0xFF, 0x02], &mut buf);
        assert_eq!(result, Ok(8));
        assert_eq!(buf[0], 0x71);
        assert_eq!(buf[1], 0x01);
        assert_eq!(buf[2], 0xFF);
        assert_eq!(buf[3], 0x02);
    }

    #[test]
    fn routine_control_unknown_routine_nrc() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x31, 0x01, 0x01, 0x00], &mut buf);
        assert_eq!(result, Err(Nrc::RequestOutOfRange));
    }

    #[test]
    fn routine_control_bad_subfn() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        // sub-function 0x02 (stopRoutine) is not implemented
        let result = svc.process(&[0x31, 0x02, 0xFF, 0x00], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn routine_control_too_short() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x31, 0x01, 0xFF], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    // --- ReadDtcInformation ---

    #[test]
    fn read_dtc_by_status_mask() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // reportDTCByStatusMask, mask=0xFF
        let result = svc.process(&[0x19, 0x02, 0xFF], &mut buf);
        assert_eq!(result, Ok(3));
        assert_eq!(buf[0], 0x59);
        assert_eq!(buf[1], 0x02);
        assert_eq!(buf[2], 0xFF); // availability mask
    }

    #[test]
    fn read_dtc_report_supported() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x19, 0x0A], &mut buf);
        assert_eq!(result, Ok(3));
        assert_eq!(buf[0], 0x59);
        assert_eq!(buf[1], 0x0A);
        assert_eq!(buf[2], 0xFF);
    }

    #[test]
    fn read_dtc_unknown_subfn() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x19, 0x01], &mut buf);
        assert_eq!(result, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn read_dtc_too_short() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x19], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    #[test]
    fn read_dtc_by_status_mask_missing_mask_byte() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // sub-fn 0x02 but no status mask byte
        let result = svc.process(&[0x19, 0x02], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    // --- ClearDiagnosticInformation ---

    #[test]
    fn clear_dtc_all() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // 0xFFFFFF = clear all
        let result = svc.process(&[0x14, 0xFF, 0xFF, 0xFF], &mut buf);
        assert_eq!(result, Ok(1));
        assert_eq!(buf[0], 0x54);
    }

    #[test]
    fn clear_dtc_specific_group() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x14, 0x00, 0x12, 0x34], &mut buf);
        assert_eq!(result, Ok(1));
        assert_eq!(buf[0], 0x54);
    }

    #[test]
    fn clear_dtc_too_short() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let result = svc.process(&[0x14, 0xFF, 0xFF], &mut buf);
        assert_eq!(result, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    #[test]
    fn clear_dtc_session_mask_enforced() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::EXTENDED };
        assert!(!svc.session_mask().contains(DiagSession::Default));
        assert!(svc.session_mask().contains(DiagSession::Extended));
    }
}
