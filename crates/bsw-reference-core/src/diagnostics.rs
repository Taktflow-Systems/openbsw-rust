//! One stateful UDS implementation shared by DoCAN, DoIP, and the console.

use bsw_time::Instant;
use bsw_uds::{DiagRouter, DiagSession, EcuReset, Nrc, SessionMask, TesterPresent};

pub const DIAG_RESPONSE_CAPACITY: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticTransport {
    DoCan,
    DoIp,
    Console,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiagnosticResponse {
    bytes: [u8; DIAG_RESPONSE_CAPACITY],
    len: usize,
}

impl DiagnosticResponse {
    fn new(bytes: &[u8]) -> Self {
        let mut response = Self {
            bytes: [0; DIAG_RESPONSE_CAPACITY],
            len: bytes.len().min(DIAG_RESPONSE_CAPACITY),
        };
        response.bytes[..response.len].copy_from_slice(&bytes[..response.len]);
        response
    }

    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    #[must_use]
    pub fn from_bytes(bytes: &[u8]) -> Self {
        Self::new(bytes)
    }
}

pub struct DiagnosticCore {
    session: DiagSession,
    cf03: [u8; 32],
    cf03_len: usize,
    persistent_counter: u32,
    dispatch_count: u64,
    last_transport: Option<DiagnosticTransport>,
    last_request_at: Instant,
}

impl DiagnosticCore {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            session: DiagSession::Default,
            cf03: [0; 32],
            cf03_len: 0,
            persistent_counter: 0,
            dispatch_count: 0,
            last_transport: None,
            last_request_at: Instant::from_nanos(0),
        }
    }

    pub fn dispatch_at(
        &mut self,
        transport: DiagnosticTransport,
        request: &[u8],
        now: Instant,
    ) -> DiagnosticResponse {
        self.dispatch_count = self.dispatch_count.saturating_add(1);
        self.last_transport = Some(transport);
        self.last_request_at = now;
        let Some(&sid) = request.first() else {
            return DiagnosticResponse::new(&[
                0x7f,
                0x00,
                Nrc::IncorrectMessageLengthOrInvalidFormat.as_byte(),
            ]);
        };
        match sid {
            0x10 => self.session_control(request),
            0x22 => self.read_did(request),
            0x2e => self.write_did(request),
            0x31 => self.routine_control(request),
            _ => self.route_common(request),
        }
    }

    fn route_common(&self, request: &[u8]) -> DiagnosticResponse {
        let tester = TesterPresent {
            session_mask: SessionMask::ALL,
        };
        let reset = EcuReset {
            session_mask: SessionMask::ALL,
        };
        let jobs: &[&dyn bsw_uds::DiagJob] = &[&tester, &reset];
        let router = DiagRouter::new(jobs);
        let mut response = [0u8; DIAG_RESPONSE_CAPACITY];
        match router.dispatch(request, self.session, &mut response) {
            Ok(length) => DiagnosticResponse::new(&response[..length]),
            Err(nrc) => DiagnosticResponse::new(&[0x7f, request[0], nrc.as_byte()]),
        }
    }

    fn session_control(&mut self, request: &[u8]) -> DiagnosticResponse {
        if request.len() != 2 {
            return Self::nrc(0x10, Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let Some(session) = DiagSession::from_byte(request[1] & 0x7f) else {
            return Self::nrc(0x10, Nrc::SubFunctionNotSupported);
        };
        self.session = session;
        DiagnosticResponse::new(&[0x50, session.as_byte(), 0x00, 0x32, 0x01, 0xf4])
    }

    fn read_did(&self, request: &[u8]) -> DiagnosticResponse {
        if request.len() != 3 {
            return Self::nrc(0x22, Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        match u16::from_be_bytes([request[1], request[2]]) {
            0xcf01 => {
                let mut bytes = [0u8; 27];
                bytes[..3].copy_from_slice(&[0x62, 0xcf, 0x01]);
                bytes[3..].copy_from_slice(b"OpenBSW Rust Reference!!");
                DiagnosticResponse::new(&bytes)
            }
            0xcf02 => DiagnosticResponse::new(&[0x62, 0xcf, 0x02, self.session.as_byte()]),
            0xcf03 => {
                let mut bytes = [0u8; 35];
                bytes[..3].copy_from_slice(&[0x62, 0xcf, 0x03]);
                bytes[3..3 + self.cf03_len].copy_from_slice(&self.cf03[..self.cf03_len]);
                DiagnosticResponse::new(&bytes[..3 + self.cf03_len])
            }
            0xf1a0 => {
                let mut bytes = [0u8; 7];
                bytes[..3].copy_from_slice(&[0x62, 0xf1, 0xa0]);
                bytes[3..].copy_from_slice(&self.persistent_counter.to_be_bytes());
                DiagnosticResponse::new(&bytes)
            }
            _ => Self::nrc(0x22, Nrc::RequestOutOfRange),
        }
    }

    fn write_did(&mut self, request: &[u8]) -> DiagnosticResponse {
        if request.len() < 4 || request.len() > 35 {
            return Self::nrc(0x2e, Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if request[1..3] != [0xcf, 0x03] {
            return Self::nrc(0x2e, Nrc::RequestOutOfRange);
        }
        if self.session != DiagSession::Extended {
            return Self::nrc(0x2e, Nrc::ServiceNotSupportedInActiveSession);
        }
        self.cf03_len = request.len() - 3;
        self.cf03[..self.cf03_len].copy_from_slice(&request[3..]);
        DiagnosticResponse::new(&[0x6e, 0xcf, 0x03])
    }

    fn routine_control(&mut self, request: &[u8]) -> DiagnosticResponse {
        if request != [0x31, 0x01, 0x12, 0x34] {
            return Self::nrc(0x31, Nrc::RequestOutOfRange);
        }
        self.persistent_counter = self.persistent_counter.saturating_add(1);
        let mut bytes = [0u8; 8];
        bytes[..4].copy_from_slice(&[0x71, 0x01, 0x12, 0x34]);
        bytes[4..].copy_from_slice(&self.persistent_counter.to_be_bytes());
        DiagnosticResponse::new(&bytes)
    }

    fn nrc(sid: u8, nrc: Nrc) -> DiagnosticResponse {
        DiagnosticResponse::new(&[0x7f, sid, nrc.as_byte()])
    }

    #[must_use]
    pub const fn session(&self) -> DiagSession {
        self.session
    }

    #[must_use]
    pub const fn dispatch_count(&self) -> u64 {
        self.dispatch_count
    }

    #[must_use]
    pub const fn persistent_counter(&self) -> u32 {
        self.persistent_counter
    }

    pub fn restore_counter(&mut self, value: u32) {
        self.persistent_counter = value;
    }
}

impl Default for DiagnosticCore {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn session_and_did_state_are_shared_across_transports() {
        let mut core = DiagnosticCore::new();
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::DoCan,
                &[0x10, 3],
                Instant::from_nanos(1)
            )
            .bytes()[0],
            0x50
        );
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::DoIp,
                &[0x22, 0xcf, 2],
                Instant::from_nanos(2)
            )
            .bytes(),
            &[0x62, 0xcf, 2, 3]
        );
        assert_eq!(core.dispatch_count(), 2);
    }

    #[test]
    fn malformed_requests_are_bounded() {
        let mut core = DiagnosticCore::new();
        for len in 0..=256 {
            let bytes = [0xff; 256];
            assert!(
                core.dispatch_at(
                    DiagnosticTransport::DoCan,
                    &bytes[..len],
                    Instant::from_nanos(len as u64)
                )
                .bytes()
                .len()
                    <= DIAG_RESPONSE_CAPACITY
            );
        }
    }
}
