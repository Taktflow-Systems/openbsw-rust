//! One stateful UDS implementation shared by DoCAN, DoIP, and the console.

use bsw_time::Instant;
use bsw_uds::{
    standard_services::{clear_diagnostic_information, read_dtc_information},
    DemManager, DiagRouter, DiagSession, EcuReset, Nrc, SessionMask, TesterPresent,
};

pub const DIAG_RESPONSE_CAPACITY: usize = 128;

/// Distinct DTC codes the composition DEM can hold (fixed pool, no heap).
pub const DEM_DTC_CAPACITY: usize = 8;

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
    dem: DemManager<DEM_DTC_CAPACITY>,
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
            dem: DemManager::new(),
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
            // 0x14/0x19 routed since re-pin be0029b: upstream tip
            // `UdsSystem::addDiagJobs` registers both services.
            0x14 => self.clear_dtc(request),
            0x19 => self.read_dtc(request),
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
        // P2* since re-pin be0029b: programming session answers 5000 x 10 ms
        // (upstream `uds/UdsConfig.h` PROGRAMMING_DIAG_RESPONSE_PENDING).
        let [p2_star_high, p2_star_low] = match session {
            DiagSession::Programming => [0x13, 0x88],
            DiagSession::Default | DiagSession::Extended => [0x01, 0xf4],
        };
        DiagnosticResponse::new(&[
            0x50,
            session.as_byte(),
            0x00,
            0x32,
            p2_star_high,
            p2_star_low,
        ])
    }

    fn clear_dtc(&mut self, request: &[u8]) -> DiagnosticResponse {
        let mut response = [0u8; DIAG_RESPONSE_CAPACITY];
        match clear_diagnostic_information(request, &mut self.dem, &mut response) {
            Ok(length) => DiagnosticResponse::new(&response[..length]),
            Err(nrc) => Self::nrc(0x14, nrc),
        }
    }

    fn read_dtc(&self, request: &[u8]) -> DiagnosticResponse {
        let mut response = [0u8; DIAG_RESPONSE_CAPACITY];
        match read_dtc_information(request, &self.dem, &mut response) {
            Ok(length) => DiagnosticResponse::new(&response[..length]),
            Err(nrc) => Self::nrc(0x19, nrc),
        }
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

    /// Application access to the shared DEM (fault reporting and recovery).
    pub fn dem_mut(&mut self) -> &mut DemManager<DEM_DTC_CAPACITY> {
        &mut self.dem
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
    fn programming_session_reports_extended_response_pending() {
        let mut core = DiagnosticCore::new();
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::DoCan,
                &[0x10, 0x02],
                Instant::from_nanos(1)
            )
            .bytes(),
            &[0x50, 0x02, 0x00, 0x32, 0x13, 0x88]
        );
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::DoCan,
                &[0x10, 0x01],
                Instant::from_nanos(2)
            )
            .bytes(),
            &[0x50, 0x01, 0x00, 0x32, 0x01, 0xf4]
        );
    }

    #[test]
    fn dtc_services_are_routed_through_the_shared_dem() {
        let mut core = DiagnosticCore::new();
        // Empty DEM: status-mask report answers header only, clear-all is OK.
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::DoIp,
                &[0x19, 0x02, 0xff],
                Instant::from_nanos(1)
            )
            .bytes(),
            &[0x59, 0x02, 0xff]
        );
        core.dem_mut().report_event(0x0012_3456, true);
        let response = core.dispatch_at(
            DiagnosticTransport::DoCan,
            &[0x19, 0x02, 0xff],
            Instant::from_nanos(2),
        );
        assert_eq!(&response.bytes()[..3], &[0x59, 0x02, 0xff]);
        assert_eq!(&response.bytes()[3..6], &[0x12, 0x34, 0x56]);
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::Console,
                &[0x19, 0x04, 0xff],
                Instant::from_nanos(3)
            )
            .bytes(),
            &[0x7f, 0x19, Nrc::SubFunctionNotSupported.as_byte()]
        );
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::Console,
                &[0x14, 0xff, 0xff, 0xff],
                Instant::from_nanos(4)
            )
            .bytes(),
            &[0x54]
        );
        assert_eq!(
            core.dispatch_at(
                DiagnosticTransport::Console,
                &[0x14, 0x00, 0x00, 0x00],
                Instant::from_nanos(5)
            )
            .bytes(),
            &[0x7f, 0x14, Nrc::RequestOutOfRange.as_byte()]
        );
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
