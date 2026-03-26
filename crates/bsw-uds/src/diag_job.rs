//! Diagnostic job routing — core UDS request dispatch framework.

use crate::nrc::Nrc;
use crate::session::{DiagSession, SessionMask};

/// Result of a diagnostic job verification or execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagResult {
    /// Job completed successfully.
    Ok,
    /// Negative response with given NRC.
    Nrc(Nrc),
    /// Job not responsible for this request.
    NotResponsible,
}

/// Trait for a diagnostic job that can verify and process requests.
pub trait DiagJob {
    /// The request prefix this job handles (e.g., `[0x22]` for `ReadDataByIdentifier`).
    fn implemented_request(&self) -> &[u8];

    /// Minimum request length (including the prefix).
    fn min_request_length(&self) -> usize {
        self.implemented_request().len()
    }

    /// Sessions in which this job is available.
    fn session_mask(&self) -> SessionMask {
        SessionMask::ALL
    }

    /// Verify whether this job handles the given request in the given session.
    ///
    /// Default implementation: prefix match + session check.
    fn verify(&self, request: &[u8], session: DiagSession) -> DiagResult {
        let prefix = self.implemented_request();
        if request.len() < prefix.len() {
            return DiagResult::NotResponsible;
        }
        if &request[..prefix.len()] != prefix {
            return DiagResult::NotResponsible;
        }
        if !self.session_mask().contains(session) {
            return DiagResult::Nrc(Nrc::ServiceNotSupportedInActiveSession);
        }
        DiagResult::Ok
    }

    /// Process the request. Only called after `verify` returned `DiagResult::Ok`.
    ///
    /// Writes the response into `response` and returns the number of bytes written.
    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc>;
}

/// Static router that dispatches to the first matching job.
pub struct DiagRouter<'a> {
    jobs: &'a [&'a dyn DiagJob],
}

impl<'a> DiagRouter<'a> {
    /// Create a new router from a slice of job references.
    pub const fn new(jobs: &'a [&'a dyn DiagJob]) -> Self {
        Self { jobs }
    }

    /// Route a request to the first matching job.
    ///
    /// Returns `Ok(len)` on success, or `Err(nrc)` if no job matches or a job
    /// returned a session-level NRC.
    pub fn dispatch(
        &self,
        request: &[u8],
        session: DiagSession,
        response: &mut [u8],
    ) -> Result<usize, Nrc> {
        for job in self.jobs {
            match job.verify(request, session) {
                DiagResult::Ok => return job.process(request, response),
                DiagResult::Nrc(nrc) => return Err(nrc),
                DiagResult::NotResponsible => {}

            }
        }
        Err(Nrc::ServiceNotSupported)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Mock job helpers ---

    struct MockJob {
        prefix: &'static [u8],
        session_mask: SessionMask,
        response_byte: u8,
    }

    impl DiagJob for MockJob {
        fn implemented_request(&self) -> &[u8] {
            self.prefix
        }
        fn session_mask(&self) -> SessionMask {
            self.session_mask
        }
        fn process(&self, _request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
            if response.is_empty() {
                return Err(Nrc::ResponseTooLong);
            }
            response[0] = self.response_byte;
            Ok(1)
        }
    }

    struct ErrorJob {
        prefix: &'static [u8],
        nrc: Nrc,
    }

    impl DiagJob for ErrorJob {
        fn implemented_request(&self) -> &[u8] {
            self.prefix
        }
        fn process(&self, _request: &[u8], _response: &mut [u8]) -> Result<usize, Nrc> {
            Err(self.nrc)
        }
    }

    // --- verify tests ---

    #[test]
    fn verify_prefix_match() {
        let job = MockJob { prefix: &[0x22], session_mask: SessionMask::ALL, response_byte: 0x62 };
        let result = job.verify(&[0x22, 0xF1, 0x90], DiagSession::Default);
        assert_eq!(result, DiagResult::Ok);
    }

    #[test]
    fn verify_prefix_mismatch() {
        let job = MockJob { prefix: &[0x22], session_mask: SessionMask::ALL, response_byte: 0x62 };
        let result = job.verify(&[0x27, 0x01], DiagSession::Default);
        assert_eq!(result, DiagResult::NotResponsible);
    }

    #[test]
    fn verify_too_short() {
        let job = MockJob { prefix: &[0x22, 0xF1], session_mask: SessionMask::ALL, response_byte: 0x62 };
        // Only 1 byte — shorter than the 2-byte prefix.
        let result = job.verify(&[0x22], DiagSession::Default);
        assert_eq!(result, DiagResult::NotResponsible);
    }

    #[test]
    fn verify_session_denied() {
        let job = MockJob {
            prefix: &[0x27],
            session_mask: SessionMask::EXTENDED,
            response_byte: 0x67,
        };
        // Default session — not in EXTENDED mask.
        let result = job.verify(&[0x27, 0x01], DiagSession::Default);
        assert_eq!(result, DiagResult::Nrc(Nrc::ServiceNotSupportedInActiveSession));
    }

    #[test]
    fn verify_session_allowed() {
        let job = MockJob {
            prefix: &[0x27],
            session_mask: SessionMask::EXTENDED,
            response_byte: 0x67,
        };
        let result = job.verify(&[0x27, 0x01], DiagSession::Extended);
        assert_eq!(result, DiagResult::Ok);
    }

    // --- dispatch tests ---

    #[test]
    fn dispatch_first_match() {
        let job = MockJob { prefix: &[0x3E], session_mask: SessionMask::ALL, response_byte: 0x7E };
        let jobs: &[&dyn DiagJob] = &[&job];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let result = router.dispatch(&[0x3E, 0x00], DiagSession::Default, &mut buf);
        assert_eq!(result, Ok(1));
        assert_eq!(buf[0], 0x7E);
    }

    #[test]
    fn dispatch_no_match_returns_service_not_supported() {
        let job = MockJob { prefix: &[0x22], session_mask: SessionMask::ALL, response_byte: 0x62 };
        let jobs: &[&dyn DiagJob] = &[&job];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        // Send 0x27 — no job matches.
        let result = router.dispatch(&[0x27, 0x01], DiagSession::Default, &mut buf);
        assert_eq!(result, Err(Nrc::ServiceNotSupported));
    }

    #[test]
    fn dispatch_session_nrc_returned() {
        let job = MockJob {
            prefix: &[0x27],
            session_mask: SessionMask::EXTENDED,
            response_byte: 0x67,
        };
        let jobs: &[&dyn DiagJob] = &[&job];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let result = router.dispatch(&[0x27, 0x01], DiagSession::Default, &mut buf);
        assert_eq!(result, Err(Nrc::ServiceNotSupportedInActiveSession));
    }

    #[test]
    fn dispatch_multiple_jobs_first_wins() {
        let job_a = MockJob { prefix: &[0x22], session_mask: SessionMask::ALL, response_byte: 0xAA };
        let job_b = MockJob { prefix: &[0x22], session_mask: SessionMask::ALL, response_byte: 0xBB };
        let jobs: &[&dyn DiagJob] = &[&job_a, &job_b];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let result = router.dispatch(&[0x22, 0xF1, 0x90], DiagSession::Default, &mut buf);
        assert_eq!(result, Ok(1));
        // First matching job (job_a) wins.
        assert_eq!(buf[0], 0xAA);
    }

    #[test]
    fn dispatch_skips_non_matching_jobs() {
        let job_a = MockJob { prefix: &[0x11], session_mask: SessionMask::ALL, response_byte: 0x51 };
        let job_b = MockJob { prefix: &[0x3E], session_mask: SessionMask::ALL, response_byte: 0x7E };
        let jobs: &[&dyn DiagJob] = &[&job_a, &job_b];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        // 0x3E should skip job_a and match job_b.
        let result = router.dispatch(&[0x3E, 0x00], DiagSession::Default, &mut buf);
        assert_eq!(result, Ok(1));
        assert_eq!(buf[0], 0x7E);
    }

    #[test]
    fn dispatch_job_process_error_propagated() {
        let job = ErrorJob { prefix: &[0x85], nrc: Nrc::ConditionsNotCorrect };
        let jobs: &[&dyn DiagJob] = &[&job];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let result = router.dispatch(&[0x85, 0x01], DiagSession::Extended, &mut buf);
        assert_eq!(result, Err(Nrc::ConditionsNotCorrect));
    }

    #[test]
    fn diag_result_debug() {
        let s = format!("{:?}", DiagResult::Nrc(Nrc::GeneralReject));
        assert!(s.contains("Nrc"));
    }

    #[test]
    fn diag_result_not_responsible_is_not_ok() {
        assert_ne!(DiagResult::NotResponsible, DiagResult::Ok);
    }

    #[test]
    fn empty_router_returns_service_not_supported() {
        let jobs: &[&dyn DiagJob] = &[];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let result = router.dispatch(&[0x22, 0xF1, 0x90], DiagSession::Default, &mut buf);
        assert_eq!(result, Err(Nrc::ServiceNotSupported));
    }

    #[test]
    fn min_request_length_defaults_to_prefix_len() {
        let job = MockJob { prefix: &[0x22, 0xF1], session_mask: SessionMask::ALL, response_byte: 0x62 };
        assert_eq!(job.min_request_length(), 2);
    }
}
