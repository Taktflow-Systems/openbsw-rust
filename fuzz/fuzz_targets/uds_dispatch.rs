#![no_main]

use bsw_uds::{
    dispatcher::{AddressingRule, DiagnosticJob, Dispatcher, JobResult, RequestAddressing},
    DiagSession, Nrc, SessionMask,
};
use libfuzzer_sys::fuzz_target;

struct TesterPresent;

impl DiagnosticJob for TesterPresent {
    fn request_prefix(&self) -> &[u8] {
        &[0x3e, 0]
    }
    fn sessions(&self) -> SessionMask {
        SessionMask::ALL
    }
    fn addressing(&self) -> AddressingRule {
        AddressingRule::PhysicalAndFunctional
    }
    fn process(&mut self, request: &[u8], response: &mut [u8]) -> JobResult {
        if request.len() != 2 {
            return JobResult::Negative(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if response.len() < 2 {
            return JobResult::Negative(Nrc::ResponseTooLong);
        }
        response[..2].copy_from_slice(&[0x7e, 0]);
        JobResult::Positive(2)
    }
}

fuzz_target!(|data: &[u8]| {
    let mut job = TesterPresent;
    let mut jobs: [&mut dyn DiagnosticJob; 1] = [&mut job];
    let mut dispatcher = Dispatcher::<1>::new();
    let _ = dispatcher.register_transport(0);
    let session = match data.first().copied().unwrap_or(0) % 3 {
        0 => DiagSession::Default,
        1 => DiagSession::Programming,
        _ => DiagSession::Extended,
    };
    let addressing = if data.get(1).copied().unwrap_or(0) & 1 == 0 {
        RequestAddressing::Physical
    } else {
        RequestAddressing::Functional
    };
    let mut response = [0_u8; 64];
    let _ = dispatcher.dispatch(
        0,
        addressing,
        session,
        data.get(2..).unwrap_or_default(),
        &mut response,
        &mut jobs,
    );
});
