//! Diagnostic job-tree dispatcher and transport registration (E14/E15).

use crate::{DiagSession, Nrc, SessionMask};

/// Addressing type of an incoming diagnostic request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RequestAddressing {
    /// Request targets this ECU directly.
    Physical,
    /// Request uses a functional/broadcast address.
    Functional,
}

/// Addressing accepted by one job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressingRule {
    /// Physical requests only.
    PhysicalOnly,
    /// Both physical and functional requests.
    PhysicalAndFunctional,
}

/// Result produced by a diagnostic job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobResult {
    /// Positive response length already written into the response buffer.
    Positive(usize),
    /// Negative response code.
    Negative(Nrc),
    /// Work continues asynchronously; timing logic owns response-pending.
    Pending,
}

/// Diagnostic job node.
pub trait DiagnosticJob {
    /// Request prefix represented by this job-tree node.
    fn request_prefix(&self) -> &[u8];

    /// Sessions in which the job is permitted.
    fn sessions(&self) -> SessionMask;

    /// Accepted request addressing.
    fn addressing(&self) -> AddressingRule {
        AddressingRule::PhysicalAndFunctional
    }

    /// Process a matched request.
    fn process(&mut self, request: &[u8], response: &mut [u8]) -> JobResult;
}

/// Dispatcher-level error that cannot be represented by a UDS response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchError {
    /// Request payload is empty.
    EmptyRequest,
    /// Source transport is not registered.
    TransportNotRegistered,
    /// Response buffer cannot hold the required negative response.
    ResponseBufferTooSmall,
}

/// Observable dispatcher outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchOutcome {
    /// A response of this length is ready.
    Response(usize),
    /// Response is intentionally suppressed by the request or functional rules.
    Suppressed,
    /// The selected job continues asynchronously.
    Pending,
}

/// Bounded transport registration and longest-prefix job dispatcher.
pub struct Dispatcher<const TRANSPORTS: usize> {
    transports: [Option<u8>; TRANSPORTS],
}

impl<const TRANSPORTS: usize> Dispatcher<TRANSPORTS> {
    /// Create a dispatcher without registered transports.
    pub const fn new() -> Self {
        Self {
            transports: [None; TRANSPORTS],
        }
    }

    /// Register one transport bus idempotently.
    pub fn register_transport(&mut self, bus_id: u8) -> bool {
        if self.transports.contains(&Some(bus_id)) {
            return true;
        }
        let Some(slot) = self.transports.iter_mut().find(|slot| slot.is_none()) else {
            return false;
        };
        *slot = Some(bus_id);
        true
    }

    /// Unregister a transport and reject its future traffic.
    pub fn unregister_transport(&mut self, bus_id: u8) -> bool {
        let Some(slot) = self
            .transports
            .iter_mut()
            .find(|slot| **slot == Some(bus_id))
        else {
            return false;
        };
        *slot = None;
        true
    }

    /// Dispatch using longest matching prefix; equal-length jobs retain
    /// registration/slice order, matching upstream job-tree ordering.
    pub fn dispatch(
        &self,
        source_bus: u8,
        addressing: RequestAddressing,
        session: DiagSession,
        request: &[u8],
        response: &mut [u8],
        jobs: &mut [&mut dyn DiagnosticJob],
    ) -> Result<DispatchOutcome, DispatchError> {
        let Some(&sid) = request.first() else {
            return Err(DispatchError::EmptyRequest);
        };
        if !self.transports.contains(&Some(source_bus)) {
            return Err(DispatchError::TransportNotRegistered);
        }
        let selected = jobs
            .iter()
            .enumerate()
            .filter(|(_, job)| request_matches(job.request_prefix(), request))
            .max_by_key(|(index, job)| (job.request_prefix().len(), usize::MAX - *index))
            .map(|(index, _)| index);

        let Some(index) = selected else {
            return Self::negative(addressing, sid, Nrc::ServiceNotSupported, response);
        };
        let job = &mut jobs[index];
        if !job.sessions().contains(session) {
            return Self::negative(
                addressing,
                sid,
                Nrc::ServiceNotSupportedInActiveSession,
                response,
            );
        }
        if addressing == RequestAddressing::Functional
            && job.addressing() == AddressingRule::PhysicalOnly
        {
            return Ok(DispatchOutcome::Suppressed);
        }

        match job.process(request, response) {
            JobResult::Positive(length) => {
                if suppress_positive(request) {
                    Ok(DispatchOutcome::Suppressed)
                } else {
                    Ok(DispatchOutcome::Response(length))
                }
            }
            JobResult::Negative(nrc) => Self::negative(addressing, sid, nrc, response),
            JobResult::Pending => Ok(DispatchOutcome::Pending),
        }
    }

    fn negative(
        addressing: RequestAddressing,
        sid: u8,
        nrc: Nrc,
        response: &mut [u8],
    ) -> Result<DispatchOutcome, DispatchError> {
        if addressing == RequestAddressing::Functional && suppress_functional_negative(nrc) {
            return Ok(DispatchOutcome::Suppressed);
        }
        if response.len() < 3 {
            return Err(DispatchError::ResponseBufferTooSmall);
        }
        response[..3].copy_from_slice(&[0x7f, sid, nrc.as_byte()]);
        Ok(DispatchOutcome::Response(3))
    }
}

impl<const TRANSPORTS: usize> Default for Dispatcher<TRANSPORTS> {
    fn default() -> Self {
        Self::new()
    }
}

fn request_matches(prefix: &[u8], request: &[u8]) -> bool {
    prefix.len() <= request.len()
        && prefix.iter().enumerate().all(|(index, expected)| {
            let actual = if index == 1 {
                request[index] & 0x7f
            } else {
                request[index]
            };
            actual == *expected
        })
}

fn suppress_positive(request: &[u8]) -> bool {
    request
        .get(1)
        .is_some_and(|subfunction| subfunction & 0x80 != 0)
}

fn suppress_functional_negative(nrc: Nrc) -> bool {
    matches!(
        nrc,
        Nrc::ServiceNotSupported
            | Nrc::SubFunctionNotSupported
            | Nrc::RequestOutOfRange
            | Nrc::SubFunctionNotSupportedInActiveSession
            | Nrc::ServiceNotSupportedInActiveSession
    )
}
