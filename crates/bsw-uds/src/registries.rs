//! Fixed-capacity DID, routine, and input/output registries (E20/E22).

use crate::{DiagSession, Nrc, SessionMask};

/// Data-identifier access implemented by application code.
pub trait DidHandler {
    /// Registered data identifier.
    fn did(&self) -> u16;
    /// Sessions allowed to read this DID.
    fn read_sessions(&self) -> SessionMask {
        SessionMask::ALL
    }
    /// Sessions allowed to write this DID.
    fn write_sessions(&self) -> SessionMask {
        SessionMask::NONE
    }
    /// Required unlocked security level, if any.
    fn security_level(&self) -> Option<u8> {
        None
    }
    /// Append the DID value and return its length.
    fn read(&mut self, output: &mut [u8]) -> Result<usize, Nrc>;
    /// Validate and durably apply a DID value.
    fn write(&mut self, _value: &[u8]) -> Result<(), Nrc> {
        Err(Nrc::RequestOutOfRange)
    }
}

/// Registry over an application-owned, statically sized handler slice.
pub struct DidRegistry<'a> {
    handlers: &'a mut [&'a mut dyn DidHandler],
}

impl<'a> DidRegistry<'a> {
    /// Create a registry. Duplicate identifiers are rejected.
    pub fn new(handlers: &'a mut [&'a mut dyn DidHandler]) -> Result<Self, Nrc> {
        for left in 0..handlers.len() {
            if handlers[(left + 1)..]
                .iter()
                .any(|right| right.did() == handlers[left].did())
            {
                return Err(Nrc::RequestOutOfRange);
            }
        }
        Ok(Self { handlers })
    }

    /// Process ReadDataByIdentifier, including a request containing multiple DIDs.
    pub fn read(
        &mut self,
        request: &[u8],
        session: DiagSession,
        unlocked_levels: u32,
        response: &mut [u8],
    ) -> Result<usize, Nrc> {
        if request.len() < 3 || request.len().is_multiple_of(2) {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if response.is_empty() {
            return Err(Nrc::ResponseTooLong);
        }
        response[0] = 0x62;
        let mut written = 1;
        for did_bytes in request[1..].chunks_exact(2) {
            let did = u16::from_be_bytes([did_bytes[0], did_bytes[1]]);
            let Some(handler) = self
                .handlers
                .iter_mut()
                .find(|handler| handler.did() == did)
            else {
                return Err(Nrc::RequestOutOfRange);
            };
            check_access(
                handler.read_sessions(),
                handler.security_level(),
                session,
                unlocked_levels,
            )?;
            if response.len().saturating_sub(written) < 2 {
                return Err(Nrc::ResponseTooLong);
            }
            response[written..written + 2].copy_from_slice(&did.to_be_bytes());
            written += 2;
            let length = handler.read(&mut response[written..])?;
            if length > response.len().saturating_sub(written) {
                return Err(Nrc::ResponseTooLong);
            }
            written += length;
        }
        Ok(written)
    }

    /// Process WriteDataByIdentifier. The handler owns validation and persistence.
    pub fn write(
        &mut self,
        request: &[u8],
        session: DiagSession,
        unlocked_levels: u32,
        response: &mut [u8],
    ) -> Result<usize, Nrc> {
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let did = u16::from_be_bytes([request[1], request[2]]);
        let Some(handler) = self
            .handlers
            .iter_mut()
            .find(|handler| handler.did() == did)
        else {
            return Err(Nrc::RequestOutOfRange);
        };
        check_access(
            handler.write_sessions(),
            handler.security_level(),
            session,
            unlocked_levels,
        )?;
        handler.write(&request[3..])?;
        if response.len() < 3 {
            return Err(Nrc::ResponseTooLong);
        }
        response[..3].copy_from_slice(&[0x6e, request[1], request[2]]);
        Ok(3)
    }
}

fn check_access(
    sessions: SessionMask,
    security_level: Option<u8>,
    session: DiagSession,
    unlocked_levels: u32,
) -> Result<(), Nrc> {
    if !sessions.contains(session) {
        return Err(Nrc::RequestOutOfRange);
    }
    if security_level.is_some_and(|level| level >= 32 || unlocked_levels & (1_u32 << level) == 0) {
        return Err(Nrc::SecurityAccessDenied);
    }
    Ok(())
}

/// Application-owned routine implementation.
pub trait RoutineHandler {
    /// Routine identifier.
    fn routine_id(&self) -> u16;
    /// Start and append optional status data.
    fn start(&mut self, input: &[u8], output: &mut [u8]) -> Result<usize, Nrc>;
    /// Stop and append optional status data.
    fn stop(&mut self, output: &mut [u8]) -> Result<usize, Nrc>;
    /// Read results and append status data.
    fn results(&mut self, output: &mut [u8]) -> Result<usize, Nrc>;
}

/// RoutineControl registry with no built-in or hard-coded routines.
pub struct RoutineRegistry<'a> {
    handlers: &'a mut [&'a mut dyn RoutineHandler],
}

impl<'a> RoutineRegistry<'a> {
    /// Create the registry.
    pub fn new(handlers: &'a mut [&'a mut dyn RoutineHandler]) -> Self {
        Self { handlers }
    }

    /// Process start, stop, and request-results subfunctions.
    pub fn process(&mut self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let subfunction = request[1] & 0x7f;
        let id = u16::from_be_bytes([request[2], request[3]]);
        let Some(handler) = self
            .handlers
            .iter_mut()
            .find(|handler| handler.routine_id() == id)
        else {
            return Err(Nrc::RequestOutOfRange);
        };
        if response.len() < 4 {
            return Err(Nrc::ResponseTooLong);
        }
        response[..4].copy_from_slice(&[0x71, subfunction, request[2], request[3]]);
        let length = match subfunction {
            1 => handler.start(&request[4..], &mut response[4..])?,
            2 => handler.stop(&mut response[4..])?,
            3 => handler.results(&mut response[4..])?,
            _ => return Err(Nrc::SubFunctionNotSupported),
        };
        if length > response.len().saturating_sub(4) {
            return Err(Nrc::ResponseTooLong);
        }
        Ok(4 + length)
    }
}

/// Application-owned InputOutputControl implementation.
pub trait IoControlHandler {
    /// Data identifier controlled by this handler.
    fn did(&self) -> u16;
    /// Apply the control parameter and append control-state data.
    fn control(&mut self, parameter: u8, input: &[u8], output: &mut [u8]) -> Result<usize, Nrc>;
}

/// InputOutputControlByIdentifier registry.
pub struct IoControlRegistry<'a> {
    handlers: &'a mut [&'a mut dyn IoControlHandler],
}

impl<'a> IoControlRegistry<'a> {
    /// Create the registry.
    pub fn new(handlers: &'a mut [&'a mut dyn IoControlHandler]) -> Self {
        Self { handlers }
    }

    /// Process one IO-control request.
    pub fn process(&mut self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let id = u16::from_be_bytes([request[1], request[2]]);
        let Some(handler) = self.handlers.iter_mut().find(|handler| handler.did() == id) else {
            return Err(Nrc::RequestOutOfRange);
        };
        if response.len() < 4 {
            return Err(Nrc::ResponseTooLong);
        }
        response[..4].copy_from_slice(&[0x6f, request[1], request[2], request[3]]);
        let length = handler.control(request[3], &request[4..], &mut response[4..])?;
        if length > response.len().saturating_sub(4) {
            return Err(Nrc::ResponseTooLong);
        }
        Ok(4 + length)
    }
}
