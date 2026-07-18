//! Stateful standard UDS services and DEM protocol views (E19/E21/E23).

use bsw_time::Instant;

use crate::dem::DemManager;
use crate::state::{DiagnosticState, KeyAlgorithm, StatePersistence};
use crate::{DiagSession, Nrc};

/// Directional communication permission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommunicationMode {
    /// Receive and transmit enabled.
    ReceiveAndTransmit,
    /// Receive enabled; transmit disabled.
    ReceiveOnly,
    /// Receive disabled; transmit enabled.
    TransmitOnly,
    /// Receive and transmit disabled.
    Disabled,
}

/// Runtime communication-control state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommunicationState {
    /// Normal application message permissions.
    pub normal: CommunicationMode,
    /// Network-management message permissions.
    pub network: CommunicationMode,
}

impl CommunicationState {
    /// All communication enabled.
    pub const ENABLED: Self = Self {
        normal: CommunicationMode::ReceiveAndTransmit,
        network: CommunicationMode::ReceiveAndTransmit,
    };
}

/// Reset action emitted only after the positive response was processed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetAction {
    Hard,
    KeyOffOn,
    Soft,
}

/// Side effect requested by a core service.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServiceEffect {
    None,
    SetDtcRecording(bool),
}

/// Core stateful services sharing one diagnostic state machine.
pub struct CoreServices<P, K> {
    /// Session, authentication, and pluggable security state.
    pub state: DiagnosticState<P, K>,
    communication: CommunicationState,
    pending_reset: Option<ResetAction>,
    confirmed_reset: Option<ResetAction>,
    p2_ms: u16,
    p2_star_10ms: u16,
}

impl<P: StatePersistence, K: KeyAlgorithm> CoreServices<P, K> {
    /// Construct the core service collection.
    pub const fn new(state: DiagnosticState<P, K>, p2_ms: u16, p2_star_10ms: u16) -> Self {
        Self {
            state,
            communication: CommunicationState::ENABLED,
            pending_reset: None,
            confirmed_reset: None,
            p2_ms,
            p2_star_10ms,
        }
    }

    /// Current communication-control state.
    pub const fn communication(&self) -> CommunicationState {
        self.communication
    }

    /// Process TesterPresent, SessionControl, ControlDTCSetting,
    /// CommunicationControl, ECUReset, or SecurityAccess.
    pub fn process(
        &mut self,
        request: &[u8],
        now: Instant,
        security_seed: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, ServiceEffect), Nrc> {
        let Some(&sid) = request.first() else {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        };
        match sid {
            0x3e => Self::tester_present(request, response),
            0x10 => self.session_control(request, response),
            0x85 => Self::control_dtc(request, response),
            0x28 => self.communication_control(request, response),
            0x11 => self.ecu_reset(request, response),
            0x27 => self.security_access(request, now, security_seed, response),
            _ => Err(Nrc::ServiceNotSupported),
        }
    }

    /// Confirm that the transport processed the last positive response. ECU
    /// reset is deliberately armed here, never before response confirmation.
    pub fn response_processed(&mut self) {
        self.confirmed_reset = self.pending_reset.take();
    }

    /// Take one deferred reset action.
    pub fn take_reset(&mut self) -> Option<ResetAction> {
        self.confirmed_reset.take()
    }

    fn tester_present(request: &[u8], response: &mut [u8]) -> Result<(usize, ServiceEffect), Nrc> {
        exact_subfunction(request, 0)?;
        write(response, &[0x7e, 0]).map(|length| (length, ServiceEffect::None))
    }

    fn session_control(
        &mut self,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, ServiceEffect), Nrc> {
        if request.len() != 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let session =
            DiagSession::from_byte(request[1] & 0x7f).ok_or(Nrc::SubFunctionNotSupported)?;
        if !self.state.set_session(session) {
            return Err(Nrc::GeneralProgrammingFailure);
        }
        let p2 = self.p2_ms.to_be_bytes();
        let p2_star = self.p2_star_10ms.to_be_bytes();
        let length = write(
            response,
            &[
                0x50,
                session.as_byte(),
                p2[0],
                p2[1],
                p2_star[0],
                p2_star[1],
            ],
        )?;
        Ok((length, ServiceEffect::None))
    }

    fn control_dtc(request: &[u8], response: &mut [u8]) -> Result<(usize, ServiceEffect), Nrc> {
        if request.len() != 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let enabled = match request[1] & 0x7f {
            1 => true,
            2 => false,
            _ => return Err(Nrc::SubFunctionNotSupported),
        };
        let length = write(response, &[0xc5, request[1] & 0x7f])?;
        Ok((length, ServiceEffect::SetDtcRecording(enabled)))
    }

    fn communication_control(
        &mut self,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, ServiceEffect), Nrc> {
        if request.len() != 3 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let control = request[1] & 0x7f;
        let mode = match control {
            0 => CommunicationMode::ReceiveAndTransmit,
            1 => CommunicationMode::ReceiveOnly,
            2 => CommunicationMode::TransmitOnly,
            3 => CommunicationMode::Disabled,
            _ => return Err(Nrc::SubFunctionNotSupported),
        };
        match request[2] & 0x0f {
            1 => {
                self.communication.normal = mode;
            }
            2 => {
                self.communication.network = mode;
            }
            3 => {
                self.communication.normal = mode;
                self.communication.network = mode;
            }
            _ => return Err(Nrc::RequestOutOfRange),
        }
        let length = write(response, &[0x68, control])?;
        Ok((length, ServiceEffect::None))
    }

    fn ecu_reset(
        &mut self,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, ServiceEffect), Nrc> {
        if request.len() != 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let reset = match request[1] & 0x7f {
            1 => ResetAction::Hard,
            2 => ResetAction::KeyOffOn,
            3 => ResetAction::Soft,
            _ => return Err(Nrc::SubFunctionNotSupported),
        };
        self.pending_reset = Some(reset);
        let length = write(response, &[0x51, request[1] & 0x7f])?;
        Ok((length, ServiceEffect::None))
    }

    fn security_access(
        &mut self,
        request: &[u8],
        now: Instant,
        security_seed: &[u8],
        response: &mut [u8],
    ) -> Result<(usize, ServiceEffect), Nrc> {
        if request.len() < 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let level = request[1] & 0x7f;
        if level & 1 != 0 {
            if request.len() != 2 {
                return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
            }
            if response.len() < 2 {
                return Err(Nrc::ResponseTooLong);
            }
            response[..2].copy_from_slice(&[0x67, level]);
            let length = self
                .state
                .issue_seed(level, security_seed, now, &mut response[2..])?;
            Ok((2 + length, ServiceEffect::None))
        } else {
            if request.len() < 3 {
                return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
            }
            self.state.submit_key(level, &request[2..], now)?;
            let length = write(response, &[0x67, level])?;
            Ok((length, ServiceEffect::None))
        }
    }
}

/// Apply a ControlDTCSetting effect to the DEM.
pub fn apply_effect<const N: usize>(effect: ServiceEffect, dem: &mut DemManager<N>) {
    if let ServiceEffect::SetDtcRecording(enabled) = effect {
        dem.set_dtc_setting(enabled);
    }
}

/// Process ClearDiagnosticInformation with exact DTC group semantics supported
/// by the bounded DEM (all or exact 24-bit group/code).
pub fn clear_diagnostic_information<const N: usize>(
    request: &[u8],
    dem: &mut DemManager<N>,
    response: &mut [u8],
) -> Result<usize, Nrc> {
    if request.len() != 4 {
        return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
    }
    let group = u32::from_be_bytes([0, request[1], request[2], request[3]]);
    if group != 0xff_ffff && dem.get(group).is_none() {
        return Err(Nrc::RequestOutOfRange);
    }
    dem.clear_group(group);
    write(response, &[0x54])
}

/// Process the mandatory ReadDTCInformation status/count/list subfunctions.
pub fn read_dtc_information<const N: usize>(
    request: &[u8],
    dem: &DemManager<N>,
    response: &mut [u8],
) -> Result<usize, Nrc> {
    if request.len() != 3 {
        return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
    }
    let subfunction = request[1] & 0x7f;
    let requested_mask = request[2];
    let available_mask = 0xff;
    match subfunction {
        0x01 => {
            let count = dem.get_by_status_mask(requested_mask).count();
            let count = u16::try_from(count)
                .map_err(|_| Nrc::ResponseTooLong)?
                .to_be_bytes();
            write(
                response,
                &[0x59, 0x01, available_mask, 0x01, count[0], count[1]],
            )
        }
        0x02 | 0x0a => {
            if response.len() < 3 {
                return Err(Nrc::ResponseTooLong);
            }
            response[..3].copy_from_slice(&[0x59, subfunction, available_mask]);
            let mask = if subfunction == 0x0a {
                0xff
            } else {
                requested_mask
            };
            let mut written = 3;
            for entry in dem.get_by_status_mask(mask) {
                if response.len().saturating_sub(written) < 4 {
                    return Err(Nrc::ResponseTooLong);
                }
                let code = entry.code.to_be_bytes();
                response[written..written + 4].copy_from_slice(&[
                    code[1],
                    code[2],
                    code[3],
                    entry.status,
                ]);
                written += 4;
            }
            Ok(written)
        }
        _ => Err(Nrc::SubFunctionNotSupported),
    }
}

fn exact_subfunction(request: &[u8], expected: u8) -> Result<(), Nrc> {
    if request.len() != 2 {
        return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
    }
    if request[1] & 0x7f != expected {
        return Err(Nrc::SubFunctionNotSupported);
    }
    Ok(())
}

fn write(output: &mut [u8], value: &[u8]) -> Result<usize, Nrc> {
    if output.len() < value.len() {
        return Err(Nrc::ResponseTooLong);
    }
    output[..value.len()].copy_from_slice(value);
    Ok(value.len())
}
