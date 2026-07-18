//! One UDS state machine shared by DoCAN and DoIP frontends.

use bsw_can::transceiver::{CanTransceiver, ErrorCode};
use bsw_can::virtual_bus::VirtualCanBus;
use bsw_docan::addressing::{
    Addressing, ConnectionInfo, DataLinkAddressPair, TransportAddressPair,
};
use bsw_docan::parameters::Parameters;
use bsw_docan::transport::{ConnectionConfig, SessionError};
use bsw_docan::virtual_transport::VirtualCanTransport;
use bsw_doip::{DiagnosticPayload, Packet, Payload, ProtocolVersion};
use bsw_time::{Duration, Instant};
use bsw_uds::{DiagRouter, DiagSession, EcuReset, Nrc, SessionMask, TesterPresent};

use crate::config::DiagnosticConfig;

pub const DIAG_RESPONSE_CAPACITY: usize = 128;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagnosticTransport {
    DoCan,
    DoIp,
    Console,
}

/// Fixed-capacity admission control for concurrent diagnostic clients.
pub struct ClientLimiter<const N: usize> {
    active: [bool; N],
    rejected: u64,
}

impl<const N: usize> ClientLimiter<N> {
    pub const fn new() -> Self {
        Self {
            active: [false; N],
            rejected: 0,
        }
    }

    pub fn try_open(&mut self) -> Option<u16> {
        let Some(index) = self.active.iter().position(|active| !active) else {
            self.rejected = self.rejected.saturating_add(1);
            return None;
        };
        self.active[index] = true;
        Some(index as u16)
    }

    pub fn close(&mut self, token: u16) -> bool {
        let Some(active) = self.active.get_mut(usize::from(token)) else {
            return false;
        };
        let was_active = *active;
        *active = false;
        was_active
    }

    pub fn active(&self) -> usize {
        self.active.iter().filter(|active| **active).count()
    }

    pub const fn rejected(&self) -> u64 {
        self.rejected
    }
}

impl<const N: usize> Default for ClientLimiter<N> {
    fn default() -> Self {
        Self::new()
    }
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

    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Stateful UDS dispatcher owned exactly once by the application.
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

    /// Dispatch at an injected instant; transports never own UDS state.
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

    pub const fn session(&self) -> DiagSession {
        self.session
    }

    pub const fn dispatch_count(&self) -> u64 {
        self.dispatch_count
    }

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

fn transport_config(config: DiagnosticConfig, server: bool) -> ConnectionConfig {
    let (rx, tx, source, target) = if server {
        (
            config.can_request_id,
            config.can_response_id,
            config.tester_address,
            config.entity_address,
        )
    } else {
        (
            config.can_response_id,
            config.can_request_id,
            config.entity_address,
            config.tester_address,
        )
    };
    ConnectionConfig {
        connection: ConnectionInfo::new(
            TransportAddressPair::new(source, target),
            DataLinkAddressPair::new(rx, tx),
        ),
        addressing: Addressing::normal(),
        frame_size: 8,
        filler_byte: 0xcc,
        block_size: 15,
        parameters: Parameters {
            wait_allocate_timeout_us: 1_000_000,
            wait_rx_timeout_us: 1_000_000,
            wait_tx_callback_timeout_us: 1_000_000,
            wait_flow_control_timeout_us: 1_000_000,
            wait_consecutive_send_timeout_us: 1_000_000,
            max_allocate_retry_count: 15,
            max_flow_control_wait_count: 15,
            min_separation_time_us: 200,
            max_block_size: 15,
        },
    }
}

fn open_node(bus: &VirtualCanBus) -> Result<bsw_can::virtual_bus::VirtualNode, SessionError> {
    let mut node = bus.add_node();
    if node.init() != ErrorCode::Ok || node.open() != ErrorCode::Ok {
        return Err(SessionError::LinkFailure);
    }
    Ok(node)
}

/// Perform a complete ISO-TP request/response over `VirtualCanBus`.
pub fn virtual_can_request(
    core: &mut DiagnosticCore,
    config: DiagnosticConfig,
    request: &[u8],
    start: Instant,
) -> Result<DiagnosticResponse, SessionError> {
    let bus = VirtualCanBus::new();
    let tester_node = open_node(&bus)?;
    let server_node = open_node(&bus)?;
    let mut tester: VirtualCanTransport<256> =
        VirtualCanTransport::new(tester_node, transport_config(config, false));
    let mut server: VirtualCanTransport<256> =
        VirtualCanTransport::new(server_node, transport_config(config, true));
    tester.send(request)?;
    let mut now = start;
    let step = Duration::from_micros(100).expect("100 us is representable");
    let mut sent_response = false;
    for _ in 0..20_000 {
        tester.cycle(now)?;
        bus.step();
        server.cycle(now)?;
        bus.step();
        if !sent_response {
            if let Some(message) = server.take_received() {
                let response = core.dispatch_at(DiagnosticTransport::DoCan, message.payload(), now);
                server.send(response.bytes())?;
                sent_response = true;
            }
        }
        if let Some(message) = tester.take_received() {
            return Ok(DiagnosticResponse::new(message.payload()));
        }
        now = now.wrapping_add(step);
    }
    Err(SessionError::Timeout(bsw_docan::transport::TimerKind::NCr))
}

/// Decode one DoIP diagnostic packet, call the shared UDS core, and encode
/// the response as a diagnostic message with swapped logical addresses.
pub fn doip_request(
    core: &mut DiagnosticCore,
    wire: &[u8],
    now: Instant,
    output: &mut [u8],
) -> Result<usize, bsw_doip::CodecError> {
    let packet = Packet::parse(wire)?;
    let Payload::DiagnosticMessage(message) = packet.payload else {
        return Err(bsw_doip::CodecError::InvalidPayload);
    };
    let response = core.dispatch_at(DiagnosticTransport::DoIp, message.data, now);
    Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::DiagnosticMessage(DiagnosticPayload {
            source_address: message.target_address,
            target_address: message.source_address,
            data: response.bytes(),
        }),
    }
    .encode(output)
}
