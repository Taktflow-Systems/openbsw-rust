//! POSIX DoCAN and DoIP adapters around the shared diagnostic core.

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

pub use bsw_reference_core::{
    DiagnosticCore, DiagnosticResponse, DiagnosticTransport, DIAG_RESPONSE_CAPACITY,
};

use crate::config::DiagnosticConfig;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClientToken {
    slot: usize,
    generation: u32,
}

/// Fixed-capacity POSIX connection admission with stale-token rejection.
pub struct ClientLimiter<const N: usize> {
    active: [bool; N],
    generation: [u32; N],
    rejected: u32,
}

impl<const N: usize> ClientLimiter<N> {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            active: [false; N],
            generation: [0; N],
            rejected: 0,
        }
    }

    pub fn try_open(&mut self) -> Option<ClientToken> {
        let Some(slot) = self.active.iter().position(|active| !active) else {
            self.rejected = self.rejected.saturating_add(1);
            return None;
        };
        self.active[slot] = true;
        self.generation[slot] = self.generation[slot].wrapping_add(1);
        Some(ClientToken {
            slot,
            generation: self.generation[slot],
        })
    }

    pub fn close(&mut self, token: ClientToken) -> bool {
        if token.slot >= N
            || !self.active[token.slot]
            || self.generation[token.slot] != token.generation
        {
            return false;
        }
        self.active[token.slot] = false;
        true
    }

    #[must_use]
    pub fn active(&self) -> usize {
        self.active.iter().filter(|&&active| active).count()
    }

    #[must_use]
    pub const fn rejected(&self) -> u32 {
        self.rejected
    }
}

impl<const N: usize> Default for ClientLimiter<N> {
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
            return Ok(DiagnosticResponse::from_bytes(message.payload()));
        }
        now = now.wrapping_add(step);
    }
    Err(SessionError::Timeout(bsw_docan::transport::TimerKind::NCr))
}

/// Decode a DoIP diagnostic packet and dispatch through the same UDS state.
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
