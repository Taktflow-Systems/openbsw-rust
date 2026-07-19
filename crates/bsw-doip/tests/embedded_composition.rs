//! Deterministic embedded composition: the portable `DoIP` entity, the
//! deterministic fake network backend, an injected clock, `bsw-lifecycle`
//! control, and the production shared UDS state of `bsw-reference-core`.
//!
//! `DiagnosticCore` is the single production UDS state/dispatcher used by
//! the POSIX and STM32 reference applications for DoCAN, `DoIP`, and the
//! console; these tests prove that the portable `DoIP` entity feeds the
//! same instance (no second `DoIP`, UDS, or diagnostic implementation).

use bsw_doip::{
    discovery::AnnouncementSchedule, DiagnosticPowerMode, DoIpApplication, DoIpEntity,
    EntityConfig, EntityState, EntityStatus, ProtocolVersion, TransportParameters,
    VehicleAnnouncement,
};
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::{fake::FakeStack, SocketApi, SocketId};
use bsw_lifecycle::{LifecycleComponent, LifecycleManager, TransitionResult};
use bsw_reference_core::{DiagnosticCore, DiagnosticTransport};
use bsw_time::Instant;
use bsw_transport::{TransportMessage, TransportResult};

const ENTITY: u16 = 0x0123;
const TESTER: u16 = 0x1234;
const ENTITY_IP: IpAddress = IpAddress::V4([10, 0, 0, 1]);
const TCP_PORT: u16 = 13400;
const PAYLOAD: usize = 128;

/// Production shared-UDS application: every `DoIP` diagnostic request is
/// dispatched through the one [`DiagnosticCore`] instance.
struct CoreApplication {
    core: DiagnosticCore,
    now: Instant,
    response: Option<TransportMessage<PAYLOAD>>,
}

impl CoreApplication {
    fn new() -> Self {
        Self {
            core: DiagnosticCore::new(),
            now: Instant::from_nanos(0),
            response: None,
        }
    }
}

impl bsw_doip::DiagnosticMessageListener<PAYLOAD> for CoreApplication {
    fn message_received(&mut self, message: &TransportMessage<PAYLOAD>) -> TransportResult {
        let reply = self
            .core
            .dispatch_at(DiagnosticTransport::DoIp, message.payload(), self.now);
        let mut response = TransportMessage::new();
        response.set_source_address(message.target_address());
        response.set_target_address(message.source_address());
        if response.append(reply.bytes()).is_err() {
            return TransportResult::BufferFull;
        }
        self.response = Some(response);
        TransportResult::Ok
    }
}

impl DoIpApplication<PAYLOAD> for CoreApplication {
    fn take_response(&mut self) -> Option<TransportMessage<PAYLOAD>> {
        self.response.take()
    }
}

type Stack = FakeStack<16, 512, 32>;
type Entity = DoIpEntity<CoreApplication, bsw_doip::DefaultActivationPolicy, 4, 2, 2, PAYLOAD>;

/// The embedded `DoIP` system: portable entity plus its network backend,
/// composed as one `bsw-lifecycle` component.
struct EmbeddedDoIpSystem {
    entity: Entity,
    stack: Stack,
}

impl EmbeddedDoIpSystem {
    fn new() -> Self {
        Self {
            entity: Entity::new(
                EntityConfig {
                    tcp_address: ENTITY_IP,
                    tcp_port: TCP_PORT,
                    udp_address: ENTITY_IP,
                    udp_port: 13400,
                    source_bus: 7,
                },
                ProtocolVersion::Iso2012,
                ENTITY,
                TransportParameters {
                    max_payload_size: PAYLOAD as u32,
                    ..TransportParameters::default()
                },
                bsw_doip::DiscoveryEntity::new(
                    VehicleAnnouncement {
                        vin: *b"TESTVIN0000000001",
                        logical_address: ENTITY,
                        eid: [2, 0, 0, 0, 0, 1],
                        gid: [1, 2, 3, 4, 5, 6],
                        further_action_required: 0,
                        sync_status: Some(0),
                    },
                    EntityStatus {
                        node_type: 1,
                        max_sockets: 5,
                        open_sockets: 0,
                        max_data_size: Some(PAYLOAD as u32),
                    },
                    DiagnosticPowerMode::Ready,
                    AnnouncementSchedule {
                        initial_delay: bsw_time::Duration::from_millis(10).unwrap(),
                        interval: bsw_time::Duration::from_millis(10).unwrap(),
                        count: 3,
                    },
                ),
                CoreApplication::new(),
                bsw_doip::DefaultActivationPolicy,
            ),
            stack: Stack::new(),
        }
    }

    fn poll(&mut self, now: Instant) {
        self.entity.application_mut().now = now;
        self.entity.poll(now, &mut self.stack).expect("poll");
    }
}

impl LifecycleComponent for EmbeddedDoIpSystem {
    fn init(&mut self) -> TransitionResult {
        match self.entity.start(&mut self.stack) {
            Ok(()) => TransitionResult::Done,
            Err(_) => TransitionResult::Error,
        }
    }

    fn run(&mut self) -> TransitionResult {
        match self.entity.enable(Instant::from_nanos(0)) {
            Ok(()) => TransitionResult::Done,
            Err(_) => TransitionResult::Error,
        }
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.entity.stop(&mut self.stack);
        TransitionResult::Done
    }

    fn name(&self) -> &str {
        "EmbeddedDoIpSystem"
    }
}

// ── protocol helpers ─────────────────────────────────────────────────────────

fn routing_activation(source: u16) -> [u8; 15] {
    let source = source.to_be_bytes();
    [
        0x02, 0xfd, 0x00, 0x05, 0, 0, 0, 7, source[0], source[1], 0, 0, 0, 0, 0,
    ]
}

fn diagnostic_request(payload: &[u8]) -> Vec<u8> {
    let length = (4 + payload.len()) as u32;
    let mut frame = vec![0x02, 0xfd, 0x80, 0x01];
    frame.extend_from_slice(&length.to_be_bytes());
    frame.extend_from_slice(&TESTER.to_be_bytes());
    frame.extend_from_slice(&ENTITY.to_be_bytes());
    frame.extend_from_slice(payload);
    frame
}

/// Run one UDS request over `DoIP` and return the UDS response payload.
fn uds_over_doip(
    system: &mut EmbeddedDoIpSystem,
    client: SocketId,
    now_ms: &mut u64,
    request: &[u8],
) -> Vec<u8> {
    system
        .stack
        .send(client, &diagnostic_request(request))
        .expect("send");
    let mut bytes = Vec::new();
    for _ in 0..10 {
        *now_ms += 1;
        system.poll(Instant::from_nanos(*now_ms * 1_000_000));
        let mut chunk = [0u8; 256];
        let read = system.stack.recv(client, &mut chunk).expect("recv");
        bytes.extend_from_slice(&chunk[..read]);
    }
    // Skip the positive acknowledgement (13 bytes + up to 4 echo bytes),
    // then strip the 12-byte diagnostic-response prefix.
    let ack_len = 13 + request.len().min(4);
    assert!(bytes.len() > ack_len + 12, "no response in {bytes:02x?}");
    assert_eq!(bytes[2..4], [0x80, 0x02], "expected ACK first");
    bytes.split_off(ack_len)[12..].to_vec()
}

fn connect_and_activate(system: &mut EmbeddedDoIpSystem, now_ms: &mut u64) -> SocketId {
    let client = system.stack.create().expect("client");
    system
        .stack
        .connect(client, ENTITY_IP, TCP_PORT)
        .expect("connect");
    for _ in 0..2 {
        *now_ms += 1;
        system.poll(Instant::from_nanos(*now_ms * 1_000_000));
    }
    system
        .stack
        .send(client, &routing_activation(TESTER))
        .expect("send");
    let mut bytes = Vec::new();
    for _ in 0..5 {
        *now_ms += 1;
        system.poll(Instant::from_nanos(*now_ms * 1_000_000));
        let mut chunk = [0u8; 64];
        let read = system.stack.recv(client, &mut chunk).expect("recv");
        bytes.extend_from_slice(&chunk[..read]);
    }
    assert_eq!(bytes[12], 0x10, "activation failed: {bytes:02x?}");
    client
}

// ── tests ────────────────────────────────────────────────────────────────────

#[test]
fn docan_and_doip_share_one_production_uds_state() {
    let mut system = EmbeddedDoIpSystem::new();
    assert_eq!(system.init(), TransitionResult::Done);
    assert_eq!(system.run(), TransitionResult::Done);
    let mut now_ms = 0u64;
    let client = connect_and_activate(&mut system, &mut now_ms);

    // DoCAN enters the extended session on the shared production core
    // (this is the production DoCAN dispatch entry point).
    let docan = system.entity.application_mut().core.dispatch_at(
        DiagnosticTransport::DoCan,
        &[0x10, 0x03],
        Instant::from_nanos(now_ms * 1_000_000),
    );
    assert_eq!(docan.bytes()[..2], [0x50, 0x03]);

    // DoIP observes the DoCAN-selected session through the same state.
    let response = uds_over_doip(&mut system, client, &mut now_ms, &[0x22, 0xcf, 0x02]);
    assert_eq!(response, [0x62, 0xcf, 0x02, 0x03]);

    // DoIP writes a DID in the extended session; DoCAN reads it back.
    let response = uds_over_doip(
        &mut system,
        client,
        &mut now_ms,
        &[0x2e, 0xcf, 0x03, 0xAA, 0xBB],
    );
    assert_eq!(response, [0x6e, 0xcf, 0x03]);
    let docan = system.entity.application_mut().core.dispatch_at(
        DiagnosticTransport::DoCan,
        &[0x22, 0xcf, 0x03],
        Instant::from_nanos(now_ms * 1_000_000),
    );
    assert_eq!(docan.bytes(), [0x62, 0xcf, 0x03, 0xAA, 0xBB]);

    // One shared dispatcher counted every request from both transports.
    assert_eq!(system.entity.application().core.dispatch_count(), 4);
    assert_eq!(system.shutdown(), TransitionResult::Done);
}

#[test]
fn lifecycle_manager_runs_stop_restart_and_network_loss_recovery() {
    let mut system = EmbeddedDoIpSystem::new();
    let mut manager: LifecycleManager<1> = LifecycleManager::new();
    let index = manager.register().expect("register");
    let mut now_ms = 0u64;

    // Start and run under lifecycle control.
    assert_eq!(
        manager.init_component(index, &mut system),
        TransitionResult::Done
    );
    assert_eq!(
        manager.run_component(index, &mut system),
        TransitionResult::Done
    );
    assert_eq!(system.entity.state(), EntityState::Running);
    let client = connect_and_activate(&mut system, &mut now_ms);
    let response = uds_over_doip(&mut system, client, &mut now_ms, &[0x3e, 0x00]);
    assert_eq!(response, [0x7e, 0x00]);

    // Network loss: connections are torn down, bounded state reset.
    system.stack.set_link_up(false);
    for _ in 0..3 {
        now_ms += 1;
        system.poll(Instant::from_nanos(now_ms * 1_000_000));
    }
    assert_eq!(system.entity.connection_count(), 0);
    assert_eq!(system.entity.receive_pool_in_use(), 0);
    assert_eq!(system.entity.send_jobs_in_use(), 0);
    assert_eq!(system.entity.state(), EntityState::Running);

    // Recovery: the link returns and full service resumes.
    system.stack.set_link_up(true);
    let client = connect_and_activate(&mut system, &mut now_ms);
    let response = uds_over_doip(&mut system, client, &mut now_ms, &[0x3e, 0x00]);
    assert_eq!(response, [0x7e, 0x00]);

    // Stop and restart under lifecycle control; every bounded resource is
    // released or deterministically reinitialized.
    assert_eq!(
        manager.shutdown_component(index, &mut system),
        TransitionResult::Done
    );
    assert_eq!(system.entity.state(), EntityState::Stopped);
    assert_eq!(system.entity.tcp_port(), 0);
    assert_eq!(system.entity.receive_pool_in_use(), 0);
    assert_eq!(system.entity.send_jobs_in_use(), 0);

    assert_eq!(system.init(), TransitionResult::Done);
    assert_eq!(system.run(), TransitionResult::Done);
    let client = connect_and_activate(&mut system, &mut now_ms);
    let response = uds_over_doip(&mut system, client, &mut now_ms, &[0x3e, 0x00]);
    assert_eq!(response, [0x7e, 0x00]);
    // The production UDS state survives the transport restart (one core).
    assert!(system.entity.application().core.dispatch_count() >= 3);
    assert_eq!(system.shutdown(), TransitionResult::Done);
}
