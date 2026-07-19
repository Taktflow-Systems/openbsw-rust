//! Deterministic behavioral suite for the portable embedded `DoIP` entity.
//!
//! Every test drives [`DoIpEntity`] — the same allocation-free core the
//! POSIX adapter wraps — over the deterministic in-memory
//! [`FakeStack`] backend with an injected clock. No operating system
//! networking, threads, or sleeps are involved; identical inputs always
//! produce identical observable byte sequences.

use bsw_doip::{
    discovery::AnnouncementSchedule, DiagnosticPowerMode, DoIpApplication, DoIpEntity,
    EntityConfig, EntityState, EntityStatus, Packet, Payload, ProtocolVersion, TransportParameters,
    VehicleAnnouncement, VehicleIdentification,
};
use bsw_ethernet::endpoint::IpEndpoint;
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::{
    fake::FakeStack, DatagramApi, DatagramId, SocketApi, SocketEvent, SocketId, SocketState,
};
use bsw_time::{Duration, Instant};
use bsw_transport::{TransportMessage, TransportResult};
use bsw_uds::{DiagJob, DiagRouter, DiagSession, SessionMask, TesterPresent};

const ENTITY: u16 = 0x0123;
const TESTER: u16 = 0x1234;
const ENTITY_IP: IpAddress = IpAddress::V4([10, 0, 0, 1]);
const CLIENT_IP: IpAddress = IpAddress::V4([10, 0, 0, 2]);
const TCP_PORT: u16 = 13400;
const UDP_PORT: u16 = 13400;
const ANNOUNCE_PORT: u16 = 13401;

// ── shared UDS application ───────────────────────────────────────────────────

#[derive(Default)]
struct UdsApplication {
    responses: std::collections::VecDeque<TransportMessage<64>>,
    requests: u32,
}

impl bsw_doip::DiagnosticMessageListener<64> for UdsApplication {
    fn message_received(&mut self, message: &TransportMessage<64>) -> TransportResult {
        self.requests += 1;
        let service = TesterPresent {
            session_mask: SessionMask::ALL,
        };
        let jobs: [&dyn DiagJob; 1] = [&service];
        let router = DiagRouter::new(&jobs);
        let mut bytes = [0; 64];
        let Ok(length) = router.dispatch(message.payload(), DiagSession::Default, &mut bytes)
        else {
            return TransportResult::Error;
        };
        let mut response = TransportMessage::new();
        response.set_source_address(message.target_address());
        response.set_target_address(message.source_address());
        if response.append(&bytes[..length]).is_err() {
            return TransportResult::BufferFull;
        }
        self.responses.push_back(response);
        TransportResult::Ok
    }
}

impl DoIpApplication<64> for UdsApplication {
    fn take_response(&mut self) -> Option<TransportMessage<64>> {
        self.responses.pop_front()
    }
}

// ── harness ──────────────────────────────────────────────────────────────────

type Stack = FakeStack<16, 512, 32>;
type Entity = DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 6, 6, 64>;

fn discovery_entity() -> bsw_doip::DiscoveryEntity {
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
            max_data_size: Some(64),
        },
        DiagnosticPowerMode::Ready,
        AnnouncementSchedule {
            initial_delay: Duration::from_millis(10).unwrap(),
            interval: Duration::from_millis(10).unwrap(),
            count: 3,
        },
    )
}

fn config() -> EntityConfig {
    EntityConfig {
        tcp_address: ENTITY_IP,
        tcp_port: TCP_PORT,
        udp_address: ENTITY_IP,
        udp_port: UDP_PORT,
        source_bus: 7,
    }
}

fn parameters() -> TransportParameters {
    TransportParameters {
        max_payload_size: 64,
        ..TransportParameters::default()
    }
}

fn new_entity() -> Entity {
    Entity::new(
        config(),
        ProtocolVersion::Iso2012,
        ENTITY,
        parameters(),
        discovery_entity(),
        UdsApplication::default(),
        bsw_doip::DefaultActivationPolicy,
    )
}

/// Start and enable a fresh entity at `t = 0`.
fn started(net: &mut Stack) -> Entity {
    let mut entity = new_entity();
    entity.start(net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    entity
}

/// Millisecond test clock.
struct TestClock {
    now_ms: u64,
    base: Instant,
}

impl TestClock {
    fn new() -> Self {
        Self {
            now_ms: 0,
            base: Instant::from_nanos(0),
        }
    }

    fn at(base: Instant) -> Self {
        Self { now_ms: 0, base }
    }

    fn now(&self) -> Instant {
        self.base
            .wrapping_add(Duration::from_millis(self.now_ms).unwrap())
    }

    fn advance(&mut self, ms: u64) {
        self.now_ms += ms;
    }
}

/// Poll `cycles` times, advancing one millisecond per cycle.
fn pump(entity: &mut Entity, net: &mut Stack, clock: &mut TestClock, cycles: u32) {
    for _ in 0..cycles {
        clock.advance(1);
        entity.poll(clock.now(), net).expect("poll");
    }
}

/// Connect one TCP client to the entity.
fn connect_client(net: &mut Stack) -> SocketId {
    let client = net.create().expect("client socket");
    net.connect(client, ENTITY_IP, TCP_PORT).expect("connect");
    client
}

/// Read every currently buffered byte of `client`.
fn drain_client(net: &mut Stack, client: SocketId) -> Vec<u8> {
    let mut bytes = Vec::new();
    let mut chunk = [0u8; 128];
    while let Ok(read) = net.recv(client, &mut chunk) {
        if read == 0 {
            break;
        }
        bytes.extend_from_slice(&chunk[..read]);
    }
    bytes
}

/// Bind a discovery client on `port`.
fn udp_client(net: &mut Stack, port: u16) -> DatagramId {
    let socket = net.create_datagram().expect("udp socket");
    net.bind_datagram(socket, CLIENT_IP, port)
        .expect("udp bind");
    socket
}

fn send_discovery(net: &mut Stack, socket: DatagramId, payload: Payload<'_>) {
    let mut request = [0u8; 64];
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload,
    }
    .encode(&mut request)
    .expect("encode");
    net.send_datagram(
        socket,
        IpEndpoint::new(ENTITY_IP, UDP_PORT),
        &request[..length],
    )
    .expect("send datagram");
}

fn recv_discovery(net: &mut Stack, socket: DatagramId) -> Option<Vec<u8>> {
    let mut response = [0u8; 128];
    net.recv_datagram(socket, &mut response)
        .expect("recv datagram")
        .map(|(length, _)| response[..length].to_vec())
}

// ── frame builders and expectations ──────────────────────────────────────────

fn routing_activation(source: u16, activation_type: u8) -> [u8; 15] {
    let source = source.to_be_bytes();
    [
        0x02,
        0xfd,
        0x00,
        0x05,
        0,
        0,
        0,
        7,
        source[0],
        source[1],
        activation_type,
        0,
        0,
        0,
        0,
    ]
}

fn activation_response(source: u16, code: u8) -> [u8; 17] {
    let source = source.to_be_bytes();
    let entity = ENTITY.to_be_bytes();
    [
        0x02, 0xfd, 0x00, 0x06, 0, 0, 0, 9, source[0], source[1], entity[0], entity[1], code, 0, 0,
        0, 0,
    ]
}

fn diagnostic_request(source: u16, target: u16, payload: &[u8]) -> Vec<u8> {
    let length = (4 + payload.len()) as u32;
    let mut frame = vec![0x02, 0xfd, 0x80, 0x01];
    frame.extend_from_slice(&length.to_be_bytes());
    frame.extend_from_slice(&source.to_be_bytes());
    frame.extend_from_slice(&target.to_be_bytes());
    frame.extend_from_slice(payload);
    frame
}

fn tester_present(source: u16) -> Vec<u8> {
    diagnostic_request(source, ENTITY, &[0x3e, 0x00])
}

/// Positive acknowledgement (0x8002) echoing up to four request bytes.
fn diagnostic_ack(source: u16, echoed: &[u8]) -> Vec<u8> {
    let echo = &echoed[..echoed.len().min(4)];
    let length = (5 + echo.len()) as u32;
    let mut frame = vec![0x02, 0xfd, 0x80, 0x02];
    frame.extend_from_slice(&length.to_be_bytes());
    frame.extend_from_slice(&ENTITY.to_be_bytes());
    frame.extend_from_slice(&source.to_be_bytes());
    frame.push(0x00);
    frame.extend_from_slice(echo);
    frame
}

/// Negative acknowledgement (0x8003): source/target echo the rejected
/// request's target/source and no payload bytes are echoed.
fn diagnostic_nack(source_field: u16, target_field: u16, code: u8) -> Vec<u8> {
    let mut frame = vec![0x02, 0xfd, 0x80, 0x03, 0, 0, 0, 5];
    frame.extend_from_slice(&source_field.to_be_bytes());
    frame.extend_from_slice(&target_field.to_be_bytes());
    frame.push(code);
    frame
}

fn uds_response(source: u16) -> Vec<u8> {
    let mut frame = vec![0x02, 0xfd, 0x80, 0x01, 0, 0, 0, 6];
    frame.extend_from_slice(&ENTITY.to_be_bytes());
    frame.extend_from_slice(&source.to_be_bytes());
    frame.extend_from_slice(&[0x7e, 0x00]);
    frame
}

fn generic_nack(code: u8) -> [u8; 9] {
    [0x02, 0xfd, 0x00, 0x00, 0, 0, 0, 1, code]
}

/// Connect and successfully activate one client.
fn activate(entity: &mut Entity, net: &mut Stack, clock: &mut TestClock, source: u16) -> SocketId {
    let client = connect_client(net);
    pump(entity, net, clock, 1);
    net.send(client, &routing_activation(source, 0))
        .expect("send");
    pump(entity, net, clock, 3);
    let bytes = drain_client(net, client);
    assert_eq!(bytes, activation_response(source, 0x10));
    client
}

/// `true` once the client observed its connection end.
fn client_closed(net: &mut Stack, client: SocketId) -> bool {
    !matches!(net.state(client), Ok(SocketState::Established))
}

// ── discovery over UDP ───────────────────────────────────────────────────────

#[test]
fn udp_vehicle_identification_request_response() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let udp = udp_client(&mut net, 40000);

    // Unfiltered request answers with the announcement record.
    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::All),
    );
    pump(&mut entity, &mut net, &mut clock, 1);
    let response = recv_discovery(&mut net, udp).expect("response");
    let packet = Packet::parse(&response).expect("parse");
    match packet.payload {
        Payload::VehicleAnnouncement(announcement) => {
            assert_eq!(announcement.vin, *b"TESTVIN0000000001");
            assert_eq!(announcement.logical_address, ENTITY);
        }
        other => panic!("unexpected payload {other:?}"),
    }

    // Matching EID answers; a foreign VIN stays silent.
    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::Eid([2, 0, 0, 0, 0, 1])),
    );
    pump(&mut entity, &mut net, &mut clock, 1);
    assert!(recv_discovery(&mut net, udp).is_some());
    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::Vin(*b"OTHERVIN000000002")),
    );
    pump(&mut entity, &mut net, &mut clock, 1);
    assert!(recv_discovery(&mut net, udp).is_none());
}

#[test]
fn scheduled_vehicle_announcements_hit_exact_deadlines() {
    let mut net = Stack::new();
    let mut entity = new_entity();
    entity.set_announcement_target(Some(IpEndpoint::new(
        IpAddress::V4([255, 255, 255, 255]),
        ANNOUNCE_PORT,
    )));
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut clock = TestClock::new();
    let listener = udp_client(&mut net, ANNOUNCE_PORT);

    // Initial delay 10 ms: nothing at 9 ms, first frame exactly at 10 ms.
    pump(&mut entity, &mut net, &mut clock, 9);
    assert!(recv_discovery(&mut net, listener).is_none());
    pump(&mut entity, &mut net, &mut clock, 1);
    let first = recv_discovery(&mut net, listener).expect("first announcement");
    assert!(matches!(
        Packet::parse(&first).expect("parse").payload,
        Payload::VehicleAnnouncement(_)
    ));

    // Interval 10 ms, count 3: two more frames, then silence.
    pump(&mut entity, &mut net, &mut clock, 10);
    assert!(recv_discovery(&mut net, listener).is_some());
    pump(&mut entity, &mut net, &mut clock, 10);
    assert!(recv_discovery(&mut net, listener).is_some());
    pump(&mut entity, &mut net, &mut clock, 50);
    assert!(recv_discovery(&mut net, listener).is_none());
}

#[test]
fn entity_status_request_response() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let udp = udp_client(&mut net, 40000);

    send_discovery(&mut net, udp, Payload::EntityStatusRequest);
    pump(&mut entity, &mut net, &mut clock, 1);
    let response = recv_discovery(&mut net, udp).expect("response");
    match Packet::parse(&response).expect("parse").payload {
        Payload::EntityStatusResponse(status) => {
            assert_eq!(status.node_type, 1);
            assert_eq!(status.max_sockets, 5);
            assert_eq!(status.open_sockets, 0);
            assert_eq!(status.max_data_size, Some(64));
        }
        other => panic!("unexpected payload {other:?}"),
    }
}

#[test]
fn diagnostic_power_mode_request_response() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let udp = udp_client(&mut net, 40000);

    send_discovery(&mut net, udp, Payload::PowerModeRequest);
    pump(&mut entity, &mut net, &mut clock, 1);
    let response = recv_discovery(&mut net, udp).expect("response");
    match Packet::parse(&response).expect("parse").payload {
        Payload::PowerModeResponse(mode) => assert_eq!(mode, DiagnosticPowerMode::Ready),
        other => panic!("unexpected payload {other:?}"),
    }
}

// ── TCP admission and routing activation ─────────────────────────────────────

#[test]
fn tcp_listen_and_accept() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    assert_eq!(entity.tcp_port(), TCP_PORT);
    assert_eq!(entity.discovery_port(), UDP_PORT);

    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);
    assert_eq!(entity.accepted_connections(), 1);
    assert_eq!(entity.connection_count(), 1);
    assert!(matches!(net.state(client), Ok(SocketState::Established)));
}

#[test]
fn routing_activation_success() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let _client = activate(&mut entity, &mut net, &mut clock, TESTER);
    assert_eq!(entity.routing_count(), 1);
}

#[test]
fn routing_activation_rejection_unsupported_type() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);
    net.send(client, &routing_activation(TESTER, 0x05))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    let bytes = drain_client(&mut net, client);
    assert_eq!(bytes, activation_response(TESTER, 0x06));
    pump(&mut entity, &mut net, &mut clock, 3);
    assert!(client_closed(&mut net, client));
    assert_eq!(entity.routing_count(), 0);
}

#[test]
fn fragmented_header_reception() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);

    let frame = routing_activation(TESTER, 0);
    // Header split 3 + 5; payload afterwards byte by byte.
    net.send(client, &frame[..3]).expect("send");
    pump(&mut entity, &mut net, &mut clock, 1);
    net.send(client, &frame[3..8]).expect("send");
    pump(&mut entity, &mut net, &mut clock, 1);
    assert!(drain_client(&mut net, client).is_empty());
    for byte in &frame[8..] {
        net.send(client, core::slice::from_ref(byte)).expect("send");
        pump(&mut entity, &mut net, &mut clock, 1);
    }
    pump(&mut entity, &mut net, &mut clock, 2);
    assert_eq!(
        drain_client(&mut net, client),
        activation_response(TESTER, 0x10)
    );
}

#[test]
fn fragmented_payload_reception() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    let request = tester_present(TESTER);
    net.send(client, &request[..8]).expect("send");
    pump(&mut entity, &mut net, &mut clock, 2);
    assert!(drain_client(&mut net, client).is_empty());
    net.send(client, &request[8..11]).expect("send");
    pump(&mut entity, &mut net, &mut clock, 2);
    net.send(client, &request[11..]).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);

    let mut expected = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(drain_client(&mut net, client), expected);
}

#[test]
fn multiple_frames_in_one_receive() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    // Two complete diagnostic frames arrive as one TCP byte stream.
    let mut burst = tester_present(TESTER);
    burst.extend_from_slice(&tester_present(TESTER));
    net.send(client, &burst).expect("send");
    pump(&mut entity, &mut net, &mut clock, 4);

    // Both acknowledgements are queued while the stream is parsed; the
    // shared-UDS responses follow.
    let mut expected = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    expected.extend_from_slice(&diagnostic_ack(TESTER, &[0x3e, 0x00]));
    expected.extend_from_slice(&uds_response(TESTER));
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(drain_client(&mut net, client), expected);
    assert_eq!(entity.application().requests, 2);
}

// ── diagnostic messages ──────────────────────────────────────────────────────

#[test]
fn shared_uds_request_response_with_positive_ack() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    net.send(client, &tester_present(TESTER)).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    let mut expected = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(drain_client(&mut net, client), expected);
    assert_eq!(entity.application().requests, 1);
    // Pools return to idle once the exchange completed.
    assert_eq!(entity.receive_pool_in_use(), 0);
    assert_eq!(entity.send_jobs_in_use(), 0);
}

#[test]
fn source_and_target_address_validation() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    // Unknown target: negative acknowledgement 0x03, connection stays.
    let request = diagnostic_request(TESTER, 0x0999, &[0x3e, 0x00]);
    net.send(client, &request).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(
        drain_client(&mut net, client),
        diagnostic_nack(0x0999, TESTER, 0x03)
    );
    assert!(!client_closed(&mut net, client));

    // Wrong source for the activated connection: 0x02 and close.
    let request = diagnostic_request(0x4444, ENTITY, &[0x3e, 0x00]);
    net.send(client, &request).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    let bytes = drain_client(&mut net, client);
    assert_eq!(bytes[..4], [0x02, 0xfd, 0x80, 0x03]);
    assert_eq!(bytes[12], 0x02);
    pump(&mut entity, &mut net, &mut clock, 3);
    assert!(client_closed(&mut net, client));
    assert_eq!(entity.application().requests, 0);
}

#[test]
fn diagnostic_negative_acknowledgement_keeps_shared_state_clean() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    let request = diagnostic_request(TESTER, 0x0999, &[0x22, 0xf1, 0x90]);
    net.send(client, &request).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(
        drain_client(&mut net, client),
        diagnostic_nack(0x0999, TESTER, 0x03)
    );
    assert_eq!(entity.application().requests, 0);
    assert_eq!(entity.receive_pool_in_use(), 0);
}

// ── alive check arbitration ──────────────────────────────────────────────────

#[test]
fn alive_check_success_rejects_duplicate_source() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let first = activate(&mut entity, &mut net, &mut clock, TESTER);

    // A second connection activates the same source address.
    let second = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);
    net.send(second, &routing_activation(TESTER, 0))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 2);

    // The existing connection receives an alive-check request and answers.
    let alive_request = drain_client(&mut net, first);
    assert_eq!(alive_request, [0x02, 0xfd, 0x00, 0x07, 0, 0, 0, 0]);
    let mut alive_response = vec![0x02, 0xfd, 0x00, 0x08, 0, 0, 0, 2];
    alive_response.extend_from_slice(&TESTER.to_be_bytes());
    net.send(first, &alive_response).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);

    // The duplicate is rejected with SourceAlreadyRegistered and closed.
    let bytes = drain_client(&mut net, second);
    assert_eq!(bytes, activation_response(TESTER, 0x03));
    pump(&mut entity, &mut net, &mut clock, 3);
    assert!(client_closed(&mut net, second));
    assert!(!client_closed(&mut net, first));
    assert_eq!(entity.routing_count(), 1);
}

#[test]
fn alive_check_timeout_displaces_stale_connection() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let stale = activate(&mut entity, &mut net, &mut clock, TESTER);

    let fresh = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);
    net.send(fresh, &routing_activation(TESTER, 0))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 2);
    assert_eq!(
        drain_client(&mut net, stale),
        vec![0x02, 0xfd, 0x00, 0x07, 0, 0, 0, 0]
    );

    // No alive-check response within 500 ms: stale is closed, fresh wins.
    pump(&mut entity, &mut net, &mut clock, 501);
    assert_eq!(
        drain_client(&mut net, fresh),
        activation_response(TESTER, 0x10)
    );
    pump(&mut entity, &mut net, &mut clock, 3);
    assert!(client_closed(&mut net, stale));
    assert!(!client_closed(&mut net, fresh));
    assert_eq!(entity.routing_count(), 1);
}

// ── teardown, recovery, lifecycle ────────────────────────────────────────────

#[test]
fn remote_close_releases_connection() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    net.close(client).expect("client close");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(entity.connection_count(), 0);
    assert!(entity.malformed_or_closed() >= 1);

    // The slot is reusable immediately.
    let _again = activate(&mut entity, &mut net, &mut clock, TESTER);
}

#[test]
fn graceful_local_close_on_stop() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    entity.stop(&mut net);
    assert_eq!(entity.state(), EntityState::Stopped);
    assert_eq!(entity.tcp_port(), 0);
    assert_eq!(entity.discovery_port(), 0);
    assert_eq!(entity.receive_pool_in_use(), 0);
    assert_eq!(entity.send_jobs_in_use(), 0);
    assert!(client_closed(&mut net, client));

    // The peer observes an orderly close.
    let mut saw_close = false;
    while let Some((socket, event)) = net.poll_event() {
        if socket == client && matches!(event, SocketEvent::ConnectionClosed { .. }) {
            saw_close = true;
        }
    }
    assert!(saw_close);
}

#[test]
fn abort_and_reconnect() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    net.abort(client);
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(entity.connection_count(), 0);

    let reconnected = activate(&mut entity, &mut net, &mut clock, TESTER);
    net.send(reconnected, &tester_present(TESTER))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    let mut expected = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(drain_client(&mut net, reconnected), expected);
}

#[test]
fn network_link_down_and_up_recovery() {
    let mut net = Stack::new();
    let mut entity = new_entity();
    entity.set_announcement_target(Some(IpEndpoint::new(
        IpAddress::V4([255, 255, 255, 255]),
        ANNOUNCE_PORT,
    )));
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut clock = TestClock::new();
    let listener = udp_client(&mut net, ANNOUNCE_PORT);
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);
    // Drain the initial announcement burst.
    pump(&mut entity, &mut net, &mut clock, 40);
    while recv_discovery(&mut net, listener).is_some() {}

    // Link loss: connections are torn down, polling stays healthy.
    net.set_link_up(false);
    pump(&mut entity, &mut net, &mut clock, 5);
    assert_eq!(entity.connection_count(), 0);
    assert!(client_closed(&mut net, client));
    assert_eq!(entity.state(), EntityState::Running);
    assert!(recv_discovery(&mut net, listener).is_none());

    // Link recovery: announcements restart and clients reconnect.
    net.set_link_up(true);
    pump(&mut entity, &mut net, &mut clock, 15);
    assert!(recv_discovery(&mut net, listener).is_some());
    let _reconnected = activate(&mut entity, &mut net, &mut clock, TESTER);
}

#[test]
fn lifecycle_stop_start_recovery() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let udp = udp_client(&mut net, 40000);

    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::All),
    );
    pump(&mut entity, &mut net, &mut clock, 1);
    assert!(recv_discovery(&mut net, udp).is_some());
    let _client = activate(&mut entity, &mut net, &mut clock, TESTER);

    entity.stop(&mut net);
    assert_eq!(entity.state(), EntityState::Stopped);

    // Restart on the same backend: full service is restored.
    entity.start(&mut net).expect("restart");
    entity.enable(clock.now()).expect("enable");
    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::All),
    );
    pump(&mut entity, &mut net, &mut clock, 1);
    assert!(recv_discovery(&mut net, udp).is_some());
    let _again = activate(&mut entity, &mut net, &mut clock, TESTER);
    entity.stop(&mut net);
    assert_eq!(entity.receive_pool_in_use(), 0);
    assert_eq!(entity.send_jobs_in_use(), 0);
}

// ── backpressure and exhaustion ──────────────────────────────────────────────

#[test]
fn partial_sends_and_transmit_backpressure() {
    // 16-byte receive buffers: the 17-byte activation response cannot be
    // delivered in one piece while the client is not draining.
    let mut net: FakeStack<8, 16, 32> = FakeStack::new();
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 2, 2, 2, 64> =
        DoIpEntity::new(
            config(),
            ProtocolVersion::Iso2012,
            ENTITY,
            parameters(),
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");

    let client = net.create().expect("client");
    net.connect(client, ENTITY_IP, TCP_PORT).expect("connect");
    let mut now = 0u64;
    let poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        2,
        2,
        2,
        64,
    >,
                net: &mut FakeStack<8, 16, 32>,
                now: &mut u64| {
        *now += 1_000_000;
        entity.poll(Instant::from_nanos(*now), net).expect("poll");
    };
    poll(&mut entity, &mut net, &mut now);
    net.send(client, &routing_activation(TESTER, 0))
        .expect("send");
    // Do not drain the client yet: the response blocks partway.
    for _ in 0..5 {
        poll(&mut entity, &mut net, &mut now);
    }
    assert!(entity.backpressure_events() > 0);

    // Draining the client lets the pending send complete unchanged.
    let mut received = Vec::new();
    let mut chunk = [0u8; 8];
    for _ in 0..10 {
        poll(&mut entity, &mut net, &mut now);
        let read = net.recv(client, &mut chunk).expect("recv");
        received.extend_from_slice(&chunk[..read]);
        if received.len() >= 17 {
            break;
        }
    }
    assert_eq!(received, activation_response(TESTER, 0x10));
}

#[test]
fn receive_pressure_processes_burst_without_loss() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    // Three requests back-to-back exceed one RECV_CHUNK-cycle of work.
    let mut burst = Vec::new();
    for _ in 0..3 {
        burst.extend_from_slice(&tester_present(TESTER));
    }
    net.send(client, &burst).expect("send");
    let mut received = Vec::new();
    for _ in 0..20 {
        pump(&mut entity, &mut net, &mut clock, 1);
        received.extend_from_slice(&drain_client(&mut net, client));
    }
    let mut expected = Vec::new();
    for _ in 0..3 {
        expected.extend_from_slice(&diagnostic_ack(TESTER, &[0x3e, 0x00]));
    }
    for _ in 0..3 {
        expected.extend_from_slice(&uds_response(TESTER));
    }
    assert_eq!(received, expected);
    assert_eq!(entity.application().requests, 3);
}

#[test]
fn malformed_header_nacks_and_closes() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);

    net.send(client, &[0x02, 0x02, 0, 1, 0, 0, 0, 0])
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(drain_client(&mut net, client), generic_nack(0x00));
    pump(&mut entity, &mut net, &mut clock, 3);
    assert!(client_closed(&mut net, client));
    assert!(entity.malformed_or_closed() >= 1);
}

#[test]
fn unsupported_payload_type_nacks_without_close() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);

    net.send(client, &[0x02, 0xfd, 0x77, 0x77, 0, 0, 0, 0])
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(drain_client(&mut net, client), generic_nack(0x01));
    assert!(!client_closed(&mut net, client));

    // The connection is still usable afterwards.
    net.send(client, &routing_activation(TESTER, 0))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(
        drain_client(&mut net, client),
        activation_response(TESTER, 0x10)
    );
}

#[test]
fn oversized_declared_payload_is_rejected() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    // Declared diagnostic payload far above max_payload_size (64).
    let mut frame = vec![0x02, 0xfd, 0x80, 0x01];
    frame.extend_from_slice(&10_000u32.to_be_bytes());
    frame.extend_from_slice(&TESTER.to_be_bytes());
    frame.extend_from_slice(&ENTITY.to_be_bytes());
    net.send(client, &frame).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    // Generic NACK MessageTooLarge (0x02); the declared payload is
    // discarded without reaching the application.
    assert_eq!(drain_client(&mut net, client), generic_nack(0x02));
    assert_eq!(entity.application().requests, 0);
}

#[test]
fn socket_pool_exhaustion_is_observable_and_recoverable() {
    // 4 TCP slots total: listener + client A + server side of A leave a
    // single slot; the second accept cannot allocate a server socket.
    let mut net: FakeStack<4, 256, 32> = FakeStack::new();
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 2, 2, 64> =
        DoIpEntity::new(
            config(),
            ProtocolVersion::Iso2012,
            ENTITY,
            parameters(),
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut now = 0u64;
    let poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        6,
        2,
        2,
        64,
    >,
                net: &mut FakeStack<4, 256, 32>,
                now: &mut u64| {
        *now += 1_000_000;
        entity.poll(Instant::from_nanos(*now), net).expect("poll");
    };

    let first = net.create().expect("first");
    net.connect(first, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    assert_eq!(entity.accepted_connections(), 1);

    let second = net.create().expect("second");
    net.connect(second, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    assert!(entity.resource_rejections() > 0);
    assert_eq!(entity.state(), EntityState::Running);

    // The first client still gets full service.
    net.send(first, &routing_activation(TESTER, 0))
        .expect("send");
    for _ in 0..3 {
        poll(&mut entity, &mut net, &mut now);
    }
    let mut chunk = [0u8; 64];
    let read = net.recv(first, &mut chunk).expect("recv");
    assert_eq!(&chunk[..read], activation_response(TESTER, 0x10));
}

#[test]
fn connection_pool_exhaustion_rejects_excess_clients() {
    let mut net = Stack::new();
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 3, 2, 2, 64> =
        DoIpEntity::new(
            config(),
            ProtocolVersion::Iso2012,
            ENTITY,
            TransportParameters {
                max_payload_size: 64,
                max_connection_count: 2,
                ..TransportParameters::default()
            },
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut now = 0u64;
    let poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        3,
        2,
        2,
        64,
    >,
                net: &mut Stack,
                now: &mut u64| {
        *now += 1_000_000;
        entity.poll(Instant::from_nanos(*now), net).expect("poll");
    };

    // max_connection_count 2 admits three connections (arbitration slack);
    // the fourth is refused and aborted.
    let mut clients = Vec::new();
    for _ in 0..3 {
        let client = net.create().expect("client");
        net.connect(client, ENTITY_IP, TCP_PORT).expect("connect");
        poll(&mut entity, &mut net, &mut now);
        clients.push(client);
    }
    assert_eq!(entity.accepted_connections(), 3);

    let excess = net.create().expect("excess");
    net.connect(excess, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    poll(&mut entity, &mut net, &mut now);
    assert!(entity.resource_rejections() > 0);
    assert!(matches!(
        net.state(excess),
        Ok(SocketState::Closed) | Err(_)
    ));
}

#[test]
fn receive_message_pool_exhaustion_nacks_out_of_memory() {
    let mut net = Stack::new();
    // One receive message: a second concurrent diagnostic transfer starves.
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 1, 2, 64> =
        DoIpEntity::new(
            config(),
            ProtocolVersion::Iso2012,
            ENTITY,
            parameters(),
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut now = 0u64;
    let poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        6,
        1,
        2,
        64,
    >,
                net: &mut Stack,
                now: &mut u64| {
        *now += 1_000_000;
        entity.poll(Instant::from_nanos(*now), net).expect("poll");
    };

    // Client A activates and starts a diagnostic message but withholds the
    // final payload byte, keeping the pool allocation open.
    let a = net.create().expect("a");
    net.connect(a, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    net.send(a, &routing_activation(TESTER, 0)).expect("send");
    for _ in 0..3 {
        poll(&mut entity, &mut net, &mut now);
    }
    let mut chunk0 = [0u8; 64];
    let read = net.recv(a, &mut chunk0).expect("recv");
    assert_eq!(&chunk0[..read], activation_response(TESTER, 0x10));
    let request = tester_present(TESTER);
    net.send(a, &request[..request.len() - 1]).expect("send");
    poll(&mut entity, &mut net, &mut now);
    assert_eq!(entity.receive_pool_in_use(), 1);

    // Client B's complete request is refused with OutOfMemory (0x05).
    let b = net.create().expect("b");
    net.connect(b, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    net.send(b, &routing_activation(0x2222, 0)).expect("send");
    for _ in 0..3 {
        poll(&mut entity, &mut net, &mut now);
    }
    let mut chunk = [0u8; 64];
    let read = net.recv(b, &mut chunk).expect("recv");
    assert_eq!(&chunk[..read], activation_response(0x2222, 0x10));
    net.send(b, &diagnostic_request(0x2222, ENTITY, &[0x3e, 0x00]))
        .expect("send");
    for _ in 0..3 {
        poll(&mut entity, &mut net, &mut now);
    }
    let read = net.recv(b, &mut chunk).expect("recv");
    assert_eq!(chunk[..4], [0x02, 0xfd, 0x80, 0x03]);
    assert_eq!(chunk[12], 0x05);
    assert!(read >= 13);

    // Client A completes; the pool drains and service recovers.
    net.send(a, &request[request.len() - 1..]).expect("send");
    for _ in 0..4 {
        poll(&mut entity, &mut net, &mut now);
    }
    assert_eq!(entity.receive_pool_in_use(), 0);
    let read = net.recv(a, &mut chunk).expect("recv");
    let mut expected = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(&chunk[..read], expected);
}

#[test]
fn send_job_exhaustion_drops_excess_and_recovers() {
    let mut net = Stack::new();
    // A single send job: two responses in one cycle exceed the pool.
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 2, 1, 64> =
        DoIpEntity::new(
            config(),
            ProtocolVersion::Iso2012,
            ENTITY,
            parameters(),
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).expect("start");
    entity.enable(Instant::from_nanos(0)).expect("enable");
    let mut now = 0u64;
    let poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        6,
        2,
        1,
        64,
    >,
                net: &mut Stack,
                now: &mut u64| {
        *now += 1_000_000;
        entity.poll(Instant::from_nanos(*now), net).expect("poll");
    };

    let client = net.create().expect("client");
    net.connect(client, ENTITY_IP, TCP_PORT).expect("connect");
    poll(&mut entity, &mut net, &mut now);
    net.send(client, &routing_activation(TESTER, 0))
        .expect("send");
    for _ in 0..3 {
        poll(&mut entity, &mut net, &mut now);
    }
    let mut chunk = [0u8; 128];
    let read = net.recv(client, &mut chunk).expect("recv");
    assert_eq!(&chunk[..read], activation_response(TESTER, 0x10));

    // Two requests in one burst: both are acknowledged, but only one
    // response job fits the pool; the second response is dropped.
    let mut burst = tester_present(TESTER);
    burst.extend_from_slice(&tester_present(TESTER));
    net.send(client, &burst).expect("send");
    let mut received = Vec::new();
    for _ in 0..10 {
        poll(&mut entity, &mut net, &mut now);
        let read = net.recv(client, &mut chunk).expect("recv");
        received.extend_from_slice(&chunk[..read]);
    }
    let ack = diagnostic_ack(TESTER, &[0x3e, 0x00]);
    let mut expected = ack.clone();
    expected.extend_from_slice(&ack);
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(received, expected);
    assert_eq!(entity.application().requests, 2);
    assert_eq!(entity.send_jobs_in_use(), 0);

    // The pool recovered: a further request is fully served.
    net.send(client, &tester_present(TESTER)).expect("send");
    let mut received = Vec::new();
    for _ in 0..5 {
        poll(&mut entity, &mut net, &mut now);
        let read = net.recv(client, &mut chunk).expect("recv");
        received.extend_from_slice(&chunk[..read]);
    }
    let mut expected = ack;
    expected.extend_from_slice(&uds_response(TESTER));
    assert_eq!(received, expected);
}

#[test]
fn stale_backend_handle_is_survived() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = activate(&mut entity, &mut net, &mut clock, TESTER);

    // Simulate the backend reclaiming the server-side socket out from
    // under the entity (e.g. a platform fault): the entity's stored
    // handle goes stale, is rejected by the backend, and the connection
    // is released without disturbing the rest of the entity.
    net.abort(client);
    // Client-side abort also invalidates the server side via the fake's
    // peer teardown; polling must clean up and keep serving.
    pump(&mut entity, &mut net, &mut clock, 3);
    assert_eq!(entity.connection_count(), 0);
    assert_eq!(entity.state(), EntityState::Running);
    let _again = activate(&mut entity, &mut net, &mut clock, TESTER);
}

// ── timers ───────────────────────────────────────────────────────────────────

#[test]
fn initial_inactivity_timeout_exact_boundary() {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let client = connect_client(&mut net);
    // Accept happens at t = 1 ms; the 2 s window closes at exactly 2001 ms.
    pump(&mut entity, &mut net, &mut clock, 1);
    assert_eq!(entity.connection_count(), 1);

    // One millisecond before the deadline the connection survives.
    clock.advance(1999);
    entity.poll(clock.now(), &mut net).expect("poll");
    assert_eq!(entity.connection_count(), 1);

    // At the exact deadline it is torn down.
    clock.advance(1);
    entity.poll(clock.now(), &mut net).expect("poll");
    entity.poll(clock.now(), &mut net).expect("poll");
    assert_eq!(entity.connection_count(), 0);
    assert!(client_closed(&mut net, client));
}

#[test]
fn timers_survive_instant_wraparound() {
    let mut net = Stack::new();
    let mut entity = new_entity();
    // Start one second before the 64-bit nanosecond counter wraps.
    let base = Instant::from_nanos(u64::MAX - 1_000_000_000);
    entity.start(&mut net).expect("start");
    entity.enable(base).expect("enable");
    let mut clock = TestClock::at(base);

    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 1);
    assert_eq!(entity.connection_count(), 1);

    // The 2 s initial-inactivity deadline lies beyond the wrap point.
    pump(&mut entity, &mut net, &mut clock, 1500);
    assert_eq!(entity.connection_count(), 1);
    pump(&mut entity, &mut net, &mut clock, 600);
    assert_eq!(entity.connection_count(), 0);
    assert!(client_closed(&mut net, client));

    // Activation still works on the wrapped clock.
    let _fresh = activate(&mut entity, &mut net, &mut clock, TESTER);
}

// ── determinism and resource shape ───────────────────────────────────────────

/// One complete scenario: discovery, activation, diagnostics, teardown.
/// Returns every protocol-visible byte in arrival order.
fn scenario_transcript() -> Vec<u8> {
    let mut net = Stack::new();
    let mut entity = started(&mut net);
    let mut clock = TestClock::new();
    let mut transcript = Vec::new();

    let udp = udp_client(&mut net, 40000);
    send_discovery(
        &mut net,
        udp,
        Payload::VehicleIdentification(VehicleIdentification::All),
    );
    pump(&mut entity, &mut net, &mut clock, 2);
    transcript.extend_from_slice(&recv_discovery(&mut net, udp).expect("discovery"));
    send_discovery(&mut net, udp, Payload::EntityStatusRequest);
    pump(&mut entity, &mut net, &mut clock, 2);
    transcript.extend_from_slice(&recv_discovery(&mut net, udp).expect("status"));

    let client = connect_client(&mut net);
    pump(&mut entity, &mut net, &mut clock, 2);
    net.send(client, &routing_activation(TESTER, 0))
        .expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    transcript.extend_from_slice(&drain_client(&mut net, client));
    net.send(client, &tester_present(TESTER)).expect("send");
    pump(&mut entity, &mut net, &mut clock, 3);
    transcript.extend_from_slice(&drain_client(&mut net, client));

    entity.stop(&mut net);
    transcript.push(u8::from(client_closed(&mut net, client)));
    transcript
}

#[test]
fn two_isolated_runs_produce_identical_transcripts() {
    let first = scenario_transcript();
    let second = scenario_transcript();
    assert!(!first.is_empty());
    assert_eq!(first, second);
}

#[test]
fn representative_composition_sizes_are_bounded() {
    // Documented in the embedded-DoIP tranche evidence: representative
    // static sizes of the portable composition (host build, informative).
    let entity = core::mem::size_of::<Entity>();
    let stack = core::mem::size_of::<Stack>();
    println!("size DoIpEntity<.., 6, 6, 6, 64> = {entity} bytes");
    println!("size FakeStack<16, 512, 32> = {stack} bytes");
    // Hard bounds so accidental structural growth is caught.
    assert!(entity <= 16 * 1024, "entity grew unexpectedly: {entity}");
    assert!(stack <= 32 * 1024, "fake stack grew unexpectedly: {stack}");
}
