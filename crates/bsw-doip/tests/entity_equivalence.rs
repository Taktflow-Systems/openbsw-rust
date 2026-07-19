//! Equivalence evidence: the POSIX adapter and the deterministic embedded
//! backend produce identical protocol-visible bytes for the same scenario,
//! because both drive the same portable `DoIpEntity` core.
//!
//! Scenario: vehicle identification over UDP, routing activation, shared
//! UDS diagnostic exchange, and a malformed header NACK.

#![cfg(feature = "std")]

use std::io::{ErrorKind, Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpStream, UdpSocket};

use bsw_doip::{
    discovery::AnnouncementSchedule, DiagnosticPowerMode, DoIpApplication, DoIpEntity,
    EntityConfig, EntityStatus, Packet, Payload, PosixDoIpEntity, ProtocolVersion,
    TransportParameters, VehicleAnnouncement, VehicleIdentification,
};
use bsw_ethernet::endpoint::IpEndpoint;
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::{fake::FakeStack, DatagramApi, SocketApi};
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::{Duration, Instant};
use bsw_transport::{TransportMessage, TransportResult};
use bsw_uds::{DiagJob, DiagRouter, DiagSession, SessionMask, TesterPresent};

const ENTITY: u16 = 0x0123;

#[derive(Default)]
struct UdsApplication {
    response: Option<TransportMessage<64>>,
}

impl bsw_doip::DiagnosticMessageListener<64> for UdsApplication {
    fn message_received(&mut self, message: &TransportMessage<64>) -> TransportResult {
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
        self.response = Some(response);
        TransportResult::Ok
    }
}

impl DoIpApplication<64> for UdsApplication {
    fn take_response(&mut self) -> Option<TransportMessage<64>> {
        self.response.take()
    }
}

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

fn parameters() -> TransportParameters {
    TransportParameters {
        max_payload_size: 64,
        ..TransportParameters::default()
    }
}

fn vehicle_identification_request() -> Vec<u8> {
    let mut request = [0u8; 16];
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    }
    .encode(&mut request)
    .unwrap();
    request[..length].to_vec()
}

fn routing_activation() -> [u8; 15] {
    [
        0x02, 0xfd, 0x00, 0x05, 0, 0, 0, 7, 0x12, 0x34, 0, 0, 0, 0, 0,
    ]
}

fn tester_present() -> [u8; 14] {
    [
        0x02, 0xfd, 0x80, 0x01, 0, 0, 0, 6, 0x12, 0x34, 0x01, 0x23, 0x3e, 0,
    ]
}

const MALFORMED: [u8; 8] = [0x02, 0x02, 0, 1, 0, 0, 0, 0];

/// Protocol-visible observation of one scenario run.
#[derive(Debug, PartialEq, Eq)]
struct Transcript {
    discovery: Vec<u8>,
    activation: Vec<u8>,
    diagnostics: Vec<u8>,
    nack: Vec<u8>,
}

// ── embedded backend run ─────────────────────────────────────────────────────

fn embedded_transcript() -> Transcript {
    const IP: IpAddress = IpAddress::V4([10, 0, 0, 1]);
    let mut net: FakeStack<16, 512, 32> = FakeStack::new();
    let mut entity: DoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 2, 2, 64> =
        DoIpEntity::new(
            EntityConfig {
                tcp_address: IP,
                tcp_port: 13400,
                udp_address: IP,
                udp_port: 13400,
                source_bus: 7,
            },
            ProtocolVersion::Iso2012,
            ENTITY,
            parameters(),
            discovery_entity(),
            UdsApplication::default(),
            bsw_doip::DefaultActivationPolicy,
        );
    entity.start(&mut net).unwrap();
    entity.enable(Instant::from_nanos(0)).unwrap();
    let mut now = 0u64;
    let mut poll = |entity: &mut DoIpEntity<
        UdsApplication,
        bsw_doip::DefaultActivationPolicy,
        6,
        2,
        2,
        64,
    >,
                    net: &mut FakeStack<16, 512, 32>| {
        now += 1_000_000;
        entity.poll(Instant::from_nanos(now), net).unwrap();
    };

    // Discovery.
    let udp = net.create_datagram().unwrap();
    net.bind_datagram(udp, IpAddress::V4([10, 0, 0, 2]), 40000)
        .unwrap();
    net.send_datagram(
        udp,
        IpEndpoint::new(IP, 13400),
        &vehicle_identification_request(),
    )
    .unwrap();
    poll(&mut entity, &mut net);
    let mut buf = [0u8; 128];
    let (length, _) = net.recv_datagram(udp, &mut buf).unwrap().unwrap();
    let discovery = buf[..length].to_vec();

    // Activation.
    let client = net.create().unwrap();
    net.connect(client, IP, 13400).unwrap();
    poll(&mut entity, &mut net);
    net.send(client, &routing_activation()).unwrap();
    for _ in 0..3 {
        poll(&mut entity, &mut net);
    }
    let read = net.recv(client, &mut buf).unwrap();
    let activation = buf[..read].to_vec();

    // Shared UDS diagnostics.
    net.send(client, &tester_present()).unwrap();
    let mut diagnostics = Vec::new();
    for _ in 0..5 {
        poll(&mut entity, &mut net);
        let read = net.recv(client, &mut buf).unwrap();
        diagnostics.extend_from_slice(&buf[..read]);
    }

    // Malformed header on a second connection.
    let malformed = net.create().unwrap();
    net.connect(malformed, IP, 13400).unwrap();
    poll(&mut entity, &mut net);
    net.send(malformed, &MALFORMED).unwrap();
    let mut nack = Vec::new();
    for _ in 0..5 {
        poll(&mut entity, &mut net);
        if let Ok(read) = net.recv(malformed, &mut buf) {
            nack.extend_from_slice(&buf[..read]);
        }
    }
    entity.stop(&mut net);
    Transcript {
        discovery,
        activation,
        diagnostics,
        nack,
    }
}

// ── POSIX adapter run ────────────────────────────────────────────────────────

type PosixEntity = PosixDoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 2, 2, 64>;

fn pump_tcp(entity: &mut PosixEntity, stream: &mut TcpStream, expected: usize) -> Vec<u8> {
    stream.set_nonblocking(true).unwrap();
    let mut bytes = Vec::new();
    let mut scratch = [0; 128];
    for step in 0..2_000u64 {
        entity.poll(Instant::from_nanos(step * 1_000_000)).unwrap();
        match stream.read(&mut scratch) {
            Ok(0) => {}
            Ok(length) => bytes.extend_from_slice(&scratch[..length]),
            Err(error) if error.kind() == ErrorKind::WouldBlock => {}
            Err(error) => panic!("TCP read failed: {error}"),
        }
        if bytes.len() >= expected {
            break;
        }
        std::thread::yield_now();
    }
    bytes
}

fn posix_transcript() -> Transcript {
    let mut entity = PosixEntity::new(
        EntityConfig {
            tcp_address: IpAddress::ipv4(127, 0, 0, 1),
            tcp_port: 0,
            udp_address: IpAddress::ipv4(127, 0, 0, 1),
            udp_port: 0,
            source_bus: 7,
        },
        ProtocolVersion::Iso2012,
        ENTITY,
        parameters(),
        discovery_entity(),
        UdsApplication::default(),
        bsw_doip::DefaultActivationPolicy,
    );
    assert_eq!(entity.init(), TransitionResult::Done);
    assert_eq!(entity.run(), TransitionResult::Done);

    // Discovery.
    let udp = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    udp.set_nonblocking(true).unwrap();
    udp.send_to(
        &vehicle_identification_request(),
        SocketAddrV4::new(Ipv4Addr::LOCALHOST, entity.discovery_port()),
    )
    .unwrap();
    let mut buf = [0u8; 128];
    let mut discovery = Vec::new();
    for step in 0..2_000u64 {
        entity.poll(Instant::from_nanos(step * 1_000_000)).unwrap();
        match udp.recv_from(&mut buf) {
            Ok((length, _)) => {
                discovery.extend_from_slice(&buf[..length]);
                break;
            }
            Err(error) if error.kind() == ErrorKind::WouldBlock => {}
            Err(error) => panic!("UDP read failed: {error}"),
        }
        std::thread::yield_now();
    }

    // Activation.
    let mut stream = TcpStream::connect((Ipv4Addr::LOCALHOST, entity.tcp_port())).unwrap();
    stream.write_all(&routing_activation()).unwrap();
    let activation = pump_tcp(&mut entity, &mut stream, 17);

    // Shared UDS diagnostics.
    stream.write_all(&tester_present()).unwrap();
    let diagnostics = pump_tcp(&mut entity, &mut stream, 29);

    // Malformed header on a second connection.
    let mut malformed = TcpStream::connect((Ipv4Addr::LOCALHOST, entity.tcp_port())).unwrap();
    malformed.write_all(&MALFORMED).unwrap();
    let nack = pump_tcp(&mut entity, &mut malformed, 9);
    assert_eq!(entity.shutdown(), TransitionResult::Done);
    Transcript {
        discovery,
        activation,
        diagnostics,
        nack,
    }
}

#[test]
fn posix_and_embedded_backends_are_protocol_equivalent() {
    let embedded = embedded_transcript();
    let posix = posix_transcript();
    assert_eq!(embedded, posix);
}
