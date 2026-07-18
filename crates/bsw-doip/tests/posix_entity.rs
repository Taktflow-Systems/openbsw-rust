#![cfg(feature = "std")]

use std::io::{ErrorKind, Read, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpStream, UdpSocket};

use bsw_doip::{
    discovery::AnnouncementSchedule, DiagnosticPowerMode, DoIpApplication, EntityConfig,
    EntityState, EntityStatus, Packet, Payload, PosixDoIpEntity, ProtocolVersion,
    TransportParameters, VehicleAnnouncement, VehicleIdentification,
};
use bsw_ethernet::ip::IpAddress;
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::{Duration, Instant};
use bsw_transport::{TransportMessage, TransportResult};
use bsw_uds::{DiagJob, DiagRouter, DiagSession, SessionMask, TesterPresent};

const ENTITY: u16 = 0x0123;

#[derive(Default)]
struct UdsApplication {
    response: Option<TransportMessage<64>>,
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
        self.response = Some(response);
        TransportResult::Ok
    }
}

impl DoIpApplication<64> for UdsApplication {
    fn take_response(&mut self) -> Option<TransportMessage<64>> {
        self.response.take()
    }
}

type TestEntity = PosixDoIpEntity<UdsApplication, bsw_doip::DefaultActivationPolicy, 6, 2, 2, 64>;

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

fn new_entity() -> TestEntity {
    TestEntity::new(
        EntityConfig {
            tcp_address: IpAddress::ipv4(127, 0, 0, 1),
            tcp_port: 0,
            udp_address: IpAddress::ipv4(127, 0, 0, 1),
            udp_port: 0,
            source_bus: 7,
        },
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters {
            max_payload_size: 64,
            ..TransportParameters::default()
        },
        discovery_entity(),
        UdsApplication::default(),
        bsw_doip::DefaultActivationPolicy,
    )
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

fn pump_tcp(entity: &mut TestEntity, stream: &mut TcpStream, expected: usize) -> Vec<u8> {
    stream.set_nonblocking(true).unwrap();
    let mut bytes = Vec::new();
    let mut scratch = [0; 128];
    for step in 0..1_000u64 {
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

fn discover(entity: &mut TestEntity) -> Vec<u8> {
    let client = UdpSocket::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    client.set_nonblocking(true).unwrap();
    let mut request = [0; 16];
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    }
    .encode(&mut request)
    .unwrap();
    client
        .send_to(
            &request[..length],
            SocketAddrV4::new(Ipv4Addr::LOCALHOST, entity.discovery_port()),
        )
        .unwrap();
    let mut response = [0; 64];
    for step in 0..1_000u64 {
        entity.poll(Instant::from_nanos(step * 1_000_000)).unwrap();
        match client.recv_from(&mut response) {
            Ok((length, _)) => return response[..length].to_vec(),
            Err(error) if error.kind() == ErrorKind::WouldBlock => {}
            Err(error) => panic!("UDP read failed: {error}"),
        }
        std::thread::yield_now();
    }
    panic!("discovery response not received")
}

fn activate(entity: &mut TestEntity) -> TcpStream {
    let mut stream = TcpStream::connect((Ipv4Addr::LOCALHOST, entity.tcp_port())).unwrap();
    stream.write_all(&routing_activation()).unwrap();
    let response = pump_tcp(entity, &mut stream, 17);
    assert_eq!(&response[..4], &[0x02, 0xfd, 0x00, 0x06]);
    assert_eq!(response[12], 0x10);
    stream
}

#[test]
fn lifecycle_entity_runs_discovery_activation_diagnostics_malformed_and_restart() {
    let mut entity = new_entity();
    assert_eq!(entity.name(), "DoIpServerSystem");
    assert_eq!(entity.init(), TransitionResult::Done);
    assert_eq!(entity.state(), EntityState::Initialized);
    assert_ne!(entity.tcp_port(), 0);
    assert_ne!(entity.discovery_port(), 0);
    assert_eq!(entity.run(), TransitionResult::Done);

    let discovery = discover(&mut entity);
    assert!(matches!(
        Packet::parse(&discovery).unwrap().payload,
        Payload::VehicleAnnouncement(_)
    ));

    let mut diagnostic_client = activate(&mut entity);
    diagnostic_client.write_all(&tester_present()).unwrap();
    let response = pump_tcp(&mut entity, &mut diagnostic_client, 29);
    assert_eq!(
        &response[..15],
        &[0x02, 0xfd, 0x80, 0x02, 0, 0, 0, 7, 0x01, 0x23, 0x12, 0x34, 0, 0x3e, 0]
    );
    assert_eq!(
        &response[15..29],
        &[0x02, 0xfd, 0x80, 0x01, 0, 0, 0, 6, 0x01, 0x23, 0x12, 0x34, 0x7e, 0]
    );
    assert_eq!(entity.application().requests, 1);

    let mut malformed = TcpStream::connect((Ipv4Addr::LOCALHOST, entity.tcp_port())).unwrap();
    malformed
        .write_all(&[0x02, 0x02, 0, 1, 0, 0, 0, 0])
        .unwrap();
    let nack = pump_tcp(&mut entity, &mut malformed, 9);
    assert_eq!(nack, [0x02, 0xfd, 0, 0, 0, 0, 0, 1, 0]);
    assert!(entity.malformed_or_closed() >= 1);
    assert!(entity.accepted_connections() >= 2);

    assert_eq!(entity.shutdown(), TransitionResult::Done);
    assert_eq!(entity.state(), EntityState::Stopped);
    assert_eq!(entity.tcp_port(), 0);
    assert_eq!(entity.discovery_port(), 0);

    assert_eq!(entity.init(), TransitionResult::Done);
    assert_eq!(entity.run(), TransitionResult::Done);
    assert!(matches!(
        Packet::parse(&discover(&mut entity)).unwrap().payload,
        Payload::VehicleAnnouncement(_)
    ));
    let _restarted_client = activate(&mut entity);
    assert_eq!(entity.shutdown(), TransitionResult::Done);
}
