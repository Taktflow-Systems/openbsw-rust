#[cfg(feature = "std")]
use std::{net::UdpSocket, time::Duration as StdDuration};

#[cfg(feature = "std")]
use bsw_doip::discovery::posix::PosixDiscoveryService;
use bsw_doip::{
    discovery::{AnnouncementSchedule, DiscoveryEntity},
    DiagnosticAck, DiagnosticPayload, DiagnosticPowerMode, EntityStatus, NackCode, Packet, Payload,
    ProtocolVersion, RoutingActivationCode, RoutingActivationRequest, RoutingActivationResponse,
    VehicleAnnouncement, VehicleIdentification,
};
#[cfg(feature = "std")]
use bsw_ethernet::ip::IpAddress;
use bsw_time::{Duration, Instant};

fn announcement() -> VehicleAnnouncement {
    VehicleAnnouncement {
        vin: *b"TESTVIN0000000001",
        logical_address: 0x0e80,
        eid: [2, 0, 0, 0, 0, 1],
        gid: [1, 2, 3, 4, 5, 6],
        further_action_required: 0,
        sync_status: Some(0),
    }
}

fn roundtrip(payload: Payload<'_>) {
    let packet = Packet {
        version: ProtocolVersion::Iso2019,
        payload,
    };
    let mut bytes = [0; 128];
    let length = packet.encode(&mut bytes).unwrap();
    assert_eq!(Packet::parse(&bytes[..length]).unwrap(), packet);
}

#[test]
fn every_standard_payload_type_roundtrips() {
    roundtrip(Payload::GenericNack(NackCode::InvalidPayloadLength));
    roundtrip(Payload::VehicleIdentification(VehicleIdentification::All));
    roundtrip(Payload::VehicleIdentification(VehicleIdentification::Eid(
        [1, 2, 3, 4, 5, 6],
    )));
    roundtrip(Payload::VehicleIdentification(VehicleIdentification::Vin(
        *b"TESTVIN0000000001",
    )));
    roundtrip(Payload::VehicleAnnouncement(announcement()));
    roundtrip(Payload::RoutingActivationRequest(
        RoutingActivationRequest {
            source_address: 0x0e00,
            activation_type: 0,
            reserved: Some([0; 4]),
            oem_specific: Some([1, 2, 3, 4]),
        },
    ));
    roundtrip(Payload::RoutingActivationResponse(
        RoutingActivationResponse {
            tester_address: 0x0e00,
            entity_address: 0x0e80,
            response_code: RoutingActivationCode::Success,
            reserved: [0; 4],
            oem_specific: None,
        },
    ));
    roundtrip(Payload::AliveCheckRequest);
    roundtrip(Payload::AliveCheckResponse(0x0e00));
    roundtrip(Payload::EntityStatusRequest);
    roundtrip(Payload::EntityStatusResponse(EntityStatus {
        node_type: 1,
        max_sockets: 4,
        open_sockets: 1,
        max_data_size: Some(4095),
    }));
    roundtrip(Payload::PowerModeRequest);
    roundtrip(Payload::PowerModeResponse(DiagnosticPowerMode::Ready));
    roundtrip(Payload::DiagnosticMessage(DiagnosticPayload {
        source_address: 0x0e00,
        target_address: 0x0e80,
        data: &[0x22, 0xf1, 0x90],
    }));
    roundtrip(Payload::DiagnosticPositiveAck(DiagnosticAck {
        source_address: 0x0e80,
        target_address: 0x0e00,
        code: 0,
        previous_data: &[0x22],
    }));
    roundtrip(Payload::DiagnosticNegativeAck(DiagnosticAck {
        source_address: 0x0e80,
        target_address: 0x0e00,
        code: 6,
        previous_data: &[],
    }));
}

#[test]
fn upstream_golden_vehicle_request_and_status_bytes() {
    let mut bytes = [0; 64];
    let request = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    };
    assert_eq!(request.encode(&mut bytes), Ok(8));
    assert_eq!(&bytes[..8], &[0x02, 0xfd, 0, 1, 0, 0, 0, 0]);

    let status = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::EntityStatusResponse(EntityStatus {
            node_type: 1,
            max_sockets: 4,
            open_sockets: 2,
            max_data_size: Some(4095),
        }),
    };
    assert_eq!(status.encode(&mut bytes), Ok(15));
    assert_eq!(
        &bytes[..15],
        &[0x02, 0xfd, 0x40, 0x02, 0, 0, 0, 7, 1, 4, 2, 0, 0, 0x0f, 0xff]
    );
}

#[test]
fn malformed_headers_lengths_and_payload_values_are_rejected() {
    assert!(Packet::parse(&[]).is_err());
    assert!(Packet::parse(&[2, 2, 0, 1, 0, 0, 0, 0]).is_err());
    assert!(Packet::parse(&[2, 0xfd, 0, 1, 0, 0, 0, 1]).is_err());
    assert!(Packet::parse(&[2, 0xfd, 0x40, 2, 0, 0, 0, 3, 1, 1, 2]).is_err());
    let packet = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleAnnouncement(announcement()),
    };
    assert!(packet.encode(&mut [0; 8]).is_err());
}

fn entity() -> DiscoveryEntity {
    DiscoveryEntity::new(
        announcement(),
        EntityStatus {
            node_type: 1,
            max_sockets: 4,
            open_sockets: 0,
            max_data_size: Some(4095),
        },
        DiagnosticPowerMode::Ready,
        AnnouncementSchedule {
            initial_delay: Duration::from_millis(10).unwrap(),
            interval: Duration::from_millis(20).unwrap(),
            count: 3,
        },
    )
}

#[test]
fn identification_filters_status_power_and_announcement_timing() {
    let mut entity = entity();
    let mut request = [0; 64];
    let mut response = [0; 64];
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleIdentification(VehicleIdentification::Eid([9; 6])),
    }
    .encode(&mut request)
    .unwrap();
    assert_eq!(
        entity.handle_datagram(&request[..length], &mut response),
        Ok(0)
    );
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::EntityStatusRequest,
    }
    .encode(&mut request)
    .unwrap();
    let response_length = entity
        .handle_datagram(&request[..length], &mut response)
        .unwrap();
    assert!(matches!(
        Packet::parse(&response[..response_length]).unwrap().payload,
        Payload::EntityStatusResponse(_)
    ));
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::PowerModeRequest,
    }
    .encode(&mut request)
    .unwrap();
    let response_length = entity
        .handle_datagram(&request[..length], &mut response)
        .unwrap();
    assert_eq!(
        Packet::parse(&response[..response_length]).unwrap().payload,
        Payload::PowerModeResponse(DiagnosticPowerMode::Ready)
    );
    entity.start_announcements(Instant::from_nanos(0));
    assert_eq!(
        entity.poll_announcement(
            Instant::from_nanos(Duration::from_millis(10).unwrap().as_nanos() - 1),
            &mut response
        ),
        Ok(0)
    );
    assert!(
        entity
            .poll_announcement(
                Instant::from_nanos(Duration::from_millis(10).unwrap().as_nanos()),
                &mut response
            )
            .unwrap()
            > 0
    );
    assert_eq!(entity.announcements_remaining(), 2);
}

#[cfg(feature = "std")]
#[test]
fn posix_loopback_serves_multiple_discovery_clients() {
    let loopback = IpAddress::ipv4(127, 0, 0, 1);
    let mut server = PosixDiscoveryService::bind(loopback, 0, entity()).unwrap();
    let target = ("127.0.0.1", server.local_port());
    let clients = [
        UdpSocket::bind(("127.0.0.1", 0)).unwrap(),
        UdpSocket::bind(("127.0.0.1", 0)).unwrap(),
    ];
    let mut request = [0; 16];
    let length = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    }
    .encode(&mut request)
    .unwrap();
    for client in &clients {
        client
            .set_read_timeout(Some(StdDuration::from_secs(1)))
            .unwrap();
        client.send_to(&request[..length], target).unwrap();
        let mut input = [0; 64];
        let mut output = [0; 64];
        for _ in 0..10_000 {
            if server.poll_client(&mut input, &mut output).unwrap() > 0 {
                break;
            }
            std::thread::yield_now();
        }
        let mut received = [0; 64];
        let received_length = client.recv(&mut received).unwrap();
        assert!(matches!(
            Packet::parse(&received[..received_length]).unwrap().payload,
            Payload::VehicleAnnouncement(_)
        ));
    }
}
