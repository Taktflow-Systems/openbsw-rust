#![no_main]

//! Bounded fuzzing of the portable live `DoIP` entity over the
//! deterministic fake stack boundary: arbitrary interleavings of TCP
//! bytes, UDP datagrams, time advancement, link loss, client teardown,
//! and lifecycle restarts must never panic, leak pool entries past stop,
//! or wedge the entity.

use bsw_doip::{
    discovery::AnnouncementSchedule, DiagnosticPowerMode, DoIpApplication, DoIpEntity,
    EntityConfig, EntityStatus, ProtocolVersion, TransportParameters, VehicleAnnouncement,
};
use bsw_ethernet::endpoint::IpEndpoint;
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::{fake::FakeStack, DatagramApi, SocketApi, SocketId};
use bsw_time::{Duration, Instant};
use bsw_transport::{TransportMessage, TransportResult};
use libfuzzer_sys::fuzz_target;

const ENTITY: u16 = 0x0123;
const IP: IpAddress = IpAddress::V4([10, 0, 0, 1]);

#[derive(Default)]
struct EchoApplication {
    response: Option<TransportMessage<64>>,
}

impl bsw_doip::DiagnosticMessageListener<64> for EchoApplication {
    fn message_received(&mut self, message: &TransportMessage<64>) -> TransportResult {
        let mut response = TransportMessage::new();
        response.set_source_address(message.target_address());
        response.set_target_address(message.source_address());
        let payload = message.payload();
        let take = payload.len().min(8);
        if response.append(&payload[..take]).is_err() {
            return TransportResult::BufferFull;
        }
        self.response = Some(response);
        TransportResult::Ok
    }
}

impl DoIpApplication<64> for EchoApplication {
    fn take_response(&mut self) -> Option<TransportMessage<64>> {
        self.response.take()
    }
}

type Entity = DoIpEntity<EchoApplication, bsw_doip::DefaultActivationPolicy, 3, 2, 2, 64>;
type Stack = FakeStack<8, 128, 16>;

fn new_entity() -> Entity {
    Entity::new(
        EntityConfig {
            tcp_address: IP,
            tcp_port: 13400,
            udp_address: IP,
            udp_port: 13400,
            source_bus: 7,
        },
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters {
            max_payload_size: 64,
            inactivity_timeout_ms: 50,
            general_inactivity_timeout_ms: 200,
            alive_check_timeout_ms: 20,
            ..TransportParameters::default()
        },
        bsw_doip::DiscoveryEntity::new(
            VehicleAnnouncement {
                vin: *b"FUZZVIN0000000001",
                logical_address: ENTITY,
                eid: [1, 2, 3, 4, 5, 6],
                gid: [6, 5, 4, 3, 2, 1],
                further_action_required: 0,
                sync_status: Some(0),
            },
            EntityStatus {
                node_type: 1,
                max_sockets: 3,
                open_sockets: 0,
                max_data_size: Some(64),
            },
            DiagnosticPowerMode::Ready,
            AnnouncementSchedule {
                initial_delay: Duration::from_millis(5).unwrap(),
                interval: Duration::from_millis(5).unwrap(),
                count: 2,
            },
        ),
        EchoApplication::default(),
        bsw_doip::DefaultActivationPolicy,
    )
}

fuzz_target!(|data: &[u8]| {
    let mut net = Stack::new();
    let mut entity = new_entity();
    entity.set_announcement_target(Some(IpEndpoint::new(
        IpAddress::V4([255, 255, 255, 255]),
        13401,
    )));
    if entity.start(&mut net).is_err() {
        return;
    }
    let _ = entity.enable(Instant::from_nanos(0));

    let mut clients: [Option<SocketId>; 3] = [None; 3];
    let udp = net.create_datagram().ok();
    let mut now_ms = 0u64;
    let mut index = 0usize;

    while index < data.len() {
        let op = data[index];
        index += 1;
        match op % 8 {
            // Advance time and poll.
            0 => {
                now_ms = now_ms.wrapping_add(u64::from(data.get(index).copied().unwrap_or(1)));
                index += 1;
            }
            // Connect one client into a free local slot.
            1 => {
                if let Some(slot) = clients.iter_mut().find(|slot| slot.is_none()) {
                    if let Ok(socket) = net.create() {
                        if net.connect(socket, IP, 13400).is_ok() {
                            *slot = Some(socket);
                        } else {
                            net.abort(socket);
                        }
                    }
                }
            }
            // Send a data chunk from a client.
            2 => {
                let which = usize::from(data.get(index).copied().unwrap_or(0)) % clients.len();
                index += 1;
                let length = usize::from(data.get(index).copied().unwrap_or(0)) % 32;
                index += 1;
                let start = index.min(data.len());
                let end = (start + length).min(data.len());
                if let Some(socket) = clients[which] {
                    // Backpressure or teardown results are both valid.
                    let _ = net.send(socket, &data[start..end]);
                }
                index = end;
            }
            // Drain a client's receive buffer.
            3 => {
                let which = usize::from(data.get(index).copied().unwrap_or(0)) % clients.len();
                index += 1;
                if let Some(socket) = clients[which] {
                    let mut buf = [0u8; 64];
                    let _ = net.recv(socket, &mut buf);
                }
            }
            // Close or abort a client.
            4 => {
                let which = usize::from(data.get(index).copied().unwrap_or(0)) % clients.len();
                index += 1;
                if let Some(socket) = clients[which].take() {
                    if which % 2 == 0 {
                        let _ = net.close(socket);
                    } else {
                        net.abort(socket);
                    }
                }
            }
            // Inject a UDP datagram.
            5 => {
                let length = usize::from(data.get(index).copied().unwrap_or(0)) % 48;
                index += 1;
                let start = index.min(data.len());
                let end = (start + length).min(data.len());
                if let Some(udp) = udp {
                    let _ = net.send_datagram(udp, IpEndpoint::new(IP, 13400), &data[start..end]);
                }
                index = end;
            }
            // Toggle the link.
            6 => {
                net.set_link_up(data.get(index).copied().unwrap_or(1) % 2 == 0);
                index += 1;
            }
            // Lifecycle stop / restart.
            _ => {
                entity.stop(&mut net);
                assert_eq!(entity.receive_pool_in_use(), 0);
                assert_eq!(entity.send_jobs_in_use(), 0);
                for slot in &mut clients {
                    if let Some(socket) = slot.take() {
                        net.abort(socket);
                    }
                }
                if entity.start(&mut net).is_err() {
                    return;
                }
                let _ = entity.enable(Instant::from_nanos(now_ms.wrapping_mul(1_000_000)));
            }
        }
        let _ = entity.poll(Instant::from_nanos(now_ms.wrapping_mul(1_000_000)), &mut net);
        now_ms = now_ms.wrapping_add(1);
    }

    entity.stop(&mut net);
    assert_eq!(entity.receive_pool_in_use(), 0);
    assert_eq!(entity.send_jobs_in_use(), 0);
});
