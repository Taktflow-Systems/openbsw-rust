//! Contract tests for the datagram ([`DatagramApi`]) and link
//! ([`LinkState`]) extensions of the portable stack boundary, proven with
//! the deterministic fake backend and, where meaningful, the POSIX
//! adapter.

use bsw_ethernet::endpoint::IpEndpoint;
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::fake::{FakeStack, UDP_DATAGRAM_MAX, UDP_QUEUE, UDP_SOCKETS};
use bsw_ethernet::lwip::{DatagramApi, LinkState, SocketApi, SocketError, SocketState};

type Stack = FakeStack<4, 64, 16>;

const A: IpAddress = IpAddress::V4([10, 0, 0, 1]);
const B: IpAddress = IpAddress::V4([10, 0, 0, 2]);
const BROADCAST: IpAddress = IpAddress::V4([255, 255, 255, 255]);

#[test]
fn datagram_unicast_roundtrip_reports_endpoints() {
    let mut stack = Stack::new();
    let server = stack.create_datagram().unwrap();
    stack.bind_datagram(server, A, 13400).unwrap();
    let client = stack.create_datagram().unwrap();
    stack.bind_datagram(client, B, 40000).unwrap();

    stack
        .send_datagram(client, IpEndpoint::new(A, 13400), b"ping")
        .unwrap();
    let mut buf = [0u8; 16];
    let (length, from) = stack.recv_datagram(server, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..length], b"ping");
    assert_eq!(from, IpEndpoint::new(B, 40000));

    // Reply to the reported endpoint.
    stack.send_datagram(server, from, b"pong").unwrap();
    let (length, from) = stack.recv_datagram(client, &mut buf).unwrap().unwrap();
    assert_eq!(&buf[..length], b"pong");
    assert_eq!(from, IpEndpoint::new(A, 13400));
    assert!(stack.recv_datagram(client, &mut buf).unwrap().is_none());
}

#[test]
fn datagram_broadcast_reaches_all_bound_ports() {
    let mut stack = Stack::new();
    let one = stack.create_datagram().unwrap();
    stack.bind_datagram(one, A, 13401).unwrap();
    let two = stack.create_datagram().unwrap();
    stack.bind_datagram(two, B, 13401).unwrap();
    let other_port = stack.create_datagram().unwrap();
    stack.bind_datagram(other_port, B, 9999).unwrap();
    let sender = stack.create_datagram().unwrap();

    stack
        .send_datagram(sender, IpEndpoint::new(BROADCAST, 13401), b"hello")
        .unwrap();
    let mut buf = [0u8; 16];
    assert!(stack.recv_datagram(one, &mut buf).unwrap().is_some());
    assert!(stack.recv_datagram(two, &mut buf).unwrap().is_some());
    assert!(stack.recv_datagram(other_port, &mut buf).unwrap().is_none());
}

#[test]
fn datagram_implicit_bind_on_send() {
    let mut stack = Stack::new();
    let server = stack.create_datagram().unwrap();
    stack.bind_datagram(server, A, 13400).unwrap();
    let client = stack.create_datagram().unwrap();
    assert_eq!(
        stack.datagram_local_endpoint(client).unwrap(),
        IpEndpoint::unset()
    );
    stack
        .send_datagram(client, IpEndpoint::new(A, 13400), b"x")
        .unwrap();
    let local = stack.datagram_local_endpoint(client).unwrap();
    assert!(local.port().is_some());
}

#[test]
fn datagram_pool_and_queue_exhaustion_are_observable() {
    let mut stack = Stack::new();
    let mut sockets = Vec::new();
    for _ in 0..UDP_SOCKETS {
        sockets.push(stack.create_datagram().unwrap());
    }
    assert_eq!(stack.create_datagram(), Err(SocketError::NoResources));

    // Receive-queue overflow drops observably (UDP loss semantics).
    let server = sockets[0];
    stack.bind_datagram(server, A, 13400).unwrap();
    let client = sockets[1];
    for _ in 0..UDP_QUEUE + 2 {
        stack
            .send_datagram(client, IpEndpoint::new(A, 13400), b"burst")
            .unwrap();
    }
    assert_eq!(stack.dropped_datagrams(), 2);
    let mut buf = [0u8; 16];
    for _ in 0..UDP_QUEUE {
        assert!(stack.recv_datagram(server, &mut buf).unwrap().is_some());
    }
    assert!(stack.recv_datagram(server, &mut buf).unwrap().is_none());
}

#[test]
fn datagram_oversize_and_truncation() {
    let mut stack = Stack::new();
    let server = stack.create_datagram().unwrap();
    stack.bind_datagram(server, A, 13400).unwrap();
    let client = stack.create_datagram().unwrap();

    // Larger than the fake's fixed datagram capacity: refused.
    let oversized = [0u8; UDP_DATAGRAM_MAX + 1];
    assert_eq!(
        stack.send_datagram(client, IpEndpoint::new(A, 13400), &oversized),
        Err(SocketError::BufferFull)
    );

    // A datagram longer than the receive buffer is truncated.
    stack
        .send_datagram(client, IpEndpoint::new(A, 13400), &[7u8; 32])
        .unwrap();
    let mut small = [0u8; 8];
    let (length, _) = stack.recv_datagram(server, &mut small).unwrap().unwrap();
    assert_eq!(length, 8);
    assert_eq!(small, [7u8; 8]);
}

#[test]
fn datagram_bind_conflicts_and_stale_handles() {
    let mut stack = Stack::new();
    let first = stack.create_datagram().unwrap();
    stack.bind_datagram(first, A, 13400).unwrap();
    let second = stack.create_datagram().unwrap();
    assert_eq!(
        stack.bind_datagram(second, A, 13400),
        Err(SocketError::AddressInUse)
    );
    assert_eq!(
        stack.bind_datagram(first, A, 13401),
        Err(SocketError::InvalidState)
    );

    // Close invalidates the handle; the slot is reused with a new one.
    stack.close_datagram(first).unwrap();
    assert_eq!(
        stack.datagram_local_endpoint(first),
        Err(SocketError::InvalidHandle)
    );
    assert_eq!(
        stack.send_datagram(first, IpEndpoint::new(A, 13400), b"x"),
        Err(SocketError::InvalidHandle)
    );
    let reused = stack.create_datagram().unwrap();
    assert_ne!(first, reused);
    assert!(stack.datagram_local_endpoint(reused).is_ok());
}

#[test]
fn link_state_gates_transmission_paths() {
    let mut stack = Stack::new();
    assert!(stack.link_up());

    // Established TCP pair.
    let listener = stack.create().unwrap();
    stack.bind(listener, A, 13400).unwrap();
    stack.listen(listener, 1).unwrap();
    let client = stack.create().unwrap();
    stack.connect(client, A, 13400).unwrap();
    let server = stack.accept(listener).unwrap().unwrap();
    stack.send(client, b"pre").unwrap();

    // Bound UDP pair.
    let udp_server = stack.create_datagram().unwrap();
    stack.bind_datagram(udp_server, A, 13500).unwrap();
    let udp_client = stack.create_datagram().unwrap();

    stack.set_link_up(false);
    assert!(!stack.link_up());
    assert_eq!(stack.send(client, b"x"), Err(SocketError::NotConnected));
    assert_eq!(
        stack.send_datagram(udp_client, IpEndpoint::new(A, 13500), b"x"),
        Err(SocketError::NotConnected)
    );
    let refused = stack.create().unwrap();
    assert_eq!(
        stack.connect(refused, A, 13400),
        Err(SocketError::NotConnected)
    );
    // Data delivered before link loss can still be drained.
    let mut buf = [0u8; 8];
    assert_eq!(stack.recv(server, &mut buf).unwrap(), 3);

    stack.set_link_up(true);
    stack.send(client, b"post").unwrap();
    assert_eq!(stack.recv(server, &mut buf).unwrap(), 4);
    assert!(matches!(stack.state(client), Ok(SocketState::Established)));
}
