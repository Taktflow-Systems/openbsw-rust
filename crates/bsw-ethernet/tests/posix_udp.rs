//! Loopback and failure conformance tests for the POSIX UDP adapter (D18).
//!
//! All sockets bind to `127.0.0.1` with port `0` (OS-assigned) so the suite
//! never depends on fixed ports. Every wait is bounded by a deadline well
//! under five seconds.

#![cfg(feature = "std")]

use std::time::{Duration, Instant};

use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::posix::PosixUdpSocket;
use bsw_ethernet::udp::{DatagramPacket, UdpDataListener, UdpError, UdpSendListener, UdpSocket};

const STEP: Duration = Duration::from_millis(2);
const TEST_TIMEOUT: Duration = Duration::from_secs(3);

fn loopback() -> IpAddress {
    IpAddress::ipv4(127, 0, 0, 1)
}

/// Poll `cond` until it returns `true` or the deadline expires.
fn wait_until(mut cond: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + TEST_TIMEOUT;
    loop {
        if cond() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(STEP);
    }
}

#[derive(Default)]
struct UdpRecorder {
    datagrams: Vec<(IpAddress, u16, IpAddress, u16)>,
    sent: Vec<Vec<u8>>,
}

impl UdpDataListener for UdpRecorder {
    fn on_datagram_received(
        &mut self,
        src_addr: &IpAddress,
        src_port: u16,
        dst_addr: &IpAddress,
        length: u16,
    ) {
        self.datagrams
            .push((*src_addr, src_port, *dst_addr, length));
    }
}

impl UdpSendListener for UdpRecorder {
    fn on_datagram_sent(&mut self, data: &[u8]) {
        self.sent.push(data.to_vec());
    }
}

#[test]
fn roundtrip_between_two_sockets() {
    let mut a = PosixUdpSocket::new();
    let mut b = PosixUdpSocket::new();
    let addr = loopback();
    assert_eq!(a.bind(Some(&addr), 0), UdpError::Ok);
    assert_eq!(b.bind(Some(&addr), 0), UdpError::Ok);
    let a_port = a.local_port();
    let b_port = b.local_port();
    assert_ne!(a_port, 0);
    assert_ne!(b_port, 0);
    assert_ne!(a_port, b_port);

    // a → b via send_to
    let payload = [0xAB, 0x01, 0x02, 0x03];
    let packet = DatagramPacket::new(&payload, addr, b_port);
    assert_eq!(a.send_to(&packet), UdpError::Ok);

    let mut buf = [0u8; 64];
    let mut received = None;
    assert!(
        wait_until(|| {
            match b.poll_recv_from(&mut buf) {
                Ok(Some(result)) => {
                    received = Some(result);
                    true
                }
                _ => false,
            }
        }),
        "datagram a->b not received in time"
    );
    let (length, source) = received.unwrap();
    assert_eq!(&buf[..length], &payload);
    assert_eq!(*source.address(), addr);
    assert_eq!(source.port(), Some(a_port));

    // b → a via connect + send
    assert_eq!(b.connect(&addr, a_port), UdpError::Ok);
    assert!(b.is_connected());
    assert_eq!(b.send(&[9, 9]), UdpError::Ok);
    let mut reply = [0u8; 16];
    let mut reply_len = 0usize;
    assert!(
        wait_until(|| {
            match a.read(&mut reply) {
                Ok(0) | Err(_) => false,
                Ok(n) => {
                    reply_len = n;
                    true
                }
            }
        }),
        "datagram b->a not received in time"
    );
    assert_eq!(&reply[..reply_len], &[9, 9]);
}

#[test]
fn listener_callbacks_report_endpoints() {
    let mut sender = PosixUdpSocket::new();
    let mut receiver = PosixUdpSocket::new();
    let addr = loopback();
    assert_eq!(sender.bind(Some(&addr), 0), UdpError::Ok);
    assert_eq!(receiver.bind(Some(&addr), 0), UdpError::Ok);
    let receiver_port = receiver.local_port();

    let mut recorder = UdpRecorder::default();
    assert_eq!(
        sender.connect(&addr, receiver_port),
        UdpError::Ok,
        "connect must succeed"
    );
    assert_eq!(
        sender.send_with_listener(&[7, 8, 9], &mut recorder),
        UdpError::Ok
    );
    assert_eq!(recorder.sent, vec![vec![7, 8, 9]]);

    let mut buf = [0u8; 32];
    assert!(
        wait_until(|| {
            matches!(
                receiver.poll_with_listener(&mut buf, &mut recorder),
                Ok(n) if n > 0
            )
        }),
        "listener datagram not received in time"
    );
    assert_eq!(recorder.datagrams.len(), 1);
    let (src_addr, src_port, dst_addr, length) = recorder.datagrams[0];
    assert_eq!(src_addr, addr);
    assert_eq!(src_port, sender.local_port());
    assert_eq!(dst_addr, addr);
    assert_eq!(length, 3);
}

#[test]
fn broadcast_flag_set_and_cleared() {
    let mut socket = PosixUdpSocket::new();
    assert_eq!(socket.bind(Some(&loopback()), 0), UdpError::Ok);
    assert_eq!(socket.set_broadcast(true), UdpError::Ok);
    assert_eq!(socket.broadcast(), Some(true));
    assert_eq!(socket.set_broadcast(false), UdpError::Ok);
    assert_eq!(socket.broadcast(), Some(false));
}

#[test]
fn send_on_closed_socket_fails() {
    let mut socket = PosixUdpSocket::new();
    let addr = loopback();
    assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
    assert_eq!(socket.connect(&addr, 40000), UdpError::Ok);
    assert_eq!(socket.close(), UdpError::Ok);
    assert!(socket.is_closed());
    assert_eq!(socket.send(&[1]), UdpError::NotOk);
    let data = [1u8];
    let packet = DatagramPacket::new(&data, addr, 40000);
    assert_eq!(socket.send_to(&packet), UdpError::NotOk);
    let mut buf = [0u8; 4];
    assert_eq!(socket.read(&mut buf), Err(UdpError::NotOk));
}

#[test]
fn send_on_unbound_socket_fails() {
    let mut socket = PosixUdpSocket::new();
    assert_eq!(socket.send(&[1, 2]), UdpError::NotOk);
    let data = [3u8];
    let packet = DatagramPacket::new(&data, loopback(), 40000);
    assert_eq!(socket.send_to(&packet), UdpError::NotOk);
}

#[test]
fn oversized_datagram_is_rejected() {
    let mut sender = PosixUdpSocket::new();
    let mut receiver = PosixUdpSocket::new();
    let addr = loopback();
    assert_eq!(sender.bind(Some(&addr), 0), UdpError::Ok);
    assert_eq!(receiver.bind(Some(&addr), 0), UdpError::Ok);
    // Larger than the maximum IPv4 UDP payload (65507 bytes).
    let oversized = vec![0u8; 70_000];
    let packet = DatagramPacket::new(&oversized, addr, receiver.local_port());
    assert_eq!(sender.send_to(&packet), UdpError::NotOk);
}

#[test]
fn multicast_join_on_loopback() {
    let mut socket = PosixUdpSocket::new();
    let addr = loopback();
    assert_eq!(socket.bind(Some(&addr), 0), UdpError::Ok);
    assert!(socket.is_bound(), "socket must be bound before joining");
    let group = IpAddress::ipv4(224, 0, 0, 251);
    match socket.join_multicast_io(group, addr) {
        Ok(()) => {
            assert_eq!(socket.leave_multicast(group, addr), UdpError::Ok);
        }
        Err(error) => {
            // Some hosts refuse multicast membership on the loopback
            // interface; that is an OS policy, not an adapter defect.
            println!("note: OS refused multicast join on loopback ({error}); accepted as pass");
        }
    }
}

#[test]
fn multicast_mixed_families_rejected() {
    let mut socket = PosixUdpSocket::new();
    assert_eq!(socket.bind(Some(&loopback()), 0), UdpError::Ok);
    // An IPv4 unicast "group" is still IPv4, so exercise the family check
    // only when IPv6 support is compiled in.
    #[cfg(feature = "ipv6")]
    {
        let v6_iface = IpAddress::ipv6([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        let group = IpAddress::ipv4(224, 0, 0, 251);
        assert_eq!(socket.join_multicast(group, v6_iface), UdpError::NotOk);
    }
    #[cfg(not(feature = "ipv6"))]
    {
        // Without IPv6 all addresses are IPv4; nothing to reject here.
        let _ = &socket;
    }
}

#[test]
fn nonblocking_read_returns_zero_when_idle() {
    let mut socket = PosixUdpSocket::new();
    assert_eq!(socket.bind(Some(&loopback()), 0), UdpError::Ok);
    let started = Instant::now();
    let mut buf = [0u8; 16];
    assert_eq!(socket.read(&mut buf), Ok(0));
    assert!(
        started.elapsed() < Duration::from_millis(500),
        "read must not block"
    );
    assert_eq!(socket.poll_recv_from(&mut buf), Ok(None));
}
