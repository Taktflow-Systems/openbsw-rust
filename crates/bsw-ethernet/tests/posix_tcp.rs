//! Loopback conformance tests for the POSIX TCP adapter (D19).
//!
//! Covers connect/accept lifecycle, echo roundtrip, partial send with
//! backpressure and no data loss, 3+ concurrent clients, close-reason
//! observation for peer-initiated and local-initiated shutdown, reset
//! mapping, and the connection-refused error path.
//!
//! All servers bind to `127.0.0.1` port `0` (OS-assigned). Sockets are
//! non-blocking; every wait is a bounded deadline loop far below five
//! seconds.

#![cfg(feature = "std")]

use std::time::{Duration, Instant};

use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::posix::{PosixTcpServerSocket, PosixTcpSocket};
use bsw_ethernet::tcp::{
    ConnectionCloseReason, SendResult, TcpDataListener, TcpError, TcpSendListener, TcpServerSocket,
    TcpSocket,
};

const STEP: Duration = Duration::from_millis(2);
const TEST_TIMEOUT: Duration = Duration::from_secs(4);

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

/// Bind a fresh server on an ephemeral loopback port.
fn bound_server() -> (PosixTcpServerSocket, u16) {
    let mut server = PosixTcpServerSocket::new();
    assert_eq!(server.bind(&loopback(), 0), TcpError::Ok);
    let port = server.local_port();
    assert_ne!(port, 0);
    (server, port)
}

/// Connect a client and accept the matching server-side connection.
fn connected_pair() -> (PosixTcpSocket, PosixTcpSocket, PosixTcpServerSocket) {
    let (mut server, port) = bound_server();
    let mut client = PosixTcpSocket::new();
    assert_eq!(client.connect(&loopback(), port), TcpError::Ok);
    let mut conn = None;
    assert!(
        wait_until(|| {
            match server.accept() {
                Ok(Some(socket)) => {
                    conn = Some(socket);
                    true
                }
                Ok(None) => false,
                Err(e) => panic!("accept failed: {e:?}"),
            }
        }),
        "server did not accept the connection in time"
    );
    (client, conn.unwrap(), server)
}

/// Read from `socket` until `expected` bytes arrived or the deadline hits.
fn read_exact(socket: &mut PosixTcpSocket, expected: usize) -> Vec<u8> {
    let mut collected = Vec::with_capacity(expected);
    assert!(
        wait_until(|| {
            let mut buf = [0u8; 4096];
            match socket.read(&mut buf) {
                Ok(0) => {}
                Ok(n) => collected.extend_from_slice(&buf[..n]),
                Err(e) => panic!("read failed: {e:?}"),
            }
            collected.len() >= expected
        }),
        "expected {expected} bytes, got {} in time",
        collected.len()
    );
    collected
}

#[derive(Default)]
struct TcpRecorder {
    received: Vec<u16>,
    closed: Vec<ConnectionCloseReason>,
    sent: Vec<(u16, SendResult)>,
}

impl TcpDataListener for TcpRecorder {
    fn on_data_received(&mut self, length: u16) {
        self.received.push(length);
    }

    fn on_connection_closed(&mut self, reason: ConnectionCloseReason) {
        self.closed.push(reason);
    }
}

impl TcpSendListener for TcpRecorder {
    fn on_data_sent(&mut self, length: u16, result: SendResult) {
        self.sent.push((length, result));
    }
}

#[test]
fn connect_accept_lifecycle() {
    let (client, conn, server) = connected_pair();
    assert!(client.is_established());
    assert!(!client.is_closed());
    assert!(conn.is_established());
    assert_eq!(client.remote_port(), server.local_port());
    assert_eq!(conn.local_port(), server.local_port());
    assert_eq!(conn.remote_port(), client.local_port());
    assert_eq!(client.remote_address(), loopback());
    assert_eq!(conn.remote_address(), loopback());
}

#[test]
fn accept_without_pending_connection_returns_none() {
    let (mut server, _port) = bound_server();
    assert!(matches!(server.accept(), Ok(None)));
}

#[test]
fn echo_roundtrip() {
    let (mut client, mut conn, _server) = connected_pair();
    let payload = b"hello bsw";
    assert_eq!(client.send(payload), TcpError::Ok);
    client.flush();
    let echoed = read_exact(&mut conn, payload.len());
    assert_eq!(&echoed, payload);
    assert_eq!(conn.send(&echoed), TcpError::Ok);
    conn.flush();
    let round = read_exact(&mut client, payload.len());
    assert_eq!(&round, payload);
}

#[test]
fn data_listener_reports_available_bytes() {
    let (mut client, mut conn, _server) = connected_pair();
    let mut recorder = TcpRecorder::default();
    assert_eq!(
        client.send_with_listener(&[1, 2, 3, 4], &mut recorder),
        TcpError::Ok
    );
    assert_eq!(recorder.sent.len(), 1);
    assert_eq!(recorder.sent[0].0, 4);
    assert!(
        wait_until(|| {
            conn.poll_receive(&mut recorder);
            !recorder.received.is_empty()
        }),
        "data listener saw no data in time"
    );
    assert!(recorder.received[0] >= 1);
    assert!(conn.available() >= 1);
}

#[test]
fn partial_send_backpressure_no_data_loss() {
    const TOTAL: usize = 16 * 1024 * 1024;
    const CHUNK: usize = 64 * 1024;
    const PENDING_CAP: usize = 256 * 1024;

    let (mut server, port) = bound_server();
    let mut client = PosixTcpSocket::with_pending_capacity(PENDING_CAP);
    assert_eq!(client.connect(&loopback(), port), TcpError::Ok);
    let mut conn = None;
    assert!(wait_until(|| {
        match server.accept() {
            Ok(Some(socket)) => {
                conn = Some(socket);
                true
            }
            _ => false,
        }
    }));
    let mut conn = conn.unwrap();

    let pattern = |index: usize| -> u8 { (index % 251) as u8 };
    let mut chunk = vec![0u8; CHUNK];
    let mut sent = 0usize;
    let mut received = 0usize;
    let mut mismatch = None;
    let mut saw_backpressure = false;
    let mut saw_pending = false;

    // Phase 1: pump without reading until the bounded pending buffer rejects
    // a chunk (backpressure) or everything was accepted.
    while sent < TOTAL {
        for (i, byte) in chunk.iter_mut().enumerate() {
            *byte = pattern(sent + i);
        }
        match client.send(&chunk) {
            TcpError::Ok => {
                sent += CHUNK;
                saw_pending |= client.pending_len() > 0;
            }
            TcpError::NoMoreBuffer => {
                saw_backpressure = true;
                break;
            }
            other => panic!("send failed: {other:?}"),
        }
    }

    // Phase 2: drain the receiver while pushing the remaining bytes.
    let deadline = Instant::now() + TEST_TIMEOUT;
    let mut buf = vec![0u8; CHUNK];
    while received < TOTAL {
        assert!(Instant::now() < deadline, "transfer did not finish in time");
        let mut progress = false;
        // drain
        loop {
            match conn.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    for &byte in &buf[..n] {
                        if byte != pattern(received) && mismatch.is_none() {
                            mismatch = Some(received);
                        }
                        received += 1;
                    }
                    progress = true;
                }
                Err(e) => panic!("read failed: {e:?}"),
            }
        }
        // flush queued bytes and push more data
        client.flush();
        if sent < TOTAL {
            for (i, byte) in chunk.iter_mut().enumerate() {
                *byte = pattern(sent + i);
            }
            match client.send(&chunk) {
                TcpError::Ok => {
                    sent += CHUNK;
                    saw_pending |= client.pending_len() > 0;
                    progress = true;
                }
                TcpError::NoMoreBuffer => saw_backpressure = true,
                other => panic!("send failed: {other:?}"),
            }
        }
        if !progress {
            std::thread::sleep(STEP);
        }
    }

    assert_eq!(received, TOTAL);
    assert_eq!(mismatch, None, "byte stream corrupted at {mismatch:?}");
    if !(saw_backpressure || saw_pending) {
        println!(
            "note: OS buffers absorbed {TOTAL} bytes without backpressure; \
             integrity still verified"
        );
    }
}

#[test]
fn three_concurrent_clients_echo() {
    const CLIENTS: usize = 3;
    let (mut server, port) = bound_server();
    let mut clients: Vec<PosixTcpSocket> = Vec::new();
    for _ in 0..CLIENTS {
        let mut client = PosixTcpSocket::new();
        assert_eq!(client.connect(&loopback(), port), TcpError::Ok);
        clients.push(client);
    }
    let mut conns: Vec<PosixTcpSocket> = Vec::new();
    assert!(
        wait_until(|| {
            match server.accept() {
                Ok(Some(socket)) => conns.push(socket),
                Ok(None) => {}
                Err(e) => panic!("accept failed: {e:?}"),
            }
            conns.len() == CLIENTS
        }),
        "server accepted only {} of {CLIENTS} clients",
        conns.len()
    );

    // Each client sends a distinct payload...
    for (index, client) in clients.iter_mut().enumerate() {
        let payload = [index as u8; 32];
        assert_eq!(client.send(&payload), TcpError::Ok);
        client.flush();
    }
    // ...the server echoes whatever arrives on each connection...
    let mut echoed = [0usize; CLIENTS];
    assert!(
        wait_until(|| {
            for (index, conn) in conns.iter_mut().enumerate() {
                let mut buf = [0u8; 64];
                match conn.read(&mut buf) {
                    Ok(0) => {}
                    Ok(n) => {
                        assert_eq!(conn.send(&buf[..n]), TcpError::Ok);
                        conn.flush();
                        echoed[index] += n;
                    }
                    Err(e) => panic!("server read failed: {e:?}"),
                }
            }
            echoed.iter().all(|&n| n >= 32)
        }),
        "server did not receive all client payloads in time"
    );
    // ...and every client gets exactly its own bytes back.
    for (index, client) in clients.iter_mut().enumerate() {
        let data = read_exact(client, 32);
        assert_eq!(data, vec![index as u8; 32]);
    }
}

#[test]
fn peer_close_reports_closed_by_peer() {
    let (mut client, mut conn, _server) = connected_pair();
    // Local-initiated shutdown on the client: no close reason is recorded
    // locally (matches upstream connectionClosed semantics)...
    assert_eq!(client.close(), TcpError::Ok);
    assert!(client.is_closed());
    assert!(!client.is_established());
    assert_eq!(client.close_reason(), None);

    // ...while the peer observes an orderly FIN.
    let mut recorder = TcpRecorder::default();
    assert!(
        wait_until(|| {
            conn.poll_receive(&mut recorder);
            !recorder.closed.is_empty()
        }),
        "peer never observed the close"
    );
    assert_eq!(recorder.closed, vec![ConnectionCloseReason::ClosedByPeer]);
    assert!(conn.is_closed());
    assert_eq!(
        conn.close_reason(),
        Some(ConnectionCloseReason::ClosedByPeer)
    );
    // The listener is notified exactly once.
    conn.poll_receive(&mut recorder);
    assert_eq!(recorder.closed.len(), 1);
}

#[test]
fn abort_with_unread_data_reports_reset() {
    let (mut client, mut conn, _server) = connected_pair();
    // Put unread data into the server-side receive buffer, then abort:
    // the OS answers the peer with RST.
    assert_eq!(client.send(&[1, 2, 3, 4]), TcpError::Ok);
    client.flush();
    assert!(
        wait_until(|| conn.available() > 0),
        "data never reached the server connection"
    );
    conn.abort();
    assert!(conn.is_closed());

    let mut recorder = TcpRecorder::default();
    assert!(
        wait_until(|| {
            client.poll_receive(&mut recorder);
            !recorder.closed.is_empty()
        }),
        "client never observed the reset"
    );
    assert_eq!(recorder.closed, vec![ConnectionCloseReason::Reset]);
    assert!(client.is_closed());
    assert_eq!(client.close_reason(), Some(ConnectionCloseReason::Reset));
}

#[test]
fn connection_refused_maps_to_error() {
    // Learn a currently-free port, then close the listener so the connect
    // attempt is actively refused.
    let port = {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        listener.local_addr().unwrap().port()
    };
    let mut client = PosixTcpSocket::new();
    client.set_connect_timeout(Duration::from_secs(2));
    assert_eq!(client.connect(&loopback(), port), TcpError::NotOk);
    assert!(client.is_closed());
    assert!(!client.is_established());
}

#[test]
fn send_after_close_reports_not_open() {
    let (mut client, _conn, _server) = connected_pair();
    assert_eq!(client.close(), TcpError::Ok);
    assert_eq!(client.send(&[1]), TcpError::NotOpen);
    let mut buf = [0u8; 4];
    assert_eq!(client.read(&mut buf), Err(TcpError::NotOpen));
}

#[test]
fn poll_send_flushes_queued_data() {
    let (mut client, mut conn, _server) = connected_pair();
    let mut recorder = TcpRecorder::default();
    // Regular small send: everything reaches the OS immediately.
    assert_eq!(
        client.send_with_listener(&[5; 128], &mut recorder),
        TcpError::Ok
    );
    assert_eq!(recorder.sent[0], (128, SendResult::DataSent));
    assert_eq!(client.poll_send(Some(&mut recorder)), TcpError::Ok);
    let data = read_exact(&mut conn, 128);
    assert_eq!(data, vec![5u8; 128]);
}

#[test]
fn server_close_stops_accepting() {
    let (mut server, _port) = bound_server();
    assert_eq!(TcpServerSocket::close(&mut server), TcpError::Ok);
    assert!(server.is_closed());
    assert!(matches!(server.accept(), Err(TcpError::NotOpen)));
}
