//! Conformance tests for [`PosixSocketStack`] against the portable stack
//! boundary, using real loopback sockets.

#![cfg(feature = "std")]

use std::time::{Duration, Instant};

use bsw_ethernet::endpoint::IpEndpoint;
use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::{DatagramApi, SocketApi, SocketError, SocketState};
use bsw_ethernet::posix::PosixSocketStack;

type Stack = PosixSocketStack<8, 2>;

const LOCALHOST: IpAddress = IpAddress::V4([127, 0, 0, 1]);

/// Poll `predicate` until it succeeds or two seconds elapse.
fn wait_for<T>(mut attempt: impl FnMut() -> Option<T>) -> T {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        if let Some(value) = attempt() {
            return value;
        }
        assert!(Instant::now() < deadline, "timed out");
        std::thread::yield_now();
    }
}

#[test]
fn tcp_listen_accept_send_recv_close() {
    let mut stack = Stack::new();
    let listener = stack.create().unwrap();
    stack.bind(listener, LOCALHOST, 0).unwrap();
    stack.listen(listener, 4).unwrap();
    let port = stack
        .local_endpoint(listener)
        .unwrap()
        .port()
        .expect("listener port");
    assert_eq!(stack.state(listener), Ok(SocketState::Listening));

    let client = stack.create().unwrap();
    stack.connect(client, LOCALHOST, port).unwrap();
    assert_eq!(stack.state(client), Ok(SocketState::Established));
    let server = wait_for(|| stack.accept(listener).unwrap());
    assert_eq!(stack.state(server), Ok(SocketState::Established));

    assert_eq!(stack.send(client, b"hello"), Ok(5));
    let mut buf = [0u8; 16];
    let read = wait_for(|| match stack.recv(server, &mut buf) {
        Ok(0) => None,
        Ok(read) => Some(read),
        Err(error) => panic!("recv failed: {error:?}"),
    });
    assert_eq!(&buf[..read], b"hello");

    assert_eq!(stack.send(server, b"world"), Ok(5));
    let read = wait_for(|| match stack.recv(client, &mut buf) {
        Ok(0) => None,
        Ok(read) => Some(read),
        Err(error) => panic!("recv failed: {error:?}"),
    });
    assert_eq!(&buf[..read], b"world");

    // Remote close is observed as a Closed state after draining.
    stack.close(client).unwrap();
    wait_for(|| {
        let _ = stack.recv(server, &mut buf);
        match stack.state(server) {
            Ok(SocketState::Closed) => Some(()),
            _ => None,
        }
    });
    stack.close(server).unwrap();
    stack.close(listener).unwrap();
    assert_eq!(stack.state(listener), Err(SocketError::InvalidHandle));
}

#[test]
fn stale_handles_are_rejected_and_slots_reused() {
    let mut stack = Stack::new();
    let socket = stack.create().unwrap();
    stack.close(socket).unwrap();
    assert_eq!(stack.state(socket), Err(SocketError::InvalidHandle));
    let reused = stack.create().unwrap();
    assert_ne!(socket, reused);
    assert_eq!(stack.state(reused), Ok(SocketState::Created));
    stack.abort(socket); // stale abort is a no-op
    assert_eq!(stack.state(reused), Ok(SocketState::Created));
}

#[test]
fn tcp_pool_exhaustion_is_reported() {
    let mut stack: PosixSocketStack<2, 1> = PosixSocketStack::new();
    let a = stack.create().unwrap();
    let b = stack.create().unwrap();
    assert_eq!(stack.create(), Err(SocketError::NoResources));
    stack.close(a).unwrap();
    stack.close(b).unwrap();
    assert!(stack.create().is_ok());
}

#[test]
fn datagram_roundtrip_and_endpoints() {
    let mut stack = Stack::new();
    let server = stack.create_datagram().unwrap();
    stack.bind_datagram(server, LOCALHOST, 0).unwrap();
    let port = stack
        .datagram_local_endpoint(server)
        .unwrap()
        .port()
        .expect("server port");

    let client = stack.create_datagram().unwrap();
    stack
        .send_datagram(client, IpEndpoint::new(LOCALHOST, port), b"ping")
        .unwrap();
    let mut buf = [0u8; 16];
    let (length, from) = wait_for(|| stack.recv_datagram(server, &mut buf).unwrap());
    assert_eq!(&buf[..length], b"ping");
    assert!(from.port().is_some());

    // Reply to the reported source endpoint.
    stack.send_datagram(server, from, b"pong").unwrap();
    let (length, _) = wait_for(|| stack.recv_datagram(client, &mut buf).unwrap());
    assert_eq!(&buf[..length], b"pong");

    stack.close_datagram(client).unwrap();
    assert_eq!(
        stack.recv_datagram(client, &mut buf),
        Err(SocketError::InvalidHandle)
    );
}

#[test]
fn datagram_pool_exhaustion_is_reported() {
    let mut stack = Stack::new();
    let _a = stack.create_datagram().unwrap();
    let b = stack.create_datagram().unwrap();
    assert_eq!(stack.create_datagram(), Err(SocketError::NoResources));
    stack.close_datagram(b).unwrap();
    assert!(stack.create_datagram().is_ok());
}
