//! Socket-contract conformance suite for the lwIP boundary fake (D20).
//!
//! Exercises every [`SocketApi`] operation and event on the deterministic
//! in-memory backend: lifecycle, connect/accept pairing, data transfer with
//! bounded-buffer backpressure, close/abort reason propagation, failure
//! injection, and handle invalidation.

use bsw_ethernet::ip::IpAddress;
use bsw_ethernet::lwip::fake::{FakeStack, MAX_BACKLOG};
use bsw_ethernet::lwip::{
    SocketApi, SocketError, SocketEvent, SocketEventListener, SocketId, SocketState,
};
use bsw_ethernet::tcp::ConnectionCloseReason;

fn server_addr() -> IpAddress {
    IpAddress::ipv4(127, 0, 0, 1)
}

/// Build a stack with an established client/server connection pair.
fn established_pair<const SOCKETS: usize, const RX: usize, const EVENTS: usize>(
    stack: &mut FakeStack<SOCKETS, RX, EVENTS>,
    port: u16,
) -> (SocketId, SocketId, SocketId) {
    let listener = stack.create().unwrap();
    stack.bind(listener, server_addr(), port).unwrap();
    stack.listen(listener, 4).unwrap();
    let client = stack.create().unwrap();
    stack.connect(client, server_addr(), port).unwrap();
    let server = stack.accept(listener).unwrap().unwrap();
    (client, server, listener)
}

#[derive(Default)]
struct EventRecorder {
    events: Vec<(SocketId, SocketEvent)>,
}

impl SocketEventListener for EventRecorder {
    fn on_socket_event(&mut self, socket: SocketId, event: SocketEvent) {
        self.events.push((socket, event));
    }
}

#[test]
fn create_bind_listen_connect_accept_lifecycle() {
    let mut stack: FakeStack = FakeStack::new();
    let listener = stack.create().unwrap();
    assert_eq!(stack.state(listener), Ok(SocketState::Created));
    assert_eq!(stack.local_endpoint(listener).unwrap().port(), None);

    stack.bind(listener, server_addr(), 13400).unwrap();
    assert_eq!(stack.state(listener), Ok(SocketState::Bound));
    assert_eq!(stack.local_endpoint(listener).unwrap().port(), Some(13400));

    stack.listen(listener, 4).unwrap();
    assert_eq!(stack.state(listener), Ok(SocketState::Listening));

    let client = stack.create().unwrap();
    stack.connect(client, server_addr(), 13400).unwrap();
    assert_eq!(stack.state(client), Ok(SocketState::Connecting));

    let server = stack.accept(listener).unwrap().unwrap();
    assert_eq!(stack.state(client), Ok(SocketState::Established));
    assert_eq!(stack.state(server), Ok(SocketState::Established));

    // endpoints pair up
    let client_local = stack.local_endpoint(client).unwrap();
    let server_remote = stack.remote_endpoint(server).unwrap();
    assert_eq!(client_local.port(), server_remote.port());
    let server_local = stack.local_endpoint(server).unwrap();
    assert_eq!(server_local.port(), Some(13400));
    assert_eq!(*server_local.address(), server_addr());
    let client_remote = stack.remote_endpoint(client).unwrap();
    assert_eq!(client_remote.port(), Some(13400));

    // the client observes exactly one Connected event
    assert_eq!(stack.poll_event(), Some((client, SocketEvent::Connected)));
    assert_eq!(stack.poll_event(), None);
}

#[test]
fn echo_roundtrip_with_events() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);
    let _ = stack.poll_event(); // drop Connected

    assert_eq!(stack.send(client, b"ping"), Ok(4));
    assert_eq!(
        stack.poll_event(),
        Some((client, SocketEvent::DataSent { length: 4 }))
    );
    assert_eq!(
        stack.poll_event(),
        Some((server, SocketEvent::DataReceived { length: 4 }))
    );

    let mut buf = [0u8; 16];
    assert_eq!(stack.recv(server, &mut buf), Ok(4));
    assert_eq!(&buf[..4], b"ping");
    // buffer drained
    assert_eq!(stack.recv(server, &mut buf), Ok(0));

    assert_eq!(stack.send(server, b"pong"), Ok(4));
    let mut reply = [0u8; 16];
    assert_eq!(stack.recv(client, &mut reply), Ok(4));
    assert_eq!(&reply[..4], b"pong");
}

#[test]
fn bounded_buffer_backpressure() {
    let mut stack: FakeStack<4, 8, 16> = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);

    // Peer buffer holds 8 bytes: a 6-byte send fits...
    assert_eq!(stack.send(client, &[1, 2, 3, 4, 5, 6]), Ok(6));
    // ...the next send is truncated to the remaining space (partial write)...
    assert_eq!(stack.send(client, &[7, 8, 9]), Ok(2));
    // ...and with a full buffer not a single byte is accepted.
    assert_eq!(stack.send(client, &[10]), Err(SocketError::BufferFull));

    // Draining the peer re-opens the window; nothing was lost or reordered.
    let mut buf = [0u8; 8];
    assert_eq!(stack.recv(server, &mut buf), Ok(8));
    assert_eq!(buf, [1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(stack.send(client, &[10]), Ok(1));
    assert_eq!(stack.recv(server, &mut buf), Ok(1));
    assert_eq!(buf[0], 10);
}

#[test]
fn connect_without_listener_is_refused() {
    let mut stack: FakeStack = FakeStack::new();
    let client = stack.create().unwrap();
    assert_eq!(
        stack.connect(client, server_addr(), 13400),
        Err(SocketError::ConnectionRefused)
    );
    assert_eq!(stack.state(client), Ok(SocketState::Created));
}

#[test]
fn injected_connect_refusal() {
    let mut stack: FakeStack = FakeStack::new();
    let listener = stack.create().unwrap();
    stack.bind(listener, server_addr(), 13400).unwrap();
    stack.listen(listener, 2).unwrap();
    let client = stack.create().unwrap();

    stack.set_refuse_connect(true);
    assert_eq!(
        stack.connect(client, server_addr(), 13400),
        Err(SocketError::ConnectionRefused)
    );
    stack.set_refuse_connect(false);
    assert_eq!(stack.connect(client, server_addr(), 13400), Ok(()));
}

#[test]
fn injected_data_drop() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);
    let _ = stack.poll_event(); // drop Connected

    stack.set_drop_data(true);
    assert_eq!(stack.send(client, b"lost"), Ok(4));
    // Sender still observes completion...
    assert_eq!(
        stack.poll_event(),
        Some((client, SocketEvent::DataSent { length: 4 }))
    );
    // ...but nothing was delivered.
    assert_eq!(stack.poll_event(), None);
    let mut buf = [0u8; 8];
    assert_eq!(stack.recv(server, &mut buf), Ok(0));

    stack.set_drop_data(false);
    assert_eq!(stack.send(client, b"kept"), Ok(4));
    assert_eq!(stack.recv(server, &mut buf), Ok(4));
    assert_eq!(&buf[..4], b"kept");
}

#[test]
fn injected_error_event() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, _server, _listener) = established_pair(&mut stack, 13400);
    let _ = stack.poll_event(); // drop Connected

    stack
        .inject_error(client, SocketError::NoResources)
        .unwrap();
    assert_eq!(
        stack.poll_event(),
        Some((
            client,
            SocketEvent::Error {
                error: SocketError::NoResources
            }
        ))
    );
    // Injecting on an invalid handle is rejected.
    let bogus = SocketId::from_raw(0xFFFF);
    assert_eq!(
        stack.inject_error(bogus, SocketError::NoResources),
        Err(SocketError::InvalidHandle)
    );
}

#[test]
fn close_notifies_peer_and_invalidates_handle() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);
    let _ = stack.poll_event(); // drop Connected

    // Leave undelivered data in the server's buffer before closing.
    assert_eq!(stack.send(client, &[1, 2, 3]), Ok(3));
    let _ = stack.poll_event(); // DataSent
    let _ = stack.poll_event(); // DataReceived

    stack.close(client).unwrap();
    // The closed handle is gone...
    assert_eq!(stack.state(client), Err(SocketError::InvalidHandle));
    assert_eq!(stack.close(client), Err(SocketError::InvalidHandle));
    // ...the peer got the reason...
    assert_eq!(
        stack.poll_event(),
        Some((
            server,
            SocketEvent::ConnectionClosed {
                reason: ConnectionCloseReason::ClosedByPeer
            }
        ))
    );
    assert_eq!(stack.state(server), Ok(SocketState::Closed));
    // ...can still drain buffered data...
    let mut buf = [0u8; 8];
    assert_eq!(stack.recv(server, &mut buf), Ok(3));
    assert_eq!(&buf[..3], &[1, 2, 3]);
    // ...but cannot send anymore.
    assert_eq!(stack.send(server, &[9]), Err(SocketError::Closed));
}

#[test]
fn abort_notifies_peer_with_reset() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);
    let _ = stack.poll_event(); // drop Connected

    stack.abort(server);
    assert_eq!(
        stack.poll_event(),
        Some((
            client,
            SocketEvent::ConnectionClosed {
                reason: ConnectionCloseReason::Reset
            }
        ))
    );
    assert_eq!(stack.state(client), Ok(SocketState::Closed));
    // Abort on an invalid handle is a no-op.
    stack.abort(server);
}

#[test]
fn accept_empty_backlog_returns_none() {
    let mut stack: FakeStack = FakeStack::new();
    let listener = stack.create().unwrap();
    stack.bind(listener, server_addr(), 13400).unwrap();
    stack.listen(listener, 2).unwrap();
    assert_eq!(stack.accept(listener), Ok(None));
}

#[test]
fn backlog_overflow_is_rejected() {
    let mut stack: FakeStack<16, 32, 32> = FakeStack::new();
    let listener = stack.create().unwrap();
    stack.bind(listener, server_addr(), 13400).unwrap();
    stack.listen(listener, MAX_BACKLOG as u8).unwrap();
    let mut clients = Vec::new();
    for _ in 0..MAX_BACKLOG {
        let client = stack.create().unwrap();
        stack.connect(client, server_addr(), 13400).unwrap();
        clients.push(client);
    }
    let overflow = stack.create().unwrap();
    assert_eq!(
        stack.connect(overflow, server_addr(), 13400),
        Err(SocketError::NoResources)
    );
    // The queued clients still get accepted in FIFO order.
    for client in &clients {
        let server = stack.accept(listener).unwrap().unwrap();
        assert_eq!(
            stack.remote_endpoint(server).unwrap().port(),
            stack.local_endpoint(*client).unwrap().port()
        );
    }
    assert_eq!(stack.accept(listener), Ok(None));
}

#[test]
fn socket_pool_exhaustion() {
    let mut stack: FakeStack<2, 16, 8> = FakeStack::new();
    let first = stack.create().unwrap();
    let _second = stack.create().unwrap();
    assert_eq!(stack.create(), Err(SocketError::NoResources));
    // Freeing a slot makes creation possible again.
    stack.close(first).unwrap();
    assert!(stack.create().is_ok());
}

#[test]
fn bind_conflicts_and_invalid_states() {
    let mut stack: FakeStack = FakeStack::new();
    let first = stack.create().unwrap();
    stack.bind(first, server_addr(), 13400).unwrap();
    let second = stack.create().unwrap();
    assert_eq!(
        stack.bind(second, server_addr(), 13400),
        Err(SocketError::AddressInUse)
    );
    // Wildcard overlaps a specific bind on the same port.
    assert_eq!(
        stack.bind(second, IpAddress::unspecified(), 13400),
        Err(SocketError::AddressInUse)
    );
    // Rebinding a bound socket is an invalid state transition.
    assert_eq!(
        stack.bind(first, server_addr(), 13401),
        Err(SocketError::InvalidState)
    );
    // listen requires Bound, accept requires Listening.
    assert_eq!(stack.listen(second, 1), Err(SocketError::InvalidState));
    assert_eq!(stack.accept(first), Err(SocketError::InvalidState));
    // send/recv require a connection.
    assert_eq!(stack.send(first, &[1]), Err(SocketError::NotConnected));
    let mut buf = [0u8; 4];
    assert_eq!(stack.recv(first, &mut buf), Err(SocketError::NotConnected));
}

#[test]
fn listener_close_refuses_pending_clients() {
    let mut stack: FakeStack = FakeStack::new();
    let listener = stack.create().unwrap();
    stack.bind(listener, server_addr(), 13400).unwrap();
    stack.listen(listener, 2).unwrap();
    let client = stack.create().unwrap();
    stack.connect(client, server_addr(), 13400).unwrap();

    stack.close(listener).unwrap();
    assert_eq!(
        stack.poll_event(),
        Some((
            client,
            SocketEvent::Error {
                error: SocketError::ConnectionRefused
            }
        ))
    );
    assert_eq!(stack.state(client), Ok(SocketState::Closed));
}

#[test]
fn dispatch_events_drains_in_order() {
    let mut stack: FakeStack = FakeStack::new();
    let (client, server, _listener) = established_pair(&mut stack, 13400);
    assert_eq!(stack.send(client, &[1, 2]), Ok(2));

    let mut recorder = EventRecorder::default();
    let dispatched = stack.dispatch_events(&mut recorder);
    assert_eq!(dispatched, 3);
    assert_eq!(
        recorder.events,
        vec![
            (client, SocketEvent::Connected),
            (client, SocketEvent::DataSent { length: 2 }),
            (server, SocketEvent::DataReceived { length: 2 }),
        ]
    );
    assert_eq!(stack.dispatch_events(&mut recorder), 0);
}

#[test]
fn invalid_handles_are_rejected_everywhere() {
    let mut stack: FakeStack = FakeStack::new();
    let bogus = SocketId::from_raw(0x00FF); // index out of range
    let mut buf = [0u8; 4];
    assert_eq!(
        stack.bind(bogus, server_addr(), 1),
        Err(SocketError::InvalidHandle)
    );
    assert_eq!(stack.listen(bogus, 1), Err(SocketError::InvalidHandle));
    assert_eq!(
        stack.connect(bogus, server_addr(), 1),
        Err(SocketError::InvalidHandle)
    );
    assert_eq!(stack.accept(bogus), Err(SocketError::InvalidHandle));
    assert_eq!(stack.send(bogus, &[1]), Err(SocketError::InvalidHandle));
    assert_eq!(stack.recv(bogus, &mut buf), Err(SocketError::InvalidHandle));
    assert_eq!(stack.close(bogus), Err(SocketError::InvalidHandle));
    assert_eq!(stack.state(bogus), Err(SocketError::InvalidHandle));
    assert_eq!(stack.local_endpoint(bogus), Err(SocketError::InvalidHandle));
    assert_eq!(
        stack.remote_endpoint(bogus),
        Err(SocketError::InvalidHandle)
    );
    stack.abort(bogus); // must not panic
}

#[test]
fn event_queue_overflow_is_counted() {
    let mut stack: FakeStack<4, 64, 2> = FakeStack::new();
    let (client, _server, _listener) = established_pair(&mut stack, 13400);
    // Queue capacity is 2: Connected + one DataSent fill it; the
    // DataReceived of the same send is dropped and counted.
    assert_eq!(stack.send(client, &[1]), Ok(1));
    assert_eq!(stack.dropped_events(), 1);
    assert_eq!(stack.poll_event(), Some((client, SocketEvent::Connected)));
    assert!(stack.poll_event().is_some());
    assert_eq!(stack.poll_event(), None);
}
