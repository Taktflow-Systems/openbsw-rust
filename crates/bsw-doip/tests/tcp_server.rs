//! E28 fake-clock tests: DoIP TCP routing activation, activation policy,
//! alive checks, inactivity timers, connection limits, and teardown.
//!
//! Golden byte vectors match the upstream tests in
//! `libs/bsw/doip/test/src/doip/server/`.  All timing uses exact
//! [`Instant`] boundaries — no sleeps.

use bsw_doip::{
    AcceptError, Action, ActivationDecision, ActivationPolicy, CloseMode, ConnectionState,
    DefaultActivationPolicy, MessageHandler, NoMessageHandler, ProtocolVersion,
    RoutingActivationCode, ServerConnection, ServerEvent, ServerTransportLayer,
    TransportParameters, WireAction,
};
use bsw_time::Instant;

const ENTITY: u16 = 0x0123;
const NANOS_PER_MS: u64 = 1_000_000;

fn at(ms: u64) -> Instant {
    Instant::from_nanos(ms * NANOS_PER_MS)
}

/// One nanosecond before `at(ms)`.
fn just_before(ms: u64) -> Instant {
    Instant::from_nanos(ms * NANOS_PER_MS - 1)
}

// ---------------------------------------------------------------------------
// Golden byte vectors (upstream test values)
// ---------------------------------------------------------------------------

fn ra_request(source: u16, activation_type: u8) -> [u8; 15] {
    let s = source.to_be_bytes();
    [
        0x02,
        0xfd,
        0x00,
        0x05,
        0x00,
        0x00,
        0x00,
        0x07,
        s[0],
        s[1],
        activation_type,
        0x00,
        0x00,
        0x00,
        0x00,
    ]
}

fn ra_response(tester: u16, code: u8) -> [u8; 17] {
    let s = tester.to_be_bytes();
    [
        0x02, 0xfd, 0x00, 0x06, 0x00, 0x00, 0x00, 0x09, s[0], s[1], 0x01, 0x23, code, 0x00, 0x00,
        0x00, 0x00,
    ]
}

const ALIVE_CHECK_REQUEST: [u8; 8] = [0x02, 0xfd, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00];

fn alive_response(source: u16) -> [u8; 10] {
    let s = source.to_be_bytes();
    [0x02, 0xfd, 0x00, 0x08, 0x00, 0x00, 0x00, 0x02, s[0], s[1]]
}

fn nack(code: u8) -> [u8; 9] {
    [0x02, 0xfd, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, code]
}

#[test]
fn golden_vectors_match_upstream_bytes() {
    assert_eq!(
        ra_request(0x1234, 0x03),
        [0x02, 0xfd, 0x00, 0x05, 0, 0, 0, 7, 0x12, 0x34, 0x03, 0, 0, 0, 0]
    );
    assert_eq!(
        ra_response(0x1234, 0x11),
        [0x02, 0xfd, 0x00, 0x06, 0, 0, 0, 9, 0x12, 0x34, 0x01, 0x23, 0x11, 0, 0, 0, 0]
    );
    assert_eq!(
        alive_response(0x1234),
        [0x02, 0xfd, 0x00, 0x08, 0, 0, 0, 2, 0x12, 0x34]
    );
    assert_eq!(nack(0x04), [0x02, 0xfd, 0x00, 0x00, 0, 0, 0, 1, 0x04]);
}

// ---------------------------------------------------------------------------
// Test policies and handlers
// ---------------------------------------------------------------------------

/// Records every check and returns a fixed decision.
struct ScriptedPolicy {
    decision: ActivationDecision,
    calls: Vec<(u16, u8, bool)>,
}

impl ScriptedPolicy {
    fn new(decision: ActivationDecision) -> Self {
        Self {
            decision,
            calls: Vec::new(),
        }
    }
}

impl ActivationPolicy for ScriptedPolicy {
    fn check_routing_activation(
        &mut self,
        source_address: u16,
        activation_type: u8,
        is_resuming: bool,
    ) -> ActivationDecision {
        self.calls
            .push((source_address, activation_type, is_resuming));
        self.decision
    }
}

/// Refuses every connection at the admission filter.
struct RefusingFilter;

impl ActivationPolicy for RefusingFilter {
    fn check_routing_activation(&mut self, _: u16, _: u8, _: bool) -> ActivationDecision {
        ActivationDecision::proceed()
    }

    fn filter_connection(&mut self) -> bool {
        false
    }
}

/// Accepts one payload type and records forwarded payload bytes.
struct RecordingHandler {
    accepted_type: u16,
    headers: Vec<(u16, u32)>,
    payload: Vec<u8>,
}

impl MessageHandler for RecordingHandler {
    fn header_received(&mut self, payload_type: u16, payload_length: u32) -> bool {
        self.headers.push((payload_type, payload_length));
        payload_type == self.accepted_type
    }

    fn payload_chunk(&mut self, bytes: &[u8]) {
        self.payload.extend_from_slice(bytes);
    }
}

// ---------------------------------------------------------------------------
// Assertion helpers
// ---------------------------------------------------------------------------

fn assert_conn_send(action: Option<Action>, expected: &[u8]) {
    match action {
        Some(Action::Send(frame)) => assert_eq!(frame.bytes(), expected),
        other => panic!("expected Send, got {other:?}"),
    }
}

fn assert_wire_send(action: Option<WireAction>, expected: &[u8]) {
    match action {
        Some(WireAction::Send(frame)) => assert_eq!(frame.bytes(), expected),
        other => panic!("expected Send, got {other:?}"),
    }
}

fn default_connection() -> ServerConnection {
    ServerConnection::new(
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters::default(),
    )
}

/// Fully activated standalone connection (source address bound, `Active`).
fn activated_connection(source: u16) -> ServerConnection {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(0),
        &ra_request(source, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
    conn.routing_activation_completed(at(0), true, RoutingActivationCode::Success);
    assert_conn_send(conn.take_action(), &ra_response(source, 0x10));
    assert_eq!(conn.take_action(), Some(Action::RoutingActive));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Active);
    conn
}

fn layer_with(max_connection_count: u8) -> ServerTransportLayer {
    let parameters = TransportParameters {
        max_connection_count,
        ..TransportParameters::default()
    };
    ServerTransportLayer::new(ProtocolVersion::Iso2012, ENTITY, parameters)
}

/// Accept and fully activate a layer connection with the default policy.
fn activate_on_layer(
    layer: &mut ServerTransportLayer,
    now: Instant,
    source: u16,
) -> bsw_doip::ConnectionId {
    let mut policy = DefaultActivationPolicy;
    let id = layer.accept(now, &mut policy).unwrap();
    assert!(layer.handle_bytes(
        now,
        id,
        &ra_request(source, 0x00),
        &mut policy,
        &mut NoMessageHandler
    ));
    assert_wire_send(layer.take_action_for(id), &ra_response(source, 0x10));
    assert_eq!(layer.take_action_for(id), None);
    assert!(matches!(
        layer.take_event(),
        Some(ServerEvent::RoutingActive { .. })
    ));
    assert_eq!(layer.connection_state(id), Some(ConnectionState::Active));
    id
}

// ---------------------------------------------------------------------------
// Connection-level tests
// ---------------------------------------------------------------------------

#[test]
fn golden_routing_activation_handshake_with_keep_policy() {
    // Upstream golden vectors: request SA 0x1234 type 0x03, response code
    // 0x11 (ConfirmationRequired) from entity 0x0123.
    let mut policy = ScriptedPolicy::new(ActivationDecision::keep(
        RoutingActivationCode::ConfirmationRequired,
    ));
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &ra_request(0x1234, 0x03),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(policy.calls, vec![(0x1234, 0x03, false)]);
    assert_conn_send(conn.take_action(), &ra_response(0x1234, 0x11));
    // Keep: response sent, connection stays open and inactive.
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Inactive);
}

#[test]
fn successful_activation_emits_success_response_and_routing_active() {
    let conn = activated_connection(0x1234);
    assert!(conn.is_or_was_routing());
    assert_eq!(conn.source_address(), Some(0x1234));
    assert_eq!(conn.internal_source_address(), Some(0x1234));
}

#[test]
fn unsupported_activation_type_rejected_with_0x06_and_close() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &ra_request(0x1234, 0x02),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &ra_response(0x1234, 0x06));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Shutdown);
}

#[test]
fn policy_reject_unknown_source_address_closes() {
    let mut policy = ScriptedPolicy::new(ActivationDecision::reject(
        RoutingActivationCode::UnknownSourceAddress,
    ));
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &ra_request(0x0e00, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &ra_response(0x0e00, 0x00));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn second_request_same_source_succeeds_without_close() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    conn.handle_bytes(
        at(5),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &ra_response(0x1234, 0x10));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Active);
}

#[test]
fn different_source_address_rejected_with_0x02_and_close() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    conn.handle_bytes(
        at(5),
        &ra_request(0x5678, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &ra_response(0x5678, 0x02));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn initial_inactivity_expires_exactly_at_2000_ms() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.poll(just_before(2000));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Inactive);
    conn.poll(at(2000));
    // INACTIVE expiry closes with FIN.
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn activating_state_ignores_timer_expiry() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
    // The initial-inactivity deadline passes while the layer arbitrates:
    // upstream ignores expiry outside INACTIVE/ACTIVE.
    conn.poll(at(2000));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Activating);
}

#[test]
fn general_inactivity_rearmed_by_traffic_then_aborts() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    // Activation at t=0 armed the deadline for t=300000.
    conn.handle_bytes(
        at(100_000),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &ra_response(0x1234, 0x10));
    conn.poll(at(300_000));
    assert_eq!(conn.take_action(), None); // re-armed to 400000
    conn.poll(just_before(400_000));
    assert_eq!(conn.take_action(), None);
    conn.poll(at(400_000));
    // General inactivity closes abortively (RST intent).
    assert_eq!(conn.take_action(), Some(Action::Abort));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn alive_check_request_golden_bytes_and_timely_response() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    conn.start_alive_check(at(1000));
    assert_conn_send(conn.take_action(), &ALIVE_CHECK_REQUEST);
    assert!(conn.alive_check_pending());
    conn.poll(just_before(1500));
    assert_eq!(conn.take_action(), None);
    // Timely response with the golden bytes keeps the connection.
    conn.handle_bytes(
        at(1499),
        &alive_response(0x1234),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(
        conn.take_action(),
        Some(Action::AliveCheckResult { alive: true })
    );
    assert_eq!(conn.take_action(), None);
    assert!(!conn.alive_check_pending());
    // The response header re-armed the general-inactivity timer.
    conn.poll(at(1500));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Active);
}

#[test]
fn alive_check_timeout_closes_exactly_at_deadline() {
    let mut conn = activated_connection(0x1234);
    conn.start_alive_check(at(2000));
    assert_conn_send(conn.take_action(), &ALIVE_CHECK_REQUEST);
    conn.poll(just_before(2500));
    assert_eq!(conn.take_action(), None);
    conn.poll(at(2500));
    // Alive-check expiry closes with FIN; the failure notification follows
    // the Closed notification (upstream ordering guarantee).
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert_eq!(
        conn.take_action(),
        Some(Action::AliveCheckResult { alive: false })
    );
    assert!(conn.is_closed());
}

#[test]
fn alive_check_response_with_unexpected_source_closes() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    conn.start_alive_check(at(0));
    assert_conn_send(conn.take_action(), &ALIVE_CHECK_REQUEST);
    conn.handle_bytes(
        at(1),
        &alive_response(0x9999),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(
        conn.take_action(),
        Some(Action::AliveCheckResult { alive: false })
    );
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn alive_check_response_with_invalid_length_nacks_and_closes() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = activated_connection(0x1234);
    let bad = [
        0x02, 0xfd, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0x12, 0x34, 0x00,
    ];
    conn.handle_bytes(at(1), &bad, &mut policy, &mut NoMessageHandler);
    assert_conn_send(conn.take_action(), &nack(0x04));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn routing_activation_with_invalid_length_nacks_and_closes() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    let bad = [
        0x02, 0xfd, 0x00, 0x05, 0x00, 0x00, 0x00, 0x05, 0, 0, 0, 0, 0,
    ];
    conn.handle_bytes(at(1), &bad, &mut policy, &mut NoMessageHandler);
    assert_conn_send(conn.take_action(), &nack(0x04));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());
}

#[test]
fn version_mismatch_nacks_incorrect_pattern_and_closes() {
    // Wrong protocol version (0x03 with a valid inverse).
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &[0x03, 0xfc, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00],
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &nack(0x00));
    assert_eq!(conn.take_action(), Some(Action::Close));
    assert_eq!(conn.take_action(), Some(Action::Closed));
    assert!(conn.is_closed());

    // Broken inverse byte.
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &[0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00],
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &nack(0x00));
    assert_eq!(conn.take_action(), Some(Action::Close));
}

#[test]
fn unknown_payload_type_nacks_without_close() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    // Entity status request is not handled by the connection itself.
    conn.handle_bytes(
        at(1),
        &[0x02, 0xfd, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00],
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &nack(0x01));
    assert_eq!(conn.take_action(), None);
    assert_eq!(conn.state(), ConnectionState::Inactive);
    // The connection remains usable afterwards.
    conn.handle_bytes(
        at(2),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
}

#[test]
fn generic_negative_ack_is_consumed_silently() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(at(1), &nack(0x04), &mut policy, &mut NoMessageHandler);
    assert_eq!(conn.take_action(), None);
    conn.handle_bytes(
        at(2),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
}

#[test]
fn fragmented_delivery_byte_by_byte_and_split_payload() {
    // One byte at a time.
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    for byte in ra_request(0x1234, 0x00) {
        conn.handle_bytes(at(1), &[byte], &mut policy, &mut NoMessageHandler);
    }
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));

    // Header split across feeds, payload split across feeds.
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    let bytes = ra_request(0x1234, 0x00);
    conn.handle_bytes(at(1), &bytes[..3], &mut policy, &mut NoMessageHandler);
    assert_eq!(conn.take_action(), None);
    conn.handle_bytes(at(1), &bytes[3..9], &mut policy, &mut NoMessageHandler);
    assert_eq!(conn.take_action(), None);
    conn.handle_bytes(at(1), &bytes[9..], &mut policy, &mut NoMessageHandler);
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
}

#[test]
fn oversized_non_diagnostic_payload_is_discarded() {
    let mut policy = DefaultActivationPolicy;
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    // Entity status request with a 256-byte payload: larger than anything the
    // connection consumes — NACK 0x01 and discard.
    conn.handle_bytes(
        at(1),
        &[0x02, 0xfd, 0x40, 0x01, 0x00, 0x00, 0x01, 0x00],
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_conn_send(conn.take_action(), &nack(0x01));
    let junk = [0xAA_u8; 100];
    conn.handle_bytes(at(2), &junk, &mut policy, &mut NoMessageHandler);
    conn.handle_bytes(at(2), &junk, &mut policy, &mut NoMessageHandler);
    conn.handle_bytes(at(2), &junk[..56], &mut policy, &mut NoMessageHandler);
    assert_eq!(conn.take_action(), None);
    // Framing recovered: the next message parses normally.
    conn.handle_bytes(
        at(3),
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(conn.take_action(), Some(Action::ActivationRequested));
}

#[test]
fn message_handler_hook_takes_over_payload() {
    let mut policy = DefaultActivationPolicy;
    let mut handler = RecordingHandler {
        accepted_type: 0x8001,
        headers: Vec::new(),
        payload: Vec::new(),
    };
    let mut conn = default_connection();
    conn.start(at(0), &mut policy);
    conn.handle_bytes(
        at(1),
        &[0x02, 0xfd, 0x80, 0x01, 0x00, 0x00, 0x00, 0x05, 0x01, 0x02],
        &mut policy,
        &mut handler,
    );
    conn.handle_bytes(at(1), &[0x03, 0x04, 0x05], &mut policy, &mut handler);
    assert_eq!(handler.headers, vec![(0x8001, 5)]);
    assert_eq!(handler.payload, vec![1, 2, 3, 4, 5]);
    // Accepted by the handler: no NACK, no close.
    assert_eq!(conn.take_action(), None);
}

#[test]
fn resumed_connection_starts_active_with_resume_check() {
    let mut policy = ScriptedPolicy::new(ActivationDecision::proceed());
    let mut conn = default_connection();
    assert!(conn.resume(0x0777));
    conn.start(at(0), &mut policy);
    assert_eq!(policy.calls, vec![(0x0777, 0x00, true)]);
    assert_eq!(conn.take_action(), Some(Action::RoutingActive));
    assert_eq!(conn.state(), ConnectionState::Active);
    assert!(conn.is_or_was_routing());
    assert_eq!(conn.internal_source_address(), Some(0x0777));
    // Resume is rejected once started.
    assert!(!conn.resume(0x0778));
    // General-inactivity timer armed at start.
    conn.poll(just_before(300_000));
    assert_eq!(conn.take_action(), None);
    conn.poll(at(300_000));
    assert_eq!(conn.take_action(), Some(Action::Abort));
    assert_eq!(conn.take_action(), Some(Action::Closed));
}

// ---------------------------------------------------------------------------
// Transport-layer tests
// ---------------------------------------------------------------------------

#[test]
fn layer_activates_default_type_with_success_response() {
    let mut layer = layer_with(5);
    let id = activate_on_layer(&mut layer, at(0), 0x1234);
    assert_eq!(layer.routing_count(), 1);
    assert_eq!(
        layer.find_routing_connection_by_internal_source_address(0x1234),
        Some(id)
    );
}

#[test]
fn layer_reports_routing_active_addresses() {
    let mut layer = layer_with(5);
    let mut policy = DefaultActivationPolicy;
    let id = layer.accept(at(0), &mut policy).unwrap();
    layer.handle_bytes(
        at(0),
        id,
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(id), &ra_response(0x1234, 0x10));
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::RoutingActive {
            id,
            source_address: 0x1234,
            internal_source_address: 0x1234,
        })
    );
}

#[test]
fn layer_uses_policy_internal_source_address() {
    let mut layer = layer_with(5);
    let decision = ActivationDecision {
        internal_source_address: Some(0x00AA),
        ..ActivationDecision::proceed()
    };
    let mut policy = ScriptedPolicy::new(decision);
    let id = layer.accept(at(0), &mut policy).unwrap();
    layer.handle_bytes(
        at(0),
        id,
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(id), &ra_response(0x1234, 0x10));
    assert_eq!(
        layer.find_routing_connection_by_internal_source_address(0x00AA),
        Some(id)
    );
    assert_eq!(
        layer.find_routing_connection_by_internal_source_address(0x1234),
        None
    );
}

#[test]
fn layer_initial_inactivity_closes_and_releases_slot() {
    let mut layer = layer_with(5);
    let mut policy = DefaultActivationPolicy;
    let id = layer.accept(at(0), &mut policy).unwrap();
    layer.poll(just_before(2000));
    assert_eq!(layer.take_action(), None);
    layer.poll(at(2000));
    assert_eq!(layer.take_action_for(id), Some(WireAction::Close));
    assert_eq!(layer.connection_state(id), Some(ConnectionState::Shutdown));
    // Never routed: no ConnectionClosed notification.
    assert_eq!(layer.take_event(), None);
    // The drained slot is freed on the next poll.
    layer.poll(at(2001));
    assert_eq!(layer.connection_state(id), None);
    assert_eq!(layer.connection_count(), 0);
}

#[test]
fn layer_general_inactivity_rearms_and_aborts() {
    let mut layer = layer_with(5);
    let mut policy = DefaultActivationPolicy;
    let id = activate_on_layer(&mut layer, at(0), 0x1234);
    // Traffic at t=100000 re-arms the deadline to t=400000.
    layer.handle_bytes(
        at(100_000),
        id,
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(id), &ra_response(0x1234, 0x10));
    layer.poll(at(300_000));
    assert_eq!(layer.take_action(), None);
    layer.poll(just_before(400_000));
    assert_eq!(layer.take_action(), None);
    layer.poll(at(400_000));
    assert_eq!(layer.take_action_for(id), Some(WireAction::Abort));
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::ConnectionClosed {
            id,
            source_address: 0x1234
        })
    );
}

#[test]
fn layer_connection_limit_releases_nonresponsive_and_activates_new() {
    let mut layer = layer_with(2);
    let mut policy = DefaultActivationPolicy;
    let a = activate_on_layer(&mut layer, at(0), 0x1001);
    let b = activate_on_layer(&mut layer, at(0), 0x1002);

    // Third connection: limit reached — all routing connections get an alive
    // check.
    let c = layer.accept(at(10), &mut policy).unwrap();
    layer.handle_bytes(
        at(10),
        c,
        &ra_request(0x1003, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(a), &ALIVE_CHECK_REQUEST);
    assert_wire_send(layer.take_action_for(b), &ALIVE_CHECK_REQUEST);
    assert_eq!(layer.take_action_for(c), None);
    assert_eq!(layer.connection_state(c), Some(ConnectionState::Activating));

    // B answers in time; A stays silent.
    layer.handle_bytes(
        at(100),
        b,
        &alive_response(0x1002),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(layer.take_action(), None);

    // A's alive-check deadline (10 + 500 ms) expires: A is released and the
    // new connection activates.
    layer.poll(at(510));
    assert_eq!(layer.take_action_for(a), Some(WireAction::Close));
    assert_wire_send(layer.take_action_for(c), &ra_response(0x1003, 0x10));
    assert_eq!(layer.connection_state(a), Some(ConnectionState::Shutdown));
    assert_eq!(layer.connection_state(c), Some(ConnectionState::Active));
    // Upstream ordering: the closed notification precedes the new
    // activation so the source-address bookkeeping stays correct.
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::ConnectionClosed {
            id: a,
            source_address: 0x1001
        })
    );
    assert!(matches!(
        layer.take_event(),
        Some(ServerEvent::RoutingActive { id, .. }) if id == c
    ));
    assert_eq!(layer.routing_count(), 2);
}

#[test]
fn layer_connection_limit_all_responsive_rejects_no_free_socket() {
    let mut layer = layer_with(2);
    let mut policy = DefaultActivationPolicy;
    let a = activate_on_layer(&mut layer, at(0), 0x1001);
    let b = activate_on_layer(&mut layer, at(0), 0x1002);

    let c = layer.accept(at(10), &mut policy).unwrap();
    layer.handle_bytes(
        at(10),
        c,
        &ra_request(0x1003, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(a), &ALIVE_CHECK_REQUEST);
    assert_wire_send(layer.take_action_for(b), &ALIVE_CHECK_REQUEST);

    layer.handle_bytes(
        at(50),
        a,
        &alive_response(0x1001),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_eq!(layer.take_action(), None);
    layer.handle_bytes(
        at(60),
        b,
        &alive_response(0x1002),
        &mut policy,
        &mut NoMessageHandler,
    );

    // Everyone answered: the new connection is rejected with NoFreeSocket.
    assert_wire_send(layer.take_action_for(c), &ra_response(0x1003, 0x01));
    assert_eq!(layer.take_action_for(c), Some(WireAction::Close));
    assert_eq!(layer.connection_state(c), Some(ConnectionState::Shutdown));
    assert_eq!(layer.connection_state(a), Some(ConnectionState::Active));
    assert_eq!(layer.connection_state(b), Some(ConnectionState::Active));
    assert_eq!(layer.routing_count(), 2);
}

#[test]
fn layer_same_source_responsive_rejects_already_registered() {
    let mut layer = layer_with(5);
    let mut policy = DefaultActivationPolicy;
    let a = activate_on_layer(&mut layer, at(0), 0x1234);

    let b = layer.accept(at(10), &mut policy).unwrap();
    layer.handle_bytes(
        at(10),
        b,
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    // Only the connection holding the same source address is checked.
    assert_wire_send(layer.take_action_for(a), &ALIVE_CHECK_REQUEST);
    assert_eq!(layer.take_action_for(b), None);

    layer.handle_bytes(
        at(100),
        a,
        &alive_response(0x1234),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(b), &ra_response(0x1234, 0x03));
    assert_eq!(layer.take_action_for(b), Some(WireAction::Close));
    assert_eq!(layer.connection_state(a), Some(ConnectionState::Active));
    assert_eq!(layer.connection_state(b), Some(ConnectionState::Shutdown));
}

#[test]
fn layer_same_source_nonresponsive_lets_new_connection_take_over() {
    let mut layer = layer_with(5);
    let mut policy = DefaultActivationPolicy;
    let a = activate_on_layer(&mut layer, at(0), 0x1234);

    let b = layer.accept(at(10), &mut policy).unwrap();
    layer.handle_bytes(
        at(10),
        b,
        &ra_request(0x1234, 0x00),
        &mut policy,
        &mut NoMessageHandler,
    );
    assert_wire_send(layer.take_action_for(a), &ALIVE_CHECK_REQUEST);

    layer.poll(at(510));
    assert_eq!(layer.take_action_for(a), Some(WireAction::Close));
    assert_wire_send(layer.take_action_for(b), &ra_response(0x1234, 0x10));
    assert_eq!(layer.connection_state(b), Some(ConnectionState::Active));
    // The old connection's closed notification precedes the takeover.
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::ConnectionClosed {
            id: a,
            source_address: 0x1234
        })
    );
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::RoutingActive {
            id: b,
            source_address: 0x1234,
            internal_source_address: 0x1234,
        })
    );
}

#[test]
fn layer_admission_allows_max_plus_one_then_refuses() {
    let mut layer = layer_with(1);
    let mut policy = DefaultActivationPolicy;
    let first = activate_on_layer(&mut layer, at(0), 0x2001);
    assert_eq!(first.index(), 0);
    // Upstream admits one extra connection for arbitration (`count <= max`).
    assert!(layer.accept(at(1), &mut policy).is_ok());
    assert_eq!(layer.accept(at(2), &mut policy), Err(AcceptError::Refused));
}

#[test]
fn layer_admission_respects_filter_and_zero_limit() {
    let mut layer = layer_with(5);
    let mut refusing = RefusingFilter;
    assert_eq!(
        layer.accept(at(0), &mut refusing),
        Err(AcceptError::Refused)
    );

    let mut layer = layer_with(0);
    let mut policy = DefaultActivationPolicy;
    assert_eq!(layer.accept(at(0), &mut policy), Err(AcceptError::Refused));
}

#[test]
fn layer_slot_exhaustion_reports_no_free_slot() {
    let mut layer: ServerTransportLayer<2> = ServerTransportLayer::new(
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters::default(),
    );
    let mut policy = DefaultActivationPolicy;
    assert!(layer.accept(at(0), &mut policy).is_ok());
    assert!(layer.accept(at(0), &mut policy).is_ok());
    assert_eq!(
        layer.accept(at(0), &mut policy),
        Err(AcceptError::NoFreeSlot)
    );
}

#[test]
fn layer_resumed_connection_is_immediately_routing() {
    let mut layer = layer_with(5);
    let mut policy = ScriptedPolicy::new(ActivationDecision::proceed());
    let id = layer.accept_resumed(at(0), &mut policy, 0x0777).unwrap();
    assert_eq!(policy.calls, vec![(0x0777, 0x00, true)]);
    assert_eq!(layer.connection_state(id), Some(ConnectionState::Active));
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::RoutingActive {
            id,
            source_address: 0x0777,
            internal_source_address: 0x0777,
        })
    );
    assert_eq!(
        layer.find_routing_connection_by_internal_source_address(0x0777),
        Some(id)
    );
}

#[test]
fn layer_close_connection_is_deferred_to_poll_and_reports_routing_inactive() {
    let mut layer = layer_with(5);
    let id = activate_on_layer(&mut layer, at(0), 0x1234);
    assert!(layer.close_connection(0x1234, CloseMode::Abort));
    // Deferred: nothing happens until poll.
    assert_eq!(layer.take_action(), None);
    assert_eq!(layer.connection_state(id), Some(ConnectionState::Active));
    layer.poll(at(50));
    assert_eq!(layer.take_action_for(id), Some(WireAction::Abort));
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::ConnectionClosed {
            id,
            source_address: 0x1234
        })
    );
    assert_eq!(
        layer.find_routing_connection_by_internal_source_address(0x1234),
        None
    );
    // Release after drain reports the routing loss.
    layer.poll(at(51));
    assert_eq!(layer.connection_state(id), None);
    assert_eq!(
        layer.take_event(),
        Some(ServerEvent::RoutingInactive {
            internal_source_address: 0x1234
        })
    );
    // Unknown internal address: nothing to close.
    assert!(!layer.close_connection(0x9999, CloseMode::Close));
}

#[test]
fn layer_close_all_closes_every_connection() {
    let mut layer = layer_with(5);
    let a = activate_on_layer(&mut layer, at(0), 0x1001);
    let b = activate_on_layer(&mut layer, at(0), 0x1002);
    layer.close_all(CloseMode::Close);
    layer.poll(at(10));
    assert_eq!(layer.take_action_for(a), Some(WireAction::Close));
    assert_eq!(layer.take_action_for(b), Some(WireAction::Close));
    assert_eq!(layer.connection_state(a), Some(ConnectionState::Shutdown));
    assert_eq!(layer.connection_state(b), Some(ConnectionState::Shutdown));
    assert_eq!(layer.routing_count(), 0);
    assert_eq!(layer.dropped_wire_actions(), 0);
}
