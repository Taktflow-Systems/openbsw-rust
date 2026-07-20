//! Drift-derived middleware workflow tests (U04 follow-up, 2026-07-19).
//!
//! Derived from upstream drift commits 6dc89c53 (Port middleware modules,
//! inline message capacity 64 bytes), 7a3c3f3f (Integrate Foo middleware
//! service into referenceApp DemoSystem), dcfc215f (Rename middleware
//! clusters from Core0/Core1 to Cluster0/Cluster1), and 34fbf932 (Fix proxy
//! dispatch missing error), all observed at drift tip be0029b.
//!
//! These tests are NOT pinned-oracle (ddbcf88a) evidence. They assert that
//! the Rust middleware contracts already accommodate the drift-tip observable
//! behavior; see docs/port/composition-drift-review-2026-07-19.md.

use bsw_middleware::generated::vehicle_control::VEHICLE_CONTROL;
use bsw_middleware::{
    ErrorState, Header, Message, MessageError, MessageKind, MiddlewareResult, MiddlewareTransport,
    MAX_MEMBER_ID,
};

/// Drift tip uses two generated clusters named Cluster0/Cluster1 (dcfc215f);
/// the Rust contracts route by numeric cluster id, so the rename is neutral.
const CLUSTER0: u8 = 0;
const CLUSTER1: u8 = 1;

fn header(kind: MessageKind, source: u8, target: u8, member_id: u16) -> Header {
    Header {
        service_id: VEHICLE_CONTROL.id,
        member_id,
        request_id: 1,
        instance_id: 1,
        source_cluster: source,
        target_cluster: target,
        address_id: 0,
        kind,
    }
}

/// Drift commit 6dc89c53 sets the upstream inline message object to 64 bytes
/// (`MAX_MESSAGE_SIZE = 64U`; the pinned oracle documented 32 bytes). The
/// Rust `Message<INLINE>` capacity is const-generic and absorbs the tip
/// capacity without a contract change; a 64-byte inline payload must be
/// accepted and 65 bytes rejected transactionally.
#[test]
fn drift_6dc89c53_tip_inline_capacity_of_64_bytes_is_bounded_and_transactional() {
    let mut message: Message<64> =
        Message::new(header(MessageKind::Request, CLUSTER1, CLUSTER0, 1));
    message.set_payload(&[0xAB; 64]).unwrap();
    assert_eq!(message.payload(), &[0xAB; 64][..]);
    assert_eq!(
        message.set_payload(&[0; 65]),
        Err(MessageError::PayloadTooLarge)
    );
    // Failed oversize write must not corrupt the previously stored payload.
    assert_eq!(message.payload(), &[0xAB; 64][..]);
}

/// Drift tip keeps the middleware error-state and message-kind values stable
/// (types.h `ErrorState` must match `IFuture::State` 4..9; Message.h
/// `MessageType` bits 1/2/4/8/16). Commit 34fbf932 relies on this mapping to
/// propagate proxy dispatch errors. The Rust contracts must stay aligned so a
/// future re-pin does not change wire-observable values.
#[test]
fn drift_tip_error_state_and_message_kind_values_match_upstream() {
    assert_eq!(ErrorState::None as u8, 0);
    assert_eq!(ErrorState::UserDefined as u8, 4);
    assert_eq!(ErrorState::ServiceBusy as u8, 5);
    assert_eq!(ErrorState::ServiceNotFound as u8, 6);
    assert_eq!(ErrorState::Serialization as u8, 7);
    assert_eq!(ErrorState::Deserialization as u8, 8);
    assert_eq!(ErrorState::QueueFull as u8, 9);

    assert_eq!(MessageKind::Request as u8, 1);
    assert_eq!(MessageKind::FireAndForget as u8, 2);
    assert_eq!(MessageKind::Response as u8, 4);
    assert_eq!(MessageKind::Event as u8, 8);
    assert_eq!(MessageKind::Error as u8, 16);

    // Tip `MAX_METHOD_ID = 128U` is unchanged from the pin.
    assert_eq!(MAX_MEMBER_ID, 128);
}

#[derive(Debug, PartialEq, Eq)]
struct Sent {
    kind: MessageKind,
    source_cluster: u8,
    target_cluster: u8,
}

struct RecordingTransport {
    cluster: u8,
    sent: Vec<Sent>,
}

impl RecordingTransport {
    fn new(cluster: u8) -> Self {
        Self {
            cluster,
            sent: Vec::new(),
        }
    }
}

impl MiddlewareTransport<64> for RecordingTransport {
    fn source_cluster(&self) -> u8 {
        self.cluster
    }

    fn send(&mut self, message: &Message<64>) -> MiddlewareResult {
        if message.header().source_cluster != self.cluster {
            return MiddlewareResult::WrongTargetCluster;
        }
        self.sent.push(Sent {
            kind: message.header().kind,
            source_cluster: message.header().source_cluster,
            target_cluster: message.header().target_cluster,
        });
        MiddlewareResult::Ok
    }
}

/// Drift commit 7a3c3f3f wires the Foo middleware service demo into the
/// referenceApp DemoSystem: at the 10 ms cyclic rate the Cluster0 skeleton
/// broadcasts every 100 cycles (1 s) and the Cluster1 proxy issues a getter
/// request every 200 cycles (2 s); shutdown deinitializes the proxy before
/// the skeleton. The Rust port has no cyclic demo composition yet (recorded
/// as re-pin-dependent), but the shared transport contract must already
/// support the exact cadence and ordering of that workflow.
#[test]
fn drift_7a3c3f3f_foo_demo_cadence_workflow_is_expressible_on_the_shared_contract() {
    let mut provider = RecordingTransport::new(CLUSTER0);
    let mut consumer = RecordingTransport::new(CLUSTER1);

    // 1000 cycles of the 10 ms demo loop.
    for cycle in 1..=1000u32 {
        if cycle % 100 == 0 {
            let mut event = Message::new(header(MessageKind::Event, CLUSTER0, CLUSTER1, 1));
            event.set_payload(&cycle.to_be_bytes()).unwrap();
            assert_eq!(provider.send(&event), MiddlewareResult::Ok);
        }
        if cycle % 200 == 0 {
            let request = Message::new(header(MessageKind::Request, CLUSTER1, CLUSTER0, 1));
            assert_eq!(consumer.send(&request), MiddlewareResult::Ok);
        }
    }

    let events: Vec<&Sent> = provider
        .sent
        .iter()
        .filter(|sent| sent.kind == MessageKind::Event)
        .collect();
    let requests: Vec<&Sent> = consumer
        .sent
        .iter()
        .filter(|sent| sent.kind == MessageKind::Request)
        .collect();
    assert_eq!(events.len(), 10, "broadcast every 1 s over 10 s");
    assert_eq!(requests.len(), 5, "getter request every 2 s over 10 s");
    assert!(events
        .iter()
        .all(|sent| sent.source_cluster == CLUSTER0 && sent.target_cluster == CLUSTER1));
    assert!(requests
        .iter()
        .all(|sent| sent.source_cluster == CLUSTER1 && sent.target_cluster == CLUSTER0));

    // A message sourced from the wrong cluster is rejected by the connection,
    // matching the tip's per-cluster connection ownership model.
    let misrouted = Message::new(header(MessageKind::Request, CLUSTER0, CLUSTER1, 1));
    assert_eq!(
        consumer.send(&misrouted),
        MiddlewareResult::WrongTargetCluster
    );
}
