//! E29 diagnostic transport, NACK mapping, send jobs, and UDS loopback.

use bsw_doip::{
    ActivationDecision, ActivationPolicy, CloseMode, DiagnosticHandlerAction,
    DiagnosticMessageHandler, DiagnosticSendError, DiagnosticSender, MessageHandler,
    NoMessageHandler, PoolDiagnosticGateway, ProtocolVersion, ServerTransportLayer,
    TransportParameters, WireAction,
};
use bsw_time::Instant;
use bsw_transport::pool::{AdmissionPolicy, AllocationRequest, ProviderError};
use bsw_transport::{TransportMessage, TransportResult};
use bsw_uds::{DiagJob, DiagRouter, DiagSession, SessionMask, TesterPresent};

const ENTITY: u16 = 0x0123;
const TESTER: u16 = 0x1234;
const BUS: u8 = 9;

fn ra_request(source: u16) -> [u8; 15] {
    let source = source.to_be_bytes();
    [
        0x02, 0xfd, 0x00, 0x05, 0, 0, 0, 7, source[0], source[1], 0, 0, 0, 0, 0,
    ]
}

fn diagnostic(source: u16, target: u16, data: &[u8]) -> Vec<u8> {
    let length = 4 + data.len();
    let mut bytes = vec![
        0x02,
        0xfd,
        0x80,
        0x01,
        0,
        0,
        0,
        length as u8,
        (source >> 8) as u8,
        source as u8,
        (target >> 8) as u8,
        target as u8,
    ];
    bytes.extend_from_slice(data);
    bytes
}

fn activate(layer: &mut ServerTransportLayer) -> bsw_doip::ConnectionId {
    let now = Instant::from_nanos(0);
    let mut policy = bsw_doip::DefaultActivationPolicy;
    let id = layer.accept(now, &mut policy).unwrap();
    assert!(layer.handle_bytes(
        now,
        id,
        &ra_request(TESTER),
        &mut policy,
        &mut NoMessageHandler,
    ));
    match layer.take_action_for(id) {
        Some(WireAction::Send(frame)) => assert_eq!(&frame.bytes()[2..4], &[0x00, 0x06]),
        other => panic!("missing routing activation response: {other:?}"),
    }
    id
}

#[derive(Clone, Copy)]
struct EntityPolicy;

impl AdmissionPolicy for EntityPolicy {
    fn admit(&self, request: &AllocationRequest<'_>) -> Result<(), ProviderError> {
        if request.source_bus != BUS {
            Err(ProviderError::NotResponsible)
        } else if request.source == u16::MAX {
            Err(ProviderError::InvalidSourceAddress)
        } else if request.target != ENTITY {
            Err(ProviderError::InvalidTargetAddress)
        } else {
            Ok(())
        }
    }
}

#[derive(Default)]
struct UdsListener {
    response: Option<TransportMessage<32>>,
    requests: usize,
}

impl bsw_doip::DiagnosticMessageListener<32> for UdsListener {
    fn message_received(&mut self, message: &TransportMessage<32>) -> TransportResult {
        self.requests += 1;
        let tester_present = TesterPresent {
            session_mask: SessionMask::ALL,
        };
        let jobs: [&dyn DiagJob; 1] = [&tester_present];
        let router = DiagRouter::new(&jobs);
        let mut response_bytes = [0; 32];
        let Ok(length) =
            router.dispatch(message.payload(), DiagSession::Default, &mut response_bytes)
        else {
            return TransportResult::Error;
        };
        let mut response = TransportMessage::new();
        response.set_source_address(message.target_address());
        response.set_target_address(message.source_address());
        if response.append(&response_bytes[..length]).is_err() {
            return TransportResult::BufferFull;
        }
        self.response = Some(response);
        TransportResult::Ok
    }
}

#[test]
fn uds_over_doip_loopback_ack_precedes_shared_uds_response() {
    let now = Instant::from_nanos(0);
    let mut layer = ServerTransportLayer::new(
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters::default(),
    );
    let id = activate(&mut layer);
    let mut gateway =
        PoolDiagnosticGateway::<2, 32, _, _>::new(EntityPolicy, UdsListener::default());
    let mut policy = bsw_doip::DefaultActivationPolicy;
    let request = diagnostic(TESTER, ENTITY, &[0x3e, 0x00]);

    {
        let mut handler = DiagnosticMessageHandler::new(
            ProtocolVersion::Iso2012,
            BUS,
            layer.source_address(id),
            ENTITY,
            32,
            &mut gateway,
        );
        // Exercise TCP fragmentation at header, addressing, and UDS boundaries.
        for fragment in [
            &request[..3],
            &request[3..10],
            &request[10..13],
            &request[13..],
        ] {
            assert!(layer.handle_bytes(now, id, fragment, &mut policy, &mut handler));
            assert!(handler.apply_actions(&mut layer, id));
        }
        assert_eq!(handler.dropped_actions(), 0);
    }

    match layer.take_action_for(id) {
        Some(WireAction::Send(frame)) => assert_eq!(
            frame.bytes(),
            &[0x02, 0xfd, 0x80, 0x02, 0, 0, 0, 7, 0x01, 0x23, 0x12, 0x34, 0, 0x3e, 0]
        ),
        other => panic!("positive ACK must be first: {other:?}"),
    }
    assert_eq!(gateway.listener().requests, 1);
    assert_eq!(gateway.in_use(), 0);

    let response = gateway.listener_mut().response.take().unwrap();
    let mut sender = DiagnosticSender::<6, 32>::new(ProtocolVersion::Iso2012);
    let token = sender.submit(&mut layer, &response).unwrap();
    match layer.take_action_for(id) {
        Some(WireAction::SendDiagnostic(queued)) => assert_eq!(queued, token),
        other => panic!("missing diagnostic response job: {other:?}"),
    }
    let wire = sender.wire_frame(token).unwrap();
    let mut output = [0; 32];
    let length = wire.encode_into(&mut output).unwrap();
    assert_eq!(
        &output[..length],
        &[0x02, 0xfd, 0x80, 0x01, 0, 0, 0, 6, 0x01, 0x23, 0x12, 0x34, 0x7e, 0]
    );
    sender.complete(token).unwrap();
    assert_eq!(
        sender.wire_frame(token).err(),
        Some(DiagnosticSendError::InvalidToken)
    );
}

#[derive(Clone, Copy)]
struct RejectPolicy(ProviderError);

impl AdmissionPolicy for RejectPolicy {
    fn admit(&self, _: &AllocationRequest<'_>) -> Result<(), ProviderError> {
        Err(self.0)
    }
}

fn nack_for(error: ProviderError) -> (u8, bool) {
    let mut gateway =
        PoolDiagnosticGateway::<1, 8, _, _>::new(RejectPolicy(error), |_: &TransportMessage<8>| {
            TransportResult::Ok
        });
    let mut handler = DiagnosticMessageHandler::new(
        ProtocolVersion::Iso2012,
        BUS,
        Some(TESTER),
        ENTITY,
        12,
        &mut gateway,
    );
    assert!(handler.header_received(0x8001, 5));
    handler.payload_chunk(&[0x12, 0x34, 0x01, 0x23, 0x3e]);
    let code = match handler.take_action() {
        Some(DiagnosticHandlerAction::Send(frame)) => {
            assert_eq!(&frame.bytes()[2..4], &[0x80, 0x03]);
            frame.bytes()[12]
        }
        other => panic!("missing diagnostic NACK: {other:?}"),
    };
    let close = matches!(
        handler.take_action(),
        Some(DiagnosticHandlerAction::Close(CloseMode::Close))
    );
    (code, close)
}

#[test]
fn provider_errors_follow_upstream_diagnostic_nack_table() {
    assert_eq!(nack_for(ProviderError::InvalidSourceAddress), (0x02, true));
    assert_eq!(nack_for(ProviderError::InvalidTargetAddress), (0x03, false));
    assert_eq!(nack_for(ProviderError::SizeTooLarge), (0x04, false));
    assert_eq!(nack_for(ProviderError::NoMessageAvailable), (0x05, false));
    assert_eq!(nack_for(ProviderError::NotResponsible), (0x07, false));
    assert_eq!(nack_for(ProviderError::InvalidHandle), (0x07, false));
}

#[test]
fn invalid_lengths_addresses_listener_errors_and_peer_acks_are_bounded() {
    let mut gateway =
        PoolDiagnosticGateway::<1, 4, _, _>::new(EntityPolicy, |_: &TransportMessage<4>| {
            TransportResult::Error
        });
    let mut handler = DiagnosticMessageHandler::new(
        ProtocolVersion::Iso2012,
        BUS,
        Some(TESTER),
        ENTITY,
        8,
        &mut gateway,
    );

    assert!(handler.header_received(0x8001, 3));
    assert!(matches!(
        handler.take_action(),
        Some(DiagnosticHandlerAction::Send(_))
    ));
    assert_eq!(
        handler.take_action(),
        Some(DiagnosticHandlerAction::Close(CloseMode::Close))
    );
    handler.payload_chunk(&[0, 0, 0]);

    assert!(handler.header_received(0x8001, 5));
    handler.payload_chunk(&[0x12, 0x35, 0x01, 0x23, 0x3e]);
    match handler.take_action() {
        Some(DiagnosticHandlerAction::Send(frame)) => assert_eq!(frame.bytes()[12], 0x02),
        other => panic!("missing invalid-source NACK: {other:?}"),
    }
    assert_eq!(
        handler.take_action(),
        Some(DiagnosticHandlerAction::Close(CloseMode::Close))
    );

    assert!(handler.header_received(0x8001, 5));
    handler.payload_chunk(&[0x12, 0x34, 0x01, 0x23, 0x3e]);
    match handler.take_action() {
        Some(DiagnosticHandlerAction::Send(frame)) => {
            assert_eq!(&frame.bytes()[2..4], &[0x80, 0x03]);
            assert_eq!(frame.bytes()[12], 0x08);
        }
        other => panic!("listener rejection must rewrite ACK: {other:?}"),
    }
    assert!(handler.header_received(0x8002, 5));
    handler.payload_chunk(&[0x01, 0x23, 0x12, 0x34, 0]);
    assert_eq!(
        handler.take_received_ack(),
        Some(bsw_doip::ReceivedDiagnosticAck {
            positive: true,
            source_address: ENTITY,
            target_address: TESTER,
            code: 0,
        })
    );
}

#[derive(Default)]
struct InternalPolicy;

impl ActivationPolicy for InternalPolicy {
    fn check_routing_activation(
        &mut self,
        _source_address: u16,
        _activation_type: u8,
        _is_resuming: bool,
    ) -> ActivationDecision {
        ActivationDecision {
            internal_source_address: Some(0x7777),
            ..ActivationDecision::proceed()
        }
    }
}

#[test]
fn send_jobs_use_internal_route_and_enforce_capacity_and_generation() {
    let now = Instant::from_nanos(0);
    let mut layer: ServerTransportLayer = ServerTransportLayer::new(
        ProtocolVersion::Iso2012,
        ENTITY,
        TransportParameters::default(),
    );
    let mut policy = InternalPolicy;
    let id = layer.accept(now, &mut policy).unwrap();
    assert!(layer.handle_bytes(
        now,
        id,
        &ra_request(TESTER),
        &mut policy,
        &mut NoMessageHandler,
    ));
    let _ = layer.take_action_for(id);

    let mut message = TransportMessage::<8>::new();
    message.set_source_address(ENTITY);
    message.set_target_address(0x7777);
    message.append(&[0x7e, 0]).unwrap();
    let mut sender = DiagnosticSender::<1, 8>::new(ProtocolVersion::Iso2012);
    let first = sender.submit(&mut layer, &message).unwrap();
    assert_eq!(
        sender.submit(&mut layer, &message),
        Err(DiagnosticSendError::QueueFull)
    );
    assert_eq!(sender.connection(first), Ok(id));
    sender.complete(first).unwrap();
    let second = sender.submit(&mut layer, &message).unwrap();
    assert_ne!(first, second);
    assert_eq!(
        sender.complete(first),
        Err(DiagnosticSendError::InvalidToken)
    );
}
