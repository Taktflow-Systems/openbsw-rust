use bsw_middleware::codegen::{generate, parse_model, SchemaErrorKind};
use bsw_middleware::generated::vehicle_control;
use bsw_middleware::{
    Header, Message, MessageError, MessageKind, MiddlewareResult, MiddlewareTransport,
};

const MODEL: &str = include_str!("../models/vehicle-control.mw");
const GOLDEN: &str = include_str!("../src/generated/vehicle_control.rs");

#[test]
fn representative_model_generation_is_deterministic_and_golden() {
    let model = parse_model(MODEL).unwrap();
    let generated = generate(&model);
    assert_eq!(generated, generate(&model));
    assert_eq!(generated, GOLDEN.replace("\r\n", "\n"));
    assert_eq!(vehicle_control::VEHICLE_CONTROL.id, 0x1200);
    assert_eq!(vehicle_control::VEHICLE_CONTROL.members.len(), 2);
    assert_eq!(vehicle_control::ROUTES[0].target_cluster, 2);
}

#[test]
fn invalid_names_ids_types_and_routes_are_precise() {
    let cases = [
        ("model bad-name", SchemaErrorKind::InvalidName),
        ("model ok\nservice S 65535", SchemaErrorKind::InvalidId),
        (
            "model ok\nservice S 1\nmethod S M 1 Missing Unit",
            SchemaErrorKind::UnknownReference,
        ),
        (
            "model ok\nservice S 1\nroute S 3 3",
            SchemaErrorKind::InvalidRoute,
        ),
        (
            "model ok\nservice S 1\nservice T 1",
            SchemaErrorKind::Duplicate,
        ),
    ];
    for (input, expected) in cases {
        assert_eq!(parse_model(input).unwrap_err().kind, expected, "{input}");
    }
}

fn header() -> Header {
    Header {
        service_id: 0x1200,
        member_id: 1,
        request_id: 7,
        instance_id: 1,
        source_cluster: 1,
        target_cluster: 2,
        address_id: 0,
        kind: MessageKind::Request,
    }
}

#[test]
fn message_inline_and_external_payloads_are_bounded() {
    let mut message: Message<8> = Message::new(header());
    message.set_payload(&[1, 2, 3]).unwrap();
    assert_eq!(message.payload(), &[1, 2, 3]);
    assert_eq!(
        message.set_payload(&[0; 9]),
        Err(MessageError::PayloadTooLarge)
    );
    assert_eq!(message.payload(), &[1, 2, 3]);
    message.set_external(16, 8, 24).unwrap();
    assert_eq!(message.external(), Some((16, 8)));
    assert_eq!(
        message.set_external(u32::MAX, 2, u32::MAX),
        Err(MessageError::ExternalPayloadOutOfRange)
    );
}

struct Simulation(usize);
struct Production(usize);

impl MiddlewareTransport<8> for Simulation {
    fn source_cluster(&self) -> u8 {
        1
    }
    fn send(&mut self, _: &Message<8>) -> MiddlewareResult {
        self.0 += 1;
        MiddlewareResult::Ok
    }
}

impl MiddlewareTransport<8> for Production {
    fn source_cluster(&self) -> u8 {
        1
    }
    fn send(&mut self, _: &Message<8>) -> MiddlewareResult {
        self.0 += 1;
        MiddlewareResult::Ok
    }
}

fn exercise(transport: &mut dyn MiddlewareTransport<8>) {
    assert_eq!(transport.source_cluster(), 1);
    assert_eq!(
        transport.send(&Message::new(header())),
        MiddlewareResult::Ok
    );
}

#[test]
fn simulation_and_production_share_exactly_one_api() {
    exercise(&mut Simulation(0));
    exercise(&mut Production(0));
}
