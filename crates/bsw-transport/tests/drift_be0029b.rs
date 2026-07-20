//! Drift-derived transport-router vectors, NOT part of the pinned-oracle
//! evidence.
//!
//! Derived from upstream drift commit (drift tip `be0029b`):
//! - `120f5688` Add tester address translation to referenceApp TP-Router
//!
//! That commit renames the logical-address converter API to 2-byte/1-byte
//! terminology, introduces a bounded 2-byte <-> 1-byte tester-address
//! mapping table (referenceApp: `0x0EF0..0x0EFB` <-> `0x00F0..0x00FB`) in
//! integrator-owned transport configuration, and makes the simple TP-router
//! rewrite source/target addresses at 1-byte-bus boundaries
//! (`TransportRouterSimpleTest.cpp` round-trip vectors).
//!
//! The Rust port's `SimpleRouter` deliberately routes full 16-bit logical
//! addresses without a per-boundary translation table (recorded as
//! divergence D5 in `composition-drift-review-2026-07-19.md`,
//! upstream-behavior-change, re-pin-dependent). These tests therefore pin
//! the CURRENT pinned-parity router behavior (addresses pass through
//! unchanged at bus boundaries) against the upstream round-trip vectors,
//! and demonstrate that the post-drift translation-table semantics are
//! expressible as integrator configuration on the current public API.
//!
//! The release parity baseline remains the pinned upstream commit
//! `ddbcf88`; these tests import post-drift upstream-demonstrated behavior
//! for comparison only and do not move that baseline. See
//! `docs/port/drift-vectors-2026-07-19.md`.

use bsw_transport::simple_router::{
    SimpleRoute, SimpleRouteResult, SimpleRouter, TransportEndpoint,
};
use bsw_transport::{LogicalAddress, TransportMessage, TransportResult};

// Upstream unitTest bus identifiers (`executables/unitTest/configuration/
// common/include/busid/BusId.h` as added by `120f5688`).
const SELFDIAG: u8 = 1;
const ETH_0: u8 = 3;

// Upstream unitTest transport-configuration vectors
// (`TransportRouterSimpleTest.cpp`): Ethernet tester #1 is 0x0ECD in 2-byte
// addressing and 0x00F0 in 1-byte addressing; the ECU itself is 0x0006.
const TESTER_2BYTE: u16 = 0x0ECD;
const ECU_ADDRESS: u16 = 0x0006;

/// Capturing endpoint recording the addresses each delivery carried.
struct CaptureEndpoint {
    bus: u8,
    delivered: Vec<(u16, u16, Vec<u8>)>,
}

impl CaptureEndpoint {
    fn new(bus: u8) -> Self {
        Self {
            bus,
            delivered: Vec::new(),
        }
    }
}

impl TransportEndpoint<64> for CaptureEndpoint {
    fn bus_id(&self) -> u8 {
        self.bus
    }

    fn send(&mut self, message: &TransportMessage<64>) -> TransportResult {
        self.delivered.push((
            message.source_address(),
            message.target_address(),
            message.payload().to_vec(),
        ));
        TransportResult::Ok
    }
}

fn message(source: u16, target: u16, payload: &[u8]) -> TransportMessage<64> {
    let mut message = TransportMessage::new();
    message.set_source_address(source);
    message.set_target_address(target);
    message.append(payload).unwrap();
    message
}

/// Upstream drift vector (`TransportRouterSimpleTest.cpp`,
/// `requestFromETH_responseFromSelfDiag_roundTrip`): a request from the
/// 2-byte ETH_0 bus is forwarded to the 1-byte SELFDIAG bus and the reply
/// comes back to the tester on ETH_0.
///
/// Post-drift upstream rewrites addresses at the 1-byte boundary (SELFDIAG
/// sees source 0x00F0); the pinned baseline and the Rust port do not. This
/// test asserts the current pinned-parity behavior: full 16-bit addresses
/// pass through both boundaries unchanged, and the end-to-end observable
/// (the reply reaches the tester at its own 2-byte address on ETH_0) is
/// identical to the post-drift upstream round trip. The per-boundary
/// rewrite itself is upstream-behavior-change, re-pin-dependent (U04
/// divergence D5).
#[test]
fn drift_120f5688_router_boundary_round_trip_forwards_addresses_unchanged() {
    let router = SimpleRouter::new([
        // Inbound: external request from ETH_0 to the ECU goes to SELFDIAG.
        SimpleRoute {
            source_bus: Some(ETH_0),
            target: LogicalAddress::new(ECU_ADDRESS),
            destination_bus: SELFDIAG,
            broadcast: false,
        },
        // Outbound: the ECU reply to the tester goes back to ETH_0.
        SimpleRoute {
            source_bus: Some(SELFDIAG),
            target: LogicalAddress::new(TESTER_2BYTE),
            destination_bus: ETH_0,
            broadcast: false,
        },
    ]);
    let mut eth = CaptureEndpoint::new(ETH_0);
    let mut selfdiag = CaptureEndpoint::new(SELFDIAG);

    // Inbound: ETH_0 -> SELFDIAG.
    let request = message(TESTER_2BYTE, ECU_ADDRESS, &[0x22, 0xf1, 0x90]);
    let mut layers: [&mut dyn TransportEndpoint<64>; 2] = [&mut eth, &mut selfdiag];
    assert_eq!(
        router.route(ETH_0, &request, &mut layers),
        SimpleRouteResult::Routed(1)
    );
    // Pinned parity: SELFDIAG sees the unchanged 2-byte tester source
    // (post-drift upstream would deliver source 0x00F0 here).
    assert_eq!(
        selfdiag.delivered,
        [(TESTER_2BYTE, ECU_ADDRESS, vec![0x22, 0xf1, 0x90])]
    );
    assert!(eth.delivered.is_empty());

    // Outbound: SELFDIAG -> ETH_0. Without boundary translation the ECU
    // replies to the address it saw, which already is the 2-byte tester.
    let reply = message(ECU_ADDRESS, TESTER_2BYTE, &[0x62, 0xf1, 0x90, 0x01]);
    let mut layers: [&mut dyn TransportEndpoint<64>; 2] = [&mut eth, &mut selfdiag];
    assert_eq!(
        router.route(SELFDIAG, &reply, &mut layers),
        SimpleRouteResult::Routed(1)
    );
    // End-to-end observable matches the post-drift upstream round trip:
    // the tester receives the reply at its own 2-byte address on ETH_0.
    assert_eq!(
        eth.delivered,
        [(ECU_ADDRESS, TESTER_2BYTE, vec![0x62, 0xf1, 0x90, 0x01])]
    );
}

/// Upstream drift vector (`TransportRouterSimpleTest.cpp`,
/// `messageReceived_fromSelfDiag_noReplyTarget`): a SELFDIAG message with
/// no external request to reply to is rejected (`RECEIVED_ERROR`). The
/// port's declarative analog: with no declared SELFDIAG return route the
/// router reports `NoRoute` and delivers nothing.
#[test]
fn drift_120f5688_selfdiag_reply_without_declared_return_route_is_not_routed() {
    let router = SimpleRouter::new([SimpleRoute {
        source_bus: Some(ETH_0),
        target: LogicalAddress::new(ECU_ADDRESS),
        destination_bus: SELFDIAG,
        broadcast: false,
    }]);
    let mut eth = CaptureEndpoint::new(ETH_0);
    let mut selfdiag = CaptureEndpoint::new(SELFDIAG);
    let mut layers: [&mut dyn TransportEndpoint<64>; 2] = [&mut eth, &mut selfdiag];

    let stray_reply = message(ECU_ADDRESS, 0x00F0, &[0x7f, 0x22, 0x21]);
    assert_eq!(
        router.route(SELFDIAG, &stray_reply, &mut layers),
        SimpleRouteResult::NoRoute
    );
    assert!(eth.delivered.is_empty());
    assert!(selfdiag.delivered.is_empty());
}

// ---------------------------------------------------------------------------
// Integrator-configuration expressibility of the post-drift translation
// ---------------------------------------------------------------------------

/// One 2-byte <-> 1-byte tester-address pair (upstream `LogicalAddress`
/// struct, post-drift field names `address2Byte`/`address1Byte`).
struct TesterAddressPair {
    two_byte: LogicalAddress,
    one_byte: LogicalAddress,
}

const fn pair(two_byte: u16, one_byte: u16) -> TesterAddressPair {
    TesterAddressPair {
        two_byte: LogicalAddress::new(two_byte),
        one_byte: LogicalAddress::new(one_byte),
    }
}

/// The post-drift referenceApp mapping table
/// (`executables/referenceApp/transportConfiguration`, `120f5688`):
/// twelve bounded pairs `0x0EF0..0x0EFB` <-> `0x00F0..0x00FB`.
const TESTER_ADDRESS_RANGE: [TesterAddressPair; 12] = [
    pair(0x0EF0, 0x00F0),
    pair(0x0EF1, 0x00F1),
    pair(0x0EF2, 0x00F2),
    pair(0x0EF3, 0x00F3),
    pair(0x0EF4, 0x00F4),
    pair(0x0EF5, 0x00F5),
    pair(0x0EF6, 0x00F6),
    pair(0x0EF7, 0x00F7),
    pair(0x0EF8, 0x00F8),
    pair(0x0EF9, 0x00F9),
    pair(0x0EFA, 0x00FA),
    pair(0x0EFB, 0x00FB),
];

/// Upstream `TransportConfiguration::convert2ByteAddressTo1Byte`: addresses
/// outside the `0x0E__` prefix pass through; mapped addresses translate;
/// prefix-matching but unmapped addresses pass through unchanged.
fn convert_2byte_to_1byte(address: LogicalAddress) -> LogicalAddress {
    if (address.as_u16() & 0xFF00) != 0x0E00 {
        return address;
    }
    TESTER_ADDRESS_RANGE
        .iter()
        .find(|entry| entry.two_byte == address)
        .map_or(address, |entry| entry.one_byte)
}

/// Upstream `TransportConfiguration::convert1ByteAddressTo2Byte`: the
/// symmetric direction with the `0x00F_` prefix guard.
fn convert_1byte_to_2byte(address: LogicalAddress) -> LogicalAddress {
    if (address.as_u16() & 0xFFF0) != 0x00F0 {
        return address;
    }
    TESTER_ADDRESS_RANGE
        .iter()
        .find(|entry| entry.one_byte == address)
        .map_or(address, |entry| entry.two_byte)
}

/// Upstream drift vectors (`TesterAddressTest.cpp` and the referenceApp
/// table of `120f5688`): the bounded 2-byte <-> 1-byte translation is
/// expressible as integrator configuration on the current public
/// `LogicalAddress` API without any production change.
#[test]
fn drift_120f5688_tester_address_translation_is_expressible_as_integrator_config() {
    // All twelve pairs round-trip in both directions, and every 1-byte side
    // actually fits one byte (`as_u8` is total on the 1-byte column).
    for entry in &TESTER_ADDRESS_RANGE {
        assert_eq!(convert_2byte_to_1byte(entry.two_byte), entry.one_byte);
        assert_eq!(convert_1byte_to_2byte(entry.one_byte), entry.two_byte);
        assert_eq!(
            convert_1byte_to_2byte(convert_2byte_to_1byte(entry.two_byte)),
            entry.two_byte
        );
        assert!(entry.one_byte.as_u8().is_some());
        assert!(entry.two_byte.as_u8().is_none());
    }

    // ECU addresses do not match the prefix guards and pass through
    // (upstream round-trip vector: 0x0006 -> 0x0006 in both directions).
    let ecu = LogicalAddress::new(ECU_ADDRESS);
    assert_eq!(convert_2byte_to_1byte(ecu), ecu);
    assert_eq!(convert_1byte_to_2byte(ecu), ecu);

    // The functional address 0x00DF does not match the 0x00F_ guard and
    // passes through unchanged, so the functional-address check still fires
    // after conversion (upstream `getTransportMessage_fromCAN_functionalAddress`).
    let functional = LogicalAddress::new(0x00DF);
    assert_eq!(convert_1byte_to_2byte(functional), functional);

    // The post-drift set is bounded, not a range: 0x0EE0 was inside the
    // pinned baseline's DoIP tester range (0x0EE0..0x0EFD) but has no table
    // entry and now passes through unchanged.
    let outside_bounded_set = LogicalAddress::new(0x0EE0);
    assert_eq!(
        convert_2byte_to_1byte(outside_bounded_set),
        outside_bounded_set
    );

    // Symmetrically, 0x00FC matches the 0x00F_ prefix guard but has no
    // table entry (the pinned 1-byte range reached 0x00FD).
    let unmapped_one_byte = LogicalAddress::new(0x00FC);
    assert_eq!(convert_1byte_to_2byte(unmapped_one_byte), unmapped_one_byte);

    // Bounded-set membership replaces the pinned range checks: 0x0EF5 is a
    // tester in both models, 0x0EE5 only under the pinned ranges.
    let in_both = LogicalAddress::new(0x0EF5);
    let only_pinned_range = LogicalAddress::new(0x0EE5);
    assert!(TESTER_ADDRESS_RANGE
        .iter()
        .any(|entry| entry.two_byte == in_both));
    assert!(!TESTER_ADDRESS_RANGE
        .iter()
        .any(|entry| entry.two_byte == only_pinned_range));
}
