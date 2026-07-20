//! Pinned-baseline DoIP parity evidence at upstream
//! `be0029bbb79fe901048a24c2665f2ba854328734` (`be0029b`).
//!
//! Promoted 2026-07-20 from the 2026-07-19 drift tranche
//! (`drift_be0029b.rs`) as part of the governed oracle re-pin
//! `ddbcf88` -> `be0029b`. Upstream source commit at the pin:
//! - `1719a648` Switch assert from std to ETL in doip server
//!
//! At the `be0029b` baseline, `DoIpSendJobHelper::prepareHeaderBuffer`
//! returns an empty span for an undersized destination buffer instead of
//! asserting (`DoIpSendJobHelperTest.cpp`: 7-byte buffer -> empty result).
//! This is now plain baseline behavior. The Rust port's framing entry
//! point (`Packet::encode`) conforms: an undersized output buffer is
//! reported as a recoverable `CodecError::OutputTooSmall`; no panic, no
//! abort. Recorded as conforming baseline behavior in the 2026-07-20
//! re-pin section of `docs/port/doip-parity.md` (see also
//! `docs/port/repin-2026-07-20.md`).

use bsw_doip::payload::{CodecError, Packet, Payload, VehicleIdentification};
use bsw_doip::{DoIpHeader, PayloadType, ProtocolVersion};

/// Upstream baseline vector (`DoIpSendJobTest.TestPrepareHeaderBuffer`,
/// first case): preparing a header for protocol version 0x03, payload type
/// 0x8002 (diagnostic message positive ack), payload length 0x210 into a
/// sufficient buffer yields the 8 golden header bytes.
#[test]
fn baseline_1719a648_header_prepare_golden_bytes_for_diagnostic_ack() {
    let header = DoIpHeader {
        version: ProtocolVersion::Iso2019,
        payload_type: PayloadType::DiagnosticMessagePositiveAck,
        payload_length: 0x210,
    };
    assert_eq!(
        header.encode(),
        [0x03, 0xfc, 0x80, 0x02, 0x00, 0x00, 0x02, 0x10]
    );
}

/// Upstream baseline vector (`DoIpSendJobTest.TestPrepareHeaderBuffer`,
/// second case): baseline upstream returns an EMPTY buffer for a 7-byte
/// destination (graceful failure; the former `ddbcf88` baseline asserted).
/// The Rust framing equivalent conforms by reporting the undersized output
/// as a recoverable `CodecError::OutputTooSmall`; no panic, no abort.
#[test]
fn baseline_1719a648_undersized_header_buffer_fails_gracefully() {
    let packet = Packet {
        version: ProtocolVersion::Iso2019,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    };
    let mut output = [0u8; 7];
    assert_eq!(packet.encode(&mut output), Err(CodecError::OutputTooSmall));
}

/// Companion boundary to the baseline vector: a buffer that fits the
/// header but not the payload is also reported gracefully, and the
/// exact-fit buffer succeeds. Mirrors the baseline upstream contract that
/// send-job buffer preparation never aborts on caller-supplied sizes.
#[test]
fn baseline_1719a648_payload_overflow_fails_gracefully_and_exact_fit_encodes() {
    let packet = Packet {
        version: ProtocolVersion::Iso2019,
        payload: Payload::AliveCheckResponse(0x0e00),
    };
    // Header fits (8 bytes) but the 2-byte payload does not.
    let mut short = [0u8; 9];
    assert_eq!(packet.encode(&mut short), Err(CodecError::OutputTooSmall));
    // Exact fit encodes the full frame.
    let mut exact = [0u8; 10];
    assert_eq!(packet.encode(&mut exact), Ok(10));
    assert_eq!(
        exact,
        [0x03, 0xfc, 0x00, 0x08, 0x00, 0x00, 0x00, 0x02, 0x0e, 0x00]
    );
}
