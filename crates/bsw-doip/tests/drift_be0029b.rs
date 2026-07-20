//! Drift-derived DoIP vectors, NOT part of the pinned-oracle evidence.
//!
//! Derived from upstream drift commit (drift tip `be0029b`):
//! - `1719a648` Switch assert from std to ETL in doip server
//!
//! That commit changed `DoIpSendJobHelper::prepareHeaderBuffer` from
//! asserting on an undersized destination buffer to returning an empty
//! span, and updated `DoIpSendJobHelperTest.cpp` accordingly (7-byte
//! buffer -> empty result instead of an assert exception). The Rust port's
//! framing entry point (`Packet::encode`) already reports an undersized
//! output buffer as a recoverable `CodecError::OutputTooSmall` instead of
//! aborting, so the post-drift graceful-failure contract holds.
//!
//! The release parity baseline remains the pinned upstream commit
//! `ddbcf88`; these tests import post-drift upstream-demonstrated behavior
//! for comparison only and do not move that baseline. See
//! `docs/port/drift-vectors-2026-07-19.md`.

use bsw_doip::payload::{CodecError, Packet, Payload, VehicleIdentification};
use bsw_doip::{DoIpHeader, PayloadType, ProtocolVersion};

/// Upstream drift vector (`DoIpSendJobTest.TestPrepareHeaderBuffer`, first
/// case): preparing a header for protocol version 0x03, payload type
/// 0x8002 (diagnostic message positive ack), payload length 0x210 into a
/// sufficient buffer yields the 8 golden header bytes.
#[test]
fn drift_1719a648_header_prepare_golden_bytes_for_diagnostic_ack() {
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

/// Upstream drift vector (`DoIpSendJobTest.TestPrepareHeaderBuffer`, second
/// case): post-drift upstream returns an EMPTY buffer for a 7-byte
/// destination instead of raising the pinned baseline's assert. The Rust
/// framing equivalent reports the undersized output as a recoverable
/// `CodecError::OutputTooSmall`; no panic, no abort.
#[test]
fn drift_1719a648_undersized_header_buffer_fails_gracefully() {
    let packet = Packet {
        version: ProtocolVersion::Iso2019,
        payload: Payload::VehicleIdentification(VehicleIdentification::All),
    };
    let mut output = [0u8; 7];
    assert_eq!(packet.encode(&mut output), Err(CodecError::OutputTooSmall));
}

/// Companion boundary to the drift vector: a buffer that fits the header
/// but not the payload is also reported gracefully, and the exact-fit
/// buffer succeeds. Mirrors the post-drift upstream contract that send-job
/// buffer preparation never aborts on caller-supplied sizes.
#[test]
fn drift_1719a648_payload_overflow_fails_gracefully_and_exact_fit_encodes() {
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
