"""NvM persistence tests for the Rust OpenBSW HIL test suite.

Tests WriteDID / ReadDID round-trips for flash-backed NvM storage via UDS.
All writes target DID 0xF190 (VIN) unless otherwise noted.

~60 parametric test cases.
"""

import pytest
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    expect_any_nrc,
    SID_READ_DID,
    SID_WRITE_DID,
    NRC_REQUEST_OUT_OF_RANGE,
    NRC_INCORRECT_MSG_LENGTH,
    NRC_SERVICE_NOT_SUPPORTED,
)
from helpers.can_transport import (
    send_recv_raw,
    send_recv_multi,
    cansend,
    flush_bus,
    REQUEST_ID,
    RESPONSE_ID,
)
from helpers.isotp import encode_sf, is_sf, is_ff

# DID under test
DID_VIN = (0xF1, 0x90)

# Helper: build WriteDID request bytes (SF, ≤ 4 data bytes)
def _write_did_req(did_high: int, did_low: int, data: bytes) -> bytes:
    return bytes([SID_WRITE_DID, did_high, did_low]) + data


# Helper: build ReadDID request bytes
def _read_did_req(did_high: int, did_low: int) -> bytes:
    return bytes([SID_READ_DID, did_high, did_low])


# Helper: read VIN and return raw payload (multi-frame aware)
def _read_vin() -> bytes | None:
    resp = send_uds_multiframe(_read_did_req(*DID_VIN))
    return resp


# ---------------------------------------------------------------------------
# 1. Write + immediate read — 10 tests
# ---------------------------------------------------------------------------

# SF WriteDID payload is UDS header (3 bytes: 0x2E DID_H DID_L) + data.
# SF max total UDS payload = 7 bytes → max 4 data bytes for DID writes.
@pytest.mark.parametrize("data", [
    b"A",
    b"AB",
    b"ABCD",
    bytes([0x00]),
    bytes([0xFF]),
    bytes(range(4)),       # 0x00 0x01 0x02 0x03
    b"\x00\x00\x00\x00",
    b"RUST",
    b"\xDE\xAD",
    b"\xCA\xFE",
])
def test_write_read_vin_immediate(data):
    """Write up to 4 data bytes to VIN DID then read back immediately."""
    req = _write_did_req(*DID_VIN, data[:4])
    resp = send_uds_sf(req)
    assert expect_positive(resp, SID_WRITE_DID), (
        f"WriteDID failed for data={data!r}: resp={resp!r}"
    )
    time.sleep(0.5)
    read_resp = _read_vin()
    assert read_resp is not None, "ReadDID returned None after write"
    # Positive response: 0x62 DID_H DID_L <data>
    assert read_resp[0] == 0x62, (
        f"ReadDID response byte 0 is 0x{read_resp[0]:02X}, expected 0x62"
    )
    assert read_resp[1:3] == bytes(DID_VIN), (
        f"ReadDID echo DID mismatch: {read_resp[1:3]!r}"
    )
    written = data[:4]
    assert written in read_resp[3:3 + len(written) + 4], (
        f"Written data {written!r} not found in ReadDID payload {read_resp[3:]!r}"
    )


# ---------------------------------------------------------------------------
# 2. Double-write (page erase) — 5 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("first,second", [
    (b"AAAA", b"BBBB"),
    (b"\xFF\xFF\xFF\xFF", b"\x00\x00\x00\x00"),
    (b"RUST", b"BSWW"),
    (b"\x01\x02\x03\x04", b"\x04\x03\x02\x01"),
    (b"XXXX", b"YYYY"),
])
def test_double_write_reads_second_value(first, second):
    """Write value A then value B; ReadDID must return B (page erase + re-write)."""
    # First write
    resp_a = send_uds_sf(_write_did_req(*DID_VIN, first))
    assert expect_positive(resp_a, SID_WRITE_DID), (
        f"First WriteDID failed (data={first!r}): {resp_a!r}"
    )
    time.sleep(0.4)

    # Second write
    resp_b = send_uds_sf(_write_did_req(*DID_VIN, second))
    assert expect_positive(resp_b, SID_WRITE_DID), (
        f"Second WriteDID failed (data={second!r}): {resp_b!r}"
    )
    time.sleep(0.5)

    read_resp = _read_vin()
    assert read_resp is not None, "ReadDID returned None after double write"
    assert second in read_resp[3:3 + len(second) + 4], (
        f"Second value {second!r} not found in ReadDID payload {read_resp[3:]!r}"
    )


# ---------------------------------------------------------------------------
# 3. Write boundary sizes — 5 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("data", [
    b"\x41",                    # 1 byte
    b"\x41\x42",               # 2 bytes
    b"\x41\x42\x43",           # 3 bytes
    b"\x41\x42\x43\x44",       # 4 bytes (SF max)
    b"\x00",                   # 1 byte — null
])
def test_write_boundary_sizes(data):
    """WriteDID with 1-4 data bytes must succeed and read back."""
    resp = send_uds_sf(_write_did_req(*DID_VIN, data))
    assert expect_positive(resp, SID_WRITE_DID), (
        f"WriteDID boundary test failed for {len(data)}-byte data {data!r}: {resp!r}"
    )
    time.sleep(0.4)
    read_resp = _read_vin()
    assert read_resp is not None, f"ReadDID returned None after {len(data)}-byte write"
    assert read_resp[0] == 0x62, (
        f"ReadDID positive byte missing after {len(data)}-byte write"
    )


# ---------------------------------------------------------------------------
# 4. Read default/empty DID — 5 tests
# ---------------------------------------------------------------------------

# These tests do NOT write first; they probe the server's default-value behaviour.
# The server must reply positively (it has a default) or with NRC, never timeout.

@pytest.mark.parametrize("did_bytes,label", [
    ((0xF1, 0x90), "VIN"),
    ((0xF1, 0x95), "SW number"),
    ((0xF1, 0x10), "System name"),
    ((0xF1, 0x01), "Boot SW fingerprint"),
    ((0xF1, 0x8C), "ECU serial number"),
])
def test_read_default_did(did_bytes, label):
    """ReadDID for a known DID must not time out — either positive or NRC."""
    resp = send_uds_multiframe(_read_did_req(*did_bytes))
    # If server returns SF-encoded error, send_uds_multiframe still decodes it.
    # Fallback: try SF path for DIDs that may return short responses.
    if resp is None:
        resp = send_uds_sf(_read_did_req(*did_bytes))
    assert resp is not None, (
        f"No response reading default {label} DID 0x{did_bytes[0]:02X}{did_bytes[1]:02X}"
    )
    is_positive = resp[0] == 0x62
    is_negative = resp[0] == 0x7F
    assert is_positive or is_negative, (
        f"ReadDID {label}: byte 0 = 0x{resp[0]:02X} is neither 0x62 nor 0x7F"
    )


# ---------------------------------------------------------------------------
# 5. Read unknown DID → NRC 0x31 — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("did_bytes", [
    (0x00, 0x00),
    (0x00, 0x01),
    (0x12, 0x34),
    (0xAB, 0xCD),
    (0xFF, 0xFF),
    (0xDE, 0xAD),
    (0x01, 0x23),
    (0xBE, 0xEF),
    (0x99, 0x99),
    (0x44, 0x44),
])
def test_read_unknown_did_nrc(did_bytes):
    """ReadDID for an unknown DID must return NRC 0x31 (requestOutOfRange)."""
    resp = send_uds_sf(_read_did_req(*did_bytes))
    assert resp is not None, (
        f"No response for unknown DID 0x{did_bytes[0]:02X}{did_bytes[1]:02X}"
    )
    assert expect_nrc(resp, SID_READ_DID, NRC_REQUEST_OUT_OF_RANGE), (
        f"Expected NRC 0x31 for unknown DID 0x{did_bytes[0]:02X}{did_bytes[1]:02X}, "
        f"got {resp!r}"
    )


# ---------------------------------------------------------------------------
# 6. WriteDID unknown DID → NRC 0x31 — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("did_bytes", [
    (0x00, 0x00),
    (0x00, 0x01),
    (0x12, 0x34),
    (0xAB, 0xCD),
    (0xFF, 0xFF),
    (0xDE, 0xAD),
    (0x01, 0x23),
    (0xBE, 0xEF),
    (0x77, 0x77),
    (0x55, 0x55),
])
def test_write_unknown_did_nrc(did_bytes):
    """WriteDID for a non-writable / unknown DID must return NRC 0x31."""
    req = _write_did_req(did_bytes[0], did_bytes[1], b"\xAA\xBB")
    resp = send_uds_sf(req)
    assert resp is not None, (
        f"No response for unknown WriteDID 0x{did_bytes[0]:02X}{did_bytes[1]:02X}"
    )
    assert expect_nrc(resp, SID_WRITE_DID, NRC_REQUEST_OUT_OF_RANGE), (
        f"Expected NRC 0x31 for unknown WriteDID "
        f"0x{did_bytes[0]:02X}{did_bytes[1]:02X}, got {resp!r}"
    )


# ---------------------------------------------------------------------------
# 7. WriteDID too short (missing data bytes) → NRC — 5 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("truncated_req,label", [
    (bytes([SID_WRITE_DID]),                  "SID only"),
    (bytes([SID_WRITE_DID, 0xF1]),            "SID + DID_H only"),
    (bytes([SID_WRITE_DID, 0xF1, 0x90]),      "SID + DID no data"),
    (bytes([SID_WRITE_DID, 0xAB]),            "SID + partial unknown DID"),
    (bytes([SID_WRITE_DID, 0xF1, 0x95]),      "SID + read-only DID no data"),
])
def test_write_did_too_short(truncated_req, label):
    """WriteDID requests that are missing data bytes must return an NRC."""
    resp = send_uds_sf(truncated_req)
    assert resp is not None, f"No response for truncated WriteDID ({label})"
    assert expect_any_nrc(resp, SID_WRITE_DID), (
        f"Expected any NRC for truncated WriteDID ({label}), got {resp!r}"
    )


# ---------------------------------------------------------------------------
# 8. Multi-write sequence — 10 tests
# ---------------------------------------------------------------------------

_WRITE_SEQ = [
    b"AA00",
    b"BB11",
    b"CC22",
    b"DD33",
    b"EE44",
    b"FF55",
    b"\x00\x00\x00\x00",
    b"\xFF\xFF\xFF\xFF",
    b"RUST",
    b"DONE",
]


@pytest.mark.parametrize("idx,data", list(enumerate(_WRITE_SEQ)))
def test_multi_write_sequence(idx, data):
    """Write each value in sequence; each write must succeed and read back correctly.

    Tests are ordered by idx so the NvM sees a realistic erase/write workload.
    """
    resp = send_uds_sf(_write_did_req(*DID_VIN, data))
    assert expect_positive(resp, SID_WRITE_DID), (
        f"WriteDID failed at sequence index {idx} (data={data!r}): {resp!r}"
    )
    time.sleep(0.5)

    read_resp = _read_vin()
    assert read_resp is not None, (
        f"ReadDID returned None at sequence index {idx}"
    )
    assert data in read_resp[3:3 + len(data) + 4], (
        f"Sequence index {idx}: written {data!r} not found in "
        f"ReadDID payload {read_resp[3:]!r}"
    )
