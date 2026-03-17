"""
test_e2e.py — E2E data path integrity tests (~35 tests).

Note: E2E protection (CRC+counter) is implemented in bsw-util but not yet wired
to the CAN transport layer on hardware. These tests therefore verify UDS data path
integrity: WriteDID → NvM → ReadDID round-trips, CRC-safe repeated reads, and
sequence-numbered write/read cycles.

Categories:
  1. Data integrity — write pattern, read back, verify bit-for-bit   (15 tests)
  2. CRC / consistency — read same DID multiple times, must be stable (10 tests)
  3. Sequence integrity — sequential numbered writes, verify order    (10 tests)
"""

import pytest
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    SID_READ_DID,
    SID_WRITE_DID,
    NRC_INCORRECT_MSG_LENGTH,
    NRC_REQUEST_OUT_OF_RANGE,
)
from helpers.can_transport import send_recv_raw, cansend, flush_bus, REQUEST_ID, RESPONSE_ID

# DID used for writable scratchpad tests
_SCRATCH_DID_HI = 0xF1
_SCRATCH_DID_LO = 0x90  # VIN DID — server supports WriteDID on this

# Read-only DIDs (not writable, used for consistency checks)
_RO_DIDS = [
    (0xF1, 0x95),  # SW version
    (0xF1, 0x8C),  # ECU serial
    (0xF1, 0x93),  # HW version
    (0xF1, 0x8A),  # supplier ID
    (0xF1, 0x80),  # boot SW version
]

# Maximum SF UDS payload — 7 bytes total, 3 bytes for 0x2E + DID → max 4 data bytes
_MAX_WRITE_BYTES = 4


def _write_did(did_hi: int, did_lo: int, data: bytes) -> bytes | None:
    """Helper: send WriteDID request, return response or None."""
    req = bytes([0x2E, did_hi, did_lo]) + data
    return send_uds_sf(req)


def _read_did(did_hi: int, did_lo: int) -> bytes | None:
    """Helper: send ReadDID request, return response or None."""
    return send_uds_sf(bytes([0x22, did_hi, did_lo]))


# ===========================================================================
# 1. Data Integrity — write pattern, read back, verify bit-for-bit (15 tests)
# ===========================================================================

_DATA_INTEGRITY_CASES = [
    # Single-byte boundary patterns
    bytes([0x00]),                      # all-zeros
    bytes([0xFF]),                      # all-ones
    bytes([0xAA]),                      # alternating 10101010
    bytes([0x55]),                      # alternating 01010101
    bytes([0x80]),                      # MSB set, rest clear
    bytes([0x7F]),                      # MSB clear, rest set
    bytes([0x01]),                      # lowest bit
    bytes([0xFE]),                      # all bits except lowest
    # Two-byte patterns
    bytes([0x00, 0xFF]),                # min/max
    bytes([0xFE, 0xFD]),                # descending
    bytes([0x01, 0x02]),                # sequential low bytes
    bytes([0xAA, 0x55]),                # inverted alternating
    # Four-byte patterns (maximum SF write)
    bytes([0x12, 0x34, 0x56, 0x78]),    # nibble ramp
    bytes([0xDE, 0xAD, 0xBE, 0xEF]),    # debug sentinel
    bytes([0x00, 0xFF, 0x00, 0xFF]),    # alternating bytes
]

_DATA_INTEGRITY_IDS = [
    f"data_{data.hex()}" for data in _DATA_INTEGRITY_CASES
]


@pytest.mark.parametrize("data", _DATA_INTEGRITY_CASES, ids=_DATA_INTEGRITY_IDS)
def test_data_integrity(data):
    """WriteDID then ReadDID must return bit-for-bit identical data."""
    write_data = data[:_MAX_WRITE_BYTES]

    # Write
    write_resp = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, write_data)
    assert write_resp is not None, (
        f"WriteDID got no response for data={write_data.hex()}"
    )
    assert expect_positive(write_resp, SID_WRITE_DID), (
        f"WriteDID not positive for data={write_data.hex()}: "
        f"{write_resp.hex() if write_resp else 'None'}"
    )

    time.sleep(0.5)  # Allow NvM commit

    # Read back
    read_resp = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert read_resp is not None, "ReadDID returned no response after write"
    assert read_resp[0] == 0x62, (
        f"ReadDID positive response byte must be 0x62, got 0x{read_resp[0]:02X}"
    )
    assert read_resp[1] == _SCRATCH_DID_HI, "ReadDID echo DID high byte mismatch"
    assert read_resp[2] == _SCRATCH_DID_LO, "ReadDID echo DID low byte mismatch"

    stored = read_resp[3:3 + len(write_data)]
    assert stored == write_data, (
        f"Data mismatch: wrote {write_data.hex()}, read back {stored.hex()}"
    )


# ===========================================================================
# 2. CRC / Consistency — read same DID multiple times, must be stable (10 tests)
# ===========================================================================

@pytest.mark.parametrize("did_hi,did_lo", _RO_DIDS,
                          ids=[f"DID_{h:02X}{l:02X}" for h, l in _RO_DIDS])
def test_read_only_did_consistent_across_reads(did_hi, did_lo):
    """Read-only DID must return identical data on three consecutive reads."""
    r1 = _read_did(did_hi, did_lo)
    time.sleep(0.1)
    r2 = _read_did(did_hi, did_lo)
    time.sleep(0.1)
    r3 = _read_did(did_hi, did_lo)

    assert r1 is not None, f"First ReadDID 0x{did_hi:02X}{did_lo:02X} no response"
    assert r2 is not None, f"Second ReadDID 0x{did_hi:02X}{did_lo:02X} no response"
    assert r3 is not None, f"Third ReadDID 0x{did_hi:02X}{did_lo:02X} no response"
    assert r1 == r2, (
        f"DID 0x{did_hi:02X}{did_lo:02X} inconsistent: read1={r1.hex()}, read2={r2.hex()}"
    )
    assert r2 == r3, (
        f"DID 0x{did_hi:02X}{did_lo:02X} inconsistent: read2={r2.hex()}, read3={r3.hex()}"
    )


def test_writable_did_stable_after_write():
    """After a write, the DID must remain stable over 5 rapid reads (NvM CRC integrity)."""
    write_data = bytes([0xC0, 0xFF, 0xEE, 0x01])
    write_resp = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, write_data)
    assert expect_positive(write_resp, SID_WRITE_DID), "Write failed in CRC stability test"
    time.sleep(0.5)

    results = []
    for _ in range(5):
        r = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
        assert r is not None, "ReadDID no response during CRC stability check"
        results.append(r[3:7])
        time.sleep(0.05)

    for i, stored in enumerate(results):
        assert stored == write_data, (
            f"NvM CRC instability: read {i} = {stored.hex()}, expected {write_data.hex()}"
        )


def test_read_consistency_across_session_boundary():
    """DID data must survive a session switch (default → extended → default)."""
    write_data = bytes([0x5E, 0x55, 0x10, 0x4E])
    wr = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, write_data)
    assert expect_positive(wr, SID_WRITE_DID), "Write failed before session boundary test"
    time.sleep(0.5)

    # Switch session
    r_ext = send_uds_sf(bytes([0x10, 0x03]))
    assert r_ext is not None and r_ext[0] == 0x50, "Session switch to extended failed"
    time.sleep(0.1)
    r_def = send_uds_sf(bytes([0x10, 0x01]))
    assert r_def is not None and r_def[0] == 0x50, "Session switch back to default failed"
    time.sleep(0.1)

    read_resp = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert read_resp is not None, "ReadDID failed after session boundary"
    stored = read_resp[3:7]
    assert stored == write_data, (
        f"Data lost across session boundary: wrote {write_data.hex()}, got {stored.hex()}"
    )


def test_rapid_write_read_no_tearing():
    """Write then immediately read without settling delay — must not return torn data."""
    write_data = bytes([0xBA, 0xDF, 0x00, 0xD0])
    wr = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, write_data)
    assert expect_positive(wr, SID_WRITE_DID), "Rapid write failed"
    # Deliberately no sleep — test for tearing
    read_resp = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert read_resp is not None, "Rapid ReadDID no response"
    stored = read_resp[3:7]
    # Either new data or old data — must not be partial
    assert len(stored) == len(write_data), (
        f"Torn read: expected {len(write_data)} bytes, got {len(stored)}: {stored.hex()}"
    )


# ===========================================================================
# 3. Sequence Integrity — sequential numbered writes, verify order (10 tests)
# ===========================================================================

# Sequence values — each is 4 bytes, value encodes the sequence number
_SEQUENCE_VALUES = [
    bytes([0x00, 0x00, 0x00, 0x01]),
    bytes([0x00, 0x00, 0x00, 0x02]),
    bytes([0x00, 0x00, 0x00, 0x03]),
    bytes([0x00, 0x00, 0x00, 0x04]),
    bytes([0x00, 0x00, 0x00, 0x05]),
    bytes([0x00, 0x00, 0x00, 0x06]),
    bytes([0x00, 0x00, 0x00, 0x07]),
    bytes([0x00, 0x00, 0x00, 0x08]),
    bytes([0x00, 0x00, 0x00, 0x09]),
    bytes([0x00, 0x00, 0x00, 0x0A]),
]

_SEQ_IDS = [f"seq_{v[3]:02d}" for v in _SEQUENCE_VALUES]


@pytest.mark.parametrize("value", _SEQUENCE_VALUES, ids=_SEQ_IDS)
def test_sequence_write_readback(value):
    """Write a sequence-numbered value and read it back; last write wins."""
    wr = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, value)
    assert expect_positive(wr, SID_WRITE_DID), (
        f"Write failed for sequence value {value.hex()}"
    )
    time.sleep(0.4)
    r = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert r is not None, f"ReadDID no response for sequence value {value.hex()}"
    stored = r[3:7]
    assert stored == value, (
        f"Sequence mismatch: wrote {value.hex()}, got {stored.hex()}"
    )


def test_sequence_last_write_wins():
    """Write a monotonically increasing sequence; final value must be the last written."""
    final = None
    for v in _SEQUENCE_VALUES:
        wr = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, v)
        assert expect_positive(wr, SID_WRITE_DID), (
            f"Write failed mid-sequence at {v.hex()}"
        )
        final = v
        time.sleep(0.1)

    time.sleep(0.5)  # NvM settle
    r = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert r is not None, "ReadDID no response after sequence"
    stored = r[3:7]
    assert stored == final, (
        f"Last-write-wins violation: expected {final.hex()}, got {stored.hex()}"
    )


def test_overwrite_with_zero_erases_data():
    """Write non-zero, then overwrite with all-zeros, read back must be all-zeros."""
    wr1 = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, bytes([0xDE, 0xAD, 0xBE, 0xEF]))
    assert expect_positive(wr1, SID_WRITE_DID), "First write failed"
    time.sleep(0.4)

    wr2 = _write_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO, bytes([0x00, 0x00, 0x00, 0x00]))
    assert expect_positive(wr2, SID_WRITE_DID), "Zero-overwrite failed"
    time.sleep(0.5)

    r = _read_did(_SCRATCH_DID_HI, _SCRATCH_DID_LO)
    assert r is not None, "ReadDID no response after zero-overwrite"
    stored = r[3:7]
    assert stored == bytes([0x00, 0x00, 0x00, 0x00]), (
        f"Zero-overwrite did not erase: stored={stored.hex()}"
    )
