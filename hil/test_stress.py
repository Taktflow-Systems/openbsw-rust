"""Stress and stability tests for the Rust OpenBSW HIL test suite.

Exercises the UDS stack under rapid bursts, session switching, mixed traffic,
long-running stability, and bus-recovery scenarios.

~80 parametric test cases.
"""

import pytest
import random
import statistics
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    expect_any_nrc,
    SID_DIAG_SESSION_CTRL,
    SID_READ_DID,
    SID_WRITE_DID,
    SID_TESTER_PRESENT,
    SID_SECURITY_ACCESS,
    SID_ROUTINE_CTRL,
    NRC_REQUEST_OUT_OF_RANGE,
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

# ---- shared requests -------------------------------------------------------

_TP = bytes([SID_TESTER_PRESENT, 0x00])
_DEFAULT_SESSION = bytes([SID_DIAG_SESSION_CTRL, 0x01])
_EXTENDED_SESSION = bytes([SID_DIAG_SESSION_CTRL, 0x03])
_READ_VIN = bytes([SID_READ_DID, 0xF1, 0x90])
_READ_SW = bytes([SID_READ_DID, 0xF1, 0x95])
_WRITE_VIN_RUST = bytes([SID_WRITE_DID, 0xF1, 0x90, 0x52, 0x55, 0x53, 0x54])
_UNKNOWN_SID = bytes([0xAA])
_UNKNOWN_DID = bytes([SID_READ_DID, 0x00, 0x01])


# ---------------------------------------------------------------------------
# 1. Rapid TesterPresent — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("count", [5, 10, 20, 50, 5, 10, 20, 50, 5, 10])
def test_rapid_tester_present(count):
    """Send N back-to-back TesterPresent requests; at least 90% must get positive responses."""
    success = 0
    for _ in range(count):
        resp = send_uds_sf(_TP)
        if expect_positive(resp, SID_TESTER_PRESENT):
            success += 1
        time.sleep(0.05)
    rate = success / count
    assert rate >= 0.9, (
        f"TesterPresent success rate {success}/{count} ({rate:.0%}) < 90%"
    )


# ---------------------------------------------------------------------------
# 2. Alternating services — 10 tests
# ---------------------------------------------------------------------------

# Each tuple: list of (request, sid) pairs to cycle through
_ALT_SERVICE_SETS = [
    [(_TP, SID_TESTER_PRESENT), (_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_TP, SID_TESTER_PRESENT), (_EXTENDED_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL), (_EXTENDED_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_TP, SID_TESTER_PRESENT), (_UNKNOWN_SID, 0xAA)],
    [(_TP, SID_TESTER_PRESENT), (_UNKNOWN_DID, SID_READ_DID)],
    [(_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL), (_TP, SID_TESTER_PRESENT), (_EXTENDED_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_WRITE_VIN_RUST, SID_WRITE_DID), (_TP, SID_TESTER_PRESENT)],
    [(_TP, SID_TESTER_PRESENT), (_WRITE_VIN_RUST, SID_WRITE_DID), (_UNKNOWN_DID, SID_READ_DID)],
    [(_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL), (_UNKNOWN_DID, SID_READ_DID), (_TP, SID_TESTER_PRESENT)],
    [(_TP, SID_TESTER_PRESENT), (_TP, SID_TESTER_PRESENT), (_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL)],
]


@pytest.mark.parametrize("service_set", _ALT_SERVICE_SETS)
def test_alternating_services(service_set):
    """Round-robin through a set of services 3 times; each response must be non-None."""
    cycles = 3
    failures = []
    for cycle in range(cycles):
        for req, sid in service_set:
            resp = send_uds_sf(req)
            if resp is None:
                failures.append(f"cycle={cycle} SID=0x{sid:02X} timed out")
            time.sleep(0.08)
    assert not failures, "Alternating service failures:\n" + "\n".join(failures)


# ---------------------------------------------------------------------------
# 3. Session switching stress — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("switch_count", [3, 5, 10, 3, 5, 10, 3, 5, 10, 10])
def test_session_switching_stress(switch_count):
    """Rapidly alternate Default ↔ Extended sessions; all must get positive responses."""
    sessions = [_DEFAULT_SESSION, _EXTENDED_SESSION]
    failures = 0
    for i in range(switch_count):
        req = sessions[i % 2]
        resp = send_uds_sf(req)
        if not expect_positive(resp, SID_DIAG_SESSION_CTRL):
            failures += 1
        time.sleep(0.06)
    # Return to default session so subsequent tests start clean
    send_uds_sf(_DEFAULT_SESSION)
    assert failures == 0, (
        f"{failures}/{switch_count} session switches got non-positive response"
    )


# ---------------------------------------------------------------------------
# 4. WriteDID burst — 5 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("burst_count,data", [
    (3, b"AAAA"),
    (5, b"BSTT"),
    (8, b"\x01\x02\x03\x04"),
    (10, b"\xFF\xFF\xFF\xFF"),
    (5, b"RUST"),
])
def test_write_did_burst(burst_count, data):
    """Write VIN DID burst_count times rapidly; all must succeed."""
    failures = 0
    for i in range(burst_count):
        resp = send_uds_sf(
            bytes([SID_WRITE_DID, 0xF1, 0x90]) + data
        )
        if not expect_positive(resp, SID_WRITE_DID):
            failures += 1
        time.sleep(0.1)
    assert failures == 0, (
        f"{failures}/{burst_count} WriteDID burst writes failed (data={data!r})"
    )


# ---------------------------------------------------------------------------
# 5. Mixed UDS traffic — 10 tests
# ---------------------------------------------------------------------------

# Each entry: list of (request, acceptable_check_fn) pairs
def _is_any_response(resp, _sid):
    return resp is not None

def _is_positive_or_nrc(resp, sid):
    if resp is None:
        return False
    return resp[0] in (sid + 0x40, 0x7F)


_MIXED_SEQUENCES = [
    [(_READ_VIN, SID_READ_DID), (_TP, SID_TESTER_PRESENT), (_WRITE_VIN_RUST, SID_WRITE_DID)],
    [(_TP, SID_TESTER_PRESENT), (_READ_SW, SID_READ_DID), (_TP, SID_TESTER_PRESENT)],
    [(_WRITE_VIN_RUST, SID_WRITE_DID), (_READ_VIN, SID_READ_DID), (_TP, SID_TESTER_PRESENT)],
    [(_UNKNOWN_DID, SID_READ_DID), (_TP, SID_TESTER_PRESENT), (_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_TP, SID_TESTER_PRESENT)] * 3 + [(_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL)],
    [(_READ_VIN, SID_READ_DID), (_WRITE_VIN_RUST, SID_WRITE_DID), (_READ_VIN, SID_READ_DID)],
    [(_EXTENDED_SESSION, SID_DIAG_SESSION_CTRL), (_WRITE_VIN_RUST, SID_WRITE_DID), (_READ_VIN, SID_READ_DID)],
    [(_UNKNOWN_SID, 0xAA), (_TP, SID_TESTER_PRESENT), (_UNKNOWN_SID, 0xAA)],
    [(_TP, SID_TESTER_PRESENT), (_WRITE_VIN_RUST, SID_WRITE_DID), (_TP, SID_TESTER_PRESENT), (_READ_VIN, SID_READ_DID)],
    [(_DEFAULT_SESSION, SID_DIAG_SESSION_CTRL)] + [(_TP, SID_TESTER_PRESENT)] * 4,
]


@pytest.mark.parametrize("sequence", _MIXED_SEQUENCES)
def test_mixed_uds_traffic(sequence):
    """Send a mixed sequence of UDS services; every request must receive a response."""
    failures = []
    for req, sid in sequence:
        resp = send_uds_sf(req)
        if resp is None:
            failures.append(f"SID=0x{sid:02X} timed out (req={req!r})")
        time.sleep(0.1)
    assert not failures, "Mixed traffic timeouts:\n" + "\n".join(failures)


# ---------------------------------------------------------------------------
# 6. Long-running stability — 5 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("duration_s,interval_s", [
    (15, 0.15),
    (20, 0.20),
    (30, 0.30),
    (15, 0.10),
    (20, 0.15),
])
def test_long_running_stability(duration_s, interval_s):
    """Send TesterPresent at regular intervals for duration_s seconds.

    At least 90% of requests must receive positive responses.
    """
    start = time.monotonic()
    total = 0
    success = 0
    while time.monotonic() - start < duration_s:
        resp = send_uds_sf(_TP)
        total += 1
        if expect_positive(resp, SID_TESTER_PRESENT):
            success += 1
        time.sleep(interval_s)
    rate = success / total if total > 0 else 0.0
    assert rate >= 0.9, (
        f"Long-running stability: {success}/{total} ({rate:.0%}) over {duration_s}s "
        f"at {interval_s}s intervals — expected >= 90%"
    )


# ---------------------------------------------------------------------------
# 7. Max payload — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("payload", [
    bytes([SID_TESTER_PRESENT, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),   # 7 bytes
    bytes([SID_TESTER_PRESENT, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00]),   # 7 bytes suppress
    bytes([SID_DIAG_SESSION_CTRL, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00]),
    bytes([SID_DIAG_SESSION_CTRL, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00]),
    bytes([SID_WRITE_DID, 0xF1, 0x90, 0x41, 0x42, 0x43, 0x44]),        # 7 bytes
    bytes([SID_WRITE_DID, 0xF1, 0x90, 0xFF, 0xFF, 0xFF, 0xFF]),
    bytes([SID_WRITE_DID, 0xF1, 0x90, 0x00, 0x00, 0x00, 0x00]),
    bytes([SID_WRITE_DID, 0xF1, 0x90, 0x52, 0x55, 0x53, 0x54]),        # "RUST"
    bytes([SID_READ_DID, 0xF1, 0x90, 0x00, 0x00, 0x00, 0x00]),          # extra padding
    bytes([SID_TESTER_PRESENT] + [0x00] * 6),
])
def test_max_payload_sf(payload):
    """SF with 7-byte UDS payload must be handled without crashing the server."""
    assert len(payload) == 7, f"Test misconfiguration: expected 7 bytes, got {len(payload)}"
    resp = send_uds_sf(payload)
    # Any response (positive or NRC) is acceptable — timeout is the failure mode.
    assert resp is not None, (
        f"Server did not respond to max-payload SF: {payload!r}"
    )


@pytest.mark.parametrize("attempt", range(5))
def test_multiframe_response_integrity(attempt):
    """ReadDID VIN multi-frame reassembly must return consistent payload length."""
    resp = send_uds_multiframe(_READ_VIN)
    assert resp is not None, f"Multi-frame reassembly returned None (attempt {attempt})"
    # At minimum: 0x62 + 2 DID bytes + 1 data byte
    assert len(resp) >= 4, (
        f"Reassembled payload too short: {len(resp)} bytes (attempt {attempt})"
    )
    assert resp[0] == 0x62, f"Expected positive 0x62, got 0x{resp[0]:02X}"


# ---------------------------------------------------------------------------
# 8. Response timing — 10 tests
# ---------------------------------------------------------------------------

_TIMING_REQUESTS = [
    (_TP,               SID_TESTER_PRESENT,    "TesterPresent"),
    (_DEFAULT_SESSION,  SID_DIAG_SESSION_CTRL, "DefaultSession"),
    (_EXTENDED_SESSION, SID_DIAG_SESSION_CTRL, "ExtendedSession"),
    (_UNKNOWN_DID,      SID_READ_DID,          "UnknownDID"),
    (_UNKNOWN_SID,      0xAA,                  "UnknownSID"),
]


@pytest.mark.parametrize("uds_req,sid,label", _TIMING_REQUESTS)
def test_response_time_within_100ms(uds_req, sid, label):
    """Single SF request must receive a response within 100 ms."""
    flush_bus(timeout=0.1)
    t0 = time.monotonic()
    resp = send_uds_sf(uds_req)
    elapsed_ms = (time.monotonic() - t0) * 1000
    assert resp is not None, f"{label}: no response (timeout)"
    assert elapsed_ms < 100.0, (
        f"{label}: response took {elapsed_ms:.1f} ms — expected < 100 ms"
    )


@pytest.mark.parametrize("sample_count", [5, 10, 20])
def test_response_time_statistical(sample_count):
    """Collect sample_count TesterPresent round-trip times; p95 must be < 80 ms."""
    times_ms = []
    for _ in range(sample_count):
        flush_bus(timeout=0.05)
        t0 = time.monotonic()
        resp = send_uds_sf(_TP)
        elapsed_ms = (time.monotonic() - t0) * 1000
        if resp is not None:
            times_ms.append(elapsed_ms)
        time.sleep(0.05)

    assert len(times_ms) >= sample_count * 0.8, (
        f"Too many timeouts: only {len(times_ms)}/{sample_count} samples collected"
    )
    times_ms.sort()
    p95_idx = int(len(times_ms) * 0.95)
    p95_ms = times_ms[min(p95_idx, len(times_ms) - 1)]
    assert p95_ms < 80.0, (
        f"p95 response time {p95_ms:.1f} ms exceeds 80 ms threshold "
        f"(n={len(times_ms)}, min={times_ms[0]:.1f}, max={times_ms[-1]:.1f})"
    )


# ---------------------------------------------------------------------------
# 9. Bus recovery — 10 tests
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("error_scenario,label", [
    # Malformed / reserved PCI nibbles
    ("F0AABBCCDDEEFF00", "reserved PCI 0xF"),
    ("80AABBCCDDEEFF00", "reserved PCI 0x8"),
    ("40AABBCCDDEEFF00", "reserved PCI 0x4"),
    # Invalid SF length
    ("0FAABBCCDD000000", "SF length > 7"),
    ("00AABBCCDDEE0000", "SF length 0"),
    # Stray CF without prior FF
    ("21DEADBEEFCAFE00", "stray CF SN=1"),
    ("2FDEADBEEFCAFE00", "stray CF SN=15"),
    # FC overflow
    ("320000",           "FC overflow 0x32"),
    # Random noise
    ("DEADBEEFCAFEBABE", "random noise frame"),
    # Rapid duplicate frames
    ("023E00",           "duplicate TesterPresent frame"),
])
def test_bus_recovery_after_error(error_scenario, label):
    """After an error frame, the server must continue responding to valid requests.

    Each scenario injects a bad or unexpected CAN frame, then verifies the server
    still responds to TesterPresent within 500 ms.
    """
    cansend(REQUEST_ID, error_scenario)
    time.sleep(0.3)  # allow server to discard / time-out any incomplete state

    resp = send_uds_sf(_TP)
    assert expect_positive(resp, SID_TESTER_PRESENT), (
        f"Server did not recover after '{label}' (frame={error_scenario}): "
        f"TesterPresent resp={resp!r}"
    )
