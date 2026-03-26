"""
test_dem_dtc.py — DTC lifecycle tests via UDS 0x14 / 0x19 (~35 tests).

Note: DEM is implemented but only partially wired on hardware. These tests
exercise the UDS service layer directly over CAN.

Categories:
  1. ReadDTCInformation (SID 0x19) sub-functions      (15 tests)
  2. ClearDiagnosticInformation (SID 0x14)             (10 tests)
  3. Invalid / unsupported sub-functions — NRC checks  (10 tests)
"""

import pytest
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    expect_any_nrc,
    NRC_SERVICE_NOT_SUPPORTED,
    NRC_SUB_FUNCTION_NOT_SUPPORTED,
    NRC_INCORRECT_MSG_LENGTH,
    NRC_CONDITIONS_NOT_CORRECT,
    NRC_REQUEST_OUT_OF_RANGE,
    SID_CLEAR_DTC,
    SID_READ_DTC_INFO,
    SID_TESTER_PRESENT,
)
from helpers.can_transport import send_recv_raw, cansend, flush_bus, REQUEST_ID, RESPONSE_ID

# Convenience: positive response SIDs
_RESP_READ_DTC = 0x59
_RESP_CLEAR_DTC = 0x54

# DTC group bytes used throughout
_GROUP_ALL    = bytes([0xFF, 0xFF, 0xFF])
_GROUP_POWERTRAIN = bytes([0x00, 0x00, 0x00])
_GROUP_CHASSIS    = bytes([0x40, 0x00, 0x00])
_GROUP_BODY       = bytes([0x80, 0x00, 0x00])
_GROUP_NETWORK    = bytes([0xC0, 0x00, 0x00])
_GROUP_SPECIFIC_1 = bytes([0xC0, 0x73, 0x00])
_GROUP_SPECIFIC_2 = bytes([0x00, 0x12, 0x34])


# ===========================================================================
# 1. ReadDTCInformation (SID 0x19) sub-functions (15 tests)
# ===========================================================================

# Supported sub-functions per ISO 14229-1 that our server implements (at minimum):
#   0x02 reportDTCByStatusMask
#   0x0A reportSupportedDTC
_READ_DTC_CASES = [
    # (sub_fn, extra_bytes, description)
    # — reportDTCByStatusMask (0x02) with various masks
    (0x02, bytes([0xFF]),   "statusMask_all"),
    (0x02, bytes([0x01]),   "statusMask_testFailed"),
    (0x02, bytes([0x08]),   "statusMask_confirmed"),
    (0x02, bytes([0x00]),   "statusMask_none"),
    (0x02, bytes([0x09]),   "statusMask_testFailed_confirmed"),
    (0x02, bytes([0x2F]),   "statusMask_emissions"),
    (0x02, bytes([0x80]),   "statusMask_warningIndicatorActive"),
    (0x02, bytes([0x40]),   "statusMask_testNotCompleted"),
    (0x02, bytes([0x0F]),   "statusMask_lower_nibble"),
    (0x02, bytes([0xF0]),   "statusMask_upper_nibble"),
    # — reportSupportedDTC (0x0A) — no extra bytes
    (0x0A, bytes([]),       "reportSupportedDTC"),
    # — Repeated calls must yield identical structural response (idempotency on 0x0A)
    (0x0A, bytes([]),       "reportSupportedDTC_repeat"),
    # — reportDTCByStatusMask with every mask byte produces response (not NRC)
    (0x02, bytes([0x55]),   "statusMask_alternating_01"),
    (0x02, bytes([0xAA]),   "statusMask_alternating_10"),
    (0x02, bytes([0x7F]),   "statusMask_all_except_bit7"),
]

_READ_DTC_IDS = [desc for _, _, desc in _READ_DTC_CASES]


@pytest.mark.parametrize("sub_fn,extra,_desc", _READ_DTC_CASES, ids=_READ_DTC_IDS)
def test_read_dtc_info(sub_fn, extra, _desc):
    """ReadDTCInformation must respond positively (0x59) with sub-function echo."""
    req = bytes([0x19, sub_fn]) + extra
    resp = send_uds_sf(req)
    assert resp is not None, (
        f"No response for 0x19 sub_fn=0x{sub_fn:02X} extra={extra.hex()}"
    )
    assert resp[0] == _RESP_READ_DTC, (
        f"Expected 0x{_RESP_READ_DTC:02X}, got 0x{resp[0]:02X} "
        f"(sub_fn=0x{sub_fn:02X} extra={extra.hex()})"
    )
    assert resp[1] == sub_fn, (
        f"Sub-function echo mismatch: expected 0x{sub_fn:02X}, got 0x{resp[1]:02X}"
    )


@pytest.mark.parametrize("sub_fn,extra,_desc",
                          [c for c in _READ_DTC_CASES if c[0] == 0x02],
                          ids=[d for _, _, d in _READ_DTC_CASES if _ == 0x02])
def test_read_dtc_by_status_mask_response_length(sub_fn, extra, _desc):
    """reportDTCByStatusMask response must be at least 3 bytes (0x59 + sub_fn + statusMask)."""
    req = bytes([0x19, sub_fn]) + extra
    resp = send_uds_sf(req)
    assert resp is not None, "No response"
    assert len(resp) >= 3, (
        f"Response too short: {len(resp)} bytes ({resp.hex()})"
    )


def test_read_dtc_supported_dtc_min_length():
    """reportSupportedDTC (0x0A) response must be at least 2 bytes."""
    resp = send_uds_sf(bytes([0x19, 0x0A]))
    assert resp is not None, "No response for reportSupportedDTC"
    assert resp[0] == 0x59, f"Expected 0x59, got 0x{resp[0]:02X}"
    assert len(resp) >= 2, f"Response too short: {len(resp)} bytes"


def test_read_dtc_status_mask_byte_present_in_response():
    """reportDTCByStatusMask response byte[2] must echo the DTCStatusAvailabilityMask."""
    resp = send_uds_sf(bytes([0x19, 0x02, 0xFF]))
    assert resp is not None, "No response"
    assert resp[0] == 0x59
    # Byte[2] = DTCStatusAvailabilityMask (server's supported status bits)
    assert len(resp) >= 3, "Response must include DTCStatusAvailabilityMask"
    # Mask must be non-zero (server supports at least one status bit)
    assert resp[2] != 0x00, (
        f"DTCStatusAvailabilityMask = 0x00 — server supports no DTC status bits"
    )


def test_read_dtc_idempotent():
    """Two identical reportSupportedDTC calls must return the same response."""
    r1 = send_uds_sf(bytes([0x19, 0x0A]))
    time.sleep(0.3)
    r2 = send_uds_sf(bytes([0x19, 0x0A]))
    assert r1 is not None and r2 is not None, "One of the ReadDTC calls timed out"
    assert r1 == r2, (
        f"ReadDTC not idempotent: r1={r1.hex()}, r2={r2.hex()}"
    )


# ===========================================================================
# 2. ClearDiagnosticInformation (SID 0x14) (10 tests)
# ===========================================================================

_CLEAR_DTC_CASES = [
    # (group_bytes, description)
    (_GROUP_ALL,       "clearAll_FFFFFF"),
    (_GROUP_SPECIFIC_1, "clearSpecific_C07300"),
    (_GROUP_SPECIFIC_2, "clearSpecific_001234"),
    (_GROUP_POWERTRAIN, "clearGroup_powertrain_000000"),
    (_GROUP_CHASSIS,    "clearGroup_chassis_400000"),
    (_GROUP_BODY,       "clearGroup_body_800000"),
    (_GROUP_NETWORK,    "clearGroup_network_C00000"),
]

_CLEAR_DTC_IDS = [desc for _, desc in _CLEAR_DTC_CASES]


@pytest.mark.parametrize("group,_desc", _CLEAR_DTC_CASES, ids=_CLEAR_DTC_IDS)
def test_clear_dtc(group, _desc):
    """ClearDiagnosticInformation must respond positively (0x54)."""
    req = bytes([0x14]) + group
    resp = send_uds_sf(req)
    assert resp is not None, (
        f"No response for 0x14 group={group.hex()}"
    )
    assert resp[0] == _RESP_CLEAR_DTC, (
        f"Expected 0x{_RESP_CLEAR_DTC:02X}, got 0x{resp[0]:02X} (group={group.hex()})"
    )


@pytest.mark.parametrize("group,_desc", _CLEAR_DTC_CASES, ids=_CLEAR_DTC_IDS)
def test_clear_dtc_response_length(group, _desc):
    """ClearDiagnosticInformation positive response must be exactly 1 byte (0x54)."""
    req = bytes([0x14]) + group
    resp = send_uds_sf(req)
    assert resp is not None, "No response"
    assert len(resp) == 1, (
        f"ClearDTC response must be 1 byte, got {len(resp)}: {resp.hex()}"
    )


def test_clear_all_then_read_dtc_consistent():
    """Clear all DTCs, then ReadDTCByStatusMask — must not crash and must respond."""
    # Clear all
    clear_resp = send_uds_sf(bytes([0x14, 0xFF, 0xFF, 0xFF]))
    assert clear_resp is not None and clear_resp[0] == 0x54, "ClearDTC failed"
    time.sleep(0.3)
    # Read — must still respond normally
    read_resp = send_uds_sf(bytes([0x19, 0x02, 0xFF]))
    assert read_resp is not None, "ReadDTC no response after ClearDTC"
    assert read_resp[0] == 0x59, (
        f"ReadDTC unexpected response after clear: 0x{read_resp[0]:02X}"
    )


def test_clear_dtc_twice_idempotent():
    """ClearDiagnosticInformation called twice with the same group must both succeed."""
    req = bytes([0x14, 0xFF, 0xFF, 0xFF])
    r1 = send_uds_sf(req)
    time.sleep(0.2)
    r2 = send_uds_sf(req)
    assert r1 is not None and r1[0] == 0x54, "First ClearDTC failed"
    assert r2 is not None and r2[0] == 0x54, "Second ClearDTC failed (idempotency)"


def test_ecu_alive_after_clear_dtc():
    """ECU must remain responsive after ClearDTC (no reboot / hang)."""
    _ = send_uds_sf(bytes([0x14, 0xFF, 0xFF, 0xFF]))
    time.sleep(0.3)
    tp = send_uds_sf(bytes([0x3E, 0x00]))
    assert tp is not None, "ECU unresponsive after ClearDTC"
    assert tp[0] == 0x7E, f"Unexpected TesterPresent response: 0x{tp[0]:02X}"


# ===========================================================================
# 3. Invalid / unsupported sub-functions — NRC 0x12 checks (10 tests)
# ===========================================================================

# ISO 14229-1 sub-functions that are defined in the standard but NOT implemented
# by our minimal DEM server (or are reserved):
#   0x00 — reserved
#   0x01 — reportNumberOfDTCByStatusMask (optional, not wired)
#   0x03 — reportDTCSnapshotIdentification (optional, not wired)
#   0x04 — reportDTCSnapshotRecordByDTCNumber (optional, not wired)
#   0x05 — reportDTCStoredDataRecordByRecordNumber (optional)
#   0x0B — reportFirstTestFailedDTC (optional)
#   0x0C — reportFirstConfirmedDTC (optional)
#   0x0D — reportMostRecentTestFailedDTC (optional)
#   0x0E — reportMostRecentConfirmedDTC (optional)
#   0xFF — reserved / vendor-specific

_INVALID_SUBFN_CASES = [
    (0x00, "reserved_00"),
    (0x01, "reportNumberOfDTCByStatusMask"),
    (0x03, "reportDTCSnapshotIdentification"),
    (0x04, "reportDTCSnapshotRecordByDTCNumber"),
    (0x05, "reportDTCStoredDataRecordByRecordNumber"),
    (0x0B, "reportFirstTestFailedDTC"),
    (0x0C, "reportFirstConfirmedDTC"),
    (0x0D, "reportMostRecentTestFailedDTC"),
    (0x0E, "reportMostRecentConfirmedDTC"),
    (0xFF, "reserved_FF"),
]

_INVALID_SUBFN_IDS = [desc for _, desc in _INVALID_SUBFN_CASES]


@pytest.mark.parametrize("sub_fn,_desc", _INVALID_SUBFN_CASES,
                          ids=_INVALID_SUBFN_IDS)
def test_read_dtc_invalid_subfn(sub_fn, _desc):
    """Unsupported 0x19 sub-function must return NRC 0x12 (subFunctionNotSupported)."""
    resp = send_uds_sf(bytes([0x19, sub_fn, 0xFF]))
    assert resp is not None, (
        f"No response for 0x19 sub_fn=0x{sub_fn:02X} (expected NRC)"
    )
    assert expect_nrc(resp, SID_READ_DTC_INFO, NRC_SUB_FUNCTION_NOT_SUPPORTED), (
        f"Expected NRC 0x7F 0x19 0x12, got {resp.hex()}"
    )


@pytest.mark.parametrize("sub_fn,_desc", _INVALID_SUBFN_CASES,
                          ids=_INVALID_SUBFN_IDS)
def test_read_dtc_invalid_subfn_nrc_length(sub_fn, _desc):
    """NRC response for unsupported sub-function must be exactly 3 bytes."""
    resp = send_uds_sf(bytes([0x19, sub_fn, 0xFF]))
    assert resp is not None, "No response"
    assert len(resp) == 3, (
        f"NRC response length {len(resp)} != 3: {resp.hex()}"
    )


def test_clear_dtc_too_short_returns_nrc():
    """ClearDTC with fewer than 3 group bytes must return NRC 0x13 (incorrectMessageLength)."""
    # Only 2 group bytes instead of 3
    resp = send_uds_sf(bytes([0x14, 0xFF, 0xFF]))
    assert resp is not None, "No response for short ClearDTC"
    assert resp[0] == 0x7F, f"Expected NRC (0x7F), got 0x{resp[0]:02X}"
    assert resp[1] == SID_CLEAR_DTC, f"NRC SID echo mismatch: 0x{resp[1]:02X}"
    assert resp[2] in (NRC_INCORRECT_MSG_LENGTH, NRC_REQUEST_OUT_OF_RANGE), (
        f"Expected NRC 0x13 or 0x31 for short ClearDTC, got 0x{resp[2]:02X}"
    )


def test_read_dtc_too_short_returns_nrc():
    """ReadDTCInformation with no sub-function byte must return NRC 0x13."""
    resp = send_uds_sf(bytes([0x19]))
    assert resp is not None, "No response for bare 0x19"
    assert resp[0] == 0x7F, f"Expected NRC (0x7F), got 0x{resp[0]:02X}"
    assert resp[1] == SID_READ_DTC_INFO, "NRC SID echo mismatch"
    assert resp[2] == NRC_INCORRECT_MSG_LENGTH, (
        f"Expected NRC 0x13 for missing sub-function, got 0x{resp[2]:02X}"
    )


def test_ecu_alive_after_invalid_subfn_flood():
    """ECU must survive receiving all invalid sub-functions back-to-back."""
    for sub_fn, _ in _INVALID_SUBFN_CASES:
        _ = send_uds_sf(bytes([0x19, sub_fn, 0xFF]))
        time.sleep(0.05)
    # Final liveness check
    tp = send_uds_sf(bytes([0x3E, 0x00]))
    assert tp is not None, "ECU unresponsive after invalid sub-function flood"
    assert tp[0] == 0x7E, f"Unexpected TesterPresent response: 0x{tp[0]:02X}"
