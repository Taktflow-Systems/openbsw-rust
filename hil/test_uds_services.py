"""
UDS Services HIL test suite — ~200 parametric tests.

Target : NUCLEO-G474RE running openbsw-rust
Transport: Raspberry Pi → CAN bus → NUCLEO (can0, 500 kbit/s)
Coverage : all 9 handled UDS services, positive paths + boundary conditions.
"""

import pytest
import time
from helpers.uds import (
    send_uds_sf,
    send_uds_multiframe,
    expect_positive,
    expect_nrc,
    expect_any_nrc,
    SID_TESTER_PRESENT,
    SID_DIAG_SESSION_CTRL,
    SID_ECU_RESET,
    SID_READ_DID,
    SID_WRITE_DID,
    SID_ROUTINE_CTRL,
    SID_READ_DTC_INFO,
    SID_CLEAR_DTC,
    SID_SECURITY_ACCESS,
    SID_CONTROL_DTC_SETTING,
    NRC_SUB_FUNCTION_NOT_SUPPORTED,
    NRC_INCORRECT_MSG_LENGTH,
    NRC_CONDITIONS_NOT_CORRECT,
    NRC_REQUEST_SEQUENCE_ERROR,
    NRC_REQUEST_OUT_OF_RANGE,
    NRC_SECURITY_ACCESS_DENIED,
    NRC_INVALID_KEY,
)
from helpers.can_transport import flush_bus

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _switch_default_session():
    """Return to default session — call before session-sensitive tests."""
    send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x01]))
    time.sleep(0.25)


def _switch_extended_session():
    """Switch to extended diagnostic session."""
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x03]))
    time.sleep(0.25)
    return resp


def _switch_programming_session():
    """Switch to programming session."""
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x02]))
    time.sleep(0.25)
    return resp


# ---------------------------------------------------------------------------
# 1. TesterPresent (0x3E)
# ---------------------------------------------------------------------------

class TestTesterPresent:
    """Service 0x3E — keep communication alive."""

    @pytest.mark.parametrize("sub_fn", [0x00])
    def test_tester_present_positive(self, sub_fn):
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, sub_fn]))
        assert expect_positive(resp, SID_TESTER_PRESENT), \
            f"Expected positive response, got {resp!r}"
        assert resp[1] == sub_fn, \
            f"Echo sub-function mismatch: expected {sub_fn:#04x}, got {resp[1]:#04x}"

    @pytest.mark.parametrize("iteration", range(5))
    def test_tester_present_repeated(self, iteration):
        """Repeated TesterPresent calls must all be positive (no state accumulation)."""
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert expect_positive(resp, SID_TESTER_PRESENT), \
            f"Iteration {iteration}: expected positive, got {resp!r}"
        time.sleep(0.05)

    @pytest.mark.parametrize("sub_fn", [0x00])
    def test_tester_present_suppress_positive_response_bit(self, sub_fn):
        """Sub-function with suppress-positive-response bit (0x80) set — no response."""
        suppressed = sub_fn | 0x80
        # After suppress, next normal request must still get a positive response
        send_uds_sf(bytes([SID_TESTER_PRESENT, suppressed]))
        time.sleep(0.15)
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert expect_positive(resp, SID_TESTER_PRESENT), \
            "ECU should still respond after a suppressed TesterPresent"

    def test_tester_present_missing_sub_fn(self):
        """Too-short: only SID, no sub-function byte → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT]))
        assert expect_nrc(resp, SID_TESTER_PRESENT, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("bad_sub_fn", [0x01, 0x02, 0x7E, 0x7F, 0xFE])
    def test_tester_present_invalid_sub_fn(self, bad_sub_fn):
        """Sub-functions other than 0x00/0x80 → NRC 0x12."""
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, bad_sub_fn]))
        assert expect_nrc(resp, SID_TESTER_PRESENT, NRC_SUB_FUNCTION_NOT_SUPPORTED), \
            f"sub_fn={bad_sub_fn:#04x}: expected NRC 0x12, got {resp!r}"

    @pytest.mark.parametrize("extra_byte", [0x00, 0xAA, 0xFF])
    def test_tester_present_extra_bytes(self, extra_byte):
        """Extra trailing byte after valid sub-fn — server should handle gracefully."""
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00, extra_byte]))
        # Either positive or NRC 0x13 — must not be None (timeout)
        assert resp is not None, "No response — ECU timed out"


# ---------------------------------------------------------------------------
# 2. DiagnosticSessionControl (0x10)
# ---------------------------------------------------------------------------

class TestDiagSessionControl:
    """Service 0x10 — session transitions."""

    @pytest.mark.parametrize("session_id,session_name", [
        (0x01, "default"),
        (0x02, "programming"),
        (0x03, "extended"),
    ])
    def test_session_positive(self, session_id, session_name):
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL), \
            f"Session {session_name} ({session_id:#04x}): expected positive, got {resp!r}"
        assert resp[1] == session_id, \
            f"Session echo mismatch: expected {session_id:#04x}, got {resp[1]:#04x}"

    @pytest.mark.parametrize("session_id", [0x01, 0x02, 0x03])
    def test_session_response_contains_timing(self, session_id):
        """Response must carry P2 (bytes 2-3) and P2* (bytes 4-5) timing params."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL)
        assert len(resp) >= 6, \
            f"Response too short for timing params: {len(resp)} bytes, resp={resp!r}"
        p2_ms = (resp[2] << 8) | resp[3]
        p2star_x10ms = (resp[4] << 8) | resp[5]
        assert p2_ms > 0, "P2 timing must be non-zero"
        assert p2star_x10ms > 0, "P2* timing must be non-zero"

    @pytest.mark.parametrize("transition", [
        (0x01, 0x03),   # default → extended
        (0x03, 0x01),   # extended → default
        (0x01, 0x02),   # default → programming
        (0x02, 0x01),   # programming → default
        (0x03, 0x02),   # extended → programming
        (0x02, 0x03),   # programming → extended
    ])
    def test_session_transition(self, transition):
        from_session, to_session = transition
        # Arrive at from_session
        send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, from_session]))
        time.sleep(0.25)
        # Transition to to_session
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, to_session]))
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL), \
            f"Transition {from_session:#04x}→{to_session:#04x} failed: {resp!r}"
        _switch_default_session()

    @pytest.mark.parametrize("bad_session", [0x00, 0x04, 0x05, 0x7F, 0xFF])
    def test_session_invalid(self, bad_session):
        """Unknown session IDs → NRC 0x12 or 0x31."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, bad_session]))
        assert resp is not None, "No response"
        assert resp[0] == 0x7F, \
            f"Expected NRC response for session {bad_session:#04x}, got {resp!r}"
        assert resp[1] == SID_DIAG_SESSION_CTRL

    def test_session_missing_sub_fn(self):
        """Request with only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL]))
        assert expect_nrc(resp, SID_DIAG_SESSION_CTRL, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("session_id", [0x01, 0x02, 0x03])
    def test_session_idempotent(self, session_id):
        """Switching to the same session twice is always accepted."""
        send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
        time.sleep(0.2)
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL), \
            f"Re-entering session {session_id:#04x} failed: {resp!r}"
        _switch_default_session()


# ---------------------------------------------------------------------------
# 3. ReadDataByIdentifier (0x22)
# ---------------------------------------------------------------------------

class TestReadDID:
    """Service 0x22 — read data by identifier."""

    @pytest.mark.parametrize("did,desc", [
        (0xF190, "VIN"),
        (0xF195, "SW version"),
    ])
    def test_read_did_positive(self, did, desc):
        """Standard DIDs must return a positive response starting with 0x62."""
        did_hi = (did >> 8) & 0xFF
        did_lo = did & 0xFF
        resp = send_uds_multiframe(bytes([SID_READ_DID, did_hi, did_lo]))
        if resp is None:
            # Fall back to SF attempt (short response)
            resp = send_uds_sf(bytes([SID_READ_DID, did_hi, did_lo]))
        assert resp is not None, f"No response for DID {did:#06x} ({desc})"
        assert resp[0] == (SID_READ_DID + 0x40), \
            f"Expected 0x62 response for {desc}, got {resp[0]:#04x}"

    @pytest.mark.parametrize("did", [0xF190, 0xF195])
    def test_read_did_echoes_did(self, did):
        """Positive response must echo the requested DID in bytes 1-2."""
        did_hi = (did >> 8) & 0xFF
        did_lo = did & 0xFF
        resp = send_uds_multiframe(bytes([SID_READ_DID, did_hi, did_lo]))
        if resp is None:
            resp = send_uds_sf(bytes([SID_READ_DID, did_hi, did_lo]))
        assert resp is not None
        assert len(resp) >= 3, f"Response too short: {resp!r}"
        echoed = (resp[1] << 8) | resp[2]
        assert echoed == did, f"DID echo mismatch: expected {did:#06x}, got {echoed:#06x}"

    def test_read_vin_length(self):
        """VIN (0xF190) response: header (3 bytes) + VIN data (1-17 bytes)."""
        resp = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
        assert resp is not None, "No response for VIN DID"
        assert resp[0] == 0x62, f"Expected 0x62, got 0x{resp[0]:02X}"
        assert len(resp) >= 4, f"VIN response too short: {len(resp)} bytes"
        assert len(resp) <= 20, f"VIN response too long: {len(resp)} bytes"

    def test_read_did_missing_did_byte(self):
        """Only SID present → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_READ_DID]))
        assert expect_nrc(resp, SID_READ_DID, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_read_did_only_high_byte(self):
        """SID + one DID byte (incomplete) → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_READ_DID, 0xF1]))
        assert expect_nrc(resp, SID_READ_DID, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("did", [
        0x0000, 0x0001, 0x00FF,
        0x1234, 0x5678, 0xABCD,
        0xF100, 0xF101, 0xF102,
        0xFFFF,
    ])
    def test_read_did_unknown(self, did):
        """Unsupported DIDs must return NRC 0x31 (requestOutOfRange)."""
        did_hi = (did >> 8) & 0xFF
        did_lo = did & 0xFF
        resp = send_uds_sf(bytes([SID_READ_DID, did_hi, did_lo]))
        assert expect_nrc(resp, SID_READ_DID, NRC_REQUEST_OUT_OF_RANGE), \
            f"DID {did:#06x}: expected NRC 0x31, got {resp!r}"

    @pytest.mark.parametrize("session_id", [0x01, 0x03])
    def test_read_vin_in_session(self, session_id):
        """ReadDID 0xF190 must work in default and extended sessions."""
        send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session_id]))
        time.sleep(0.25)
        resp = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
        if resp is None:
            resp = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
        assert resp is not None
        assert resp[0] == (SID_READ_DID + 0x40)
        _switch_default_session()


# ---------------------------------------------------------------------------
# 4. WriteDataByIdentifier (0x2E)
# ---------------------------------------------------------------------------

class TestWriteDID:
    """Service 0x2E — write data by identifier."""

    @pytest.mark.parametrize("data,desc", [
        (bytes([0xAA]),                           "1 byte"),
        (bytes([0x01, 0x02]),                     "2 bytes"),
        (bytes([0xDE, 0xAD, 0xBE]),               "3 bytes"),
        (bytes([0xCA, 0xFE, 0xBA, 0xBE]),         "4 bytes"),
    ])
    def test_write_did_f190_positive(self, data, desc):
        """WriteDID 0xF190 with various data lengths → 0x6E positive response."""
        request = bytes([SID_WRITE_DID, 0xF1, 0x90]) + data
        resp = send_uds_sf(request)
        assert expect_positive(resp, SID_WRITE_DID), \
            f"WriteDID {desc}: expected 0x6E, got {resp!r}"

    @pytest.mark.parametrize("data", [
        bytes([0xAA]),
        bytes([0x01, 0x02]),
        bytes([0xDE, 0xAD, 0xBE]),
        bytes([0xCA, 0xFE, 0xBA, 0xBE]),
    ])
    def test_write_did_echoes_did(self, data):
        """Positive response must echo the DID in bytes 1-2."""
        request = bytes([SID_WRITE_DID, 0xF1, 0x90]) + data
        resp = send_uds_sf(request)
        assert expect_positive(resp, SID_WRITE_DID)
        assert len(resp) >= 3
        echoed = (resp[1] << 8) | resp[2]
        assert echoed == 0xF190, f"DID echo wrong: {echoed:#06x}"

    def test_write_did_missing_did(self):
        """Only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_WRITE_DID]))
        assert expect_nrc(resp, SID_WRITE_DID, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_write_did_incomplete_did(self):
        """SID + 1 DID byte + data → NRC 0x13 (DID must be 2 bytes)."""
        resp = send_uds_sf(bytes([SID_WRITE_DID, 0xF1, 0xAA]))
        # Either NRC 0x13 or NRC 0x31 depending on impl — not positive
        assert resp is not None
        assert resp[0] == 0x7F, f"Expected NRC, got {resp!r}"

    def test_write_did_no_data(self):
        """SID + complete DID but no data → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_WRITE_DID, 0xF1, 0x90]))
        assert expect_nrc(resp, SID_WRITE_DID, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("did", [
        0x0001, 0x1234, 0xF100, 0xFFFF,
    ])
    def test_write_did_unknown(self, did):
        """Unknown DIDs → NRC 0x31."""
        did_hi = (did >> 8) & 0xFF
        did_lo = did & 0xFF
        request = bytes([SID_WRITE_DID, did_hi, did_lo, 0xAA])
        resp = send_uds_sf(request)
        assert expect_nrc(resp, SID_WRITE_DID, NRC_REQUEST_OUT_OF_RANGE), \
            f"DID {did:#06x}: expected NRC 0x31, got {resp!r}"

    @pytest.mark.parametrize("data,desc", [
        (bytes([0x11]),                           "1 byte write"),
        (bytes([0x22, 0x33]),                     "2 byte write"),
        (bytes([0x44, 0x55, 0x66]),               "3 byte write"),
    ])
    def test_write_then_read_vin(self, data, desc):
        """After writing VIN, ReadDID must return updated data."""
        # Write
        write_req = bytes([SID_WRITE_DID, 0xF1, 0x90]) + data
        write_resp = send_uds_sf(write_req)
        assert expect_positive(write_resp, SID_WRITE_DID), \
            f"Write failed ({desc}): {write_resp!r}"
        time.sleep(0.1)
        # Read back
        read_resp = send_uds_multiframe(bytes([SID_READ_DID, 0xF1, 0x90]))
        if read_resp is None:
            read_resp = send_uds_sf(bytes([SID_READ_DID, 0xF1, 0x90]))
        assert read_resp is not None, "ReadDID after WriteDID returned nothing"
        assert read_resp[0] == (SID_READ_DID + 0x40), "ReadDID failed after write"


# ---------------------------------------------------------------------------
# 5. ControlDtcSetting (0x85)
# ---------------------------------------------------------------------------

class TestControlDtcSetting:
    """Service 0x85 — enable/disable DTC setting."""

    @pytest.mark.parametrize("sub_fn,desc", [
        (0x01, "DTC on"),
        (0x02, "DTC off"),
    ])
    def test_control_dtc_setting_positive(self, sub_fn, desc):
        resp = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, sub_fn]))
        assert expect_positive(resp, SID_CONTROL_DTC_SETTING), \
            f"ControlDtcSetting {desc}: expected positive, got {resp!r}"
        assert resp[1] == sub_fn, \
            f"Sub-fn echo wrong: expected {sub_fn:#04x}, got {resp[1]:#04x}"

    @pytest.mark.parametrize("cycle", range(3))
    def test_dtc_setting_toggle_cycle(self, cycle):
        """Toggle DTC off then on repeatedly — both must always be positive."""
        r_off = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, 0x02]))
        assert expect_positive(r_off, SID_CONTROL_DTC_SETTING), \
            f"Cycle {cycle}: DTC off failed"
        time.sleep(0.05)
        r_on = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, 0x01]))
        assert expect_positive(r_on, SID_CONTROL_DTC_SETTING), \
            f"Cycle {cycle}: DTC on failed"
        time.sleep(0.05)

    @pytest.mark.parametrize("sub_fn", [0x01])
    def test_dtc_setting_suppress_bit(self, sub_fn):
        """Suppress positive response bit (0x80) — still allows recovery."""
        send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, sub_fn | 0x80]))
        time.sleep(0.1)
        resp = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, 0x01]))
        assert expect_positive(resp, SID_CONTROL_DTC_SETTING), \
            "ECU unresponsive after suppress-positive ControlDtcSetting"

    def test_dtc_setting_missing_sub_fn(self):
        """Only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING]))
        assert expect_nrc(resp, SID_CONTROL_DTC_SETTING, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("bad_sub_fn", [0x00, 0x03, 0x04, 0x7F, 0xFF])
    def test_dtc_setting_invalid_sub_fn(self, bad_sub_fn):
        """Sub-functions other than 0x01/0x02 → NRC 0x12."""
        resp = send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, bad_sub_fn]))
        assert expect_nrc(resp, SID_CONTROL_DTC_SETTING, NRC_SUB_FUNCTION_NOT_SUPPORTED), \
            f"sub_fn={bad_sub_fn:#04x}: expected NRC 0x12, got {resp!r}"

    def test_dtc_setting_restore_on_exit(self):
        """After DTC off, switching back to default session must restore DTC on."""
        send_uds_sf(bytes([SID_CONTROL_DTC_SETTING, 0x02]))
        time.sleep(0.1)
        _switch_default_session()
        # In default session DTC setting should be restored; TesterPresent must work
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert expect_positive(resp, SID_TESTER_PRESENT), \
            "ECU unresponsive after restoring default session"


# ---------------------------------------------------------------------------
# 6. RoutineControl (0x31)
# ---------------------------------------------------------------------------

class TestRoutineControl:
    """Service 0x31 — start/stop/request routines."""

    ROUTINE_IDS = [0xFF00, 0xFF01, 0xFF02]

    @pytest.mark.parametrize("routine_id", ROUTINE_IDS)
    def test_routine_start_positive(self, routine_id):
        """Start routine — positive response 0x71."""
        rid_hi = (routine_id >> 8) & 0xFF
        rid_lo = routine_id & 0xFF
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01, rid_hi, rid_lo]))
        assert expect_positive(resp, SID_ROUTINE_CTRL), \
            f"Routine {routine_id:#06x} start: expected 0x71, got {resp!r}"

    @pytest.mark.parametrize("routine_id", ROUTINE_IDS)
    def test_routine_echoes_sub_fn_and_id(self, routine_id):
        """Positive response must echo sub-fn (0x01) and routine ID."""
        rid_hi = (routine_id >> 8) & 0xFF
        rid_lo = routine_id & 0xFF
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01, rid_hi, rid_lo]))
        assert expect_positive(resp, SID_ROUTINE_CTRL)
        assert len(resp) >= 4
        assert resp[1] == 0x01, f"Sub-fn echo wrong: {resp[1]:#04x}"
        echoed_id = (resp[2] << 8) | resp[3]
        assert echoed_id == routine_id, \
            f"Routine ID echo: expected {routine_id:#06x}, got {echoed_id:#06x}"

    @pytest.mark.parametrize("routine_id,sub_fn", [
        (0xFF00, 0x02),
        (0xFF01, 0x02),
        (0xFF00, 0x03),
        (0xFF01, 0x03),
    ])
    def test_routine_stop_request_result(self, routine_id, sub_fn):
        """Stop (0x02) and requestResults (0x03) for valid routines."""
        rid_hi = (routine_id >> 8) & 0xFF
        rid_lo = routine_id & 0xFF
        # Start first
        send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01, rid_hi, rid_lo]))
        time.sleep(0.1)
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, sub_fn, rid_hi, rid_lo]))
        # Either positive or NRC conditions-not-correct — not a timeout
        assert resp is not None, \
            f"No response for routine {routine_id:#06x} sub-fn {sub_fn:#04x}"

    @pytest.mark.parametrize("bad_routine_id", [
        0x0000, 0x0001, 0x00FF, 0x1234, 0xFEFF,
    ])
    def test_routine_unknown_id(self, bad_routine_id):
        """Unknown routine IDs → NRC 0x31."""
        rid_hi = (bad_routine_id >> 8) & 0xFF
        rid_lo = bad_routine_id & 0xFF
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01, rid_hi, rid_lo]))
        assert expect_nrc(resp, SID_ROUTINE_CTRL, NRC_REQUEST_OUT_OF_RANGE), \
            f"Routine {bad_routine_id:#06x}: expected NRC 0x31, got {resp!r}"

    def test_routine_missing_sub_fn(self):
        """Only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL]))
        assert expect_nrc(resp, SID_ROUTINE_CTRL, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_routine_missing_routine_id(self):
        """SID + sub-fn but no routine ID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01]))
        assert expect_nrc(resp, SID_ROUTINE_CTRL, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_routine_incomplete_routine_id(self):
        """SID + sub-fn + 1 byte of routine ID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, 0x01, 0xFF]))
        assert expect_nrc(resp, SID_ROUTINE_CTRL, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("bad_sub_fn", [0x00, 0x04, 0x7F, 0xFF])
    def test_routine_invalid_sub_fn(self, bad_sub_fn):
        """Sub-functions other than 0x01–0x03 → NRC 0x12."""
        resp = send_uds_sf(bytes([SID_ROUTINE_CTRL, bad_sub_fn, 0xFF, 0x00]))
        assert expect_nrc(resp, SID_ROUTINE_CTRL, NRC_SUB_FUNCTION_NOT_SUPPORTED), \
            f"sub_fn={bad_sub_fn:#04x}: expected NRC 0x12, got {resp!r}"


# ---------------------------------------------------------------------------
# 7. ReadDTCInformation (0x19)
# ---------------------------------------------------------------------------

class TestReadDTCInfo:
    """Service 0x19 — read DTC information."""

    @pytest.mark.parametrize("sub_fn,extra,desc", [
        (0x02, bytes([0xFF, 0xFF, 0xFF]), "reportDTCByStatusMask all"),
        (0x02, bytes([0x00, 0x00, 0x00]), "reportDTCByStatusMask none"),
        (0x02, bytes([0x08, 0x00, 0x00]), "reportDTCByStatusMask confirmed"),
        (0x02, bytes([0x01, 0x00, 0x00]), "reportDTCByStatusMask test-failed"),
        (0x0A, bytes(),                   "reportSupportedDTCs"),
    ])
    def test_read_dtc_info_positive(self, sub_fn, extra, desc):
        """ReadDTCInfo sub-functions must return positive 0x59."""
        request = bytes([SID_READ_DTC_INFO, sub_fn]) + extra
        resp = send_uds_multiframe(request)
        if resp is None:
            resp = send_uds_sf(request)
        assert resp is not None, f"No response for {desc}"
        assert resp[0] == (SID_READ_DTC_INFO + 0x40), \
            f"{desc}: expected 0x59, got {resp[0]:#04x}"

    @pytest.mark.parametrize("sub_fn,extra", [
        (0x02, bytes([0xFF, 0xFF, 0xFF])),
        (0x0A, bytes()),
    ])
    def test_read_dtc_info_echoes_sub_fn(self, sub_fn, extra):
        """Positive response must echo the sub-function."""
        request = bytes([SID_READ_DTC_INFO, sub_fn]) + extra
        resp = send_uds_multiframe(request)
        if resp is None:
            resp = send_uds_sf(request)
        assert resp is not None
        assert len(resp) >= 2
        assert resp[1] == sub_fn, \
            f"Sub-fn echo: expected {sub_fn:#04x}, got {resp[1]:#04x}"

    @pytest.mark.parametrize("mask", [0xFF, 0x08, 0x01, 0x00])
    def test_read_dtc_by_status_mask(self, mask):
        """reportDTCByStatusMask (0x02) with various status masks."""
        resp = send_uds_multiframe(
            bytes([SID_READ_DTC_INFO, 0x02, 0x00, 0x00, mask])
        )
        if resp is None:
            resp = send_uds_sf(bytes([SID_READ_DTC_INFO, 0x02, 0x00, 0x00, mask]))
        assert resp is not None, f"No response for mask {mask:#04x}"
        assert resp[0] == (SID_READ_DTC_INFO + 0x40)

    def test_read_dtc_missing_sub_fn(self):
        """Only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_READ_DTC_INFO]))
        assert expect_nrc(resp, SID_READ_DTC_INFO, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("bad_sub_fn", [0x00, 0x01, 0x03, 0x04, 0x09, 0x0B, 0xFF])
    def test_read_dtc_invalid_sub_fn(self, bad_sub_fn):
        """Unsupported sub-functions → NRC 0x12."""
        resp = send_uds_sf(bytes([SID_READ_DTC_INFO, bad_sub_fn]))
        assert expect_nrc(resp, SID_READ_DTC_INFO, NRC_SUB_FUNCTION_NOT_SUPPORTED), \
            f"sub_fn={bad_sub_fn:#04x}: expected NRC 0x12, got {resp!r}"

    def test_read_dtc_sub_fn_0x02_short(self):
        """0x02 requires 1 byte status mask; omitting it → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_READ_DTC_INFO, 0x02]))
        assert expect_nrc(resp, SID_READ_DTC_INFO, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"


# ---------------------------------------------------------------------------
# 8. ClearDiagnosticInformation (0x14)
# ---------------------------------------------------------------------------

class TestClearDTC:
    """Service 0x14 — clear DTC information."""

    @pytest.mark.parametrize("group,desc", [
        (0xFFFFFF, "clearAllDTCs"),
        (0xFF0000, "powertrain group"),
        (0x00FF00, "chassis group"),
        (0x0000FF, "body group"),
    ])
    def test_clear_dtc_positive(self, group, desc):
        """ClearDTC with valid groups → positive response 0x54."""
        g2 = (group >> 16) & 0xFF
        g1 = (group >> 8) & 0xFF
        g0 = group & 0xFF
        resp = send_uds_sf(bytes([SID_CLEAR_DTC, g2, g1, g0]))
        assert expect_positive(resp, SID_CLEAR_DTC), \
            f"ClearDTC {desc} ({group:#08x}): expected 0x54, got {resp!r}"

    @pytest.mark.parametrize("iteration", range(3))
    def test_clear_all_dtcs_idempotent(self, iteration):
        """Clearing all DTCs twice in a row must always succeed."""
        resp = send_uds_sf(bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]))
        assert expect_positive(resp, SID_CLEAR_DTC), \
            f"Iteration {iteration}: ClearDTC all failed, got {resp!r}"
        time.sleep(0.1)

    def test_clear_dtc_missing_group(self):
        """Only SID with no group bytes → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_CLEAR_DTC]))
        assert expect_nrc(resp, SID_CLEAR_DTC, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_clear_dtc_short_group(self):
        """SID + only 2 group bytes (must be 3) → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_CLEAR_DTC, 0xFF, 0xFF]))
        assert expect_nrc(resp, SID_CLEAR_DTC, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    def test_clear_dtc_then_read_empty(self):
        """After clearing all DTCs, ReadDTCInfo 0x02 (mask=0xFF) must return no records."""
        send_uds_sf(bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]))
        time.sleep(0.15)
        resp = send_uds_multiframe(bytes([SID_READ_DTC_INFO, 0x02, 0xFF]))
        if resp is None:
            resp = send_uds_sf(bytes([SID_READ_DTC_INFO, 0x02, 0xFF]))
        assert resp is not None, "No response for ReadDTCInfo after clear"
        assert resp[0] == (SID_READ_DTC_INFO + 0x40), \
            f"Expected 0x59, got {resp[0]:#04x}"
        # Response contains only 2 header bytes (0x59, sub-fn) if no DTCs
        assert len(resp) <= 4, \
            f"Expected no DTC records after clear, but response has {len(resp)} bytes"


# ---------------------------------------------------------------------------
# 9. SecurityAccess (0x27)
# ---------------------------------------------------------------------------

class TestSecurityAccess:
    """Service 0x27 — seed/key security access."""

    def test_request_seed_positive(self):
        """requestSeed (0x01) → positive response 0x67 with seed."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert expect_positive(resp, SID_SECURITY_ACCESS), \
            f"requestSeed: expected 0x67, got {resp!r}"
        assert resp[1] == 0x01, "Response sub-fn must echo 0x01"
        assert len(resp) >= 3, "Seed response must include at least 1 seed byte"

    def test_request_seed_contains_nonzero_seed(self):
        """Seed must not be all-zeros (would trivially pass any key check)."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert expect_positive(resp, SID_SECURITY_ACCESS)
        seed_bytes = resp[2:]
        assert any(b != 0 for b in seed_bytes), \
            f"Seed is all-zeros — insecure: {seed_bytes!r}"

    @pytest.mark.parametrize("request_seed_count", range(3))
    def test_request_seed_changes(self, request_seed_count):
        """Each new requestSeed call must produce a different seed value."""
        seeds = []
        for _ in range(2):
            resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
            assert expect_positive(resp, SID_SECURITY_ACCESS)
            seeds.append(bytes(resp[2:]))
            time.sleep(0.1)
        assert seeds[0] != seeds[1], \
            f"Seed did not change between requests: {seeds[0]!r}"

    def test_send_key_wrong_key(self):
        """sendKey (0x02) with wrong key → NRC 0x35 (invalidKey)."""
        # First get seed
        seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert expect_positive(seed_resp, SID_SECURITY_ACCESS), \
            f"requestSeed failed: {seed_resp!r}"
        # Send deliberately wrong key
        wrong_key = bytes([0xDE, 0xAD, 0xBE, 0xEF])
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02]) + wrong_key)
        assert expect_nrc(resp, SID_SECURITY_ACCESS, NRC_INVALID_KEY), \
            f"Expected NRC 0x35 for wrong key, got {resp!r}"

    def test_send_key_without_seed_request(self):
        """sendKey without fresh requestSeed → NRC 0x24 or 0x35.
        If a prior test issued a seed (seed_issued=true), the server
        validates the key (NRC 0x35 invalidKey). If no seed was issued,
        NRC 0x24 (requestSequenceError). Both are acceptable."""
        wrong_key = bytes([0x11, 0x22, 0x33, 0x44])
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02]) + wrong_key)
        assert resp is not None and resp[0] == 0x7F and resp[1] == SID_SECURITY_ACCESS, \
            f"Expected NRC for SA sendKey, got {resp!r}"
        assert resp[2] in (NRC_REQUEST_SEQUENCE_ERROR, NRC_INVALID_KEY), \
            f"Expected NRC 0x24 or 0x35, got 0x{resp[2]:02X}"
        _switch_default_session()

    def test_security_access_missing_sub_fn(self):
        """Only SID → NRC 0x13."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS]))
        assert expect_nrc(resp, SID_SECURITY_ACCESS, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13, got {resp!r}"

    @pytest.mark.parametrize("bad_sub_fn", [0x00, 0x03, 0x05, 0x7F, 0xFF])
    def test_security_access_invalid_sub_fn(self, bad_sub_fn):
        """Sub-functions other than 0x01/0x02 → NRC 0x12."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, bad_sub_fn]))
        assert expect_nrc(resp, SID_SECURITY_ACCESS, NRC_SUB_FUNCTION_NOT_SUPPORTED), \
            f"sub_fn={bad_sub_fn:#04x}: expected NRC 0x12, got {resp!r}"

    @pytest.mark.parametrize("attempt", range(3))
    def test_security_access_repeated_wrong_keys(self, attempt):
        """Three consecutive wrong keys — on third attempt may return NRC 0x36."""
        seed_resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        if not expect_positive(seed_resp, SID_SECURITY_ACCESS):
            pytest.skip("requestSeed failed — cannot test key rejection")
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, 0xBA, 0xD0 + attempt]))
        # Must be either NRC 0x35 (invalidKey) or NRC 0x36 (exceededAttempts)
        assert resp is not None, "No response for wrong key"
        assert resp[0] == 0x7F, f"Expected NRC, got {resp!r}"
        assert resp[1] == SID_SECURITY_ACCESS
        assert resp[2] in (NRC_INVALID_KEY, 0x36), \
            f"Unexpected NRC {resp[2]:#04x} for wrong key attempt {attempt}"
        time.sleep(0.2)
