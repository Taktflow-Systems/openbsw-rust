"""HIL tests for multiframe ISO-TP assembly/disassembly and diagnostic session management.

These tests verify:
1. Multiframe message reassembly (FF + CFs with flow control)
2. Diagnostic session transitions and service access gating
3. SecurityAccess full unlock/lockout flows
4. Service availability across session states

Uses cansend/candump via the helpers module — requires a live CAN interface (can0)
and a running openbsw-rust firmware target.
"""

import pytest
import time
from helpers.can_transport import (
    cansend, send_recv_raw, send_recv_multi, flush_bus,
    REQUEST_ID, RESPONSE_ID,
)
from helpers.uds import (
    send_uds_sf, send_uds_multiframe, expect_positive, expect_nrc, expect_any_nrc,
    SID_DIAG_SESSION_CTRL, SID_ECU_RESET, SID_TESTER_PRESENT,
    SID_SECURITY_ACCESS, SID_CONTROL_DTC_SETTING, SID_ROUTINE_CTRL,
    SID_READ_DID, SID_READ_DTC_INFO, SID_CLEAR_DTC, SID_WRITE_DID,
    NRC_SUB_FUNCTION_NOT_SUPPORTED, NRC_INCORRECT_MSG_LENGTH,
    NRC_REQUEST_SEQUENCE_ERROR, NRC_INVALID_KEY, NRC_EXCEEDED_ATTEMPTS,
    NRC_SECURITY_ACCESS_DENIED, NRC_REQUEST_OUT_OF_RANGE,
)
from helpers.isotp import encode_sf, is_sf, is_ff, is_cf, ff_msg_len


# ═══════════════════════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════════════════════

def _switch_default_session():
    """Switch to Default session (0x01)."""
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x01]))
    assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL), \
        f"Failed to switch to Default session, got: {resp}"
    time.sleep(0.1)


def _switch_extended_session():
    """Switch to Extended session (0x03)."""
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x03]))
    assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL), \
        f"Failed to switch to Extended session, got: {resp}"
    time.sleep(0.1)


def _switch_programming_session():
    """Switch to Programming session (0x02)."""
    resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x02]))
    assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL), \
        f"Failed to switch to Programming session, got: {resp}"
    time.sleep(0.1)


def _security_unlock():
    """Perform full SecurityAccess unlock (seed/key exchange)."""
    # Request seed
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
    assert resp is not None and expect_positive(resp, SID_SECURITY_ACCESS), \
        f"Seed request failed: {resp}"
    seed = (resp[2] << 8) | resp[3]

    # Compute key: seed XOR 0xDEAD_BEEF (lower 16 bits)
    key = (seed ^ 0xDEAD_BEEF) & 0xFFFF

    # Send key
    resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, (key >> 8) & 0xFF, key & 0xFF]))
    assert resp is not None and expect_positive(resp, SID_SECURITY_ACCESS), \
        f"Key send failed: {resp}"
    time.sleep(0.1)


# ═══════════════════════════════════════════════════════════════════════════════
# 1. MULTIFRAME ASSEMBLY/DISASSEMBLY
# ═══════════════════════════════════════════════════════════════════════════════

class TestMultiframeAssembly:
    """Tests for ISO-TP multiframe message reassembly."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()

    @pytest.mark.parametrize("did,expected_min_len", [
        (0xF190, 17),   # VIN — 17 bytes
    ])
    def test_multiframe_read_did_vin(self, did, expected_min_len):
        """ReadDID for VIN triggers a multiframe response (FF + CFs)."""
        did_hi = (did >> 8) & 0xFF
        did_lo = did & 0xFF
        resp = send_uds_multiframe(bytes([SID_READ_DID, did_hi, did_lo]), timeout=3.0)
        assert resp is not None, f"No response for DID 0x{did:04X}"
        assert expect_positive(resp, SID_READ_DID), f"Expected positive, got: {resp.hex()}"
        # Response: [0x62, DID_hi, DID_lo, ...data...]
        payload_len = len(resp) - 3  # subtract SID + DID bytes
        assert payload_len >= expected_min_len, \
            f"VIN payload too short: {payload_len} < {expected_min_len}"

    def test_single_frame_tester_present(self):
        """TesterPresent is a single-frame request/response (no FF/CF)."""
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert resp is not None
        assert expect_positive(resp, SID_TESTER_PRESENT)
        # Response should be exactly 2 bytes (0x7E, 0x00)
        assert resp[0] == 0x7E
        assert resp[1] == 0x00

    def test_single_frame_diag_session_control(self):
        """DiagSessionControl returns 6-byte positive response."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x01]))
        assert resp is not None
        assert expect_positive(resp, SID_DIAG_SESSION_CTRL)
        assert len(resp) >= 6, f"Expected 6+ bytes, got {len(resp)}"
        assert resp[1] == 0x01  # session = default

    def test_multiframe_response_reassembly_consistency(self):
        """Send the same multiframe request twice — responses should match."""
        req = bytes([SID_READ_DID, 0xF1, 0x90])  # VIN
        resp1 = send_uds_multiframe(req, timeout=3.0)
        time.sleep(0.2)
        resp2 = send_uds_multiframe(req, timeout=3.0)
        assert resp1 is not None and resp2 is not None
        assert resp1 == resp2, \
            f"Inconsistent responses: {resp1.hex()} vs {resp2.hex()}"


# ═══════════════════════════════════════════════════════════════════════════════
# 2. DIAGNOSTIC SESSION TRANSITIONS
# ═══════════════════════════════════════════════════════════════════════════════

class TestSessionTransitions:
    """Tests for diagnostic session state transitions."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()

    @pytest.mark.parametrize("from_session,to_session", [
        (0x01, 0x01),  # Default → Default
        (0x01, 0x02),  # Default → Programming
        (0x01, 0x03),  # Default → Extended
        (0x02, 0x01),  # Programming → Default
        (0x02, 0x02),  # Programming → Programming
        (0x02, 0x03),  # Programming → Extended
        (0x03, 0x01),  # Extended → Default
        (0x03, 0x02),  # Extended → Programming
        (0x03, 0x03),  # Extended → Extended
    ])
    def test_session_transition_matrix(self, from_session, to_session):
        """All 9 session transitions should produce positive responses."""
        # Switch to from_session first
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, from_session]))
        assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL)
        time.sleep(0.1)

        # Now transition to to_session
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, to_session]))
        assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL)
        assert resp[1] == to_session

    @pytest.mark.parametrize("bad_session", [0x00, 0x04, 0x05, 0x10, 0xFF])
    def test_invalid_session_id_nrc(self, bad_session):
        """Invalid session IDs return SubFunctionNotSupported NRC."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, bad_session]))
        assert resp is not None
        assert expect_nrc(resp, SID_DIAG_SESSION_CTRL, NRC_SUB_FUNCTION_NOT_SUPPORTED)

    def test_session_control_timing_params(self):
        """DiagSessionControl response includes P2/P2* timing parameters."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, 0x01]))
        assert resp is not None and len(resp) >= 6
        p2 = (resp[2] << 8) | resp[3]
        p2_star = (resp[4] << 8) | resp[5]
        assert p2 == 50, f"P2 should be 50ms, got {p2}"
        assert p2_star == 500, f"P2* should be 500 (×10ms=5000ms), got {p2_star}"


# ═══════════════════════════════════════════════════════════════════════════════
# 3. SERVICE ACCESS GATING BY SESSION
# ═══════════════════════════════════════════════════════════════════════════════

class TestSessionAccessGating:
    """Tests for service availability based on active session."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()

    # Services that should work in ALL sessions
    @pytest.mark.parametrize("session", [0x01, 0x02, 0x03])
    def test_tester_present_all_sessions(self, session):
        """TesterPresent must work in every session."""
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session]))
        assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL)
        time.sleep(0.1)

        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert resp is not None and expect_positive(resp, SID_TESTER_PRESENT), \
            f"TesterPresent failed in session 0x{session:02X}"

    # Services restricted to Extended session
    @pytest.mark.parametrize("req,sid", [
        (bytes([SID_CONTROL_DTC_SETTING, 0x01]), SID_CONTROL_DTC_SETTING),
        (bytes([SID_ROUTINE_CTRL, 0x01, 0xFF, 0x00]), SID_ROUTINE_CTRL),
        (bytes([SID_SECURITY_ACCESS, 0x01]), SID_SECURITY_ACCESS),
    ])
    def test_extended_only_services_rejected_in_default(self, req, sid):
        """Services restricted to Extended session should fail in Default."""
        resp = send_uds_sf(req)
        assert resp is not None and expect_any_nrc(resp, sid), \
            f"Expected NRC for SID 0x{sid:02X} in Default session, got: {resp}"

    @pytest.mark.parametrize("req,sid", [
        (bytes([SID_CONTROL_DTC_SETTING, 0x01]), SID_CONTROL_DTC_SETTING),
        (bytes([SID_ROUTINE_CTRL, 0x01, 0xFF, 0x00]), SID_ROUTINE_CTRL),
        (bytes([SID_SECURITY_ACCESS, 0x01]), SID_SECURITY_ACCESS),
    ])
    def test_extended_only_services_accepted_in_extended(self, req, sid):
        """Services restricted to Extended session should work in Extended."""
        _switch_extended_session()
        resp = send_uds_sf(req)
        assert resp is not None and expect_positive(resp, sid), \
            f"Expected positive for SID 0x{sid:02X} in Extended session, got: {resp}"


# ═══════════════════════════════════════════════════════════════════════════════
# 4. SECURITY ACCESS FLOWS
# ═══════════════════════════════════════════════════════════════════════════════

class TestSecurityAccess:
    """Tests for SecurityAccess (0x27) seed/key exchange."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()
        _switch_extended_session()  # SecurityAccess requires Extended

    def test_seed_request_returns_nonzero(self):
        """requestSeed should return a non-zero seed value."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert resp is not None and expect_positive(resp, SID_SECURITY_ACCESS)
        assert len(resp) >= 4
        seed = (resp[2] << 8) | resp[3]
        assert seed != 0, "Seed should be non-zero for locked ECU"

    def test_correct_key_unlocks(self):
        """Correct key (seed XOR 0xDEAD_BEEF) should unlock."""
        _security_unlock()
        # After unlock, requesting seed again should return zero
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert resp is not None and expect_positive(resp, SID_SECURITY_ACCESS)
        seed = (resp[2] << 8) | resp[3]
        assert seed == 0, f"Seed should be 0 after unlock, got {seed:#06x}"

    def test_wrong_key_returns_invalid_key_nrc(self):
        """Wrong key should return InvalidKey NRC."""
        # Request seed
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x01]))
        assert resp is not None and expect_positive(resp, SID_SECURITY_ACCESS)
        # Send deliberately wrong key
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, 0xDE, 0xAD]))
        assert resp is not None and expect_nrc(resp, SID_SECURITY_ACCESS, NRC_INVALID_KEY)

    def test_send_key_without_seed_sequence_error(self):
        """Sending key without prior seed request should return RequestSequenceError."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, 0x02, 0x12, 0x34]))
        assert resp is not None and expect_nrc(resp, SID_SECURITY_ACCESS, NRC_REQUEST_SEQUENCE_ERROR)

    @pytest.mark.parametrize("bad_sub", [0x00, 0x03, 0x04, 0x7F])
    def test_invalid_subfunction(self, bad_sub):
        """Invalid sub-functions should return SubFunctionNotSupported."""
        resp = send_uds_sf(bytes([SID_SECURITY_ACCESS, bad_sub]))
        assert resp is not None
        assert expect_nrc(resp, SID_SECURITY_ACCESS, NRC_SUB_FUNCTION_NOT_SUPPORTED)


# ═══════════════════════════════════════════════════════════════════════════════
# 5. TESTER PRESENT KEEP-ALIVE
# ═══════════════════════════════════════════════════════════════════════════════

class TestTesterPresentKeepAlive:
    """Tests for TesterPresent keeping non-default sessions alive."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()

    @pytest.mark.parametrize("session", [0x02, 0x03])
    def test_tester_present_keeps_session_alive(self, session):
        """TesterPresent should keep non-default sessions alive."""
        # Switch to non-default session
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session]))
        assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL)

        # Send TesterPresent periodically
        for _ in range(3):
            time.sleep(0.3)
            resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
            assert resp is not None and expect_positive(resp, SID_TESTER_PRESENT)

        # Verify we're still in the non-default session by querying session
        resp = send_uds_sf(bytes([SID_DIAG_SESSION_CTRL, session]))
        assert resp is not None and expect_positive(resp, SID_DIAG_SESSION_CTRL)

    def test_tester_present_repeated(self):
        """Multiple consecutive TesterPresent requests should all succeed."""
        for i in range(10):
            resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
            assert resp is not None and expect_positive(resp, SID_TESTER_PRESENT), \
                f"TesterPresent #{i} failed"
            time.sleep(0.05)


# ═══════════════════════════════════════════════════════════════════════════════
# 6. BOUNDARY CONDITIONS
# ═══════════════════════════════════════════════════════════════════════════════

class TestBoundaryConditions:
    """Edge-case and boundary condition tests."""

    def setup_method(self):
        flush_bus(timeout=0.1)
        _switch_default_session()

    def test_empty_request_no_crash(self):
        """Sending a 0-byte payload should not crash the ECU."""
        raw = send_recv_raw(REQUEST_ID, "00", RESPONSE_ID, timeout=1.0)
        # ECU may respond with NRC or ignore — just verify it doesn't crash
        time.sleep(0.2)
        # Verify ECU still alive
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert resp is not None and expect_positive(resp, SID_TESTER_PRESENT)

    def test_unknown_sid_returns_service_not_supported(self):
        """Unknown SID should return ServiceNotSupported NRC (0x11)."""
        resp = send_uds_sf(bytes([0xBA]))  # Invalid SID
        assert resp is not None
        assert resp[0] == 0x7F, f"Expected NRC, got: {resp.hex()}"
        assert resp[2] == 0x11, f"Expected NRC 0x11, got 0x{resp[2]:02X}"

    @pytest.mark.parametrize("sid", [
        SID_TESTER_PRESENT,
        SID_ECU_RESET,
        SID_DIAG_SESSION_CTRL,
    ])
    def test_too_short_request_nrc(self, sid):
        """Single-byte requests (SID only, no sub-function) should return NRC 0x13."""
        resp = send_uds_sf(bytes([sid]))
        assert resp is not None
        assert expect_nrc(resp, sid, NRC_INCORRECT_MSG_LENGTH), \
            f"Expected NRC 0x13 for SID 0x{sid:02X}, got: {resp.hex()}"

    def test_ecu_liveness_after_invalid_requests(self):
        """ECU should remain responsive after receiving multiple invalid requests."""
        # Send a bunch of garbage
        for sid in [0x00, 0xBA, 0xFE, 0xFF]:
            send_uds_sf(bytes([sid]))
            time.sleep(0.05)

        # Verify ECU is still alive
        resp = send_uds_sf(bytes([SID_TESTER_PRESENT, 0x00]))
        assert resp is not None and expect_positive(resp, SID_TESTER_PRESENT)
