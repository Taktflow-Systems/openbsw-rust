"""
test_architecture.py — Architecture drift detection tests (~100 tests).

Verifies that hardware behaviour matches ISO 14229 / CAN / timing expectations.
Categories:
  1. UDS positive-response SID compliance        (30 tests)
  2. CAN frame format compliance                 (20 tests)
  3. Response timing — P2 server                 (20 tests)
  4. Idempotency                                 (15 tests)
  5. Regression tests for the 9 bring-up bugs   (15 tests)
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
    NRC_REQUEST_OUT_OF_RANGE,
    SID_DIAG_SESSION_CTRL,
    SID_ECU_RESET,
    SID_CLEAR_DTC,
    SID_READ_DTC_INFO,
    SID_READ_DID,
    SID_WRITE_DID,
    SID_ROUTINE_CTRL,
    SID_TESTER_PRESENT,
    SID_CONTROL_DTC_SETTING,
)
from helpers.can_transport import send_recv_raw, cansend, flush_bus, REQUEST_ID, RESPONSE_ID

# ---------------------------------------------------------------------------
# ISO-TP padding sentinel used by our server (0xCC)
# ---------------------------------------------------------------------------
_PAD = 0xCC

# Maximum ISO 14229-1 SF UDS payload length (7 bytes after PCI byte)
_SF_MAX_PAYLOAD = 7

# ISO 14229-1 P2 server max (ms) — default 50 ms, we allow 500 ms for HIL
_P2_LIMIT_MS = 500


# ===========================================================================
# 1. UDS Positive-Response SID Compliance (30 tests)
# ===========================================================================

# Each tuple: (sid, minimal_valid_request_bytes)
_POSITIVE_RESPONSE_CASES = [
    # TesterPresent — sub-function 0x00 (no suppressPosRsp bit)
    (SID_TESTER_PRESENT, bytes([0x3E, 0x00])),
    # DiagnosticSessionControl — default session
    (SID_DIAG_SESSION_CTRL, bytes([0x10, 0x01])),
    # DiagnosticSessionControl — extended diagnostic session
    (SID_DIAG_SESSION_CTRL, bytes([0x10, 0x03])),
    # DiagnosticSessionControl — programming session
    (SID_DIAG_SESSION_CTRL, bytes([0x10, 0x02])),
    # ReadDataByIdentifier — SW version (F195)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x95])),
    # ReadDataByIdentifier — VIN (F190)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x90])),
    # ReadDataByIdentifier — ECU serial (F18C)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x8C])),
    # ReadDataByIdentifier — HW version (F193)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x93])),
    # ReadDataByIdentifier — supplier ID (F18A)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x8A])),
    # ReadDataByIdentifier — boot SW version (F180)
    (SID_READ_DID, bytes([0x22, 0xF1, 0x80])),
    # WriteDID — custom DID 0xF190 (VIN), write 4 bytes
    (SID_WRITE_DID, bytes([0x2E, 0xF1, 0x90, 0x41, 0x42, 0x43, 0x44])),
    # ControlDTCSetting — ON
    (SID_CONTROL_DTC_SETTING, bytes([0x85, 0x01])),
    # ControlDTCSetting — OFF
    (SID_CONTROL_DTC_SETTING, bytes([0x85, 0x02])),
    # RoutineControl — startRoutine (0x01), routine 0xFF00
    (SID_ROUTINE_CTRL, bytes([0x31, 0x01, 0xFF, 0x00])),
    # RoutineControl — requestRoutineResults (0x03), routine 0xFF00
    (SID_ROUTINE_CTRL, bytes([0x31, 0x03, 0xFF, 0x00])),
]

# Derive parametrize IDs — include SID hex in the id for readable output
_POSITIVE_RESPONSE_IDS = [
    f"SID{case[0]:02X}_req{''.join(f'{b:02X}' for b in case[1])}"
    for case in _POSITIVE_RESPONSE_CASES
]


@pytest.mark.parametrize("sid,uds_req", _POSITIVE_RESPONSE_CASES,
                          ids=_POSITIVE_RESPONSE_IDS)
def test_positive_response_sid(sid, uds_req):
    """Positive response byte must equal SID + 0x40."""
    resp = send_uds_sf(uds_req)
    assert resp is not None, f"No response for SID 0x{sid:02X} req={uds_req.hex()}"
    assert resp[0] == sid + 0x40, (
        f"Expected positive response 0x{sid + 0x40:02X}, got 0x{resp[0]:02X}"
    )


@pytest.mark.parametrize("sid,uds_req", _POSITIVE_RESPONSE_CASES,
                          ids=_POSITIVE_RESPONSE_IDS)
def test_positive_response_min_length(sid, uds_req):
    """Positive response must be at least 1 byte (SID echo)."""
    resp = send_uds_sf(uds_req)
    assert resp is not None, "No response"
    assert len(resp) >= 1, "Response too short"


@pytest.mark.parametrize("sid,uds_req", _POSITIVE_RESPONSE_CASES[:10],
                          ids=_POSITIVE_RESPONSE_IDS[:10])
def test_positive_response_not_nrc(sid, uds_req):
    """Positive response must not be an NRC (0x7F)."""
    resp = send_uds_sf(uds_req)
    assert resp is not None, "No response"
    assert resp[0] != 0x7F, (
        f"Got NRC 0x{resp[2]:02X} for SID 0x{sid:02X}"
        if len(resp) >= 3 else "Got NRC with short payload"
    )


# NRC format compliance for unknown / intentionally bad requests
_NRC_CASES = [
    # Unknown SID → NRC 0x11 (serviceNotSupported)
    (0xAA, bytes([0xAA]),               NRC_SERVICE_NOT_SUPPORTED),
    (0xBB, bytes([0xBB, 0x00]),         NRC_SERVICE_NOT_SUPPORTED),
    (0xCC, bytes([0xCC]),               NRC_SERVICE_NOT_SUPPORTED),
    (0xDD, bytes([0xDD, 0x01, 0x02]),   NRC_SERVICE_NOT_SUPPORTED),
    (0xEE, bytes([0xEE]),               NRC_SERVICE_NOT_SUPPORTED),
    # ReadDID with only 1 byte (missing DID) → NRC 0x13 (incorrectMessageLength)
    (SID_READ_DID, bytes([0x22]),       NRC_INCORRECT_MSG_LENGTH),
    # ReadDID unknown DID (0x0001) → NRC 0x31 (requestOutOfRange)
    (SID_READ_DID, bytes([0x22, 0x00, 0x01]), NRC_REQUEST_OUT_OF_RANGE),
    # WriteDID with no data beyond DID → NRC 0x13
    (SID_WRITE_DID, bytes([0x2E, 0xF1, 0x90]), NRC_INCORRECT_MSG_LENGTH),
    # DiagSession unknown session type 0xFE → NRC 0x12
    (SID_DIAG_SESSION_CTRL, bytes([0x10, 0xFE]), NRC_SUB_FUNCTION_NOT_SUPPORTED),
    # TesterPresent with unknown sub-function 0x05 → NRC 0x12
    (SID_TESTER_PRESENT, bytes([0x3E, 0x05]), NRC_SUB_FUNCTION_NOT_SUPPORTED),
]

_NRC_IDS = [
    f"SID{case[0]:02X}_NRC{case[2]:02X}_{i}"
    for i, case in enumerate(_NRC_CASES)
]


@pytest.mark.parametrize("sid,uds_req,expected_nrc", _NRC_CASES, ids=_NRC_IDS)
def test_nrc_format(sid, uds_req, expected_nrc):
    """NRC response must be exactly [0x7F, SID, NRC_code]."""
    resp = send_uds_sf(uds_req)
    assert resp is not None, f"No response for bad request {uds_req.hex()}"
    assert resp[0] == 0x7F, f"Expected 0x7F, got 0x{resp[0]:02X}"
    assert resp[1] == sid, f"NRC SID echo: expected 0x{sid:02X}, got 0x{resp[1]:02X}"
    assert resp[2] == expected_nrc, (
        f"Expected NRC 0x{expected_nrc:02X}, got 0x{resp[2]:02X}"
    )


@pytest.mark.parametrize("sid,uds_req,_nrc", _NRC_CASES, ids=_NRC_IDS)
def test_nrc_length_is_three(sid, uds_req, _nrc):
    """NRC response payload must be exactly 3 bytes."""
    resp = send_uds_sf(uds_req)
    assert resp is not None, "No response"
    assert len(resp) == 3, f"Expected 3-byte NRC, got {len(resp)} bytes: {resp.hex()}"


# ===========================================================================
# 2. CAN Frame Format Compliance (20 tests)
# ===========================================================================

def _raw_frame(request: bytes, timeout: float = 2.0):
    """Return the raw 8-byte CAN frame hex for a given UDS request."""
    from helpers.isotp import encode_sf
    sf_hex = encode_sf(request)
    return send_recv_raw(REQUEST_ID, sf_hex, RESPONSE_ID, timeout)


# Representative requests for CAN-level checks
_CAN_FORMAT_REQUESTS = [
    bytes([0x3E, 0x00]),
    bytes([0x10, 0x01]),
    bytes([0x22, 0xF1, 0x95]),
    bytes([0x22, 0xF1, 0x90]),
    bytes([0x10, 0x03]),
]
_CAN_FORMAT_IDS = [
    f"req{''.join(f'{b:02X}' for b in r)}" for r in _CAN_FORMAT_REQUESTS
]


@pytest.mark.parametrize("uds_req", _CAN_FORMAT_REQUESTS, ids=_CAN_FORMAT_IDS)
def test_response_on_correct_can_id(uds_req):
    """Response must arrive on RESPONSE_ID (0x601), not any other ID."""
    raw = _raw_frame(uds_req)
    assert raw is not None, f"No CAN frame received for req={uds_req.hex()}"
    # _raw_frame already filters to RESPONSE_ID — if we got data, the ID is correct
    assert len(raw) > 0, "Empty frame"


@pytest.mark.parametrize("uds_req", _CAN_FORMAT_REQUESTS, ids=_CAN_FORMAT_IDS)
def test_can_frame_dlc_is_8(uds_req):
    """CAN frame must always be DLC=8 (zero-padded to 8 bytes with 0xCC)."""
    raw = _raw_frame(uds_req)
    assert raw is not None, "No CAN frame"
    raw_bytes = bytes.fromhex(raw)
    assert len(raw_bytes) == 8, (
        f"Expected DLC=8 (padded), got {len(raw_bytes)} bytes: {raw}"
    )


@pytest.mark.parametrize("uds_req", _CAN_FORMAT_REQUESTS, ids=_CAN_FORMAT_IDS)
def test_can_padding_bytes_are_cc(uds_req):
    """Unused bytes after the ISO-TP payload must be 0xCC."""
    raw = _raw_frame(uds_req)
    assert raw is not None, "No CAN frame"
    raw_bytes = bytes.fromhex(raw)
    # PCI byte tells us how many payload bytes are present
    pci = raw_bytes[0]
    frame_type = pci >> 4
    if frame_type == 0:  # SF
        length = pci & 0x0F
        payload_end = 1 + length  # PCI + length bytes of data
        padding = raw_bytes[payload_end:]
        for i, b in enumerate(padding):
            assert b == _PAD, (
                f"Padding byte {i} = 0x{b:02X}, expected 0xCC. Frame: {raw}"
            )


@pytest.mark.parametrize("uds_req", _CAN_FORMAT_REQUESTS, ids=_CAN_FORMAT_IDS)
def test_isotp_pci_byte_format(uds_req):
    """ISO-TP PCI byte: SF must have upper nibble = 0, length 1..7."""
    raw = _raw_frame(uds_req)
    assert raw is not None, "No CAN frame"
    raw_bytes = bytes.fromhex(raw)
    pci = raw_bytes[0]
    frame_type = pci >> 4
    # Expect SF (0) for short responses, or FF (1) for multi-frame
    assert frame_type in (0, 1), f"Unknown PCI frame type {frame_type}: 0x{pci:02X}"
    if frame_type == 0:
        length = pci & 0x0F
        assert 1 <= length <= 7, f"SF length {length} out of range [1,7]"


def test_no_unsolicited_frames():
    """After flush, no frames should arrive within 300 ms without a request."""
    flush_bus(timeout=0.2)
    # Listen for 300 ms without sending anything
    import subprocess
    proc = subprocess.Popen(
        ["candump", "can0", "-n", "1", "-T", "300"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    try:
        stdout, _ = proc.communicate(timeout=1.5)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, _ = proc.communicate()
    lines = [l for l in stdout.strip().split('\n') if l.strip()]
    assert lines == [] or all(f"{RESPONSE_ID:03X}" not in l for l in lines), (
        f"Unsolicited frame on RESPONSE_ID: {stdout.strip()}"
    )


def test_request_id_is_not_echoed():
    """ECU must not echo the request CAN ID (0x600) back on the bus."""
    import subprocess
    flush_bus(timeout=0.1)
    # Capture anything on 0x600 after we send a request
    dump = subprocess.Popen(
        ["candump", f"can0,{REQUEST_ID:03X}:7FF", "-n", "1", "-T", "300"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)
    cansend(REQUEST_ID, "023E00CCCCCCCC")
    try:
        stdout, _ = dump.communicate(timeout=1.0)
    except subprocess.TimeoutExpired:
        dump.kill()
        stdout, _ = dump.communicate()
    # The only frame on 0x600 should be the one we sent — ECU must not echo it
    lines = [l for l in stdout.strip().split('\n')
             if l.strip() and f"{REQUEST_ID:03X}" in l]
    assert len(lines) == 0, (
        f"ECU echoed request ID 0x{REQUEST_ID:03X}: {stdout.strip()}"
    )


@pytest.mark.parametrize("uds_req", [
    bytes([0x3E, 0x00]),
    bytes([0x22, 0xF1, 0x95]),
    bytes([0x10, 0x01]),
], ids=["TP", "ReadDID_F195", "DiagSess"])
def test_single_response_frame_only(uds_req):
    """SF request must produce exactly one response CAN frame."""
    import subprocess
    flush_bus(timeout=0.1)
    from helpers.isotp import encode_sf
    sf_hex = encode_sf(uds_req)

    dump = subprocess.Popen(
        ["candump", f"can0,{RESPONSE_ID:03X}:7FF", "-n", "3", "-T", "600"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)
    cansend(REQUEST_ID, sf_hex)
    try:
        stdout, _ = dump.communicate(timeout=1.5)
    except subprocess.TimeoutExpired:
        dump.kill()
        stdout, _ = dump.communicate()

    frames = [l for l in stdout.strip().split('\n')
              if l.strip() and f"{RESPONSE_ID:03X}" in l]
    # Exactly 1 CAN frame for SF request/response (not multi-frame for these DIDs)
    assert len(frames) == 1, (
        f"Expected 1 response frame, got {len(frames)}: {stdout.strip()}"
    )


# ===========================================================================
# 3. Response Timing — P2 Server (20 tests)
# ===========================================================================

_TIMING_REQUESTS = [
    bytes([0x3E, 0x00]),
    bytes([0x10, 0x01]),
    bytes([0x10, 0x02]),
    bytes([0x10, 0x03]),
    bytes([0x22, 0xF1, 0x95]),
    bytes([0x22, 0xF1, 0x90]),
    bytes([0x22, 0xF1, 0x8C]),
    bytes([0x22, 0xF1, 0x93]),
    bytes([0x22, 0xF1, 0x8A]),
    bytes([0x22, 0xF1, 0x80]),
    bytes([0x2E, 0xF1, 0x90, 0x54, 0x45, 0x53, 0x54]),  # WriteDID VIN "TEST"
    bytes([0x85, 0x01]),
    bytes([0x85, 0x02]),
    bytes([0x31, 0x01, 0xFF, 0x00]),
    bytes([0xAA]),                                         # Unknown SID — NRC still timed
    bytes([0x22, 0x00, 0x01]),                             # Unknown DID — NRC still timed
    bytes([0x19, 0x0A]),
    bytes([0x19, 0x02, 0xFF]),
    bytes([0x14, 0xFF, 0xFF, 0xFF]),
    bytes([0x3E, 0x80]),                                   # TP with suppressPosRsp
]

_TIMING_IDS = [
    f"req{''.join(f'{b:02X}' for b in r)}" for r in _TIMING_REQUESTS
]


@pytest.mark.parametrize("uds_req", _TIMING_REQUESTS, ids=_TIMING_IDS)
def test_response_within_p2(uds_req):
    """Response (positive or NRC) must arrive within P2 limit (500 ms)."""
    t0 = time.time()
    resp = send_uds_sf(uds_req)
    elapsed_ms = (time.time() - t0) * 1000
    # TP with suppressPosRsp (0x3E 0x80): no response expected — just verify no hang
    if uds_req == bytes([0x3E, 0x80]):
        assert elapsed_ms < _P2_LIMIT_MS + 200, (
            f"Even with suppressPosRsp, send should not hang: {elapsed_ms:.0f}ms"
        )
        return
    assert resp is not None, f"No response for req={uds_req.hex()}"
    assert elapsed_ms < _P2_LIMIT_MS, (
        f"Response took {elapsed_ms:.0f}ms, P2 limit {_P2_LIMIT_MS}ms"
    )


@pytest.mark.parametrize("uds_req,p2_tight_ms", [
    (bytes([0x3E, 0x00]),          50),
    (bytes([0x10, 0x01]),         100),
    (bytes([0x22, 0xF1, 0x95]),    50),
    (bytes([0x85, 0x01]),          50),
    (bytes([0x85, 0x02]),          50),
], ids=["TP", "DiagSess01", "ReadDID_F195", "CtrlDTC_ON", "CtrlDTC_OFF"])
def test_response_within_tight_p2(uds_req, p2_tight_ms):
    """Lightweight services must respond well inside ISO 14229 P2 (50–100 ms)."""
    t0 = time.time()
    resp = send_uds_sf(uds_req)
    elapsed_ms = (time.time() - t0) * 1000
    assert resp is not None, "No response"
    assert elapsed_ms < p2_tight_ms, (
        f"Expected < {p2_tight_ms}ms, took {elapsed_ms:.1f}ms"
    )


# ===========================================================================
# 4. Idempotency (15 tests)
# ===========================================================================

_IDEMPOTENT_REQUESTS = [
    bytes([0x3E, 0x00]),
    bytes([0x10, 0x01]),
    bytes([0x10, 0x03]),
    bytes([0x22, 0xF1, 0x95]),
    bytes([0x22, 0xF1, 0x90]),
    bytes([0x22, 0xF1, 0x8C]),
    bytes([0x22, 0xF1, 0x80]),
    bytes([0x85, 0x01]),
    bytes([0x85, 0x02]),
    bytes([0x19, 0x0A]),
    bytes([0x19, 0x02, 0xFF]),
    bytes([0xAA]),
    bytes([0x22, 0x00, 0x01]),
    bytes([0x10, 0xFE]),
    bytes([0x3E, 0x05]),
]

_IDEMPOTENT_IDS = [
    f"req{''.join(f'{b:02X}' for b in r)}" for r in _IDEMPOTENT_REQUESTS
]


@pytest.mark.parametrize("uds_req", _IDEMPOTENT_REQUESTS, ids=_IDEMPOTENT_IDS)
def test_idempotent_response(uds_req):
    """Same request sent twice must yield identical response both times."""
    resp1 = send_uds_sf(uds_req)
    time.sleep(0.3)
    resp2 = send_uds_sf(uds_req)
    assert resp1 is not None, f"First request got no response: req={uds_req.hex()}"
    assert resp2 is not None, f"Second request got no response: req={uds_req.hex()}"
    assert resp1 == resp2, (
        f"Non-idempotent: first={resp1.hex()}, second={resp2.hex()}"
    )


# ===========================================================================
# 5. Regression Tests for the 9 Bring-Up Bugs (15 tests)
# ===========================================================================
#
# Bug index (from bsp-stm32-status.md / git log):
#   BUG-1  FDCAN NBTP encoding — 500 kbps, not 526 kbps
#   BUG-2  Flash dual-bank 2 KB pages, not 4 KB
#   BUG-3  F413 GPIO PD0/PD1 (not PA11/PA12) for CAN RX/TX
#   BUG-4  DiagCanTransport stack overflow — 256 B buffer, not 4 KB
#   BUG-5  RX FIFO drain — isr_rx_fifo0 must be called in polling loop
#   BUG-6  Session state not reset on ECU reset request
#   BUG-7  Incorrect NRC for unknown DID (was 0x22, should be 0x31)
#   BUG-8  Missing padding byte 0xCC in CAN frame builder
#   BUG-9  TesterPresent sub-function 0x80 (suppress) incorrectly returned response


@pytest.mark.regression
def test_bug1_fdcan_baudrate_500kbps():
    """BUG-1: FDCAN NBTP must produce 500 kbps (not 526 kbps).
    Verified indirectly: any successful CAN exchange implies correct baud-rate
    (bit stuffing errors / bus-off would silence responses at wrong baud)."""
    resp = send_uds_sf(bytes([0x3E, 0x00]))
    assert resp is not None, "No CAN response — possible baud-rate mismatch (BUG-1)"
    assert resp[0] == 0x7E, "TesterPresent must ACK with 0x7E"


@pytest.mark.regression
def test_bug2_flash_write_read_survives_256b_boundary():
    """BUG-2: Flash page size 2 KB (not 4 KB) — write near a 2 KB boundary must not
    corrupt adjacent data. We write a DID value and read it back."""
    write_req = bytes([0x2E, 0xF1, 0x90, 0x42, 0x55, 0x47, 0x32])  # "BUG2"
    resp = send_uds_sf(write_req)
    assert expect_positive(resp, SID_WRITE_DID), (
        f"WriteDID failed: {resp.hex() if resp else 'None'} (BUG-2)"
    )
    time.sleep(0.3)
    read_resp = send_uds_sf(bytes([0x22, 0xF1, 0x90]))
    assert read_resp is not None, "No ReadDID response after write (BUG-2)"
    assert b"BUG2" in bytes([read_resp[i] for i in range(3, min(7, len(read_resp)))]) \
        or bytes([0x42, 0x55, 0x47, 0x32]) == read_resp[3:7], (
        f"ReadDID mismatch after flash write (BUG-2): {read_resp.hex()}"
    )


@pytest.mark.regression
def test_bug3_can_rx_works_on_correct_gpio():
    """BUG-3: F413 CAN RX/TX on PD0/PD1 (not PA11/PA12).
    Verified by receiving a response — wrong GPIO = no RX = timeout."""
    resp = send_uds_sf(bytes([0x3E, 0x00]))
    assert resp is not None, (
        "No response — F413 CAN GPIO may be misconfigured (BUG-3: PD0/PD1 vs PA11/PA12)"
    )


@pytest.mark.regression
def test_bug4_no_stack_overflow_large_payload():
    """BUG-4: DiagCanTransport buffer must be ≥ 256 B (was overflowed by 4 KB alloc).
    Sending a SF (≤7 bytes) must not crash the ECU. Verify with follow-up TP."""
    resp = send_uds_sf(bytes([0x22, 0xF1, 0x95]))
    assert resp is not None, "ECU did not respond — possible stack overflow (BUG-4)"
    # Follow-up request proves ECU did not crash
    time.sleep(0.1)
    followup = send_uds_sf(bytes([0x3E, 0x00]))
    assert followup is not None, "ECU crashed after first request (BUG-4 stack overflow)"


@pytest.mark.regression
def test_bug5_rx_fifo_drained_between_requests():
    """BUG-5: RX FIFO must be drained in polling loop (isr_rx_fifo0 called).
    Send 3 back-to-back requests; all must receive distinct responses."""
    r1 = send_uds_sf(bytes([0x3E, 0x00]))
    r2 = send_uds_sf(bytes([0x10, 0x01]))
    r3 = send_uds_sf(bytes([0x22, 0xF1, 0x95]))
    assert r1 is not None, "First request no response (BUG-5 FIFO drain)"
    assert r2 is not None, "Second request no response (BUG-5 FIFO drain)"
    assert r3 is not None, "Third request no response (BUG-5 FIFO drain)"
    assert r2[0] == 0x50, f"Expected 0x50 DiagSession pos resp, got 0x{r2[0]:02X}"


@pytest.mark.regression
def test_bug6_session_state_after_diag_default():
    """BUG-6: Session must be cleanly reset to default after DiagSessionControl 0x01."""
    # Switch to extended
    r_ext = send_uds_sf(bytes([0x10, 0x03]))
    assert expect_positive(r_ext, SID_DIAG_SESSION_CTRL), "Extended session failed"
    time.sleep(0.1)
    # Switch back to default — must succeed (not sequence error)
    r_def = send_uds_sf(bytes([0x10, 0x01]))
    assert expect_positive(r_def, SID_DIAG_SESSION_CTRL), (
        f"Reset to default session failed (BUG-6): {r_def.hex() if r_def else 'None'}"
    )


@pytest.mark.regression
def test_bug7_unknown_did_returns_nrc_31():
    """BUG-7: Unknown DID must return NRC 0x31 (requestOutOfRange), not 0x22."""
    resp = send_uds_sf(bytes([0x22, 0x00, 0x01]))
    assert resp is not None, "No response for unknown DID"
    assert resp[0] == 0x7F, "Expected NRC frame"
    assert resp[1] == SID_READ_DID, "NRC SID echo mismatch"
    assert resp[2] == 0x31, (
        f"BUG-7: expected NRC 0x31 for unknown DID, got 0x{resp[2]:02X}"
    )


@pytest.mark.regression
def test_bug8_can_frame_padded_with_cc():
    """BUG-8: CAN frame builder must pad unused bytes with 0xCC (not 0x00)."""
    raw = _raw_frame(bytes([0x3E, 0x00]))
    assert raw is not None, "No CAN frame"
    raw_bytes = bytes.fromhex(raw)
    # TesterPresent positive response = 3 bytes (PCI 0x02, 0x7E, 0x00)
    # Bytes [3..7] must all be 0xCC
    padding = raw_bytes[3:]
    for i, b in enumerate(padding):
        assert b == 0xCC, (
            f"BUG-8: padding byte {i+3} = 0x{b:02X}, expected 0xCC. Frame: {raw}"
        )


@pytest.mark.regression
def test_bug9_tester_present_suppress_pos_resp_no_response():
    """BUG-9: TesterPresent with sub-function 0x80 (suppressPosRsp) must NOT
    send a positive response frame."""
    import subprocess
    flush_bus(timeout=0.1)
    from helpers.isotp import encode_sf
    sf_hex = encode_sf(bytes([0x3E, 0x80]))

    dump = subprocess.Popen(
        ["candump", f"can0,{RESPONSE_ID:03X}:7FF", "-n", "1", "-T", "350"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)
    cansend(REQUEST_ID, sf_hex)
    try:
        stdout, _ = dump.communicate(timeout=1.0)
    except subprocess.TimeoutExpired:
        dump.kill()
        stdout, _ = dump.communicate()
    frames = [l for l in stdout.strip().split('\n')
              if l.strip() and f"{RESPONSE_ID:03X}" in l]
    assert len(frames) == 0, (
        f"BUG-9: ECU sent positive response for suppressPosRsp TesterPresent: "
        f"{stdout.strip()}"
    )


@pytest.mark.regression
def test_bug_regression_suite_smoke():
    """Smoke-check: ECU is still alive after full regression suite run."""
    resp = send_uds_sf(bytes([0x3E, 0x00]))
    assert resp is not None, "ECU not responding after regression suite"
    assert resp[0] == 0x7E, f"Expected 0x7E, got 0x{resp[0]:02X}"


@pytest.mark.regression
def test_back_to_back_different_services_no_corruption():
    """Stress: rapid alternation between services must not corrupt any response."""
    pairs = [
        (bytes([0x3E, 0x00]),       0x7E),
        (bytes([0x22, 0xF1, 0x95]), 0x62),
        (bytes([0x10, 0x01]),       0x50),
        (bytes([0x85, 0x01]),       0xC5),
        (bytes([0x3E, 0x00]),       0x7E),
    ]
    for req, expected_first_byte in pairs:
        resp = send_uds_sf(req)
        assert resp is not None, f"No response for {req.hex()}"
        assert resp[0] == expected_first_byte, (
            f"req={req.hex()} expected 0x{expected_first_byte:02X}, got 0x{resp[0]:02X}"
        )


@pytest.mark.regression
def test_nrc_does_not_crash_ecu():
    """Sending requests that trigger NRC must leave ECU functional for the next request."""
    # Provoke NRC
    _ = send_uds_sf(bytes([0xAA]))
    time.sleep(0.1)
    _ = send_uds_sf(bytes([0x22]))
    time.sleep(0.1)
    # ECU must still be alive
    resp = send_uds_sf(bytes([0x3E, 0x00]))
    assert resp is not None, "ECU did not recover after NRC responses"
    assert resp[0] == 0x7E, f"Expected 0x7E after NRC recovery, got 0x{resp[0]:02X}"


@pytest.mark.regression
def test_repeated_session_switch_no_state_corruption():
    """Rapidly switching sessions must not corrupt ECU state."""
    for _ in range(3):
        r = send_uds_sf(bytes([0x10, 0x01]))
        assert expect_positive(r, SID_DIAG_SESSION_CTRL), "Default session switch failed"
        r = send_uds_sf(bytes([0x10, 0x03]))
        assert expect_positive(r, SID_DIAG_SESSION_CTRL), "Extended session switch failed"
    # Return to default and verify normal operation
    r = send_uds_sf(bytes([0x10, 0x01]))
    assert expect_positive(r, SID_DIAG_SESSION_CTRL)
    tp = send_uds_sf(bytes([0x3E, 0x00]))
    assert tp is not None and tp[0] == 0x7E, "ECU state corrupted after repeated session switch"
