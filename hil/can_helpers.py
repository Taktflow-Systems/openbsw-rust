"""CAN + ISO-TP + UDS helpers for HIL testing."""

import can
import time
from typing import Optional, List, Tuple

REQUEST_ID = 0x600
RESPONSE_ID = 0x601

def create_bus(channel='can0', bitrate=500000):
    """Create a SocketCAN bus instance."""
    return can.Bus(channel=channel, interface='socketcan', bitrate=bitrate)

def isotp_encode_sf(data: bytes) -> bytes:
    """Encode a UDS payload as ISO-TP Single Frame."""
    assert len(data) <= 7
    return bytes([len(data)]) + data

def isotp_decode_sf(frame_data: bytes) -> Optional[bytes]:
    """Decode an ISO-TP SF. Returns UDS payload or None."""
    if len(frame_data) < 1:
        return None
    pci = frame_data[0]
    if (pci >> 4) != 0:
        return None
    length = pci & 0x0F
    if length == 0 or length > 7 or len(frame_data) < 1 + length:
        return None
    return frame_data[1:1+length]

def send_uds_request(bus, data: bytes, request_id=REQUEST_ID):
    """Send a UDS request as ISO-TP SF."""
    payload = isotp_encode_sf(data)
    msg = can.Message(arbitration_id=request_id, data=payload, is_extended_id=False)
    bus.send(msg)

def receive_uds_response(bus, timeout=2.0, response_id=RESPONSE_ID) -> Optional[bytes]:
    """Receive a UDS response (SF only). Returns UDS payload or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=deadline - time.time())
        if msg is None:
            return None
        if msg.arbitration_id == response_id:
            return isotp_decode_sf(msg.data)
    return None

def send_recv_uds(bus, request: bytes, timeout=2.0) -> Optional[bytes]:
    """Send UDS request and wait for SF response."""
    send_uds_request(bus, request)
    return receive_uds_response(bus, timeout)

def receive_multiframe_response(bus, timeout=2.0, response_id=RESPONSE_ID, request_id=REQUEST_ID) -> Optional[bytes]:
    """Receive a multi-frame ISO-TP response (FF + FC + CFs).
    Sends FC(CTS) automatically after receiving FF.
    Returns reassembled UDS payload or None."""
    deadline = time.time() + timeout

    # Wait for first frame (FF or SF)
    while time.time() < deadline:
        msg = bus.recv(timeout=deadline - time.time())
        if msg is None:
            return None
        if msg.arbitration_id != response_id:
            continue

        pci = msg.data[0]
        frame_type = pci >> 4

        if frame_type == 0:  # SF
            length = pci & 0x0F
            return bytes(msg.data[1:1+length])

        if frame_type == 1:  # FF
            msg_len = ((pci & 0x0F) << 8) | msg.data[1]
            result = bytearray(msg.data[2:8])  # first 6 bytes

            # Send FC (CTS, BS=0, STmin=0)
            fc = can.Message(arbitration_id=request_id, data=bytes([0x30, 0x00, 0x00]), is_extended_id=False)
            bus.send(fc)

            # Collect CFs
            expected_seq = 1
            while len(result) < msg_len and time.time() < deadline:
                cf_msg = bus.recv(timeout=deadline - time.time())
                if cf_msg is None:
                    break
                if cf_msg.arbitration_id != response_id:
                    continue
                cf_pci = cf_msg.data[0]
                if (cf_pci >> 4) != 2:
                    continue
                seq = cf_pci & 0x0F
                if seq != (expected_seq & 0x0F):
                    break
                remaining = msg_len - len(result)
                chunk = min(remaining, 7)
                result.extend(cf_msg.data[1:1+chunk])
                expected_seq += 1

            return bytes(result[:msg_len])

    return None

def send_recv_uds_multiframe(bus, request: bytes, timeout=2.0) -> Optional[bytes]:
    """Send UDS request, receive response (handles both SF and multi-frame)."""
    send_uds_request(bus, request)
    return receive_multiframe_response(bus, timeout)
