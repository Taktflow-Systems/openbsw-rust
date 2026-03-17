"""Low-level CAN transport using cansend/candump subprocesses."""

import subprocess
import time
from typing import Optional, Tuple

IFACE = "can0"
REQUEST_ID = 0x600
RESPONSE_ID = 0x601
DEFAULT_TIMEOUT = 2.0


def cansend(can_id: int, data_hex: str) -> None:
    """Send a single CAN frame. data_hex like '023E00'."""
    frame = f"{can_id:03X}#{data_hex}"
    subprocess.run(["cansend", IFACE, frame], check=True, timeout=2)


def send_recv_raw(can_id: int, data_hex: str, resp_id: int = RESPONSE_ID,
                  timeout: float = DEFAULT_TIMEOUT) -> Optional[str]:
    """Send a CAN frame, receive one frame on resp_id. Returns hex data or None."""
    dump = subprocess.Popen(
        ["candump", f"{IFACE},{resp_id:03X}:7FF", "-n", "1",
         "-T", str(int(timeout * 1000))],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)
    cansend(can_id, data_hex)
    try:
        stdout, _ = dump.communicate(timeout=timeout + 1)
        for line in stdout.strip().split('\n'):
            if f"{resp_id:03X}" in line and ']' in line:
                parts = line.split(']')
                if len(parts) >= 2:
                    return parts[1].strip().replace(' ', '')
    except subprocess.TimeoutExpired:
        dump.kill()
    return None


def send_recv_multi(can_id: int, data_hex: str, resp_id: int = RESPONSE_ID,
                    fc_id: int = REQUEST_ID, timeout: float = 3.0,
                    max_frames: int = 10) -> Optional[str]:
    """Send a CAN frame, receive multi-frame response (FF+FC+CFs).
    Returns reassembled hex data or None."""
    dump = subprocess.Popen(
        ["candump", f"{IFACE},{resp_id:03X}:7FF", "-n", str(max_frames),
         "-T", str(int(timeout * 1000))],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    time.sleep(0.05)
    cansend(can_id, data_hex)
    time.sleep(0.1)
    # Send FC(CTS) for multi-frame
    cansend(fc_id, "300000")

    try:
        stdout, _ = dump.communicate(timeout=timeout + 1)
    except subprocess.TimeoutExpired:
        dump.kill()
        stdout, _ = dump.communicate()

    frames = []
    for line in stdout.strip().split('\n'):
        if f"{resp_id:03X}" in line and ']' in line:
            parts = line.split(']')
            if len(parts) >= 2:
                frames.append(parts[1].strip().replace(' ', ''))

    if not frames:
        return None

    first_pci = int(frames[0][:2], 16)
    ft = first_pci >> 4

    if ft == 0:  # SF
        length = first_pci & 0x0F
        return frames[0][2:2 + length * 2]

    if ft == 1:  # FF
        msg_len = ((first_pci & 0x0F) << 8) | int(frames[0][2:4], 16)
        result = frames[0][4:]
        for f in frames[1:]:
            f_pci = int(f[:2], 16)
            if (f_pci >> 4) == 2:
                result += f[2:]
        return result[:msg_len * 2]

    return None


def flush_bus(timeout: float = 0.2) -> None:
    """Drain any pending frames from the CAN bus."""
    try:
        subprocess.run(
            ["candump", IFACE, "-n", "100", "-T", str(int(timeout * 1000))],
            capture_output=True, timeout=timeout + 0.5
        )
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
        pass
