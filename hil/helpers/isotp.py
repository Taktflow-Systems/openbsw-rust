"""ISO-TP (ISO 15765-2) frame encoding/decoding helpers."""


def encode_sf(data: bytes) -> str:
    """Encode UDS payload as ISO-TP Single Frame hex string."""
    assert len(data) <= 7, f"SF max 7 bytes, got {len(data)}"
    pci = len(data)
    return f"{pci:02X}" + data.hex().upper()


def decode_sf(frame_hex: str) -> bytes:
    """Decode ISO-TP SF hex → UDS payload bytes. Raises on non-SF."""
    raw = bytes.fromhex(frame_hex)
    pci = raw[0]
    if (pci >> 4) != 0:
        raise ValueError(f"Not a SF: PCI=0x{pci:02X}")
    length = pci & 0x0F
    if length == 0 or length > 7:
        raise ValueError(f"Invalid SF length: {length}")
    return raw[1:1 + length]


def is_sf(frame_hex: str) -> bool:
    """Check if hex frame is a Single Frame."""
    try:
        pci = int(frame_hex[:2], 16)
        return (pci >> 4) == 0 and (pci & 0x0F) > 0
    except (ValueError, IndexError):
        return False


def is_ff(frame_hex: str) -> bool:
    """Check if hex frame is a First Frame."""
    try:
        pci = int(frame_hex[:2], 16)
        return (pci >> 4) == 1
    except (ValueError, IndexError):
        return False


def encode_fc_cts() -> str:
    """Encode Flow Control CTS (Continue To Send), BS=0, STmin=0."""
    return "300000"


def ff_msg_len(frame_hex: str) -> int:
    """Extract message length from First Frame."""
    raw = bytes.fromhex(frame_hex)
    pci = raw[0]
    return ((pci & 0x0F) << 8) | raw[1]
