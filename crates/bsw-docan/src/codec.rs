//! ISO-TP frame encoder and decoder.
//!
//! Handles encoding/decoding of Protocol Control Information (PCI) bytes for
//! all four ISO 15765-2 frame types: Single (SF), First (FF), Consecutive (CF),
//! and Flow Control (FC).
//!
//! # Frame Formats (classic CAN, 8 bytes, `pci_offset = 0`)
//!
//! **Single Frame (SF)**
//! - Short form (data ≤ 7):  `[0x0N, d0..dN-1]`  where N = data length
//! - Long form (data 8–62):  `[0x00, N, d0..dN-1]` (CAN-FD escape)
//!
//! **First Frame (FF)**
//! - Normal (len ≤ 4095):   `[0x1M, LL, d0..d5]` where ML = len in 12 bits
//! - Extended (len > 4095): `[0x10, 0x00, LL, LL, LL, LL, d0..d1]` (32-bit BE len)
//!
//! **Consecutive Frame (CF)**
//! - `[0x2N, d0..d6]` where N = `sequence_number` & 0x0F
//!
//! **Flow Control (FC)**
//! - `[0x3S, block_size, st_min, <padding>]` where S = [`FlowStatus`]

use crate::constants::{CodecResult, FlowStatus, FrameType};

// ── Codec configuration ───────────────────────────────────────────────────────

/// Codec configuration controlling addressing offset and padding byte.
#[derive(Debug, Clone, Copy)]
pub struct CodecConfig {
    /// Number of bytes before the PCI byte (e.g., 1 for normal addressing mode).
    pub pci_offset: usize,
    /// Filler byte used when padding frames to CAN DLC (typically `0xCC` or `0xAA`).
    pub filler_byte: u8,
}

impl Default for CodecConfig {
    fn default() -> Self {
        Self {
            pci_offset: 0,
            filler_byte: 0xCC,
        }
    }
}

// ── Decoded frame types ────────────────────────────────────────────────────────

/// A decoded single frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SingleFrame<'a> {
    /// Length of the payload as encoded in the PCI byte.
    pub data_length: u16,
    /// Payload slice (references the original buffer).
    pub data: &'a [u8],
}

/// A decoded first frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FirstFrame<'a> {
    /// Total message length as encoded in the FF PCI field.
    pub message_length: u32,
    /// Payload carried in this first frame (references original buffer).
    pub data: &'a [u8],
}

/// A decoded consecutive frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsecutiveFrame<'a> {
    /// Sequence number (0–15, wraps).
    pub sequence_number: u8,
    /// Payload carried in this frame (references original buffer).
    pub data: &'a [u8],
}

/// A decoded flow control frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlowControlFrame {
    /// Flow status (CTS / Wait / Overflow).
    pub status: FlowStatus,
    /// Block size: how many consecutive frames to send before waiting for next FC.
    /// 0 = unlimited.
    pub block_size: u8,
    /// Raw encoded separation time byte (decode with [`crate::parameters::decode_separation_time`]).
    pub separation_time: u8,
}

/// The result of decoding a raw CAN frame buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodedFrame<'a> {
    /// Single frame — complete message.
    Single(SingleFrame<'a>),
    /// First frame — start of a segmented message.
    First(FirstFrame<'a>),
    /// Consecutive frame — segment continuation.
    Consecutive(ConsecutiveFrame<'a>),
    /// Flow control — receiver feedback.
    FlowControl(FlowControlFrame),
}

// ── Frame type detection ───────────────────────────────────────────────────────

/// Decode the frame type from the PCI byte at `config.pci_offset`.
///
/// Returns [`CodecResult::InvalidFrameSize`] if the buffer is too short to
/// contain the PCI byte, or [`CodecResult::InvalidFrameType`] for unknown types.
pub fn decode_frame_type(data: &[u8], config: &CodecConfig) -> Result<FrameType, CodecResult> {
    let pci = data.get(config.pci_offset).ok_or(CodecResult::InvalidFrameSize)?;
    match pci >> 4 {
        0 => Ok(FrameType::Single),
        1 => Ok(FrameType::First),
        2 => Ok(FrameType::Consecutive),
        3 => Ok(FrameType::FlowControl),
        _ => Err(CodecResult::InvalidFrameType),
    }
}

// ── Decode ─────────────────────────────────────────────────────────────────────

/// Decode a complete frame from a raw CAN data buffer.
///
/// The `data` slice must contain all bytes of the received CAN frame.
/// Returned sub-slices borrow from `data`, so no copying occurs.
#[allow(clippy::too_many_lines)] // Frame decoder is inherently a large match over 4 frame types
pub fn decode_frame<'a>(data: &'a [u8], config: &CodecConfig) -> Result<DecodedFrame<'a>, CodecResult> {
    let off = config.pci_offset;
    // Need at least the PCI byte itself
    if data.len() <= off {
        return Err(CodecResult::InvalidFrameSize);
    }
    let pci = data[off];
    match pci >> 4 {
        // ── Single Frame ────────────────────────────────────────────────────
        0 => decode_single_frame(data, off, pci),
        // ── First Frame ─────────────────────────────────────────────────────
        1 => decode_first_frame(data, off, pci),
        // ── Consecutive Frame ────────────────────────────────────────────────
        2 => {
            let seq = pci & 0x0F;
            let payload_start = off + 1;
            if payload_start > data.len() {
                return Err(CodecResult::InvalidFrameSize);
            }
            Ok(DecodedFrame::Consecutive(ConsecutiveFrame {
                sequence_number: seq,
                data: &data[payload_start..],
            }))
        }
        // ── Flow Control ─────────────────────────────────────────────────────
        3 => {
            if data.len() < off + 3 {
                return Err(CodecResult::InvalidFrameSize);
            }
            let raw_status = pci & 0x0F;
            let status = match raw_status {
                0 => FlowStatus::ContinueToSend,
                1 => FlowStatus::Wait,
                2 => FlowStatus::Overflow,
                _ => return Err(CodecResult::InvalidFrameType),
            };
            Ok(DecodedFrame::FlowControl(FlowControlFrame {
                status,
                block_size: data[off + 1],
                separation_time: data[off + 2],
            }))
        }
        _ => Err(CodecResult::InvalidFrameType),
    }
}

fn decode_single_frame(data: &[u8], off: usize, pci: u8) -> Result<DecodedFrame<'_>, CodecResult> {
    let dl = pci & 0x0F;
    if dl == 0 {
        // Long SF (CAN-FD escape): second byte holds actual length
        if data.len() < off + 2 {
            return Err(CodecResult::InvalidFrameSize);
        }
        let long_dl = u16::from(data[off + 1]);
        if long_dl < 8 {
            return Err(CodecResult::InvalidMessageSize);
        }
        let payload_start = off + 2;
        let payload_end = payload_start + long_dl as usize;
        if payload_end > data.len() {
            return Err(CodecResult::InvalidFrameSize);
        }
        Ok(DecodedFrame::Single(SingleFrame {
            data_length: long_dl,
            data: &data[payload_start..payload_end],
        }))
    } else {
        // Short SF
        let payload_start = off + 1;
        let payload_end = payload_start + dl as usize;
        if payload_end > data.len() {
            return Err(CodecResult::InvalidFrameSize);
        }
        Ok(DecodedFrame::Single(SingleFrame {
            data_length: u16::from(dl),
            data: &data[payload_start..payload_end],
        }))
    }
}

fn decode_first_frame(data: &[u8], off: usize, pci: u8) -> Result<DecodedFrame<'_>, CodecResult> {
    if data.len() < off + 2 {
        return Err(CodecResult::InvalidFrameSize);
    }
    let high_nibble = u32::from(pci & 0x0F);
    let low_byte = u32::from(data[off + 1]);

    if high_nibble == 0 && low_byte == 0 {
        // Extended FF: next 4 bytes are the 32-bit big-endian length
        if data.len() < off + 6 {
            return Err(CodecResult::InvalidFrameSize);
        }
        let len = u32::from_be_bytes([
            data[off + 2],
            data[off + 3],
            data[off + 4],
            data[off + 5],
        ]);
        if len <= 4095 {
            return Err(CodecResult::InvalidMessageSize);
        }
        let payload_start = off + 6;
        Ok(DecodedFrame::First(FirstFrame {
            message_length: len,
            data: &data[payload_start..],
        }))
    } else {
        // Normal FF: 12-bit length
        let len = (high_nibble << 8) | low_byte;
        if len < 8 {
            return Err(CodecResult::InvalidMessageSize);
        }
        let payload_start = off + 2;
        if payload_start > data.len() {
            return Err(CodecResult::InvalidFrameSize);
        }
        Ok(DecodedFrame::First(FirstFrame {
            message_length: len,
            data: &data[payload_start..],
        }))
    }
}

// ── Encode ─────────────────────────────────────────────────────────────────────

/// Calculate the total number of CAN frames needed to transmit a message.
///
/// - `message_length`: total bytes in the ISO-TP payload.
/// - `first_frame_data_size`: payload bytes that fit in the first frame.
/// - `consecutive_frame_data_size`: payload bytes per consecutive frame.
pub fn frame_count(
    message_length: u32,
    first_frame_data_size: usize,
    consecutive_frame_data_size: usize,
) -> u32 {
    if message_length as usize <= first_frame_data_size {
        1
    } else {
        let remaining = message_length as usize - first_frame_data_size;
        let cf_count = remaining.div_ceil(consecutive_frame_data_size);
        // cf_count is bounded by message_length/1 at most = u32::MAX frames; safe cast.
        #[allow(clippy::cast_possible_truncation)]
        { 1 + cf_count as u32 }
    }
}

/// Encode a single frame into `buf`.
///
/// Chooses long form (CAN-FD escape `0x00`) automatically when
/// `data.len() >= 8`.
///
/// Returns the number of bytes written, or an error if `buf` is too small.
pub fn encode_single_frame(
    buf: &mut [u8],
    data: &[u8],
    config: &CodecConfig,
) -> Result<usize, CodecResult> {
    let off = config.pci_offset;
    let dl = data.len();

    if dl == 0 || dl > 62 {
        return Err(CodecResult::InvalidMessageSize);
    }

    if dl <= 7 {
        // Short form — dl fits in nibble (1..=7)
        let needed = off + 1 + dl;
        if buf.len() < needed {
            return Err(CodecResult::InvalidFrameSize);
        }
        // dl ≤ 7, safe truncation to u8
        #[allow(clippy::cast_possible_truncation)]
        { buf[off] = dl as u8; }
        buf[off + 1..off + 1 + dl].copy_from_slice(data);
        Ok(off + 1 + dl)
    } else {
        // Long form (CAN-FD) — dl ≤ 62, safe truncation to u8
        let needed = off + 2 + dl;
        if buf.len() < needed {
            return Err(CodecResult::InvalidFrameSize);
        }
        buf[off] = 0x00;
        #[allow(clippy::cast_possible_truncation)]
        { buf[off + 1] = dl as u8; }
        buf[off + 2..off + 2 + dl].copy_from_slice(data);
        Ok(off + 2 + dl)
    }
}

/// Encode a first frame into `buf`.
///
/// Uses extended (32-bit) length encoding when `message_length > 4095`.
///
/// Returns the number of bytes written.
pub fn encode_first_frame(
    buf: &mut [u8],
    message_length: u32,
    data: &[u8],
    config: &CodecConfig,
) -> Result<usize, CodecResult> {
    let off = config.pci_offset;

    if message_length < 8 {
        return Err(CodecResult::InvalidMessageSize);
    }

    if message_length <= 4095 {
        // Normal FF: 2-byte header.
        // (message_length >> 8) & 0x0F fits in 4 bits → safe to truncate to u8.
        #[allow(clippy::cast_possible_truncation)]
        let high = 0x10u8 | ((message_length >> 8) as u8 & 0x0F);
        #[allow(clippy::cast_possible_truncation)]
        let low = (message_length & 0xFF) as u8;
        let needed = off + 2 + data.len();
        if buf.len() < needed {
            return Err(CodecResult::InvalidFrameSize);
        }
        buf[off] = high;
        buf[off + 1] = low;
        buf[off + 2..off + 2 + data.len()].copy_from_slice(data);
        Ok(off + 2 + data.len())
    } else {
        // Extended FF: 6-byte header (0x10 0x00 + 4-byte BE length)
        let needed = off + 6 + data.len();
        if buf.len() < needed {
            return Err(CodecResult::InvalidFrameSize);
        }
        buf[off] = 0x10;
        buf[off + 1] = 0x00;
        let len_bytes = message_length.to_be_bytes();
        buf[off + 2] = len_bytes[0];
        buf[off + 3] = len_bytes[1];
        buf[off + 4] = len_bytes[2];
        buf[off + 5] = len_bytes[3];
        buf[off + 6..off + 6 + data.len()].copy_from_slice(data);
        Ok(off + 6 + data.len())
    }
}

/// Encode a consecutive frame into `buf`.
///
/// The sequence number is masked to the lower 4 bits (wraps at 16).
///
/// Returns the number of bytes written.
pub fn encode_consecutive_frame(
    buf: &mut [u8],
    sequence_number: u8,
    data: &[u8],
    config: &CodecConfig,
) -> Result<usize, CodecResult> {
    let off = config.pci_offset;
    let needed = off + 1 + data.len();
    if buf.len() < needed {
        return Err(CodecResult::InvalidFrameSize);
    }
    buf[off] = 0x20 | (sequence_number & 0x0F);
    buf[off + 1..off + 1 + data.len()].copy_from_slice(data);
    Ok(off + 1 + data.len())
}

/// Encode a flow control frame into `buf`.
///
/// The buffer is padded with `config.filler_byte` up to its full length.
///
/// Returns the number of bytes written (always `buf.len()` if successful,
/// minimum 3 + `pci_offset`).
pub fn encode_flow_control(
    buf: &mut [u8],
    status: FlowStatus,
    block_size: u8,
    separation_time: u8,
    config: &CodecConfig,
) -> Result<usize, CodecResult> {
    let off = config.pci_offset;
    let needed = off + 3;
    if buf.len() < needed {
        return Err(CodecResult::InvalidFrameSize);
    }
    buf[off] = 0x30 | (status as u8);
    buf[off + 1] = block_size;
    buf[off + 2] = separation_time;
    // Pad remainder with filler byte
    for b in buf.iter_mut().skip(off + 3) {
        *b = config.filler_byte;
    }
    Ok(buf.len())
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_cfg() -> CodecConfig {
        CodecConfig::default()
    }

    // ── decode: single frame ──────────────────────────────────────────────────

    #[test]
    fn decode_single_frame_short() {
        let data = [0x03, 0xAA, 0xBB, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        assert_eq!(
            frame,
            DecodedFrame::Single(SingleFrame {
                data_length: 3,
                data: &[0xAA, 0xBB, 0xCC],
            })
        );
    }

    #[test]
    fn decode_single_frame_1byte() {
        let data = [0x01, 0x42];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::Single(sf) = frame {
            assert_eq!(sf.data_length, 1);
            assert_eq!(sf.data, &[0x42]);
        } else {
            panic!("Expected Single frame");
        }
    }

    #[test]
    fn decode_single_frame_long() {
        // Long SF (CAN-FD escape): 0x00 0x08 followed by 8 data bytes
        let mut data = [0u8; 10];
        data[0] = 0x00;
        data[1] = 8;
        data[2..10].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::Single(sf) = frame {
            assert_eq!(sf.data_length, 8);
            assert_eq!(sf.data, &[1, 2, 3, 4, 5, 6, 7, 8]);
        } else {
            panic!("Expected Single frame");
        }
    }

    // ── decode: first frame ───────────────────────────────────────────────────

    #[test]
    fn decode_first_frame_normal() {
        // FF normal: len = 0x012 = 18
        let data = [0x10, 0x12, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::First(ff) = frame {
            assert_eq!(ff.message_length, 18);
            assert_eq!(ff.data, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF]);
        } else {
            panic!("Expected First frame");
        }
    }

    #[test]
    fn decode_first_frame_extended() {
        // Extended FF: 0x10 0x00 + 4-byte big-endian length = 5000
        let len: u32 = 5000;
        let lb = len.to_be_bytes();
        let data = [0x10, 0x00, lb[0], lb[1], lb[2], lb[3], 0xAA, 0xBB];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::First(ff) = frame {
            assert_eq!(ff.message_length, 5000);
            assert_eq!(ff.data, &[0xAA, 0xBB]);
        } else {
            panic!("Expected First frame");
        }
    }

    // ── decode: consecutive frame ─────────────────────────────────────────────

    #[test]
    fn decode_consecutive_frame() {
        let data = [0x21, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::Consecutive(cf) = frame {
            assert_eq!(cf.sequence_number, 1);
            assert_eq!(cf.data, &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]);
        } else {
            panic!("Expected Consecutive frame");
        }
    }

    // ── decode: flow control ──────────────────────────────────────────────────

    #[test]
    fn decode_flow_control_cts() {
        let data = [0x30, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::FlowControl(fc) = frame {
            assert_eq!(fc.status, FlowStatus::ContinueToSend);
            assert_eq!(fc.block_size, 0);
            assert_eq!(fc.separation_time, 0);
        } else {
            panic!("Expected FlowControl frame");
        }
    }

    #[test]
    fn decode_flow_control_wait() {
        let data = [0x31, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::FlowControl(fc) = frame {
            assert_eq!(fc.status, FlowStatus::Wait);
        } else {
            panic!("Expected FlowControl frame");
        }
    }

    #[test]
    fn decode_flow_control_overflow() {
        let data = [0x32, 0x10, 0x25, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        let frame = decode_frame(&data, &default_cfg()).unwrap();
        if let DecodedFrame::FlowControl(fc) = frame {
            assert_eq!(fc.status, FlowStatus::Overflow);
            assert_eq!(fc.block_size, 0x10);
            assert_eq!(fc.separation_time, 0x25);
        } else {
            panic!("Expected FlowControl frame");
        }
    }

    // ── decode: error cases ───────────────────────────────────────────────────

    #[test]
    fn decode_empty_frame_error() {
        let data: [u8; 0] = [];
        assert_eq!(
            decode_frame(&data, &default_cfg()),
            Err(CodecResult::InvalidFrameSize)
        );
    }

    #[test]
    fn decode_invalid_type() {
        // Upper nibble 0xF → invalid
        let data = [0xF0, 0x00, 0x00];
        assert_eq!(
            decode_frame(&data, &default_cfg()),
            Err(CodecResult::InvalidFrameType)
        );
    }

    // ── encode: single frame ──────────────────────────────────────────────────

    #[test]
    fn encode_single_frame_1byte() {
        let mut buf = [0u8; 8];
        let n = encode_single_frame(&mut buf, &[0x42], &default_cfg()).unwrap();
        assert_eq!(n, 2);
        assert_eq!(buf[0], 0x01);
        assert_eq!(buf[1], 0x42);
    }

    #[test]
    fn encode_single_frame_7bytes() {
        let mut buf = [0u8; 8];
        let payload = [1u8, 2, 3, 4, 5, 6, 7];
        let n = encode_single_frame(&mut buf, &payload, &default_cfg()).unwrap();
        assert_eq!(n, 8);
        assert_eq!(buf[0], 0x07);
        assert_eq!(&buf[1..8], &payload);
    }

    // ── encode: first frame ───────────────────────────────────────────────────

    #[test]
    fn encode_first_frame_normal() {
        let mut buf = [0u8; 8];
        let payload = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let n = encode_first_frame(&mut buf, 18, &payload, &default_cfg()).unwrap();
        assert_eq!(n, 8);
        assert_eq!(buf[0], 0x10);
        assert_eq!(buf[1], 0x12);
        assert_eq!(&buf[2..8], &payload);
    }

    #[test]
    fn encode_first_frame_extended() {
        let mut buf = [0u8; 8];
        let payload = [0xAA, 0xBB];
        let n = encode_first_frame(&mut buf, 5000, &payload, &default_cfg()).unwrap();
        assert_eq!(n, 8);
        assert_eq!(buf[0], 0x10);
        assert_eq!(buf[1], 0x00);
        let encoded_len = u32::from_be_bytes([buf[2], buf[3], buf[4], buf[5]]);
        assert_eq!(encoded_len, 5000);
        assert_eq!(&buf[6..8], &payload);
    }

    // ── encode: consecutive frame ─────────────────────────────────────────────

    #[test]
    fn encode_consecutive_frame_seq0() {
        let mut buf = [0u8; 8];
        let payload = [1u8, 2, 3, 4, 5, 6, 7];
        let n = encode_consecutive_frame(&mut buf, 0, &payload, &default_cfg()).unwrap();
        assert_eq!(n, 8);
        assert_eq!(buf[0], 0x20);
        assert_eq!(&buf[1..8], &payload);
    }

    #[test]
    fn encode_consecutive_frame_seq15_wrap() {
        let mut buf = [0u8; 8];
        let payload = [0xAAu8; 7];
        // sequence_number = 15 → PCI = 0x2F
        encode_consecutive_frame(&mut buf, 15, &payload, &default_cfg()).unwrap();
        assert_eq!(buf[0], 0x2F);
        // sequence_number = 16 wraps to 0 (mask &0x0F)
        encode_consecutive_frame(&mut buf, 16, &payload, &default_cfg()).unwrap();
        assert_eq!(buf[0], 0x20);
        // sequence_number = 17 wraps to 1
        encode_consecutive_frame(&mut buf, 17, &payload, &default_cfg()).unwrap();
        assert_eq!(buf[0], 0x21);
    }

    // ── encode: flow control ──────────────────────────────────────────────────

    #[test]
    fn encode_flow_control_cts() {
        let mut buf = [0u8; 8];
        let n = encode_flow_control(&mut buf, FlowStatus::ContinueToSend, 0, 0, &default_cfg())
            .unwrap();
        assert_eq!(n, 8); // full buffer
        assert_eq!(buf[0], 0x30);
        assert_eq!(buf[1], 0x00);
        assert_eq!(buf[2], 0x00);
        // Padding
        assert!(buf[3..].iter().all(|&b| b == 0xCC));
    }

    #[test]
    fn encode_flow_control_wait() {
        let mut buf = [0u8; 8];
        encode_flow_control(&mut buf, FlowStatus::Wait, 0, 0, &default_cfg()).unwrap();
        assert_eq!(buf[0], 0x31);
    }

    // ── frame_count ───────────────────────────────────────────────────────────

    #[test]
    fn frame_count_single() {
        // 6 bytes fits in first frame (capacity 6) → 1 frame
        assert_eq!(frame_count(6, 6, 7), 1);
    }

    #[test]
    fn frame_count_two_frames() {
        // FF carries 6 bytes, one CF carries 7: total 13 → FF + 1 CF
        assert_eq!(frame_count(13, 6, 7), 2);
    }

    #[test]
    fn frame_count_many_frames() {
        // FF: 6, remaining 64: ceil(64/7) = 10 CFs → 11 total
        assert_eq!(frame_count(70, 6, 7), 11);
    }

    // ── roundtrip ─────────────────────────────────────────────────────────────

    #[test]
    fn encode_decode_roundtrip_sf() {
        let mut buf = [0xCC_u8; 8];
        let payload = [0x11u8, 0x22, 0x33, 0x44];
        encode_single_frame(&mut buf, &payload, &default_cfg()).unwrap();
        let frame = decode_frame(&buf, &default_cfg()).unwrap();
        if let DecodedFrame::Single(sf) = frame {
            assert_eq!(sf.data_length, 4);
            assert_eq!(sf.data, &payload);
        } else {
            panic!("Expected Single frame");
        }
    }

    #[test]
    fn encode_decode_roundtrip_ff_cf() {
        // Encode FF with 6 bytes payload for a 20-byte message
        let mut ff_buf = [0u8; 8];
        let ff_payload = [0xAA_u8; 6];
        encode_first_frame(&mut ff_buf, 20, &ff_payload, &default_cfg()).unwrap();
        let ff = decode_frame(&ff_buf, &default_cfg()).unwrap();
        if let DecodedFrame::First(f) = ff {
            assert_eq!(f.message_length, 20);
            assert_eq!(f.data, &ff_payload);
        } else {
            panic!("Expected First frame");
        }

        // Encode CF with sequence 1
        let mut cf_buf = [0u8; 8];
        let cf_payload = [0xBB_u8; 7];
        encode_consecutive_frame(&mut cf_buf, 1, &cf_payload, &default_cfg()).unwrap();
        let cf = decode_frame(&cf_buf, &default_cfg()).unwrap();
        if let DecodedFrame::Consecutive(c) = cf {
            assert_eq!(c.sequence_number, 1);
            assert_eq!(c.data, &cf_payload);
        } else {
            panic!("Expected Consecutive frame");
        }
    }
}
