//! Integration tests for ISO-TP transport layer (bsw-docan).
//!
//! Covers:
//! - ISO-TP segmentation: single frame, first frame, consecutive frames
//! - Flow control handling: CTS, Wait, Overflow
//! - Timeout behavior for RX and TX sessions
//! - End-to-end multiframe assembly and disassembly
//! - Block size enforcement and sequence wrapping

use bsw_docan::codec::{
    decode_frame, decode_frame_type, encode_consecutive_frame, encode_first_frame,
    encode_flow_control, encode_single_frame, frame_count, CodecConfig, DecodedFrame,
};
use bsw_docan::constants::{CodecResult, FlowStatus, FrameType, ProtocolMessage};
use bsw_docan::parameters::{decode_separation_time, encode_separation_time, Parameters};
use bsw_docan::rx_handler::{RxProtocolHandler, RxState, RxTimeout};
use bsw_docan::tx_handler::{TxProtocolHandler, TxState, TxTimeout};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. ISO-TP SEGMENTATION — CODEC
// ═══════════════════════════════════════════════════════════════════════════════

mod codec_segmentation {
    use super::*;

    fn cfg() -> CodecConfig {
        CodecConfig::default()
    }

    // -- Single Frame encode/decode for every valid length (1..7) --

    #[test]
    fn single_frame_all_valid_lengths() {
        for len in 1u8..=7 {
            let payload: Vec<u8> = (0..len).collect();
            let mut buf = [0xCC_u8; 8];
            let n = encode_single_frame(&mut buf, &payload, &cfg()).unwrap();
            assert_eq!(n, 1 + len as usize);
            assert_eq!(buf[0] & 0x0F, len);

            let frame = decode_frame(&buf, &cfg()).unwrap();
            if let DecodedFrame::Single(sf) = frame {
                assert_eq!(sf.data_length, u16::from(len));
                assert_eq!(sf.data, &payload[..]);
            } else {
                panic!("Expected SingleFrame for length {len}");
            }
        }
    }

    // -- Frame type detection --

    #[test]
    fn frame_type_detection_all_types() {
        assert_eq!(decode_frame_type(&[0x05, 0, 0], &cfg()), Ok(FrameType::Single));
        assert_eq!(decode_frame_type(&[0x10, 0x0A, 0, 0, 0, 0, 0, 0], &cfg()), Ok(FrameType::First));
        assert_eq!(decode_frame_type(&[0x21, 0, 0], &cfg()), Ok(FrameType::Consecutive));
        assert_eq!(decode_frame_type(&[0x30, 0, 0], &cfg()), Ok(FrameType::FlowControl));
        assert_eq!(decode_frame_type(&[0x40], &cfg()), Err(CodecResult::InvalidFrameType));
        assert_eq!(decode_frame_type(&[0xF0], &cfg()), Err(CodecResult::InvalidFrameType));
    }

    // -- First Frame normal (12-bit length) encode/decode --

    #[test]
    fn first_frame_12bit_length_boundary() {
        let mut buf = [0u8; 8];

        // Minimum valid FF length = 8
        let payload = [0xAA; 6];
        let n = encode_first_frame(&mut buf, 8, &payload, &cfg()).unwrap();
        assert_eq!(n, 8);
        if let DecodedFrame::First(ff) = decode_frame(&buf, &cfg()).unwrap() {
            assert_eq!(ff.message_length, 8);
        } else {
            panic!("Expected FirstFrame");
        }

        // Maximum 12-bit length = 4095
        let n = encode_first_frame(&mut buf, 4095, &payload, &cfg()).unwrap();
        assert_eq!(n, 8);
        if let DecodedFrame::First(ff) = decode_frame(&buf, &cfg()).unwrap() {
            assert_eq!(ff.message_length, 4095);
        } else {
            panic!("Expected FirstFrame");
        }
    }

    // -- First Frame extended (32-bit length) encode/decode --

    #[test]
    fn first_frame_extended_32bit_length() {
        let mut buf = [0u8; 8];
        let payload = [0xBB; 2];

        // Minimum extended FF: 4096
        let n = encode_first_frame(&mut buf, 4096, &payload, &cfg()).unwrap();
        assert_eq!(n, 8);
        if let DecodedFrame::First(ff) = decode_frame(&buf, &cfg()).unwrap() {
            assert_eq!(ff.message_length, 4096);
        } else {
            panic!("Expected FirstFrame");
        }

        // Large message
        let n = encode_first_frame(&mut buf, 100_000, &payload, &cfg()).unwrap();
        assert_eq!(n, 8);
        if let DecodedFrame::First(ff) = decode_frame(&buf, &cfg()).unwrap() {
            assert_eq!(ff.message_length, 100_000);
        } else {
            panic!("Expected FirstFrame");
        }
    }

    // -- Consecutive Frame sequence number wrapping --

    #[test]
    fn consecutive_frame_sequence_wrapping() {
        let payload = [0xDD; 7];
        let mut buf = [0u8; 8];

        for seq in 0u8..=20 {
            encode_consecutive_frame(&mut buf, seq, &payload, &cfg()).unwrap();
            if let DecodedFrame::Consecutive(cf) = decode_frame(&buf, &cfg()).unwrap() {
                assert_eq!(cf.sequence_number, seq & 0x0F);
            } else {
                panic!("Expected ConsecutiveFrame for seq {seq}");
            }
        }
    }

    // -- Flow Control all statuses --

    #[test]
    fn flow_control_encode_decode_all_statuses() {
        let mut buf = [0u8; 8];
        let statuses = [
            (FlowStatus::ContinueToSend, 0x30),
            (FlowStatus::Wait, 0x31),
            (FlowStatus::Overflow, 0x32),
        ];
        for (status, expected_pci) in statuses {
            encode_flow_control(&mut buf, status, 5, 0x14, &cfg()).unwrap();
            assert_eq!(buf[0], expected_pci);
            assert_eq!(buf[1], 5);
            assert_eq!(buf[2], 0x14);
            // Padding
            assert!(buf[3..].iter().all(|&b| b == 0xCC));
        }
    }

    // -- Frame count calculations --

    #[test]
    fn frame_count_edge_cases() {
        // Message fits in single frame
        assert_eq!(frame_count(1, 6, 7), 1);
        assert_eq!(frame_count(6, 6, 7), 1);
        // Exactly 1 CF needed
        assert_eq!(frame_count(7, 6, 7), 2);
        // FF(6) + 14 remaining: ceil(14/7) = 2 CFs → total 3
        assert_eq!(frame_count(20, 6, 7), 3);
        // Large message: FF(6) + ceil(4089/7) = 585 CFs → 586 total
        assert_eq!(frame_count(4095, 6, 7), 585 + 1);
    }

    // -- Error cases --

    #[test]
    fn encode_single_frame_empty_data_rejected() {
        let mut buf = [0u8; 8];
        assert_eq!(
            encode_single_frame(&mut buf, &[], &cfg()),
            Err(CodecResult::InvalidMessageSize)
        );
    }

    #[test]
    fn encode_first_frame_too_small_message_rejected() {
        let mut buf = [0u8; 8];
        assert_eq!(
            encode_first_frame(&mut buf, 7, &[0u8; 6], &cfg()),
            Err(CodecResult::InvalidMessageSize)
        );
    }

    #[test]
    fn decode_frame_too_short_for_flow_control() {
        // FC needs 3 bytes minimum
        let data = [0x30, 0x00];
        assert_eq!(decode_frame(&data, &cfg()), Err(CodecResult::InvalidFrameSize));
    }

    #[test]
    fn decode_frame_invalid_flow_status() {
        // FC status nibble 0x3 (0x33) is invalid
        let data = [0x33, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC];
        assert_eq!(decode_frame(&data, &cfg()), Err(CodecResult::InvalidFrameType));
    }

    // -- PCI offset (normal addressing) --

    #[test]
    fn pci_offset_shifts_frame_encoding() {
        let cfg_offset = CodecConfig { pci_offset: 1, filler_byte: 0xAA };
        let mut buf = [0u8; 8];
        let payload = [0x11, 0x22, 0x33];
        let n = encode_single_frame(&mut buf, &payload, &cfg_offset).unwrap();
        assert_eq!(n, 5); // offset(1) + PCI(1) + 3
        assert_eq!(buf[1], 0x03); // PCI at offset 1
        assert_eq!(&buf[2..5], &payload);
    }

    // -- Custom filler byte --

    #[test]
    fn custom_filler_byte_in_flow_control() {
        let cfg_custom = CodecConfig { pci_offset: 0, filler_byte: 0x55 };
        let mut buf = [0u8; 8];
        encode_flow_control(&mut buf, FlowStatus::ContinueToSend, 0, 0, &cfg_custom).unwrap();
        assert!(buf[3..].iter().all(|&b| b == 0x55));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. MULTIFRAME MESSAGE ASSEMBLY/DISASSEMBLY
// ═══════════════════════════════════════════════════════════════════════════════

mod multiframe_assembly {
    use super::*;

    /// Simulate a complete multiframe TX→RX transfer of a message.
    /// Encodes FF + N CFs and verifies reassembly produces the original payload.
    #[test]
    fn reassemble_20_byte_message() {
        let cfg = CodecConfig::default();
        let original: Vec<u8> = (0u8..20).collect();

        // Encode first frame (6 data bytes)
        let mut ff_buf = [0u8; 8];
        encode_first_frame(&mut ff_buf, 20, &original[..6], &cfg).unwrap();

        // Decode FF to get message length
        let ff = decode_frame(&ff_buf, &cfg).unwrap();
        let msg_len = if let DecodedFrame::First(f) = &ff {
            f.message_length
        } else {
            panic!("Expected FirstFrame");
        };
        assert_eq!(msg_len, 20);

        // Reassembly buffer
        let mut assembled = Vec::new();
        if let DecodedFrame::First(f) = &ff {
            assembled.extend_from_slice(f.data);
        }

        // Encode and decode consecutive frames (7 data bytes each)
        let mut seq = 1u8;
        let mut offset = 6usize;
        while offset < 20 {
            let end = core::cmp::min(offset + 7, 20);
            let cf_data = &original[offset..end];
            let mut cf_buf = [0u8; 8];
            encode_consecutive_frame(&mut cf_buf, seq, cf_data, &cfg).unwrap();

            let decoded = decode_frame(&cf_buf, &cfg).unwrap();
            if let DecodedFrame::Consecutive(cf) = decoded {
                assert_eq!(cf.sequence_number, seq & 0x0F);
                assembled.extend_from_slice(cf.data);
            } else {
                panic!("Expected ConsecutiveFrame at offset {offset}");
            }

            seq += 1;
            offset = end;
        }

        // Verify full reassembly
        assert_eq!(&assembled[..20], &original[..]);
    }

    /// Simulate reassembly of a message spanning >16 CFs (sequence number wraps).
    #[test]
    fn reassemble_message_with_sequence_wrap() {
        let cfg = CodecConfig::default();
        // 125 bytes: FF(6) + ceil(119/7) = 17 CFs → sequence wraps from 0xF→0x0
        let msg_len = 125u32;
        let original: Vec<u8> = (0..msg_len as u8).collect();

        let expected_cfs = frame_count(msg_len, 6, 7) - 1; // subtract FF
        assert!(expected_cfs >= 16, "Need >=16 CFs to test sequence wrap");

        let mut assembled = Vec::new();
        assembled.extend_from_slice(&original[..6]); // FF payload

        let mut seq = 1u8;
        let mut offset = 6usize;
        while offset < msg_len as usize {
            let end = core::cmp::min(offset + 7, msg_len as usize);
            let cf_data = &original[offset..end];
            let mut cf_buf = [0u8; 8];
            encode_consecutive_frame(&mut cf_buf, seq, cf_data, &cfg).unwrap();

            if let DecodedFrame::Consecutive(cf) = decode_frame(&cf_buf, &cfg).unwrap() {
                assert_eq!(cf.sequence_number, seq & 0x0F);
                assembled.extend_from_slice(cf.data);
            }

            seq = seq.wrapping_add(1);
            offset = end;
        }

        assert_eq!(&assembled[..msg_len as usize], &original[..]);
    }

    /// Verify frame_count matches actual CFs needed for various message sizes.
    #[test]
    fn frame_count_matches_actual_encoding() {
        let test_sizes = [8, 13, 20, 50, 100, 125, 4095];
        for &size in &test_sizes {
            let expected = frame_count(size, 6, 7);
            // Manual calculation: 1 FF + ceil((size - 6) / 7) CFs
            let cfs = if size <= 6 {
                0
            } else {
                ((size as usize - 6) + 6) / 7 // ceil division
            };
            let manual = if size <= 6 { 1 } else { 1 + cfs as u32 };
            assert_eq!(
                expected, manual,
                "frame_count mismatch for size {size}: got {expected}, expected {manual}"
            );
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. RX HANDLER — FLOW CONTROL AND TIMEOUTS
// ═══════════════════════════════════════════════════════════════════════════════

mod rx_handler_integration {
    use super::*;

    /// Full lifecycle: single frame allocation → processing → done.
    #[test]
    fn single_frame_lifecycle() {
        let mut h = RxProtocolHandler::new_single();
        assert_eq!(h.state(), RxState::Allocate);
        assert!(!h.is_multi_frame());
        assert_eq!(h.timeout(), RxTimeout::Allocate);

        let t = h.allocated(true);
        assert!(t.state_changed);
        assert!(!t.send_flow_control);
        assert_eq!(h.state(), RxState::Processing);
        assert_eq!(h.timeout(), RxTimeout::None);

        let t = h.processed(true);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Done);
    }

    /// Full lifecycle: multi-frame with 5 CFs, unlimited block.
    #[test]
    fn multi_frame_full_lifecycle_unlimited_block() {
        let mut h = RxProtocolHandler::new_multi(5, 0, 3);
        assert!(h.is_multi_frame());

        // Allocate → Send (CTS FC)
        let t = h.allocated(true);
        assert!(t.send_flow_control);
        assert!(!t.flow_control_wait);
        assert_eq!(h.state(), RxState::Send);

        // FC sent → Wait
        h.frame_sent(true);
        assert_eq!(h.state(), RxState::Wait);
        assert_eq!(h.timeout(), RxTimeout::Rx);

        // Receive 4 CFs → still waiting
        for i in 0..4 {
            let t = h.consecutive_frame_received();
            assert!(!t.state_changed, "CF {i} should not change state");
            assert_eq!(h.frame_index(), (i + 1) as u32);
        }

        // 5th CF → Processing
        let t = h.consecutive_frame_received();
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Processing);

        // Process → Done
        let t = h.processed(true);
        assert_eq!(h.state(), RxState::Done);
        assert!(t.state_changed);
    }

    /// Multi-frame with block_size=3 and 7 CFs — verify FC is sent at each block boundary.
    #[test]
    fn block_size_boundary_sends_multiple_fcs() {
        let mut h = RxProtocolHandler::new_multi(7, 3, 3);

        h.allocated(true);
        h.frame_sent(true);

        // Block 1: 3 CFs
        h.consecutive_frame_received();
        h.consecutive_frame_received();
        let t = h.consecutive_frame_received();
        assert_eq!(h.state(), RxState::Send);
        assert!(t.send_flow_control);

        h.frame_sent(true);

        // Block 2: 3 CFs
        h.consecutive_frame_received();
        h.consecutive_frame_received();
        let t = h.consecutive_frame_received();
        assert_eq!(h.state(), RxState::Send);
        assert!(t.send_flow_control);

        h.frame_sent(true);

        // Block 3: 1 CF (last)
        let t = h.consecutive_frame_received();
        assert_eq!(h.state(), RxState::Processing);
        assert!(!t.send_flow_control);
    }

    /// Allocation failure with retries sends WAIT FC, then succeeds.
    #[test]
    fn allocation_retry_with_wait_fc_then_success() {
        let mut h = RxProtocolHandler::new_multi(3, 0, 2);

        // First alloc failure → WAIT FC
        let t = h.allocated(false);
        assert!(!t.state_changed);
        assert!(t.send_flow_control);
        assert!(t.flow_control_wait);
        assert_eq!(h.state(), RxState::Allocate);

        // Timer expires → retry
        h.expired();

        // Second alloc failure → still WAIT FC
        let t = h.allocated(false);
        assert!(!t.state_changed);
        assert!(t.send_flow_control);
        assert!(t.flow_control_wait);

        // Timer expires → retry
        h.expired();

        // Third alloc failure → Fail (exceeded max_allocate_retry=2)
        let t = h.allocated(false);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::AllocationFailed);
    }

    /// RX timeout during Wait state → Fail.
    #[test]
    fn rx_timeout_during_wait_fails() {
        let mut h = RxProtocolHandler::new_multi(5, 0, 3);
        h.allocated(true);
        h.frame_sent(true);
        assert_eq!(h.state(), RxState::Wait);
        assert_eq!(h.timeout(), RxTimeout::Rx);

        // Partial reception
        h.consecutive_frame_received();
        h.consecutive_frame_received();

        // Timeout
        let t = h.expired();
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
    }

    /// FC send failure → Fail.
    #[test]
    fn fc_send_failure_transitions_to_fail() {
        let mut h = RxProtocolHandler::new_multi(3, 0, 3);
        h.allocated(true);
        assert_eq!(h.state(), RxState::Send);

        let t = h.frame_sent(false);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::SendFailed);
    }

    /// Processing failure → Fail.
    #[test]
    fn processing_failure_transitions_to_fail() {
        let mut h = RxProtocolHandler::new_single();
        h.allocated(true);
        assert_eq!(h.state(), RxState::Processing);

        let t = h.processed(false);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
    }

    /// Operations on terminal states are no-ops.
    #[test]
    fn operations_on_terminal_states_are_noop() {
        let mut h = RxProtocolHandler::new_single();
        h.allocated(true);
        h.processed(true);
        assert_eq!(h.state(), RxState::Done);

        // All operations should be no-ops
        let t = h.allocated(true);
        assert!(!t.state_changed);
        let t = h.consecutive_frame_received();
        assert!(!t.state_changed);
        let t = h.frame_sent(true);
        assert!(!t.state_changed);
        let t = h.processed(true);
        assert!(!t.state_changed);
        let t = h.expired();
        assert!(!t.state_changed);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. TX HANDLER — FLOW CONTROL AND TIMEOUTS
// ═══════════════════════════════════════════════════════════════════════════════

mod tx_handler_integration {
    use super::*;

    /// Single frame TX: Initialized → Send → Success.
    #[test]
    fn single_frame_tx_lifecycle() {
        let mut h = TxProtocolHandler::new(1, 10);
        assert_eq!(h.state(), TxState::Initialized);
        assert!(!h.is_complete());

        let t = h.start();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Send);
        assert_eq!(h.timeout(), TxTimeout::TxCallback);

        h.frame_sending();
        assert_eq!(h.frame_index(), 1);

        let t = h.frames_sent();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Success);
        assert!(h.is_complete());
    }

    /// Multi-frame TX with unlimited block: FF → FC(CTS) → CFs → Success.
    #[test]
    fn multi_frame_tx_unlimited_block() {
        let mut h = TxProtocolHandler::new(4, 10); // FF + 3 CFs
        h.start();

        // Send FF
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Wait);
        assert_eq!(h.timeout(), TxTimeout::FlowControl);

        // Receive CTS with block_size=0 (unlimited)
        let t = h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Send);
        assert!(t.actions.store_separation_time);
        assert_eq!(h.timeout(), TxTimeout::SeparationTime);

        // Send CF1
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Send); // unlimited, no FC needed

        // Send CF2
        h.frame_sending();
        h.frames_sent();

        // Send CF3 (last)
        h.frame_sending();
        let t = h.frames_sent();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Success);
    }

    /// Multi-frame TX with block_size=2: verify FC required at block boundaries.
    #[test]
    fn multi_frame_tx_block_size_boundary() {
        // 5 frames: FF + 4 CFs, block_size=2
        let mut h = TxProtocolHandler::new(5, 10);
        h.start();

        // FF
        h.frame_sending();
        h.frames_sent(); // → Wait for initial FC

        // CTS, block_size=2 → block_end = 1 + 2 = 3
        h.handle_flow_control(FlowStatus::ContinueToSend, 2);

        // CF1 (index=2)
        h.frame_sending();
        h.frames_sent(); // index=2 < block_end=3 → Send

        // CF2 (index=3) → block boundary → Wait
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Wait);

        // Second CTS, block_size=2 → block_end = 3 + 2 = 5
        h.handle_flow_control(FlowStatus::ContinueToSend, 2);

        // CF3 (index=4)
        h.frame_sending();
        h.frames_sent();

        // CF4 (index=5) → Success (frame_count reached)
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Success);
    }

    /// Flow Control WAIT: allow up to max_wait_count, then fail.
    #[test]
    fn flow_control_wait_count_enforcement() {
        let mut h = TxProtocolHandler::new(3, 3); // max 3 WAITs
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        // 2 WAITs: still alive
        for _ in 0..2 {
            let t = h.handle_flow_control(FlowStatus::Wait, 0);
            assert!(!t.state_changed);
            assert_eq!(h.state(), TxState::Wait);
        }

        // 3rd WAIT: exceeds max → Fail
        let t = h.handle_flow_control(FlowStatus::Wait, 0);
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::WaitCountExceeded);
        assert!(t.actions.cancel_pending_send);
    }

    /// Flow Control CTS resets wait counter.
    #[test]
    fn cts_resets_wait_counter() {
        let mut h = TxProtocolHandler::new(5, 3);
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        // 2 WAITs
        h.handle_flow_control(FlowStatus::Wait, 0);
        h.handle_flow_control(FlowStatus::Wait, 0);

        // CTS resets wait counter
        h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert_eq!(h.state(), TxState::Send);

        // Send more frames, get FC again
        h.frame_sending();
        h.frames_sent();
        h.frame_sending();
        h.frames_sent();
        h.frame_sending();
        h.frames_sent();
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Success);
    }

    /// Flow Control Overflow → Fail immediately.
    #[test]
    fn flow_control_overflow_immediate_fail() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        let t = h.handle_flow_control(FlowStatus::Overflow, 0);
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Overflow);
        assert!(t.actions.cancel_pending_send);
    }

    /// TX callback timeout → Fail.
    #[test]
    fn tx_callback_timeout_fails() {
        let mut h = TxProtocolHandler::new(1, 10);
        h.start();
        assert_eq!(h.timeout(), TxTimeout::TxCallback);

        let t = h.expired();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
        assert!(t.actions.cancel_pending_send);
    }

    /// FC timeout during Wait → Fail.
    #[test]
    fn flow_control_timeout_fails() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Wait);
        assert_eq!(h.timeout(), TxTimeout::FlowControl);

        let t = h.expired();
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
    }

    /// FC on non-Wait state returns InvalidFlowControl.
    #[test]
    fn flow_control_in_wrong_state() {
        let mut h = TxProtocolHandler::new(1, 10);
        h.start(); // state = Send

        let t = h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert!(!t.state_changed);
        assert_eq!(t.message, ProtocolMessage::InvalidFlowControl);
    }

    /// Expired on completed session is a no-op.
    #[test]
    fn expired_on_complete_is_noop() {
        let mut h = TxProtocolHandler::new(1, 10);
        h.start();
        h.frame_sending();
        h.frames_sent();
        assert_eq!(h.state(), TxState::Success);

        let t = h.expired();
        assert!(!t.state_changed);
        assert_eq!(h.state(), TxState::Success);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. SEPARATION TIME PARAMETERS
// ═══════════════════════════════════════════════════════════════════════════════

mod separation_time_params {
    use super::*;

    #[test]
    fn encode_decode_all_ms_values() {
        for ms in 0u32..=127 {
            let encoded = encode_separation_time(ms * 1_000);
            let decoded = decode_separation_time(encoded);
            assert_eq!(decoded, ms * 1_000, "Failed for {ms} ms");
        }
    }

    #[test]
    fn encode_decode_sub_ms_values() {
        for step in 1u32..=9 {
            let us = step * 100;
            let encoded = encode_separation_time(us);
            let decoded = decode_separation_time(encoded);
            assert_eq!(decoded, us, "Failed for {us} µs");
            // Verify encoded byte is in F1-F9 range
            assert!(encoded >= 0xF1 && encoded <= 0xF9);
        }
    }

    #[test]
    fn encode_clamps_above_127ms() {
        assert_eq!(encode_separation_time(128_000), 0x7F);
        assert_eq!(encode_separation_time(200_000), 0x7F);
        assert_eq!(encode_separation_time(u32::MAX), 0x7F);
    }

    #[test]
    fn decode_reserved_ranges_return_max() {
        for byte in 0x80..=0xF0 {
            assert_eq!(decode_separation_time(byte), 127_000, "Reserved byte 0x{byte:02X}");
        }
        for byte in 0xFA..=0xFF {
            assert_eq!(decode_separation_time(byte), 127_000, "Reserved byte 0x{byte:02X}");
        }
    }

    #[test]
    fn default_parameters_are_sane() {
        let p = Parameters::default();
        assert!(p.wait_rx_timeout_us > 0);
        assert!(p.wait_flow_control_timeout_us > 0);
        assert!(p.wait_tx_callback_timeout_us > 0);
        assert!(p.wait_allocate_timeout_us > 0);
        // max_block_size=0 means unlimited
        assert_eq!(p.max_block_size, 0);
    }
}
