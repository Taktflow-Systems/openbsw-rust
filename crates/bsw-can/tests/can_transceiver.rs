//! Integration tests for CAN transceiver, frame, and filter contracts.
//!
//! Covers:
//! - Transceiver state machine transitions (valid and invalid)
//! - Filter accept/reject behavior
//! - Frame construction, payload, ID, and equality semantics
//! - Listener slot management and capacity enforcement
//! - Statistics tracking

use bsw_can::can_id::CanId;
use bsw_can::filter::{BitFieldFilter, Filter, IntervalFilter};
use bsw_can::frame::CanFrame;
use bsw_can::transceiver::{
    AbstractTransceiver, ErrorCode, State, TransceiverState,
    BAUDRATE_HIGHSPEED, BAUDRATE_LOWSPEED,
};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. TRANSCEIVER STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════════

mod state_machine {
    use super::*;

    type Trx = AbstractTransceiver<8>;

    /// Full valid lifecycle: Closed → Initialized → Open → Muted → Open → Closed.
    #[test]
    fn full_lifecycle() {
        let mut t = Trx::new(0);
        assert_eq!(t.state(), State::Closed);

        assert_eq!(t.transition_to_initialized(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Initialized);

        assert_eq!(t.transition_to_open(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Open);

        assert_eq!(t.transition_to_muted(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Muted);

        assert_eq!(t.transition_to_open(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Open);

        assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
        assert_eq!(t.state(), State::Closed);
    }

    /// All invalid transitions return IllegalState.
    #[test]
    fn invalid_transitions() {
        let mut t = Trx::new(0);

        // Closed: can't go to Open or Muted directly
        assert_eq!(t.transition_to_open(), ErrorCode::IllegalState);
        assert_eq!(t.transition_to_muted(), ErrorCode::IllegalState);

        // Initialized: can't init again, can't mute
        t.transition_to_initialized();
        assert_eq!(t.transition_to_initialized(), ErrorCode::IllegalState);
        assert_eq!(t.transition_to_muted(), ErrorCode::IllegalState);

        // Open: can't init
        t.transition_to_open();
        assert_eq!(t.transition_to_initialized(), ErrorCode::IllegalState);

        // Muted: can't init, can't mute again
        t.transition_to_muted();
        assert_eq!(t.transition_to_initialized(), ErrorCode::IllegalState);
        assert_eq!(t.transition_to_muted(), ErrorCode::IllegalState);
    }

    /// transition_to_closed works from every state.
    #[test]
    fn close_from_any_state() {
        for initial in [State::Closed, State::Initialized, State::Open, State::Muted] {
            let mut t = Trx::new(0);
            t.set_state(initial);
            assert_eq!(t.transition_to_closed(), ErrorCode::Ok);
            assert_eq!(t.state(), State::Closed);
        }
    }

    /// is_in_state correctly identifies current state.
    #[test]
    fn is_in_state_all_states() {
        let all_states = [State::Closed, State::Initialized, State::Waking, State::Open, State::Muted];
        for &current in &all_states {
            let mut t = Trx::new(0);
            t.set_state(current);
            for &check in &all_states {
                if check == current {
                    assert!(t.is_in_state(check));
                } else {
                    assert!(!t.is_in_state(check));
                }
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. CAN FRAME TX/RX CONTRACTS
// ═══════════════════════════════════════════════════════════════════════════════

mod frame_contracts {
    use super::*;

    /// Frame with data: payload is correctly stored and retrieved.
    #[test]
    fn frame_with_data_roundtrip() {
        let data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let frame = CanFrame::with_data(CanId::base(0x100), &data);
        assert_eq!(frame.payload_length(), 8);
        assert_eq!(frame.payload(), &data);
        assert_eq!(frame.id(), CanId::base(0x100));
    }

    /// Empty frame has zero payload length.
    #[test]
    fn empty_frame_has_zero_length() {
        let frame = CanFrame::new();
        assert_eq!(frame.payload_length(), 0);
        assert_eq!(frame.id(), CanId::base(0));
    }

    /// Frame payload can be mutated in-place.
    #[test]
    fn frame_payload_mutation() {
        let mut frame = CanFrame::with_data(CanId::base(0x200), &[0; 8]);
        frame.payload_mut()[0] = 0xAA;
        frame.payload_mut()[7] = 0xBB;
        assert_eq!(frame.payload()[0], 0xAA);
        assert_eq!(frame.payload()[7], 0xBB);
    }

    /// Timestamp is not considered in equality comparison.
    #[test]
    fn timestamp_ignored_in_equality() {
        let data = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut f1 = CanFrame::with_data(CanId::base(0x100), &data);
        let mut f2 = CanFrame::with_data(CanId::base(0x100), &data);
        f1.set_timestamp(1000);
        f2.set_timestamp(2000);
        assert_eq!(f1, f2);
    }

    /// Frames with different IDs are not equal.
    #[test]
    fn different_ids_not_equal() {
        let data = [1, 2, 3];
        let f1 = CanFrame::with_data(CanId::base(0x100), &data);
        let f2 = CanFrame::with_data(CanId::base(0x200), &data);
        assert_ne!(f1, f2);
    }

    /// Frames with different payload lengths are not equal.
    #[test]
    fn different_lengths_not_equal() {
        let f1 = CanFrame::with_data(CanId::base(0x100), &[1, 2, 3]);
        let f2 = CanFrame::with_data(CanId::base(0x100), &[1, 2, 3, 4]);
        assert_ne!(f1, f2);
    }

    /// Frames with different payloads are not equal.
    #[test]
    fn different_payloads_not_equal() {
        let f1 = CanFrame::with_data(CanId::base(0x100), &[1, 2, 3]);
        let f2 = CanFrame::with_data(CanId::base(0x100), &[1, 2, 4]);
        assert_ne!(f1, f2);
    }

    /// Index access (u8 index) works correctly.
    #[test]
    fn byte_index_access() {
        let data = [0x10, 0x20, 0x30, 0x40];
        let frame = CanFrame::with_data(CanId::base(0x100), &data);
        assert_eq!(frame[0u8], 0x10);
        assert_eq!(frame[3u8], 0x40);
    }

    /// set_payload / set_payload_length work correctly.
    #[test]
    fn set_payload_and_length() {
        let mut frame = CanFrame::new();
        let data = [0xAA, 0xBB, 0xCC];
        frame.set_payload(&data);
        frame.set_payload_length(3);
        assert_eq!(frame.payload_length(), 3);
        assert_eq!(&frame.payload()[..3], &data);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. CAN ID ENCODING
// ═══════════════════════════════════════════════════════════════════════════════

mod can_id_encoding {
    use super::*;

    #[test]
    fn base_id_properties() {
        let id = CanId::base(0x123);
        assert!(id.is_base());
        assert!(!id.is_extended());
        assert!(id.is_valid());
        assert_eq!(id.raw_id(), 0x123);
    }

    #[test]
    fn extended_id_properties() {
        let id = CanId::extended(0x1234_5678);
        assert!(!id.is_base());
        assert!(id.is_extended());
        assert!(id.is_valid());
        assert_eq!(id.raw_id(), 0x1234_5678);
    }

    #[test]
    fn base_id_max_is_0x7ff() {
        let id = CanId::base(0x7FF);
        assert!(id.is_valid());
        assert_eq!(id.raw_id(), 0x7FF);
    }

    #[test]
    fn extended_id_max_is_29bit() {
        let id = CanId::extended(0x1FFF_FFFF);
        assert!(id.is_valid());
        assert_eq!(id.raw_id(), 0x1FFF_FFFF);
    }

    #[test]
    fn id_equality() {
        assert_eq!(CanId::base(0x100), CanId::base(0x100));
        assert_ne!(CanId::base(0x100), CanId::base(0x200));
        assert_ne!(CanId::base(0x100), CanId::extended(0x100));
    }

    #[test]
    fn id_ordering() {
        assert!(CanId::base(0x100) < CanId::base(0x200));
        assert!(CanId::base(0x7FF) > CanId::base(0x000));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. RECEIVE FILTER
// ═══════════════════════════════════════════════════════════════════════════════

mod receive_filter {
    use super::*;

    #[test]
    fn bitfield_filter_default_rejects_all() {
        let f = BitFieldFilter::new();
        for id in [0x000, 0x100, 0x7FF] {
            assert!(!f.matches(id));
        }
    }

    #[test]
    fn bitfield_filter_add_and_match() {
        let mut f = BitFieldFilter::new();
        f.add(0x100);
        f.add(0x200);
        assert!(f.matches(0x100));
        assert!(f.matches(0x200));
        assert!(!f.matches(0x300));
    }

    #[test]
    fn bitfield_filter_add_range() {
        let mut f = BitFieldFilter::new();
        f.add_range(0x100, 0x105);
        for id in 0x100..=0x105 {
            assert!(f.matches(id), "Expected {id:#X} to match");
        }
        assert!(!f.matches(0x0FF));
        assert!(!f.matches(0x106));
    }

    #[test]
    fn bitfield_filter_open_accepts_all() {
        let mut f = BitFieldFilter::new();
        f.open();
        for id in [0x000, 0x100, 0x7FF] {
            assert!(f.matches(id));
        }
    }

    #[test]
    fn bitfield_filter_clear_rejects_all() {
        let mut f = BitFieldFilter::new();
        f.add(0x100);
        f.clear();
        assert!(!f.matches(0x100));
    }

    #[test]
    fn interval_filter_empty_rejects_all() {
        let f = IntervalFilter::new();
        assert!(!f.matches(0));
        assert!(!f.matches(0x100));
        assert!(!f.matches(u32::MAX));
    }

    #[test]
    fn interval_filter_with_range() {
        let f = IntervalFilter::with_range(0x600, 0x6FF);
        assert!(f.matches(0x600));
        assert!(f.matches(0x650));
        assert!(f.matches(0x6FF));
        assert!(!f.matches(0x5FF));
        assert!(!f.matches(0x700));
    }

    #[test]
    fn interval_filter_add_extends_bounds() {
        let mut f = IntervalFilter::new();
        f.add(0x200);
        assert!(f.matches(0x200));
        f.add(0x100); // extends lower bound
        assert!(f.matches(0x100));
        assert!(f.matches(0x150)); // between bounds
    }

    #[test]
    fn filter_merge_combines_two_bitfield_filters() {
        let mut f1 = BitFieldFilter::new();
        f1.add(0x100);
        // merge_into copies self into target
        let mut f2 = BitFieldFilter::new();
        f2.add(0x200);
        f2.merge_into(&mut f1);
        assert!(f1.matches(0x100));
        assert!(f1.matches(0x200));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. LISTENER SLOT MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

mod listener_management {
    use super::*;

    #[test]
    fn listener_capacity_enforcement() {
        let mut t: AbstractTransceiver<4> = AbstractTransceiver::new(0);
        assert_eq!(t.listener_count(), 0);

        for i in 0..4 {
            assert_eq!(t.add_listener(), ErrorCode::Ok, "Failed at listener {i}");
        }
        assert_eq!(t.listener_count(), 4);
        assert_eq!(t.add_listener(), ErrorCode::NoMoreListenersPossible);
    }

    #[test]
    fn remove_listener_allows_readd() {
        let mut t: AbstractTransceiver<2> = AbstractTransceiver::new(0);
        t.add_listener();
        t.add_listener();
        assert_eq!(t.add_listener(), ErrorCode::NoMoreListenersPossible);

        t.remove_listener();
        assert_eq!(t.listener_count(), 1);
        assert_eq!(t.add_listener(), ErrorCode::Ok);
    }

    #[test]
    fn remove_listener_at_zero_is_noop() {
        let mut t: AbstractTransceiver<4> = AbstractTransceiver::new(0);
        assert_eq!(t.listener_count(), 0);
        t.remove_listener(); // should not panic or underflow
        assert_eq!(t.listener_count(), 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. STATISTICS AND CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

mod statistics_and_config {
    use super::*;

    type Trx = AbstractTransceiver<8>;

    #[test]
    fn statistics_default_all_zero() {
        let t = Trx::new(0);
        let s = t.statistics();
        assert_eq!(s.rx, 0);
        assert_eq!(s.tx, 0);
        assert_eq!(s.errors, 0);
        assert_eq!(s.rx_dropped, 0);
        assert_eq!(s.tx_dropped, 0);
    }

    #[test]
    fn statistics_mutation() {
        let mut t = Trx::new(0);
        t.statistics_mut().rx = 100;
        t.statistics_mut().tx = 50;
        t.statistics_mut().errors = 3;
        assert_eq!(t.statistics().rx, 100);
        assert_eq!(t.statistics().tx, 50);
        assert_eq!(t.statistics().errors, 3);
    }

    #[test]
    fn bus_id_preserved() {
        let t: Trx = AbstractTransceiver::new(7);
        assert_eq!(t.bus_id(), 7);
    }

    #[test]
    fn baudrate_defaults_and_configurable() {
        let mut t = Trx::new(0);
        assert_eq!(t.baudrate(), BAUDRATE_HIGHSPEED);
        t.set_baudrate(BAUDRATE_LOWSPEED);
        assert_eq!(t.baudrate(), BAUDRATE_LOWSPEED);
    }

    #[test]
    fn transceiver_state_tracking() {
        let mut t = Trx::new(0);
        assert_eq!(t.transceiver_state(), TransceiverState::Active);
        t.set_transceiver_state(TransceiverState::Passive);
        assert_eq!(t.transceiver_state(), TransceiverState::Passive);
        t.set_transceiver_state(TransceiverState::BusOff);
        assert_eq!(t.transceiver_state(), TransceiverState::BusOff);
    }

    #[test]
    fn filter_accessible_through_transceiver() {
        let mut t = Trx::new(0);
        assert!(!t.filter().matches(0x100));
        t.filter_mut().add(0x100);
        assert!(t.filter().matches(0x100));
    }
}
