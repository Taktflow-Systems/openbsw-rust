//! Integration tests for UDS diagnostic services and session management.
//!
//! Covers:
//! - Diagnostic session management (session transitions, masks, access control)
//! - Service routing and dispatch
//! - TesterPresent, EcuReset, DiagnosticSessionControl
//! - SecurityAccess seed/key exchange, lockout, reset
//! - WriteDataByIdentifier with security gating
//! - ControlDtcSetting, ReadDtcInformation, ClearDiagnosticInformation
//! - RoutineControl
//! - DEM lifecycle: report, clear, serialize, operation cycles

use bsw_uds::dem::{status_bits, DemManager};
use bsw_uds::diag_job::{DiagJob, DiagRouter};
use bsw_uds::nrc::Nrc;
use bsw_uds::service_id::ServiceId;
use bsw_uds::services::{
    ClearDiagnosticInformation, ControlDtcSetting, DiagnosticSessionControl, EcuReset,
    ReadDtcInformation, RoutineControl, SecurityAccess, TesterPresent,
    WriteDataByIdentifier,
};
use bsw_uds::session::{DiagSession, SessionMask};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. SESSION MANAGEMENT
// ═══════════════════════════════════════════════════════════════════════════════

mod session_management {
    use super::*;

    /// All valid session byte values round-trip through from_byte/as_byte.
    #[test]
    fn session_roundtrip_all_valid() {
        let sessions = [
            (0x01, DiagSession::Default),
            (0x02, DiagSession::Programming),
            (0x03, DiagSession::Extended),
        ];
        for (byte, expected) in sessions {
            let session = DiagSession::from_byte(byte).unwrap();
            assert_eq!(session, expected);
            assert_eq!(session.as_byte(), byte);
        }
    }

    /// Invalid session bytes return None.
    #[test]
    fn invalid_session_bytes() {
        for byte in [0x00, 0x04, 0x05, 0x10, 0xFF] {
            assert!(DiagSession::from_byte(byte).is_none(), "Byte 0x{byte:02X} should be invalid");
        }
    }

    /// SessionMask × DiagSession access control matrix.
    #[test]
    fn session_mask_access_matrix() {
        // ALL allows everything
        for session in [DiagSession::Default, DiagSession::Programming, DiagSession::Extended] {
            assert!(SessionMask::ALL.contains(session));
        }
        // NONE allows nothing
        for session in [DiagSession::Default, DiagSession::Programming, DiagSession::Extended] {
            assert!(!SessionMask::NONE.contains(session));
        }
        // DEFAULT only
        assert!(SessionMask::DEFAULT.contains(DiagSession::Default));
        assert!(!SessionMask::DEFAULT.contains(DiagSession::Programming));
        assert!(!SessionMask::DEFAULT.contains(DiagSession::Extended));
        // EXTENDED only
        assert!(!SessionMask::EXTENDED.contains(DiagSession::Default));
        assert!(!SessionMask::EXTENDED.contains(DiagSession::Programming));
        assert!(SessionMask::EXTENDED.contains(DiagSession::Extended));
        // DEFAULT_AND_EXTENDED
        assert!(SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Default));
        assert!(!SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Programming));
        assert!(SessionMask::DEFAULT_AND_EXTENDED.contains(DiagSession::Extended));
    }

    /// Session to_mask produces single-bit masks.
    #[test]
    fn session_to_mask_single_bit() {
        for session in [DiagSession::Default, DiagSession::Programming, DiagSession::Extended] {
            let mask = session.to_mask();
            assert!(mask.contains(session));
            // Count of other sessions NOT contained
            let others: Vec<_> = [DiagSession::Default, DiagSession::Programming, DiagSession::Extended]
                .iter()
                .filter(|&&s| s != session)
                .filter(|&&s| mask.contains(s))
                .collect();
            assert!(others.is_empty(), "Mask for {:?} should not contain other sessions", session);
        }
    }

    /// Mask union and intersection behave correctly.
    #[test]
    fn mask_set_operations() {
        let a = SessionMask::DEFAULT.union(SessionMask::EXTENDED);
        let b = SessionMask::DEFAULT.union(SessionMask::PROGRAMMING);

        let u = a.union(b);
        assert!(u.contains(DiagSession::Default));
        assert!(u.contains(DiagSession::Programming));
        assert!(u.contains(DiagSession::Extended));

        let i = a.intersection(b);
        assert!(i.contains(DiagSession::Default));
        assert!(!i.contains(DiagSession::Programming));
        assert!(!i.contains(DiagSession::Extended));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. DIAGNOSTIC JOB ROUTING
// ═══════════════════════════════════════════════════════════════════════════════

mod routing {
    use super::*;

    /// Router with multiple services dispatches to the correct one.
    #[test]
    fn router_dispatches_to_correct_service() {
        let tp = TesterPresent { session_mask: SessionMask::ALL };
        let ecu_rst = EcuReset { session_mask: SessionMask::ALL };
        let dsc = DiagnosticSessionControl { current_session: DiagSession::Default };

        let jobs: &[&dyn DiagJob] = &[&tp, &ecu_rst, &dsc];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 16];

        // TesterPresent
        let r = router.dispatch(&[0x3E, 0x00], DiagSession::Default, &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[0], 0x7E);

        // EcuReset
        let r = router.dispatch(&[0x11, 0x01], DiagSession::Default, &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[0], 0x51);

        // DiagnosticSessionControl
        let r = router.dispatch(&[0x10, 0x03], DiagSession::Default, &mut buf);
        assert_eq!(r, Ok(6));
        assert_eq!(buf[0], 0x50);
    }

    /// Unregistered service returns ServiceNotSupported.
    #[test]
    fn unregistered_service_nrc() {
        let tp = TesterPresent { session_mask: SessionMask::ALL };
        let jobs: &[&dyn DiagJob] = &[&tp];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];

        let r = router.dispatch(&[0x22, 0xF1, 0x90], DiagSession::Default, &mut buf);
        assert_eq!(r, Err(Nrc::ServiceNotSupported));
    }

    /// Service restricted to EXTENDED rejects DEFAULT session.
    #[test]
    fn session_restriction_enforced_by_router() {
        let dtc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let jobs: &[&dyn DiagJob] = &[&dtc];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];

        let r = router.dispatch(&[0x85, 0x01], DiagSession::Default, &mut buf);
        assert_eq!(r, Err(Nrc::ServiceNotSupportedInActiveSession));

        // Same request in Extended session should succeed
        let r = router.dispatch(&[0x85, 0x01], DiagSession::Extended, &mut buf);
        assert_eq!(r, Ok(2));
    }

    /// Empty router returns ServiceNotSupported.
    #[test]
    fn empty_router() {
        let jobs: &[&dyn DiagJob] = &[];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 8];
        let r = router.dispatch(&[0x3E, 0x00], DiagSession::Default, &mut buf);
        assert_eq!(r, Err(Nrc::ServiceNotSupported));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. SERVICE IMPLEMENTATIONS
// ═══════════════════════════════════════════════════════════════════════════════

mod services {
    use super::*;

    // --- TesterPresent ---

    #[test]
    fn tester_present_positive_response() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x3E, 0x00], &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[0], 0x7E);
        assert_eq!(buf[1], 0x00);
    }

    #[test]
    fn tester_present_suppress_positive_bit() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        // 0x80 = suppress-positive-response bit
        let r = svc.process(&[0x3E, 0x80], &mut buf);
        assert_eq!(r, Ok(2));
    }

    #[test]
    fn tester_present_invalid_subfunction() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x3E, 0x01], &mut buf);
        assert_eq!(r, Err(Nrc::SubFunctionNotSupported));
    }

    #[test]
    fn tester_present_too_short() {
        let svc = TesterPresent { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x3E], &mut buf);
        assert_eq!(r, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    // --- EcuReset ---

    #[test]
    fn ecu_reset_all_types() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];

        for sub_fn in [0x01, 0x02, 0x03] {
            let r = svc.process(&[0x11, sub_fn], &mut buf);
            assert_eq!(r, Ok(2));
            assert_eq!(buf[0], 0x51);
            assert_eq!(buf[1], sub_fn);
        }
    }

    #[test]
    fn ecu_reset_invalid_type() {
        let svc = EcuReset { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x11, 0x04], &mut buf);
        assert_eq!(r, Err(Nrc::SubFunctionNotSupported));
    }

    // --- DiagnosticSessionControl ---

    #[test]
    fn session_control_all_sessions() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];

        for sub_fn in [0x01, 0x02, 0x03] {
            let r = svc.process(&[0x10, sub_fn], &mut buf);
            assert_eq!(r, Ok(6));
            assert_eq!(buf[0], 0x50);
            assert_eq!(buf[1], sub_fn);
        }
    }

    #[test]
    fn session_control_timing_parameters() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        svc.process(&[0x10, 0x01], &mut buf).unwrap();
        let p2 = u16::from_be_bytes([buf[2], buf[3]]);
        let p2_star = u16::from_be_bytes([buf[4], buf[5]]);
        assert_eq!(p2, 50);     // P2 = 50 ms
        assert_eq!(p2_star, 500); // P2* = 500 × 10ms = 5000 ms
    }

    #[test]
    fn session_control_invalid_session_id() {
        let svc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let mut buf = [0u8; 8];
        for bad in [0x00, 0x04, 0xFF] {
            let r = svc.process(&[0x10, bad], &mut buf);
            assert_eq!(r, Err(Nrc::SubFunctionNotSupported));
        }
    }

    // --- ControlDtcSetting ---

    #[test]
    fn dtc_setting_on_and_off() {
        let svc = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x85, 0x01], &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[0], 0xC5);
        assert_eq!(buf[1], 0x01);

        let r = svc.process(&[0x85, 0x02], &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[1], 0x02);
    }

    // --- ReadDtcInformation ---

    #[test]
    fn read_dtc_report_by_status_mask() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x19, 0x02, 0xFF], &mut buf);
        assert_eq!(r, Ok(3));
        assert_eq!(buf[0], 0x59);
        assert_eq!(buf[1], 0x02);
        assert_eq!(buf[2], 0xFF);
    }

    #[test]
    fn read_dtc_report_supported() {
        let svc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x19, 0x0A], &mut buf);
        assert_eq!(r, Ok(3));
        assert_eq!(buf[0], 0x59);
        assert_eq!(buf[1], 0x0A);
    }

    // --- ClearDiagnosticInformation ---

    #[test]
    fn clear_dtc_all_group() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x14, 0xFF, 0xFF, 0xFF], &mut buf);
        assert_eq!(r, Ok(1));
        assert_eq!(buf[0], 0x54);
    }

    #[test]
    fn clear_dtc_too_short() {
        let svc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x14, 0xFF, 0xFF], &mut buf);
        assert_eq!(r, Err(Nrc::IncorrectMessageLengthOrInvalidFormat));
    }

    // --- RoutineControl ---

    #[test]
    fn routine_control_all_routines() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];

        // LED blink
        let r = svc.process(&[0x31, 0x01, 0xFF, 0x00], &mut buf);
        assert_eq!(r, Ok(5));
        assert_eq!(buf[0], 0x71);
        assert_eq!(buf[4], 0x01); // started

        // CAN self-test
        let r = svc.process(&[0x31, 0x01, 0xFF, 0x01], &mut buf);
        assert_eq!(r, Ok(5));
        assert_eq!(buf[4], 0x00); // pass

        // Read uptime
        let r = svc.process(&[0x31, 0x01, 0xFF, 0x02], &mut buf);
        assert_eq!(r, Ok(8));
    }

    #[test]
    fn routine_control_unknown_routine() {
        let svc = RoutineControl { session_mask: SessionMask::EXTENDED };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x31, 0x01, 0x00, 0x00], &mut buf);
        assert_eq!(r, Err(Nrc::RequestOutOfRange));
    }

    // --- WriteDataByIdentifier ---

    #[test]
    fn write_did_positive() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: false,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(r, Ok(3));
        assert_eq!(buf[0], 0x6E);
        assert_eq!(buf[1], 0xF1);
        assert_eq!(buf[2], 0x90);
    }

    #[test]
    fn write_did_security_denied() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: true,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(r, Err(Nrc::SecurityAccessDenied));
    }

    #[test]
    fn write_did_security_unlocked() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: true,
            security_unlocked: true,
        };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x2E, 0xF1, 0x90, 0xAB], &mut buf);
        assert_eq!(r, Ok(3));
    }

    #[test]
    fn write_did_reserved_zero_did() {
        let svc = WriteDataByIdentifier {
            session_mask: SessionMask::ALL,
            security_required: false,
            security_unlocked: false,
        };
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x2E, 0x00, 0x00, 0xAB], &mut buf);
        assert_eq!(r, Err(Nrc::RequestOutOfRange));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. SECURITY ACCESS
// ═══════════════════════════════════════════════════════════════════════════════

mod security_access {
    use super::*;

    /// Full unlock sequence: requestSeed → sendKey with correct key.
    #[test]
    fn full_unlock_sequence() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];

        // Request seed
        let r = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(r, Ok(4));
        assert_eq!(buf[0], 0x67);
        assert_eq!(buf[1], 0x01);
        let seed = (u32::from(buf[2]) << 8) | u32::from(buf[3]);
        assert_ne!(seed, 0);

        // Send correct key (seed XOR 0xDEAD_BEEF, lower 16 bits)
        let key = (seed ^ 0xDEAD_BEEF) as u16;
        let r = svc.process(&[0x27, 0x02, (key >> 8) as u8, key as u8], &mut buf);
        assert_eq!(r, Ok(2));
        assert_eq!(buf[0], 0x67);
        assert_eq!(buf[1], 0x02);
        assert!(svc.is_unlocked());
    }

    /// Wrong key returns InvalidKey NRC.
    #[test]
    fn wrong_key_returns_invalid_key() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();

        let r = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        assert_eq!(r, Err(Nrc::InvalidKey));
        assert!(!svc.is_unlocked());
    }

    /// Exceeded max attempts locks out until reset.
    #[test]
    fn max_attempts_lockout() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 2);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();

        // Fail twice → lockout
        svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf).ok();
        let r = svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf);
        assert_eq!(r, Err(Nrc::ExceededNumberOfAttempts));

        // Even seed request is locked
        let r = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(r, Err(Nrc::ExceededNumberOfAttempts));
    }

    /// Reset clears lockout and allows new seed request.
    #[test]
    fn reset_clears_lockout() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 1);
        let mut buf = [0u8; 8];
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        svc.process(&[0x27, 0x02, 0xDE, 0xAD], &mut buf).ok();
        assert_eq!(
            svc.process(&[0x27, 0x01], &mut buf),
            Err(Nrc::ExceededNumberOfAttempts)
        );

        svc.reset();
        let r = svc.process(&[0x27, 0x01], &mut buf);
        assert_eq!(r, Ok(4));
    }

    /// sendKey without requestSeed returns RequestSequenceError.
    #[test]
    fn key_without_seed_sequence_error() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        let r = svc.process(&[0x27, 0x02, 0x12, 0x34], &mut buf);
        assert_eq!(r, Err(Nrc::RequestSequenceError));
    }

    /// Already-unlocked seed request returns zero seed.
    #[test]
    fn already_unlocked_returns_zero_seed() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];

        // Unlock
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        let seed = (u32::from(buf[2]) << 8) | u32::from(buf[3]);
        let key = (seed ^ 0xDEAD_BEEF) as u16;
        svc.process(&[0x27, 0x02, (key >> 8) as u8, key as u8], &mut buf).unwrap();

        // Request seed again
        svc.process(&[0x27, 0x01], &mut buf).unwrap();
        assert_eq!(buf[2], 0x00);
        assert_eq!(buf[3], 0x00);
    }

    /// Invalid sub-function returns SubFunctionNotSupported.
    #[test]
    fn invalid_subfunction() {
        let svc = SecurityAccess::new(SessionMask::EXTENDED, 3);
        let mut buf = [0u8; 8];
        for bad_sub in [0x00, 0x03, 0x04, 0x7F] {
            let r = svc.process(&[0x27, bad_sub], &mut buf);
            assert_eq!(r, Err(Nrc::SubFunctionNotSupported), "Sub-fn 0x{bad_sub:02X}");
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. SERVICE ID AND NRC ENUMERATION
// ═══════════════════════════════════════════════════════════════════════════════

mod service_id_and_nrc {
    use super::*;

    #[test]
    fn service_id_positive_response_is_sid_plus_0x40() {
        let test_cases = [
            (ServiceId::DiagnosticSessionControl, 0x50),
            (ServiceId::EcuReset, 0x51),
            (ServiceId::ReadDataByIdentifier, 0x62),
            (ServiceId::TesterPresent, 0x7E),
        ];
        for (sid, expected_pos) in test_cases {
            assert_eq!(sid.positive_response_id(), expected_pos);
        }
    }

    #[test]
    fn service_id_roundtrip() {
        let sids = [
            ServiceId::DiagnosticSessionControl,
            ServiceId::EcuReset,
            ServiceId::SecurityAccess,
            ServiceId::TesterPresent,
            ServiceId::ReadDataByIdentifier,
            ServiceId::WriteDataByIdentifier,
            ServiceId::RoutineControl,
            ServiceId::NegativeResponse,
        ];
        for sid in sids {
            let byte = sid.as_byte();
            let decoded = ServiceId::from_byte(byte);
            assert_eq!(decoded, Some(sid), "Roundtrip failed for {:?}", sid);
        }
    }

    #[test]
    fn nrc_is_busy_classification() {
        assert!(Nrc::BusyRepeatRequest.is_busy());
        assert!(Nrc::RequestCorrectlyReceivedResponsePending.is_busy());
        assert!(!Nrc::GeneralReject.is_busy());
        assert!(!Nrc::ServiceNotSupported.is_busy());
        assert!(!Nrc::InvalidKey.is_busy());
    }

    #[test]
    fn nrc_roundtrip_all_defined() {
        let all_nrcs = [
            Nrc::GeneralReject, Nrc::ServiceNotSupported, Nrc::SubFunctionNotSupported,
            Nrc::IncorrectMessageLengthOrInvalidFormat, Nrc::ResponseTooLong,
            Nrc::BusyRepeatRequest, Nrc::ConditionsNotCorrect, Nrc::RequestSequenceError,
            Nrc::RequestOutOfRange, Nrc::SecurityAccessDenied, Nrc::InvalidKey,
            Nrc::ExceededNumberOfAttempts, Nrc::ServiceNotSupportedInActiveSession,
        ];
        for nrc in all_nrcs {
            assert_eq!(Nrc::from_byte(nrc.as_byte()), Some(nrc));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. DEM — DIAGNOSTIC EVENT MANAGER
// ═══════════════════════════════════════════════════════════════════════════════

mod dem_tests {
    use super::*;

    #[test]
    fn dem_report_fail_sets_status_bits() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0xC0_7300, true);

        let e = dem.get(0xC0_7300).unwrap();
        assert_ne!(e.status & status_bits::TEST_FAILED, 0);
        assert_ne!(e.status & status_bits::CONFIRMED_DTC, 0);
        assert_ne!(e.status & status_bits::PENDING_DTC, 0);
        assert_ne!(e.status & status_bits::TEST_FAILED_THIS_OP_CYCLE, 0);
        assert_ne!(e.status & status_bits::TEST_FAILED_SINCE_CLEAR, 0);
        assert_eq!(e.occurrence_count, 1);
    }

    #[test]
    fn dem_report_pass_clears_transient_bits() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0xC0_7300, true);
        dem.report_event(0xC0_7300, false);

        let e = dem.get(0xC0_7300).unwrap();
        assert_eq!(e.status & status_bits::TEST_FAILED, 0);
        assert_eq!(e.status & status_bits::TEST_FAILED_THIS_OP_CYCLE, 0);
        // Confirmed survives pass
        assert_ne!(e.status & status_bits::CONFIRMED_DTC, 0);
        assert_eq!(e.aging_count, 1);
    }

    #[test]
    fn dem_dtc_setting_disabled_ignores_events() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.set_dtc_setting(false);
        dem.report_event(0xAA_0001, true);
        assert_eq!(dem.count(), 0);

        dem.set_dtc_setting(true);
        dem.report_event(0xAA_0001, true);
        assert_eq!(dem.count(), 1);
    }

    #[test]
    fn dem_clear_all() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0x01_0000, true);
        dem.report_event(0x02_0000, true);
        dem.clear_all();
        assert_eq!(dem.count(), 0);
    }

    #[test]
    fn dem_clear_specific_group() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0xAA_0001, true);
        dem.report_event(0xAA_0002, true);
        dem.clear_group(0xAA_0001);
        assert_eq!(dem.count(), 1);
        assert!(dem.get(0xAA_0001).is_none());
        assert!(dem.get(0xAA_0002).is_some());
    }

    #[test]
    fn dem_new_operation_cycle_clears_per_cycle_bits() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0xCC_0001, true);
        dem.new_operation_cycle();
        let e = dem.get(0xCC_0001).unwrap();
        assert_eq!(e.status & status_bits::TEST_FAILED_THIS_OP_CYCLE, 0);
        assert_ne!(e.status & status_bits::CONFIRMED_DTC, 0);
    }

    #[test]
    fn dem_serialize_deserialize_roundtrip() {
        let mut dem1: DemManager<8> = DemManager::new();
        dem1.report_event(0xC0_7300, true);
        dem1.report_event(0xB1_0000, false);

        let mut buf = [0u8; 128];
        let written = dem1.serialize(&mut buf);
        assert!(written > 0);

        let mut dem2: DemManager<8> = DemManager::new();
        assert!(dem2.deserialize(&buf[..written]));
        assert_eq!(dem2.count(), dem1.count());

        let e1 = dem1.get(0xC0_7300).unwrap();
        let e2 = dem2.get(0xC0_7300).unwrap();
        assert_eq!(e1.status, e2.status);
        assert_eq!(e1.occurrence_count, e2.occurrence_count);
    }

    #[test]
    fn dem_max_capacity_enforcement() {
        let mut dem: DemManager<2> = DemManager::new();
        dem.report_event(0x01, true);
        dem.report_event(0x02, true);
        dem.report_event(0x03, true); // silently dropped
        assert_eq!(dem.count(), 2);
        assert!(dem.get(0x03).is_none());
    }

    #[test]
    fn dem_get_by_status_mask() {
        let mut dem: DemManager<16> = DemManager::new();
        dem.report_event(0xAA, true);   // confirmed, failed
        dem.report_event(0xBB, false);  // no failure bits

        let confirmed: Vec<u32> = dem
            .get_by_status_mask(status_bits::CONFIRMED_DTC)
            .map(|e| e.code)
            .collect();
        assert!(confirmed.contains(&0xAA));
        assert!(!confirmed.contains(&0xBB));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 7. END-TO-END: FULL ROUTER WITH ALL SERVICES
// ═══════════════════════════════════════════════════════════════════════════════

mod e2e_router {
    use super::*;

    /// Build a router with all standard services and verify dispatch for each.
    #[test]
    fn full_service_stack_dispatch() {
        let tp = TesterPresent { session_mask: SessionMask::ALL };
        let ecu_rst = EcuReset { session_mask: SessionMask::ALL };
        let dsc = DiagnosticSessionControl { current_session: DiagSession::Default };
        let dtc_ctrl = ControlDtcSetting { session_mask: SessionMask::EXTENDED };
        let read_dtc = ReadDtcInformation { session_mask: SessionMask::ALL };
        let clear_dtc = ClearDiagnosticInformation { session_mask: SessionMask::ALL };
        let routine = RoutineControl { session_mask: SessionMask::EXTENDED };
        let sec = SecurityAccess::new(SessionMask::EXTENDED, 3);

        let jobs: &[&dyn DiagJob] = &[
            &tp, &ecu_rst, &dsc, &dtc_ctrl, &read_dtc, &clear_dtc, &routine, &sec,
        ];
        let router = DiagRouter::new(jobs);
        let mut buf = [0u8; 16];

        // TesterPresent
        assert!(router.dispatch(&[0x3E, 0x00], DiagSession::Default, &mut buf).is_ok());
        // EcuReset
        assert!(router.dispatch(&[0x11, 0x01], DiagSession::Default, &mut buf).is_ok());
        // DiagSessionControl
        assert!(router.dispatch(&[0x10, 0x01], DiagSession::Default, &mut buf).is_ok());
        // ControlDtcSetting — only in Extended
        assert_eq!(
            router.dispatch(&[0x85, 0x01], DiagSession::Default, &mut buf),
            Err(Nrc::ServiceNotSupportedInActiveSession)
        );
        assert!(router.dispatch(&[0x85, 0x01], DiagSession::Extended, &mut buf).is_ok());
        // ReadDtcInfo
        assert!(router.dispatch(&[0x19, 0x0A], DiagSession::Default, &mut buf).is_ok());
        // ClearDTC
        assert!(router.dispatch(&[0x14, 0xFF, 0xFF, 0xFF], DiagSession::Default, &mut buf).is_ok());
        // RoutineControl — only in Extended
        assert_eq!(
            router.dispatch(&[0x31, 0x01, 0xFF, 0x00], DiagSession::Default, &mut buf),
            Err(Nrc::ServiceNotSupportedInActiveSession)
        );
        assert!(router.dispatch(&[0x31, 0x01, 0xFF, 0x00], DiagSession::Extended, &mut buf).is_ok());
        // SecurityAccess — only in Extended
        assert_eq!(
            router.dispatch(&[0x27, 0x01], DiagSession::Default, &mut buf),
            Err(Nrc::ServiceNotSupportedInActiveSession)
        );
        assert!(router.dispatch(&[0x27, 0x01], DiagSession::Extended, &mut buf).is_ok());
    }
}
