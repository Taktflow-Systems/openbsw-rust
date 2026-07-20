//! Pinned-baseline UDS parity evidence at upstream commit `be0029b`.
//!
//! Promoted 2026-07-20 from the 2026-07-19 drift tranche as part of the
//! governed oracle re-pin `ddbcf88` -> `be0029b`
//! (`docs/port/upstream-repin-decision-2026-07-19.md`,
//! `docs/port/repin-2026-07-20.md`). The vectors originate from the upstream
//! feature commits now inside the pinned baseline:
//! - `f8132091` feat(uds): add ReadDTCInformation service (0x19)
//! - `9558c245` Add UDS service ClearDiagnosticInformation (0x14)
//! - `39212d01` User specific configuration of module uds
//!
//! Two deliberate native differences between the upstream demo services and
//! the Rust port's DEM-backed native services are recorded parity decisions
//! in `docs/port/uds-parity.md` and are pinned by the tests below.

use std::{cell::RefCell, rc::Rc};

use bsw_time::{Duration, Instant};
use bsw_uds::{
    standard_services::{clear_diagnostic_information, read_dtc_information, CoreServices},
    state::{DiagnosticState, KeyAlgorithm, SecurityResetPolicy, StatePersistence},
    DemManager, DiagSession, Nrc,
};

// ─── Shared scaffolding ──────────────────────────────────────────────────────

#[derive(Clone, Default)]
struct Persist(Rc<RefCell<Option<(DiagSession, u8)>>>);

impl StatePersistence for Persist {
    fn load(&mut self) -> Option<(DiagSession, u8)> {
        *self.0.borrow()
    }
    fn store(&mut self, session: DiagSession, attempts: u8) -> bool {
        *self.0.borrow_mut() = Some((session, attempts));
        true
    }
}

struct NoKey;

impl KeyAlgorithm for NoKey {
    fn verify(&mut self, _level: u8, _seed: &[u8], _key: &[u8]) -> bool {
        false
    }
}

fn core(p2_ms: u16, p2_star_10ms: u16) -> CoreServices<Persist, NoKey> {
    let state = DiagnosticState::new(
        Persist::default(),
        NoKey,
        2,
        Duration::from_millis(10).unwrap(),
        SecurityResetPolicy::OnSessionChange,
    );
    CoreServices::new(state, p2_ms, p2_star_10ms)
}

/// Seed a DEM with exactly the upstream demo DTC `0x123456`, status `0x09`,
/// via the serialization format (report_event cannot produce status 0x09).
fn dem_with_demo_dtc() -> DemManager<4> {
    let mut dem = DemManager::<4>::new();
    // [count u16 LE][code u32 LE][status][occurrence u16 LE][aging]
    let image = [
        0x01, 0x00, // count = 1
        0x56, 0x34, 0x12, 0x00, // code 0x123456 LE
        0x09, // status 0x09 (testFailed | confirmedDTC)
        0x01, 0x00, // occurrence count 1
        0x00, // aging 0
    ];
    assert!(dem.deserialize(&image));
    dem
}

// ─── f8132091: ReadDTCInformation (0x19) demo service ────────────────────────

/// Baseline vector: `0x19 0x02 <mask>` answers with availability mask
/// 0xFF and the demo DTC record `0x123456` status `0x09`
/// (`ReadDTCInformationTest.cpp`,
/// `execute_with_subfunction_0x02_and_valid_status_mask_should_return_OK`).
/// Same golden response bytes as the pinned upstream demo payload.
#[test]
fn baseline_f8132091_report_dtc_by_status_mask_returns_demo_golden_bytes() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    let length = read_dtc_information(&[0x19, 0x02, 0xff], &dem, &mut response).unwrap();
    assert_eq!(
        &response[..length],
        &[0x59, 0x02, 0xff, 0x12, 0x34, 0x56, 0x09]
    );
}

/// Baseline vector: `0x19 0x02` without a status mask is rejected with
/// NRC 0x13 (incorrectMessageLengthOrInvalidFormat).
#[test]
fn baseline_f8132091_report_by_status_mask_without_mask_is_invalid_format() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    assert_eq!(
        read_dtc_information(&[0x19, 0x02], &dem, &mut response),
        Err(Nrc::IncorrectMessageLengthOrInvalidFormat)
    );
}

/// Baseline vector: sub-function 0x04
/// (reportDTCSnapshotRecordByDTCNumber) answers NRC 0x12.
#[test]
fn baseline_f8132091_snapshot_subfunction_not_supported() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    assert_eq!(
        read_dtc_information(&[0x19, 0x04, 0xff], &dem, &mut response),
        Err(Nrc::SubFunctionNotSupported)
    );
}

/// Baseline vector: sub-function 0x06
/// (reportDTCExtendedDataRecordByDTCNumber) answers NRC 0x12.
#[test]
fn baseline_f8132091_extended_data_subfunction_not_supported() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    assert_eq!(
        read_dtc_information(&[0x19, 0x06, 0xff], &dem, &mut response),
        Err(Nrc::SubFunctionNotSupported)
    );
}

/// Baseline vector: an unknown sub-function (0x00) answers NRC 0x12.
#[test]
fn baseline_f8132091_unknown_subfunction_not_supported() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    assert_eq!(
        read_dtc_information(&[0x19, 0x00, 0xff], &dem, &mut response),
        Err(Nrc::SubFunctionNotSupported)
    );
}

/// RECORDED PARITY DECISION (`docs/port/uds-parity.md`, re-pin 2026-07-20):
/// the upstream demo service answers NRC 0x12 for sub-function 0x0A
/// (reportSupportedDTCs); the Rust port's DEM-backed native service
/// implements 0x0A and returns the supported-DTC list. The native DEM
/// extension is broader than upstream's demo-grade service and is kept as a
/// deliberate native difference; this test pins the Rust behavior.
#[test]
fn baseline_f8132091_supported_dtcs_subfunction_diverges_from_upstream_demo() {
    let dem = dem_with_demo_dtc();
    let mut response = [0; 32];
    // Pinned upstream demo answers NRC 0x12 here (recorded native difference).
    let length = read_dtc_information(&[0x19, 0x0a, 0xff], &dem, &mut response).unwrap();
    assert_eq!(
        &response[..length],
        &[0x59, 0x0a, 0xff, 0x12, 0x34, 0x56, 0x09]
    );
}

// ─── 9558c245: ClearDiagnosticInformation (0x14) demo service ────────────────

/// Baseline vector: groupOfDTC 0xFFFFFF (all DTCs) answers the
/// positive response 0x54 (`ClearDiagnosticInformationTest.cpp`,
/// `execute_with_group_all_dtcs_should_return_DiagReturnCode_OK`).
#[test]
fn baseline_9558c245_clear_all_group_returns_positive_response() {
    let mut dem = dem_with_demo_dtc();
    let mut response = [0; 8];
    let length =
        clear_diagnostic_information(&[0x14, 0xff, 0xff, 0xff], &mut dem, &mut response).unwrap();
    assert_eq!(&response[..length], &[0x54]);
    assert_eq!(dem.count(), 0);
}

/// Baseline vector: an unsupported groupOfDTC (0x123456, not stored)
/// answers NRC 0x31 (requestOutOfRange).
#[test]
fn baseline_9558c245_unknown_group_returns_request_out_of_range() {
    let mut dem = DemManager::<4>::new();
    let mut response = [0; 8];
    assert_eq!(
        clear_diagnostic_information(&[0x14, 0x12, 0x34, 0x56], &mut dem, &mut response),
        Err(Nrc::RequestOutOfRange)
    );
}

/// Baseline vector: the powertrain group 0x000000 also answers
/// NRC 0x31 in the upstream demo service.
#[test]
fn baseline_9558c245_powertrain_group_returns_request_out_of_range() {
    let mut dem = DemManager::<4>::new();
    let mut response = [0; 8];
    assert_eq!(
        clear_diagnostic_information(&[0x14, 0x00, 0x00, 0x00], &mut dem, &mut response),
        Err(Nrc::RequestOutOfRange)
    );
}

/// Baseline vector: a truncated request (3 bytes, low group byte
/// missing) answers NRC 0x13. Upstream validates the fixed 4-byte request
/// length in the service framework; the Rust port validates it in the
/// service function itself.
#[test]
fn baseline_9558c245_short_request_is_invalid_format() {
    let mut dem = DemManager::<4>::new();
    let mut response = [0; 8];
    assert_eq!(
        clear_diagnostic_information(&[0x14, 0xff, 0xff], &mut dem, &mut response),
        Err(Nrc::IncorrectMessageLengthOrInvalidFormat)
    );
}

/// RECORDED PARITY DECISION (`docs/port/uds-parity.md`, re-pin 2026-07-20):
/// the upstream demo answers NRC 0x31 for EVERY group other than 0xFFFFFF,
/// even one that names a stored DTC. The Rust port's DEM-backed native
/// service clears an exact stored 24-bit group/code and answers positively.
/// The native DEM extension is kept as a deliberate native difference; this
/// test pins the Rust behavior.
#[test]
fn baseline_9558c245_known_group_clears_diverging_from_upstream_demo() {
    let mut dem = dem_with_demo_dtc();
    let mut response = [0; 8];
    // Pinned upstream demo answers NRC 0x31 here (recorded native difference).
    let length =
        clear_diagnostic_information(&[0x14, 0x12, 0x34, 0x56], &mut dem, &mut response).unwrap();
    assert_eq!(&response[..length], &[0x54]);
    assert_eq!(dem.count(), 0);
}

// ─── 39212d01: user-specific configuration of module uds ─────────────────────

/// Baseline vector: the default/extended-session DiagnosticSessionControl
/// positive response carries `DEFAULT_DIAG_RESPONSE_TIME` = 50 ms and
/// `DEFAULT_DIAG_RESPONSE_PENDING` = 500 (10 ms units) from the
/// integrator-owned `uds/UdsConfig.h`. The Rust port treats them as
/// integrator configuration (`CoreServices::new` parameters); the reference
/// composition ships the same defaults.
#[test]
fn baseline_39212d01_default_session_response_carries_configured_p2_values() {
    let mut core = core(50, 500);
    let mut response = [0; 8];
    let (length, _) = core
        .process(&[0x10, 0x01], Instant::from_nanos(0), &[], &mut response)
        .unwrap();
    assert_eq!(&response[..length], &[0x50, 0x01, 0x00, 0x32, 0x01, 0xf4]);
}

/// Baseline vector: for the programming session the pinned upstream
/// `uds/UdsConfig.h` sets `PROGRAMMING_DIAG_RESPONSE_PENDING` = 5000
/// (0x1388, 10 ms units), so `10 02` answers `50 02 00 32 13 88`. The Rust
/// port keeps the pair as integrator configuration per `CoreServices`
/// instance; the reference composition adopted the 5000 value at the
/// 2026-07-20 re-pin (`bsw-reference-core` `session_control`).
#[test]
fn baseline_39212d01_programming_session_post_drift_pending_is_expressible() {
    let mut core = core(50, 5000);
    let mut response = [0; 8];
    let (length, _) = core
        .process(&[0x10, 0x02], Instant::from_nanos(0), &[], &mut response)
        .unwrap();
    // Pinned upstream: 00 32 (P2 = 50 ms), 13 88 (P2* = 5000 x 10 ms).
    assert_eq!(&response[..length], &[0x50, 0x02, 0x00, 0x32, 0x13, 0x88]);
    assert_eq!(core.state.session(), DiagSession::Programming);
}
