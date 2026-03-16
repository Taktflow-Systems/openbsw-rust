//! Diagnostic Event Manager (DEM) — DTC lifecycle per ISO 14229-1.
//!
//! Manages Diagnostic Trouble Code (DTC) storage, status-bit transitions,
//! operation-cycle handling, and NvM serialization.  Designed for `no_std`
//! fixed-capacity embedded targets.

// ─── Status byte bit definitions ─────────────────────────────────────────────

/// DTC status byte bits (ISO 14229-1 §D.2).
pub mod status_bits {
    /// Bit 0 — test currently failed.
    pub const TEST_FAILED: u8 = 0x01;
    /// Bit 1 — test failed at least once this operation cycle.
    pub const TEST_FAILED_THIS_OP_CYCLE: u8 = 0x02;
    /// Bit 2 — DTC is pending (failed this or previous cycle).
    pub const PENDING_DTC: u8 = 0x04;
    /// Bit 3 — DTC has been confirmed.
    pub const CONFIRMED_DTC: u8 = 0x08;
    /// Bit 4 — test has not completed since the last DTC clear.
    pub const TEST_NOT_COMPLETED_SINCE_CLEAR: u8 = 0x10;
    /// Bit 5 — test failed at least once since the last DTC clear.
    pub const TEST_FAILED_SINCE_CLEAR: u8 = 0x20;
    /// Bit 6 — test has not completed this operation cycle.
    pub const TEST_NOT_COMPLETED_THIS_OP_CYCLE: u8 = 0x40;
    /// Bit 7 — warning indicator requested.
    pub const WARNING_INDICATOR: u8 = 0x80;
}

// ─── DtcEntry ────────────────────────────────────────────────────────────────

/// A single DTC storage entry.
#[derive(Debug, Clone, Copy)]
pub struct DtcEntry {
    /// 3-byte DTC code (e.g., `0xC0_7300` for CAN bus-off).
    pub code: u32,
    /// ISO 14229 status byte.
    pub status: u8,
    /// Number of times `report_event(code, true)` has been called for this DTC.
    /// Saturates at [`u16::MAX`].
    pub occurrence_count: u16,
    /// Aging counter — incremented each time the test passes.  Reset to 0 on
    /// failure.  Callers can use this to age-out confirmed DTCs after N cycles.
    pub aging_count: u8,
}

impl DtcEntry {
    const fn zeroed() -> Self {
        Self {
            code: 0,
            status: 0,
            occurrence_count: 0,
            aging_count: 0,
        }
    }
}

// ─── DemManager ──────────────────────────────────────────────────────────────

/// DEM manager with fixed-capacity DTC storage.
///
/// `MAX_DTCS` is the maximum number of distinct DTC codes that can be stored.
/// Once the table is full, new DTC codes are silently dropped (same behaviour
/// as most production DEMs — the existing confirmed DTCs take priority).
pub struct DemManager<const MAX_DTCS: usize> {
    dtcs: [DtcEntry; MAX_DTCS],
    count: usize,
    /// Mirrors UDS 0x85 ControlDtcSetting — when `false`, `report_event` is
    /// a no-op (ISO 14229-1 §9.8.5).
    dtc_setting_enabled: bool,
}

impl<const MAX_DTCS: usize> DemManager<MAX_DTCS> {
    /// Create a new, empty DEM manager with DTC setting enabled.
    pub const fn new() -> Self {
        Self {
            dtcs: [DtcEntry::zeroed(); MAX_DTCS],
            count: 0,
            dtc_setting_enabled: true,
        }
    }

    // ─── Event reporting ─────────────────────────────────────────────────────

    /// Report a test result for the given DTC code.
    ///
    /// * `failed = true`  — test has failed; sets/confirms the DTC.
    /// * `failed = false` — test has passed; clears transient bits and ages
    ///   the entry.
    ///
    /// If [`dtc_setting_enabled`](Self::dtc_setting_enabled) is `false` the
    /// call is silently ignored per ISO 14229-1 §9.8.5.
    pub fn report_event(&mut self, dtc_code: u32, failed: bool) {
        if !self.dtc_setting_enabled {
            return;
        }

        let idx = self.find_or_create(dtc_code);
        let Some(idx) = idx else { return };

        let entry = &mut self.dtcs[idx];

        if failed {
            // Set transient + latching failure bits.
            entry.status |= status_bits::TEST_FAILED
                | status_bits::TEST_FAILED_THIS_OP_CYCLE
                | status_bits::PENDING_DTC
                | status_bits::TEST_FAILED_SINCE_CLEAR
                | status_bits::CONFIRMED_DTC;
            // Clear "not completed" bits — we now know the result.
            entry.status &= !(status_bits::TEST_NOT_COMPLETED_THIS_OP_CYCLE
                | status_bits::TEST_NOT_COMPLETED_SINCE_CLEAR);
            // Saturating increment.
            entry.occurrence_count = entry.occurrence_count.saturating_add(1);
            // Reset aging on failure.
            entry.aging_count = 0;
        } else {
            // Test passed — clear current-failure bits.
            entry.status &= !(status_bits::TEST_FAILED | status_bits::TEST_FAILED_THIS_OP_CYCLE);
            // Keep PENDING_DTC and CONFIRMED_DTC — they require explicit clear.
            // Clear "not completed" bits.
            entry.status &= !(status_bits::TEST_NOT_COMPLETED_THIS_OP_CYCLE
                | status_bits::TEST_NOT_COMPLETED_SINCE_CLEAR);
            // Age the entry.
            entry.aging_count = entry.aging_count.saturating_add(1);
        }
    }

    // ─── Queries ──────────────────────────────────────────────────────────────

    /// Iterate over DTC entries whose status byte has at least one bit in
    /// common with `mask` (i.e., `entry.status & mask != 0`).
    ///
    /// Pass `0xFF` to retrieve all stored DTCs regardless of status.
    pub fn get_by_status_mask(&self, mask: u8) -> impl Iterator<Item = &DtcEntry> {
        self.dtcs[..self.count]
            .iter()
            .filter(move |e| e.status & mask != 0)
    }

    /// Return a reference to the entry for `dtc_code`, or `None` if the DTC
    /// is not currently stored.
    pub fn get(&self, dtc_code: u32) -> Option<&DtcEntry> {
        self.dtcs[..self.count]
            .iter()
            .find(|e| e.code == dtc_code)
    }

    /// Number of DTC entries currently stored.
    pub fn count(&self) -> usize {
        self.count
    }

    // ─── Clear operations ────────────────────────────────────────────────────

    /// Clear all stored DTCs (UDS 0x14 with group `0xFFFFFF`).
    pub fn clear_all(&mut self) {
        // Zero out the entries so no stale data lingers.
        for entry in &mut self.dtcs[..self.count] {
            *entry = DtcEntry::zeroed();
        }
        self.count = 0;
    }

    /// Clear DTCs whose code matches `group`.
    ///
    /// * `0xFFFFFF` delegates to [`clear_all`](Self::clear_all).
    /// * Any other value removes only the single DTC with that exact code (if
    ///   present).  ISO 14229-1 specifies group ranges; this simplified
    ///   implementation treats the value as an exact code match unless it is
    ///   the all-group sentinel.
    pub fn clear_group(&mut self, group: u32) {
        if group == 0xFF_FFFF {
            self.clear_all();
            return;
        }
        // Find and remove the matching entry (swap-remove to keep array dense).
        if let Some(pos) = self.dtcs[..self.count].iter().position(|e| e.code == group) {
            self.count -= 1;
            self.dtcs[pos] = self.dtcs[self.count];
            self.dtcs[self.count] = DtcEntry::zeroed();
        }
    }

    // ─── DTC setting (UDS 0x85) ───────────────────────────────────────────────

    /// Enable or disable DTC setting.  When disabled, `report_event` is a
    /// no-op (ISO 14229-1 §9.8.5).
    pub fn set_dtc_setting(&mut self, enabled: bool) {
        self.dtc_setting_enabled = enabled;
    }

    /// Returns `true` if DTC setting is currently enabled.
    pub fn dtc_setting_enabled(&self) -> bool {
        self.dtc_setting_enabled
    }

    // ─── Operation cycle ─────────────────────────────────────────────────────

    /// Signal the start of a new operation cycle.
    ///
    /// Clears per-cycle bits for every stored entry:
    /// * [`TEST_FAILED_THIS_OP_CYCLE`](status_bits::TEST_FAILED_THIS_OP_CYCLE)
    /// * [`TEST_NOT_COMPLETED_THIS_OP_CYCLE`](status_bits::TEST_NOT_COMPLETED_THIS_OP_CYCLE)
    pub fn new_operation_cycle(&mut self) {
        let mask = !(status_bits::TEST_FAILED_THIS_OP_CYCLE
            | status_bits::TEST_NOT_COMPLETED_THIS_OP_CYCLE);
        for entry in &mut self.dtcs[..self.count] {
            entry.status &= mask;
        }
    }

    // ─── NvM serialization ───────────────────────────────────────────────────

    /// Serialize the DTC table into `buf`.
    ///
    /// Format (little-endian):
    /// ```text
    /// [count: u16 LE]
    /// for each entry:
    ///   [code: u32 LE][status: u8][occurrence_count: u16 LE][aging_count: u8]
    /// ```
    /// Each entry is 8 bytes; total = `2 + count * 8` bytes.
    ///
    /// Returns the number of bytes written, or `0` if `buf` is too small.
    pub fn serialize(&self, buf: &mut [u8]) -> usize {
        let needed = 2 + self.count * 8;
        if buf.len() < needed {
            return 0;
        }
        let count_u16 = self.count as u16;
        buf[0] = count_u16 as u8;
        buf[1] = (count_u16 >> 8) as u8;
        let mut offset = 2usize;
        for entry in &self.dtcs[..self.count] {
            buf[offset] = entry.code as u8;
            buf[offset + 1] = (entry.code >> 8) as u8;
            buf[offset + 2] = (entry.code >> 16) as u8;
            buf[offset + 3] = (entry.code >> 24) as u8;
            buf[offset + 4] = entry.status;
            buf[offset + 5] = entry.occurrence_count as u8;
            buf[offset + 6] = (entry.occurrence_count >> 8) as u8;
            buf[offset + 7] = entry.aging_count;
            offset += 8;
        }
        needed
    }

    /// Deserialize the DTC table from `data` (written by [`serialize`](Self::serialize)).
    ///
    /// Returns `true` on success, `false` if `data` is too short or the encoded
    /// count exceeds `MAX_DTCS`.  On failure the manager state is left unchanged.
    pub fn deserialize(&mut self, data: &[u8]) -> bool {
        if data.len() < 2 {
            return false;
        }
        let count = u16::from_le_bytes([data[0], data[1]]) as usize;
        if count > MAX_DTCS {
            return false;
        }
        let needed = 2 + count * 8;
        if data.len() < needed {
            return false;
        }
        // Commit — clear existing table then load.
        self.clear_all();
        let mut offset = 2usize;
        for i in 0..count {
            let code = u32::from_le_bytes([
                data[offset],
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
            ]);
            let status = data[offset + 4];
            let occurrence_count =
                u16::from_le_bytes([data[offset + 5], data[offset + 6]]);
            let aging_count = data[offset + 7];
            self.dtcs[i] = DtcEntry { code, status, occurrence_count, aging_count };
            offset += 8;
        }
        self.count = count;
        true
    }

    // ─── Internal helpers ─────────────────────────────────────────────────────

    /// Return the index of the entry with `dtc_code`, creating a new entry if
    /// it does not yet exist.  Returns `None` if the table is full.
    fn find_or_create(&mut self, dtc_code: u32) -> Option<usize> {
        // Search existing entries.
        for (i, entry) in self.dtcs[..self.count].iter().enumerate() {
            if entry.code == dtc_code {
                return Some(i);
            }
        }
        // Create new entry.
        if self.count >= MAX_DTCS {
            return None;
        }
        let idx = self.count;
        self.dtcs[idx] = DtcEntry {
            code: dtc_code,
            // New entries start with "not completed since clear" bits set —
            // status is unknown until a test result arrives.
            status: status_bits::TEST_NOT_COMPLETED_SINCE_CLEAR
                | status_bits::TEST_NOT_COMPLETED_THIS_OP_CYCLE,
            occurrence_count: 0,
            aging_count: 0,
        };
        self.count += 1;
        Some(idx)
    }
}

// ─── Unit tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::{status_bits::*, *};

    // Helper: create a small DEM for most tests.
    fn dem() -> DemManager<16> {
        DemManager::new()
    }

    // ── T01: report failed → TEST_FAILED + CONFIRMED_DTC set ─────────────────
    #[test]
    fn t01_report_failed_sets_test_failed_and_confirmed() {
        let mut d = dem();
        d.report_event(0xC0_7300, true);
        let e = d.get(0xC0_7300).expect("entry should exist");
        assert_ne!(e.status & TEST_FAILED, 0, "TEST_FAILED must be set");
        assert_ne!(e.status & CONFIRMED_DTC, 0, "CONFIRMED_DTC must be set");
        assert_ne!(e.status & PENDING_DTC, 0, "PENDING_DTC must be set");
        assert_ne!(e.status & TEST_FAILED_SINCE_CLEAR, 0, "TEST_FAILED_SINCE_CLEAR must be set");
        assert_ne!(
            e.status & TEST_FAILED_THIS_OP_CYCLE,
            0,
            "TEST_FAILED_THIS_OP_CYCLE must be set"
        );
    }

    // ── T02: report passed → TEST_FAILED cleared ──────────────────────────────
    #[test]
    fn t02_report_passed_clears_test_failed() {
        let mut d = dem();
        d.report_event(0xC0_7300, true);
        d.report_event(0xC0_7300, false);
        let e = d.get(0xC0_7300).unwrap();
        assert_eq!(e.status & TEST_FAILED, 0, "TEST_FAILED must be cleared after pass");
        assert_eq!(
            e.status & TEST_FAILED_THIS_OP_CYCLE,
            0,
            "TEST_FAILED_THIS_OP_CYCLE must be cleared after pass"
        );
        // CONFIRMED_DTC must survive a single pass.
        assert_ne!(e.status & CONFIRMED_DTC, 0, "CONFIRMED_DTC must survive a pass");
    }

    // ── T03: multiple distinct DTCs stored independently ─────────────────────
    #[test]
    fn t03_multiple_dtcs_stored_independently() {
        let mut d = dem();
        d.report_event(0xC0_7300, true);
        d.report_event(0xB1_0000, false);
        assert_eq!(d.count(), 2);
        let e1 = d.get(0xC0_7300).unwrap();
        assert_ne!(e1.status & TEST_FAILED, 0);
        let e2 = d.get(0xB1_0000).unwrap();
        assert_eq!(e2.status & TEST_FAILED, 0);
    }

    // ── T04: get_by_status_mask returns only matching entries ────────────────
    #[test]
    fn t04_get_by_status_mask_filters_correctly() {
        let mut d = dem();
        d.report_event(0xAA_0001, true);  // confirmed, failed
        d.report_event(0xAA_0002, false); // no failure bits
        // Filter for CONFIRMED_DTC.
        let confirmed: Vec<u32> = d
            .get_by_status_mask(CONFIRMED_DTC)
            .map(|e| e.code)
            .collect();
        assert!(confirmed.contains(&0xAA_0001));
        assert!(!confirmed.contains(&0xAA_0002));
    }

    // ── T05: clear_all removes all entries ───────────────────────────────────
    #[test]
    fn t05_clear_all_removes_all_entries() {
        let mut d = dem();
        d.report_event(0x11_0001, true);
        d.report_event(0x11_0002, true);
        assert_eq!(d.count(), 2);
        d.clear_all();
        assert_eq!(d.count(), 0);
        assert!(d.get(0x11_0001).is_none());
        assert!(d.get(0x11_0002).is_none());
    }

    // ── T06: DTC setting disabled → report_event is no-op ────────────────────
    #[test]
    fn t06_dtc_setting_disabled_ignores_report() {
        let mut d = dem();
        d.set_dtc_setting(false);
        assert!(!d.dtc_setting_enabled());
        d.report_event(0xDE_AD00, true);
        assert_eq!(d.count(), 0, "No entry must be created while DTC setting is off");
    }

    // ── T07: re-enable DTC setting works normally after disable ───────────────
    #[test]
    fn t07_dtc_setting_reenable_works() {
        let mut d = dem();
        d.set_dtc_setting(false);
        d.report_event(0xDE_AD00, true);
        assert_eq!(d.count(), 0);
        d.set_dtc_setting(true);
        d.report_event(0xDE_AD00, true);
        assert_eq!(d.count(), 1);
    }

    // ── T08: new_operation_cycle clears per-cycle bits ────────────────────────
    #[test]
    fn t08_new_operation_cycle_clears_per_cycle_bits() {
        let mut d = dem();
        d.report_event(0xCC_0001, true);
        {
            let e = d.get(0xCC_0001).unwrap();
            assert_ne!(e.status & TEST_FAILED_THIS_OP_CYCLE, 0);
        }
        d.new_operation_cycle();
        let e = d.get(0xCC_0001).unwrap();
        assert_eq!(
            e.status & TEST_FAILED_THIS_OP_CYCLE,
            0,
            "TEST_FAILED_THIS_OP_CYCLE must be cleared after new op-cycle"
        );
        // Confirmed must survive the cycle boundary.
        assert_ne!(e.status & CONFIRMED_DTC, 0);
    }

    // ── T09: serialize / deserialize round-trip ───────────────────────────────
    #[test]
    fn t09_serialize_deserialize_roundtrip() {
        let mut d1: DemManager<8> = DemManager::new();
        d1.report_event(0xC0_7300, true);
        d1.report_event(0xB1_0000, false);

        let mut buf = [0u8; 128];
        let written = d1.serialize(&mut buf);
        assert!(written > 0);

        let mut d2: DemManager<8> = DemManager::new();
        let ok = d2.deserialize(&buf[..written]);
        assert!(ok, "deserialize must succeed");
        assert_eq!(d2.count(), d1.count());

        let e1 = d1.get(0xC0_7300).unwrap();
        let e2 = d2.get(0xC0_7300).unwrap();
        assert_eq!(e1.code, e2.code);
        assert_eq!(e1.status, e2.status);
        assert_eq!(e1.occurrence_count, e2.occurrence_count);
        assert_eq!(e1.aging_count, e2.aging_count);
    }

    // ── T10: occurrence_count increments on each failure ─────────────────────
    #[test]
    fn t10_occurrence_count_increments_on_failure() {
        let mut d = dem();
        d.report_event(0xAB_CD00, true);
        d.report_event(0xAB_CD00, true);
        d.report_event(0xAB_CD00, true);
        let e = d.get(0xAB_CD00).unwrap();
        assert_eq!(e.occurrence_count, 3);
    }

    // ── T11: aging_count increments on each pass ──────────────────────────────
    #[test]
    fn t11_aging_count_increments_on_pass() {
        let mut d = dem();
        d.report_event(0x55_0000, true);
        d.report_event(0x55_0000, false);
        d.report_event(0x55_0000, false);
        let e = d.get(0x55_0000).unwrap();
        assert_eq!(e.aging_count, 2);
    }

    // ── T12: aging_count resets to 0 on re-failure ────────────────────────────
    #[test]
    fn t12_aging_count_resets_on_failure() {
        let mut d = dem();
        d.report_event(0x55_0001, true);
        d.report_event(0x55_0001, false);
        d.report_event(0x55_0001, false);
        assert_eq!(d.get(0x55_0001).unwrap().aging_count, 2);
        d.report_event(0x55_0001, true);
        assert_eq!(d.get(0x55_0001).unwrap().aging_count, 0);
    }

    // ── T13: max capacity — new DTC silently dropped when full ───────────────
    #[test]
    fn t13_max_capacity_drops_new_dtc() {
        let mut d: DemManager<2> = DemManager::new();
        d.report_event(0x01_0000, true);
        d.report_event(0x02_0000, true);
        assert_eq!(d.count(), 2);
        // Third DTC exceeds capacity.
        d.report_event(0x03_0000, true);
        assert_eq!(d.count(), 2, "capacity must not be exceeded");
        assert!(d.get(0x03_0000).is_none(), "overflow DTC must not be stored");
    }

    // ── T14: occurrence_count saturates at u16::MAX ───────────────────────────
    #[test]
    fn t14_occurrence_count_saturates() {
        let mut d = dem();
        // Prime the entry.
        d.report_event(0xFF_0000, true);
        // Manually set occurrence_count close to saturation.
        d.dtcs[0].occurrence_count = u16::MAX;
        d.report_event(0xFF_0000, true);
        assert_eq!(d.dtcs[0].occurrence_count, u16::MAX, "must saturate at u16::MAX");
    }

    // ── T15: clear_group removes specific DTC only ────────────────────────────
    #[test]
    fn t15_clear_group_removes_specific_dtc() {
        let mut d = dem();
        d.report_event(0xAA_0001, true);
        d.report_event(0xAA_0002, true);
        d.clear_group(0xAA_0001);
        assert_eq!(d.count(), 1);
        assert!(d.get(0xAA_0001).is_none());
        assert!(d.get(0xAA_0002).is_some());
    }

    // ── T16: clear_group(0xFFFFFF) clears all ────────────────────────────────
    #[test]
    fn t16_clear_group_all_sentinel() {
        let mut d = dem();
        d.report_event(0x11_0000, true);
        d.report_event(0x22_0000, true);
        d.clear_group(0xFF_FFFF);
        assert_eq!(d.count(), 0);
    }

    // ── T17: deserialize rejects truncated data ───────────────────────────────
    #[test]
    fn t17_deserialize_rejects_truncated_data() {
        let mut d: DemManager<4> = DemManager::new();
        // 2-byte header claims 3 entries but buffer is too short.
        let data = [3u8, 0u8, 0u8, 0u8]; // header says 3 entries, only 2 bytes of entry data
        assert!(!d.deserialize(&data));
        assert_eq!(d.count(), 0);
    }

    // ── T18: deserialize rejects count > MAX_DTCS ────────────────────────────
    #[test]
    fn t18_deserialize_rejects_count_exceeding_capacity() {
        let mut d: DemManager<2> = DemManager::new();
        // Header claims 5 entries — exceeds MAX_DTCS=2.
        let mut data = [0u8; 2 + 5 * 8];
        data[0] = 5;
        assert!(!d.deserialize(&data));
        assert_eq!(d.count(), 0);
    }

    // ── T19: get_by_status_mask with mask=0x00 returns nothing ───────────────
    #[test]
    fn t19_get_by_status_mask_zero_mask_returns_empty() {
        let mut d = dem();
        d.report_event(0x77_0000, true);
        let v: Vec<_> = d.get_by_status_mask(0x00).collect();
        assert!(v.is_empty());
    }

    // ── T20: serialize returns 0 for buffer too small ─────────────────────────
    #[test]
    fn t20_serialize_returns_zero_on_small_buffer() {
        let mut d = dem();
        d.report_event(0x11_0000, true);
        let mut tiny = [0u8; 4]; // needs 2 + 8 = 10 bytes
        let written = d.serialize(&mut tiny);
        assert_eq!(written, 0);
    }
}
