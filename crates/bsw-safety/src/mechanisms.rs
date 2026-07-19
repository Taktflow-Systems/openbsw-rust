//! Watchdog fast-test, retained handoff, ROM CRC, MPU, and ECC capabilities.

use bsw_time::{Duration, Instant};
use bsw_util::crc::{CrcDigest32, CRC32_ETHERNET};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastTestState {
    Idle,
    Armed,
    AwaitingReset,
    Passed,
    Failed,
}

pub trait WatchdogFastTestBackend {
    fn arm_shortest_timeout(&mut self) -> bool;
    fn reset_marker_observed(&self) -> bool;
    fn request_controlled_reset(&mut self);
    fn retained_state(&self) -> Option<RetainedWatchdogFastTest>;
    fn store_retained_state(&mut self, state: RetainedWatchdogFastTest);
}

const WATCHDOG_RETAINED_MAGIC: u32 = 0x5744_5446;
const WATCHDOG_RETAINED_VERSION: u16 = 1;

/// Reset-retained watchdog proof state. A completed or exhausted state is
/// retained so ordinary software resets cannot restart an unbounded test loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetainedWatchdogFastTest {
    pub attempts: u8,
    pub pending: bool,
    pub passed: bool,
}

impl RetainedWatchdogFastTest {
    pub const ENCODED_LEN: usize = 12;

    pub fn encode(self) -> [u8; Self::ENCODED_LEN] {
        let mut out = [0u8; Self::ENCODED_LEN];
        out[0..4].copy_from_slice(&WATCHDOG_RETAINED_MAGIC.to_le_bytes());
        out[4..6].copy_from_slice(&WATCHDOG_RETAINED_VERSION.to_le_bytes());
        out[6] = self.attempts;
        out[7] = u8::from(self.pending) | (u8::from(self.passed) << 1);
        let crc = CRC32_ETHERNET.checksum(&out[..8]);
        out[8..12].copy_from_slice(&crc.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8; Self::ENCODED_LEN]) -> Option<Self> {
        if u32::from_le_bytes(bytes[0..4].try_into().ok()?) != WATCHDOG_RETAINED_MAGIC
            || u16::from_le_bytes(bytes[4..6].try_into().ok()?) != WATCHDOG_RETAINED_VERSION
            || u32::from_le_bytes(bytes[8..12].try_into().ok()?)
                != CRC32_ETHERNET.checksum(&bytes[..8])
        {
            return None;
        }
        Some(Self {
            attempts: bytes[6],
            pending: bytes[7] & 1 != 0,
            passed: bytes[7] & 2 != 0,
        })
    }
}

pub struct WatchdogFastTest {
    state: FastTestState,
    deadline: Instant,
    timeout: Duration,
    attempts: u8,
    max_attempts: u8,
}
impl WatchdogFastTest {
    pub const fn new(timeout: Duration, max_attempts: u8) -> Self {
        Self {
            state: FastTestState::Idle,
            deadline: Instant::from_nanos(0),
            timeout,
            attempts: 0,
            max_attempts,
        }
    }
    pub fn start<B: WatchdogFastTestBackend>(
        &mut self,
        now: Instant,
        backend: &mut B,
    ) -> FastTestState {
        let retained = backend
            .retained_state()
            .unwrap_or(RetainedWatchdogFastTest {
                attempts: 0,
                pending: false,
                passed: false,
            });
        self.attempts = retained.attempts;
        if retained.passed {
            self.state = FastTestState::Passed;
            return self.state;
        }
        if self.max_attempts == 0 || self.attempts >= self.max_attempts {
            self.state = FastTestState::Failed;
            return self.state;
        }
        self.attempts = self.attempts.saturating_add(1);
        backend.store_retained_state(RetainedWatchdogFastTest {
            attempts: self.attempts,
            pending: true,
            passed: false,
        });
        if !backend.arm_shortest_timeout() {
            backend.store_retained_state(RetainedWatchdogFastTest {
                attempts: self.attempts,
                pending: false,
                passed: false,
            });
            self.state = FastTestState::Failed;
            return self.state;
        }
        self.deadline = now.wrapping_add(self.timeout);
        self.state = FastTestState::AwaitingReset;
        self.state
    }
    pub fn resume_after_boot<B: WatchdogFastTestBackend>(
        &mut self,
        backend: &mut B,
    ) -> FastTestState {
        let retained = backend
            .retained_state()
            .unwrap_or(RetainedWatchdogFastTest {
                attempts: 0,
                pending: false,
                passed: false,
            });
        self.attempts = retained.attempts;
        self.state = if retained.passed {
            FastTestState::Passed
        } else if retained.pending && backend.reset_marker_observed() {
            backend.store_retained_state(RetainedWatchdogFastTest {
                attempts: retained.attempts,
                pending: false,
                passed: true,
            });
            FastTestState::Passed
        } else if retained.pending && retained.attempts < self.max_attempts {
            FastTestState::Idle
        } else if retained.attempts >= self.max_attempts {
            backend.store_retained_state(RetainedWatchdogFastTest {
                attempts: retained.attempts,
                pending: false,
                passed: false,
            });
            FastTestState::Failed
        } else {
            FastTestState::Idle
        };
        self.state
    }
    pub fn poll<B: WatchdogFastTestBackend>(
        &mut self,
        now: Instant,
        backend: &mut B,
    ) -> FastTestState {
        if self.state == FastTestState::AwaitingReset && now.is_at_or_after(self.deadline) {
            backend.request_controlled_reset();
            self.state = FastTestState::Failed;
        }
        self.state
    }
    pub const fn state(&self) -> FastTestState {
        self.state
    }
}

const RETAINED_MAGIC: u32 = 0x5341_4645;
const RETAINED_VERSION: u16 = 1;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetainedSafetyEvent {
    pub code: u16,
    pub detail: u16,
    pub timestamp_ns: u64,
}
impl RetainedSafetyEvent {
    pub const ENCODED_LEN: usize = 24;
    pub fn encode(self) -> [u8; Self::ENCODED_LEN] {
        let mut out = [0u8; Self::ENCODED_LEN];
        out[0..4].copy_from_slice(&RETAINED_MAGIC.to_le_bytes());
        out[4..6].copy_from_slice(&RETAINED_VERSION.to_le_bytes());
        out[6..8].copy_from_slice(&self.code.to_le_bytes());
        out[8..10].copy_from_slice(&self.detail.to_le_bytes());
        out[12..20].copy_from_slice(&self.timestamp_ns.to_le_bytes());
        let crc = CRC32_ETHERNET.checksum(&out[..20]);
        out[20..24].copy_from_slice(&crc.to_le_bytes());
        out
    }
    pub fn decode(bytes: &[u8; Self::ENCODED_LEN]) -> Option<Self> {
        if u32::from_le_bytes(bytes[0..4].try_into().ok()?) != RETAINED_MAGIC
            || u16::from_le_bytes(bytes[4..6].try_into().ok()?) != RETAINED_VERSION
        {
            return None;
        }
        let stored = u32::from_le_bytes(bytes[20..24].try_into().ok()?);
        (stored == CRC32_ETHERNET.checksum(&bytes[..20])).then(|| Self {
            code: u16::from_le_bytes([bytes[6], bytes[7]]),
            detail: u16::from_le_bytes([bytes[8], bytes[9]]),
            timestamp_ns: u64::from_le_bytes(bytes[12..20].try_into().unwrap()),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RomCheckStatus {
    InProgress,
    Passed,
    Failed,
}
pub struct RomCrcChecker<'a> {
    region: &'a [u8],
    expected: u32,
    offset: usize,
    chunk: usize,
    digest: CrcDigest32<'static>,
    status: RomCheckStatus,
}
impl<'a> RomCrcChecker<'a> {
    pub fn new(region: &'a [u8], expected: u32, chunk: usize) -> Option<Self> {
        (chunk > 0 && !region.is_empty()).then(|| Self {
            region,
            expected,
            offset: 0,
            chunk,
            digest: CRC32_ETHERNET.digest(),
            status: RomCheckStatus::InProgress,
        })
    }
    pub fn step(&mut self) -> RomCheckStatus {
        if self.status != RomCheckStatus::InProgress {
            return self.status;
        }
        let end = (self.offset + self.chunk).min(self.region.len());
        self.digest.update(&self.region[self.offset..end]);
        self.offset = end;
        if end == self.region.len() {
            let digest = core::mem::replace(&mut self.digest, CRC32_ETHERNET.digest()).finalize();
            self.status = if digest == self.expected {
                RomCheckStatus::Passed
            } else {
                RomCheckStatus::Failed
            };
        }
        self.status
    }
    pub const fn checked_bytes(&self) -> usize {
        self.offset
    }
    pub const fn status(&self) -> RomCheckStatus {
        self.status
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuRegionKind {
    ReadOnlyExecutable,
    ReadOnlyData,
    ReadWriteNoExecute,
    DeviceNoExecute,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MpuRegion {
    pub base: u32,
    pub size: u32,
    pub kind: MpuRegionKind,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuError {
    Empty,
    NotPowerOfTwo,
    Misaligned,
    Overlap,
    WritableExecutable,
}
pub struct MpuConfig<'a> {
    regions: &'a [MpuRegion],
}
impl<'a> MpuConfig<'a> {
    pub fn validate(regions: &'a [MpuRegion]) -> Result<Self, MpuError> {
        for (i, a) in regions.iter().enumerate() {
            if a.size == 0 {
                return Err(MpuError::Empty);
            }
            if !a.size.is_power_of_two() {
                return Err(MpuError::NotPowerOfTwo);
            }
            if a.base & (a.size - 1) != 0 {
                return Err(MpuError::Misaligned);
            }
            for b in &regions[..i] {
                let overlaps = a.base < b.base.saturating_add(b.size)
                    && b.base < a.base.saturating_add(a.size);
                let bounded_write_overlay = a.kind == MpuRegionKind::ReadWriteNoExecute
                    && a.base >= b.base
                    && a.base.saturating_add(a.size) <= b.base.saturating_add(b.size);
                if overlaps && !bounded_write_overlay {
                    return Err(MpuError::Overlap);
                }
            }
        }
        Ok(Self { regions })
    }
    pub const fn regions(&self) -> &'a [MpuRegion] {
        self.regions
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EccCapabilities {
    pub flash_ecc_reporting: bool,
    pub ram_ecc_reporting: bool,
    pub injection_supported: bool,
    pub limitation: &'static str,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[derive(Default)]
    struct Backend {
        armed: bool,
        marker: bool,
        reset: u8,
        retained: Option<RetainedWatchdogFastTest>,
    }
    impl WatchdogFastTestBackend for Backend {
        fn arm_shortest_timeout(&mut self) -> bool {
            self.armed = true;
            true
        }
        fn reset_marker_observed(&self) -> bool {
            self.marker
        }
        fn request_controlled_reset(&mut self) {
            self.reset += 1;
        }
        fn retained_state(&self) -> Option<RetainedWatchdogFastTest> {
            self.retained
        }
        fn store_retained_state(&mut self, state: RetainedWatchdogFastTest) {
            self.retained = Some(state);
        }
    }
    #[test]
    fn watchdog_fast_test_success_and_bounded_failure() {
        let mut b = Backend::default();
        let mut t = WatchdogFastTest::new(Duration::from_nanos(10), 1);
        assert_eq!(
            t.start(Instant::from_nanos(5), &mut b),
            FastTestState::AwaitingReset
        );
        assert_eq!(
            t.poll(Instant::from_nanos(14), &mut b),
            FastTestState::AwaitingReset
        );
        assert_eq!(
            t.poll(Instant::from_nanos(15), &mut b),
            FastTestState::Failed
        );
        assert_eq!(b.reset, 1);
        b.marker = true;
        assert_eq!(t.resume_after_boot(&mut b), FastTestState::Passed);
        assert_eq!(
            b.retained,
            Some(RetainedWatchdogFastTest {
                attempts: 1,
                pending: false,
                passed: true
            })
        );
        let mut next_boot = WatchdogFastTest::new(Duration::from_nanos(10), 1);
        b.marker = false;
        assert_eq!(next_boot.resume_after_boot(&mut b), FastTestState::Passed);
    }
    #[test]
    fn retained_watchdog_state_rejects_corruption() {
        let state = RetainedWatchdogFastTest {
            attempts: 2,
            pending: true,
            passed: false,
        };
        let mut raw = state.encode();
        assert_eq!(RetainedWatchdogFastTest::decode(&raw), Some(state));
        raw[6] ^= 1;
        assert_eq!(RetainedWatchdogFastTest::decode(&raw), None);
    }
    #[test]
    fn retained_integrity_and_version_are_rejected() {
        let e = RetainedSafetyEvent {
            code: 2,
            detail: 3,
            timestamp_ns: 4,
        };
        let mut raw = e.encode();
        assert_eq!(RetainedSafetyEvent::decode(&raw), Some(e));
        raw[8] ^= 1;
        assert_eq!(RetainedSafetyEvent::decode(&raw), None);
    }
    #[test]
    fn rom_crc_is_incremental_and_detects_corruption() {
        let image = b"incremental-rom-image";
        let expected = CRC32_ETHERNET.checksum(image);
        let mut c = RomCrcChecker::new(image, expected, 3).unwrap();
        while c.step() == RomCheckStatus::InProgress {}
        assert_eq!(c.status(), RomCheckStatus::Passed);
        let mut bad = RomCrcChecker::new(image, expected ^ 1, 4).unwrap();
        while bad.step() == RomCheckStatus::InProgress {}
        assert_eq!(bad.status(), RomCheckStatus::Failed);
    }
    #[test]
    fn mpu_rejects_overlap_and_accepts_bounded_regions() {
        let valid = [
            MpuRegion {
                base: 0x0800_0000,
                size: 0x10000,
                kind: MpuRegionKind::ReadOnlyExecutable,
            },
            MpuRegion {
                base: 0x2000_0000,
                size: 0x10000,
                kind: MpuRegionKind::ReadWriteNoExecute,
            },
        ];
        assert!(MpuConfig::validate(&valid).is_ok());
        let overlap = [
            valid[0],
            MpuRegion {
                base: 0x0800_8000,
                size: 0x8000,
                kind: MpuRegionKind::ReadOnlyData,
            },
        ];
        assert_eq!(MpuConfig::validate(&overlap).err(), Some(MpuError::Overlap));
    }
}
