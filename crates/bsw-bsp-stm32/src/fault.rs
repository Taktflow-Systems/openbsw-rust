//! HardFault handler — register dump to NO_INIT RAM for post-mortem.
//!
//! Captures the full CPU state (R0-R12, SP, LR, PC, xPSR) plus 240 bytes
//! of stack content into a fixed RAM region with XOR checksum.
//!
//! Maps to C++: `contribution/platforms/stm32/hardFaultHandler/src/hardFaultHandler.s`

use core::ptr;

unsafe extern "C" {
    /// Supplied by the selected board linker script; the whole 1 KiB NOINIT
    /// region has one owner in this module.
    static mut _noinit_start: u32;
}

/// Size of the fault dump in bytes (0x140 = 320 bytes).
pub const DUMP_SIZE: usize = 0x140;

/// Number of stack-content bytes captured after the exception frame.
pub const STACK_CAPTURE_BYTES: usize = 240;
const SAFETY_EVENT_OFFSET: usize = DUMP_SIZE;
const WATCHDOG_TEST_OFFSET: usize =
    SAFETY_EVENT_OFFSET + bsw_safety::RetainedSafetyEvent::ENCODED_LEN;
const STORAGE_CAMPAIGN_OFFSET: usize =
    WATCHDOG_TEST_OFFSET + bsw_safety::RetainedWatchdogFastTest::ENCODED_LEN;

/// Fault dump structure layout offsets (in 32-bit word indices).
#[allow(dead_code)]
mod offset {
    /// Handler xPSR at dump start.
    pub const HANDLER_PSR: usize = 0;
    /// Exception return LR (EXC_RETURN).
    pub const HANDLER_LR: usize = 1;
    /// Origin R0 (from exception frame).
    pub const ORIGIN_R0: usize = 2;
    /// Origin R1.
    pub const ORIGIN_R1: usize = 3;
    /// Origin R2.
    pub const ORIGIN_R2: usize = 4;
    /// Origin R3.
    pub const ORIGIN_R3: usize = 5;
    /// Origin R4 (manually saved).
    pub const ORIGIN_R4: usize = 6;
    /// Origin R5.
    pub const ORIGIN_R5: usize = 7;
    /// Origin R6.
    pub const ORIGIN_R6: usize = 8;
    /// Origin R7.
    pub const ORIGIN_R7: usize = 9;
    /// Origin R8.
    pub const ORIGIN_R8: usize = 10;
    /// Origin R9.
    pub const ORIGIN_R9: usize = 11;
    /// Origin R10.
    pub const ORIGIN_R10: usize = 12;
    /// Origin R11.
    pub const ORIGIN_R11: usize = 13;
    /// Origin R12 (from exception frame).
    pub const ORIGIN_R12: usize = 14;
    /// Origin SP (calculated: frame SP + 0x20).
    pub const ORIGIN_SP: usize = 15;
    /// Origin LR (from exception frame).
    pub const ORIGIN_LR: usize = 16;
    /// Origin PC (from exception frame — the faulting instruction).
    pub const ORIGIN_PC: usize = 17;
    /// Origin xPSR (from exception frame).
    pub const ORIGIN_PSR: usize = 18;
    /// Start of captured stack contents (60 words = 240 bytes).
    pub const STACK_START: usize = 19;
    /// XOR checksum (word index = DUMP_SIZE/4 - 1 = 79).
    pub const CHECKSUM: usize = DUMP_SIZE_WORDS - 1;

    const DUMP_SIZE_WORDS: usize = super::DUMP_SIZE / 4;
}

/// Write the fault dump to NO_INIT RAM.
///
/// # Safety
/// Called from the HardFault exception handler. `ef` must point to a valid
/// Cortex-M exception frame on the appropriate stack (MSP or PSP).
#[allow(clippy::too_many_lines)]
unsafe fn write_fault_dump(ef: &cortex_m_rt::ExceptionFrame) {
    let dump = core::ptr::addr_of_mut!(_noinit_start);
    let words = DUMP_SIZE / 4;

    // Zero the dump region
    for i in 0..words {
        unsafe { ptr::write_volatile(dump.add(i), 0) };
    }

    // Handler PSR
    let psr: u32;
    unsafe { core::arch::asm!("mrs {}, xpsr", out(reg) psr) };
    unsafe { ptr::write_volatile(dump.add(offset::HANDLER_PSR), psr) };

    // Exception return LR
    let exc_lr: u32;
    unsafe { core::arch::asm!("mov {}, lr", out(reg) exc_lr) };
    unsafe { ptr::write_volatile(dump.add(offset::HANDLER_LR), exc_lr) };

    // R0-R3 from exception frame (stacked by hardware)
    unsafe {
        ptr::write_volatile(dump.add(offset::ORIGIN_R0), ef.r0());
        ptr::write_volatile(dump.add(offset::ORIGIN_R1), ef.r1());
        ptr::write_volatile(dump.add(offset::ORIGIN_R2), ef.r2());
        ptr::write_volatile(dump.add(offset::ORIGIN_R3), ef.r3());
    }

    // R12, LR, PC, xPSR from exception frame
    unsafe {
        ptr::write_volatile(dump.add(offset::ORIGIN_R12), ef.r12());
        ptr::write_volatile(dump.add(offset::ORIGIN_LR), ef.lr());
        ptr::write_volatile(dump.add(offset::ORIGIN_PC), ef.pc());
        ptr::write_volatile(dump.add(offset::ORIGIN_PSR), ef.xpsr());
    }

    // Calculate original SP (exception frame pointer + 8 words)
    let frame_ptr = ef as *const cortex_m_rt::ExceptionFrame as u32;
    let original_sp = frame_ptr.wrapping_add(0x20); // 8 words = 32 bytes
    unsafe { ptr::write_volatile(dump.add(offset::ORIGIN_SP), original_sp) };

    // Capture stack contents (240 bytes / 60 words after the exception frame)
    let stack_ptr = original_sp as *const u32;
    let stack_words = STACK_CAPTURE_BYTES / 4;
    for i in 0..stack_words {
        let val = unsafe { ptr::read_volatile(stack_ptr.add(i)) };
        unsafe { ptr::write_volatile(dump.add(offset::STACK_START + i), val) };
    }

    // XOR checksum over all words except the checksum word itself
    let mut checksum: u32 = 0;
    for i in 0..(words - 1) {
        checksum ^= unsafe { ptr::read_volatile(dump.add(i)) };
    }
    unsafe { ptr::write_volatile(dump.add(offset::CHECKSUM), checksum) };

    let retained = bsw_safety::RetainedSafetyEvent {
        code: bsw_safety::SafetyEventCode::HardFault as u16,
        detail: (ef.pc() & 0xffff) as u16,
        timestamp_ns: 0,
    };
    write_retained_safety_event(retained);
}

fn safety_event_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(_noinit_start)
        .cast::<u8>()
        .wrapping_add(SAFETY_EVENT_OFFSET)
}

/// Store a versioned, CRC-protected event in linker-owned no-init RAM.
pub fn write_retained_safety_event(event: bsw_safety::RetainedSafetyEvent) {
    let encoded = event.encode();
    for (index, byte) in encoded.iter().copied().enumerate() {
        // SAFETY: offset 320..344 is inside the exclusive 1 KiB NOINIT region.
        unsafe { ptr::write_volatile(safety_event_ptr().add(index), byte) };
    }
}

/// Validate and return the retained safety event without clearing it.
pub fn retained_safety_event() -> Option<bsw_safety::RetainedSafetyEvent> {
    let mut encoded = [0u8; bsw_safety::RetainedSafetyEvent::ENCODED_LEN];
    for (index, byte) in encoded.iter_mut().enumerate() {
        // SAFETY: offset 320..344 is inside the exclusive 1 KiB NOINIT region.
        *byte = unsafe { ptr::read_volatile(safety_event_ptr().add(index)) };
    }
    bsw_safety::RetainedSafetyEvent::decode(&encoded)
}

/// Clear the retained event only after durable handoff.
pub fn clear_retained_safety_event() {
    for index in 0..bsw_safety::RetainedSafetyEvent::ENCODED_LEN {
        // SAFETY: offset 320..344 is inside the exclusive 1 KiB NOINIT region.
        unsafe { ptr::write_volatile(safety_event_ptr().add(index), 0) };
    }
}

fn watchdog_test_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(_noinit_start)
        .cast::<u8>()
        .wrapping_add(WATCHDOG_TEST_OFFSET)
}

pub fn write_retained_watchdog_test(state: bsw_safety::RetainedWatchdogFastTest) {
    let encoded = state.encode();
    for (index, byte) in encoded.iter().copied().enumerate() {
        // SAFETY: this record follows the safety event inside exclusive NOINIT.
        unsafe { ptr::write_volatile(watchdog_test_ptr().add(index), byte) };
    }
}

pub fn retained_watchdog_test() -> Option<bsw_safety::RetainedWatchdogFastTest> {
    let mut encoded = [0u8; bsw_safety::RetainedWatchdogFastTest::ENCODED_LEN];
    for (index, byte) in encoded.iter_mut().enumerate() {
        // SAFETY: this record follows the safety event inside exclusive NOINIT.
        *byte = unsafe { ptr::read_volatile(watchdog_test_ptr().add(index)) };
    }
    bsw_safety::RetainedWatchdogFastTest::decode(&encoded)
}

pub fn clear_retained_watchdog_test() {
    for index in 0..bsw_safety::RetainedWatchdogFastTest::ENCODED_LEN {
        // SAFETY: this record follows the safety event inside exclusive NOINIT.
        unsafe { ptr::write_volatile(watchdog_test_ptr().add(index), 0) };
    }
}

const STORAGE_CAMPAIGN_MAGIC: u32 = 0x4a43_414d;
const STORAGE_CAMPAIGN_VERSION: u8 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetainedStorageCampaign {
    pub board: u8,
    pub cut_index: u8,
    pub pending_recovery: bool,
    pub wear: [u16; 2],
}

impl RetainedStorageCampaign {
    const ENCODED_LEN: usize = 16;

    fn encode(self) -> [u8; Self::ENCODED_LEN] {
        use bsw_util::crc::CRC32_ETHERNET;
        let mut out = [0u8; Self::ENCODED_LEN];
        out[0..4].copy_from_slice(&STORAGE_CAMPAIGN_MAGIC.to_le_bytes());
        out[4] = STORAGE_CAMPAIGN_VERSION;
        out[5] = self.board;
        out[6] = self.cut_index;
        out[7] = u8::from(self.pending_recovery);
        out[8..10].copy_from_slice(&self.wear[0].to_le_bytes());
        out[10..12].copy_from_slice(&self.wear[1].to_le_bytes());
        let crc = CRC32_ETHERNET.checksum(&out[..12]);
        out[12..16].copy_from_slice(&crc.to_le_bytes());
        out
    }

    fn decode(bytes: &[u8; Self::ENCODED_LEN]) -> Option<Self> {
        use bsw_util::crc::CRC32_ETHERNET;
        if u32::from_le_bytes(bytes[0..4].try_into().ok()?) != STORAGE_CAMPAIGN_MAGIC
            || bytes[4] != STORAGE_CAMPAIGN_VERSION
            || u32::from_le_bytes(bytes[12..16].try_into().ok()?)
                != CRC32_ETHERNET.checksum(&bytes[..12])
        {
            return None;
        }
        Some(Self {
            board: bytes[5],
            cut_index: bytes[6],
            pending_recovery: bytes[7] == 1,
            wear: [
                u16::from_le_bytes([bytes[8], bytes[9]]),
                u16::from_le_bytes([bytes[10], bytes[11]]),
            ],
        })
    }
}

fn storage_campaign_ptr() -> *mut u8 {
    core::ptr::addr_of_mut!(_noinit_start)
        .cast::<u8>()
        .wrapping_add(STORAGE_CAMPAIGN_OFFSET)
}

pub fn write_retained_storage_campaign(state: RetainedStorageCampaign) {
    let encoded = state.encode();
    for (index, byte) in encoded.iter().copied().enumerate() {
        // SAFETY: the record is inside the exclusive linker-owned NOINIT area.
        unsafe { ptr::write_volatile(storage_campaign_ptr().add(index), byte) };
    }
}

pub fn retained_storage_campaign() -> Option<RetainedStorageCampaign> {
    let mut encoded = [0u8; RetainedStorageCampaign::ENCODED_LEN];
    for (index, byte) in encoded.iter_mut().enumerate() {
        // SAFETY: the record is inside the exclusive linker-owned NOINIT area.
        *byte = unsafe { ptr::read_volatile(storage_campaign_ptr().add(index)) };
    }
    RetainedStorageCampaign::decode(&encoded)
}

pub fn clear_retained_storage_campaign() {
    for index in 0..RetainedStorageCampaign::ENCODED_LEN {
        // SAFETY: the record is inside the exclusive linker-owned NOINIT area.
        unsafe { ptr::write_volatile(storage_campaign_ptr().add(index), 0) };
    }
}

/// Validate the retained dump after reset without changing it.
#[must_use]
pub fn fault_dump_valid() -> bool {
    let dump = core::ptr::addr_of_mut!(_noinit_start);
    let mut checksum = 0u32;
    for index in 0..offset::CHECKSUM {
        // SAFETY: linker guarantees the 1 KiB NOINIT region; the dump uses 320 B.
        checksum ^= unsafe { ptr::read_volatile(dump.add(index)) };
    }
    // SAFETY: checksum is the final word of the reserved dump.
    let stored = unsafe { ptr::read_volatile(dump.add(offset::CHECKSUM)) };
    stored != 0 && checksum == stored
}

/// Clear a retained dump after startup has persisted or reported it.
pub fn clear_fault_dump() {
    let dump = core::ptr::addr_of_mut!(_noinit_start);
    for index in 0..DUMP_SIZE / 4 {
        // SAFETY: linker guarantees the reserved NOINIT range.
        unsafe { ptr::write_volatile(dump.add(index), 0) };
    }
}

/// HardFault exception handler.
///
/// Dumps CPU state to linker-owned NO_INIT RAM, then requests reset so startup
/// can validate and persist the record.
#[cortex_m_rt::exception]
#[allow(non_snake_case)]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe { write_fault_dump(ef) };
    cortex_m::peripheral::SCB::sys_reset()
}
