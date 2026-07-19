//! Audited Cortex-M safety hardware adapter.
//!
//! The MPU setup is a defense-in-depth baseline for privileged bare-metal
//! execution. It does not create process isolation and is not certification
//! evidence by itself.

use core::sync::atomic::{AtomicU32, Ordering};

use bsw_safety::{EccCapabilities, MpuConfig, MpuRegion, MpuRegionKind};

const MPU_BASE: usize = 0xE000_ED90;
const MPU_CTRL: usize = 0x04;
const MPU_RNR: usize = 0x08;
const MPU_RBAR: usize = 0x0C;
const MPU_RASR: usize = 0x10;

#[cfg(feature = "stm32f413")]
const REGIONS: [MpuRegion; 4] = [
    MpuRegion {
        base: 0x0800_0000,
        size: 0x0020_0000,
        kind: MpuRegionKind::ReadOnlyExecutable,
    },
    MpuRegion {
        base: 0x2000_0000,
        size: 0x0008_0000,
        kind: MpuRegionKind::ReadWriteNoExecute,
    },
    MpuRegion {
        base: 0x4000_0000,
        size: 0x2000_0000,
        kind: MpuRegionKind::DeviceNoExecute,
    },
    MpuRegion {
        base: 0x0814_0000,
        size: 0x0004_0000,
        kind: MpuRegionKind::ReadWriteNoExecute,
    },
];

#[cfg(feature = "stm32g474")]
const REGIONS: [MpuRegion; 4] = [
    MpuRegion {
        base: 0x0800_0000,
        size: 0x0008_0000,
        kind: MpuRegionKind::ReadOnlyExecutable,
    },
    MpuRegion {
        base: 0x2000_0000,
        size: 0x0002_0000,
        kind: MpuRegionKind::ReadWriteNoExecute,
    },
    MpuRegion {
        base: 0x4000_0000,
        size: 0x2000_0000,
        kind: MpuRegionKind::DeviceNoExecute,
    },
    MpuRegion {
        base: 0x0807_E000,
        size: 0x0000_2000,
        kind: MpuRegionKind::ReadWriteNoExecute,
    },
];

/// Validate and install the fixed per-MCU MPU regions.
pub fn install_mpu(_token: &mut cortex_m::peripheral::MPU) -> Result<(), bsw_safety::MpuError> {
    let config = MpuConfig::validate(&REGIONS)?;
    // SAFETY: the Cortex-M peripheral token proves unique configuration
    // ownership and every region has passed alignment/overlap validation.
    unsafe {
        crate::mmio::write((MPU_BASE + MPU_CTRL) as *mut u32, 0);
        for (index, region) in config.regions().iter().enumerate() {
            crate::mmio::write((MPU_BASE + MPU_RNR) as *mut u32, index as u32);
            crate::mmio::write((MPU_BASE + MPU_RBAR) as *mut u32, region.base);
            crate::mmio::write((MPU_BASE + MPU_RASR) as *mut u32, rasr(region));
        }
        // Enable MPU and the privileged default memory map. HardFault/NMI keep
        // the default map so failure capture remains available.
        crate::mmio::write((MPU_BASE + MPU_CTRL) as *mut u32, (1 << 2) | 1);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
    Ok(())
}

fn rasr(region: &MpuRegion) -> u32 {
    let size_field = region.size.trailing_zeros() - 1;
    let (xn, ap, tex_scb) = match region.kind {
        MpuRegionKind::ReadOnlyExecutable => (0, 0b110, 0b000_010),
        MpuRegionKind::ReadOnlyData => (1, 0b110, 0b000_010),
        MpuRegionKind::ReadWriteNoExecute => (1, 0b011, 0b000_010),
        MpuRegionKind::DeviceNoExecute => (1, 0b011, 0b000_101),
    };
    (xn << 28) | (ap << 24) | (tex_scb << 16) | (size_field << 1) | 1
}

static ISR_DEPTH: AtomicU32 = AtomicU32::new(0);

/// Common ISR pre-hook used for balanced-entry diagnostics.
pub fn isr_pre_hook() {
    ISR_DEPTH.fetch_add(1, Ordering::AcqRel);
}
/// Common ISR post-hook. Returns false if an unmatched exit was attempted.
pub fn isr_post_hook() -> bool {
    ISR_DEPTH
        .fetch_update(Ordering::AcqRel, Ordering::Acquire, |depth| {
            depth.checked_sub(1)
        })
        .is_ok()
}
pub fn isr_depth() -> u32 {
    ISR_DEPTH.load(Ordering::Acquire)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EccObservation {
    pub corrected_flash_error: bool,
    pub uncorrectable_flash_error: bool,
}

#[cfg(feature = "stm32f413")]
pub const ECC_CAPABILITIES: EccCapabilities = EccCapabilities {
    flash_ecc_reporting: false,
    ram_ecc_reporting: false,
    injection_supported: false,
    limitation:
        "STM32F413 main flash/SRAM expose no production-qualified ECC diagnostic path in this port",
};

#[cfg(feature = "stm32g474")]
pub const ECC_CAPABILITIES: EccCapabilities = EccCapabilities {
    flash_ecc_reporting: true, ram_ecc_reporting: false, injection_supported: false,
    limitation: "STM32G474 FLASH_ECCR status is reported; RAM ECC and safe fault injection are not supported",
};

#[cfg(feature = "stm32f413")]
pub fn observe_ecc() -> EccObservation {
    EccObservation {
        corrected_flash_error: false,
        uncorrectable_flash_error: false,
    }
}

#[cfg(feature = "stm32g474")]
pub fn observe_ecc() -> EccObservation {
    // SAFETY: FLASH_ECCR is a read-only observation here; W1C clearing is left
    // to the fault owner so evidence cannot be lost by a diagnostic read.
    let eccr = unsafe { crate::mmio::read(0x4002_2018 as *const u32) };
    EccObservation {
        corrected_flash_error: eccr & (1 << 30) != 0,
        uncorrectable_flash_error: eccr & (1 << 31) != 0,
    }
}

unsafe extern "C" {
    static _stext: u8;
    static __etext: u8;
}

/// Patched after linking by the release finalizer. It intentionally sits
/// outside the executable interval covered by the checksum.
#[used]
#[unsafe(link_section = ".openbsw.rom-crc")]
pub static ROM_CRC_EXPECTED: u32 = u32::MAX;

/// Return the linker-bounded executable ROM slice for incremental CRC.
pub fn executable_rom() -> &'static [u8] {
    let start = core::ptr::addr_of!(_stext) as usize;
    let end = core::ptr::addr_of!(__etext) as usize;
    if end <= start {
        return &[];
    }
    // SAFETY: the linker owns both symbols and the immutable flash interval.
    unsafe { core::slice::from_raw_parts(start as *const u8, end - start) }
}

/// Expected CRC embedded into the selected release ELF.
#[must_use]
pub fn expected_rom_crc() -> Option<u32> {
    let value = core::ptr::addr_of!(ROM_CRC_EXPECTED);
    // SAFETY: the linker keeps one aligned immutable u32 in flash.
    let expected = unsafe { core::ptr::read_volatile(value) };
    (expected != u32::MAX).then_some(expected)
}
