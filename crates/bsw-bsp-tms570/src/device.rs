//! Exact TMS570LC4357 revision-B memory and flash geometry.
//!
//! Constants in this module come from SPNS195C tables 6-25, 6-29 and
//! 6-52. They are deliberately not presented as a family-wide TMS570 map.

/// Half-open address region in the 32-bit physical address space.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemoryRegion {
    /// First included address.
    pub start: u32,
    /// Address immediately after the region.
    pub end: u32,
}

impl MemoryRegion {
    /// Create a half-open region.
    pub const fn new(start: u32, end: u32) -> Self {
        assert!(start < end);
        Self { start, end }
    }

    /// Region size in bytes.
    pub const fn len(self) -> u32 {
        self.end - self.start
    }

    /// Whether the region is empty. Valid device regions are never empty.
    pub const fn is_empty(self) -> bool {
        self.start == self.end
    }

    /// Whether `address` is inside the region.
    pub const fn contains(self, address: u32) -> bool {
        address >= self.start && address < self.end
    }

    /// Whether the non-empty range `[start, start + length)` is contained.
    pub const fn contains_range(self, start: u32, length: u32) -> bool {
        if length == 0 || start < self.start {
            return false;
        }
        match start.checked_add(length) {
            Some(end) => end <= self.end,
            None => false,
        }
    }
}

/// Public JTAG identification code for TMS570LC4357 silicon revision B.
pub const JTAG_IDCODE_REV_B: u32 = 0x1b95_a02f;
/// Memory-mapped device identification register.
pub const DEVICE_ID_REGISTER: u32 = 0xffff_fff0;
/// Public device identification value for revision B.
pub const DEVICE_ID_REV_B: u32 = 0x8044_ad0d;

/// Main executable flash, banks 0 and 1.
pub const MAIN_FLASH: MemoryRegion = MemoryRegion::new(0x0000_0000, 0x0040_0000);
/// Implemented level-2 SRAM/TCRAM.
pub const SRAM: MemoryRegion = MemoryRegion::new(0x0800_0000, 0x0808_0000);
/// Level-2 SRAM ECC alias.
pub const SRAM_ECC: MemoryRegion = MemoryRegion::new(0x0840_0000, 0x0848_0000);
/// Implemented data-flash bank 7.
pub const DATA_FLASH: MemoryRegion = MemoryRegion::new(0xf020_0000, 0xf022_0000);
/// EMAC CPPI descriptor RAM.
pub const EMAC_CPPI_RAM: MemoryRegion = MemoryRegion::new(0xfc52_0000, 0xfc52_2000);
/// EMAC register frame.
pub const EMAC_REGISTERS: MemoryRegion = MemoryRegion::new(0xfcf7_8000, 0xfcf7_8800);
/// EMAC subsystem wrapper register frame.
pub const EMAC_WRAPPER: MemoryRegion = MemoryRegion::new(0xfcf7_8800, 0xfcf7_8900);
/// MDIO register frame.
pub const MDIO_REGISTERS: MemoryRegion = MemoryRegion::new(0xfcf7_8900, 0xfcf7_8a00);
/// VIM vector RAM frame.
pub const VIM_RAM: MemoryRegion = MemoryRegion::new(0xfff8_2000, 0xfff8_3000);
/// RTI and digital-windowed watchdog registers.
pub const RTI_DWWD_REGISTERS: MemoryRegion = MemoryRegion::new(0xffff_fc00, 0xffff_fd00);
/// VIM control registers.
pub const VIM_REGISTERS: MemoryRegion = MemoryRegion::new(0xffff_fd00, 0xffff_ff00);
/// Level-2 SRAM wrapper registers.
pub const L2RAMW_REGISTERS: MemoryRegion = MemoryRegion::new(0xffff_f900, 0xffff_fa00);

const _: () = {
    assert!(MAIN_FLASH.len() == 4 * 1024 * 1024);
    assert!(SRAM.len() == 512 * 1024);
    assert!(SRAM_ECC.len() == SRAM.len());
    assert!(DATA_FLASH.len() == 128 * 1024);
    assert!(EMAC_CPPI_RAM.len() == 8 * 1024);
    assert!(EMAC_REGISTERS.end <= EMAC_WRAPPER.start);
    assert!(EMAC_WRAPPER.end <= MDIO_REGISTERS.start);
};

/// Exact main-flash sector description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlashSector {
    /// Bank number: 0 or 1.
    pub bank: u8,
    /// Sector number within the bank.
    pub sector: u8,
    /// Half-open address region.
    pub region: MemoryRegion,
}

/// Return one of the 32 main-flash sectors from SPNS195C table 6-29.
pub const fn main_flash_sector(index: usize) -> Option<FlashSector> {
    if index >= 32 {
        return None;
    }
    if index >= 16 {
        let sector = (index - 16) as u8;
        let start = 0x0020_0000 + sector as u32 * 0x0002_0000;
        return Some(FlashSector {
            bank: 1,
            sector,
            region: MemoryRegion::new(start, start + 0x0002_0000),
        });
    }
    let (start, length) = match index {
        0..=5 => (index as u32 * 0x0000_4000, 0x0000_4000),
        6 => (0x0001_8000, 0x0000_8000),
        7..=9 => (0x0002_0000 + (index as u32 - 7) * 0x0002_0000, 0x0002_0000),
        10..=15 => (0x0008_0000 + (index as u32 - 10) * 0x0004_0000, 0x0004_0000),
        _ => return None,
    };
    Some(FlashSector {
        bank: 0,
        sector: index as u8,
        region: MemoryRegion::new(start, start + length),
    })
}

/// Return one of the 32 4-KiB data-flash sectors in bank 7.
pub const fn data_flash_sector(index: usize) -> Option<FlashSector> {
    if index >= 32 {
        return None;
    }
    let start = DATA_FLASH.start + index as u32 * 0x1000;
    Some(FlashSector {
        bank: 7,
        sector: index as u8,
        region: MemoryRegion::new(start, start + 0x1000),
    })
}

/// Recognize only the proven revision-B JTAG code.
pub const fn is_supported_jtag_id(idcode: u32) -> bool {
    idcode == JTAG_IDCODE_REV_B
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ranges_reject_overflow_and_end_address() {
        assert!(SRAM.contains(SRAM.start));
        assert!(!SRAM.contains(SRAM.end));
        assert!(SRAM.contains_range(SRAM.start, SRAM.len()));
        assert!(!SRAM.contains_range(SRAM.start, 0));
        assert!(!SRAM.contains_range(u32::MAX - 3, 8));
    }

    #[test]
    fn main_flash_sectors_are_exact_and_contiguous() {
        let mut cursor = MAIN_FLASH.start;
        for index in 0..32 {
            let sector = main_flash_sector(index).unwrap();
            assert_eq!(sector.region.start, cursor);
            cursor = sector.region.end;
        }
        assert_eq!(cursor, MAIN_FLASH.end);
        assert!(main_flash_sector(32).is_none());
        assert_eq!(main_flash_sector(6).unwrap().region.len(), 32 * 1024);
        assert_eq!(main_flash_sector(10).unwrap().region.len(), 256 * 1024);
        assert_eq!(main_flash_sector(16).unwrap().region.len(), 128 * 1024);
    }

    #[test]
    fn data_flash_sectors_cover_only_implemented_bank() {
        let first = data_flash_sector(0).unwrap();
        let last = data_flash_sector(31).unwrap();
        assert_eq!(first.region.start, DATA_FLASH.start);
        assert_eq!(last.region.end, DATA_FLASH.end);
        assert!(data_flash_sector(32).is_none());
    }

    #[test]
    fn only_revision_b_id_is_supported() {
        assert!(is_supported_jtag_id(JTAG_IDCODE_REV_B));
        assert!(!is_supported_jtag_id(0x0b95_a02f));
    }
}
