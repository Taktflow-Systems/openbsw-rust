//! Board-independent LC4357 MDIO configuration helpers.

/// IEEE 802.3 MDIO maximum clock supported by the LC4357 module.
pub const MAX_MDIO_CLOCK_HZ: u32 = 2_500_000;
/// Number of PHY addresses on the MDIO bus.
pub const PHY_ADDRESS_COUNT: u8 = 32;

/// MDIO configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MdioError {
    /// Peripheral clock was zero.
    ClockStopped,
    /// A PHY address was outside the five-bit hardware field.
    InvalidPhyAddress,
    /// No PHY responded to the alive scan.
    NoPhy,
    /// More than one PHY responded; selecting by probability is forbidden.
    MultiplePhys,
}

/// Five-bit IEEE 802.3 PHY address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhyAddress(u8);

impl PhyAddress {
    /// Validate a PHY address.
    pub const fn new(address: u8) -> Result<Self, MdioError> {
        if address < PHY_ADDRESS_COUNT {
            Ok(Self(address))
        } else {
            Err(MdioError::InvalidPhyAddress)
        }
    }

    /// Raw address field.
    pub const fn get(self) -> u8 {
        self.0
    }
}

/// MDIO clock divider derived from VCLK3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MdioClock {
    peripheral_hz: u32,
    divider: u16,
}

impl MdioClock {
    /// Select the fastest non-zero divider no faster than 2.5 MHz.
    ///
    /// SPNU563A section 32.4.2 defines
    /// `MDIO_CLK = VCLK3 / (CLKDIV + 1)` and reserves divider zero for a
    /// disabled clock.
    pub const fn from_vclk3(peripheral_hz: u32) -> Result<Self, MdioError> {
        if peripheral_hz == 0 {
            return Err(MdioError::ClockStopped);
        }
        let ratio = peripheral_hz.div_ceil(MAX_MDIO_CLOCK_HZ);
        let mut divider = ratio.saturating_sub(1);
        if divider == 0 {
            divider = 1;
        }
        Ok(Self {
            peripheral_hz,
            divider: divider as u16,
        })
    }

    /// Value for the MDIO CONTROL.CLKDIV field.
    pub const fn divider(self) -> u16 {
        self.divider
    }

    /// Resulting integer MDIO clock frequency.
    pub const fn clock_hz(self) -> u32 {
        self.peripheral_hz / (self.divider as u32 + 1)
    }
}

/// Decode a hardware ALIVE mask only when exactly one PHY responded.
pub fn single_alive_phy(alive_mask: u32) -> Result<PhyAddress, MdioError> {
    match alive_mask.count_ones() {
        0 => Err(MdioError::NoPhy),
        1 => PhyAddress::new(alive_mask.trailing_zeros() as u8),
        _ => Err(MdioError::MultiplePhys),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mdio_clock_never_exceeds_limit_or_uses_disabled_divider() {
        for peripheral_hz in [1_000_000, 20_000_000, 75_000_000, 150_000_000] {
            let clock = MdioClock::from_vclk3(peripheral_hz).unwrap();
            assert_ne!(clock.divider(), 0);
            assert!(clock.clock_hz() <= MAX_MDIO_CLOCK_HZ);
        }
        assert_eq!(MdioClock::from_vclk3(0), Err(MdioError::ClockStopped));
    }

    #[test]
    fn phy_selection_refuses_ambiguity() {
        assert_eq!(single_alive_phy(0), Err(MdioError::NoPhy));
        assert_eq!(single_alive_phy(1 << 7).unwrap().get(), 7);
        assert_eq!(single_alive_phy(0b11), Err(MdioError::MultiplePhys));
        assert_eq!(PhyAddress::new(32), Err(MdioError::InvalidPhyAddress));
    }
}
