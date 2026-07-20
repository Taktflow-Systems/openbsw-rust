//! Exact LAUNCHXL2-570LC43 board description.
//!
//! Board wiring comes from TI schematic SPRR397 revision A1, sheets 2 and 12.
//! DP83630 electrical and strap behavior comes from TI data sheet SNLS335B.
//! These constants describe wiring only; they do not perform pinmux, clock,
//! reset, MDIO, or EMAC register writes.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::mdio::PhyAddress;

/// Public TI board model selected by the Cargo feature.
pub const MODEL: &str = "LAUNCHXL2-570LC43";
/// Public schematic identity used for board wiring.
pub const SCHEMATIC: &str = "SPRR397 A1";
/// Crystal Y1 connected to the LC4357 OSCIN/OSCOUT pins.
pub const MCU_OSCILLATOR_HZ: u32 = 16_000_000;
/// External reference frequency required at DP83630 XIN in MII mode.
pub const PHY_REFERENCE_CLOCK_HZ: u32 = 25_000_000;
/// Width of each MII transmit and receive data path.
pub const MII_DATA_BITS: u8 = 4;
/// External board resistor used for the relevant DP83630 strap nets.
pub const PHY_STRAP_RESISTANCE_OHMS: u16 = 2_200;

/// Logic level for a board signal or physical pull.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// Logic zero.
    Low,
    /// Logic one.
    High,
}

/// LC4357 general-purpose I/O port used by a board signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GioPort {
    /// GIO port A.
    A,
}

/// Typed board-level GIO connection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GioLine {
    /// GIO port.
    pub port: GioPort,
    /// Zero-based pin within the port.
    pub pin: u8,
    /// Level that asserts the external function.
    pub asserted: Level,
    /// External board pull, excluding pulls internal to the connected device.
    pub board_pull: Option<Level>,
}

/// Ethernet MAC/PHY media interface selected by the board straps and wiring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MediaInterface {
    /// Four-bit 10/100 Media Independent Interface.
    Mii,
}

/// Ethernet PHY populated on the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyModel {
    /// Texas Instruments DP83630 Precision PHYTER.
    Dp83630,
}

/// MCU output routed to the PHY reference-clock input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyReferenceClockSource {
    /// LC4357 ECLK1T output routed to DP83630 XIN.
    Eclk1T,
}

/// Exact board capability declaration used by later platform adapters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EthernetBoard {
    /// MAC/PHY electrical interface.
    pub interface: MediaInterface,
    /// Populated PHY model.
    pub phy: PhyModel,
    /// PHY address latched by the board straps.
    pub phy_address: PhyAddress,
    /// PHY reference-clock source.
    pub reference_clock: PhyReferenceClockSource,
    /// PHY reset input.
    pub reset: GioLine,
    /// Shared PHY power-down input and programmable interrupt output.
    pub powerdown_interrupt: GioLine,
    /// Whether the PHY connects directly to the populated RJ-45 magnetics.
    pub direct_rj45: bool,
}

const PHY_ADDRESS_RAW: u8 = 1;

/// Board PHY address `00001b`; address zero would enter MII isolate mode.
pub const PHY_ADDRESS: PhyAddress = match PhyAddress::new(PHY_ADDRESS_RAW) {
    Ok(address) => address,
    Err(_) => panic!("LAUNCHXL2 PHY address must fit the MDIO field"),
};

/// DP83630 active-low reset routed to LC4357 GIOA[4].
pub const PHY_RESET: GioLine = GioLine {
    port: GioPort::A,
    pin: 4,
    asserted: Level::Low,
    board_pull: None,
};

/// DP83630 active-low power-down/interrupt routed to GIOA[3].
///
/// SPRR397 installs a 2.2-kOhm board pull-down, so early startup must keep the
/// PHY intentionally quiescent until clocks and pin ownership are established.
pub const PHY_POWERDOWN_INTERRUPT: GioLine = GioLine {
    port: GioPort::A,
    pin: 3,
    asserted: Level::Low,
    board_pull: Some(Level::Low),
};

/// Exact Ethernet capability of LAUNCHXL2-570LC43.
pub const ETHERNET: EthernetBoard = EthernetBoard {
    interface: MediaInterface::Mii,
    phy: PhyModel::Dp83630,
    phy_address: PHY_ADDRESS,
    reference_clock: PhyReferenceClockSource::Eclk1T,
    reset: PHY_RESET,
    powerdown_interrupt: PHY_POWERDOWN_INTERRUPT,
    direct_rj45: true,
};

static PERIPHERALS_OWNED: AtomicBool = AtomicBool::new(false);

/// Unique ownership of the board resources implemented by this BSP.
///
/// The set grows as later optional-plan packages implement more peripherals.
/// Tokens have private constructors, so safe code cannot create two drivers
/// for the same register frame.
pub struct BoardPeripherals {
    /// RTI counter block 0.
    pub rti: Rti,
    /// CPU interrupt-mask ownership.
    pub interrupts: InterruptController,
    /// Vectored interrupt manager register and vector-RAM ownership.
    pub vim: Vim,
}

/// Ownership token for RTI counter block 0.
pub struct Rti(());

/// Ownership token for CPU IRQ/FIQ mask state.
pub struct InterruptController(());

/// Ownership token for the VIM control frame and vector RAM.
pub struct Vim(());

#[cfg(test)]
pub(crate) const fn interrupt_test_token() -> InterruptController {
    InterruptController(())
}

impl BoardPeripherals {
    /// Claim the currently implemented board resources exactly once.
    pub fn take() -> Option<Self> {
        PERIPHERALS_OWNED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| Self {
                rti: Rti(()),
                interrupts: InterruptController(()),
                vim: Vim(()),
            })
    }
}

const _: () = {
    assert!(MCU_OSCILLATOR_HZ == 16_000_000);
    assert!(PHY_REFERENCE_CLOCK_HZ == 25_000_000);
    assert!(MII_DATA_BITS == 4);
    assert!(PHY_ADDRESS_RAW > 0 && PHY_ADDRESS_RAW < 32);
    assert!(PHY_RESET.pin < 8);
    assert!(PHY_POWERDOWN_INTERRUPT.pin < 8);
    assert!(ETHERNET.direct_rj45);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_public_board_identity_is_selected() {
        assert_eq!(MODEL, "LAUNCHXL2-570LC43");
        assert_eq!(SCHEMATIC, "SPRR397 A1");
        assert_eq!(MCU_OSCILLATOR_HZ, 16_000_000);
    }

    #[test]
    fn ethernet_path_is_direct_mii_through_dp83630() {
        assert_eq!(ETHERNET.interface, MediaInterface::Mii);
        assert_eq!(ETHERNET.phy, PhyModel::Dp83630);
        assert_eq!(ETHERNET.phy_address.get(), 1);
        assert_eq!(ETHERNET.reference_clock, PhyReferenceClockSource::Eclk1T);
    }

    #[test]
    fn phy_control_lines_preserve_active_low_startup_state() {
        assert_eq!(ETHERNET.reset.port, GioPort::A);
        assert_eq!(ETHERNET.reset.pin, 4);
        assert_eq!(ETHERNET.reset.asserted, Level::Low);
        assert_eq!(ETHERNET.powerdown_interrupt.port, GioPort::A);
        assert_eq!(ETHERNET.powerdown_interrupt.pin, 3);
        assert_eq!(ETHERNET.powerdown_interrupt.asserted, Level::Low);
        assert_eq!(ETHERNET.powerdown_interrupt.board_pull, Some(Level::Low));
    }

    #[test]
    fn phy_reference_and_strap_requirements_are_bounded() {
        assert_eq!(PHY_REFERENCE_CLOCK_HZ, 25_000_000);
        assert_eq!(PHY_STRAP_RESISTANCE_OHMS, 2_200);
        assert_ne!(PHY_ADDRESS.get(), 0);
    }

    #[test]
    fn peripheral_tokens_have_one_owner() {
        let _owner = BoardPeripherals::take().unwrap();
        assert!(BoardPeripherals::take().is_none());
    }
}
