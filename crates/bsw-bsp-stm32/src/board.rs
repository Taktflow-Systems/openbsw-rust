//! Typed board configuration and single-owner peripheral tokens (G01).

use core::sync::atomic::{AtomicBool, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mcu {
    Stm32F413,
    Stm32G474,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortId {
    A,
    B,
    C,
    D,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinConfig {
    pub port: PortId,
    pub pin: u8,
    pub alternate_function: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardConfig {
    pub mcu: Mcu,
    pub name: &'static str,
    pub core_clock_hz: u32,
    pub peripheral_clock_hz: u32,
    pub can_rx: PinConfig,
    pub can_tx: PinConfig,
    pub uart_rx: PinConfig,
    pub uart_tx: PinConfig,
    pub led: PinConfig,
    pub pwm: PinConfig,
    pub adc: PinConfig,
}

pub const F413: BoardConfig = BoardConfig {
    mcu: Mcu::Stm32F413,
    name: "nucleo-f413zh",
    core_clock_hz: 96_000_000,
    peripheral_clock_hz: 48_000_000,
    can_rx: PinConfig {
        port: PortId::D,
        pin: 0,
        alternate_function: Some(9),
    },
    can_tx: PinConfig {
        port: PortId::D,
        pin: 1,
        alternate_function: Some(9),
    },
    uart_rx: PinConfig {
        port: PortId::D,
        pin: 9,
        alternate_function: Some(7),
    },
    uart_tx: PinConfig {
        port: PortId::D,
        pin: 8,
        alternate_function: Some(7),
    },
    led: PinConfig {
        port: PortId::A,
        pin: 5,
        alternate_function: None,
    },
    pwm: PinConfig {
        port: PortId::A,
        pin: 6,
        alternate_function: Some(2),
    },
    adc: PinConfig {
        port: PortId::A,
        pin: 0,
        alternate_function: None,
    },
};

pub const G474: BoardConfig = BoardConfig {
    mcu: Mcu::Stm32G474,
    name: "nucleo-g474re",
    core_clock_hz: 170_000_000,
    peripheral_clock_hz: 170_000_000,
    can_rx: PinConfig {
        port: PortId::A,
        pin: 11,
        alternate_function: Some(9),
    },
    can_tx: PinConfig {
        port: PortId::A,
        pin: 12,
        alternate_function: Some(9),
    },
    uart_rx: PinConfig {
        port: PortId::A,
        pin: 3,
        alternate_function: Some(7),
    },
    uart_tx: PinConfig {
        port: PortId::A,
        pin: 2,
        alternate_function: Some(7),
    },
    led: PinConfig {
        port: PortId::A,
        pin: 5,
        alternate_function: None,
    },
    pwm: PinConfig {
        port: PortId::A,
        pin: 6,
        alternate_function: Some(2),
    },
    adc: PinConfig {
        port: PortId::A,
        pin: 0,
        alternate_function: None,
    },
};

#[cfg(feature = "stm32f413")]
pub const ACTIVE: BoardConfig = F413;
#[cfg(feature = "stm32g474")]
pub const ACTIVE: BoardConfig = G474;

static OWNED: AtomicBool = AtomicBool::new(false);

/// Unique ownership of the board peripheral set.
pub struct BoardPeripherals {
    pub can1: Can1,
    pub uart: Uart,
    pub gpio: Gpio,
    pub timer: Timer,
    pub pwm: Pwm,
    pub adc: Adc,
    pub watchdog: Watchdog,
    pub flash: Flash,
    pub reset: Reset,
}

pub struct Can1(());
pub struct Uart(());
pub struct Gpio(());
pub struct Timer(());
pub struct Pwm(());
pub struct Adc(());
pub struct Watchdog(());
pub struct Flash(());
pub struct Reset(());

impl BoardPeripherals {
    /// Claim every board peripheral exactly once until the owner is dropped.
    pub fn take() -> Option<Self> {
        OWNED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| Self {
                can1: Can1(()),
                uart: Uart(()),
                gpio: Gpio(()),
                timer: Timer(()),
                pwm: Pwm(()),
                adc: Adc(()),
                watchdog: Watchdog(()),
                flash: Flash(()),
                reset: Reset(()),
            })
    }
}

pub struct Stm32PlatformInfo(pub BoardConfig);

impl bsw_platform::PlatformInfo for Stm32PlatformInfo {
    fn platform_name(&self) -> &'static str {
        self.0.name
    }

    fn cpu_frequency_hz(&self) -> u32 {
        self.0.core_clock_hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_platform::PlatformInfo;

    #[test]
    fn board_tables_hold_all_shared_pin_and_clock_choices() {
        assert_eq!(
            F413.can_rx,
            PinConfig {
                port: PortId::D,
                pin: 0,
                alternate_function: Some(9)
            }
        );
        assert_eq!(
            G474.can_rx,
            PinConfig {
                port: PortId::A,
                pin: 11,
                alternate_function: Some(9)
            }
        );
        assert_eq!(Stm32PlatformInfo(G474).cpu_frequency_hz(), 170_000_000);
    }

    #[test]
    fn ownership_is_unique_for_the_process_lifetime() {
        let _first = BoardPeripherals::take().unwrap();
        assert!(BoardPeripherals::take().is_none());
    }
}
