//! STM32 F4/G4 Board Support Package for OpenBSW-Rust.
//!
//! Provides concrete implementations of the BSW hardware-abstraction traits
//! (`SystemTimer`, `InterruptLock`, `CanTransceiver`) for STM32F413ZH (bxCAN)
//! and STM32G474RE (FDCAN).
//!
//! Enable exactly one chip feature: `stm32f413` or `stm32g474`.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(test)]
extern crate std;

#[cfg(all(feature = "stm32f413", feature = "stm32g474"))]
compile_error!("enable exactly one MCU feature: stm32f413 or stm32g474");

#[cfg(all(
    target_arch = "arm",
    not(any(feature = "stm32f413", feature = "stm32g474"))
))]
compile_error!("an ARM build requires exactly one MCU feature: stm32f413 or stm32g474");

// Shared Cortex-M modules (no chip dependency)
pub mod adc;
pub mod board;
#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
pub mod board_apps;
pub mod can_health;
pub mod can_isr;
pub mod diag_can;
#[cfg(target_arch = "arm")]
pub mod fault;
pub mod gpio;
pub mod gpio_manager;
#[cfg(target_arch = "arm")]
pub mod interrupt;
mod mmio;
pub mod pwm;
pub mod reset;
#[cfg(target_arch = "arm")]
pub mod scheduler;
pub mod timer;
#[cfg(target_arch = "arm")]
pub mod watchdog;

// F4-specific modules
#[cfg(feature = "stm32f413")]
pub mod can_bxcan;
#[cfg(feature = "stm32f413")]
pub mod clock_f4;
#[cfg(feature = "stm32f413")]
pub mod flash_f4;
#[cfg(feature = "stm32f413")]
pub mod uart_f4;

// G4-specific modules
#[cfg(feature = "stm32g474")]
pub mod can_fdcan;
#[cfg(feature = "stm32g474")]
pub mod clock_g4;
#[cfg(feature = "stm32g474")]
pub mod flash_g4;
#[cfg(all(target_arch = "arm", feature = "stm32g474"))]
pub mod storage_conformance;
#[cfg(feature = "stm32g474")]
pub mod storage_g4;
#[cfg(feature = "stm32g474")]
pub mod uart_g4;

// Re-exports for convenience
pub use gpio::{InputPin, OutputPin, Port, Pull};
#[cfg(target_arch = "arm")]
pub use interrupt::PrimaskLock;
#[cfg(target_arch = "arm")]
pub use scheduler::Scheduler;
#[cfg(target_arch = "arm")]
pub use timer::DwtTimer;
#[cfg(target_arch = "arm")]
pub use watchdog::Iwdg;

#[cfg(feature = "stm32f413")]
pub use can_bxcan::BxCanTransceiver;
#[cfg(feature = "stm32f413")]
pub use clock_f4::configure_clocks_f413;

#[cfg(feature = "stm32g474")]
pub use can_fdcan::FdCanTransceiver;
#[cfg(feature = "stm32g474")]
pub use clock_g4::configure_clocks_g474;
