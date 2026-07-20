//! LAUNCHXL2-570LC43 support with a TMS570LC4357 revision-B device.
//!
//! This crate is the optional TMS570 platform boundary. It contains only
//! device and board adaptation; protocol, diagnostic, lifecycle, storage and
//! application behavior stays in shared crates.
//!
//! The connected MCU was proven by TAP identity and the operator identified
//! the carrier as LAUNCHXL2-570LC43. Board constants are therefore limited to
//! that exact public TI schematic. No writable flash region is selected.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(test)]
extern crate std;

#[cfg(all(target_arch = "arm", not(feature = "tms570lc4357-revb")))]
compile_error!("an Arm target build requires feature tms570lc4357-revb");

#[cfg(all(
    target_arch = "arm",
    feature = "tms570lc4357-revb",
    not(feature = "launchxl2-570lc43")
))]
compile_error!("an Arm BSP build requires board feature launchxl2-570lc43");

#[cfg(all(
    target_arch = "arm",
    feature = "tms570lc4357-revb",
    not(target_endian = "big")
))]
compile_error!("TMS570LC4357 requires an Armv7-R BE32 target");

pub mod device;
pub mod emac;
pub mod exception;
pub mod interrupt;
pub mod mdio;
pub mod rti;
pub mod startup;
pub mod vim;

#[cfg(feature = "launchxl2-570lc43")]
pub mod board;
#[cfg(feature = "launchxl2-570lc43")]
pub mod clock;

#[cfg(target_arch = "arm")]
mod exception_arm;
#[cfg(target_arch = "arm")]
mod startup_arm;
#[cfg(target_arch = "arm")]
pub use exception_arm::link_exception_capture;
#[cfg(target_arch = "arm")]
pub use startup_arm::link_startup_probe;
