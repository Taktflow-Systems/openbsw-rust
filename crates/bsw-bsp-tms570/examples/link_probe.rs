//! Compile/link probe for the Armv7-R BE32 hard-float toolchain.
//!
//! This image is for map and disassembly checks only. It is not a bootable BSP
//! image and must not be loaded into a target.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use bsw_bsp_tms570::device::{DEVICE_ID_REV_B, JTAG_IDCODE_REV_B};
#[cfg(target_os = "none")]
use core::panic::PanicInfo;

#[cfg(target_os = "none")]
#[used]
#[link_section = ".target_identity"]
static TARGET_IDENTITY: [u32; 2] = [JTAG_IDCODE_REV_B, DEVICE_ID_REV_B];

#[cfg(target_os = "none")]
#[no_mangle]
pub extern "C" fn _start() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
#[cfg(target_os = "none")]
#[panic_handler]
fn panic(_info: &PanicInfo<'_>) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(not(target_os = "none"))]
fn main() {
    let _ = (JTAG_IDCODE_REV_B, DEVICE_ID_REV_B);
}
