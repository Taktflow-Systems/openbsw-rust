//! Compile/link/disassembly probe for LC4357 flash startup.
//!
//! This image has not been physically verified, is not an authorized firmware
//! artifact, and must not be loaded or flashed into a target.

#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg(target_os = "none")]
use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicU32, Ordering},
};

#[cfg(target_os = "none")]
#[used]
#[link_section = ".noinit.tms570"]
static BOOT_MARKER: AtomicU32 = AtomicU32::new(0);

#[cfg(target_os = "none")]
#[no_mangle]
pub extern "C" fn tms570_rust_entry() -> ! {
    bsw_bsp_tms570::link_startup_probe();
    BOOT_MARKER.store(0x544d_5331, Ordering::Release);
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
fn main() {}
