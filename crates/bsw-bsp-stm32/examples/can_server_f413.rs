#![no_std]
#![no_main]

use bsw_bsp_stm32::board_apps::{self, DemoRole};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    board_apps::run(DemoRole::CanServer)
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
