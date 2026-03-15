//! Minimal blink + UART hello + CAN loopback for NUCLEO-F413ZH.
//!
//! Build: `cargo build --features stm32f413 --target thumbv7em-none-eabihf --example blink_f413`

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;

use bsw_bsp_stm32::clock_f4::configure_clocks_f413;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_f4::PolledUart;

use cortex_m::peripheral::Peripherals as CorePeripherals;

// NUCLEO-F413ZH: LD2 = PA5 (active-high, directly driven)
const LED_PIN: u32 = 5;

/// Register base addresses (STM32F413).
const RCC_BASE: u32 = 0x4002_3800;
const GPIOA_BASE: u32 = 0x4002_0000;

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // 1. Configure clocks (96 MHz)
    let sys_clock = configure_clocks_f413();

    // 2. Init DWT timer
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // 3. Configure PA5 as output (LED)
    unsafe {
        let rcc = RCC_BASE as *mut u32;
        // Enable GPIOA clock (AHB1ENR bit 0)
        let ahb1enr = rcc.add(0x30 / 4);
        core::ptr::write_volatile(ahb1enr, core::ptr::read_volatile(ahb1enr) | 1);

        let gpioa = GPIOA_BASE as *mut u32;
        // MODER: set PA5 to output (bits 11:10 = 01)
        let moder = gpioa;
        let val = core::ptr::read_volatile(moder);
        let val = (val & !(3 << (LED_PIN * 2))) | (1 << (LED_PIN * 2));
        core::ptr::write_volatile(moder, val);
    }

    // 4. Init UART (115200 baud on USART2 PA2/PA3)
    let mut uart = PolledUart::new();
    unsafe { uart.init() };
    let _ = writeln!(uart, "Hello from Rust BSP on STM32F413ZH @ {} MHz", sys_clock / 1_000_000);

    // 5. Blink loop
    let mut led_on = false;
    let mut last_toggle = timer.system_time_us_64();

    loop {
        let now = timer.system_time_us_64();

        // Toggle LED every 500 ms
        if now.wrapping_sub(last_toggle) >= 500_000 {
            last_toggle = now;
            led_on = !led_on;

            unsafe {
                let gpioa = GPIOA_BASE as *mut u32;
                let bsrr = gpioa.add(0x18 / 4);
                if led_on {
                    core::ptr::write_volatile(bsrr, 1 << LED_PIN); // set
                } else {
                    core::ptr::write_volatile(bsrr, 1 << (LED_PIN + 16)); // reset
                }
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // In a real application, dump to UART or NO_INIT RAM
    let _ = info;
    loop {
        cortex_m::asm::bkpt();
    }
}
