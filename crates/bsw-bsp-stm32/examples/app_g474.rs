//! Full BSW application for NUCLEO-G474RE.
//!
//! Demonstrates: BSP → scheduler → GPIO → UART → watchdog.
//! Pure Rust firmware — no C++, no RTOS.
//!
//! Build:
//! ```
//! cargo build --features stm32g474 --example app_g474 --release
//! ```

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;

use bsw_bsp_stm32::clock_g4::configure_clocks_g474;
use bsw_bsp_stm32::gpio::{OutputPin, Port};
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_g4::PolledUart;

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // 1. Clock: HSI → PLL → 170 MHz
    let sys_clock = configure_clocks_g474();

    // 2. DWT timer
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // 3. UART
    let mut uart = PolledUart::new();
    uart.init();
    let _ = writeln!(uart, "");
    let _ = writeln!(uart, "=== OpenBSW-Rust on STM32G474RE @ {} MHz ===", sys_clock / 1_000_000);
    let _ = writeln!(uart, "Scheduler: direct 1ms/10ms/100ms in main loop");

    // 4. LED (PA5)
    let led = OutputPin::new(Port::A, 5);
    led.set_low();

    // 5. Watchdog (IWDG, 1 second timeout, kicked every 1ms tick)
    unsafe {
        let wdg: *mut u32 = 0x4000_3000 as *mut u32;
        core::ptr::write_volatile(wdg, 0xCCCC);          // start IWDG + LSI
        core::ptr::write_volatile(wdg, 0x5555);          // unlock PR/RLR
        core::ptr::write_volatile(wdg.add(1), 0b011);    // PR = /32
        core::ptr::write_volatile(wdg.add(2), 999);      // RLR = 999 → ~1s
        while core::ptr::read_volatile(wdg.add(3)) & 3 != 0 {} // wait PVU/RVU
        core::ptr::write_volatile(wdg, 0xAAAA);          // initial kick
    }
    let _ = writeln!(uart, "[  0.0s] Watchdog started (IWDG ~1s)");

    // 6. Scheduler state — local variables (no static mut UB)
    let mut tick_1ms: u32 = 0;
    let mut heartbeats: u32 = 0;
    let mut last_1ms = timer.system_time_us_64();
    let mut last_10ms = last_1ms;
    let mut last_100ms = last_1ms;

    let _ = writeln!(uart, "[  0.0s] Running");

    // 6. Main loop — cooperative scheduling without function pointers
    loop {
        let now = timer.system_time_us_64();

        // --- 1 ms task ---
        if now.wrapping_sub(last_1ms) >= 1_000 {
            last_1ms = now;
            tick_1ms += 1;
            // Kick watchdog
            unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
        }

        // --- 10 ms task ---
        if now.wrapping_sub(last_10ms) >= 10_000 {
            last_10ms = now;
            // Future: CAN RX processing, signal debouncing
        }

        // --- 100 ms task ---
        if now.wrapping_sub(last_100ms) >= 100_000 {
            last_100ms = now;
            heartbeats += 1;

            // Toggle LED every 500 ms
            if heartbeats % 5 == 0 {
                led.toggle();
            }

            // Print status every 2 seconds
            if heartbeats % 20 == 0 {
                let _ = writeln!(
                    uart,
                    "[{:>5}.{}s] ticks={} heartbeats={}",
                    heartbeats / 10,
                    heartbeats % 10,
                    tick_1ms,
                    heartbeats
                );
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut uart = PolledUart::new();
    uart.init();
    let _ = writeln!(uart, "!!! PANIC: {}", info);
    loop {
        cortex_m::asm::bkpt();
    }
}
