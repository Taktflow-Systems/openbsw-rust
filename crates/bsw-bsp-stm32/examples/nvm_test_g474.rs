//! NvM persistence test for NUCLEO-G474RE.
//!
//! Exercises the full write → power-cycle → read-back flow for
//! [`NvmManager`] / [`FlashG4`] on the STM32G474RE's internal flash.
//!
//! # What it does
//!
//! **Run 1 (fresh flash or after factory-reset):**
//! 1. Init clocks (170 MHz), DWT timer, USART2, LED (PA5).
//! 2. Attempt to read the VIN block — reports "No VIN stored".
//! 3. Write `"RUST_NVM_TEST_01"` (16 bytes) to the VIN block.
//! 4. Immediately reads it back and verifies the payload.
//! 5. Prints `"Power cycle now, then re-flash to verify persistence"`.
//! 6. Blinks LED forever (500 ms period).
//!
//! **Run 2 (after power cycle — VIN persists in flash):**
//! - Step 2 finds `"RUST_NVM_TEST_01"` already stored.
//! - Prints `"VIN persisted: PASS"`.
//! - Skips the write and goes straight to blinking.
//!
//! # Build
//!
//! ```text
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!     --example nvm_test_g474
//! ```
//!
//! # Flash
//!
//! ```text
//! STM32_Programmer_CLI.exe --connect port=SWD \
//!     --download target/thumbv7em-none-eabihf/debug/examples/nvm_test_g474 \
//!     --verify --start
//! ```

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;

use bsw_bsp_stm32::clock_g4::configure_clocks_g474;
use bsw_bsp_stm32::flash_g4::FlashG4;
use bsw_bsp_stm32::nvm::{NvmBlockId, NvmManager};
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_g4::PolledUart;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// NUCLEO-G474RE: LD2 = PA5 (active-high).
const LED_PIN: u32 = 5;

/// VIN test payload — exactly 16 bytes (≤ 17-byte VIN max).
const TEST_VIN: &[u8] = b"RUST_NVM_TEST_01";

/// RCC base address (STM32G474, RM0440 Table 3).
const RCC_BASE: u32 = 0x4002_1000;
/// GPIOA base address.
const GPIOA_BASE: u32 = 0x4800_0000;

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // ------------------------------------------------------------------
    // 1. System init: clocks, timer, UART, LED
    // ------------------------------------------------------------------

    let sys_clock = configure_clocks_g474();

    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // PA5 → push-pull output for the user LED.
    unsafe {
        let rcc = RCC_BASE as *mut u32;
        // Enable GPIOA clock: RCC_AHB2ENR bit 0.
        let ahb2enr = rcc.add(0x4C / 4);
        core::ptr::write_volatile(ahb2enr, core::ptr::read_volatile(ahb2enr) | 1);

        let gpioa = GPIOA_BASE as *mut u32;
        // MODER PA5 = 01 (output).
        let val = core::ptr::read_volatile(gpioa);
        let val = (val & !(0x3 << (LED_PIN * 2))) | (0x1 << (LED_PIN * 2));
        core::ptr::write_volatile(gpioa, val);
    }

    let mut uart = PolledUart::new();
    uart.init();

    let _ = writeln!(
        uart,
        "\r\n=== NvM Test — STM32G474RE @ {} MHz ===",
        sys_clock / 1_000_000
    );

    // ------------------------------------------------------------------
    // 2. Phase 1: Read — check whether VIN was stored in a previous run.
    // ------------------------------------------------------------------

    let _ = writeln!(uart, "[Phase 1] Reading VIN block from NvM...");

    let mut vin_buf = [0u8; 17]; // VIN_MAX_LEN
    let read_len = NvmManager::read_block(NvmBlockId::Vin, &mut vin_buf);

    if read_len > 0 {
        // VIN present with valid CRC — show it.
        let stored = &vin_buf[..read_len];

        let _ = write!(uart, "[Phase 1] VIN found ({} bytes): \"", read_len);
        for &b in stored {
            if b.is_ascii_graphic() || b == b' ' {
                let _ = uart.write_char(b as char);
            } else {
                let _ = write!(uart, "\\x{:02X}", b);
            }
        }
        let _ = writeln!(uart, "\"");

        // Verify it matches our test payload.
        if stored == TEST_VIN {
            let _ = writeln!(uart, "[Phase 1] VIN persisted: PASS");
        } else {
            let _ = writeln!(uart, "[Phase 1] VIN content mismatch — unexpected data in flash.");
        }

        // Skip the write phase; go straight to the blink loop.
        blink_loop(&mut uart, &mut timer, &mut vin_buf);
    }

    // ------------------------------------------------------------------
    // 3. VIN is empty / invalid on first run.
    // ------------------------------------------------------------------

    let _ = writeln!(uart, "[Phase 1] No VIN stored — writing test VIN...");

    // ------------------------------------------------------------------
    // 4 & 5. Write "RUST_NVM_TEST_01" to VIN block via NvmManager.
    //         NvmManager::write_block requires flash to be unlocked first.
    // ------------------------------------------------------------------

    let write_ok = unsafe {
        let _was_already_unlocked = FlashG4::unlock();
        let ok = NvmManager::write_block(NvmBlockId::Vin, TEST_VIN);
        FlashG4::lock();
        ok
    };

    if write_ok {
        let _ = writeln!(uart, "[Phase 2] write_block(VIN): OK");
    } else {
        let _ = writeln!(uart, "[Phase 2] write_block(VIN): FAILED — check flash controller.");
        // Still blink so the board is not left silent.
        blink_loop(&mut uart, &mut timer, &mut vin_buf);
    }

    // ------------------------------------------------------------------
    // 5. Immediate read-back: verify the write succeeded.
    // ------------------------------------------------------------------

    let _ = writeln!(uart, "[Phase 2] Immediate read-back verification...");

    let verify_len = NvmManager::read_block(NvmBlockId::Vin, &mut vin_buf);

    if verify_len == TEST_VIN.len() && &vin_buf[..verify_len] == TEST_VIN {
        let _ = writeln!(uart, "[Phase 2] Immediate verify: PASS");
    } else if verify_len == 0 {
        let _ = writeln!(uart, "[Phase 2] Immediate verify: FAIL — read returned 0 (CRC error or erased).");
    } else {
        let _ = writeln!(
            uart,
            "[Phase 2] Immediate verify: FAIL — got {} bytes, payload mismatch.",
            verify_len
        );
    }

    // ------------------------------------------------------------------
    // 6. Prompt for power cycle, then blink forever.
    // ------------------------------------------------------------------

    let _ = writeln!(
        uart,
        "[Phase 2] Power cycle now, then re-flash to verify persistence."
    );
    let _ = writeln!(uart, "[Phase 2] On next boot: Phase 1 should print \"VIN persisted: PASS\".");

    blink_loop(&mut uart, &mut timer, &mut vin_buf);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Blink LD2 (PA5) at 500 ms intervals forever.
///
/// Accepts `_uart` and `_scratch` so the compiler sees them as used when this
/// function is reached from multiple call sites (avoids dead-code warnings).
fn blink_loop(uart: &mut PolledUart, timer: &mut DwtTimer, _scratch: &mut [u8]) -> ! {
    let _ = writeln!(uart, "[Done] Blinking LED forever (500 ms). Connect a serial terminal to observe output on next boot.");

    let mut led_on = false;
    let mut last_toggle = timer.system_time_us_64();

    loop {
        let now = timer.system_time_us_64();
        if now.wrapping_sub(last_toggle) >= 500_000 {
            last_toggle = now;
            led_on = !led_on;
            unsafe {
                let bsrr = (GPIOA_BASE as *mut u32).add(0x18 / 4);
                if led_on {
                    core::ptr::write_volatile(bsrr, 1 << LED_PIN); // set
                } else {
                    core::ptr::write_volatile(bsrr, 1 << (LED_PIN + 16)); // reset
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // On a panic: halt with a breakpoint so a connected debugger catches it.
    // UART may not be available here; don't attempt to print.
    let _ = info;
    loop {
        cortex_m::asm::bkpt();
    }
}
