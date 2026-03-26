// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! bxCAN loopback test for NUCLEO-F413ZH.
//!
//! Verifies the bxCAN driver in hardware loopback mode: a frame sent on CAN1
//! is received back via the internal TX→RX loopback path (no external bus or
//! transceiver needed).
//!
//! # Build
//!
//! ```sh
//! cargo build \
//!     --features stm32f413 \
//!     --target thumbv7em-none-eabihf \
//!     --example can_loopback_f413
//! ```
//!
//! # Hardware
//!
//! | Resource   | Assignment                                      |
//! |------------|-------------------------------------------------|
//! | CAN1 TX    | PD1 (AF9) — not needed for loopback, but wired  |
//! | CAN1 RX    | PD0 (AF9) — not needed for loopback, but wired  |
//! | UART TX    | PD8 (USART3 AF7) → ST-LINK VCP                 |
//! | UART RX    | PD9 (USART3 AF7)                                |
//!
//! # Expected serial output (115 200 baud on ST-LINK VCP)
//!
//! ```text
//! CAN loopback test starting...
//! TX frame: ID=0x123 DLC=4 data=[DE AD BE EF]
//! RX frame: ID=0x123 DLC=4 data=[DE AD BE EF]
//! PASS: ID and payload match.
//! ```

#![no_std]
#![no_main]

use core::fmt::Write as _;

use cortex_m::peripheral::Peripherals as CorePeripherals;
use cortex_m_rt::entry;

use bsw_bsp_stm32::can_bxcan::BxCanTransceiver;
use bsw_bsp_stm32::clock_f4::configure_clocks_f413;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_f4::PolledUart;
use bsw_can::{CanFrame, CanId, CanTransceiver, ErrorCode};
use stm32f4xx_hal::pac;

// ---------------------------------------------------------------------------
// Test frame constants
// ---------------------------------------------------------------------------

/// Arbitration ID for the test frame (11-bit base frame).
const TEST_ID: u16 = 0x123;

/// Payload for the test frame.
const TEST_DATA: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

/// Maximum number of spin iterations to wait for FIFO0 to become non-empty.
const RX_POLL_TIMEOUT: u32 = 1_000_000;

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // -----------------------------------------------------------------------
    // 1. Configure clocks: HSE 8 MHz → PLL → SYSCLK 96 MHz, APB1 48 MHz
    // -----------------------------------------------------------------------
    let sys_clock = configure_clocks_f413();

    // -----------------------------------------------------------------------
    // 2. Init DWT timer (microsecond monotonic counter via CYCCNT)
    // -----------------------------------------------------------------------
    let mut _timer = DwtTimer::new();
    _timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // -----------------------------------------------------------------------
    // 3. Init UART (USART3 PD8/PD9, 115 200 baud) and greet
    // -----------------------------------------------------------------------
    let mut uart = PolledUart::new();
    // SAFETY: called once at startup after clock configuration.
    unsafe { uart.init() };
    let _ = writeln!(uart, "CAN loopback test starting...");

    // -----------------------------------------------------------------------
    // 4. Init bxCAN transceiver (CAN1, PD0/PD1 AF9, 500 kbit/s)
    // -----------------------------------------------------------------------
    let mut can = BxCanTransceiver::new(0);

    let rc = can.init();
    if rc != ErrorCode::Ok {
        let _ = writeln!(uart, "FAIL: can.init() returned {:?}", rc);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // -----------------------------------------------------------------------
    // 5. Enable bxCAN loopback mode (BTR.LBKM = bit 30) while in init mode.
    //
    //    After can.init() the peripheral is in init mode (INRQ=1, INAK=1).
    //    can.open() will later clear LBKM when it calls
    //    `btr().modify(|r,w| w.bits(r.bits() & !(1<<30 | 1<<31)))` before
    //    leaving init mode.  To work around this, we call open() first (which
    //    takes the state machine to Open and leaves init mode with LBKM=0),
    //    then immediately re-enter init mode via raw register access, set
    //    LBKM, and leave init mode again.  The software state machine remains
    //    Open throughout; only the hardware briefly re-enters init mode.
    // -----------------------------------------------------------------------

    // First, open the transceiver (state: Initialized → Open).
    let rc = can.open();
    if rc != ErrorCode::Ok {
        let _ = writeln!(uart, "FAIL: can.open() returned {:?}", rc);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // Now re-enter init mode, set LBKM, leave init mode.
    // SAFETY: PAC singleton access in single-threaded bare-metal startup.
    unsafe {
        let can1 = &*pac::CAN1::ptr();

        // Request init mode (INRQ=1) and clear sleep mode (SLEEP=0).
        can1.mcr().modify(|_, w| w.sleep().clear_bit().inrq().set_bit());
        // Wait for INAK acknowledgement from hardware.
        while can1.msr().read().inak().bit_is_clear() {}

        // Set LBKM (bit 30) in BTR.  All other timing bits are already
        // programmed by can.init() and must not be disturbed.
        can1.btr().modify(|r, w| w.bits(r.bits() | (1 << 30)));

        // Leave init mode (INRQ=0); hardware acknowledges by clearing INAK.
        can1.mcr().modify(|_, w| w.inrq().clear_bit());
        while can1.msr().read().inak().bit_is_set() {}
    }

    // -----------------------------------------------------------------------
    // 6. Build and send the test frame
    // -----------------------------------------------------------------------
    let tx_frame = CanFrame::with_data(CanId::base(TEST_ID), &TEST_DATA);

    let _ = writeln!(
        uart,
        "TX frame: ID=0x{:03X} DLC={} data=[{:02X} {:02X} {:02X} {:02X}]",
        TEST_ID,
        TEST_DATA.len(),
        TEST_DATA[0],
        TEST_DATA[1],
        TEST_DATA[2],
        TEST_DATA[3],
    );

    let rc = can.write(&tx_frame);
    if rc != ErrorCode::Ok {
        let _ = writeln!(uart, "FAIL: can.write() returned {:?}", rc);
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // -----------------------------------------------------------------------
    // 7. Poll CAN1 FIFO0 until a frame is pending (RF0R.FMP0 > 0).
    //
    //    In loopback mode the transmitted frame is immediately looped back
    //    into FIFO0 without requiring an external ACK.
    // -----------------------------------------------------------------------
    let mut timed_out = true;
    // SAFETY: read-only polling of RF0R; no mutation; single-threaded context.
    for _ in 0..RX_POLL_TIMEOUT {
        let fmp = unsafe { (*pac::CAN1::ptr()).rf0r().read().fmp().bits() };
        if fmp > 0 {
            timed_out = false;
            break;
        }
    }

    if timed_out {
        let _ = writeln!(uart, "FAIL: timeout waiting for loopback frame in FIFO0.");
        loop {
            cortex_m::asm::bkpt();
        }
    }

    // -----------------------------------------------------------------------
    // 8. Drain FIFO0 into the software receive queue via receive_isr().
    //
    //    receive_isr() is the same path used by the CAN1_RX0 interrupt handler.
    //    Calling it directly here (with interrupts disabled by default in bare-
    //    metal startup) is safe because we are the sole owner of the CAN driver
    //    and no interrupt handler is registered.
    // -----------------------------------------------------------------------
    // SAFETY: single-threaded bare-metal context; no concurrent ISR.
    unsafe { can.receive_isr() };

    // -----------------------------------------------------------------------
    // 9. Dequeue the received frame and verify ID and payload
    // -----------------------------------------------------------------------
    let rx_frame = match can.dequeue_frame() {
        Some(f) => f,
        None => {
            let _ = writeln!(uart, "FAIL: dequeue_frame() returned None after receive_isr().");
            loop {
                cortex_m::asm::bkpt();
            }
        }
    };

    let rx_id = rx_frame.id();
    let rx_payload = rx_frame.payload();

    let _ = writeln!(
        uart,
        "RX frame: ID=0x{:03X} DLC={} data=[{}]",
        rx_id.raw_id(),
        rx_payload.len(),
        // Format each byte as two hex digits separated by spaces.
        // core::fmt doesn't support iterators directly; unroll up to 8 bytes.
        RxDataFmt(rx_payload),
    );

    // -----------------------------------------------------------------------
    // 10. Compare ID and payload, print PASS or FAIL
    // -----------------------------------------------------------------------
    let id_ok = rx_id == CanId::base(TEST_ID);
    let data_ok = rx_payload == TEST_DATA;

    if id_ok && data_ok {
        let _ = writeln!(uart, "PASS: ID and payload match.");
    } else {
        if !id_ok {
            let _ = writeln!(
                uart,
                "FAIL: ID mismatch — expected 0x{:03X}, got 0x{:03X}.",
                TEST_ID,
                rx_id.raw_id(),
            );
        }
        if !data_ok {
            let _ = writeln!(uart, "FAIL: payload mismatch.");
        }
    }

    loop {
        cortex_m::asm::bkpt();
    }
}

// ---------------------------------------------------------------------------
// Helper: format a byte slice as space-separated hex pairs for core::fmt
// ---------------------------------------------------------------------------

/// Newtype wrapper that implements `core::fmt::Display` for a `&[u8]`
/// payload, printing each byte as a two-digit uppercase hex value separated
/// by spaces (e.g. `"DE AD BE EF"`).
///
/// This avoids heap allocation while keeping the `writeln!` call readable.
struct RxDataFmt<'a>(&'a [u8]);

impl core::fmt::Display for RxDataFmt<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let bytes = self.0;
        for (i, &b) in bytes.iter().enumerate() {
            if i > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{b:02X}")?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // Best-effort: try to print the panic message over UART.
    // PolledUart::new() is safe to call here because it only stores a
    // const pointer; no re-initialisation of the peripheral is done.
    let mut uart = PolledUart::new();
    let _ = writeln!(uart, "PANIC: {}", info);
    loop {
        cortex_m::asm::bkpt();
    }
}
