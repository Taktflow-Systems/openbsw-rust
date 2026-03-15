//! FDCAN1 internal loopback test for NUCLEO-G474RE.
//!
//! Verifies the FDCAN1 hardware by sending a frame in internal-loopback mode
//! (no external CAN bus or termination required) and confirming the received
//! frame matches the transmitted ID and payload.
//!
//! # What it does
//!
//! 1. Init clocks — 170 MHz boost mode via `clock_g4::configure_clocks_g474`.
//! 2. Init DWT timer for microsecond delays.
//! 3. Init USART2 at 115200 baud for test output.
//! 4. Configure FDCAN1 in internal-loopback mode using raw register access
//!    (RM0440 §44.3.3 — TEST.LBCK).
//! 5. Send a test frame: ID=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF].
//! 6. Poll RX FIFO 0 for the looped-back frame (500 ms timeout).
//! 7. Verify received ID and payload match.
//! 8. Print PASS/FAIL over UART and blink LED2 (PA5) to signal completion:
//!    - Fast blink (100 ms) = PASS
//!    - Slow blink (500 ms) = FAIL
//!
//! # Loopback mode
//!
//! Internal loopback (CCCR.TEST=1, TEST.LBCK=1) connects the FDCAN TX output
//! back to the RX input inside the peripheral.  Frames sent appear in RX FIFO0
//! without needing external bus wiring.  The mode requires entering init/CCE
//! mode to modify the TEST register.
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!             --example can_loopback_g474
//! ```
//!
//! # Reference
//!
//! RM0440 Rev 8, chapter 44 (FDCAN).

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::Peripherals as CorePeripherals;

use bsw_bsp_stm32::clock_g4::configure_clocks_g474;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_g4::PolledUart;

// ---------------------------------------------------------------------------
// Register-map constants
// ---------------------------------------------------------------------------

/// RCC base address (RM0440 §6.4).
const RCC_BASE: usize = 0x4002_1000;
/// RCC AHB2ENR — GPIOA/B/C clock enables.
const RCC_AHB2ENR_OFFSET: usize = 0x4C;
/// RCC APB1ENR1 — FDCAN1 clock is on APB1 (bit 25).
const RCC_APB1ENR1_OFFSET: usize = 0x58;
/// FDCAN1EN bit in APB1ENR1 (bit 25).
const FDCAN1EN: u32 = 1 << 25;
/// GPIOA clock enable (AHB2ENR bit 0).
const GPIOAEN: u32 = 1 << 0;
/// GPIOB clock enable (AHB2ENR bit 1).
const GPIOBEN: u32 = 1 << 1;

/// GPIOA base address.
const GPIOA_BASE: usize = 0x4800_0000;
/// GPIOB base address.
const GPIOB_BASE: usize = 0x4800_0400;

/// GPIO register offsets.
const GPIO_MODER_OFFSET: usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_AFRL_OFFSET: usize = 0x20;
const GPIO_AFRH_OFFSET: usize = 0x24;
const GPIO_BSRR_OFFSET: usize = 0x18;

/// FDCAN1 register-block base address (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// Message RAM base address for all FDCAN instances (RM0440 §44.3.3).
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets
// ---------------------------------------------------------------------------

const FDCAN_CCCR_OFFSET: usize = 0x018; // CC control register
const FDCAN_TEST_OFFSET: usize = 0x010; // Test register
const FDCAN_NBTP_OFFSET: usize = 0x01C; // Nominal bit timing
const FDCAN_RXGFC_OFFSET: usize = 0x080; // Global filter config
const FDCAN_SIDFC_OFFSET: usize = 0x084; // Std ID filter config
const FDCAN_XIDFC_OFFSET: usize = 0x088; // Ext ID filter config
const FDCAN_RXF0C_OFFSET: usize = 0x0A0; // RX FIFO 0 config
const FDCAN_RXF1C_OFFSET: usize = 0x0B0; // RX FIFO 1 config
const FDCAN_RXF0S_OFFSET: usize = 0x0A4; // RX FIFO 0 status
const FDCAN_RXF0A_OFFSET: usize = 0x0A8; // RX FIFO 0 acknowledge
const FDCAN_TXBC_OFFSET: usize = 0x0C4;  // TX buffer config
const FDCAN_TXEFC_OFFSET: usize = 0x0F0; // TX event FIFO config
const FDCAN_TXFQS_OFFSET: usize = 0x0C8; // TX FIFO/queue status
const FDCAN_TXBAR_OFFSET: usize = 0x0D0; // TX buffer add request

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

/// INIT — initialisation mode.
const CCCR_INIT: u32 = 1 << 0;
/// CCE — configuration change enable (requires INIT=1).
const CCCR_CCE: u32 = 1 << 1;
/// TEST — test mode enable; gates access to the TEST register.
const CCCR_TEST: u32 = 1 << 7;
/// FDOE — FD operation; 0 = classic CAN only.
const CCCR_FDOE: u32 = 1 << 8;
/// BRSE — bit-rate switching; 0 = disabled.
const CCCR_BRSE: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// FDCAN_TEST bit masks
// ---------------------------------------------------------------------------

/// LBCK (bit 4) — internal loopback mode.
const TEST_LBCK: u32 = 1 << 4;

// ---------------------------------------------------------------------------
// Message RAM offsets (byte offsets from SRAMCAN_BASE for FDCAN1)
// These match the layout in can_fdcan.rs.
// ---------------------------------------------------------------------------

const MRAM_STDFILTER_OFFSET: usize = 0x000; // 28 × 1 W std filters
const MRAM_EXTFILTER_OFFSET: usize = 0x070; // 8 × 2 W  ext filters
const MRAM_RXF0_OFFSET: usize = 0x0B0;      // 3 × 4 W  RX FIFO 0
const MRAM_RXF1_OFFSET: usize = 0x0E0;      // 3 × 4 W  RX FIFO 1
const MRAM_TXEVT_OFFSET: usize = 0x110;     // 3 × 2 W  TX event FIFO
const MRAM_TXBUF_OFFSET: usize = 0x128;     // 3 × 4 W  TX buffers

/// Word size in bytes.
const WORD: usize = 4;

// ---------------------------------------------------------------------------
// Bit timing for 500 kbit/s @ 170 MHz FDCAN_CLK
//
// prescaler=17 (NBRP=16), NTSEG1=13 (field=13), NTSEG2=4 (field=3),
// NSJW=4 (field=3).  Bit time = 1 + 14 + 5 = 20 TQ @ 10 MHz = 500 kbit/s.
// (Loopback doesn't require a specific baud rate but keeping it consistent
// with the production driver simplifies verification.)
// ---------------------------------------------------------------------------

const NBTP_500KBPS: u32 = {
    let nbrp: u32 = 16;   // prescaler − 1
    let ntseg1: u32 = 13; // Tseg1 − 1
    let ntseg2: u32 = 3;  // Tseg2 − 1
    let nsjw: u32 = 3;    // SJW − 1
    (nsjw << 25) | (ntseg1 << 16) | (ntseg2 << 8) | nbrp
};

// ---------------------------------------------------------------------------
// NUCLEO-G474RE LED pin
//
// LD2 = PA5 (active-high, 1 kΩ series to VDD).
// ---------------------------------------------------------------------------

const LED_PIN: u32 = 5; // PA5

// ---------------------------------------------------------------------------
// Test frame constants
// ---------------------------------------------------------------------------

const TEST_ID: u32 = 0x123;
const TEST_DATA: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const TEST_DLC: u8 = 4;

// ---------------------------------------------------------------------------
// Raw MMIO helpers (file-local; identical pattern to can_fdcan.rs)
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_modify(base: usize, offset: usize, mask: u32, bits: u32) {
    let v = unsafe { reg_read(base, offset) };
    unsafe { reg_write(base, offset, (v & !mask) | bits) };
}

#[inline(always)]
unsafe fn mram_read(addr: usize) -> u32 {
    // SAFETY: caller guarantees addr is inside FDCAN1 message RAM.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
unsafe fn mram_write(addr: usize, val: u32) {
    // SAFETY: caller guarantees addr is inside FDCAN1 message RAM.
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

/// Configure a GPIO pin as push-pull output, high-speed, no pull.
///
/// `base` = GPIOx_BASE, `pin` = 0..15.
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32G4.
    unsafe {
        // MODER: 01 = output
        reg_modify(base, GPIO_MODER_OFFSET, 0b11 << (pin * 2), 0b01 << (pin * 2));
        // OSPEEDR: 10 = high speed
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

/// Set or clear a GPIO pin via BSRR.
unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    // SAFETY: base + BSRR is valid GPIO MMIO.
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

/// Configure a GPIO pin as alternate function.
///
/// `base` = GPIOx_BASE, `pin` = 0..15, `af` = 0..15.
unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    // SAFETY: base + AF offsets are valid GPIO MMIO on STM32G4.
    unsafe {
        // AF mode: MODER = 10
        reg_modify(base, GPIO_MODER_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
        // High speed
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
        // AFR — pins 0–7 in AFRL, 8–15 in AFRH
        if pin < 8 {
            let shift = pin * 4;
            reg_modify(base, GPIO_AFRL_OFFSET, 0xF << shift, af << shift);
        } else {
            let shift = (pin - 8) * 4;
            reg_modify(base, GPIO_AFRH_OFFSET, 0xF << shift, af << shift);
        }
    }
}

// ---------------------------------------------------------------------------
// FDCAN1 loopback initialisation
// ---------------------------------------------------------------------------

/// Enable FDCAN1 clock and configure PB8 (TX) / PB9 (RX) as AF9.
///
/// NUCLEO-G474RE: FDCAN1 is routed to PB8/PB9 (CN10 pins 3/5).
/// Alternate function AF9 selects FDCAN1 on these pins (RM0440 Table 17).
unsafe fn fdcan1_gpio_clock_init() {
    // SAFETY: RCC, GPIOB addresses are valid on STM32G474RE.
    unsafe {
        // Enable GPIOA (LED) and GPIOB (FDCAN pins) clocks.
        reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, GPIOAEN | GPIOBEN);
        let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET); // read-back delay

        // PB8 = FDCAN1_TX (AF9), PB9 = FDCAN1_RX (AF9).
        gpio_af(GPIOB_BASE, 8, 9);
        gpio_af(GPIOB_BASE, 9, 9);

        // Select PCLK1 as FDCAN kernel clock (CCIPR bits [25:24] = 0b10).
        // Default is HSE (0b00) which isn't running on this board.
        const RCC_CCIPR_OFFSET: usize = 0x88;
        reg_modify(RCC_BASE, RCC_CCIPR_OFFSET, 0b11 << 24, 0b10 << 24);

        // Enable FDCAN1 peripheral clock (APB1).
        reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, FDCAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET); // read-back delay
    }
}

/// Put FDCAN1 into INIT+CCE mode (required before touching configuration).
unsafe fn fdcan1_enter_init() {
    // SAFETY: FDCAN1_BASE + CCCR_OFFSET is a valid MMIO address.
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_INIT);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0 {}
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_CCE);
        // Classic CAN only — ensure FD and BRS bits are clear.
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_FDOE | CCCR_BRSE, 0);
    }
}

/// Leave INIT mode so FDCAN1 can participate in bus traffic.
unsafe fn fdcan1_leave_init() {
    // SAFETY: FDCAN1_BASE + CCCR_OFFSET is valid.
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_INIT | CCCR_CCE, 0);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) != 0 {}
    }
}

/// Configure FDCAN1 message RAM pointers — identical layout to can_fdcan.rs.
unsafe fn fdcan1_configure_mram() {
    // SAFETY: all base addresses and offsets are valid for STM32G474.
    unsafe {
        reg_write(FDCAN1_BASE, FDCAN_SIDFC_OFFSET, (28 << 16) | (MRAM_STDFILTER_OFFSET / WORD) as u32);
        reg_write(FDCAN1_BASE, FDCAN_XIDFC_OFFSET, (8  << 16) | (MRAM_EXTFILTER_OFFSET / WORD) as u32);
        reg_write(FDCAN1_BASE, FDCAN_RXF0C_OFFSET, (3  << 16) | (MRAM_RXF0_OFFSET    / WORD) as u32);
        reg_write(FDCAN1_BASE, FDCAN_RXF1C_OFFSET, (3  << 16) | (MRAM_RXF1_OFFSET    / WORD) as u32);
        reg_write(FDCAN1_BASE, FDCAN_TXEFC_OFFSET, (3  << 16) | (MRAM_TXEVT_OFFSET   / WORD) as u32);
        reg_write(FDCAN1_BASE, FDCAN_TXBC_OFFSET,  (3  << 16) | (MRAM_TXBUF_OFFSET   / WORD) as u32);
        // RXGFC: accept all non-matching std/ext frames to FIFO 0.
        reg_write(FDCAN1_BASE, FDCAN_RXGFC_OFFSET, 0x0000_0000);
    }
}

/// Enable FDCAN1 internal loopback (TEST.LBCK).
///
/// Must be called while FDCAN1 is in INIT + CCE mode.
/// Sequence per RM0440 §44.3.3:
///   1. CCCR.TEST = 1  (enables write access to the TEST register)
///   2. TEST.LBCK = 1  (routes TX output back to RX input internally)
///
/// In internal loopback the CAN TX pin is still driven, but the RX pin is
/// ignored.  Frames sent appear in RX FIFO 0 without needing a CAN bus.
unsafe fn fdcan1_enable_loopback() {
    // SAFETY: FDCAN1_BASE offsets are valid for STM32G474RE.
    unsafe {
        // Step 1 — enable test register access.
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_TEST);
        // Step 2 — internal loopback.
        reg_modify(FDCAN1_BASE, FDCAN_TEST_OFFSET, 0, TEST_LBCK);
    }
}

/// Full FDCAN1 init in internal-loopback mode.
///
/// Returns `true` if the peripheral came up correctly.
unsafe fn fdcan1_init_loopback() -> bool {
    unsafe {
        fdcan1_gpio_clock_init();
        fdcan1_enter_init();

        // Bit timing (500 kbit/s — consistent with production driver).
        reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);

        fdcan1_configure_mram();
        fdcan1_enable_loopback();

        // Leave init → peripheral is now operational in loopback mode.
        fdcan1_leave_init();

        // Sanity check: INIT bit must be 0 after leaving init.
        (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0
    }
}

// ---------------------------------------------------------------------------
// CAN TX / RX helpers (raw message RAM access)
// ---------------------------------------------------------------------------

/// Write a TX buffer element (slot 0) to FDCAN1 message RAM and request TX.
///
/// Transmits a standard-ID (11-bit) classic CAN frame.
///
/// Message RAM TX element layout (RM0440 §44.4.18):
/// - T0[28:18] = std ID, T0[30]=XTD=0, T0[29]=RTR=0
/// - T1[19:16] = DLC, T1[20]=BRS=0, T1[21]=FDF=0
/// - DB0 = data bytes [3:0] (little-endian)
/// - DB1 = data bytes [7:4] (little-endian)
unsafe fn fdcan1_send(id: u32, data: &[u8]) {
    // SAFETY: message RAM and FDCAN1 register addresses valid for STM32G474.
    unsafe {
        // Wait until TX FIFO has a free slot (TFQF = 0).
        loop {
            if (reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) & (1 << 21)) == 0 {
                break;
            }
        }

        let txbuf_addr = SRAMCAN_BASE + MRAM_TXBUF_OFFSET; // slot 0

        // T0: standard ID in bits [28:18], XTD=0, RTR=0.
        let t0 = (id & 0x7FF) << 18;
        // T1: DLC in bits [19:16].
        let dlc = data.len().min(8) as u32;
        let t1 = dlc << 16;

        // Pack up to 8 payload bytes into two 32-bit words (little-endian).
        let mut raw = [0u8; 8];
        raw[..data.len().min(8)].copy_from_slice(&data[..data.len().min(8)]);
        let db0 = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let db1 = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);

        mram_write(txbuf_addr,              t0);
        mram_write(txbuf_addr +     WORD,   t1);
        mram_write(txbuf_addr + 2 * WORD,   db0);
        mram_write(txbuf_addr + 3 * WORD,   db1);

        // Request transmission of TX buffer 0.
        reg_write(FDCAN1_BASE, FDCAN_TXBAR_OFFSET, 1 << 0);
    }
}

/// Result of a single RX FIFO 0 read.
struct RxFrame {
    id: u32,
    dlc: u8,
    data: [u8; 8],
}

/// Read one frame from RX FIFO 0 and acknowledge it.
///
/// Returns `None` if the FIFO is empty.
///
/// Message RAM RX element layout (RM0440 §44.4.16):
/// - R0[28:18] = std ID (XTD=0), R0[30]=XTD, R0[29]=RTR
/// - R1[19:16] = DLC
/// - DB0/DB1 = payload bytes (little-endian)
unsafe fn fdcan1_rx_fifo0_read() -> Option<RxFrame> {
    // SAFETY: FDCAN1 and message RAM addresses valid for STM32G474.
    unsafe {
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);
        let fill = rxf0s & 0x7F; // F0FL bits [6:0]
        if fill == 0 {
            return None;
        }

        let get_idx = ((rxf0s >> 8) & 0x3F) as usize; // F0GI bits [13:8]
        let elem_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * 4 * WORD;

        let r0  = mram_read(elem_addr);
        let r1  = mram_read(elem_addr + WORD);
        let db0 = mram_read(elem_addr + 2 * WORD);
        let db1 = mram_read(elem_addr + 3 * WORD);

        // Decode ID — standard only (XTD=0 expected for this test).
        let is_ext = (r0 & (1 << 30)) != 0;
        let id = if is_ext {
            r0 & 0x1FFF_FFFF
        } else {
            (r0 >> 18) & 0x7FF
        };

        let dlc = ((r1 >> 16) & 0xF) as u8;

        let w0 = db0.to_le_bytes();
        let w1 = db1.to_le_bytes();
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&w0);
        data[4..8].copy_from_slice(&w1);

        // Acknowledge — release the slot back to the controller.
        reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32);

        Some(RxFrame { id, dlc, data })
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // ------------------------------------------------------------------
    // 1. Clocks — 170 MHz boost mode
    // ------------------------------------------------------------------
    let sys_clock = configure_clocks_g474();

    // ------------------------------------------------------------------
    // 2. DWT timer
    // ------------------------------------------------------------------
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // ------------------------------------------------------------------
    // 3. UART — USART2 115200 8N1 on PA2 (TX) / PA3 (RX)
    // ------------------------------------------------------------------
    let mut uart = PolledUart::new();
    uart.init();
    let _ = writeln!(uart, "\r\n--- CAN loopback test starting @ {} MHz ---",
                     sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    // ------------------------------------------------------------------
    // GPIOA clock is enabled by PolledUart::init(); configure PA5 here.
    unsafe {
        gpio_output(GPIOA_BASE, LED_PIN);
    }

    // ------------------------------------------------------------------
    // 5. FDCAN1 in internal-loopback mode
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Initialising FDCAN1 (loopback mode)...");
    let init_ok = unsafe { fdcan1_init_loopback() };
    if !init_ok {
        let _ = writeln!(uart, "FAIL: FDCAN1 did not leave init mode");
        blink_forever(&mut timer, 500_000);
    }
    let _ = writeln!(uart, "FDCAN1 init OK, loopback enabled.");

    // ------------------------------------------------------------------
    // 6. Send test frame: ID=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF]
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Sending: ID=0x{:03X}  data={:02X?}",
                     TEST_ID, &TEST_DATA[..]);
    unsafe { fdcan1_send(TEST_ID, &TEST_DATA) };

    // ------------------------------------------------------------------
    // 7. Poll RX FIFO 0 for the looped-back frame (500 ms timeout)
    // ------------------------------------------------------------------
    let deadline = timer.system_time_us_64() + 500_000;
    let rx = loop {
        let frame = unsafe { fdcan1_rx_fifo0_read() };
        if let Some(f) = frame {
            break Some(f);
        }
        if timer.system_time_us_64() >= deadline {
            break None;
        }
    };

    // ------------------------------------------------------------------
    // 8. Verify and report result
    // ------------------------------------------------------------------
    let passed = match rx {
        None => {
            let _ = writeln!(uart, "FAIL: RX timeout — no frame received within 500 ms");
            false
        }
        Some(frame) => {
            let _ = writeln!(uart, "Received: ID=0x{:03X}  DLC={}  data={:02X?}",
                             frame.id,
                             frame.dlc,
                             &frame.data[..frame.dlc.min(8) as usize]);

            let id_ok   = frame.id == TEST_ID;
            let dlc_ok  = frame.dlc == TEST_DLC;
            let data_ok = &frame.data[..TEST_DLC as usize] == &TEST_DATA[..];

            if !id_ok {
                let _ = writeln!(uart, "  ID mismatch: expected 0x{:03X}, got 0x{:03X}",
                                 TEST_ID, frame.id);
            }
            if !dlc_ok {
                let _ = writeln!(uart, "  DLC mismatch: expected {}, got {}",
                                 TEST_DLC, frame.dlc);
            }
            if !data_ok {
                let _ = writeln!(uart, "  Data mismatch");
            }

            id_ok && dlc_ok && data_ok
        }
    };

    if passed {
        let _ = writeln!(uart, "*** RESULT: PASS ***");
    } else {
        let _ = writeln!(uart, "*** RESULT: FAIL ***");
    }
    uart.flush();

    // ------------------------------------------------------------------
    // 9. Blink LED to signal completion
    //    PASS = fast (100 ms), FAIL = slow (500 ms)
    // ------------------------------------------------------------------
    let blink_period = if passed { 100_000u64 } else { 500_000u64 };
    blink_forever(&mut timer, blink_period);
}

/// Toggle PA5 (LED) indefinitely at the given half-period (µs).
fn blink_forever(timer: &mut DwtTimer, half_period_us: u64) -> ! {
    let mut led_on = false;
    let mut last = timer.system_time_us_64();
    loop {
        let now = timer.system_time_us_64();
        if now.wrapping_sub(last) >= half_period_us {
            last = now;
            led_on = !led_on;
            unsafe { gpio_set(GPIOA_BASE, LED_PIN, led_on) };
        }
    }
}

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
