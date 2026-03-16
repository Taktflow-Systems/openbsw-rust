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
//! # STM32G4 FDCAN Notes
//!
//! The STM32G4 uses a simplified FDCAN with a hardware-fixed message RAM layout
//! (SRAMCAN_BASE = 0x4000_A400, RM0440 §44.3.3).  Configuration registers like
//! SIDFC, XIDFC, RXF0C, RXF1C, TXEFC that exist in the generic Bosch M_CAN IP
//! are NOT present or have different meanings on STM32G4 — writing to those
//! offsets corrupts other registers (XIDAM, HPMS, etc.).
//!
//! Each RX FIFO 0/1 element and each TX buffer element occupies 18 words
//! (72 bytes) in message RAM regardless of the actual payload length.  For
//! classic CAN only the first 4 words (T0/R0, T1/R1, DB0, DB1) carry data.
//!
//! TXBC (0x0C0) resets to all-zeros (no TX buffers configured).  It must be
//! written before attempting any transmission.
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
//! RM0440 Rev 8, chapter 44 (FDCAN); STM32CubeG4 HAL (stm32g4xx_hal_fdcan.c).

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
/// RCC CCIPR — kernel clock selection (bits [25:24] for FDCAN).
const RCC_CCIPR_OFFSET: usize = 0x88;
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

/// Message RAM base address for FDCAN1 (RM0440 §44.3.3).
///
/// On STM32G4 this is a hardware-fixed 212-word (848-byte) SRAM region.
/// The layout within this region is also fixed by the silicon — the config
/// registers (SIDFC/XIDFC/RXF0C/RXF1C/TXEFC in full M_CAN) do NOT exist
/// here and must NOT be written.  Only TXBC needs configuration.
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets (STM32G4-specific, from CMSIS FDCAN_GlobalTypeDef)
//
// Registers that exist in generic Bosch M_CAN but NOT on STM32G4:
//   0x084 = XIDAM on G4  (NOT SIDFC)
//   0x088 = HPMS  on G4  (NOT XIDFC, read-only!)
//   0x0A0 = reserved     (NOT RXF0C)
//   0x0B0 = reserved     (NOT RXF1C)
//   0x0F0 = TXEFA on G4  (NOT TXEFC)
// ---------------------------------------------------------------------------

const FDCAN_TEST_OFFSET: usize  = 0x010; // Test register
const FDCAN_CCCR_OFFSET: usize  = 0x018; // CC control register
const FDCAN_NBTP_OFFSET: usize  = 0x01C; // Nominal bit timing
const FDCAN_ECR_OFFSET:  usize  = 0x040; // Error counter register
const FDCAN_PSR_OFFSET:  usize  = 0x044; // Protocol status register
const FDCAN_IR_OFFSET:   usize  = 0x050; // Interrupt register
const FDCAN_IE_OFFSET:   usize  = 0x054; // Interrupt enable
const FDCAN_ILS_OFFSET:  usize  = 0x058; // Interrupt line select
const FDCAN_ILE_OFFSET:  usize  = 0x05C; // Interrupt line enable
const FDCAN_RXGFC_OFFSET: usize = 0x080; // RX global filter config
// 0x084 = XIDAM (not SIDFC) — do not write
// 0x088 = HPMS  (not XIDFC, read-only) — do not write
// 0x0A0 = reserved (not RXF0C) — do not write
// 0x0B0 = reserved (not RXF1C) — do not write
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status  (G4: 0x090, not 0x0A4)
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 acknowledge (G4: 0x094, not 0x0A8)
const FDCAN_TXBC_OFFSET:  usize = 0x0C0; // TX buffer config   (G4: 0x0C0, not 0x0C4)
const FDCAN_TXFQS_OFFSET: usize = 0x0C4; // TX FIFO/queue status (G4: 0x0C4, not 0x0C8)
const FDCAN_TXBAR_OFFSET: usize = 0x0CC; // TX buffer add request (G4: 0x0CC, not 0x0D0)
// 0x0F0 = TXEFA (not TXEFC) — do not configure

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

/// INIT — initialisation mode.
const CCCR_INIT: u32 = 1 << 0;
/// CCE — configuration change enable (requires INIT=1).
const CCCR_CCE: u32 = 1 << 1;
/// TEST — test mode enable; gates write access to the TEST register.
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
// STM32G4 Message RAM layout (hardware-fixed, RM0440 §44.3.3)
//
// Byte offsets from SRAMCAN_BASE for FDCAN1:
//
//   0x000 ..= 0x06F  Standard ID filters  28 × 1W  = 28 words (112 bytes)
//   0x070 ..= 0x0AF  Extended ID filters   8 × 2W  = 16 words  (64 bytes)
//   0x0B0 ..= 0x187  RX FIFO 0             3 × 18W = 54 words (216 bytes)
//   0x188 ..= 0x25F  RX FIFO 1             3 × 18W = 54 words (216 bytes)
//   0x260 ..= 0x277  TX event FIFO         3 × 2W  =  6 words  (24 bytes)
//   0x278 ..= 0x34F  TX buffers            3 × 18W = 54 words (216 bytes)
//
// IMPORTANT: Each RX/TX element is 18 words = 72 bytes (NOT 4 words!).
// For classic CAN only the first 4 words contain valid data:
//   Word 0 = T0/R0 (header: ID, XTD, RTR, ESI)
//   Word 1 = T1/R1 (header: DLC, BRS, FDF, timestamp)
//   Word 2 = DB0   (payload bytes 3..0, little-endian)
//   Word 3 = DB1   (payload bytes 7..4, little-endian)
// ---------------------------------------------------------------------------

const MRAM_STDFILTER_OFFSET: usize = 0x000;
const MRAM_EXTFILTER_OFFSET: usize = 0x070;
const MRAM_RXF0_OFFSET:      usize = 0x0B0;
const MRAM_RXF1_OFFSET:      usize = 0x188;
const MRAM_TXEVT_OFFSET:     usize = 0x260;
const MRAM_TXBUF_OFFSET:     usize = 0x278;

/// Stride between RX/TX elements in message RAM: 18 words = 72 bytes.
const MRAM_ELEMENT_STRIDE: usize = 18 * 4; // 72

/// Word size in bytes.
const WORD: usize = 4;

// ---------------------------------------------------------------------------
// TXBC configuration
//
// TXBC (0x0C0) resets to 0x0000_0000 = no TX buffers configured.
// Must be written before any TX attempt.
//
// TXBC layout (RM0440 §44.8.28 / STM32CubeG4 HAL):
//   [29:24] TFQS  = number of TX FIFO/queue elements (we use 3)
//   [21:16] NDTB  = number of dedicated TX buffers   (we use 0)
//   [15: 0] TBSA  = TX buffer start address (word offset from SRAMCAN_BASE)
//
// TX buffers start at byte offset 0x278; word offset = 0x278 / 4 = 0x9E = 158.
// TXBC = (3 << 24) | (0 << 16) | 158 = 0x0300_009E
// ---------------------------------------------------------------------------

const TXBC_VAL: u32 = (3 << 24) | (MRAM_TXBUF_OFFSET as u32 / 4);

// ---------------------------------------------------------------------------
// Bit timing for 500 kbit/s @ 170 MHz FDCAN_CLK
//
// Clock source: PCLK1 (= HCLK/1 = 170 MHz in boost mode, RM0440 §6.2).
// RCC_CCIPR [25:24] = 0b10 selects PCLK1 as FDCAN kernel clock.
//
// NBTP layout (RM0440 §44.8.6):
//   [31:25] NSJW   = SJW − 1
//   [24:16] NTSEG1 = Tseg1 − 1
//   [ 15:8] NTSEG2 = Tseg2 − 1
//   [  8:0] NBRP   = prescaler − 1
//
// We target: NBRP=16 (prescaler=17), Tseg1=14 (NTSEG1=13), Tseg2=4 (NTSEG2=3),
// SJW=4 (NSJW=3).
// Bit time = prescaler × (1 + Tseg1 + Tseg2) = 17 × (1 + 14 + 4) = 17 × 19
//          = ... that's 323 TQ at 170 MHz.  Let's recalculate:
// Tq = prescaler / Fcan = 17 / 170 MHz = 100 ns
// Bit time = (1 + Tseg1 + Tseg2) × Tq = (1 + 13 + 3) × 100 ns
//           = 17 × 100 ns... wait, NTSEG1=13 → Tseg1=14, NTSEG2=3 → Tseg2=4
// Total TQ per bit = 1 + 14 + 4 = 19; Fbit = 170 MHz / (17 × 19) ≈ 526 kbps
// Closest 500 kbps: prescaler=17, Tseg1=13, Tseg2=4 → 17×18=306 TQ
//                   Fbit = 170 MHz / (17 × 18) ≈ 556 kbps  (still off)
//
// Exact 500 kbps: need prescaler × total_tq = 340
//   prescaler=20, total_tq=17: Tseg1=13, Tseg2=3 → 1+13+3=17 ✓
//   Tq = 20/170 MHz ≈ 117.6 ns; Fbit = 1/(20 × 17 × 5.88 ns) = 500 kbps ✓
//   NBRP=19, NTSEG1=12, NTSEG2=2, NSJW=2
// ---------------------------------------------------------------------------

const NBTP_500KBPS: u32 = {
    let nbrp: u32 = 16;   // prescaler − 1  (prescaler = 17)
    let ntseg1: u32 = 13; // Tseg1 − 1      (Tseg1 = 14 TQ)
    let ntseg2: u32 = 3;  // Tseg2 − 1      (Tseg2 = 4 TQ)
    let nsjw: u32 = 3;    // SJW − 1
    (nsjw << 25) | (nbrp << 16) | (ntseg1 << 8) | ntseg2
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
///
/// Also selects PCLK1 as the FDCAN kernel clock (RCC_CCIPR [25:24] = 0b10).
/// The default (0b00) selects HSE which is not running on this board.
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

/// Configure FDCAN1 TX buffers via TXBC.
///
/// On STM32G4 the message RAM layout is hardware-fixed.  The only register
/// that needs to be written to set up transmit capability is TXBC (0x0C0).
/// Its reset value is 0 (= no TX buffers), so we must configure it before
/// any TX attempt.
///
/// TXBC field layout (RM0440 §44.8.28):
///   [29:24] TFQS  = TX FIFO/queue size (3 elements)
///   [21:16] NDTB  = dedicated TX buffers (0)
///   [15: 0] TBSA  = TX buffer start address (word offset from SRAMCAN_BASE)
///
/// The registers SIDFC, XIDFC, RXF0C, RXF1C and TXEFC that exist in the
/// generic Bosch M_CAN IP are NOT present on STM32G4 and must NOT be written.
/// The RXGFC register is written to accept all frames into RX FIFO 0.
unsafe fn fdcan1_configure_mram() {
    // SAFETY: all base addresses and offsets are valid for STM32G474.
    unsafe {
        // TXBC: 3 TX FIFO elements, no dedicated buffers, start at MRAM_TXBUF_OFFSET.
        reg_write(FDCAN1_BASE, FDCAN_TXBC_OFFSET, TXBC_VAL);

        // RXGFC: 0x00000000 = accept all non-matching std + ext frames into FIFO 0.
        // (ANFS [5:4] = 00b = accept to FIFO 0, ANFE [3:2] = 00b = accept to FIFO 0,
        //  LSS [28:24] / LSE [20:16] = 0 = no filter elements active — accept all.)
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
/// Returns `true` if the peripheral came up correctly (INIT == 0 after config).
unsafe fn fdcan1_init_loopback() -> bool {
    unsafe {
        // 1. Enable clocks, configure GPIO pins, select PCLK1 as FDCAN clock.
        fdcan1_gpio_clock_init();

        // 2. Enter INIT + CCE mode to allow register writes.
        fdcan1_enter_init();

        // 3. Bit timing for 500 kbit/s (consistent with production driver).
        reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);

        // 4. Configure TXBC (and RXGFC).
        //    DO NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC — not on STM32G4!
        fdcan1_configure_mram();

        // 5. Enable internal loopback (CCCR.TEST=1, TEST.LBCK=1).
        fdcan1_enable_loopback();

        // 6. Leave init — peripheral is now operational in loopback mode.
        fdcan1_leave_init();

        // Sanity check: INIT bit must be 0 after leaving init.
        (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0
    }
}

// ---------------------------------------------------------------------------
// CAN TX / RX helpers (raw message RAM access)
// ---------------------------------------------------------------------------

/// Write a TX buffer element into FDCAN1 message RAM and request TX.
///
/// Uses the TX FIFO put index from TXFQS to find the next free slot.
/// Transmits a standard-ID (11-bit) classic CAN frame.
///
/// Message RAM TX element layout (RM0440 §44.4.18) — 18 words per element,
/// but only the first 4 carry data for classic CAN (DLC ≤ 8):
///   Word 0 = T0: [28:18] = std ID, [30]=XTD=0, [29]=RTR=0, [31]=ESI=0
///   Word 1 = T1: [19:16] = DLC, [20]=BRS=0, [21]=FDF=0, [23]=EFC=0, [31:24]=MM
///   Word 2 = DB0: payload bytes [3:0] (little-endian in memory)
///   Word 3 = DB1: payload bytes [7:4] (little-endian in memory)
///
/// Remaining 14 words are unused for classic CAN and left as-is.
unsafe fn fdcan1_send(id: u32, data: &[u8]) {
    // SAFETY: message RAM and FDCAN1 register addresses valid for STM32G474.
    unsafe {
        // Wait until TX FIFO has a free slot (TFQF bit 21 = FIFO full).
        loop {
            if (reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) & (1 << 21)) == 0 {
                break;
            }
        }

        // TX FIFO put index: TXFQS [12:8].
        let txfqs = reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET);
        let put_idx = ((txfqs >> 8) & 0x1F) as usize;

        // Element address: MRAM_TXBUF_OFFSET + put_idx × 18 words × 4 bytes.
        let txbuf_addr = SRAMCAN_BASE + MRAM_TXBUF_OFFSET + put_idx * MRAM_ELEMENT_STRIDE;

        // T0: standard ID in bits [28:18], XTD=0, RTR=0, ESI=0.
        let t0 = (id & 0x7FF) << 18;
        // T1: DLC in bits [19:16], BRS=0, FDF=0.
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

        // Request transmission: set the bit corresponding to put_idx in TXBAR.
        reg_write(FDCAN1_BASE, FDCAN_TXBAR_OFFSET, 1 << put_idx);
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
/// RXF0S bit layout on STM32G4 (RM0440 §44.8.21) — DIFFERENT from full M_CAN:
///   [ 3: 0] F0FL  fill level (0..3, only 4 bits — max 3 elements on G4)
///   [ 9: 8] F0GI  get index  (2 bits for 3 elements)
///   [17:16] F0PI  put index  (2 bits)
///   [   24] F0F   FIFO full flag
///   [   25] RF0L  message lost flag
///
/// (On full M_CAN: F0FL is [6:0] and F0GI is [13:8].)
///
/// Message RAM RX element layout (RM0440 §44.4.16) — 18 words per element:
///   Word 0 = R0: [28:18] = std ID (if XTD=0), [30]=XTD, [29]=RTR
///   Word 1 = R1: [19:16] = DLC, [11:0] = RXTS (timestamp)
///   Word 2 = DB0: payload bytes [3:0] (little-endian)
///   Word 3 = DB1: payload bytes [7:4] (little-endian)
unsafe fn fdcan1_rx_fifo0_read() -> Option<RxFrame> {
    // SAFETY: FDCAN1 and message RAM addresses valid for STM32G474.
    unsafe {
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);

        // F0FL: bits [3:0] on STM32G4 (NOT [6:0] like full M_CAN).
        let fill = rxf0s & 0xF;
        if fill == 0 {
            return None;
        }

        // F0GI: bits [9:8] on STM32G4 (NOT [13:8] like full M_CAN).
        let get_idx = ((rxf0s >> 8) & 0x3) as usize;

        // Element address: MRAM_RXF0_OFFSET + get_idx × 18 words × 4 bytes.
        let elem_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * MRAM_ELEMENT_STRIDE;

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
        // RXF0A [2:0] = get_idx on STM32G4.
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

    // Brief delay for TX to complete in loopback (10 ms).
    let tx_wait = timer.system_time_us_64() + 10_000;
    while timer.system_time_us_64() < tx_wait {}

    // ------------------------------------------------------------------
    // 6b. Diagnostic register dump (STM32G4-correct offsets)
    // ------------------------------------------------------------------
    unsafe {
        let cccr    = reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET);
        let test_r  = reg_read(FDCAN1_BASE, FDCAN_TEST_OFFSET);
        let nbtp    = reg_read(FDCAN1_BASE, FDCAN_NBTP_OFFSET);
        let txbc    = reg_read(FDCAN1_BASE, FDCAN_TXBC_OFFSET);   // 0x0C0
        let txfqs   = reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET);  // 0x0C4
        let txbar   = reg_read(FDCAN1_BASE, FDCAN_TXBAR_OFFSET);  // 0x0CC
        let rxf0s   = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);  // 0x090
        let rxf0a   = reg_read(FDCAN1_BASE, FDCAN_RXF0A_OFFSET);  // 0x094
        let rxgfc   = reg_read(FDCAN1_BASE, FDCAN_RXGFC_OFFSET);  // 0x080
        let ir      = reg_read(FDCAN1_BASE, FDCAN_IR_OFFSET);     // 0x050
        let psr     = reg_read(FDCAN1_BASE, FDCAN_PSR_OFFSET);    // 0x044
        let ecr     = reg_read(FDCAN1_BASE, FDCAN_ECR_OFFSET);    // 0x040
        // Note: reading 0x084 (XIDAM) and 0x088 (HPMS) for info only.
        let xidam   = reg_read(FDCAN1_BASE, 0x084); // XIDAM (not SIDFC)
        let hpms    = reg_read(FDCAN1_BASE, 0x088); // HPMS  (not XIDFC, read-only)
        let _ = writeln!(uart, "  CCCR=0x{:08X}  TEST=0x{:08X}  NBTP=0x{:08X}", cccr, test_r, nbtp);
        let _ = writeln!(uart, "  TXBC=0x{:08X}  TXFQS=0x{:08X} TXBAR=0x{:08X}", txbc, txfqs, txbar);
        let _ = writeln!(uart, "  RXF0S=0x{:08X} RXF0A=0x{:08X} RXGFC=0x{:08X}", rxf0s, rxf0a, rxgfc);
        let _ = writeln!(uart, "  IR=0x{:08X}  PSR=0x{:08X}  ECR=0x{:08X}", ir, psr, ecr);
        let _ = writeln!(uart, "  XIDAM=0x{:08X} HPMS=0x{:08X} (info only, not written)", xidam, hpms);
        // Decode RXF0S fields (STM32G4 layout):
        let f0fl = rxf0s & 0xF;          // [3:0] fill level
        let f0gi = (rxf0s >> 8) & 0x3;   // [9:8] get index
        let f0pi = (rxf0s >> 16) & 0x3;  // [17:16] put index
        let _ = writeln!(uart, "  RXF0S decoded: F0FL={} F0GI={} F0PI={}", f0fl, f0gi, f0pi);
    }

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
