// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Polled UART driver for STM32G474RE — USART2 at 115200 baud.
//!
//! The G4 family uses a revised USART register map compared to F4:
//!
//! | Register | G4 name | F4 equivalent |
//! |----------|---------|---------------|
//! | Status   | ISR     | SR            |
//! | TX data  | TDR     | DR (write)    |
//! | RX data  | RDR     | DR (read)     |
//!
//! Key status bits:
//! - `ISR.TXE`  (bit 7) — TX data register empty (safe to write TDR).
//! - `ISR.RXNE` (bit 5) — RX data register not empty (data ready in RDR).
//! - `ISR.TC`   (bit 6) — transmission complete (last byte shifted out).
//! - `ISR.ORE`  (bit 3) — overrun error.
//!
//! # Clock assumptions
//!
//! USART2 is on APB1.  After `configure_clocks_g474()` APB1 = 170 MHz
//! (no APB1 divider).  BRR = 170_000_000 / 115_200 = 1476 (0x5C4).
//!
//! # Usage
//!
//! ```ignore
//! let mut uart = PolledUart::new();
//! uart.init();
//! uart.write_bytes(b"Hello G474\r\n");
//! if let Some(b) = uart.read_byte() { /* process */ }
//! ```
//!
//! # Reference
//!
//! RM0440 Rev 8, chapter 37 (USART).

// ---------------------------------------------------------------------------
// Peripheral base address
// ---------------------------------------------------------------------------

/// USART2 register block base address (RM0440 §37.7 memory map).
const USART2_BASE: usize = 0x4000_4400;

// ---------------------------------------------------------------------------
// USART register offsets (byte offsets from USART2_BASE)
// ---------------------------------------------------------------------------

/// Control register 1 — UE, TE, RE bits.
const USART_CR1_OFFSET: usize = 0x00;
/// Control register 2 — stop bits.
const USART_CR2_OFFSET: usize = 0x04;
/// Control register 3 — hardware flow control.
#[allow(dead_code)]
const USART_CR3_OFFSET: usize = 0x08;
/// Baud rate register.
const USART_BRR_OFFSET: usize = 0x0C;
/// Interrupt and status register (read-only).
const USART_ISR_OFFSET: usize = 0x1C;
/// Interrupt flag clear register (write-only).
const USART_ICR_OFFSET: usize = 0x20;
/// Receive data register (read-only; 9-bit field [8:0]).
const USART_RDR_OFFSET: usize = 0x24;
/// Transmit data register (write-only; 9-bit field [8:0]).
const USART_TDR_OFFSET: usize = 0x28;

// ---------------------------------------------------------------------------
// CR1 bit masks
// ---------------------------------------------------------------------------

/// UE — USART enable.
const CR1_UE: u32 = 1 << 0;
/// TE — transmitter enable.
const CR1_TE: u32 = 1 << 3;
/// RE — receiver enable.
const CR1_RE: u32 = 1 << 2;
/// M0 / M1 — word length (0,0 = 8 data bits).
const CR1_M0: u32 = 1 << 12;
const CR1_M1: u32 = 1 << 28;
/// PCE — parity control enable; 0 = no parity.
const CR1_PCE: u32 = 1 << 10;
/// OVER8 — oversampling by 8; 0 = by 16.
const CR1_OVER8: u32 = 1 << 15;

// ---------------------------------------------------------------------------
// CR2 bit masks
// ---------------------------------------------------------------------------

/// STOP[13:12] — stop bits: 00 = 1 stop bit.
const CR2_STOP_MASK: u32 = 0b11 << 12;

// ---------------------------------------------------------------------------
// ISR bit masks
// ---------------------------------------------------------------------------

/// RXNE — RX data register not empty (bit 5).
const ISR_RXNE: u32 = 1 << 5;
/// TXE — TX data register empty (bit 7).
const ISR_TXE: u32 = 1 << 7;
/// TC — transmission complete (bit 6).
const ISR_TC: u32 = 1 << 6;
/// ORE — overrun error (bit 3).
const ISR_ORE: u32 = 1 << 3;

// ---------------------------------------------------------------------------
// ICR bit masks
// ---------------------------------------------------------------------------

/// ORECF — clear overrun error flag.
const ICR_ORECF: u32 = 1 << 3;
/// TCCF — clear TC flag (required after last byte to detect completion).
const ICR_TCCF: u32 = 1 << 6;

// ---------------------------------------------------------------------------
// Baud rate register value
//
// G4 USART BRR formula (OVER8=0, 16× oversampling):
//   BRR = f_PCLK / baud_rate
//   BRR = 170_000_000 / 115_200 = 1476.0  → 1476 (0x5C4)
//   Actual baud = 170_000_000 / 1476 = 115_176 bps (error < 0.03 %)
// ---------------------------------------------------------------------------

/// BRR value for 115200 baud with PCLK1 = 170 MHz.
const BRR_115200: u32 = 1476;

// ---------------------------------------------------------------------------
// Raw MMIO helpers
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

// ---------------------------------------------------------------------------
// RCC helper — enable USART2 clock
// ---------------------------------------------------------------------------

/// RCC base address.
const RCC_BASE: usize = 0x4002_1000;
/// RCC_APB1ENR1 offset.
const RCC_APB1ENR1_OFFSET: usize = 0x58;
/// RCC AHB2ENR — GPIOA clock enable (bit 0).
const RCC_AHB2ENR_OFFSET: usize = 0x4C;
/// GPIOA register block base address.
const GPIOA_BASE: usize = 0x4800_0000;
/// GPIOA register offsets.
const GPIO_MODER_OFFSET: usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_AFRL_OFFSET: usize = 0x20;
/// USART2EN bit in APB1ENR1.
const USART2EN: u32 = 1 << 17;

// ---------------------------------------------------------------------------
// PolledUart
// ---------------------------------------------------------------------------

/// Polled UART driver for USART2 on STM32G474RE.
///
/// All TX and RX operations are synchronous (busy-wait on status flags).
/// This is suitable for debug output and early-bringup traces; for production
/// use an interrupt-driven or DMA-based driver.
///
/// The struct contains no heap allocation and is safe to place in `static`
/// memory or on the stack.
pub struct PolledUart {
    /// Cached RX overrun count for diagnostics.
    overrun_count: u32,
    /// `true` after `init()` succeeds.
    initialised: bool,
}

impl PolledUart {
    /// Creates a new, uninitialised UART handle.
    ///
    /// Call [`init()`](Self::init) before using any TX/RX methods.
    pub const fn new() -> Self {
        Self {
            overrun_count: 0,
            initialised: false,
        }
    }

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    /// Initialise USART2 for 115200 8N1 polled operation.
    ///
    /// Steps performed:
    /// 1. Enable USART2 clock via RCC_APB1ENR1.
    /// 2. Disable USART while configuring (UE=0).
    /// 3. Set CR2: 1 stop bit (STOP=00).
    /// 4. Set CR1: 8 data bits (M=00), no parity, 16× oversampling.
    /// 5. Write BRR for 115200 baud.
    /// 6. Enable USART, TX, and RX (UE=1, TE=1, RE=1).
    ///
    /// # Note on GPIO muxing
    ///
    /// This function does not configure GPIO alternate functions (PA2/PA3
    /// for USART2 TX/RX).  The caller must configure the GPIO AF before
    /// calling `init()`, or use the HAL GPIO API.
    ///
    /// # Safety
    ///
    /// Accesses RCC and USART2 MMIO registers directly.  Must be called
    /// once at startup after clocks are configured.
    pub fn init(&mut self) {
        // SAFETY: RCC, GPIOA, and USART2 addresses are valid on STM32G474RE.
        unsafe {
            // 0. Enable GPIOA clock and configure PA2 (TX) / PA3 (RX) as AF7.
            reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, 1); // GPIOAEN
            let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET); // read-back delay

            // PA2 = AF mode (MODER[5:4] = 0b10), PA3 = AF mode (MODER[7:6] = 0b10)
            reg_modify(
                GPIOA_BASE,
                GPIO_MODER_OFFSET,
                (0b11 << 4) | (0b11 << 6),    // clear
                (0b10 << 4) | (0b10 << 6),     // set AF mode
            );
            // High speed for PA2 and PA3
            reg_modify(
                GPIOA_BASE,
                GPIO_OSPEEDR_OFFSET,
                (0b11 << 4) | (0b11 << 6),
                (0b10 << 4) | (0b10 << 6),
            );
            // AFRL: AF7 for PA2 [bits 11:8] and PA3 [bits 15:12]
            reg_modify(
                GPIOA_BASE,
                GPIO_AFRL_OFFSET,
                (0xF << 8) | (0xF << 12),
                (7 << 8) | (7 << 12),
            );

            // 1. Enable USART2 peripheral clock.
            reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, USART2EN);
            // Read-back delay to let the clock gate propagate.
            let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET);

            // 2. Disable USART while configuring.
            reg_modify(USART2_BASE, USART_CR1_OFFSET, CR1_UE, 0);

            // 3. CR2: 1 stop bit (STOP[13:12] = 00).
            reg_modify(USART2_BASE, USART_CR2_OFFSET, CR2_STOP_MASK, 0);

            // 4. CR1: 8 data bits (M0=0, M1=0), no parity (PCE=0),
            //    16× oversampling (OVER8=0).  Clear all word-length and
            //    parity bits to guarantee a clean 8N1 configuration.
            reg_modify(
                USART2_BASE,
                USART_CR1_OFFSET,
                CR1_M0 | CR1_M1 | CR1_PCE | CR1_OVER8,
                0,
            );

            // 5. BRR for 115200 baud (see constant comment for calculation).
            reg_write(USART2_BASE, USART_BRR_OFFSET, BRR_115200);

            // 6. Enable USART + TX + RX.
            reg_modify(USART2_BASE, USART_CR1_OFFSET, 0, CR1_UE | CR1_TE | CR1_RE);
        }

        self.initialised = true;
    }

    // -----------------------------------------------------------------------
    // Transmit
    // -----------------------------------------------------------------------

    /// Transmit a single byte, blocking until the TDR is ready.
    ///
    /// Returns immediately without transmitting if the UART is not
    /// initialised.
    #[inline]
    pub fn write_byte(&self, byte: u8) {
        if !self.initialised {
            return;
        }
        // SAFETY: USART2_BASE + offsets are valid MMIO addresses.
        unsafe {
            // Wait for TXE (TX data register empty).
            while (reg_read(USART2_BASE, USART_ISR_OFFSET) & ISR_TXE) == 0 {}
            // Write the byte.  Only the low 8 bits are used (9-bit field, M=0).
            reg_write(USART2_BASE, USART_TDR_OFFSET, byte as u32);
        }
    }

    /// Transmit a byte slice, blocking for each byte.
    #[inline]
    pub fn write_bytes(&self, data: &[u8]) {
        for &b in data {
            self.write_byte(b);
        }
    }

    /// Transmit a string slice.
    #[inline]
    pub fn write_str(&self, s: &str) {
        self.write_bytes(s.as_bytes());
    }

    /// Transmit a byte slice followed by `"\r\n"`.
    #[inline]
    pub fn write_line(&self, data: &[u8]) {
        self.write_bytes(data);
        self.write_bytes(b"\r\n");
    }

    /// Block until the last byte has been physically shifted out (TC flag).
    ///
    /// Useful before entering low-power mode or reconfiguring the UART.
    #[inline]
    pub fn flush(&self) {
        if !self.initialised {
            return;
        }
        // SAFETY: USART2_BASE + offsets are valid MMIO addresses.
        unsafe {
            while (reg_read(USART2_BASE, USART_ISR_OFFSET) & ISR_TC) == 0 {}
            // Clear TC so the flag does not trip the next flush() call.
            reg_write(USART2_BASE, USART_ICR_OFFSET, ICR_TCCF);
        }
    }

    // -----------------------------------------------------------------------
    // Receive
    // -----------------------------------------------------------------------

    /// Non-blocking receive: returns `Some(byte)` if data is available,
    /// `None` otherwise.
    ///
    /// Checks and clears the overrun error flag (ORE) on each call.  Overrun
    /// events are counted in `overrun_count()`.
    pub fn read_byte(&mut self) -> Option<u8> {
        if !self.initialised {
            return None;
        }
        // SAFETY: USART2_BASE + offsets are valid MMIO addresses.
        unsafe {
            let isr = reg_read(USART2_BASE, USART_ISR_OFFSET);

            // Clear overrun error if set (ORE prevents further reception until
            // cleared on some G4 silicon revisions).
            if (isr & ISR_ORE) != 0 {
                reg_write(USART2_BASE, USART_ICR_OFFSET, ICR_ORECF);
                self.overrun_count = self.overrun_count.saturating_add(1);
            }

            if (isr & ISR_RXNE) != 0 {
                // Read RDR — this also clears RXNE.
                let byte = reg_read(USART2_BASE, USART_RDR_OFFSET) as u8;
                Some(byte)
            } else {
                None
            }
        }
    }

    /// Blocking receive: waits indefinitely until a byte is available.
    pub fn read_byte_blocking(&mut self) -> u8 {
        loop {
            if let Some(b) = self.read_byte() {
                return b;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    /// Returns the number of RX overrun errors detected since `init()`.
    #[inline]
    pub const fn overrun_count(&self) -> u32 {
        self.overrun_count
    }

    /// Returns `true` if the UART has been successfully initialised.
    #[inline]
    pub const fn is_initialised(&self) -> bool {
        self.initialised
    }
}

impl Default for PolledUart {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// core::fmt::Write impl — enables `write!` / `writeln!` macros
// ---------------------------------------------------------------------------

impl core::fmt::Write for PolledUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        PolledUart::write_str(self, s);
        Ok(())
    }
}
