// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Polled UART for STM32F413ZH — uses SR/DR register interface.
//!
//! Provides a blocking, interrupt-free UART driver suitable for early
//! bring-up logging and fault handlers where IRQs may be disabled.
//!
//! # Hardware assignment (NUCLEO-F413ZH, 144-pin)
//!
//! The NUCLEO-F413ZH routes the ST-LINK VCP to **USART3** (not USART2).
//!
//! | Resource  | Assignment                                     |
//! |-----------|------------------------------------------------|
//! | Peripheral| USART3                                         |
//! | TX pin    | PD8 (AF7)                                      |
//! | RX pin    | PD9 (AF7)                                      |
//! | APB1 clk  | 48 MHz (configured by `clock_f4`)              |
//! | Baudrate  | 115 200 baud                                   |
//!
//! # BRR calculation
//!
//! ```text
//! BRR = F_APB1 / baud = 48_000_000 / 115_200 = 416.67 → 417 (0x01A1)
//! ```

use stm32f4xx_hal::pac;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// BRR value for 115 200 baud with APB1 = 48 MHz, OVER8 = 0 (×16).
const BRR_115200_APB1_48MHZ: u16 = 0x01A1;

/// Spin-loop iteration limit for polled TX/RX operations.
const WRITE_TIMEOUT: u32 = 100_000;

// ---------------------------------------------------------------------------
// PolledUart
// ---------------------------------------------------------------------------

/// Blocking, polled USART3 driver for NUCLEO-F413ZH (VCP via PD8/PD9).
pub struct PolledUart {
    usart: *const pac::usart1::RegisterBlock,
}

// SAFETY: single-threaded bare-metal use.
unsafe impl Send for PolledUart {}

impl PolledUart {
    /// Create a new `PolledUart` pointing at USART3.
    pub const fn new() -> Self {
        Self {
            usart: pac::USART3::ptr(),
        }
    }

    /// Initialise USART3 for 115 200 baud, 8N1, TX+RX enabled.
    ///
    /// Configures PD8 (TX) / PD9 (RX) as AF7 and programs the USART.
    ///
    /// # Safety
    ///
    /// Must be called once at startup after clock configuration.
    pub unsafe fn init(&mut self) {
        let rcc = unsafe { &*pac::RCC::ptr() };
        let gpiod = unsafe { &*pac::GPIOD::ptr() };
        let usart = unsafe { &*self.usart };

        // 1. Enable clocks: GPIOD (AHB1) and USART3 (APB1)
        rcc.ahb1enr().modify(|_, w| w.gpioden().enabled());
        rcc.apb1enr().modify(|_, w| w.usart3en().enabled());
        let _ = rcc.apb1enr().read();

        // 2. Configure PD8 (TX) and PD9 (RX) as AF7
        // MODER: PD8 bits[17:16] = 0b10 (AF), PD9 bits[19:18] = 0b10 (AF)
        gpiod.moder().modify(|r, w| unsafe {
            let val = r.bits();
            let val = (val & !(0b11 << 16)) | (0b10 << 16); // PD8
            let val = (val & !(0b11 << 18)) | (0b10 << 18); // PD9
            w.bits(val)
        });

        // OTYPER: push-pull for PD8/PD9
        gpiod
            .otyper()
            .modify(|r, w| unsafe { w.bits(r.bits() & !((1 << 8) | (1 << 9))) });

        // OSPEEDR: high speed for PD8/PD9
        gpiod.ospeedr().modify(|r, w| unsafe {
            let val = r.bits();
            let val = (val & !(0b11 << 16)) | (0b10 << 16);
            let val = (val & !(0b11 << 18)) | (0b10 << 18);
            w.bits(val)
        });

        // PUPDR: no pull
        gpiod
            .pupdr()
            .modify(|r, w| unsafe { w.bits(r.bits() & !((0b11 << 16) | (0b11 << 18))) });

        // AFRH: AF7 for PD8 [bits 3:0] and PD9 [bits 7:4]
        // PD8 is pin 8, PD9 is pin 9 → both in AFRH (pins 8-15)
        gpiod.afrh().modify(|r, w| unsafe {
            let val = r.bits();
            let val = (val & !(0xF << 0)) | (7 << 0);   // PD8 → AF7
            let val = (val & !(0xF << 4)) | (7 << 4);    // PD9 → AF7
            w.bits(val)
        });

        // 3. Configure USART3
        usart.cr1().write(|w| unsafe { w.bits(0) });
        usart
            .brr()
            .write(|w| unsafe { w.bits(BRR_115200_APB1_48MHZ) });
        usart.cr2().write(|w| unsafe { w.bits(0) });
        usart.cr3().write(|w| unsafe { w.bits(0) });
        usart.cr1().write(|w| w.ue().enabled().te().enabled().re().enabled());
    }

    /// Transmit a single byte, blocking until TXE or timeout.
    pub fn write_byte(&mut self, byte: u8) -> bool {
        let usart = unsafe { &*self.usart };
        let mut timeout = WRITE_TIMEOUT;
        while usart.sr().read().txe().bit_is_clear() {
            timeout = match timeout.checked_sub(1) {
                Some(t) => t,
                None => return false,
            };
        }
        usart.dr().write(|w| w.dr().set(u16::from(byte)));
        true
    }

    /// Transmit a slice of bytes.
    pub fn write_bytes(&mut self, data: &[u8]) {
        for &byte in data {
            if !self.write_byte(byte) {
                break;
            }
        }
    }

    /// Receive a single byte, blocking until RXNE or timeout.
    pub fn read_byte(&mut self) -> Option<u8> {
        let usart = unsafe { &*self.usart };
        let mut timeout = WRITE_TIMEOUT;
        while usart.sr().read().rxne().bit_is_clear() {
            timeout = match timeout.checked_sub(1) {
                Some(t) => t,
                None => return None,
            };
        }
        #[allow(clippy::cast_possible_truncation)]
        Some(usart.dr().read().dr().bits() as u8)
    }
}

impl core::fmt::Write for PolledUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes());
        Ok(())
    }
}
