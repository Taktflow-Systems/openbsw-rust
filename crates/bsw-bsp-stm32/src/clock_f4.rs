// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! F413ZH clock configuration — HSE 8 MHz → PLL → 96 MHz SYSCLK.
//!
//! Configures the system clocks for the STM32F413ZH on the NUCLEO-F413ZH
//! board (ST-LINK MCO supplies the 8 MHz HSE bypass clock):
//!
//! | Domain   | Frequency |
//! |----------|-----------|
//! | SYSCLK   | 96 MHz    |
//! | AHB      | 96 MHz    |
//! | APB1     | 48 MHz    |
//! | APB2     | 96 MHz    |
//!
//! **PLL configuration**: HSE / M=8 * N=384 / P=4 = 96 MHz.
//! **Flash**: 3 wait states, prefetch, instruction-cache, data-cache.

use stm32f4xx_hal::pac;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Configure system clocks for STM32F413ZH on NUCLEO board.
///
/// HSE 8 MHz bypass → PLL M=8, N=384, P=4 → SYSCLK 96 MHz.
/// APB1 = 48 MHz (÷2), APB2 = 96 MHz.
/// Flash: 3 wait states + prefetch + I/D cache.
///
/// Returns the achieved SYSCLK frequency in Hz (always 96_000_000).
///
/// # Safety
///
/// This function accesses RCC and FLASH peripheral registers directly via
/// raw pointer dereferences.  It must be called exactly once at startup
/// before any peripheral that depends on the clock tree.
pub fn configure_clocks_f413() -> u32 {
    // SAFETY: single-call startup function; no concurrent access possible.
    let rcc = unsafe { &*pac::RCC::ptr() };
    let flash = unsafe { &*pac::FLASH::ptr() };

    // -----------------------------------------------------------------------
    // 1. Enable HSE in bypass mode (ST-LINK MCO provides 8 MHz square wave)
    // -----------------------------------------------------------------------
    rcc.cr().modify(|_, w| w.hsebyp().bypassed().hseon().on());
    while rcc.cr().read().hserdy().is_not_ready() {}

    // -----------------------------------------------------------------------
    // 2. Configure PLL: VCO_in = HSE/8 = 1 MHz, VCO_out = 384 MHz, PLLCLK = 96 MHz
    //    PLLM = 8, PLLN = 384, PLLP = /4
    // -----------------------------------------------------------------------
    // SAFETY: raw bits for PLLM (6-bit) and PLLN (9-bit) are within legal ranges.
    rcc.pllcfgr().write(|w| unsafe {
        w.pllm()
            .bits(8)
            .plln()
            .bits(384)
            .pllp()
            .div4()
            .pllsrc()
            .hse()
    });

    // -----------------------------------------------------------------------
    // 3. Enable PLL and wait for lock
    // -----------------------------------------------------------------------
    rcc.cr().modify(|_, w| w.pllon().on());
    while rcc.cr().read().pllrdy().is_not_ready() {}

    // -----------------------------------------------------------------------
    // 4. Flash latency — must be set BEFORE switching SYSCLK source.
    //    96 MHz at VDD ≥ 2.7 V → 3 wait states (reference manual Table 6).
    //    Enable prefetch, instruction-cache and data-cache.
    // -----------------------------------------------------------------------
    // SAFETY: latency bits (4-bit field) value 3 is within range.
    flash.acr().write(|w| unsafe {
        w.latency()
            .bits(3)
            .prften()
            .enabled()
            .icen()
            .enabled()
            .dcen()
            .enabled()
    });

    // -----------------------------------------------------------------------
    // 5. Configure bus prescalers and switch SYSCLK to PLL
    //    APB1 must not exceed 50 MHz → divide AHB by 2 → 48 MHz
    //    APB2 can run at 96 MHz → no divider
    // -----------------------------------------------------------------------
    rcc.cfgr().modify(|_, w| {
        w.ppre1()
            .div2() // APB1 = 48 MHz
            .ppre2()
            .div1() // APB2 = 96 MHz
            .sw()
            .pll() // SYSCLK source = PLL
    });
    // Wait until the PLL is accepted as the clock source.
    while !rcc.cfgr().read().sws().is_pll() {}

    96_000_000
}
