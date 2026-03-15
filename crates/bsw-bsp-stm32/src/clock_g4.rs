// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! G474RE clock configuration — HSI 16 MHz → PLL → 170 MHz SYSCLK (boost mode).
//!
//! Configures the system clocks for the STM32G474RE:
//!
//! | Domain   | Frequency |
//! |----------|-----------|
//! | SYSCLK   | 170 MHz   |
//! | AHB      | 170 MHz   |
//! | APB1     | 170 MHz   |
//! | APB2     | 170 MHz   |
//!
//! **PLL configuration**: HSI 16 MHz / M=4 * N=85 / R=2 = 170 MHz.
//! **Voltage scaling**: Range 1 boost mode (PWR_CR5.R1MODE = 0).
//! **Flash**: 4 wait states, prefetch, instruction-cache, data-cache.
//!
//! # Why raw register access
//!
//! `stm32g4xx-hal` 0.1 does not expose the boost-mode voltage scaling path
//! (`PWR_CR5.R1MODE`) through its clock configuration API.  We therefore
//! access the RCC, PWR, and FLASH register blocks directly via raw pointer
//! casts to their known peripheral base addresses.
//!
//! # Reference
//!
//! RM0440 Rev 8, sections 5.2 (power modes), 6.2 (clocks), 3.3 (flash).

// ---------------------------------------------------------------------------
// Peripheral base addresses (RM0440 Rev 8, memory map)
// ---------------------------------------------------------------------------

/// RCC peripheral base address.
const RCC_BASE: usize = 0x4002_1000;

/// PWR peripheral base address.
const PWR_BASE: usize = 0x4000_7000;

/// FLASH peripheral base address.
const FLASH_BASE: usize = 0x4002_2000;

// ---------------------------------------------------------------------------
// RCC register offsets (byte offsets from RCC_BASE)
// ---------------------------------------------------------------------------

/// RCC clock control register.
const RCC_CR_OFFSET: usize = 0x00;

/// RCC PLL configuration register.
const RCC_PLLCFGR_OFFSET: usize = 0x0C;

/// RCC clock configuration register.
const RCC_CFGR_OFFSET: usize = 0x08;

/// RCC APB1 peripheral clock enable register 1.
const RCC_APB1ENR1_OFFSET: usize = 0x58;

// ---------------------------------------------------------------------------
// PWR register offsets
// ---------------------------------------------------------------------------

/// PWR control register 1.
#[allow(dead_code)]
const PWR_CR1_OFFSET: usize = 0x00;

/// PWR control register 5 — contains R1MODE bit for boost.
const PWR_CR5_OFFSET: usize = 0x80;

/// PWR status register 2 — contains VOSF flag.
const PWR_SR2_OFFSET: usize = 0x14;

// ---------------------------------------------------------------------------
// FLASH register offsets
// ---------------------------------------------------------------------------

/// FLASH access control register.
const FLASH_ACR_OFFSET: usize = 0x00;

// ---------------------------------------------------------------------------
// Bit masks
// ---------------------------------------------------------------------------

// RCC_APB1ENR1
const PWREN: u32 = 1 << 28;

// PWR_CR5
/// Bit 8 — R1MODE: 0 = Range 1 boost, 1 = Range 1 normal.
const PWR_CR5_R1MODE: u32 = 1 << 8;

// PWR_SR2
/// Bit 10 — VOSF: voltage scaling flag (1 = transitioning).
const PWR_SR2_VOSF: u32 = 1 << 10;

// RCC_CR
const HSION: u32 = 1 << 0;
const HSIRDY: u32 = 1 << 10;
const PLLON: u32 = 1 << 24;
const PLLRDY: u32 = 1 << 25;

// RCC_PLLCFGR
/// PLLSRC = HSI16 (binary 10).
const PLLSRC_HSI: u32 = 0b10;
/// PLLM = 4 → register value M−1 = 3, at bits [7:4].
const PLLM_4: u32 = (4 - 1) << 4;
/// PLLN = 85, at bits [14:8].
const PLLN_85: u32 = 85 << 8;
/// PLLR = 2 → register encoding 0b00 at bits [26:25] (00=/2, 01=/4, 10=/6, 11=/8).
const PLLR_2: u32 = 0b00 << 25;
/// PLLREN — enable the R output (SYSCLK source), bit 24.
const PLLREN: u32 = 1 << 24;

// RCC_CFGR
/// SW bits [1:0] — system clock switch.
const SW_MASK: u32 = 0b11;
/// SW = PLL (binary 11).
const SW_PLL: u32 = 0b11;
/// SWS bits [3:2] — system clock switch status.
const SWS_MASK: u32 = 0b11 << 2;
/// SWS = PLL accepted.
const SWS_PLL: u32 = 0b11 << 2;
/// HPRE bits [7:4] — AHB prescaler. 0000 = ÷1, 1000 = ÷2.
const HPRE_MASK: u32 = 0xF << 4;
/// HPRE = ÷2 (0b1000).
const HPRE_DIV2: u32 = 0b1000 << 4;
/// HPRE = ÷1 (0b0000).
const HPRE_DIV1: u32 = 0b0000 << 4;

// FLASH_ACR
/// Latency bits [3:0] — flash wait states.
const FLASH_ACR_LATENCY_MASK: u32 = 0xF;
/// 4 wait states for 170 MHz / Range 1 boost (RM0440 Table 9).
const FLASH_ACR_LATENCY_4WS: u32 = 4;
/// Prefetch enable, bit 8.
const FLASH_ACR_PRFTEN: u32 = 1 << 8;
/// Instruction cache enable, bit 9.
const FLASH_ACR_ICEN: u32 = 1 << 9;
/// Data cache enable, bit 10.
const FLASH_ACR_DCEN: u32 = 1 << 10;
/// DBG_SWEN — debug software enable, bit 18.  Must be preserved.
const FLASH_ACR_DBG_SWEN: u32 = 1 << 18;

// ---------------------------------------------------------------------------
// Helper: volatile register read/write
// ---------------------------------------------------------------------------

/// Read a 32-bit peripheral register at `base + offset`.
///
/// # Safety
///
/// Caller must ensure `base + offset` is a valid, word-aligned MMIO address.
#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

/// Write a 32-bit peripheral register at `base + offset`.
///
/// # Safety
///
/// Caller must ensure `base + offset` is a valid, word-aligned MMIO address.
#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

/// Read-modify-write: clear `mask` bits then OR in `bits`.
///
/// # Safety
///
/// Caller must ensure `base + offset` is a valid, word-aligned MMIO address.
#[inline(always)]
unsafe fn reg_modify(base: usize, offset: usize, mask: u32, bits: u32) {
    // SAFETY: caller guarantees valid MMIO address.
    let val = unsafe { reg_read(base, offset) };
    unsafe { reg_write(base, offset, (val & !mask) | bits) };
}

// ---------------------------------------------------------------------------
// Cortex-M NOP loop for short spin delays
// ---------------------------------------------------------------------------

/// Spin for approximately `n` CPU cycles by issuing NOP instructions.
///
/// Used for the mandatory short delays required by the boost-mode transition
/// sequence (RM0440 §5.2.3).  The delay is a pessimistic upper bound because
/// the compiler may optimise or pipeline NOPs; for safety-critical clock code
/// an accurate cycle counter would be preferred, but these delays are
/// specified only as "several cycles" in the reference manual.
#[inline(never)]
fn spin_cycles(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Configure system clocks for STM32G474RE — HSI 16 MHz → PLL → 170 MHz.
///
/// Executes the boost-mode voltage-scaling and PLL bringup sequence from
/// RM0440 Rev 8 §5.2.3 and §6.2.7:
///
/// 1. Enable PWR clock via RCC_APB1ENR1.PWREN.
/// 2. Select voltage Range 1 boost (`PWR_CR5.R1MODE = 0`), wait VOSF clear.
/// 3. Ensure HSI is on and stable.
/// 4. Configure PLL: HSI/4 × 85 / 2 = 170 MHz on the R output.
/// 5. Enable PLL, wait PLLRDY.
/// 6. Set FLASH to 4 wait states + prefetch + caches (preserve DBG_SWEN).
/// 7. Boost-mode AHB switch: set AHB÷2, switch SYSCLK to PLL, wait SWS,
///    spin 100 cycles, restore AHB÷1.
///
/// Returns the achieved SYSCLK frequency in Hz (always `170_000_000`).
///
/// # Safety
///
/// This function writes to RCC, PWR, and FLASH peripheral registers via raw
/// pointer dereferences.  It must be called exactly once at startup before any
/// peripheral that depends on the clock tree.
pub fn configure_clocks_g474() -> u32 {
    // -----------------------------------------------------------------------
    // 1. Enable PWR clock and insert a read-back delay
    // -----------------------------------------------------------------------
    // SAFETY: RCC_APB1ENR1 address is valid on every STM32G4 device.
    unsafe {
        let enr = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET);
        reg_write(RCC_BASE, RCC_APB1ENR1_OFFSET, enr | PWREN);
        // Read-back to flush the write through the AHB bus matrix.
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET);
    }

    // -----------------------------------------------------------------------
    // 2. Voltage scaling: Range 1 boost mode (R1MODE = 0)
    //
    //    After POR the device is in Range 1 normal (R1MODE = 1).  Clear the
    //    bit to request boost mode, then wait for VOSF to clear (indicates
    //    the regulator has settled).
    // -----------------------------------------------------------------------
    // SAFETY: PWR_CR5 address is valid on STM32G4 devices with boost support.
    unsafe {
        reg_modify(PWR_BASE, PWR_CR5_OFFSET, PWR_CR5_R1MODE, 0);
        // Wait for the voltage scaling flag to clear.
        while (reg_read(PWR_BASE, PWR_SR2_OFFSET) & PWR_SR2_VOSF) != 0 {}
    }

    // -----------------------------------------------------------------------
    // 3. Ensure HSI16 oscillator is on and stable
    //
    //    HSI is on by default after reset but may have been disabled; enable
    //    it unconditionally and wait for HSIRDY.
    // -----------------------------------------------------------------------
    // SAFETY: RCC_CR address is valid on all STM32G4 devices.
    unsafe {
        reg_modify(RCC_BASE, RCC_CR_OFFSET, 0, HSION);
        while (reg_read(RCC_BASE, RCC_CR_OFFSET) & HSIRDY) == 0 {}
    }

    // -----------------------------------------------------------------------
    // 4. Configure PLL
    //
    //    VCO_in  = HSI16 / M   = 16 MHz / 4 = 4 MHz   (must be 2.66–16 MHz)
    //    VCO_out = VCO_in * N  = 4 MHz * 85 = 340 MHz  (must be 96–344 MHz)
    //    PLLRCLK = VCO_out / R = 340 MHz / 2 = 170 MHz
    //
    //    The PLL must be off before writing PLLCFGR (RM0440 §6.2.7).
    // -----------------------------------------------------------------------
    // SAFETY: RCC_CR and RCC_PLLCFGR addresses are valid on all STM32G4 devices.
    unsafe {
        // Disable PLL before reconfiguring.
        reg_modify(RCC_BASE, RCC_CR_OFFSET, PLLON, 0);
        while (reg_read(RCC_BASE, RCC_CR_OFFSET) & PLLRDY) != 0 {}

        // Write PLLCFGR: PLLSRC=HSI, M=4, N=85, R=2, PLLREN=1.
        reg_write(
            RCC_BASE,
            RCC_PLLCFGR_OFFSET,
            PLLSRC_HSI | PLLM_4 | PLLN_85 | PLLR_2 | PLLREN,
        );
    }

    // -----------------------------------------------------------------------
    // 5. Enable PLL and wait for lock
    // -----------------------------------------------------------------------
    // SAFETY: RCC_CR address is valid on all STM32G4 devices.
    unsafe {
        reg_modify(RCC_BASE, RCC_CR_OFFSET, 0, PLLON);
        while (reg_read(RCC_BASE, RCC_CR_OFFSET) & PLLRDY) == 0 {}
    }

    // -----------------------------------------------------------------------
    // 6. Flash: 4 wait states + prefetch + caches
    //
    //    Must be configured BEFORE switching SYSCLK source.
    //    DBG_SWEN (bit 18) must be preserved to keep the debug port functional
    //    during bringup; read the current value and OR in the new settings.
    // -----------------------------------------------------------------------
    // SAFETY: FLASH_ACR address is valid on all STM32G4 devices.
    unsafe {
        let acr = reg_read(FLASH_BASE, FLASH_ACR_OFFSET);
        let dbg_swen = acr & FLASH_ACR_DBG_SWEN;
        reg_write(
            FLASH_BASE,
            FLASH_ACR_OFFSET,
            FLASH_ACR_LATENCY_4WS
                | FLASH_ACR_PRFTEN
                | FLASH_ACR_ICEN
                | FLASH_ACR_DCEN
                | dbg_swen,
        );
        // Read-back to confirm the latency write was accepted.
        while (reg_read(FLASH_BASE, FLASH_ACR_OFFSET) & FLASH_ACR_LATENCY_MASK)
            != FLASH_ACR_LATENCY_4WS
        {}
    }

    // -----------------------------------------------------------------------
    // 7. Boost-mode AHB transition sequence (RM0440 §5.2.3 step 4)
    //
    //    When entering boost mode the sequence must be:
    //      a. Set AHB prescaler to ÷2 (HPRE = 1000).
    //      b. Switch SYSCLK source to PLL (SW = 11).
    //      c. Wait for SWS = PLL (switch accepted).
    //      d. Spin ≥ 1 μs (≈ 100 cycles @ 85 MHz AHB) to let bus matrix settle.
    //      e. Restore AHB prescaler to ÷1 (HPRE = 0000).
    // -----------------------------------------------------------------------
    // SAFETY: RCC_CFGR address is valid on all STM32G4 devices.
    unsafe {
        // a. AHB ÷2
        reg_modify(RCC_BASE, RCC_CFGR_OFFSET, HPRE_MASK, HPRE_DIV2);

        // b. Switch SYSCLK to PLL
        reg_modify(RCC_BASE, RCC_CFGR_OFFSET, SW_MASK, SW_PLL);

        // c. Wait until PLL is the active SYSCLK source
        while (reg_read(RCC_BASE, RCC_CFGR_OFFSET) & SWS_MASK) != SWS_PLL {}

        // d. Short settling delay — at AHB = 85 MHz, 100 NOPs ≈ 1.2 μs
        spin_cycles(100);

        // e. Restore AHB ÷1
        reg_modify(RCC_BASE, RCC_CFGR_OFFSET, HPRE_MASK, HPRE_DIV1);
    }

    // PWR_CR1 VOS bits default to Range 1 after reset; no change needed.
    // APB1 and APB2 prescalers default to ÷1; both busses run at 170 MHz.
    // No further action required.

    170_000_000
}

// ---------------------------------------------------------------------------
// Expose for optional read-back from application code
// ---------------------------------------------------------------------------

/// Returns the configured SYSCLK frequency in Hz.
///
/// This is a pure compile-time constant; call `configure_clocks_g474()` at
/// startup to actually achieve this frequency.
pub const SYSCLK_HZ: u32 = 170_000_000;
