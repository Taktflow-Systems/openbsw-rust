//! UDS tester on NUCLEO-F413ZH — exercises `can_server_f413` over a real CAN bus.
//!
//! This example sends four UDS requests to the server board and verifies the
//! responses.  Both boards must be connected via a TJA105x transceiver with
//! 120 Ω termination at each end of the bus.
//!
//! # Test sequence
//!
//! | # | Service              | Request             | Expected response        |
//! |---|----------------------|---------------------|--------------------------|
//! | 1 | TesterPresent        | `3E 00`             | SF `[02, 7E, 00]`        |
//! | 2 | DiagSessionCtrl ext  | `10 03`             | SF `[07, 50, 03, ...]`   |
//! | 3 | ReadDID unknown      | `22 00 01`          | SF `[04, 7F, 22, 31]`    |
//! | 4 | Unknown service      | `AA`                | SF `[04, 7F, AA, 11]`    |
//!
//! All four tests use Single Frame ISO-TP — no multi-frame handling required
//! in the tester.  This proves real-bus bxCAN communication end-to-end.
//!
//! # GPIO / wiring (NUCLEO-F413ZH)
//!
//! ```text
//! PA11  →  TJA RXD   (CAN1_RX, AF9)
//! PA12  →  TJA TXD   (CAN1_TX, AF9)
//! GND / VCC as per transceiver module datasheet.
//! CAN_H / CAN_L connected to server board transceiver.
//! 120 Ω termination at each end.
//! ```
//!
//! # CAN IDs
//!
//! | Direction        | CAN ID |
//! |------------------|--------|
//! | Tester → Server  | 0x600  |
//! | Server → Tester  | 0x601  |
//!
//! # Bit timing: 500 kbit/s @ 48 MHz APB1
//!
//! ```text
//! Prescaler (BRP+1) = 6  → BRP = 5
//! Tseg1 (TS1+1)     = 11 → TS1 = 10
//! Tseg2 (TS2+1)     = 4  → TS2 = 3
//! SJW   (SJW+1)     = 1  → SJW = 0
//! Bit time = 16 Tq → 1/(16 × 125 ns) = 500 000 bit/s ✓
//! NOTE: LBKM (bit 30) is NOT set — real bus mode.
//! ```
//!
//! # LED indication
//!
//! * Fast blink (100 ms half-period) — all 4 tests passed.
//! * Slow blink (500 ms half-period) — one or more tests failed.
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32f413 --target thumbv7em-none-eabihf \
//!             --example can_tester_f413
//! ```
//!
//! # Reference
//!
//! RM0402 Rev 8, §32 (bxCAN); ISO 15765-2:2016 (ISO-TP); ISO 14229-1:2020 (UDS).

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::Peripherals as CorePeripherals;

use bsw_bsp_stm32::clock_f4::configure_clocks_f413;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_f4::PolledUart;

// ---------------------------------------------------------------------------
// Register-map constants — STM32F413ZH
// ---------------------------------------------------------------------------

/// RCC base address (RM0402 §6.3).
const RCC_BASE: usize = 0x4002_3800;
/// RCC AHB1ENR — GPIO clock enables.
const RCC_AHB1ENR_OFFSET: usize = 0x30;
/// RCC APB1ENR — CAN1 clock enable (bit 25).
const RCC_APB1ENR_OFFSET: usize = 0x40;

/// CAN1 clock enable in APB1ENR (bit 25).
const CAN1EN: u32 = 1 << 25;
/// GPIOA clock enable (AHB1ENR bit 0).
const GPIOAEN: u32 = 1 << 0;

/// GPIOA base address.
const GPIOA_BASE: usize = 0x4002_0000;

/// GPIO register offsets (same layout across STM32F4 family).
const GPIO_MODER_OFFSET:   usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_PUPDR_OFFSET:   usize = 0x0C;
const GPIO_AFRL_OFFSET:    usize = 0x20;
const GPIO_AFRH_OFFSET:    usize = 0x24;
const GPIO_BSRR_OFFSET:    usize = 0x18;

// ---------------------------------------------------------------------------
// bxCAN1 register-block base address and offsets (RM0402 §32.9)
// ---------------------------------------------------------------------------

/// bxCAN1 base address.
const CAN1_BASE: usize = 0x4000_6400;

const CAN_MCR_OFFSET:  usize = 0x000;
const CAN_MSR_OFFSET:  usize = 0x004;
const CAN_TSR_OFFSET:  usize = 0x008;
const CAN_RF0R_OFFSET: usize = 0x00C;
const CAN_IER_OFFSET:  usize = 0x014;
const CAN_BTR_OFFSET:  usize = 0x01C;

// TX mailbox 0.
const CAN_TI0R_OFFSET:  usize = 0x180;
const CAN_TDT0R_OFFSET: usize = 0x184;
const CAN_TDL0R_OFFSET: usize = 0x188;
const CAN_TDH0R_OFFSET: usize = 0x18C;

// RX FIFO 0.
const CAN_RI0R_OFFSET:  usize = 0x1B0;
const CAN_RDT0R_OFFSET: usize = 0x1B4;
const CAN_RDL0R_OFFSET: usize = 0x1B8;
const CAN_RDH0R_OFFSET: usize = 0x1BC;

// Filter bank 0.
const CAN_FMR_OFFSET:   usize = 0x200;
const CAN_FS1R_OFFSET:  usize = 0x20C;
const CAN_FFA1R_OFFSET: usize = 0x214;
const CAN_FA1R_OFFSET:  usize = 0x21C;
const CAN_F0R1_OFFSET:  usize = 0x240;
const CAN_F0R2_OFFSET:  usize = 0x244;

// ---------------------------------------------------------------------------
// bxCAN bit masks
// ---------------------------------------------------------------------------

const MCR_INRQ:  u32 = 1 << 0;
const MCR_SLEEP: u32 = 1 << 1;
const MCR_TXFP:  u32 = 1 << 2;
const MCR_ABOM:  u32 = 1 << 6;

const MSR_INAK: u32 = 1 << 0;

const TSR_TME0: u32 = 1 << 26;
const TSR_TME1: u32 = 1 << 27;
const TSR_TME2: u32 = 1 << 28;

const RF0R_FMP0_MASK: u32 = 0b11;
const RF0R_RFOM0:     u32 = 1 << 5;

const FMR_FINIT: u32 = 1 << 0;

// ---------------------------------------------------------------------------
// bxCAN BTR — 500 kbit/s @ 48 MHz APB1, real bus (NO loopback)
//
// BRP=5 (prescaler=6), TS1=10 (Tseg1=11 Tq), TS2=3 (Tseg2=4 Tq), SJW=0.
// Tq=125 ns; bit=16 Tq → 500 000 bit/s.
// NOTE: bit 30 (LBKM) is NOT set.
// ---------------------------------------------------------------------------

const BTR_500KBPS_REAL: u32 =
    (0 << 24)   // SJW  = 0
    | (3 << 20) // TS2  = 3
    | (10 << 16)// TS1  = 10
    | 5;        // BRP  = 5

// ---------------------------------------------------------------------------
// LED pin, CAN IDs, test count
// ---------------------------------------------------------------------------

const LED_PIN:    u32 = 5;
const REQUEST_ID: u32 = 0x600;
const RESPONSE_ID: u32 = 0x601;
const NUM_TESTS:  usize = 4;

// ---------------------------------------------------------------------------
// Raw MMIO helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_modify(base: usize, offset: usize, mask: u32, bits: u32) {
    let v = unsafe { reg_read(base, offset) };
    unsafe { reg_write(base, offset, (v & !mask) | bits) };
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

unsafe fn gpio_output(base: usize, pin: u32) {
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b01 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b10 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
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
// bxCAN1 initialisation (real bus — PD0/PD1 AF9, no loopback)
// ---------------------------------------------------------------------------

/// GPIOD base address (F413: 0x4002_0C00).
const GPIOD_BASE: usize = 0x4002_0C00;
/// GPIOD clock enable (AHB1ENR bit 3).
const GPIODEN: u32 = 1 << 3;

/// Enable CAN1 + GPIOD clocks; configure PD0 (RX) / PD1 (TX) as AF9.
///
/// PD0 = CAN1_RX: AF mode, pull-up.
/// PD1 = CAN1_TX: AF mode, very-high speed push-pull output.
/// (Production wiring per rzc_f4_hw_stm32f4.c)
unsafe fn bxcan1_gpio_clock_init() {
    unsafe {
        // Enable GPIOA (LED) and GPIOD (CAN pins) clocks.
        reg_modify(RCC_BASE, RCC_AHB1ENR_OFFSET, 0, GPIOAEN | GPIODEN);
        let _ = reg_read(RCC_BASE, RCC_AHB1ENR_OFFSET);

        // PD0 = CAN1_RX (AF9), pull-up.
        gpio_af(GPIOD_BASE, 0, 9);
        reg_modify(GPIOD_BASE, GPIO_PUPDR_OFFSET,
                   0b11 << (0 * 2), 0b01 << (0 * 2));

        // PD1 = CAN1_TX (AF9), very-high speed.
        gpio_af(GPIOD_BASE, 1, 9);
        reg_modify(GPIOD_BASE, GPIO_OSPEEDR_OFFSET,
                   0b11 << (1 * 2), 0b11 << (1 * 2));

        // Enable CAN1 clock on APB1.
        reg_modify(RCC_BASE, RCC_APB1ENR_OFFSET, 0, CAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR_OFFSET);
    }
}

unsafe fn bxcan1_enter_init() {
    unsafe {
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, MCR_SLEEP, MCR_INRQ);
        while (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) == 0 {}
    }
}

unsafe fn bxcan1_leave_init() {
    unsafe {
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, MCR_INRQ, 0);
        while (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) != 0 {}
    }
}

unsafe fn bxcan1_configure_filters() {
    unsafe {
        reg_modify(CAN1_BASE, CAN_FMR_OFFSET,   0, FMR_FINIT);
        reg_modify(CAN1_BASE, CAN_FS1R_OFFSET,  0, 1 << 0);
        reg_modify(CAN1_BASE, CAN_FFA1R_OFFSET, 1 << 0, 0);
        reg_write(CAN1_BASE, CAN_F0R1_OFFSET, 0x0000_0000); // ID = 0
        reg_write(CAN1_BASE, CAN_F0R2_OFFSET, 0x0000_0000); // mask = 0 (accept all)
        reg_modify(CAN1_BASE, CAN_FA1R_OFFSET,  0, 1 << 0);
        reg_modify(CAN1_BASE, CAN_FMR_OFFSET,  FMR_FINIT, 0);
    }
}

/// Full bxCAN1 init for real CAN bus (no loopback).  Returns `true` on success.
unsafe fn bxcan1_init() -> bool {
    unsafe {
        bxcan1_gpio_clock_init();
        bxcan1_enter_init();
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, 0, MCR_ABOM | MCR_TXFP);
        // NOTE: BTR_500KBPS_REAL does NOT set LBKM (bit 30).
        reg_write(CAN1_BASE, CAN_BTR_OFFSET, BTR_500KBPS_REAL);
        reg_write(CAN1_BASE, CAN_IER_OFFSET, 0x0000_0000); // polled mode
        bxcan1_configure_filters();
        bxcan1_leave_init();
        (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) == 0
    }
}

// ---------------------------------------------------------------------------
// bxCAN TX / RX helpers
// ---------------------------------------------------------------------------

/// Transmit a standard-ID (11-bit) classic CAN frame via TX mailbox 0.
unsafe fn bxcan1_send(id: u32, data: &[u8]) {
    unsafe {
        // Wait for an empty TX mailbox.
        loop {
            let tsr = reg_read(CAN1_BASE, CAN_TSR_OFFSET);
            if (tsr & (TSR_TME0 | TSR_TME1 | TSR_TME2)) != 0 {
                break;
            }
        }

        let dlc = data.len().min(8) as u32;
        let mut raw = [0u8; 8];
        let n = data.len().min(8);
        raw[..n].copy_from_slice(&data[..n]);
        let tdlr = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let tdhr = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);

        reg_write(CAN1_BASE, CAN_TI0R_OFFSET,  (id & 0x7FF) << 21);
        reg_write(CAN1_BASE, CAN_TDT0R_OFFSET, dlc & 0xF);
        reg_write(CAN1_BASE, CAN_TDL0R_OFFSET, tdlr);
        reg_write(CAN1_BASE, CAN_TDH0R_OFFSET, tdhr);
        reg_modify(CAN1_BASE, CAN_TI0R_OFFSET, 0, 1 << 0); // TXRQ
    }
}

/// One received CAN frame.
struct RxFrame {
    id:   u32,
    dlc:  u8,
    data: [u8; 8],
}

/// Read one frame from RX FIFO 0 and release it.  Returns `None` if empty.
unsafe fn bxcan1_rx_fifo0_read() -> Option<RxFrame> {
    unsafe {
        let rf0r = reg_read(CAN1_BASE, CAN_RF0R_OFFSET);
        if (rf0r & RF0R_FMP0_MASK) == 0 {
            return None;
        }

        let ri0r  = reg_read(CAN1_BASE, CAN_RI0R_OFFSET);
        let rdt0r = reg_read(CAN1_BASE, CAN_RDT0R_OFFSET);
        let rdl0r = reg_read(CAN1_BASE, CAN_RDL0R_OFFSET);
        let rdh0r = reg_read(CAN1_BASE, CAN_RDH0R_OFFSET);

        let is_ext = (ri0r & (1 << 2)) != 0;
        let id = if is_ext {
            (ri0r >> 3) & 0x1FFF_FFFF
        } else {
            (ri0r >> 21) & 0x7FF
        };
        let dlc = (rdt0r & 0xF) as u8;

        let lo = rdl0r.to_le_bytes();
        let hi = rdh0r.to_le_bytes();
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&lo);
        data[4..].copy_from_slice(&hi);

        reg_modify(CAN1_BASE, CAN_RF0R_OFFSET, 0, RF0R_RFOM0);
        Some(RxFrame { id, dlc, data })
    }
}

// ---------------------------------------------------------------------------
// ISO-TP Single Frame helpers
// ---------------------------------------------------------------------------

/// Encode a UDS request as an ISO-TP Single Frame payload.
///
/// SF PCI: `byte[0] = (0x0 << 4) | payload_len`, then payload bytes.
fn isotp_encode_sf(uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    let payload_len = uds_data.len().min(7);
    out[0] = payload_len as u8;
    out[1..1 + payload_len].copy_from_slice(&uds_data[..payload_len]);
    1 + payload_len
}

/// Decode a received CAN frame as an ISO-TP Single Frame.
///
/// Returns `Some(payload_len)` if valid SF, `None` otherwise.
fn isotp_decode_sf(frame_data: &[u8]) -> Option<usize> {
    let pci = *frame_data.first()?;
    if (pci >> 4) != 0 {
        return None; // not a SF
    }
    let payload_len = (pci & 0x0F) as usize;
    if payload_len == 0 || payload_len > 7 {
        return None;
    }
    Some(payload_len)
}

// ---------------------------------------------------------------------------
// Test helper: send SF request and wait for SF response.
// ---------------------------------------------------------------------------

/// Send a Single Frame UDS request on 0x600 and wait for a Single Frame
/// response on 0x601.
///
/// Returns `Some((data, dlc))` — the raw CAN frame — on success, or `None`
/// on timeout (1 s).
unsafe fn send_recv_sf(
    req:   &[u8],
    timer: &mut DwtTimer,
) -> Option<([u8; 8], u8)> {
    let mut tx_buf = [0u8; 8];
    let tx_len = isotp_encode_sf(req, &mut tx_buf);
    unsafe { bxcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

    let deadline = timer.system_time_us_64() + 1_000_000; // 1 s timeout
    loop {
        if let Some(f) = unsafe { bxcan1_rx_fifo0_read() } {
            if f.id == RESPONSE_ID {
                return Some((f.data, f.dlc));
            }
            // Discard frames on other IDs (bus noise, own TX echo, etc.)
        }
        if timer.system_time_us_64() >= deadline {
            return None;
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // ------------------------------------------------------------------
    // 1. Clocks — 96 MHz via HSE 8 MHz bypass + PLL
    // ------------------------------------------------------------------
    let sys_clock = configure_clocks_f413();

    // ------------------------------------------------------------------
    // 2. DWT timer
    // ------------------------------------------------------------------
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // ------------------------------------------------------------------
    // 3. UART — USART3 115200 8N1 on PD8 (TX) / PD9 (RX), AF7
    // ------------------------------------------------------------------
    let mut uart = PolledUart::new();
    unsafe { uart.init() };
    let _ = writeln!(uart,
        "\r\n=== CAN tester (real bus) F413ZH starting @ {} MHz ===",
        sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    //
    //    Enable GPIOA clock explicitly (UART init enables GPIOD for PD8/PD9;
    //    CAN init will also enable it, but do it here for the LED first).
    // ------------------------------------------------------------------
    unsafe {
        reg_modify(RCC_BASE, RCC_AHB1ENR_OFFSET, 0, GPIOAEN);
        let _ = reg_read(RCC_BASE, RCC_AHB1ENR_OFFSET);
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }

    // ------------------------------------------------------------------
    // 5. bxCAN1 — PA11/PA12 AF9, 500 kbps, real bus (no loopback)
    // ------------------------------------------------------------------
    let _ = writeln!(uart,
        "Initialising bxCAN1 (real bus, PD0=RX PD1=TX, 500 kbps)...");
    let init_ok = unsafe { bxcan1_init() };
    if !init_ok {
        let _ = writeln!(uart, "FAIL: bxCAN1 did not leave init mode");
        blink_forever(&mut timer, 200_000);
    }
    let _ = writeln!(uart,
        "bxCAN1 ready.  TX=0x{:03X}  Listen=0x{:03X}",
        REQUEST_ID, RESPONSE_ID);

    // ------------------------------------------------------------------
    // 6. Wait 1 second for server to finish booting.
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Waiting 1 s for server to boot...");
    // F4 PolledUart is blocking — no flush needed
    let boot_wait = timer.system_time_us_64() + 1_000_000;
    while timer.system_time_us_64() < boot_wait {}

    // ------------------------------------------------------------------
    // 7. Test sequence
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    // 6b. CAN bus diagnostic — send one frame and check TX status
    // ------------------------------------------------------------------
    {
        let _ = writeln!(uart, "CAN bus diagnostic...");
        // Send a dummy frame and check what happens
        let mut tx_buf = [0u8; 8];
        tx_buf[0] = 0x02; tx_buf[1] = 0x3E; tx_buf[2] = 0x00;
        unsafe { bxcan1_send(REQUEST_ID, &tx_buf[..3]) };

        // Wait a bit for TX to complete or error
        let diag_wait = timer.system_time_us_64() + 100_000; // 100ms
        while timer.system_time_us_64() < diag_wait {}

        unsafe {
            let msr = reg_read(CAN1_BASE, 0x004); // MSR
            let tsr = reg_read(CAN1_BASE, 0x008); // TSR
            let rf0r = reg_read(CAN1_BASE, 0x00C); // RF0R
            let esr = reg_read(CAN1_BASE, 0x018); // ESR
            let btr = reg_read(CAN1_BASE, 0x01C); // BTR
            let _ = writeln!(uart, "  MSR=0x{:08X}  TSR=0x{:08X}  RF0R=0x{:08X}", msr, tsr, rf0r);
            let _ = writeln!(uart, "  ESR=0x{:08X}  BTR=0x{:08X}", esr, btr);
            let tec = (esr >> 16) & 0xFF;
            let rec = (esr >> 24) & 0xFF;
            let lec = (esr >> 4) & 0x7;
            let boff = (esr >> 2) & 1;
            let epvf = (esr >> 1) & 1;
            let _ = writeln!(uart, "  TEC={} REC={} LEC={} BOFF={} EPVF={}", tec, rec, lec, boff, epvf);
            // Check if mailbox 0 TX completed
            let rqcp0 = (tsr >> 0) & 1;
            let txok0 = (tsr >> 1) & 1;
            let alst0 = (tsr >> 2) & 1;
            let terr0 = (tsr >> 3) & 1;
            let tme0 = (tsr >> 26) & 1;
            let _ = writeln!(uart, "  RQCP0={} TXOK0={} ALST0={} TERR0={} TME0={}",
                rqcp0, txok0, alst0, terr0, tme0);
        }

        // Drain any RX frames from the diagnostic send
        while unsafe { bxcan1_rx_fifo0_read() }.is_some() {}
    }

    let _ = writeln!(uart, "\r\n--- Test sequence ---");

    let mut pass_count: usize = 0;
    let mut results = [false; NUM_TESTS];

    // ------------------------------------------------------------------
    // Test 1: TesterPresent (0x3E 0x00) → expect SF [02, 7E, 00]
    // ------------------------------------------------------------------
    {
        let req = [0x3E_u8, 0x00];
        let _ = writeln!(uart, "[1] TesterPresent  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect: 0x7E subFunction=0x00
                        let ok = uds.get(0) == Some(&0x7E)
                               && uds.get(1) == Some(&0x00);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[0] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 2: DiagnosticSessionControl extended (0x10 0x03)
    //         → expect SF [07, 50, 03, 00, 19, 01, F4]
    // ------------------------------------------------------------------
    {
        let req = [0x10_u8, 0x03];
        let _ = writeln!(uart, "[2] DiagSessionCtrl extended  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect: 0x50 sessionType=0x03
                        let ok = uds.get(0) == Some(&0x50)
                               && uds.get(1) == Some(&0x03);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[1] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 3: ReadDID unknown DID 0x0001 (0x22 0x00 0x01)
    //         → expect NRC SF [04, 7F, 22, 31]
    // ------------------------------------------------------------------
    {
        let req = [0x22_u8, 0x00, 0x01];
        let _ = writeln!(uart, "[3] ReadDID unknown DID  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect NRC: 0x7F SID=0x22 NRC=0x31 (requestOutOfRange)
                        let ok = uds.get(0) == Some(&0x7F)
                               && uds.get(1) == Some(&0x22)
                               && uds.get(2) == Some(&0x31);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[2] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 4: Unknown service (0xAA)
    //         → expect NRC SF [04, 7F, AA, 11]
    // ------------------------------------------------------------------
    {
        let req = [0xAA_u8];
        let _ = writeln!(uart, "[4] Unknown service 0xAA  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect NRC: 0x7F SID=0xAA NRC=0x11 (serviceNotSupported)
                        let ok = uds.get(0) == Some(&0x7F)
                               && uds.get(1) == Some(&0xAA)
                               && uds.get(2) == Some(&0x11);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[3] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // 8. Summary
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "\r\n=== RESULT: {}/{} PASS ===", pass_count, NUM_TESTS);
    for (i, &ok) in results.iter().enumerate() {
        let _ = writeln!(uart, "  Test {}: {}", i + 1, if ok { "PASS" } else { "FAIL" });
    }
    // F4 PolledUart is blocking — no flush needed

    // ------------------------------------------------------------------
    // 9. LED indication and loop
    //
    //    Fast blink (100 ms half-period) = all tests passed.
    //    Slow blink (500 ms half-period) = one or more tests failed.
    // ------------------------------------------------------------------
    let all_pass      = pass_count == NUM_TESTS;
    let half_period_us: u64 = if all_pass { 100_000 } else { 500_000 };

    blink_forever(&mut timer, half_period_us);
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

/// Toggle PA5 (LED) indefinitely at the given half-period (µs).  Never returns.
fn blink_forever(timer: &mut DwtTimer, half_period_us: u64) -> ! {
    let mut led_on = false;
    let mut last   = timer.system_time_us_64();
    loop {
        let now = timer.system_time_us_64();
        if now.wrapping_sub(last) >= half_period_us {
            last   = now;
            led_on = !led_on;
            unsafe { gpio_set(GPIOA_BASE, LED_PIN, led_on) };
        }
    }
}
