//! UDS diagnostic server on NUCLEO-F413ZH — real CAN bus via TJA transceiver.
//!
//! This is the real-bus variant of `uds_server_f413.rs`.  All ISO-TP and UDS
//! logic is identical; the differences are:
//!
//! * **GPIO**: PA11 (CAN1_RX) / PA12 (CAN1_TX), AF9 — connects through a
//!   TJA1050/TJA1051 transceiver on the NUCLEO morpho connector.  The loopback
//!   variant used PD0/PD1 (Arduino header).
//! * **No loopback mode**: LBKM (BTR bit 30) is never set.  The peripheral
//!   drives the physical CAN bus.
//! * **No startup self-test**: there is no loopback path to echo our own
//!   transmissions, so the self-test is removed.  The server simply prints
//!   "Waiting for UDS requests..." and enters the diagnostic loop immediately.
//! * **Multi-frame TX**: waits for a real Flow Control frame from the tester
//!   after sending the First Frame (no auto-proceed on timeout).
//!
//! # Services
//!
//! | SID  | Service                  | Positive response |
//! |------|--------------------------|-------------------|
//! | 0x3E | TesterPresent            | 0x7E              |
//! | 0x10 | DiagnosticSessionControl | 0x50              |
//! | 0x22 | ReadDataByIdentifier     | 0x62              |
//!
//! # ISO-TP
//!
//! Full Single Frame + Multi-frame (First Frame / Consecutive Frame / Flow
//! Control) ISO-TP is implemented.  Responses >7 bytes (e.g., ReadDID VIN =
//! 19 bytes) are automatically segmented into FF + CF sequence after receiving
//! a Flow Control frame from the tester.
//!
//! # CAN IDs
//!
//! | Direction        | CAN ID |
//! |------------------|--------|
//! | Tester → Server  | 0x600  |
//! | Server → Tester  | 0x601  |
//!
//! # Hardware connections (NUCLEO-F413ZH)
//!
//! ```text
//! PA11  →  TJA RXD   (CAN1_RX, AF9)
//! PA12  →  TJA TXD   (CAN1_TX, AF9)
//! GND   →  TJA GND
//! 3.3 V →  TJA VCC  (check your transceiver module — some need 5 V)
//! CAN_H / CAN_L connected between the two boards with 120 Ω termination
//! at each end.
//! ```
//!
//! # Bit timing: 500 kbit/s @ 48 MHz APB1
//!
//! ```text
//! Prescaler (BRP+1) = 6  → BRP = 5
//! Tseg1 (TS1+1)     = 11 → TS1 = 10
//! Tseg2 (TS2+1)     = 4  → TS2 = 3
//! SJW   (SJW+1)     = 1  → SJW = 0
//! Tq = 1/(48 MHz / 6) = 125 ns
//! Bit time = 1 + 11 + 4 = 16 Tq → 1/(16 × 125 ns) = 500 000 bit/s ✓
//! NOTE: LBKM (bit 30) is NOT set — real bus mode.
//! ```
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32f413 --target thumbv7em-none-eabihf \
//!             --example can_server_f413
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

/// GPIOA base address (RM0402 §8.4).
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

/// CAN master control register.
const CAN_MCR_OFFSET:  usize = 0x000;
/// CAN master status register.
const CAN_MSR_OFFSET:  usize = 0x004;
/// CAN transmit status register.
const CAN_TSR_OFFSET:  usize = 0x008;
/// CAN receive FIFO 0 register.
const CAN_RF0R_OFFSET: usize = 0x00C;
/// CAN interrupt enable register.
const CAN_IER_OFFSET:  usize = 0x014;
/// CAN bit timing register.
const CAN_BTR_OFFSET:  usize = 0x01C;

// TX mailbox 0 registers (RM0402 §32.9.3).
const CAN_TI0R_OFFSET:  usize = 0x180;
const CAN_TDT0R_OFFSET: usize = 0x184;
const CAN_TDL0R_OFFSET: usize = 0x188;
const CAN_TDH0R_OFFSET: usize = 0x18C;

// RX FIFO 0 mailbox registers (RM0402 §32.9.4).
const CAN_RI0R_OFFSET:  usize = 0x1B0;
const CAN_RDT0R_OFFSET: usize = 0x1B4;
const CAN_RDL0R_OFFSET: usize = 0x1B8;
const CAN_RDH0R_OFFSET: usize = 0x1BC;

// Filter bank registers (RM0402 §32.9.6).
const CAN_FMR_OFFSET:   usize = 0x200;
const CAN_FS1R_OFFSET:  usize = 0x20C;
const CAN_FFA1R_OFFSET: usize = 0x214;
const CAN_FA1R_OFFSET:  usize = 0x21C;
const CAN_F0R1_OFFSET:  usize = 0x240;
const CAN_F0R2_OFFSET:  usize = 0x244;

// ---------------------------------------------------------------------------
// bxCAN MCR bit masks
// ---------------------------------------------------------------------------

/// INRQ — initialisation request.
const MCR_INRQ:  u32 = 1 << 0;
/// SLEEP — sleep mode request.
const MCR_SLEEP: u32 = 1 << 1;
/// TXFP — transmit FIFO priority (chronological order).
const MCR_TXFP:  u32 = 1 << 2;
/// ABOM — automatic bus-off management.
const MCR_ABOM:  u32 = 1 << 6;

// ---------------------------------------------------------------------------
// bxCAN MSR / TSR / RF0R bit masks
// ---------------------------------------------------------------------------

/// INAK — init mode acknowledge.
const MSR_INAK: u32 = 1 << 0;

/// TME0/TME1/TME2 — transmit mailbox empty flags.
const TSR_TME0: u32 = 1 << 26;
const TSR_TME1: u32 = 1 << 27;
const TSR_TME2: u32 = 1 << 28;

/// FMP0 — FIFO 0 message pending (bits [1:0]).
const RF0R_FMP0_MASK: u32 = 0b11;
/// RFOM0 — release FIFO 0 output mailbox.
const RF0R_RFOM0: u32 = 1 << 5;

// ---------------------------------------------------------------------------
// bxCAN BTR value — 500 kbit/s @ APB1 = 48 MHz, real bus (NO loopback)
//
// Prescaler (BRP+1) = 6  → BRP = 5
// Tseg1 (TS1+1)     = 11 → TS1 = 10
// Tseg2 (TS2+1)     = 4  → TS2 = 3
// SJW   (SJW+1)     = 1  → SJW = 0
//
// Tq = 1/(48 MHz / 6) = 125 ns
// Bit time = 1 + 11 + 4 = 16 Tq → 500 000 bit/s ✓
//
// NOTE: bit 30 (LBKM) is NOT set — this is real bus mode.
// ---------------------------------------------------------------------------

const BTR_500KBPS_REAL: u32 =
    (0 << 24)  // SJW  = 0 (SJW+1 = 1 Tq)
    | (3 << 20)// TS2  = 3 (TS2+1 = 4 Tq)
    | (10 << 16)// TS1 = 10 (TS1+1 = 11 Tq)
    | 5;        // BRP  = 5 (BRP+1 = 6)

// ---------------------------------------------------------------------------
// bxCAN FMR bit masks
// ---------------------------------------------------------------------------

/// FINIT — filter init mode.
const FMR_FINIT: u32 = 1 << 0;

// ---------------------------------------------------------------------------
// LED pin: PA5 (LD2, NUCLEO-F413ZH)
// ---------------------------------------------------------------------------

const LED_PIN: u32 = 5;

// ---------------------------------------------------------------------------
// CAN IDs for ISO-TP (physical addressing, classic CAN 11-bit)
// ---------------------------------------------------------------------------

/// Physical request CAN ID: tester → server.
const REQUEST_ID:  u32 = 0x600;
/// Physical response CAN ID: server → tester.
const RESPONSE_ID: u32 = 0x601;

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
// GPIO helpers
// ---------------------------------------------------------------------------

/// Configure a GPIO pin as push-pull output, medium-speed, no pull.
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32F4.
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b01 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

/// Set or clear a GPIO pin via BSRR.
unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    // SAFETY: base + BSRR is valid GPIO MMIO.
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

/// Configure a GPIO pin as alternate function (MODER=0b10).
unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    // SAFETY: base + AF offsets are valid GPIO MMIO on STM32F4.
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

        // Enable CAN1 peripheral clock on APB1.
        reg_modify(RCC_BASE, RCC_APB1ENR_OFFSET, 0, CAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR_OFFSET); // read-back delay
    }
}

/// Enter bxCAN1 init mode (MCR.INRQ=1, wait for MSR.INAK=1).
/// Also clear sleep mode (MCR.SLEEP=0) per RM0402 §32.7.2.
unsafe fn bxcan1_enter_init() {
    // SAFETY: CAN1 registers valid on STM32F413ZH.
    unsafe {
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, MCR_SLEEP, MCR_INRQ);
        while (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) == 0 {}
    }
}

/// Leave bxCAN1 init mode (MCR.INRQ=0, wait for MSR.INAK=0).
unsafe fn bxcan1_leave_init() {
    // SAFETY: CAN1 registers valid on STM32F413ZH.
    unsafe {
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, MCR_INRQ, 0);
        while (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) != 0 {}
    }
}

/// Configure accept-all filter (bank 0, 32-bit mask mode, ID=0, mask=0).
///
/// All standard and extended frames will be accepted into FIFO 0.
unsafe fn bxcan1_configure_filters() {
    // SAFETY: CAN1 filter registers valid on STM32F413ZH.
    unsafe {
        reg_modify(CAN1_BASE, CAN_FMR_OFFSET,   0, FMR_FINIT);
        reg_modify(CAN1_BASE, CAN_FS1R_OFFSET,  0, 1 << 0);       // 32-bit scale
        reg_modify(CAN1_BASE, CAN_FFA1R_OFFSET, 1 << 0, 0);        // FIFO 0
        reg_write(CAN1_BASE, CAN_F0R1_OFFSET, 0x0000_0000);        // ID = 0
        reg_write(CAN1_BASE, CAN_F0R2_OFFSET, 0x0000_0000);        // mask = 0 (accept all)
        reg_modify(CAN1_BASE, CAN_FA1R_OFFSET,  0, 1 << 0);        // activate bank 0
        reg_modify(CAN1_BASE, CAN_FMR_OFFSET,  FMR_FINIT, 0);      // leave filter init
    }
}

/// Full bxCAN1 init for real CAN bus (no loopback).
///
/// Returns `true` if the peripheral successfully left init mode.
unsafe fn bxcan1_init() -> bool {
    // SAFETY: called once at startup in single-threaded bare-metal context.
    unsafe {
        bxcan1_gpio_clock_init();
        bxcan1_enter_init();

        // Options: ABOM (automatic bus-off recovery) + TXFP (chronological TX).
        reg_modify(CAN1_BASE, CAN_MCR_OFFSET, 0, MCR_ABOM | MCR_TXFP);

        // Bit timing: 500 kbps @ 48 MHz APB1.  NOTE: LBKM bit is NOT set.
        reg_write(CAN1_BASE, CAN_BTR_OFFSET, BTR_500KBPS_REAL);

        // Disable all interrupts (polled mode).
        reg_write(CAN1_BASE, CAN_IER_OFFSET, 0x0000_0000);

        bxcan1_configure_filters();
        bxcan1_leave_init();

        // Verify: INAK must be 0 after leaving init mode.
        (reg_read(CAN1_BASE, CAN_MSR_OFFSET) & MSR_INAK) == 0
    }
}

// ---------------------------------------------------------------------------
// bxCAN TX / RX helpers
// ---------------------------------------------------------------------------

/// Write to TX mailbox 0 and request transmission.
///
/// Sends a standard-ID (11-bit) classic CAN frame.  Waits for any empty
/// mailbox (TME0|TME1|TME2 in TSR) before writing.
///
/// TX mailbox 0 register layout (RM0402 §32.9.3):
///   TI0R [31:21] = standard ID (STID), bit [0] TXRQ
///   TDT0R [3:0]  = DLC
///   TDL0R        = data bytes [3:0] little-endian
///   TDH0R        = data bytes [7:4] little-endian
unsafe fn bxcan1_send(id: u32, data: &[u8]) {
    // SAFETY: CAN1 registers valid on STM32F413ZH.
    unsafe {
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

        // Write TI0R: standard ID in bits [31:21], TXRQ=0 for now.
        reg_write(CAN1_BASE, CAN_TI0R_OFFSET,  (id & 0x7FF) << 21);
        // Write TDT0R: DLC only (no timestamp).
        reg_write(CAN1_BASE, CAN_TDT0R_OFFSET, dlc & 0xF);
        // Write data.
        reg_write(CAN1_BASE, CAN_TDL0R_OFFSET, tdlr);
        reg_write(CAN1_BASE, CAN_TDH0R_OFFSET, tdhr);
        // Set TXRQ bit (bit 0 of TI0R) to request transmission.
        reg_modify(CAN1_BASE, CAN_TI0R_OFFSET, 0, 1 << 0);
    }
}

/// One received CAN frame.
struct RxFrame {
    id:   u32,
    dlc:  u8,
    data: [u8; 8],
}

/// Read one frame from RX FIFO 0 and release it.
///
/// Returns `None` if FIFO 0 is empty.
///
/// RX FIFO 0 mailbox layout (RM0402 §32.9.4):
///   RF0R [1:0]   = FMP0 (messages pending)
///   RI0R [31:21] = STID (standard ID), bit [2] IDE
///   RDT0R [3:0]  = DLC
///   RDL0R / RDH0R = data bytes little-endian
unsafe fn bxcan1_rx_fifo0_read() -> Option<RxFrame> {
    // SAFETY: CAN1 registers valid on STM32F413ZH.
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

        // Release FIFO 0 output mailbox (RF0R.RFOM0).
        reg_modify(CAN1_BASE, CAN_RF0R_OFFSET, 0, RF0R_RFOM0);

        Some(RxFrame { id, dlc, data })
    }
}

// ---------------------------------------------------------------------------
// ISO-TP frame types (ISO 15765-2)
//
//   0x0N = Single Frame      (N = payload length, 1-7)
//   0x1M = First Frame       (M:LL = total message length, 12-bit)
//   0x2N = Consecutive Frame (N = sequence number, 0-F wrapping)
//   0x30 = Flow Control      (CTS / Wait / Overflow)
// ---------------------------------------------------------------------------

/// ISO-TP frame type decoded from upper nibble of PCI byte.
#[derive(Debug, Clone, Copy, PartialEq)]
enum IsoTpFrameType {
    SingleFrame,
    FirstFrame,
    ConsecutiveFrame,
    FlowControl,
}

fn isotp_frame_type(pci: u8) -> Option<IsoTpFrameType> {
    match pci >> 4 {
        0 => Some(IsoTpFrameType::SingleFrame),
        1 => Some(IsoTpFrameType::FirstFrame),
        2 => Some(IsoTpFrameType::ConsecutiveFrame),
        3 => Some(IsoTpFrameType::FlowControl),
        _ => None,
    }
}

/// Encode a UDS response as an ISO-TP Single Frame CAN payload.
///
/// SF PCI: `byte[0] = (0x0 << 4) | payload_len`, then payload bytes.
/// Returns number of bytes written to `out`.  Caller must ensure `payload <= 7`.
fn isotp_encode_sf(uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    let payload_len = uds_data.len().min(7);
    out[0] = payload_len as u8;
    out[1..1 + payload_len].copy_from_slice(&uds_data[..payload_len]);
    1 + payload_len
}

/// Encode an ISO-TP First Frame.
///
/// FF PCI: `byte[0] = 0x10 | (msg_len >> 8)`, `byte[1] = msg_len & 0xFF`,
/// then up to 6 payload bytes.
fn isotp_encode_ff(msg_len: u16, uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    out[0] = 0x10 | ((msg_len >> 8) as u8 & 0x0F);
    out[1] = (msg_len & 0xFF) as u8;
    let first_chunk = uds_data.len().min(6);
    out[2..2 + first_chunk].copy_from_slice(&uds_data[..first_chunk]);
    for b in &mut out[2 + first_chunk..8] {
        *b = 0xCC; // ISO-TP padding
    }
    8
}

/// Encode an ISO-TP Consecutive Frame.
///
/// CF PCI: `byte[0] = 0x20 | (seq_num & 0x0F)`, then up to 7 payload bytes.
fn isotp_encode_cf(seq_num: u8, data: &[u8], out: &mut [u8; 8]) -> usize {
    out[0] = 0x20 | (seq_num & 0x0F);
    let chunk = data.len().min(7);
    out[1..1 + chunk].copy_from_slice(&data[..chunk]);
    for b in &mut out[1 + chunk..8] {
        *b = 0xCC;
    }
    8
}

/// Encode an ISO-TP Flow Control frame (CTS, block_size=0, STmin=0).
fn isotp_encode_fc_cts(out: &mut [u8; 8]) -> usize {
    out[0] = 0x30; // FC, ContinueToSend
    out[1] = 0x00; // Block size = 0 (no limit)
    out[2] = 0x00; // STmin = 0 ms
    for b in &mut out[3..8] {
        *b = 0xCC;
    }
    8
}

/// Send a complete UDS response over ISO-TP.
///
/// * SF (≤7 bytes): one CAN frame.
/// * Multi-frame (>7 bytes): sends First Frame, then waits for a real Flow
///   Control frame from the tester before sending Consecutive Frames.  If no
///   FC arrives within 1 s (N_Bs timeout), the transfer is aborted.
///
/// # Safety
/// Calls `bxcan1_send` and `bxcan1_rx_fifo0_read` which access CAN1 MMIO.
unsafe fn isotp_send_response(
    resp_data: &[u8],
    resp_len:  usize,
    timer:     &mut DwtTimer,
    uart:      &mut PolledUart,
) {
    if resp_len <= 7 {
        // Single Frame.
        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&resp_data[..resp_len], &mut tx_buf);
        let _ = write!(uart, "  tx=[");
        for (i, b) in tx_buf[..tx_len].iter().enumerate() {
            if i > 0 { let _ = write!(uart, ", "); }
            let _ = write!(uart, "{:02X}", b);
        }
        let _ = writeln!(uart, "]");
        unsafe { bxcan1_send(RESPONSE_ID, &tx_buf[..tx_len]) };
    } else {
        // Multi-frame: First Frame + Consecutive Frames.
        let msg_len   = resp_len as u16;
        let ff_payload = resp_len.min(6);

        // 1. Send First Frame.
        let mut tx_buf = [0u8; 8];
        isotp_encode_ff(msg_len, &resp_data[..ff_payload], &mut tx_buf);
        let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, ff_payload);
        unsafe { bxcan1_send(RESPONSE_ID, &tx_buf) };

        // 2. Wait for real Flow Control from the tester (1 s N_Bs timeout).
        //    The tester sends FC on REQUEST_ID (0x600) after receiving our FF.
        let fc_deadline = timer.system_time_us_64() + 1_000_000;
        let mut got_fc  = false;
        loop {
            if let Some(f) = unsafe { bxcan1_rx_fifo0_read() } {
                if f.id == REQUEST_ID {
                    let pci = f.data[0];
                    if let Some(IsoTpFrameType::FlowControl) = isotp_frame_type(pci) {
                        let fc_status = pci & 0x0F;
                        if fc_status == 0 {
                            // CTS — Continue To Send.
                            got_fc = true;
                            let _ = writeln!(uart, "  FC: CTS received");
                            break;
                        } else if fc_status == 1 {
                            // WAIT — reset deadline and keep polling.
                            let _ = writeln!(uart, "  FC: WAIT");
                            continue;
                        } else {
                            // Overflow — abort transfer.
                            let _ = writeln!(uart, "  FC: OVERFLOW, aborting");
                            return;
                        }
                    }
                }
                // Discard stray frames (bus noise, etc.).
            }
            if timer.system_time_us_64() >= fc_deadline {
                let _ = writeln!(uart, "  FC: timeout (1 s N_Bs), aborting multi-frame TX");
                return;
            }
        }

        if !got_fc {
            return;
        }

        // 3. Send Consecutive Frames.
        let mut offset = ff_payload;
        let mut seq: u8 = 1;
        while offset < resp_len {
            let remaining = resp_len - offset;
            let chunk     = remaining.min(7);
            isotp_encode_cf(seq, &resp_data[offset..offset + chunk], &mut tx_buf);
            let _ = writeln!(uart, "  CF[{}]: offset={} chunk={}", seq, offset, chunk);
            unsafe { bxcan1_send(RESPONSE_ID, &tx_buf) };
            offset += chunk;
            seq = (seq + 1) & 0x0F;

            // Brief inter-frame delay (STmin=0 but give HW time).
            let cf_wait = timer.system_time_us_64() + 500;
            while timer.system_time_us_64() < cf_wait {}
        }
        let _ = writeln!(uart, "  Multi-frame TX complete: {} bytes", resp_len);
    }
}

/// Decode an incoming ISO-TP frame (SF or FF+CF multi-frame).
///
/// For multi-frame RX: sends FC(CTS) on RESPONSE_ID, collects CFs and
/// reassembles into `rx_buf`.  Returns the total payload length, or 0 on
/// failure.
///
/// # Safety
/// Calls `bxcan1_send` and `bxcan1_rx_fifo0_read` for FC/CF exchange.
unsafe fn isotp_receive_request(
    frame_data: &[u8],
    dlc:        usize,
    rx_buf:     &mut [u8; 256],
    timer:      &mut DwtTimer,
    uart:       &mut PolledUart,
) -> usize {
    let pci = frame_data[0];
    match isotp_frame_type(pci) {
        Some(IsoTpFrameType::SingleFrame) => {
            let payload_len = (pci & 0x0F) as usize;
            if payload_len == 0 || payload_len > 7 || payload_len + 1 > dlc {
                return 0;
            }
            rx_buf[..payload_len].copy_from_slice(&frame_data[1..1 + payload_len]);
            payload_len
        }
        Some(IsoTpFrameType::FirstFrame) => {
            let msg_len = (((pci & 0x0F) as usize) << 8) | (frame_data[1] as usize);
            if msg_len == 0 || msg_len > rx_buf.len() {
                let _ = writeln!(uart, "  FF: msg_len={} too large, dropping", msg_len);
                return 0;
            }
            let first_chunk = (dlc - 2).min(6).min(msg_len);
            rx_buf[..first_chunk].copy_from_slice(&frame_data[2..2 + first_chunk]);
            let mut received = first_chunk;

            let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, first_chunk);

            // Send Flow Control: CTS, BS=0, STmin=0 on the response channel.
            let mut fc_buf = [0u8; 8];
            isotp_encode_fc_cts(&mut fc_buf);
            unsafe { bxcan1_send(RESPONSE_ID, &fc_buf) };

            // Collect Consecutive Frames on REQUEST_ID.
            let mut expected_seq: u8 = 1;
            let cf_deadline = timer.system_time_us_64() + 1_000_000;
            while received < msg_len {
                if let Some(f) = unsafe { bxcan1_rx_fifo0_read() } {
                    if f.id != REQUEST_ID {
                        continue;
                    }
                    let cf_pci = f.data[0];
                    if let Some(IsoTpFrameType::ConsecutiveFrame) = isotp_frame_type(cf_pci) {
                        let seq = cf_pci & 0x0F;
                        if seq != (expected_seq & 0x0F) {
                            let _ = writeln!(uart,
                                "  CF: seq mismatch expected={} got={}",
                                expected_seq & 0x0F, seq);
                            return 0;
                        }
                        let remaining = msg_len - received;
                        let chunk  = remaining.min(7);
                        let cf_dlc = f.dlc.min(8) as usize;
                        let avail  = (cf_dlc - 1).min(chunk);
                        rx_buf[received..received + avail]
                            .copy_from_slice(&f.data[1..1 + avail]);
                        received += avail;
                        expected_seq = expected_seq.wrapping_add(1);
                    }
                }
                if timer.system_time_us_64() >= cf_deadline {
                    let _ = writeln!(uart, "  CF: timeout after {} of {} bytes",
                                     received, msg_len);
                    return 0;
                }
            }
            let _ = writeln!(uart, "  Multi-frame RX complete: {} bytes", msg_len);
            msg_len
        }
        _ => {
            // FC or CF without context — ignore.
            0
        }
    }
}

// ---------------------------------------------------------------------------
// UDS request handler
// ---------------------------------------------------------------------------

/// Dispatch a decoded UDS request and write the positive (or NRC) response.
///
/// Returns the number of bytes written to `response`.
///
/// | SID  | Service                     |
/// |------|-----------------------------|
/// | 0x3E | TesterPresent               |
/// | 0x10 | DiagnosticSessionControl    |
/// | 0x22 | ReadDataByIdentifier        |
///
/// | DID    | Description      | Value               |
/// |--------|------------------|---------------------|
/// | 0xF190 | VIN              | `TAKTFLOW_F413_01`  |
/// | 0xF195 | SW version       | `BSP-0.1.0`         |
fn handle_uds_request(request: &[u8], response: &mut [u8]) -> usize {
    match request.first() {
        // ------------------------------------------------------------------
        // 0x3E TesterPresent
        // ------------------------------------------------------------------
        Some(&0x3E) => {
            response[0] = 0x7E;
            response[1] = request.get(1).copied().unwrap_or(0x00);
            2
        }

        // ------------------------------------------------------------------
        // 0x10 DiagnosticSessionControl
        // ------------------------------------------------------------------
        Some(&0x10) => {
            let session = request.get(1).copied().unwrap_or(0x01);
            response[0] = 0x50;
            response[1] = session;
            response[2] = 0x00; // P2 high byte   (P2  = 25 ms)
            response[3] = 0x19; // P2 low byte
            response[4] = 0x01; // P2* high byte  (P2* = 5000 ms)
            response[5] = 0xF4; // P2* low byte
            6
        }

        // ------------------------------------------------------------------
        // 0x22 ReadDataByIdentifier
        // ------------------------------------------------------------------
        Some(&0x22) => {
            let did_hi = request.get(1).copied().unwrap_or(0x00);
            let did_lo = request.get(2).copied().unwrap_or(0x00);
            let did    = u16::from_be_bytes([did_hi, did_lo]);
            match did {
                0xF190 => {
                    // VIN — 16-char ASCII (multi-frame response: 19 bytes total)
                    response[0] = 0x62;
                    response[1] = 0xF1;
                    response[2] = 0x90;
                    let vin = b"TAKTFLOW_F413_01";
                    let len = vin.len().min(17).min(response.len() - 3);
                    response[3..3 + len].copy_from_slice(&vin[..len]);
                    3 + len
                }
                0xF195 => {
                    // Software version (12 bytes total — fits in SF)
                    response[0] = 0x62;
                    response[1] = 0xF1;
                    response[2] = 0x95;
                    let ver = b"BSP-0.1.0";
                    let len = ver.len().min(response.len() - 3);
                    response[3..3 + len].copy_from_slice(&ver[..len]);
                    3 + len
                }
                _ => {
                    // NRC 0x31 requestOutOfRange
                    response[0] = 0x7F;
                    response[1] = 0x22;
                    response[2] = 0x31;
                    3
                }
            }
        }

        // ------------------------------------------------------------------
        // All other services — NRC 0x11 serviceNotSupported
        // ------------------------------------------------------------------
        _ => {
            response[0] = 0x7F;
            response[1] = request.first().copied().unwrap_or(0x00);
            response[2] = 0x11;
            3
        }
    }
}

// ---------------------------------------------------------------------------
// Main loop: process one incoming request frame and emit a UDS response.
// ---------------------------------------------------------------------------

/// Process one pending request from RX FIFO 0.
///
/// Returns `true` if a REQUEST_ID frame was decoded and a response was sent,
/// `false` if the FIFO was empty or the frame was not a recognised request.
unsafe fn process_one_request(uart: &mut PolledUart, timer: &mut DwtTimer) -> bool {
    let frame = match unsafe { bxcan1_rx_fifo0_read() } {
        Some(f) => f,
        None    => return false,
    };

    // Only process tester requests (0x600).  Discard stray frames.
    if frame.id != REQUEST_ID {
        return false;
    }

    let dlc = frame.dlc.min(8) as usize;

    // ISO-TP: decode incoming frame (SF or FF+CF multi-frame).
    let mut rx_buf = [0u8; 256];
    let payload_len = unsafe {
        isotp_receive_request(&frame.data[..dlc], dlc, &mut rx_buf, timer, uart)
    };
    if payload_len == 0 {
        return false;
    }

    let uds_request = &rx_buf[..payload_len];

    // UDS dispatch.
    let mut resp_buf = [0u8; 256];
    let resp_len = handle_uds_request(uds_request, &mut resp_buf);

    // ISO-TP encode + transmit response.
    unsafe { isotp_send_response(&resp_buf, resp_len, timer, uart) };

    // Heartbeat blink: toggle LED on every successful exchange (500 ms).
    let ts  = timer.system_time_us_64();
    let led = ((ts / 500_000) & 1) != 0;
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, led) };

    let _ = writeln!(uart,
        "UDS: SID=0x{:02X}  req_len={}  resp_len={}",
        uds_request.first().copied().unwrap_or(0),
        payload_len,
        resp_len);

    true
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
        "\r\n=== CAN server (real bus) F413ZH starting @ {} MHz ===",
        sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    // ------------------------------------------------------------------
    // GPIOA clock is enabled by bxcan1_gpio_clock_init(); configure PA5 here.
    // Call order: UART init enables GPIOD; CAN init enables GPIOA.
    // To be safe, enable GPIOA clock explicitly now.
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
        blink_forever(&mut timer, 200_000); // fast blink = fatal error
    }
    let _ = writeln!(uart,
        "bxCAN1 ready.  Listen=0x{:03X}  Reply=0x{:03X}",
        REQUEST_ID, RESPONSE_ID);
    let _ = writeln!(uart, "Waiting for UDS requests from tester...\r\n");
    // F4 PolledUart is blocking — no flush needed

    // Solid LED = peripheral initialised successfully.
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, true) };

    // ------------------------------------------------------------------
    // 6. Continuous diagnostic server loop
    //
    //    - Poll RX FIFO 0 for request frames (CAN ID 0x600).
    //    - Decode ISO-TP SF / FF+CF, dispatch to UDS handler.
    //    - Send response on CAN ID 0x601.
    //    - Blink LED on each successful exchange (500 ms half-period).
    //    - Slow heartbeat when idle (1 Hz).
    // ------------------------------------------------------------------
    loop {
        let processed = unsafe { process_one_request(&mut uart, &mut timer) };

        // Slow heartbeat blink (1 Hz) when idle.
        if !processed {
            let ts  = timer.system_time_us_64();
            let led = ((ts / 1_000_000) & 1) != 0;
            unsafe { gpio_set(GPIOA_BASE, LED_PIN, led) };
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
