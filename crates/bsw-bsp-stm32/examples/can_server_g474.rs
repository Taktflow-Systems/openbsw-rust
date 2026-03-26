//! UDS diagnostic server on NUCLEO-G474RE — real CAN bus via TJA transceiver.
//!
//! This is a real-bus variant of `uds_server_g474.rs`.  All ISO-TP and UDS
//! logic is identical; the differences are:
//!
//! * **GPIO**: PA11 (FDCAN1_RX) / PA12 (FDCAN1_TX), AF9 — matches the
//!   production wiring through a TJA1050/TJA1051 transceiver on the NUCLEO
//!   morpho connector.  The loopback variant used PB8/PB9 (Arduino header).
//! * **No loopback mode**: CCCR.TEST and TEST.LBCK are never set.  The
//!   peripheral drives the physical CAN bus.
//! * **No startup self-test**: there is no loopback path to echo our own
//!   transmissions, so the self-test is removed.  The server simply prints
//!   "ready" and enters the diagnostic loop immediately.
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
//! # Hardware connections (NUCLEO-G474RE)
//!
//! ```text
//! PA11  →  TJA RXD   (FDCAN1_RX, AF9)
//! PA12  →  TJA TXD   (FDCAN1_TX, AF9)
//! GND   →  TJA GND
//! 3.3 V →  TJA VCC  (check your transceiver module — some need 5 V)
//! CAN_H / CAN_L connected between the two boards with 120 Ω termination
//! at each end.
//! ```
//!
//! # Bit timing: 500 kbit/s @ 170 MHz PCLK1
//!
//! ```text
//! Prescaler = 17  (NBRP field = 16)
//! Tseg1     = 15  (NTSEG1 field = 14)   — 15 TQ
//! Tseg2     =  4  (NTSEG2 field =  3)   —  4 TQ
//! SJW       =  4  (NSJW  field =  3)
//! Bit time  = 1 + 15 + 4 = 20 TQ
//! Baud      = 170 MHz / (17 × 20) = 500 000 bit/s ✓
//! ```
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!             --example can_server_g474
//! ```
//!
//! # Reference
//!
//! RM0440 Rev 8, §44 (FDCAN); ISO 15765-2:2016 (ISO-TP); ISO 14229-1:2020 (UDS).

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

/// GPIOA base address.
const GPIOA_BASE: usize = 0x4800_0000;

/// GPIO register offsets.
const GPIO_MODER_OFFSET:   usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_AFRL_OFFSET:    usize = 0x20;
const GPIO_AFRH_OFFSET:    usize = 0x24;
const GPIO_BSRR_OFFSET:    usize = 0x18;

/// FDCAN1 register-block base address (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// Message RAM base address for FDCAN1 (RM0440 §44.3.3).
///
/// On STM32G4 this is a hardware-fixed 212-word (848-byte) SRAM region.
/// Layout is silicon-fixed — do NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC.
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets (STM32G4-specific)
// ---------------------------------------------------------------------------

const FDCAN_CCCR_OFFSET:  usize = 0x018; // CC control register
const FDCAN_NBTP_OFFSET:  usize = 0x01C; // Nominal bit timing
const FDCAN_RXGFC_OFFSET: usize = 0x080; // RX global filter config
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 ack
const FDCAN_TXBC_OFFSET:  usize = 0x0C0; // TX buffer config
const FDCAN_TXFQS_OFFSET: usize = 0x0C4; // TX FIFO/queue status
const FDCAN_TXBAR_OFFSET: usize = 0x0CC; // TX buffer add request

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

/// INIT — initialisation mode.
const CCCR_INIT: u32 = 1 << 0;
/// CCE — configuration change enable (requires INIT=1).
const CCCR_CCE:  u32 = 1 << 1;
/// FDOE — FD operation; 0 = classic CAN only.
const CCCR_FDOE: u32 = 1 << 8;
/// BRSE — bit-rate switching; 0 = disabled.
const CCCR_BRSE: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// STM32G4 Message RAM layout (hardware-fixed, RM0440 §44.3.3)
// ---------------------------------------------------------------------------

const MRAM_RXF0_OFFSET:  usize = 0x0B0;
const MRAM_TXBUF_OFFSET: usize = 0x278;

/// Stride between RX/TX elements: 18 words = 72 bytes.
const MRAM_ELEMENT_STRIDE: usize = 18 * 4; // 72

/// Word size in bytes.
const WORD: usize = 4;

// ---------------------------------------------------------------------------
// TXBC value
//
// TXBC [29:24] TFQS=3, [21:16] NDTB=0, [15:0] TBSA = 0x278/4 = 0x9E.
// ---------------------------------------------------------------------------

const TXBC_VAL: u32 = (3 << 24) | (MRAM_TXBUF_OFFSET as u32 / 4);

// ---------------------------------------------------------------------------
// Bit timing: 500 kbit/s @ 170 MHz PCLK1
//
// NBRP=16 (prescaler=17), NTSEG1=14 (Tseg1=15 TQ), NTSEG2=3 (Tseg2=4 TQ),
// NSJW=3 (SJW=4 TQ).  Tq = 17/170 MHz = 100 ns; bit = 20 × 100 ns = 500 kbps.
// ---------------------------------------------------------------------------

// STM32G4 FDCAN NBTP layout (DIFFERENT from standard M_CAN!):
//   [31:25] NSJW   = SJW − 1
//   [24:16] NBRP   = prescaler − 1    ← M_CAN has NTSEG1 here!
//   [15: 8] NTSEG1 = Tseg1 − 1        ← M_CAN has NTSEG2 here!
//   [ 6: 0] NTSEG2 = Tseg2 − 1        ← M_CAN has NBRP here!
const NBTP_500KBPS: u32 = {
    let nbrp:   u32 = 16;  // prescaler = 17
    let ntseg1: u32 = 14;  // Tseg1 = 15 TQ
    let ntseg2: u32 =  3;  // Tseg2 = 4 TQ
    let nsjw:   u32 =  3;  // SJW = 4 TQ
    (nsjw << 25) | (nbrp << 16) | (ntseg1 << 8) | ntseg2
};

// ---------------------------------------------------------------------------
// LED pin: PA5 (LD2, NUCLEO-G474RE)
// ---------------------------------------------------------------------------

const LED_PIN: u32 = 5;

// ---------------------------------------------------------------------------
// CAN IDs for ISO-TP (physical addressing, classic CAN 11-bit)
// ---------------------------------------------------------------------------

/// Physical request CAN ID: tester → server.
const REQUEST_ID: u32 = 0x600;
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
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32G4.
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

/// Configure a GPIO pin as alternate function (AF mode = MODER=10).
unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    // SAFETY: base + AF offsets are valid GPIO MMIO on STM32G4.
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
// FDCAN1 initialisation (real bus — no loopback)
// ---------------------------------------------------------------------------

/// Enable FDCAN1 clock, configure PA11 (RX) / PA12 (TX) as AF9, select PCLK1.
///
/// PA11 and PA12 connect to a TJA105x transceiver on the morpho header.
/// AF9 = FDCAN1_RX / FDCAN1_TX on these pins (RM0440 §4, AF table).
unsafe fn fdcan1_gpio_clock_init() {
    // SAFETY: RCC, GPIOA addresses are valid on STM32G474RE.
    unsafe {
        // Enable GPIOA clock (USART2 init may have done this already; safe to
        // set again — it is write-only-set in the enable register).
        reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, GPIOAEN);
        let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET); // read-back delay

        // PA11 = FDCAN1_RX (AF9), PA12 = FDCAN1_TX (AF9).
        gpio_af(GPIOA_BASE, 11, 9);
        gpio_af(GPIOA_BASE, 12, 9);

        // Select PCLK1 as FDCAN kernel clock (CCIPR bits [25:24] = 0b10).
        reg_modify(RCC_BASE, RCC_CCIPR_OFFSET, 0b11 << 24, 0b10 << 24);

        // Enable FDCAN1 peripheral clock.
        reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, FDCAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET); // read-back delay
    }
}

/// Enter FDCAN1 INIT+CCE mode (required before touching configuration).
unsafe fn fdcan1_enter_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_INIT);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0 {}
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_CCE);
        // Classic CAN only — clear FD and BRS bits.
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_FDOE | CCCR_BRSE, 0);
    }
}

/// Leave INIT mode so FDCAN1 can participate in bus traffic.
unsafe fn fdcan1_leave_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_INIT | CCCR_CCE, 0);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) != 0 {}
    }
}

/// Configure TXBC and RXGFC.  Do NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC.
unsafe fn fdcan1_configure_mram() {
    unsafe {
        // TXBC: 3 TX FIFO elements, no dedicated buffers.
        reg_write(FDCAN1_BASE, FDCAN_TXBC_OFFSET, TXBC_VAL);
        // RXGFC=0: accept all non-matching std + ext frames into FIFO 0.
        reg_write(FDCAN1_BASE, FDCAN_RXGFC_OFFSET, 0x0000_0000);
    }
}

/// Full FDCAN1 init for real CAN bus operation (no loopback).
///
/// Returns `true` if the peripheral left init mode successfully.
unsafe fn fdcan1_init() -> bool {
    unsafe {
        fdcan1_gpio_clock_init();
        fdcan1_enter_init();
        reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);
        fdcan1_configure_mram();
        // NOTE: no fdcan1_enable_loopback() call — real bus mode.
        fdcan1_leave_init();
        (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0
    }
}

// ---------------------------------------------------------------------------
// CAN TX / RX helpers (raw message RAM access)
// ---------------------------------------------------------------------------

/// Write a TX buffer element and request transmission.
///
/// Sends a standard-ID (11-bit) classic CAN frame.  Uses the TX FIFO put
/// index from TXFQS to locate the next free slot.
///
/// TX element layout (RM0440 §44.4.18):
///   Word 0 = T0: std ID in [28:18], XTD=0, RTR=0, ESI=0
///   Word 1 = T1: DLC in [19:16], BRS=0, FDF=0
///   Word 2 = DB0: payload bytes [3:0] little-endian
///   Word 3 = DB1: payload bytes [7:4] little-endian
unsafe fn fdcan1_send(id: u32, data: &[u8]) {
    // SAFETY: message RAM and FDCAN1 register addresses valid for STM32G474.
    unsafe {
        // Wait until TX FIFO has a free slot (TXFQS bit 21 = FIFO full).
        loop {
            if (reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) & (1 << 21)) == 0 {
                break;
            }
        }

        // TXFQS [12:8] = put index.
        let txfqs   = reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET);
        let put_idx = ((txfqs >> 8) & 0x1F) as usize;

        let txbuf_addr = SRAMCAN_BASE + MRAM_TXBUF_OFFSET + put_idx * MRAM_ELEMENT_STRIDE;

        let t0  = (id & 0x7FF) << 18;
        let dlc = data.len().min(8) as u32;
        let t1  = dlc << 16;

        let mut raw = [0u8; 8];
        let n = data.len().min(8);
        raw[..n].copy_from_slice(&data[..n]);
        let db0 = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let db1 = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);

        mram_write(txbuf_addr,            t0);
        mram_write(txbuf_addr +     WORD, t1);
        mram_write(txbuf_addr + 2 * WORD, db0);
        mram_write(txbuf_addr + 3 * WORD, db1);

        reg_write(FDCAN1_BASE, FDCAN_TXBAR_OFFSET, 1 << put_idx);
    }
}

/// One received CAN frame from RX FIFO 0.
struct RxFrame {
    id:   u32,
    dlc:  u8,
    data: [u8; 8],
}

/// Read one frame from RX FIFO 0 and acknowledge it.
///
/// Returns `None` if the FIFO is empty.
///
/// RXF0S on STM32G4 (RM0440 §44.8.21):
///   [3:0]   F0FL fill level (max 3)
///   [9:8]   F0GI get index
///   [17:16] F0PI put index
///
/// RX element layout (RM0440 §44.4.16):
///   Word 0 = R0: std ID [28:18] (if XTD=0), XTD [30], RTR [29]
///   Word 1 = R1: DLC [19:16]
///   Word 2 = DB0: payload bytes [3:0] little-endian
///   Word 3 = DB1: payload bytes [7:4] little-endian
unsafe fn fdcan1_rx_fifo0_read() -> Option<RxFrame> {
    // SAFETY: FDCAN1 and message RAM addresses valid for STM32G474.
    unsafe {
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);

        // F0FL: bits [3:0] on STM32G4.
        let fill = rxf0s & 0xF;
        if fill == 0 {
            return None;
        }

        // F0GI: bits [9:8] on STM32G4.
        let get_idx = ((rxf0s >> 8) & 0x3) as usize;

        let elem_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * MRAM_ELEMENT_STRIDE;

        let r0  = mram_read(elem_addr);
        let r1  = mram_read(elem_addr + WORD);
        let db0 = mram_read(elem_addr + 2 * WORD);
        let db1 = mram_read(elem_addr + 3 * WORD);

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
        data[4..].copy_from_slice(&w1);

        // Acknowledge — RXF0A [2:0] = get_idx.
        reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32);

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

/// ISO-TP frame type, decoded from upper nibble of PCI byte.
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
/// Returns number of bytes written to `out`. Caller must ensure `resp_len <= 7`.
fn isotp_encode_sf(uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    let payload_len = uds_data.len().min(7);
    out[0] = payload_len as u8; // PCI: type=0 (SF), length
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
        *b = 0xCC;
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
/// Uses Single Frame for payloads <= 7 bytes.  For longer responses, sends
/// FF then waits for a real Flow Control frame from the tester before
/// sending Consecutive Frames.
///
/// # Safety
/// Calls `fdcan1_send` and `fdcan1_rx_fifo0_read` which access FDCAN1 MMIO.
unsafe fn isotp_send_response(
    resp_data: &[u8],
    resp_len:  usize,
    timer:     &mut DwtTimer,
    uart:      &mut PolledUart,
) {
    if resp_len <= 7 {
        // Single Frame — fits in one CAN frame.
        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&resp_data[..resp_len], &mut tx_buf);
        let _ = write!(uart, "  tx=[");
        for (i, b) in tx_buf[..tx_len].iter().enumerate() {
            if i > 0 { let _ = write!(uart, ", "); }
            let _ = write!(uart, "{:02X}", b);
        }
        let _ = writeln!(uart, "]");
        unsafe { fdcan1_send(RESPONSE_ID, &tx_buf[..tx_len]) };
    } else {
        // Multi-frame: First Frame + Consecutive Frames.
        let msg_len = resp_len as u16;

        // 1. Send First Frame (6 payload bytes).
        let mut tx_buf = [0u8; 8];
        let ff_payload = resp_len.min(6);
        isotp_encode_ff(msg_len, &resp_data[..ff_payload], &mut tx_buf);
        let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, ff_payload);
        unsafe { fdcan1_send(RESPONSE_ID, &tx_buf) };

        // 2. Wait for real Flow Control frame from the tester.
        //    The tester sends FC on REQUEST_ID (0x600) after receiving our FF.
        let fc_deadline = timer.system_time_us_64() + 1_000_000; // 1 s N_Bs timeout
        let mut got_fc = false;
        loop {
            if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                // FC arrives on the request channel (0x600).
                if f.id == REQUEST_ID {
                    let pci = f.data[0];
                    if let Some(IsoTpFrameType::FlowControl) = isotp_frame_type(pci) {
                        let fc_status = pci & 0x0F;
                        if fc_status == 0 {
                            // CTS (Continue To Send)
                            got_fc = true;
                            let _ = writeln!(uart, "  FC: CTS received");
                            break;
                        } else if fc_status == 1 {
                            // WAIT — reset deadline and keep polling
                            let _ = writeln!(uart, "  FC: WAIT");
                            continue;
                        } else {
                            // Overflow — abort
                            let _ = writeln!(uart, "  FC: OVERFLOW, aborting");
                            return;
                        }
                    }
                }
            }
            if timer.system_time_us_64() >= fc_deadline {
                let _ = writeln!(uart, "  FC: timeout, aborting multi-frame TX");
                return;
            }
        }

        if !got_fc {
            return;
        }

        // 3. Send Consecutive Frames.
        let mut offset = ff_payload; // bytes already sent in FF
        let mut seq: u8 = 1;
        while offset < resp_len {
            let remaining = resp_len - offset;
            let chunk = remaining.min(7);
            isotp_encode_cf(seq, &resp_data[offset..offset + chunk], &mut tx_buf);
            let _ = writeln!(uart, "  CF[{}]: offset={} chunk={}", seq, offset, chunk);
            unsafe { fdcan1_send(RESPONSE_ID, &tx_buf) };
            offset += chunk;
            seq = (seq + 1) & 0x0F; // wrap at 16

            // Brief delay between CFs (STmin=0 but give HW time)
            let cf_wait = timer.system_time_us_64() + 500;
            while timer.system_time_us_64() < cf_wait {}
        }
        let _ = writeln!(uart, "  Multi-frame TX complete: {} bytes in {} CFs",
                         resp_len, seq);
    }
}

/// Decode an incoming ISO-TP frame (SF or FF+CF multi-frame).
///
/// For multi-frame RX: sends FC(CTS) on RESPONSE_ID, collects CFs,
/// reassembles into `rx_buf`.  Returns the total payload length, or 0 on
/// failure.
///
/// # Safety
/// Calls `fdcan1_send` and `fdcan1_rx_fifo0_read` for FC/CF exchange.
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
            // Decode message length (12-bit).
            let msg_len = (((pci & 0x0F) as usize) << 8) | (frame_data[1] as usize);
            if msg_len == 0 || msg_len > rx_buf.len() {
                let _ = writeln!(uart, "  FF: msg_len={} too large, dropping", msg_len);
                return 0;
            }
            let first_chunk = (dlc - 2).min(6).min(msg_len);
            rx_buf[..first_chunk].copy_from_slice(&frame_data[2..2 + first_chunk]);
            let mut received = first_chunk;

            let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, first_chunk);

            // Send Flow Control: CTS, BS=0, STmin=0 — on the RESPONSE channel
            // because this is the server sending FC back to the tester.
            let mut fc_buf = [0u8; 8];
            isotp_encode_fc_cts(&mut fc_buf);
            unsafe { fdcan1_send(RESPONSE_ID, &fc_buf) };

            // Collect Consecutive Frames on REQUEST_ID.
            let mut expected_seq: u8 = 1;
            let cf_deadline = timer.system_time_us_64() + 1_000_000; // 1 s N_Cr timeout
            while received < msg_len {
                if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                    if f.id != REQUEST_ID {
                        continue;
                    }
                    let cf_pci = f.data[0];
                    if let Some(IsoTpFrameType::ConsecutiveFrame) = isotp_frame_type(cf_pci) {
                        let seq = cf_pci & 0x0F;
                        if seq != (expected_seq & 0x0F) {
                            let _ = writeln!(uart, "  CF: seq mismatch expected={} got={}",
                                             expected_seq & 0x0F, seq);
                            return 0;
                        }
                        let remaining = msg_len - received;
                        let chunk = remaining.min(7);
                        let cf_dlc = f.dlc.min(8) as usize;
                        let avail = (cf_dlc - 1).min(chunk);
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
/// | DID    | Description      | Value              |
/// |--------|------------------|--------------------|
/// | 0xF190 | VIN              | `TAKTFLOW_G474_01` |
/// | 0xF195 | SW version       | `BSP-0.1.0`        |
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
            let did = u16::from_be_bytes([did_hi, did_lo]);
            match did {
                0xF190 => {
                    // VIN — 16-char ASCII (multi-frame response: 19 bytes total)
                    response[0] = 0x62;
                    response[1] = 0xF1;
                    response[2] = 0x90;
                    let vin = b"TAKTFLOW_G474_01";
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
    let frame = match unsafe { fdcan1_rx_fifo0_read() } {
        Some(f) => f,
        None    => return false,
    };

    // Only process tester requests (0x600).  Discard any stray frames.
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

    // ISO-TP encode + send response.
    unsafe { isotp_send_response(&resp_buf, resp_len, timer, uart) };

    // Heartbeat blink: toggle LED on every successful exchange.
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
    let _ = writeln!(uart,
        "\r\n=== CAN server (real bus) starting @ {} MHz ===",
        sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    // ------------------------------------------------------------------
    // GPIOA clock is enabled by PolledUart::init(); configure PA5 here.
    unsafe {
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }

    // ------------------------------------------------------------------
    // 5. FDCAN1 — PA11/PA12 AF9, 500 kbps, real bus (no loopback)
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Initialising FDCAN1 (real bus, PA11=RX PA12=TX, 500 kbps)...");
    let init_ok = unsafe { fdcan1_init() };
    if !init_ok {
        let _ = writeln!(uart, "FAIL: FDCAN1 did not leave init mode");
        blink_forever(&mut timer, 200_000); // fast blink = fatal error
    }
    let _ = writeln!(uart,
        "FDCAN1 ready.  Listen=0x{:03X}  Reply=0x{:03X}",
        REQUEST_ID, RESPONSE_ID);
    let _ = writeln!(uart, "Waiting for UDS requests from tester...\r\n");
    uart.flush();

    // GPIO + Clock diagnostic — verify pin config and FDCAN clock source
    unsafe {
        let moder = reg_read(GPIOA_BASE, GPIO_MODER_OFFSET);
        let afrh  = reg_read(GPIOA_BASE, GPIO_AFRH_OFFSET);
        let ccipr = reg_read(RCC_BASE, RCC_CCIPR_OFFSET);
        let apb1enr1 = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET);
        // PA11 MODER bits [23:22], PA12 MODER bits [25:24] — expect 0b10 (AF)
        let pa11_mode = (moder >> 22) & 0x3;
        let pa12_mode = (moder >> 24) & 0x3;
        // AFRH: PA11 AF at bits [15:12], PA12 AF at bits [19:16] — expect 9
        let pa11_af = (afrh >> 12) & 0xF;
        let pa12_af = (afrh >> 16) & 0xF;
        let fdcan_clksel = (ccipr >> 24) & 0x3;
        let fdcan1en = (apb1enr1 >> 25) & 1;
        let _ = writeln!(uart, "GPIO: PA11=mode{}/AF{} PA12=mode{}/AF{}", pa11_mode, pa11_af, pa12_mode, pa12_af);
        let _ = writeln!(uart, "CLK: FDCANSEL={} (0=HSE,1=PLL_Q,2=PCLK1) FDCAN1EN={}", fdcan_clksel, fdcan1en);
    }

    // Diagnostic dump — FDCAN status after init
    unsafe {
        let cccr = reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET);
        let psr  = reg_read(FDCAN1_BASE, 0x044); // PSR
        let ecr  = reg_read(FDCAN1_BASE, 0x040); // ECR
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);
        let txbc = reg_read(FDCAN1_BASE, FDCAN_TXBC_OFFSET);
        let txfqs = reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET);
        let nbtp = reg_read(FDCAN1_BASE, FDCAN_NBTP_OFFSET);
        let _ = writeln!(uart, "DIAG: CCCR=0x{:08X} PSR=0x{:08X} ECR=0x{:08X}", cccr, psr, ecr);
        let _ = writeln!(uart, "DIAG: RXF0S=0x{:08X} TXBC=0x{:08X} TXFQS=0x{:08X} NBTP=0x{:08X}", rxf0s, txbc, txfqs, nbtp);
        let act = (psr >> 3) & 0x3;
        let ep = (psr >> 5) & 1;
        let bo = (psr >> 7) & 1;
        let lec = psr & 0x7;
        let _ = writeln!(uart, "DIAG: ACT={} EP={} BO={} LEC={}", act, ep, bo, lec);
    }

    // Solid LED = peripheral initialised successfully.
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, true) };

    // Periodic diagnostic — print FDCAN status every 5 seconds
    let mut last_diag = timer.system_time_us_64();

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

        // Periodic FDCAN diagnostic every 5 seconds
        let now = timer.system_time_us_64();
        if now - last_diag >= 5_000_000 {
            last_diag = now;
            unsafe {
                let psr = reg_read(FDCAN1_BASE, 0x044);
                let ecr = reg_read(FDCAN1_BASE, 0x040);
                let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);
                let ir = reg_read(FDCAN1_BASE, 0x050);
                let _ = writeln!(uart, "DIAG: PSR=0x{:08X} ECR=0x{:08X} RXF0S=0x{:08X} IR=0x{:08X}",
                    psr, ecr, rxf0s, ir);
            }
        }

        // Slow heartbeat blink (1 Hz) when idle; suppress during exchanges
        // (process_one_request handles blink there).
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
