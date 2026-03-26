// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Interrupt-driven CAN RX — ISR building blocks and shared ring buffer.
//!
//! This module provides the hardware-reading primitives and a lock-free
//! `static mut` ring buffer for ISR → main-loop communication.  It is
//! intentionally **not** the ISR itself: the `#[interrupt]` handler lives in
//! the example binary, which owns the static transceiver and calls the
//! helpers here.
//!
//! # Design
//!
//! A `volatile`-accessed ring buffer is used instead of the SPSC queue from
//! `bsw-util` because `Queue::split()` requires `&mut Queue`, which cannot be
//! obtained from a `static` without a critical-section.  The ring buffer here
//! follows the classic single-producer / single-consumer lock-free pattern:
//!
//! - The ISR is the **sole writer** — it calls [`isr_push`] and advances `RX_HEAD`.
//! - The main loop is the **sole reader** — it calls [`main_pop`] and advances `RX_TAIL`.
//!
//! No mutex is needed because:
//! 1. Cortex-M is single-core → only one of ISR / main runs at any moment.
//! 2. `read_volatile` / `write_volatile` prevent the compiler from reordering
//!    or eliding index reads.
//!
//! # Usage sketch
//!
//! In the example binary:
//!
//! ```ignore
//! // Startup — enable hardware interrupt after peripheral is open.
//! #[cfg(feature = "stm32g474")]
//! unsafe { bsp::can_isr::fdcan1_enable_rx_interrupt(); }
//!
//! // ISR (in examples/can_rx_isr.rs):
//! #[interrupt]
//! fn FDCAN1_IT0() {
//!     if let Some(frame) = unsafe { bsp::can_isr::fdcan1_read_rx_fifo0() } {
//!         let _ = unsafe { bsp::can_isr::isr_push(frame) };
//!     }
//! }
//!
//! // Main loop:
//! while let Some(frame) = bsp::can_isr::main_pop() {
//!     process(frame);
//! }
//! ```
//!
//! # Safety contract
//!
//! - [`isr_push`] **must** be called exclusively from interrupt context.
//! - [`main_pop`] **must** be called exclusively from task/main context.
//! - The two contexts must never be the same execution level (i.e. never call
//!   both from main without disabling interrupts).
//!
//! # Reference
//!
//! - STM32G474 FDCAN: RM0440 Rev 8 §44 (register offsets are G4-specific).
//! - STM32F413 bxCAN: RM0430 Rev 9 §32.

use bsw_can::can_id::CanId;
use bsw_can::frame::CanFrame;

// ---------------------------------------------------------------------------
// Ring buffer constants
// ---------------------------------------------------------------------------

/// Backing-buffer size — one slot is sacrificed to distinguish full from empty,
/// so the usable capacity is `RX_QUEUE_SIZE - 1 = 31` frames.
pub const RX_QUEUE_SIZE: usize = 32;

// ---------------------------------------------------------------------------
// Ring buffer state (static mut — single-producer, single-consumer)
// ---------------------------------------------------------------------------

/// Frame backing store.
///
/// Written by the ISR at `RX_HEAD`, read by main at `RX_TAIL`.
/// Unoccupied slots contain uninitialised data; only the range
/// `[RX_TAIL, RX_HEAD)` (modulo `RX_QUEUE_SIZE`) is valid.
static mut RX_BUF: [CanFrame; RX_QUEUE_SIZE] = {
    const EMPTY: CanFrame = CanFrame::new();
    [EMPTY; RX_QUEUE_SIZE]
};

/// Write cursor — advanced by the ISR after a successful push.
static mut RX_HEAD: usize = 0;

/// Read cursor — advanced by main after a successful pop.
static mut RX_TAIL: usize = 0;

/// Count of frames dropped by the ISR because the ring buffer was full.
static mut RX_DROPPED: u32 = 0;

// ---------------------------------------------------------------------------
// Ring buffer API
// ---------------------------------------------------------------------------

/// Push a frame into the RX ring buffer.
///
/// Called **from ISR only**.  Returns `false` (and increments the drop counter)
/// when the buffer is full.
///
/// # Safety
///
/// Must be called exclusively from interrupt context.  No concurrent call to
/// [`main_pop`] must access the buffer at the same time — Cortex-M
/// single-core preemption rules guarantee this as long as [`main_pop`] is
/// only called from non-interrupt context.
#[inline]
pub unsafe fn isr_push(frame: CanFrame) -> bool {
    // SAFETY: volatile reads prevent the compiler from caching these indices.
    let head = unsafe { core::ptr::read_volatile(&RX_HEAD) };
    let next = (head + 1) % RX_QUEUE_SIZE;
    let tail = unsafe { core::ptr::read_volatile(&RX_TAIL) };

    if next == tail {
        // Buffer full — count and discard.
        // SAFETY: ISR is the sole writer of RX_DROPPED.
        let dropped = unsafe { core::ptr::read_volatile(&RX_DROPPED) };
        unsafe { core::ptr::write_volatile(&mut RX_DROPPED, dropped.wrapping_add(1)) };
        return false;
    }

    // Write the frame then advance the head.
    // SAFETY: `head` is a valid index; no other code writes this slot
    // until `RX_HEAD` advances past it.
    unsafe { RX_BUF[head] = frame };
    unsafe { core::ptr::write_volatile(&mut RX_HEAD, next) };
    true
}

/// Pop the oldest frame from the RX ring buffer.
///
/// Called **from main-loop / task context only**.
/// Returns `None` when the buffer is empty.
///
/// # Safety
///
/// Safe to call from any non-interrupt context.  The function itself is safe
/// (no `unsafe` qualifier) because reading the shared statics is guarded by
/// the single-core single-consumer guarantee documented on this module.
pub fn main_pop() -> Option<CanFrame> {
    // SAFETY: volatile reads prevent stale index caching across loop iterations.
    let tail = unsafe { core::ptr::read_volatile(&RX_TAIL) };
    let head = unsafe { core::ptr::read_volatile(&RX_HEAD) };

    if tail == head {
        return None; // empty
    }

    // SAFETY: `tail` is valid; ISR will not overwrite this slot until tail
    // advances past it after the write_volatile below.
    let frame = unsafe { RX_BUF[tail].clone() };
    unsafe { core::ptr::write_volatile(&mut RX_TAIL, (tail + 1) % RX_QUEUE_SIZE) };
    Some(frame)
}

/// Returns the number of frames currently in the ring buffer (best-effort snapshot).
///
/// May be momentarily imprecise if called while an ISR is mid-push.
pub fn queue_len() -> usize {
    let head = unsafe { core::ptr::read_volatile(&RX_HEAD) };
    let tail = unsafe { core::ptr::read_volatile(&RX_TAIL) };
    (head + RX_QUEUE_SIZE - tail) % RX_QUEUE_SIZE
}

/// Returns the cumulative count of frames dropped due to a full ring buffer.
pub fn rx_dropped_count() -> u32 {
    // SAFETY: read-only access; worst case a slightly stale count.
    unsafe { core::ptr::read_volatile(&RX_DROPPED) }
}

// ---------------------------------------------------------------------------
// MMIO helpers (shared between FDCAN and bxCAN helpers below)
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    // SAFETY: caller ensures the address is a valid MMIO register.
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    // SAFETY: caller ensures the address is a valid MMIO register.
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn mram_read(addr: usize) -> u32 {
    // SAFETY: caller ensures the address is inside FDCAN message RAM.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

// ---------------------------------------------------------------------------
// NVIC helpers
// ---------------------------------------------------------------------------

/// NVIC ISER base address (Interrupt Set-Enable Registers).
const NVIC_ISER_BASE: usize = 0xE000_E100;

/// Enable an interrupt in the NVIC by IRQ number.
///
/// Writes bit `(irq % 32)` in `NVIC_ISER[irq / 32]`.
///
/// # Safety
///
/// The IRQ number must be a valid peripheral interrupt for this MCU.
#[inline]
unsafe fn nvic_enable_irq(irq: u32) {
    let reg_idx = (irq / 32) as usize;
    let bit = 1u32 << (irq % 32);
    // SAFETY: NVIC ISER is always accessible from privileged mode.
    unsafe {
        core::ptr::write_volatile((NVIC_ISER_BASE + reg_idx * 4) as *mut u32, bit);
    }
}

// ---------------------------------------------------------------------------
// FDCAN1 (STM32G474RE) — hardware read + interrupt enable
// ---------------------------------------------------------------------------

/// FDCAN1 register block base (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// FDCAN message-RAM base for FDCAN1 (RM0440 §44.3).
const SRAMCAN_BASE: usize = 0x4000_A400;

// Register offsets (STM32G4-specific values — see can_fdcan.rs module doc)
const FDCAN_IE_OFFSET: usize = 0x054; // Interrupt enable
const FDCAN_ILS_OFFSET: usize = 0x058; // Interrupt line select
const FDCAN_ILE_OFFSET: usize = 0x05C; // Interrupt line enable
const FDCAN_IR_OFFSET: usize = 0x050;  // Interrupt register (W1C)
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status  (G4: 0x090)
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 acknowledge (G4: 0x094)

// IE/IR bit masks
const IE_RF0NE: u32 = 1 << 0; // RX FIFO 0 new-element interrupt enable
const IR_RF0N: u32 = 1 << 0;  // RX FIFO 0 new-element flag (W1C)

// RXF0S field masks (STM32G4: 4-bit fill-level, 2-bit get-index)
const RXF0S_F0FL_MASK: u32 = 0xF;       // [3:0] fill level
const RXF0S_F0GI_MASK: u32 = 0x3 << 8;  // [9:8] get index
const RXF0S_F0GI_SHIFT: u32 = 8;

// Message RAM: RX FIFO 0 offset and element stride (STM32G4: 18 words = 72 bytes)
const MRAM_RXF0_OFFSET: usize = 0x0B0;
const MRAM_ELEMENT_STRIDE: usize = 72; // 18 × 4 bytes

/// Read one CAN frame from the FDCAN1 RX FIFO 0 hardware registers.
///
/// Returns `None` if the FIFO is empty.  The element is acknowledged (slot
/// released back to the controller) before returning.
///
/// Call this from the `FDCAN1_IT0` interrupt handler immediately before
/// forwarding the frame to [`isr_push`].
///
/// # Safety
///
/// - Must be called on an STM32G474RE target.
/// - Must be called from FDCAN1 interrupt context (or with FDCAN1 interrupt
///   masked) to serialise FIFO access.
#[cfg(feature = "stm32g474")]
pub unsafe fn fdcan1_read_rx_fifo0() -> Option<CanFrame> {
    // SAFETY: FDCAN1_BASE + offsets are valid MMIO on STM32G474.
    let rxf0s = unsafe { reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET) };
    let fill = rxf0s & RXF0S_F0FL_MASK;
    if fill == 0 {
        // Acknowledge the interrupt flag even on empty read to avoid re-entry.
        unsafe { reg_write(FDCAN1_BASE, FDCAN_IR_OFFSET, IR_RF0N) };
        return None;
    }

    let get_idx = ((rxf0s & RXF0S_F0GI_MASK) >> RXF0S_F0GI_SHIFT) as usize;
    let base_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * MRAM_ELEMENT_STRIDE;

    // Read the four words of the RX FIFO element.
    // SAFETY: base_addr is inside FDCAN1 message RAM; get_idx < 3.
    let r0 = unsafe { mram_read(base_addr) };          // ID word
    let r1 = unsafe { mram_read(base_addr + 4) };      // DLC / flags
    let db0 = unsafe { mram_read(base_addr + 8) };     // data bytes 3..0
    let db1 = unsafe { mram_read(base_addr + 12) };    // data bytes 7..4

    // Acknowledge: release the FIFO slot back to the controller.
    // SAFETY: FDCAN1_BASE + RXF0A is a valid MMIO address.
    unsafe { reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32) };

    // Clear the RF0N interrupt flag (W1C).
    unsafe { reg_write(FDCAN1_BASE, FDCAN_IR_OFFSET, IR_RF0N) };

    // Decode R0: bit 30 = XTD (extended ID flag).
    // Extended: ID in bits [28:0]; Standard: ID in bits [28:18].
    let is_ext = (r0 & (1 << 30)) != 0;
    let raw_id = if is_ext {
        r0 & 0x1FFF_FFFF
    } else {
        (r0 >> 18) & 0x7FF
    };

    // Decode R1: DLC in bits [19:16], clamped to 8.
    let dlc = (((r1 >> 16) & 0xF) as u8).min(8) as usize;

    // Reassemble payload (little-endian word packing).
    let mut data = [0u8; 8];
    data[..4].copy_from_slice(&db0.to_le_bytes());
    data[4..8].copy_from_slice(&db1.to_le_bytes());

    let can_id = CanId::id(raw_id, is_ext);
    Some(CanFrame::with_data(can_id, &data[..dlc]))
}

/// Enable the FDCAN1 RX FIFO 0 new-element interrupt and unmask it in the NVIC.
///
/// Sets `FDCAN_IE.RF0NE`, routes all interrupts to line 0 (`ILS = 0`),
/// enables line 0 (`ILE.EINT0 = 1`), and enables `FDCAN1_IT0` in the NVIC
/// (IRQ 19 on STM32G474, RM0440 Table 91).
///
/// Call this once, after [`crate::can_fdcan::FdCanTransceiver::open`], from
/// startup code before enabling global interrupts.
///
/// # Safety
///
/// - Must be called on an STM32G474RE target.
/// - FDCAN1 must have been initialised (i.e. not in `Closed` state) before
///   calling this, otherwise IE writes have no effect.
/// - Must be called from privileged mode (startup / main context).
#[cfg(feature = "stm32g474")]
pub unsafe fn fdcan1_enable_rx_interrupt() {
    // SAFETY: FDCAN1_BASE offsets are valid on STM32G474.
    unsafe {
        // Enable RF0N interrupt source.
        reg_write(FDCAN1_BASE, FDCAN_IE_OFFSET, IE_RF0NE);
        // Route all enabled sources to interrupt line 0.
        reg_write(FDCAN1_BASE, FDCAN_ILS_OFFSET, 0x0000_0000);
        // Enable interrupt line 0.
        reg_write(FDCAN1_BASE, FDCAN_ILE_OFFSET, 0x0000_0001);
        // Enable FDCAN1_IT0 in NVIC (IRQ 19 — RM0440 Table 91 / Vector 35).
        nvic_enable_irq(19);
    }
}

// ---------------------------------------------------------------------------
// bxCAN1 (STM32F413ZH) — hardware read + interrupt enable
// ---------------------------------------------------------------------------

/// CAN1 register block base (RM0430 §32.9).
const CAN1_BASE: usize = 0x4000_6400;

// bxCAN register offsets
const CAN_IER_OFFSET: usize = 0x014; // Interrupt enable register
const CAN_RF0R_OFFSET: usize = 0x00C; // RX FIFO 0 register

// bxCAN FIFO 0 register bits
const RF0R_FMP0_MASK: u32 = 0x3;      // [1:0] FIFO message pending count
const RF0R_RFOM0: u32 = 1 << 5;       // Release FIFO 0 output mailbox

// bxCAN IER bits
const IER_FMPIE0: u32 = 1 << 1;       // FIFO 0 message-pending interrupt enable

// bxCAN RX mailbox 0 register offsets (relative to CAN1_BASE)
//   Each receive mailbox: RIR (id), RDTR (dlc), RDLR (data low), RDHR (data high)
const CAN_RX0_RIR_OFFSET: usize = 0x1B0;
const CAN_RX0_RDTR_OFFSET: usize = 0x1B4;
const CAN_RX0_RDLR_OFFSET: usize = 0x1B8;
const CAN_RX0_RDHR_OFFSET: usize = 0x1BC;

/// Read one CAN frame from the bxCAN1 RX FIFO 0 hardware mailbox.
///
/// Returns `None` if the FIFO is empty (FMP0 == 0).  Releases the mailbox
/// (RFOM0) before returning to make room for the next pending message.
///
/// Call this from the `CAN1_RX0` interrupt handler before forwarding to
/// [`isr_push`].
///
/// # Safety
///
/// - Must be called on an STM32F413ZH target.
/// - Must be called from `CAN1_RX0` interrupt context (or with interrupt
///   masked) to serialise FIFO access.
#[cfg(feature = "stm32f413")]
pub unsafe fn bxcan1_read_rx_fifo0() -> Option<CanFrame> {
    // SAFETY: CAN1_BASE + RF0R_OFFSET is a valid MMIO address on STM32F413.
    let rf0r = unsafe { reg_read(CAN1_BASE, CAN_RF0R_OFFSET) };
    if (rf0r & RF0R_FMP0_MASK) == 0 {
        return None; // FIFO empty
    }

    // Read the mailbox registers.
    // SAFETY: all offsets are valid bxCAN1 register addresses.
    let rir = unsafe { reg_read(CAN1_BASE, CAN_RX0_RIR_OFFSET) };
    let rdtr = unsafe { reg_read(CAN1_BASE, CAN_RX0_RDTR_OFFSET) };
    let rdlr = unsafe { reg_read(CAN1_BASE, CAN_RX0_RDLR_OFFSET) };
    let rdhr = unsafe { reg_read(CAN1_BASE, CAN_RX0_RDHR_OFFSET) };

    // Release the output mailbox so hardware can load the next queued message.
    // SAFETY: writing RF0R.RFOM0 releases FIFO0 output mailbox.
    unsafe {
        let rf0r_cur = reg_read(CAN1_BASE, CAN_RF0R_OFFSET);
        reg_write(CAN1_BASE, CAN_RF0R_OFFSET, rf0r_cur | RF0R_RFOM0);
    }

    // Decode RIR: bit 2 = IDE (extended ID flag).
    // Extended: EXID in bits [31:3] (29-bit value at [31:3]).
    // Standard: STID in bits [31:21] (11-bit value at [31:21]).
    let is_ext = (rir & (1 << 2)) != 0;
    let raw_id: u32 = if is_ext {
        rir >> 3  // bits [31:3] → 29-bit extended ID
    } else {
        rir >> 21 // bits [31:21] → 11-bit standard ID
    };

    // Decode RDTR: DLC in bits [3:0], clamped to 8.
    let dlc = ((rdtr & 0xF) as u8).min(8) as usize;

    // Reassemble payload from the two data registers (little-endian byte order).
    let data = [
        (rdlr & 0xFF) as u8,
        ((rdlr >> 8) & 0xFF) as u8,
        ((rdlr >> 16) & 0xFF) as u8,
        ((rdlr >> 24) & 0xFF) as u8,
        (rdhr & 0xFF) as u8,
        ((rdhr >> 8) & 0xFF) as u8,
        ((rdhr >> 16) & 0xFF) as u8,
        ((rdhr >> 24) & 0xFF) as u8,
    ];

    let can_id = CanId::id(raw_id, is_ext);
    Some(CanFrame::with_data(can_id, &data[..dlc]))
}

/// Enable the bxCAN1 RX FIFO 0 message-pending interrupt in the NVIC.
///
/// Sets `CAN_IER.FMPIE0` and enables `CAN1_RX0` in the NVIC
/// (IRQ 20 on STM32F413, RM0430 Table 62).
///
/// Call this once, after [`crate::can_bxcan::BxCanTransceiver::open`], from
/// startup code before enabling global interrupts.
///
/// # Safety
///
/// - Must be called on an STM32F413ZH target.
/// - CAN1 must be initialised (clock enabled) before calling this.
/// - Must be called from privileged mode.
#[cfg(feature = "stm32f413")]
pub unsafe fn bxcan1_enable_rx_interrupt() {
    // SAFETY: CAN1_BASE + IER_OFFSET is valid on STM32F413.
    unsafe {
        // Enable FIFO0 message-pending interrupt source.
        let ier = reg_read(CAN1_BASE, CAN_IER_OFFSET);
        reg_write(CAN1_BASE, CAN_IER_OFFSET, ier | IER_FMPIE0);
        // Enable CAN1_RX0 in NVIC (IRQ 20 — RM0430 Table 62 / Vector 36).
        nvic_enable_irq(20);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_can::can_id::CanId;
    use bsw_can::frame::CanFrame;

    /// Reset the ring buffer to a clean state between tests.
    ///
    /// # Safety
    ///
    /// Only safe in single-threaded test context.
    unsafe fn reset_queue() {
        unsafe {
            core::ptr::write_volatile(&mut RX_HEAD, 0);
            core::ptr::write_volatile(&mut RX_TAIL, 0);
            core::ptr::write_volatile(&mut RX_DROPPED, 0);
        }
    }

    fn make_frame(id: u32, data: &[u8]) -> CanFrame {
        CanFrame::with_data(CanId::base(id as u16), data)
    }

    // 1 — empty queue returns None on pop
    #[test]
    fn empty_queue_pop_returns_none() {
        unsafe { reset_queue() };
        assert_eq!(main_pop(), None);
    }

    // 2 — push then pop returns the same frame
    #[test]
    fn push_pop_round_trip() {
        unsafe { reset_queue() };
        let frame = make_frame(0x123, &[0xAA, 0xBB, 0xCC]);
        assert!(unsafe { isr_push(frame.clone()) });
        let popped = main_pop().expect("expected a frame");
        assert_eq!(popped, frame);
        assert_eq!(main_pop(), None);
    }

    // 3 — FIFO ordering preserved across multiple pushes
    #[test]
    fn fifo_order_preserved() {
        unsafe { reset_queue() };
        for i in 0..8u32 {
            let f = make_frame(i, &[i as u8]);
            assert!(unsafe { isr_push(f) });
        }
        for i in 0..8u32 {
            let f = main_pop().expect("expected frame");
            assert_eq!(f.id().raw_id(), i);
            assert_eq!(f.payload(), &[i as u8]);
        }
        assert_eq!(main_pop(), None);
    }

    // 4 — queue full: push to capacity succeeds, next push fails and increments drop counter
    #[test]
    fn full_queue_drops_and_counts() {
        unsafe { reset_queue() };
        // RX_QUEUE_SIZE - 1 = 31 usable slots.
        let capacity = RX_QUEUE_SIZE - 1;
        for i in 0..capacity {
            let f = make_frame(i as u32, &[0]);
            assert!(unsafe { isr_push(f) }, "push {i} should succeed");
        }
        assert_eq!(rx_dropped_count(), 0);
        // One more push must fail.
        let overflow = make_frame(0xFFF, &[0xFF]);
        assert!(!unsafe { isr_push(overflow) });
        assert_eq!(rx_dropped_count(), 1);
    }

    // 5 — queue_len reports correct occupancy
    #[test]
    fn queue_len_tracks_occupancy() {
        unsafe { reset_queue() };
        assert_eq!(queue_len(), 0);
        unsafe { isr_push(make_frame(1, &[])) };
        assert_eq!(queue_len(), 1);
        unsafe { isr_push(make_frame(2, &[])) };
        assert_eq!(queue_len(), 2);
        main_pop();
        assert_eq!(queue_len(), 1);
        main_pop();
        assert_eq!(queue_len(), 0);
    }

    // 6 — wrap-around: fill, drain, fill again
    #[test]
    fn wrap_around() {
        unsafe { reset_queue() };
        let capacity = RX_QUEUE_SIZE - 1;
        // Fill
        for i in 0..capacity {
            assert!(unsafe { isr_push(make_frame(i as u32, &[])) });
        }
        // Drain
        for _ in 0..capacity {
            assert!(main_pop().is_some());
        }
        assert_eq!(queue_len(), 0);
        // Fill again (indices have wrapped)
        for i in 0..capacity {
            assert!(unsafe { isr_push(make_frame(i as u32, &[])) });
        }
        assert_eq!(queue_len(), capacity);
        for _ in 0..capacity {
            assert!(main_pop().is_some());
        }
        assert_eq!(queue_len(), 0);
    }

    // 7 — extended-ID frame round-trips correctly
    #[test]
    fn extended_id_round_trip() {
        unsafe { reset_queue() };
        let frame = CanFrame::with_data(CanId::id(0x1234_5678, true), &[0xDE, 0xAD]);
        assert!(unsafe { isr_push(frame.clone()) });
        let out = main_pop().unwrap();
        assert!(out.id().is_extended());
        assert_eq!(out.id().raw_id(), 0x1234_5678);
        assert_eq!(out.payload(), &[0xDE, 0xAD]);
    }

    // 8 — rx_dropped_count wraps gracefully on overflow
    #[test]
    fn dropped_count_increments() {
        unsafe { reset_queue() };
        // Fill the buffer completely.
        for i in 0..(RX_QUEUE_SIZE - 1) {
            let _ = unsafe { isr_push(make_frame(i as u32, &[])) };
        }
        // Force 5 drops.
        for _ in 0..5 {
            let _ = unsafe { isr_push(make_frame(0, &[])) };
        }
        assert_eq!(rx_dropped_count(), 5);
    }
}
