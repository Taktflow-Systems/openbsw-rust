// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! bxCAN transceiver for STM32F413 — implements [`CanTransceiver`].
//!
//! # Hardware resources
//!
//! | Resource       | Assignment                          |
//! |----------------|-------------------------------------|
//! | Peripheral     | CAN1                                |
//! | TX pin         | PD1 (AF9)                           |
//! | RX pin         | PD0 (AF9)                           |
//! | APB1 clock     | 48 MHz (configured by `clock_f4`)   |
//! | Baudrate       | 500 kbit/s                          |
//!
//! # Bit timing @ 48 MHz APB1 → 500 kbit/s
//!
//! | Parameter  | Value | Notes                             |
//! |------------|-------|-----------------------------------|
//! | Prescaler  | 6     | TQ = 6/48 MHz = 125 ns            |
//! | BS1        | 11    | seg1 = 11 TQ                      |
//! | BS2        | 4     | seg2 = 4 TQ                       |
//! | SJW        | 1     | re-sync jump width = 1 TQ         |
//! | Bit time   | 16 TQ | 1 + 11 + 4                        |
//! | Baudrate   | 48 MHz / (6 × 16) = 500 kbit/s           |
//!
//! # State machine
//!
//! ```text
//! Closed ──init()──► Initialized ──open()──► Open ⇌ mute()/unmute() ⇌ Muted
//!   ▲                    │                     │
//!   └────────shutdown()──┘─────────close()─────┘
//! ```
//!
//! # RX buffering
//!
//! The bxCAN hardware provides a 3-deep FIFO0.  The ISR drains it into a
//! [`RX_QUEUE_SIZE`]-entry software circular buffer.  [`receive_isr`] must be
//! called from the `USB_LP_CAN1_RX0` (or `CAN1_RX0`) interrupt handler.

use bsw_can::transceiver::RX_QUEUE_SIZE;
use bsw_can::{AbstractTransceiver, CanFrame, CanId, CanTransceiver, ErrorCode, State, TransceiverState};
use stm32f4xx_hal::pac;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Baud rate supported by this driver.
const BAUDRATE_500K: u32 = 500_000;

/// Transmit queue timeout returned to callers (milliseconds).
const HW_QUEUE_TIMEOUT_MS: u16 = 10;

/// Bit timing prescaler for 500 kbit/s @ APB1 48 MHz.
/// TQ = PSC / F_APB1 = 6 / 48e6 = 125 ns.
const BIT_TIMING_PSC: u16 = 6;

/// Time segment 1 (11 TQ — encoded as value minus 1 in the register).
const BIT_TIMING_BS1: u8 = 11;

/// Time segment 2 (4 TQ — encoded as value minus 1 in the register).
const BIT_TIMING_BS2: u8 = 4;

/// Re-synchronisation jump width (1 TQ — encoded as value minus 1).
const BIT_TIMING_SJW: u8 = 1;

// ---------------------------------------------------------------------------
// BxCanTransceiver
// ---------------------------------------------------------------------------

/// bxCAN hardware driver for STM32F413ZH implementing [`CanTransceiver`].
///
/// Embed one instance in your application and call [`init`](Self::init) once
/// at startup, then [`open`](Self::open) to join the bus.
pub struct BxCanTransceiver {
    /// Abstract base: state machine, statistics, filter, bus identity.
    base: AbstractTransceiver<8>,
    /// Software receive queue (circular buffer, interrupt-filled).
    rx_queue: [CanFrame; RX_QUEUE_SIZE],
    /// Index of the oldest unconsumed frame.
    rx_head: usize,
    /// Number of frames currently in the queue.
    rx_count: usize,
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

impl BxCanTransceiver {
    /// Creates a new, uninitialised `BxCanTransceiver` for the given bus index.
    ///
    /// Call [`init`](Self::init) before any other operation.
    pub const fn new(bus_id: u8) -> Self {
        // CanFrame does not implement Copy, so we must use a const initialiser.
        const EMPTY_FRAME: CanFrame = CanFrame::new();
        Self {
            base: AbstractTransceiver::new(bus_id),
            rx_queue: [EMPTY_FRAME; RX_QUEUE_SIZE],
            rx_head: 0,
            rx_count: 0,
        }
    }

    // -----------------------------------------------------------------------
    // ISR entry points
    // -----------------------------------------------------------------------

    /// Drain the bxCAN FIFO0 into the software receive queue.
    ///
    /// Call this from the `CAN1_RX0` interrupt handler.
    ///
    /// # Safety
    ///
    /// The caller must ensure that this is called exclusively from an interrupt
    /// context and that no other code concurrently mutates the RX queue.
    pub unsafe fn receive_isr(&mut self) {
        // SAFETY: single-owner PAC access from ISR context.
        let can = unsafe { &*pac::CAN1::ptr() };

        // FMP0 holds the number of pending messages in FIFO0 (0–3).
        while can.rf0r().read().fmp().bits() > 0 {
            // Read the three receive registers for mailbox 0 (RDT0R, RDL0R, RDH0R).
            let rir = can.rx(0).rir().read();
            let rdtr = can.rx(0).rdtr().read();
            let rdlr = can.rx(0).rdlr().read();
            let rdhr = can.rx(0).rdhr().read();

            let dlc = (rdtr.dlc().bits() as u8).min(8);
            let ide = rir.ide().bit_is_set();

            let raw_id = if ide {
                // Extended frame: bits [28:3] of RIR hold the 29-bit EXID.
                rir.exid().bits()
            } else {
                // Base frame: bits [31:21] of RIR hold the 11-bit STID.
                u32::from(rir.stid().bits())
            };

            // Pack payload from the two 32-bit data registers (little-endian).
            let mut data = [0u8; 8];
            let lo = rdlr.bits();
            let hi = rdhr.bits();
            data[0] = (lo & 0xFF) as u8;
            data[1] = ((lo >> 8) & 0xFF) as u8;
            data[2] = ((lo >> 16) & 0xFF) as u8;
            data[3] = ((lo >> 24) & 0xFF) as u8;
            data[4] = (hi & 0xFF) as u8;
            data[5] = ((hi >> 8) & 0xFF) as u8;
            data[6] = ((hi >> 16) & 0xFF) as u8;
            data[7] = ((hi >> 24) & 0xFF) as u8;

            // Release the mailbox so the hardware can accept the next message.
            // SAFETY: writing RFOM0 releases the FIFO0 output mailbox.
            can.rf0r().modify(|_, w| w.rfom().release());

            // Store in the software circular buffer.
            if self.rx_count < RX_QUEUE_SIZE {
                let tail = (self.rx_head + self.rx_count) % RX_QUEUE_SIZE;
                let frame = &mut self.rx_queue[tail];
                frame.set_id(CanId::id(raw_id, ide));
                frame.set_payload(&data[..dlc as usize]);
                self.rx_count += 1;
                self.base.statistics_mut().rx += 1;
            } else {
                // Queue full — drop frame and count it.
                self.base.statistics_mut().rx_dropped += 1;
            }
        }
    }

    /// TX-complete ISR entry point — updates TX statistics.
    ///
    /// Call this from the `CAN1_TX` interrupt handler.
    ///
    /// # Safety
    ///
    /// Must be called exclusively from interrupt context.
    pub unsafe fn transmit_isr(&mut self) {
        // SAFETY: single-owner PAC access from ISR context.
        let can = unsafe { &*pac::CAN1::ptr() };
        let tsr = can.tsr().read();

        // Check RQCP (Request Complete) bits for all three mailboxes.
        if tsr.rqcp0().bit_is_set() {
            if tsr.txok0().bit_is_set() {
                self.base.statistics_mut().tx += 1;
            } else {
                self.base.statistics_mut().errors += 1;
            }
            // Clear RQCP0 by writing 1.
            can.tsr().modify(|_, w| w.rqcp0().set_bit());
        }
        if tsr.rqcp1().bit_is_set() {
            if tsr.txok1().bit_is_set() {
                self.base.statistics_mut().tx += 1;
            } else {
                self.base.statistics_mut().errors += 1;
            }
            can.tsr().modify(|_, w| w.rqcp1().set_bit());
        }
        if tsr.rqcp2().bit_is_set() {
            if tsr.txok2().bit_is_set() {
                self.base.statistics_mut().tx += 1;
            } else {
                self.base.statistics_mut().errors += 1;
            }
            can.tsr().modify(|_, w| w.rqcp2().set_bit());
        }
    }

    /// Dequeue the next received frame, if any.
    ///
    /// Returns `Some(frame)` or `None` when the queue is empty.
    /// Call from task context (not ISR) with interrupts disabled around the
    /// head/count manipulation, or use [`InterruptLock`] from `bsw-can`.
    pub fn dequeue_frame(&mut self) -> Option<CanFrame> {
        if self.rx_count == 0 {
            return None;
        }
        let frame = self.rx_queue[self.rx_head].clone();
        self.rx_head = (self.rx_head + 1) % RX_QUEUE_SIZE;
        self.rx_count -= 1;
        Some(frame)
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Enable CAN1 peripheral clock via RCC APB1ENR.
    ///
    /// # Safety
    ///
    /// Must only be called from [`init`](Self::init) before the peripheral is used.
    unsafe fn enable_can_clock() {
        // SAFETY: startup-only write; RCC is always accessible.
        let rcc = unsafe { &*pac::RCC::ptr() };
        rcc.apb1enr().modify(|_, w| w.can1en().enabled());
        // Short read-back to flush the AHB/APB bridge write buffer.
        let _ = rcc.apb1enr().read();
    }

    /// Configure PD0/PD1 as CAN1 RX/TX with AF9, push-pull, high-speed.
    ///
    /// # Safety
    ///
    /// Must be called before enabling the CAN peripheral.
    unsafe fn configure_gpio() {
        // SAFETY: startup-only GPIO configuration; no concurrent access.
        let rcc = unsafe { &*pac::RCC::ptr() };
        let gpiod = unsafe { &*pac::GPIOD::ptr() };

        // Enable GPIOD clock.
        rcc.ahb1enr().modify(|_, w| w.gpioden().enabled());
        let _ = rcc.ahb1enr().read();

        // PD0 = CAN1_RX, PD1 = CAN1_TX — alternate function mode (MODER = 0b10).
        gpiod.moder().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 | (0b11 << 2))) | (0b10 | (0b10 << 2)))
        });

        // Output type: push-pull (reset default, OTYPER bits 0 and 1 = 0).
        gpiod
            .otyper()
            .modify(|r, w| unsafe { w.bits(r.bits() & !0b11) });

        // Output speed: high (OSPEEDR[1:0] = 0b10, OSPEEDR[3:2] = 0b10).
        gpiod.ospeedr().modify(|r, w| unsafe {
            w.bits((r.bits() & !(0b11 | (0b11 << 2))) | (0b10 | (0b10 << 2)))
        });

        // No pull-up/pull-down (PUPDR bits = 0).
        gpiod
            .pupdr()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 | (0b11 << 2))) });

        // Alternate function AF9 for PD0 and PD1 (AFRL[3:0] and AFRL[7:4]).
        gpiod.afrl().modify(|r, w| unsafe {
            w.bits((r.bits() & !0xFF) | 0x99) // AF9 = 0x9
        });
    }

    /// Enter bxCAN initialisation mode and wait for acknowledgement.
    ///
    /// # Safety
    ///
    /// CAN1 clock must be enabled before calling this.
    unsafe fn enter_init_mode(can: &pac::can1::RegisterBlock) {
        // Exit sleep mode (SLEEP bit) and request init mode (INRQ bit).
        can.mcr().modify(|_, w| w.sleep().clear_bit().inrq().set_bit());
        // Wait for INAK in MSR.
        while can.msr().read().inak().bit_is_clear() {}
    }

    /// Leave initialisation mode (enter normal mode).
    ///
    /// # Safety
    ///
    /// Bit timing and filters must be fully configured before calling this.
    unsafe fn leave_init_mode(can: &pac::can1::RegisterBlock) {
        can.mcr().modify(|_, w| w.inrq().clear_bit());
        while can.msr().read().inak().bit_is_set() {}
    }

    /// Program bit timing register for 500 kbit/s @ 48 MHz APB1.
    ///
    /// # Safety
    ///
    /// Must be called while the peripheral is in initialisation mode.
    unsafe fn set_bit_timing(can: &pac::can1::RegisterBlock) {
        // BTR register layout:
        //   [9:0]   BRP   — baud rate prescaler (value = prescaler - 1)
        //   [19:16] TS1   — time segment 1 (value = bs1 - 1)
        //   [22:20] TS2   — time segment 2 (value = bs2 - 1)
        //   [25:24] SJW   — sync jump width  (value = sjw - 1)
        //   [30]    LBKM  — loopback mode
        //   [31]    SILM  — silent mode
        let brp = (BIT_TIMING_PSC - 1) as u32;
        let ts1 = (BIT_TIMING_BS1 - 1) as u32;
        let ts2 = (BIT_TIMING_BS2 - 1) as u32;
        let sjw = (BIT_TIMING_SJW - 1) as u32;
        let btr_val = brp | (ts1 << 16) | (ts2 << 20) | (sjw << 24);
        // SAFETY: all fields are within their documented bit-widths.
        can.btr().write(|w| unsafe { w.bits(btr_val) });
    }

    /// Configure filter bank 0 in mask mode to accept all frames on FIFO0.
    ///
    /// # Safety
    ///
    /// Must be called in initialisation mode (FINIT bit is set automatically
    /// during the filter init window).
    unsafe fn configure_accept_all_filter(can: &pac::can1::RegisterBlock) {
        // Enter filter initialisation mode.
        can.fmr().modify(|_, w| w.finit().set_bit());

        // Deactivate filter bank 0 before modifying it.
        can.fa1r().modify(|r, w| unsafe { w.bits(r.bits() & !1) });

        // 32-bit mask mode (FBM0 = 0 → mask mode; FSC0 = 1 → 32-bit scale).
        can.fm1r().modify(|r, w| unsafe { w.bits(r.bits() & !1) }); // mask mode
        can.fs1r().modify(|r, w| unsafe { w.bits(r.bits() | 1) }); // 32-bit

        // ID register = 0, mask register = 0 → accept everything.
        can.fb(0).fr1().write(|w| unsafe { w.bits(0) });
        can.fb(0).fr2().write(|w| unsafe { w.bits(0) });

        // Assign filter bank 0 to FIFO0 (FFA1R bit 0 = 0).
        can.ffa1r().modify(|r, w| unsafe { w.bits(r.bits() & !1) });

        // Activate filter bank 0.
        can.fa1r().modify(|r, w| unsafe { w.bits(r.bits() | 1) });

        // Leave filter initialisation mode.
        can.fmr().modify(|_, w| w.finit().clear_bit());
    }

    /// Find the first empty TX mailbox.  Returns `Some(0..=2)` or `None`.
    fn find_empty_mailbox(can: &pac::can1::RegisterBlock) -> Option<usize> {
        let tsr = can.tsr().read();
        if tsr.tme0().bit_is_set() {
            Some(0)
        } else if tsr.tme1().bit_is_set() {
            Some(1)
        } else if tsr.tme2().bit_is_set() {
            Some(2)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// CanTransceiver implementation
// ---------------------------------------------------------------------------

impl CanTransceiver for BxCanTransceiver {
    /// Initialise the bxCAN peripheral.
    ///
    /// Enables the RCC clock, configures GPIO pins, programs the bit timing
    /// registers, and sets up an accept-all receive filter on FIFO0.
    /// Transitions the state machine `Closed → Initialized`.
    fn init(&mut self) -> ErrorCode {
        if !self.base.is_in_state(State::Closed) {
            return ErrorCode::IllegalState;
        }

        // SAFETY: startup sequence; called once before any concurrent access.
        unsafe {
            Self::enable_can_clock();
            Self::configure_gpio();
        }

        // SAFETY: CAN1 clock is now enabled.
        let can = unsafe { &*pac::CAN1::ptr() };

        unsafe {
            Self::enter_init_mode(can);
            Self::set_bit_timing(can);
            Self::configure_accept_all_filter(can);
            // Disable automatic bus-off recovery and automatic wakeup.
            can.mcr().modify(|_, w| w.abom().clear_bit().awum().clear_bit());
        }

        self.base.transition_to_initialized()
    }

    /// Shut down the bxCAN peripheral and reset the state machine to `Closed`.
    fn shutdown(&mut self) {
        // SAFETY: always safe to enter sleep mode.
        let can = unsafe { &*pac::CAN1::ptr() };
        can.mcr().modify(|_, w| w.sleep().set_bit().inrq().clear_bit());
        self.base.transition_to_closed();
    }

    /// Open the bus — leave init mode, enable FIFO0/TX interrupts.
    ///
    /// Transitions `Initialized → Open`.
    fn open(&mut self) -> ErrorCode {
        if !self.base.is_in_state(State::Initialized) {
            return ErrorCode::IllegalState;
        }

        let can = unsafe { &*pac::CAN1::ptr() };
        unsafe {
            // Clear silent/loopback bits to go on the live bus.
            can.btr().modify(|r, w| w.bits(r.bits() & !(1 << 30 | 1 << 31)));
            Self::leave_init_mode(can);
            // Enable FIFO0 message-pending interrupt (FMPIE0) and TX mailbox
            // empty interrupt (TMEIE).
            can.ier().modify(|_, w| w.fmpie0().set_bit().tmeie().set_bit());
        }

        self.base.transition_to_open()
    }

    /// Open the bus and immediately enqueue `frame` for transmission.
    fn open_with_frame(&mut self, frame: &CanFrame) -> ErrorCode {
        let result = self.open();
        if result != ErrorCode::Ok {
            return result;
        }
        self.write(frame)
    }

    /// Close the bus — re-enter init mode, disable interrupts.
    ///
    /// Transitions any state → `Closed`.
    fn close(&mut self) -> ErrorCode {
        let can = unsafe { &*pac::CAN1::ptr() };
        unsafe {
            can.ier().modify(|_, w| w.fmpie0().clear_bit().tmeie().clear_bit());
            Self::enter_init_mode(can);
        }
        self.base.transition_to_closed()
    }

    /// Mute — enable silent (listen-only) mode; TX is suppressed by hardware.
    ///
    /// Transitions `Open → Muted`.
    fn mute(&mut self) -> ErrorCode {
        if !self.base.is_in_state(State::Open) {
            return ErrorCode::IllegalState;
        }

        let can = unsafe { &*pac::CAN1::ptr() };
        // Must enter init mode to modify BTR.SILM.
        unsafe {
            Self::enter_init_mode(can);
            can.btr().modify(|r, w| w.bits(r.bits() | (1 << 31)));
            Self::leave_init_mode(can);
        }

        self.base.transition_to_muted()
    }

    /// Unmute — clear silent mode, resume transmission.
    ///
    /// Transitions `Muted → Open`.
    fn unmute(&mut self) -> ErrorCode {
        if !self.base.is_in_state(State::Muted) {
            return ErrorCode::IllegalState;
        }

        let can = unsafe { &*pac::CAN1::ptr() };
        unsafe {
            Self::enter_init_mode(can);
            can.btr().modify(|r, w| w.bits(r.bits() & !(1 << 31)));
            Self::leave_init_mode(can);
        }

        self.base.transition_to_open()
    }

    #[inline]
    fn state(&self) -> State {
        self.base.state()
    }

    #[inline]
    fn baudrate(&self) -> u32 {
        BAUDRATE_500K
    }

    #[inline]
    fn hw_queue_timeout(&self) -> u16 {
        HW_QUEUE_TIMEOUT_MS
    }

    #[inline]
    fn bus_id(&self) -> u8 {
        self.base.bus_id()
    }

    /// Write a frame to the first available TX mailbox.
    ///
    /// Returns [`ErrorCode::TxHwQueueFull`] when all three hardware mailboxes
    /// are occupied, [`ErrorCode::TxOffline`] when the transceiver is not Open
    /// or Muted.
    fn write(&mut self, frame: &CanFrame) -> ErrorCode {
        match self.base.state() {
            State::Open => {}
            State::Muted => return ErrorCode::TxOffline,
            _ => return ErrorCode::TxOffline,
        }

        let can = unsafe { &*pac::CAN1::ptr() };

        let mb = match Self::find_empty_mailbox(can) {
            Some(i) => i,
            None => {
                self.base.statistics_mut().tx_dropped += 1;
                return ErrorCode::TxHwQueueFull;
            }
        };

        let id = frame.id();
        let dlc = frame.payload_length().min(8);
        let data = frame.payload();

        // Pack up to 8 payload bytes into two 32-bit words.
        let get = |i: usize| if i < data.len() { data[i] } else { 0 };
        let tdlr: u32 = u32::from(get(0))
            | (u32::from(get(1)) << 8)
            | (u32::from(get(2)) << 16)
            | (u32::from(get(3)) << 24);
        let tdhr: u32 = u32::from(get(4))
            | (u32::from(get(5)) << 8)
            | (u32::from(get(6)) << 16)
            | (u32::from(get(7)) << 24);

        // Build TIR value — IDE bit[2], RTR bit[1], TXRQ bit[0].
        let tir: u32 = if id.is_extended() {
            // Extended ID occupies bits [31:3]; IDE = 1.
            (id.raw_id() << 3) | (1 << 2)
        } else {
            // Base ID occupies bits [31:21].
            u32::from(id.raw_id() as u16) << 21
        };

        // SAFETY: single-writer to a free mailbox selected by TMEx bits in TSR.
        unsafe {
            can.tx(mb).tdtr().write(|w| w.dlc().bits(dlc));
            can.tx(mb).tdlr().write(|w| w.bits(tdlr));
            can.tx(mb).tdhr().write(|w| w.bits(tdhr));
            // Set TXRQ (bit 0) last to arm transmission.
            can.tx(mb).tir().write(|w| w.bits(tir | 1));
        }

        ErrorCode::Ok
    }

    /// Query the low-level hardware bus state from the CAN ESR register.
    fn transceiver_state(&self) -> TransceiverState {
        let can = unsafe { &*pac::CAN1::ptr() };
        let esr = can.esr().read();
        if esr.boff().bit_is_set() {
            TransceiverState::BusOff
        } else if esr.epvf().bit_is_set() {
            TransceiverState::Passive
        } else {
            TransceiverState::Active
        }
    }
}

// Implement CanReceiver for DiagCanTransport integration.
impl crate::diag_can::CanReceiver for BxCanTransceiver {
    fn receive(&mut self) -> Option<bsw_can::frame::CanFrame> {
        self.dequeue_frame()
    }
}
