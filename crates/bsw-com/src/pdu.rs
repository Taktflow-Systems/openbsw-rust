// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! PDU (Protocol Data Unit) descriptor — maps a CAN frame to its signals.
//!
//! A [`PduDescriptor`] is a compile-time-constant struct that ties a CAN ID
//! to a set of signals, a direction (TX/RX), a payload length, and timing
//! configuration:
//!
//! - TX PDUs carry a [`TxMode`] (cyclic grid, event-only, or mixed) plus an
//!   optional minimum delay between event transmissions;
//! - RX PDUs carry an optional reception deadline plus the
//!   [`RxTimeoutAction`] applied when that deadline is missed.
//!
//! All timing values are [`bsw_time::Duration`]s — the COM layer has no raw
//! millisecond API (package E33).

use bsw_time::Duration;

// ---------------------------------------------------------------------------
// PduDirection
// ---------------------------------------------------------------------------

/// Transmission direction of a PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PduDirection {
    /// Transmitted by this node.
    Tx,
    /// Received from the bus.
    Rx,
}

// ---------------------------------------------------------------------------
// TxMode
// ---------------------------------------------------------------------------

/// Transmission mode of a TX PDU (AUTOSAR `ComTxModeMode`-style).
///
/// Only meaningful when the PDU direction is [`PduDirection::Tx`]; RX PDUs
/// ignore this field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxMode {
    /// Transmit on a fixed, drift-free grid with the given period.
    /// Event triggers ([`crate::com::ComManager::trigger`] and writes to
    /// [`TransferProperty::Triggered`](crate::signal::TransferProperty)
    /// signals) are ignored.
    Cyclic(Duration),
    /// Transmit only when triggered by an event; never fires cyclically.
    EventOnly,
    /// Cyclic grid with the given period, plus event-triggered
    /// transmissions between grid points.
    Mixed(Duration),
}

impl TxMode {
    /// The cyclic period, if this mode has one.
    #[must_use]
    pub const fn period(&self) -> Option<Duration> {
        match *self {
            Self::Cyclic(period) | Self::Mixed(period) => Some(period),
            Self::EventOnly => None,
        }
    }

    /// Whether this mode accepts event triggers.
    #[must_use]
    pub const fn accepts_events(&self) -> bool {
        matches!(self, Self::EventOnly | Self::Mixed(_))
    }
}

// ---------------------------------------------------------------------------
// RxTimeoutAction
// ---------------------------------------------------------------------------

/// Action applied to an RX PDU's shadow buffer when its reception deadline
/// is missed (AUTOSAR `ComRxDataTimeoutAction`-style).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxTimeoutAction {
    /// Keep the last received bytes; signal validity is unaffected.
    KeepLast,
    /// Re-apply every signal's init value to the shadow buffer and mark the
    /// PDU's signals invalid until the next reception.
    RestoreInit,
}

// ---------------------------------------------------------------------------
// PduDescriptor
// ---------------------------------------------------------------------------

/// PDU descriptor — maps a CAN ID to its signals and timing parameters.
///
/// All fields are `pub` to allow `const` table initialisation in application
/// code; the [`PduDescriptor::tx`] / [`PduDescriptor::rx`] constructors plus
/// the `with_*` builders cover the common cases.
#[derive(Debug, Clone, Copy)]
pub struct PduDescriptor {
    /// CAN ID for this PDU (11-bit base or 29-bit extended raw value).
    pub can_id: u32,
    /// PDU payload length in bytes (1–8 for classic CAN, 1–64 for CAN-FD).
    pub length: u8,
    /// TX or RX.
    pub direction: PduDirection,
    /// Transmission mode; meaningful for TX PDUs only.
    pub tx_mode: TxMode,
    /// Minimum delay between event transmissions of this PDU
    /// ([`Duration::ZERO`] = no throttling).  Measured from the PDU's most
    /// recent transmission (event or cyclic) to the next *event*
    /// transmission; cyclic grid transmissions are never delayed.
    /// Meaningful for TX PDUs only.
    pub min_event_interval: Duration,
    /// Reception deadline for RX PDUs.  `None` = not monitored (the
    /// documented default policy).  Meaningful for RX PDUs only.
    pub rx_timeout: Option<Duration>,
    /// Action applied when `rx_timeout` fires.  Meaningful for RX PDUs with
    /// a configured timeout only.
    pub timeout_action: RxTimeoutAction,
    /// Number of signals that belong to this PDU.
    pub signal_count: u8,
    /// Start index into the global [`SignalDescriptor`] table held by
    /// [`ComManager`](crate::com::ComManager).
    ///
    /// [`SignalDescriptor`]: crate::signal::SignalDescriptor
    pub signal_start_index: u16,
}

impl PduDescriptor {
    /// Create a TX PDU descriptor with the given transmission mode, no
    /// event throttling.
    #[must_use]
    pub const fn tx(can_id: u32, length: u8, tx_mode: TxMode) -> Self {
        Self {
            can_id,
            length,
            direction: PduDirection::Tx,
            tx_mode,
            min_event_interval: Duration::ZERO,
            rx_timeout: None,
            timeout_action: RxTimeoutAction::KeepLast,
            signal_count: 0,
            signal_start_index: 0,
        }
    }

    /// Create an RX PDU descriptor without deadline monitoring (default
    /// policy: `rx_timeout = None` means the PDU is not monitored).
    #[must_use]
    pub const fn rx(can_id: u32, length: u8) -> Self {
        Self {
            can_id,
            length,
            direction: PduDirection::Rx,
            tx_mode: TxMode::EventOnly,
            min_event_interval: Duration::ZERO,
            rx_timeout: None,
            timeout_action: RxTimeoutAction::KeepLast,
            signal_count: 0,
            signal_start_index: 0,
        }
    }

    /// Attach a minimum delay between event transmissions (builder style).
    #[must_use]
    pub const fn with_min_event_interval(mut self, interval: Duration) -> Self {
        self.min_event_interval = interval;
        self
    }

    /// Attach a reception deadline and timeout action (builder style).
    #[must_use]
    pub const fn with_rx_timeout(mut self, timeout: Duration, action: RxTimeoutAction) -> Self {
        self.rx_timeout = Some(timeout);
        self.timeout_action = action;
        self
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{PduDescriptor, PduDirection, RxTimeoutAction, TxMode};
    use bsw_time::Duration;

    const MS10: Duration = Duration::from_nanos(10_000_000);

    #[test]
    fn tx_pdu_descriptor_fields() {
        let pdu = PduDescriptor::tx(0x123, 8, TxMode::Cyclic(MS10));
        assert_eq!(pdu.can_id, 0x123);
        assert_eq!(pdu.length, 8);
        assert_eq!(pdu.direction, PduDirection::Tx);
        assert_eq!(pdu.tx_mode, TxMode::Cyclic(MS10));
        assert_eq!(pdu.min_event_interval, Duration::ZERO);
    }

    #[test]
    fn rx_pdu_descriptor_defaults_unmonitored() {
        let pdu = PduDescriptor::rx(0x456, 4);
        assert_eq!(pdu.direction, PduDirection::Rx);
        assert_eq!(pdu.rx_timeout, None);
    }

    #[test]
    fn builders_attach_timing_config() {
        let pdu = PduDescriptor::tx(0x100, 8, TxMode::Mixed(MS10)).with_min_event_interval(MS10);
        assert_eq!(pdu.min_event_interval, MS10);

        let pdu = PduDescriptor::rx(0x200, 8).with_rx_timeout(MS10, RxTimeoutAction::RestoreInit);
        assert_eq!(pdu.rx_timeout, Some(MS10));
        assert_eq!(pdu.timeout_action, RxTimeoutAction::RestoreInit);
    }

    #[test]
    fn tx_mode_accessors() {
        assert_eq!(TxMode::Cyclic(MS10).period(), Some(MS10));
        assert_eq!(TxMode::Mixed(MS10).period(), Some(MS10));
        assert_eq!(TxMode::EventOnly.period(), None);
        assert!(!TxMode::Cyclic(MS10).accepts_events());
        assert!(TxMode::EventOnly.accepts_events());
        assert!(TxMode::Mixed(MS10).accepts_events());
    }
}
