//! # bsw-com
//!
//! AUTOSAR-style COM layer — signal packing/unpacking for CAN PDUs.
//!
//! Provides the bridge between application-level signal values (typed, named)
//! and raw CAN frame bytes (packed PDUs), plus TX scheduling (cyclic /
//! event / mixed transmission modes), RX deadline monitoring, per-signal
//! update and invalid flags, and polled error reporting (package E33).
//!
//! ## Architecture
//!
//! ```text
//! Application (SWCs)
//!     ↓ write_signal() / read_signal() / take_updated()
//! COM Layer  (this crate)
//!     ↓ pack into PDU / unpack from PDU, tick() scheduling
//! CAN Transceiver  (bsw-can)
//!     ↓ CanFrame::with_data()
//! Hardware
//! ```
//!
//! Time is injected: the manager consumes [`bsw_time::Instant`] values and
//! never reads a clock itself, so the whole layer is deterministic under a
//! fake clock.  See [`com`] for the timing model and polling contract.
//!
//! ## Quick start
//!
//! ```rust
//! use bsw_com::com::ComManager;
//! use bsw_com::pdu::{PduDescriptor, TxMode};
//! use bsw_com::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
//! use bsw_time::{Duration, Instant};
//!
//! const PERIOD: Duration = Duration::from_nanos(10_000_000); // 10 ms
//!
//! // 1. Create a manager with capacity for 4 PDUs and 16 signals.
//! let mut com: ComManager<4, 16> = ComManager::new();
//!
//! // 2. Register a cyclic TX PDU (10 ms period) with two signals.
//! let signals = [
//!     SignalDescriptor::new(1, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0),
//!     SignalDescriptor::new(2, 8, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0),
//! ];
//! com.add_pdu(
//!     PduDescriptor::tx(0x123, 8, TxMode::Cyclic(PERIOD)),
//!     &signals,
//! );
//!
//! // 3. Start scheduling — the cyclic grid is anchored here.
//! com.start(Instant::from_nanos(0));
//!
//! // 4. Application writes a signal.
//! com.write_signal(1, SignalValue::U8(0x42));
//! com.write_signal(2, SignalValue::U16(0x1234));
//!
//! // 5. Periodic tick — collect frames ready for TX.
//! for (can_id, data) in com.tick(Instant::from_nanos(0)) {
//!     // hand (can_id, data) to the CAN transceiver …
//!     let _ = (can_id, data);
//! }
//!
//! // 6. Poll for errors (RX deadline misses, TX overruns, …).
//! assert!(com.take_event().is_none());
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod com;
pub mod packer;
pub mod pdu;
pub mod signal;

pub use com::{ComE2eError, ComEvent, ComManager, ComTxIterator, EVENT_QUEUE_CAP, PDU_BUFFER_LEN};
pub use packer::{
    pack_physical, pack_signal, unpack_physical, unpack_signal, PackError, MAX_BYTE_ARRAY_LEN,
};
pub use pdu::{PduDescriptor, PduDirection, RxTimeoutAction, TxMode};
pub use signal::{ByteOrder, Scaling, SignalDescriptor, SignalType, SignalValue, TransferProperty};
