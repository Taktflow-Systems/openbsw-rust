//! # bsw-can
//!
//! CAN bus abstraction for embedded BSW — Rust port of `OpenBSW` `cpp2can`.
//!
//! Provides CAN frame types, ID encoding, receive filters, and transceiver traits.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod can_id;
pub mod filter;
pub mod frame;
pub mod transceiver;

pub use can_id::CanId;
pub use filter::{BitFieldFilter, Filter, IntervalFilter};
pub use frame::{CanFrame, MAX_FRAME_LENGTH};
pub use transceiver::{
    AbstractTransceiver, CanTransceiver, ErrorCode, State, Statistics, TransceiverState,
};
