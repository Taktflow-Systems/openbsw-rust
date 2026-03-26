//! # bsw-docan
//!
//! ISO-TP (ISO 15765) transport protocol for CAN — Rust port of `OpenBSW` `docan`.
//!
//! Provides frame codec, protocol state machines, and addressing for segmented CAN messaging.
//!
//! ## Design
//!
//! - `no_std` compatible — uses only `core`, no heap allocations.
//! - All protocol logic is pure (no I/O, no timers, no callbacks).
//! - State machines return transition structs describing required side-effects.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod addressing;
pub mod codec;
pub mod constants;
pub mod parameters;
pub mod rx_handler;
pub mod tx_handler;

pub use addressing::{ConnectionInfo, DataLinkAddressPair, TransportAddressPair};
pub use codec::{
    CodecConfig, ConsecutiveFrame, DecodedFrame, FirstFrame, FlowControlFrame, SingleFrame,
};
pub use constants::{CodecResult, FlowStatus, FrameType, ProtocolMessage, SendResult};
pub use parameters::{decode_separation_time, encode_separation_time, Parameters};
pub use rx_handler::{RxProtocolHandler, RxState, RxTimeout, RxTransition};
pub use tx_handler::{TxActions, TxProtocolHandler, TxState, TxTimeout, TxTransition};
