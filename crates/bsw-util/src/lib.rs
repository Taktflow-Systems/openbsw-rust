//! # bsw-util
//!
//! Utility modules for embedded BSW — Rust port of OpenBSW `util`.
//!
//! Only modules with no direct Rust core equivalent are implemented here.
//! Types like `ConstString` → `&str`, `Defer` → `Drop`, `Enum` → native enum,
//! format/printf → `core::fmt` are intentionally omitted.
//!
//! ## Provided modules
//!
//! - [`crc`] — CRC calculator with lookup tables (CRC-8, CRC-16, CRC-32)
//! - [`spsc`] — Lock-free single-producer single-consumer queue
//! - [`buddy`] — Buddy memory manager (binary-tree allocator)
//! - [`moving_average`] — Fixed-window moving average filter

#![cfg_attr(not(feature = "std"), no_std)]

pub mod buddy;
pub mod crc;
pub mod e2e;
pub mod moving_average;
pub mod spsc;
