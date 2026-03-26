//! # bsw-io
//!
//! I/O abstractions for embedded BSW — Rust port of OpenBSW `io`.
//!
//! Provides zero-copy reader/writer traits and adapters for SPSC communication.
//!
//! ## Provided types
//!
//! - [`Reader`] / [`Writer`] — zero-copy I/O traits (peek/release, allocate/commit)
//! - [`BufferedWriter`] — batches small writes into larger destination allocations
//! - [`ForwardingReader`] — transparently copies data from source reader to destination writer
//! - [`JoinReader`] — multiplexes N readers into one (round-robin)
//! - [`SplitWriter`] — duplicates writes to N destinations
//! - [`MemoryQueue`] — lock-free SPSC queue for variable-size byte messages

#![cfg_attr(not(feature = "std"), no_std)]

pub mod adapters;
pub mod memory_queue;
pub mod traits;

pub use adapters::{BufferedWriter, ForwardingReader, JoinReader, SplitWriter};
pub use memory_queue::MemoryQueue;
pub use traits::{Reader, Writer};
