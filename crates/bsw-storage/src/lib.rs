//! Persistent block storage for the OpenBSW Rust port (packages D06, D07,
//! D08, D11).
//!
//! This crate ports the behaviour of the upstream `storage` module
//! (`EepStorage` / `FeeStorage` / `MappingStorage` / `QueuingStorage` /
//! `StorageJob`) onto native Rust contracts:
//!
//! - [`backend`] — the low-level [`backend::StorageBackend`] trait modelling
//!   NOR-flash-like hardware (erase units, program units, program-only-clears
//!   bits) plus the shared [`backend::StorageError`] code set.
//! - [`block`] — flash-geometry-independent block storage:
//!   [`block::BlockId`], [`block::BlockMetadata`], and the
//!   [`block::BlockStore`] trait (the analogue of upstream's `IStorage`
//!   block API).
//! - [`mem`] — [`mem::MemBackend`], an in-memory backend that faithfully
//!   models flash semantics, with per-unit erase counters for wear tests.
//! - [`jobs`] — a poll-based asynchronous job layer
//!   ([`jobs::StorageJob`], [`jobs::JobQueue`]) replacing upstream's
//!   callback-driven `StorageJob` / `QueuingStorage`.
//! - [`journal`] — [`journal::JournalStore`], a journaled, power-fail-safe
//!   [`block::BlockStore`] over any backend (upstream `FeeStorage`
//!   analogue) using a two-area copy-on-write log with commit marks.
//! - [`fault`] *(std)* — [`fault::CutPointBackend`], deterministic power-cut
//!   and torn-write injection for recovery testing.
//! - [`file`] *(std)* — [`file::FileBackend`], a POSIX file-backed backend
//!   with persisted geometry (upstream `EepStorage`-style persistent
//!   storage for host builds).
//! - [`blob`] — bounded streaming of payloads larger than one block
//!   ([`blob::BlobWriter`], [`blob::BlobReader`]).
//! - [`conformance`] *(std)* — shared conformance and cut-point-enumeration
//!   suites, generic over the backend so [`mem::MemBackend`] and
//!   [`file::FileBackend`] run identical checks.
//!
//! Core modules are `no_std` and allocation-free; only the std-gated test
//! harness and file backend allocate.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod backend;
pub mod blob;
pub mod block;
#[cfg(feature = "std")]
pub mod conformance;
#[cfg(feature = "std")]
pub mod fault;
#[cfg(feature = "std")]
pub mod file;
pub mod jobs;
pub mod journal;
pub mod mem;

pub use backend::{StorageBackend, StorageError};
pub use block::{BlockId, BlockMetadata, BlockStore};
