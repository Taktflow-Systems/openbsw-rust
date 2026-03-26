//! Zero-copy I/O traits for embedded BSW.
//!
//! # Design
//!
//! These traits model the two-phase I/O pattern used by OpenBSW's C++ `IReader`
//! and `IWriter` interfaces.  All operations work on `&[u8]` byte slices to
//! keep the interface simple and `no_std`-friendly.
//!
//! ## Reader — peek / release
//!
//! 1. Call [`Reader::peek`] to borrow the next available byte slice without
//!    consuming it.  Returns `None` when no data is available.
//! 2. Process the borrowed bytes.
//! 3. Call [`Reader::release`] to acknowledge consumption and advance the
//!    internal cursor.
//!
//! `peek` followed by `release` is the only valid sequence.  Calling
//! `release` without a preceding `peek`, or calling `peek` twice without an
//! intervening `release`, is allowed but implementations are permitted to
//! treat it as a no-op.
//!
//! ## Writer — allocate / commit / flush
//!
//! 1. Call [`Writer::allocate`] with the desired number of bytes.  Returns a
//!    mutable slice of exactly that size, or `None` if the destination cannot
//!    satisfy the request right now (e.g. full).
//! 2. Fill the returned slice with data.
//! 3. Call [`Writer::commit`] to finalise the write and make the data visible
//!    to the consumer.
//! 4. Optionally call [`Writer::flush`] to push buffered data downstream.

// ── Reader ──────────────────────────────────────────────────────────────────

/// Zero-copy reader.
///
/// Implementors expose data via a *peek-then-release* protocol:
///
/// 1. [`peek`](Reader::peek) — borrow the next available byte slice without
///    consuming it.
/// 2. [`release`](Reader::release) — acknowledge that the bytes returned by
///    the previous `peek` have been consumed and advance the cursor.
///
/// # Contract
///
/// - `peek` must return the **same** slice on repeated calls until `release`
///   is called.
/// - `release` without a prior `peek` (or after the data was already released)
///   is a no-op in all built-in implementations; custom implementations must
///   not panic.
/// - `max_size` returns the maximum number of bytes that a single `peek` may
///   return.
pub trait Reader {
    /// Maximum number of bytes that a single [`peek`](Self::peek) can return.
    fn max_size(&self) -> usize;

    /// Borrow the next available byte slice without consuming it.
    ///
    /// Returns `None` when no data is available.  The returned slice is valid
    /// until the next call to [`release`](Self::release).
    fn peek(&self) -> Option<&[u8]>;

    /// Acknowledge that the bytes returned by the last [`peek`](Self::peek)
    /// have been consumed and advance the internal cursor.
    ///
    /// Calling `release` when there is no pending peek is a no-op.
    fn release(&mut self);
}

// ── Writer ──────────────────────────────────────────────────────────────────

/// Zero-copy writer.
///
/// Implementors expose writable space via an *allocate-then-commit* protocol:
///
/// 1. [`allocate`](Writer::allocate) — reserve `size` bytes and receive a
///    mutable slice to fill.
/// 2. (fill the slice with data)
/// 3. [`commit`](Writer::commit) — finalise the write and make the data
///    visible to the consumer.
/// 4. [`flush`](Writer::flush) — push any buffered data downstream.
///
/// # Contract
///
/// - Only one allocation may be outstanding at a time.  Calling `allocate`
///   again before `commit` discards the previous allocation in all built-in
///   implementations.
/// - `commit` without a prior `allocate` is a no-op.
/// - `max_size` returns the largest value that can be passed to `allocate`
///   and is guaranteed to succeed (assuming the destination is not full).
pub trait Writer {
    /// Maximum number of bytes that a single [`allocate`](Self::allocate) call
    /// can satisfy.
    fn max_size(&self) -> usize;

    /// Reserve `size` bytes for writing.
    ///
    /// Returns a mutable slice of exactly `size` bytes on success, or `None`
    /// if the destination cannot accommodate the request (e.g. full, or
    /// `size > max_size()`).
    fn allocate(&mut self, size: usize) -> Option<&mut [u8]>;

    /// Commit the data written to the slice returned by the last
    /// [`allocate`](Self::allocate) call.
    ///
    /// After `commit` the data is visible to the consumer.  Calling `commit`
    /// without a prior `allocate` is a no-op.
    fn commit(&mut self);

    /// Flush any buffered data to the underlying destination.
    ///
    /// For writers that do not buffer, this is a no-op.
    fn flush(&mut self);
}
