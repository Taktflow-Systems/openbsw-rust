//! Lock-free SPSC queue for variable-size byte messages.
//!
//! This module provides [`MemoryQueue`], a ring-buffer-backed queue that stores
//! arbitrary byte slices of varying length.  It is the Rust port of the C++
//! OpenBSW `io::MemoryQueue`.
//!
//! # Design
//!
//! Each message is stored in the ring buffer as:
//! ```text
//! [ size_hi : u8 ][ size_lo : u8 ][ payload : size bytes ]
//! ```
//! The two-byte big-endian length prefix is the only overhead per message.
//!
//! ## Double-width logical indices
//!
//! `sent` and `received` are logical indices that run from `0` to
//! `2 * CAPACITY - 1` and wrap at `2 * CAPACITY`.  The physical buffer
//! position is `logical % CAPACITY`.  This double-width scheme removes the
//! ambiguity between "full" and "empty" that arises when head and tail
//! coincide in a single-width design.
//!
//! ## Wrap-around
//!
//! If a message does not fit contiguously before the end of the buffer, the
//! writer emits a **zero-length wrap marker** (`[0x00, 0x00]`) at the current
//! physical position and advances the write cursor to the next buffer
//! boundary.  The reader recognises the zero-length marker and skips past it
//! symmetrically.
//!
//! # Usage
//!
//! ```rust
//! use bsw_io::memory_queue::MemoryQueue;
//!
//! let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
//! let (mut writer, mut reader) = q.split();
//!
//! if let Some(buf) = writer.allocate(5) {
//!     buf.copy_from_slice(b"hello");
//!     writer.commit();
//! }
//!
//! if let Some(msg) = reader.peek() {
//!     assert_eq!(msg, b"hello");
//!     reader.release();
//! }
//! ```

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicUsize, Ordering};

// ── MemoryQueue ──────────────────────────────────────────────────────────────

/// Lock-free SPSC queue for variable-size byte messages.
///
/// - `CAPACITY` — total ring-buffer size in bytes.
/// - `MAX_ELEMENT_SIZE` — maximum payload size of any single message.
///
/// Use [`MemoryQueue::split`] to obtain a [`QueueWriter`] / [`QueueReader`]
/// pair.  Only one writer and one reader may be active at a time; this is
/// enforced by Rust's borrow checker via the `'_` lifetime returned from
/// `split`.
pub struct MemoryQueue<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> {
    data: UnsafeCell<[u8; CAPACITY]>,
    /// Write cursor (producer advances). Wraps at `2 * CAPACITY`.
    sent: AtomicUsize,
    /// Read cursor (consumer advances). Wraps at `2 * CAPACITY`.
    received: AtomicUsize,
}

// SAFETY: `MemoryQueue` is designed for single-producer single-consumer use.
// The producer is the sole writer of `sent`; the consumer is the sole writer
// of `received`.  Atomic Release/Acquire pairs on these two indices establish
// the necessary happens-before relationship so that data written to `data`
// before a `sent` Release store is always visible after the corresponding
// `sent` Acquire load.  `UnsafeCell` suppresses the default aliasing rules for
// the buffer, which is required because producer and consumer hold shared
// references to the same `MemoryQueue` while accessing non-overlapping regions
// of the buffer.
unsafe impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> Send
    for MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>
{
}
// SAFETY: Same reasoning as `Send`.
unsafe impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> Sync
    for MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>
{
}

impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize>
    MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>
{
    /// Create a new, empty `MemoryQueue`.
    ///
    /// Compile-time invariants:
    /// - `MAX_ELEMENT_SIZE > 0`
    /// - `MAX_ELEMENT_SIZE <= 65535` (header is a big-endian `u16`)
    /// - `CAPACITY >= MAX_ELEMENT_SIZE + 2` (room for at least one full message)
    #[must_use]
    pub const fn new() -> Self {
        const {
            assert!(MAX_ELEMENT_SIZE > 0, "MAX_ELEMENT_SIZE must be > 0");
            assert!(
                MAX_ELEMENT_SIZE <= 65535,
                "MAX_ELEMENT_SIZE must fit in a u16 (<=65535)"
            );
            assert!(
                CAPACITY >= MAX_ELEMENT_SIZE + 2,
                "CAPACITY must be >= MAX_ELEMENT_SIZE + 2"
            );
        }

        Self {
            // SAFETY: A zero-initialised byte array is a valid `[u8; CAPACITY]`.
            data: UnsafeCell::new([0u8; CAPACITY]),
            sent: AtomicUsize::new(0),
            received: AtomicUsize::new(0),
        }
    }

    /// Split the queue into a [`QueueWriter`] and a [`QueueReader`].
    ///
    /// Both handles borrow `self` via `&mut self`, so at most one writer and
    /// one reader exist at any time (statically enforced by the borrow
    /// checker).
    #[inline]
    pub fn split(
        &mut self,
    ) -> (
        QueueWriter<'_, CAPACITY, MAX_ELEMENT_SIZE>,
        QueueReader<'_, CAPACITY, MAX_ELEMENT_SIZE>,
    ) {
        let write_pos = self.sent.load(Ordering::Relaxed);
        let read_pos = self.received.load(Ordering::Relaxed);
        (
            QueueWriter {
                queue: self,
                write_pos,
                allocated_size: 0,
            },
            QueueReader {
                queue: self,
                read_pos,
                has_peeked: false,
                peek_total_size: 0,
            },
        )
    }
}

impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> Default
    for MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>
{
    fn default() -> Self {
        Self::new()
    }
}

// ── QueueWriter ──────────────────────────────────────────────────────────────

/// The write-end of a [`MemoryQueue`].
///
/// Obtained via [`MemoryQueue::split`].  Only one `QueueWriter` may exist at
/// a time, enforced by the `'a` lifetime borrow of the parent queue.
pub struct QueueWriter<'a, const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> {
    queue: &'a MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>,
    /// Cached logical write position (only writer modifies `sent`).
    write_pos: usize,
    /// Payload size of the current pending allocation (`0` if none).
    allocated_size: usize,
}

impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize>
    QueueWriter<'_, CAPACITY, MAX_ELEMENT_SIZE>
{
    /// Number of free bytes currently available in the ring buffer.
    ///
    /// This is a conservative snapshot.  The consumer may free more space
    /// concurrently; this method will not see that until the next call.
    #[inline]
    pub fn available(&self) -> usize {
        let received = self.queue.received.load(Ordering::Acquire);
        let used = logical_distance(received, self.write_pos, CAPACITY);
        CAPACITY.saturating_sub(used)
    }

    /// Returns `true` if the queue cannot accommodate another maximum-size
    /// message (i.e. fewer than `MAX_ELEMENT_SIZE + 2` bytes are free).
    #[inline]
    pub fn is_full(&self) -> bool {
        self.available() < MAX_ELEMENT_SIZE + 2
    }

    /// Allocate space for a message of `size` bytes.
    ///
    /// On success, returns a mutable slice of exactly `size` bytes for the
    /// caller to fill.  Call [`commit`](Self::commit) to publish the message.
    ///
    /// Returns `None` if:
    /// - `size == 0` or `size > MAX_ELEMENT_SIZE`, or
    /// - there is not enough contiguous free space.
    pub fn allocate(&mut self, size: usize) -> Option<&mut [u8]> {
        if size == 0 || size > MAX_ELEMENT_SIZE {
            return None;
        }

        let total_needed = size + 2; // payload + 2-byte header

        // Acquire: see all releases the consumer has published.
        let received = self.queue.received.load(Ordering::Acquire);
        let used = logical_distance(received, self.write_pos, CAPACITY);
        let available = CAPACITY.saturating_sub(used);

        let phys_pos = self.write_pos % CAPACITY;
        let contiguous_to_end = CAPACITY - phys_pos;

        let actual_phys_pos = if contiguous_to_end >= total_needed {
            // The message fits contiguously before the end of the buffer.
            // Also verify that the space is actually free (not overwriting
            // unread consumer data).
            if available < total_needed {
                return None;
            }
            phys_pos
        } else {
            // Does not fit contiguously before the end.  Attempt to wrap to
            // the start of the buffer.
            //
            // Wrapping requires at least 2 bytes for the zero-length marker,
            // and the wrapped space must accommodate the full message.
            if contiguous_to_end < 2 {
                // Cannot write a 2-byte wrap marker — fail.
                return None;
            }

            // After wrapping, the logical write cursor advances by
            // `contiguous_to_end` to reach the next CAPACITY boundary
            // (physical 0).
            let wrapped_write_pos =
                advance_logical(self.write_pos, contiguous_to_end, CAPACITY);
            let used_after_wrap = logical_distance(received, wrapped_write_pos, CAPACITY);
            let available_after_wrap = CAPACITY.saturating_sub(used_after_wrap);

            if available_after_wrap < total_needed {
                return None; // not enough space at the start of the buffer
            }

            // Emit the zero-length wrap marker at the current physical
            // position.
            // SAFETY: `phys_pos < CAPACITY` and `phys_pos + 2 <= CAPACITY`
            // (guaranteed by `contiguous_to_end >= 2`).  We are the sole
            // writer; the consumer is behind us in logical space.
            unsafe {
                let buf = &mut *self.queue.data.get();
                buf[phys_pos] = 0x00;
                buf[phys_pos + 1] = 0x00;
            }

            // Advance the logical write cursor past the padding gap.
            self.write_pos = wrapped_write_pos;
            0usize // new physical write position is at the start of the buffer
        };

        self.allocated_size = size;

        // Return a mutable slice over the payload region (after the 2-byte
        // header at `actual_phys_pos`).
        //
        // SAFETY: `actual_phys_pos + 2 + size <= CAPACITY`:
        //   - Non-wrap path: `available >= total_needed` and
        //     `contiguous_to_end >= total_needed`, so `phys_pos + size + 2
        //     <= CAPACITY`.
        //   - Wrap path: `actual_phys_pos == 0` and
        //     `available_after_wrap >= total_needed`, so `0 + size + 2
        //     <= CAPACITY`.
        // The consumer cannot access this region until `commit` publishes
        // `sent` with Release.  `&mut self` prevents concurrent `allocate`
        // calls.
        unsafe {
            let buf = &mut *self.queue.data.get();
            Some(&mut buf[actual_phys_pos + 2..actual_phys_pos + 2 + size])
        }
    }

    /// Commit the allocation made by the last [`allocate`](Self::allocate),
    /// making the message visible to the reader.
    ///
    /// Calling `commit` without a preceding `allocate` is a no-op.
    pub fn commit(&mut self) {
        if self.allocated_size == 0 {
            return;
        }

        let size = self.allocated_size;
        let phys_pos = self.write_pos % CAPACITY;

        // Write the 2-byte big-endian length header.
        // SAFETY: `phys_pos + 2 <= CAPACITY` — guaranteed by `allocate`.  We
        // write the header before advancing `sent`; the Release store below
        // makes both the header and the payload written by the caller visible
        // to the consumer once it loads `sent` with Acquire.
        unsafe {
            let buf = &mut *self.queue.data.get();
            #[allow(clippy::cast_possible_truncation)]
            {
                buf[phys_pos] = (size >> 8) as u8;
                buf[phys_pos + 1] = size as u8;
            }
        }

        // Advance the logical write cursor past header + payload.
        self.write_pos = advance_logical(self.write_pos, size + 2, CAPACITY);

        // Publish to the consumer.
        // Release: makes the buffer writes (header + payload) visible.
        self.queue.sent.store(self.write_pos, Ordering::Release);

        self.allocated_size = 0;
    }
}

// ── QueueReader ──────────────────────────────────────────────────────────────

/// The read-end of a [`MemoryQueue`].
///
/// Obtained via [`MemoryQueue::split`].  Only one `QueueReader` may exist at
/// a time, enforced by the `'a` lifetime borrow of the parent queue.
pub struct QueueReader<'a, const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize> {
    queue: &'a MemoryQueue<CAPACITY, MAX_ELEMENT_SIZE>,
    /// Cached logical read position (only reader modifies `received`).
    read_pos: usize,
    /// Whether a message is currently peeked (pending release).
    has_peeked: bool,
    /// Total bytes to advance on `release` (2-byte header + payload).
    peek_total_size: usize,
}

impl<const CAPACITY: usize, const MAX_ELEMENT_SIZE: usize>
    QueueReader<'_, CAPACITY, MAX_ELEMENT_SIZE>
{
    /// Returns `true` if no messages are currently available.
    #[inline]
    pub fn is_empty(&self) -> bool {
        // Acquire: see the latest `sent` published by the producer.
        let sent = self.queue.sent.load(Ordering::Acquire);
        self.read_pos == sent
    }

    /// Peek at the next available message without consuming it.
    ///
    /// Returns the payload bytes (without the 2-byte length header), or
    /// `None` if the queue is empty.  Repeated calls without an intervening
    /// [`release`](Self::release) return the same slice.
    pub fn peek(&mut self) -> Option<&[u8]> {
        // Return the cached view if we already hold a peeked message.
        if self.has_peeked {
            let phys_pos = self.read_pos % CAPACITY;
            let size = self.peek_total_size - 2;
            // SAFETY: Same invariants as the initial peek below — established
            // when `has_peeked` was set to `true`.
            let buf = unsafe { &*self.queue.data.get() };
            return Some(&buf[phys_pos + 2..phys_pos + 2 + size]);
        }

        // Acquire: see all Release stores the producer has made to `sent`.
        let sent = self.queue.sent.load(Ordering::Acquire);

        loop {
            if self.read_pos == sent {
                return None; // queue is empty
            }

            let phys_pos = self.read_pos % CAPACITY;

            // Read the 2-byte big-endian length header.
            // SAFETY: `phys_pos < CAPACITY`.  `phys_pos + 2 <= CAPACITY` holds
            // because any valid write leaves at least 2 bytes for the header.
            // The Acquire load of `sent` above establishes happens-before with
            // the producer's Release store, so the header bytes are visible.
            let (size_hi, size_lo) = unsafe {
                let buf = &*self.queue.data.get();
                (buf[phys_pos], buf[phys_pos + 1])
            };

            let size = (usize::from(size_hi) << 8) | usize::from(size_lo);

            if size == 0 {
                // Zero-length wrap marker: skip to the next buffer boundary.
                let remaining_to_end = CAPACITY - phys_pos;
                self.read_pos = advance_logical(self.read_pos, remaining_to_end, CAPACITY);
                continue;
            }

            // Valid message.
            self.has_peeked = true;
            self.peek_total_size = size + 2;

            // SAFETY: `phys_pos + 2 + size <= CAPACITY` — the producer
            // verified this before committing.  The Acquire/Release pair on
            // `sent`/`received` ensures the payload bytes are visible.
            let buf = unsafe { &*self.queue.data.get() };
            return Some(&buf[phys_pos + 2..phys_pos + 2 + size]);
        }
    }

    /// Consume the message returned by the last [`peek`](Self::peek).
    ///
    /// Calling `release` without a preceding `peek` is a no-op.
    pub fn release(&mut self) {
        if !self.has_peeked {
            return;
        }

        self.read_pos = advance_logical(self.read_pos, self.peek_total_size, CAPACITY);

        // Publish the updated read cursor to the producer.
        // Release: makes the freed buffer space visible to the producer.
        self.queue.received.store(self.read_pos, Ordering::Release);

        self.has_peeked = false;
        self.peek_total_size = 0;
    }

    /// Discard all pending messages, resetting the queue to empty.
    pub fn clear(&mut self) {
        while self.peek().is_some() {
            self.release();
        }
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Compute the number of bytes logically "between" two cursors in the
/// double-width index space `[0, 2 * capacity)`.
///
/// `from` is the consumer cursor; `to` is the producer cursor.  The result is
/// the number of bytes currently occupied in the ring buffer.
#[inline]
const fn logical_distance(from: usize, to: usize, capacity: usize) -> usize {
    let wrap = 2 * capacity;
    if to >= from {
        to - from
    } else {
        wrap - from + to
    }
}

/// Advance a logical cursor by `delta` bytes, wrapping at `2 * capacity`.
#[inline]
const fn advance_logical(pos: usize, delta: usize, capacity: usize) -> usize {
    let wrap = 2 * capacity;
    let next = pos + delta;
    if next >= wrap {
        next - wrap
    } else {
        next
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::MemoryQueue;

    // ── 1. New queue is empty ──────────────────────────────────────────────
    #[test]
    fn new_queue_is_empty() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (_writer, mut reader) = q.split();
        assert!(reader.is_empty());
        assert!(reader.peek().is_none());
    }

    // ── 2. Write and read a single message ────────────────────────────────
    #[test]
    fn write_and_read_single_message() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let buf = writer.allocate(5).expect("should allocate");
        buf.copy_from_slice(b"hello");
        writer.commit();

        let msg = reader.peek().expect("should have message");
        assert_eq!(msg, b"hello");
        reader.release();

        assert!(reader.is_empty());
    }

    // ── 3. Write multiple messages, read in FIFO order ────────────────────
    #[test]
    fn fifo_order() {
        let mut q: MemoryQueue<128, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let messages: &[&[u8]] = &[b"one", b"two", b"three", b"four"];
        for msg in messages {
            let buf = writer.allocate(msg.len()).expect("should allocate");
            buf.copy_from_slice(msg);
            writer.commit();
        }

        for expected in messages {
            let got = reader.peek().expect("should have message");
            assert_eq!(got, *expected);
            reader.release();
        }
        assert!(reader.is_empty());
    }

    // ── 4. Fill queue, writer returns None ────────────────────────────────
    #[test]
    fn fill_queue_returns_none() {
        // CAPACITY=20, MAX_ELEMENT_SIZE=8: each message uses 10 bytes (8+2).
        // Two messages fill exactly 20 bytes → queue full.
        let mut q: MemoryQueue<20, 8> = MemoryQueue::new();
        let (mut writer, _reader) = q.split();

        let buf = writer.allocate(8).expect("first alloc");
        buf.fill(0xAA);
        writer.commit();

        let buf = writer.allocate(8).expect("second alloc");
        buf.fill(0xBB);
        writer.commit();

        assert!(writer.is_full());
        assert!(writer.allocate(1).is_none());
    }

    // ── 5. Read from empty returns None ───────────────────────────────────
    #[test]
    fn read_from_empty_returns_none() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (_writer, mut reader) = q.split();
        assert!(reader.peek().is_none());
    }

    // ── 6. Fill and drain multiple cycles (wrap-around) ───────────────────
    #[test]
    fn fill_drain_multiple_cycles() {
        // CAPACITY=30, MAX=4: each message = 6 bytes; 5 messages per cycle.
        let mut q: MemoryQueue<30, 4> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        for cycle in 0_u8..10 {
            // Fill with 5 messages of 4 bytes each (5 * 6 = 30 bytes).
            for i in 0_u8..5 {
                let buf = writer.allocate(4).expect("should allocate");
                buf[0] = cycle;
                buf[1] = i;
                buf[2] = 0xAB;
                buf[3] = 0xCD;
                writer.commit();
            }

            // Drain all 5.
            for i in 0_u8..5 {
                let msg = reader.peek().expect("should have message");
                assert_eq!(msg[0], cycle);
                assert_eq!(msg[1], i);
                assert_eq!(msg[2], 0xAB);
                assert_eq!(msg[3], 0xCD);
                reader.release();
            }
            assert!(reader.is_empty());
        }
    }

    // ── 7. Variable-size messages ─────────────────────────────────────────
    #[test]
    fn variable_size_messages() {
        let mut q: MemoryQueue<128, 32> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let buf = writer.allocate(1).expect("allocate 1");
        buf[0] = 0x01;
        writer.commit();

        let buf = writer.allocate(10).expect("allocate 10");
        buf.fill(0x0A);
        writer.commit();

        let buf = writer.allocate(32).expect("allocate 32");
        buf.fill(0x20);
        writer.commit();

        let msg = reader.peek().expect("msg 1");
        assert_eq!(msg.len(), 1);
        assert_eq!(msg[0], 0x01);
        reader.release();

        let msg = reader.peek().expect("msg 2");
        assert_eq!(msg.len(), 10);
        assert!(msg.iter().all(|&b| b == 0x0A));
        reader.release();

        let msg = reader.peek().expect("msg 3");
        assert_eq!(msg.len(), 32);
        assert!(msg.iter().all(|&b| b == 0x20));
        reader.release();

        assert!(reader.is_empty());
    }

    // ── 8. Message at end of buffer wraps to beginning ────────────────────
    //
    // Layout: CAPACITY=12, MAX=8.
    //   Step 1 — write A: 6 payload + 2 hdr = 8 bytes at physical [0..8).
    //            write_pos = 8.
    //   Step 2 — read A: received advances to 8.
    //   Step 3 — write B: 6 payload + 2 hdr = 8 bytes needed.
    //            contiguous_to_end = 12 - 8 = 4 < 8 → wrap.
    //            Wrap marker written at [8..10).
    //            After wrap: write_pos = 12, phys = 0.
    //            used_after_wrap = logical_distance(8, 12, 12) = 4.
    //            available = 12 - 4 = 8 >= 8 ✓.
    //            B placed at physical [0..8).
    //   Step 4 — read B: read_pos = 8, phys = 8.
    //            Reads header [8..10) → size = 0 (wrap marker).
    //            Skips to remaining_to_end = 4, read_pos = 12, phys = 0.
    //            Reads header [0..2) → size = 6.
    //            Returns payload [2..8).
    #[test]
    fn message_wraps_to_beginning() {
        let mut q: MemoryQueue<12, 8> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        // Step 1: write A.
        let buf = writer.allocate(6).expect("alloc A");
        buf.copy_from_slice(b"AAAAAA");
        writer.commit();

        // Step 2: read A to free [0..8) and allow B to wrap there.
        let msg = reader.peek().expect("msg A");
        assert_eq!(msg, b"AAAAAA");
        reader.release();

        // Step 3: write B — wraps to the start of the buffer.
        let buf = writer.allocate(6).expect("alloc B");
        buf.copy_from_slice(b"BBBBBB");
        writer.commit();

        // Step 4: read B (reader skips the wrap marker and finds B at [0..8)).
        let msg = reader.peek().expect("msg B");
        assert_eq!(msg, b"BBBBBB");
        reader.release();

        assert!(reader.is_empty());
    }

    // ── 9. MAX_ELEMENT_SIZE message fits and reads back ───────────────────
    #[test]
    fn max_size_message_round_trip() {
        let mut q: MemoryQueue<64, 32> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let payload: [u8; 32] = core::array::from_fn(|i| i as u8);
        let buf = writer.allocate(32).expect("allocate max");
        buf.copy_from_slice(&payload);
        writer.commit();

        let msg = reader.peek().expect("read max");
        assert_eq!(msg.len(), 32);
        assert_eq!(msg, payload);
        reader.release();
    }

    // ── 10. available() tracks correctly ──────────────────────────────────
    #[test]
    fn available_tracks_correctly() {
        // CAPACITY=32, MAX=8. Initial available = 32.
        let mut q: MemoryQueue<32, 8> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        assert_eq!(writer.available(), 32);

        // Write 6 bytes payload → 8 bytes consumed (6+2).
        let buf = writer.allocate(6).expect("alloc 1");
        buf.fill(0x11);
        writer.commit();
        assert_eq!(writer.available(), 24);

        // Write another 6 bytes.
        let buf = writer.allocate(6).expect("alloc 2");
        buf.fill(0x22);
        writer.commit();
        assert_eq!(writer.available(), 16);

        // Release first message: 8 bytes freed → 24 available.
        let _msg = reader.peek().expect("peek 1");
        reader.release();
        assert_eq!(writer.available(), 24);
    }

    // ── 11. clear() drains all messages ───────────────────────────────────
    #[test]
    fn clear_drains_all() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        for _ in 0..4 {
            let buf = writer.allocate(8).expect("alloc");
            buf.fill(0xFF);
            writer.commit();
        }

        assert!(!reader.is_empty());
        reader.clear();
        assert!(reader.is_empty());
    }

    // ── 12. Writer is_full / Reader is_empty ──────────────────────────────
    #[test]
    fn is_full_is_empty() {
        // CAPACITY=10, MAX=4: each message = 6 bytes.
        // One message leaves 4 bytes free, which is < MAX+2=6 → is_full.
        let mut q: MemoryQueue<10, 4> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        assert!(!writer.is_full());
        assert!(reader.is_empty());

        let buf = writer.allocate(4).expect("alloc");
        buf.fill(0xAA);
        writer.commit();

        assert!(writer.is_full());
        assert!(!reader.is_empty());

        let _msg = reader.peek().expect("peek");
        reader.release();

        assert!(!writer.is_full());
        assert!(reader.is_empty());
    }

    // ── 13. Multiple wrap-arounds in sequence ─────────────────────────────
    #[test]
    fn multiple_wrap_arounds() {
        // CAPACITY=12, MAX=8. Each iteration writes and reads one 4-byte
        // message.  The buffer (6 bytes per message) wraps at various points.
        let mut q: MemoryQueue<12, 8> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        for cycle in 0_u8..20 {
            let buf = writer.allocate(4).expect("alloc");
            buf.iter_mut().for_each(|b| *b = cycle);
            writer.commit();

            let msg = reader.peek().expect("peek");
            assert_eq!(msg.len(), 4);
            assert!(msg.iter().all(|&b| b == cycle));
            reader.release();
        }
    }

    // ── 14. Single-byte messages (minimum payload size) ───────────────────
    #[test]
    fn single_byte_messages() {
        let mut q: MemoryQueue<64, 8> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        for i in 0_u8..20 {
            let buf = writer.allocate(1).expect("alloc 1");
            buf[0] = i;
            writer.commit();
        }

        for i in 0_u8..20 {
            let msg = reader.peek().expect("peek");
            assert_eq!(msg.len(), 1);
            assert_eq!(msg[0], i);
            reader.release();
        }
        assert!(reader.is_empty());
    }

    // ── 15. Message content integrity after wrap-around ───────────────────
    #[test]
    fn content_integrity_after_wrap() {
        // CAPACITY=16, MAX=8.
        // Message 1: 4 bytes payload → 6 bytes at [0..6).
        // Message 2: 8 bytes payload → 10 bytes at [6..16).
        // Drain both; write cursor now at 16 (wraps to 0).
        // Message 3: 6 bytes payload → 8 bytes at [0..8).
        let mut q: MemoryQueue<16, 8> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let buf = writer.allocate(4).expect("alloc 1");
        buf.copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        writer.commit();

        let buf = writer.allocate(8).expect("alloc 2");
        buf.copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x02]);
        writer.commit();

        let msg = reader.peek().expect("peek 1");
        assert_eq!(msg, &[0x11, 0x22, 0x33, 0x44]);
        reader.release();

        let msg = reader.peek().expect("peek 2");
        assert_eq!(msg, &[0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x02]);
        reader.release();

        // Write cursor is now at logical 16 = 0 (mod 16 wrap in double-width
        // is 32, so pos=16, phys=16%16=0).  Read cursor at logical 16, phys=0.
        // Full 16 bytes available again.
        let buf = writer.allocate(6).expect("alloc 3");
        buf.copy_from_slice(&[0x55, 0x66, 0x77, 0x88, 0x99, 0xAA]);
        writer.commit();

        let msg = reader.peek().expect("peek 3");
        assert_eq!(msg, &[0x55, 0x66, 0x77, 0x88, 0x99, 0xAA]);
        reader.release();

        assert!(reader.is_empty());
    }

    // ── 16. Drop does not need special handling ───────────────────────────
    //
    // MemoryQueue is pure bytes — no heap, no destructors.
    // Simply verify that dropping mid-use does not panic.
    #[test]
    fn drop_no_special_handling() {
        {
            let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
            let (mut writer, _reader) = q.split();
            let buf = writer.allocate(8).expect("alloc");
            buf.fill(0xBE);
            writer.commit();
            // Drop writer then queue — no explicit drain needed.
        }
        // No panic == pass.
    }

    // ── 17. allocate(0) returns None ──────────────────────────────────────
    #[test]
    fn allocate_zero_returns_none() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, _reader) = q.split();
        assert!(writer.allocate(0).is_none());
    }

    // ── 18. allocate(> MAX_ELEMENT_SIZE) returns None ─────────────────────
    #[test]
    fn allocate_over_max_returns_none() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, _reader) = q.split();
        assert!(writer.allocate(17).is_none());
    }

    // ── 19. peek is idempotent (same data before release) ─────────────────
    #[test]
    fn peek_is_idempotent() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let buf = writer.allocate(4).expect("alloc");
        buf.copy_from_slice(b"test");
        writer.commit();

        {
            let msg1 = reader.peek().expect("peek 1");
            assert_eq!(msg1, b"test");
        }
        {
            let msg2 = reader.peek().expect("peek 2");
            assert_eq!(msg2, b"test");
        }
        reader.release();
        assert!(reader.is_empty());
    }

    // ── 20. release() without peek is a no-op ─────────────────────────────
    #[test]
    fn release_without_peek_is_noop() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        let buf = writer.allocate(4).expect("alloc");
        buf.copy_from_slice(b"data");
        writer.commit();

        reader.release(); // no-op: no preceding peek

        let msg = reader.peek().expect("peek after no-op release");
        assert_eq!(msg, b"data");
        reader.release();
        assert!(reader.is_empty());
    }

    // ── 21. commit() without allocate is a no-op ──────────────────────────
    #[test]
    fn commit_without_allocate_is_noop() {
        let mut q: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, mut reader) = q.split();

        writer.commit(); // no-op
        assert!(reader.is_empty());

        // A real write after the spurious commit still works.
        let buf = writer.allocate(3).expect("alloc");
        buf.copy_from_slice(b"abc");
        writer.commit();

        let msg = reader.peek().expect("peek");
        assert_eq!(msg, b"abc");
        reader.release();
    }
}
