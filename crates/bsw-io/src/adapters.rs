//! Adapters that compose [`Reader`] and [`Writer`] implementations.
//!
//! All types are `no_std`-compatible and use no heap allocation.
//!
//! | Adapter | Trait implemented | Description |
//! |---------|-------------------|-------------|
//! | [`BufferedWriter<W, BUF_SIZE>`] | [`Writer`] | Batches small writes into an internal `[u8; BUF_SIZE]` buffer before forwarding to `W`. |
//! | [`ForwardingReader<R, W>`] | [`Reader`] | Transparently copies each message peeked from `R` into `W` before exposing it. |
//! | [`JoinReader<R, N>`] | [`Reader`] | Round-robin multiplexer: each `peek` returns data from the next non-empty reader in `sources`. |
//! | [`SplitWriter<W, N>`] | [`Writer`] | Broadcasts every committed write to all destinations in `destinations`. |

use crate::traits::{Reader, Writer};

// ── BufferedWriter ───────────────────────────────────────────────────────────

/// A [`Writer`] adapter that accumulates data in an internal `[u8; BUF_SIZE]`
/// buffer and forwards bulk writes to the underlying `W`.
///
/// # Lifecycle
///
/// 1. [`allocate`](Writer::allocate) fills space in the internal buffer.  If
///    the request would overflow the remaining buffer space, the buffer is
///    first flushed to `W`.
/// 2. [`commit`](Writer::commit) advances the `used` cursor, marking the
///    previously allocated bytes as ready.
/// 3. [`flush`](Writer::flush) drains the buffer to `W` via a single
///    `allocate`/`commit` pair.
///
/// # Design trade-off
///
/// Using a fixed inline buffer keeps this type `no_std`-compatible and
/// avoids any dependency on `W`'s allocation lifetime.  The trade-off is
/// that `BUF_SIZE` must be chosen to be ≥ the maximum single-write size.
pub struct BufferedWriter<W: Writer, const BUF_SIZE: usize> {
    /// The downstream writer that receives flushed chunks.
    destination: W,
    /// Internal staging buffer.
    buffer: [u8; BUF_SIZE],
    /// Number of bytes in `buffer` that have been `commit`ted (ready to flush).
    committed: usize,
    /// Number of bytes currently `allocate`d but not yet `commit`ted.
    /// Invariant: `committed + pending <= BUF_SIZE`.
    pending: usize,
}

impl<W: Writer, const BUF_SIZE: usize> BufferedWriter<W, BUF_SIZE> {
    /// Creates a new `BufferedWriter` wrapping `destination`.
    #[must_use]
    pub const fn new(destination: W) -> Self {
        Self {
            destination,
            buffer: [0u8; BUF_SIZE],
            committed: 0,
            pending: 0,
        }
    }

    /// Returns a shared reference to the underlying writer.
    #[must_use]
    #[inline]
    pub fn inner(&self) -> &W {
        &self.destination
    }

    /// Returns a mutable reference to the underlying writer.
    #[inline]
    pub fn inner_mut(&mut self) -> &mut W {
        &mut self.destination
    }

    /// Flush all `committed` bytes in the internal buffer to `destination`.
    ///
    /// After this call `committed` is 0 (or unchanged if the downstream
    /// writer rejects the allocation).
    fn flush_internal(&mut self) {
        if self.committed == 0 {
            return;
        }
        if let Some(dest) = self.destination.allocate(self.committed) {
            dest[..self.committed].copy_from_slice(&self.buffer[..self.committed]);
            self.destination.commit();
            self.committed = 0;
        }
        // If downstream allocation fails we keep the data and try again later.
    }
}

impl<W: Writer, const BUF_SIZE: usize> Writer for BufferedWriter<W, BUF_SIZE> {
    #[inline]
    fn max_size(&self) -> usize {
        BUF_SIZE
    }

    fn allocate(&mut self, size: usize) -> Option<&mut [u8]> {
        // Abandon any previously allocated but uncommitted region.
        self.pending = 0;

        if size == 0 || size > BUF_SIZE {
            return None;
        }

        // If the request doesn't fit in the remaining buffer space, flush first.
        if self.committed + size > BUF_SIZE {
            self.flush_internal();
            // If the buffer is still too full (downstream rejected), fail.
            if self.committed + size > BUF_SIZE {
                return None;
            }
        }

        let start = self.committed;
        self.pending = size;
        // SAFETY: `start + size <= BUF_SIZE` is guaranteed by the check above.
        // We return a mutable subslice of `self.buffer`, which is uniquely
        // owned by `self`.  No other code can access this slice until the next
        // call on `self`.
        Some(&mut self.buffer[start..start + size])
    }

    fn commit(&mut self) {
        // Advance the committed cursor by the pending allocation size.
        self.committed += self.pending;
        self.pending = 0;
    }

    fn flush(&mut self) {
        self.flush_internal();
        self.destination.flush();
    }
}

// ── ForwardingReader ─────────────────────────────────────────────────────────

/// A [`Reader`] adapter that copies every message from `source` into
/// `destination` as it is peeked, then exposes the destination's copy to
/// the caller.
///
/// This is useful for transparent logging or bridging between two queues while
/// keeping the read interface intact.
///
/// # Lifecycle
///
/// 1. [`peek`](Reader::peek): peek the source, allocate+copy into the
///    destination, then peek the destination and return it.
/// 2. [`release`](Reader::release): commit+release both sides.
///
/// [`failed_allocations`](ForwardingReader::failed_allocations) counts how
/// many times the destination had no space to accept a copy.
pub struct ForwardingReader<R: Reader, W: Writer> {
    /// Source to read from.
    source: R,
    /// Destination to copy into.
    destination: W,
    /// Counts destination allocation failures.
    pub failed_allocations: usize,
    /// True while a successful peek/allocate/commit is outstanding.
    active: bool,
}

impl<R: Reader, W: Writer> ForwardingReader<R, W> {
    /// Creates a new `ForwardingReader`.
    #[must_use]
    pub const fn new(source: R, destination: W) -> Self {
        Self {
            source,
            destination,
            failed_allocations: 0,
            active: false,
        }
    }

    /// Returns a shared reference to the source reader.
    #[must_use]
    #[inline]
    pub fn source(&self) -> &R {
        &self.source
    }

    /// Returns a shared reference to the destination writer.
    #[must_use]
    #[inline]
    pub fn destination(&self) -> &W {
        &self.destination
    }
}

impl<R: Reader, W: Writer> Reader for ForwardingReader<R, W> {
    #[inline]
    fn max_size(&self) -> usize {
        self.source.max_size().min(self.destination.max_size())
    }

    fn peek(&self) -> Option<&[u8]> {
        // ForwardingReader::peek is a logically-pure view — the heavy lifting
        // happens in release() which requires &mut self.
        // We return source data directly; the copy to destination is deferred
        // to release() to avoid needing &mut self here.
        self.source.peek()
    }

    fn release(&mut self) {
        // Peek the source to find out how many bytes to forward.
        if let Some(src_data) = self.source.peek() {
            let len = src_data.len();
            // Attempt to allocate space in the destination.
            if let Some(dst_buf) = self.destination.allocate(len) {
                dst_buf.copy_from_slice(src_data);
                self.destination.commit();
                self.active = true;
            } else {
                self.failed_allocations += 1;
                self.active = false;
            }
        } else {
            self.active = false;
        }
        self.source.release();
    }
}

// ── JoinReader ───────────────────────────────────────────────────────────────

/// A [`Reader`] that round-robins across `N` readers of the same type `R`.
///
/// Each call to [`peek`](Reader::peek) starts from `sources[current]` and
/// advances through the array until a non-empty source is found.  After a
/// successful [`release`](Reader::release) the `current` index advances,
/// giving fair scheduling across all sources.
///
/// [`stats`](JoinReader::stats) counts how many messages were released from
/// each source.
pub struct JoinReader<R: Reader, const N: usize> {
    /// Array of source readers.
    sources: [R; N],
    /// Index of the source that was last successfully peeked.
    /// `usize::MAX` means no source is currently active.
    current: usize,
    /// Per-source release counters.
    pub stats: [usize; N],
}

impl<R: Reader, const N: usize> JoinReader<R, N> {
    /// Creates a new `JoinReader` from an array of readers.
    #[must_use]
    pub fn new(sources: [R; N]) -> Self {
        Self {
            sources,
            current: usize::MAX,
            stats: [0; N],
        }
    }

    /// Returns a shared reference to the source array.
    #[must_use]
    #[inline]
    pub fn sources(&self) -> &[R; N] {
        &self.sources
    }
}

impl<R: Reader, const N: usize> Reader for JoinReader<R, N> {
    #[inline]
    fn max_size(&self) -> usize {
        self.sources.iter().map(Reader::max_size).max().unwrap_or(0)
    }

    fn peek(&self) -> Option<&[u8]> {
        if N == 0 {
            return None;
        }

        // If we have an active source from a previous peek, return its data.
        if self.current != usize::MAX {
            return self.sources[self.current].peek();
        }

        // Otherwise scan from 0 to find the first non-empty source.
        // Note: this is a read-only scan; `current` is only updated in release.
        for i in 0..N {
            if let Some(data) = self.sources[i].peek() {
                return Some(data);
            }
        }
        None
    }

    fn release(&mut self) {
        if N == 0 {
            return;
        }

        // Determine which source to release.
        let start = if self.current == usize::MAX { 0 } else { self.current };

        // Walk from `start` to find the first non-empty source.
        for offset in 0..N {
            let i = (start + offset) % N;
            if self.sources[i].peek().is_some() {
                self.sources[i].release();
                self.stats[i] += 1;
                // Advance current to the next index for round-robin.
                self.current = (i + 1) % N;
                return;
            }
        }
        // No data was available; reset current.
        self.current = usize::MAX;
    }
}

// ── SplitWriter ──────────────────────────────────────────────────────────────

/// A [`Writer`] that broadcasts every committed write to all `N` destinations.
///
/// Allocation uses the *most constrained* strategy: the maximum `size` that
/// can be passed to [`allocate`](Writer::allocate) is capped by the smallest
/// `max_size()` across all destinations.  If any destination fails to
/// allocate, that destination's write is skipped and
/// [`drops`](SplitWriter::drops) is incremented for it.
///
/// [`sent`](SplitWriter::sent) counts successful commits per destination.
/// [`drops`](SplitWriter::drops) counts failed allocations per destination.
pub struct SplitWriter<W: Writer, const N: usize> {
    /// Destination writers.
    destinations: [W; N],
    /// Which destinations have a successful allocation outstanding.
    /// Stored as a bitmask (supports up to 64 destinations).
    active_mask: u64,
    /// The size of the current allocation (0 when no allocation is pending).
    alloc_size: usize,
    /// Internal staging buffer used to let the caller fill data once.
    /// We copy this to each destination on commit.
    buffer: [u8; 256],
    /// Per-destination commit counters.
    pub sent: [usize; N],
    /// Per-destination allocation-failure counters.
    pub drops: [usize; N],
}

impl<W: Writer, const N: usize> SplitWriter<W, N> {
    /// Creates a new `SplitWriter` from an array of writers.
    ///
    /// # Panics
    ///
    /// Panics at compile time if `N > 64` (the bitmask only supports 64
    /// destinations).
    #[must_use]
    pub fn new(destinations: [W; N]) -> Self {
        // Compile-time assertion: N must be <= 64 for the bitmask to work.
        const { assert!(N <= 64, "SplitWriter: N must be <= 64") }
        Self {
            destinations,
            active_mask: 0,
            alloc_size: 0,
            buffer: [0u8; 256],
            sent: [0; N],
            drops: [0; N],
        }
    }

    /// Returns a shared reference to the destination array.
    #[must_use]
    #[inline]
    pub fn destinations(&self) -> &[W; N] {
        &self.destinations
    }
}

impl<W: Writer, const N: usize> Writer for SplitWriter<W, N> {
    fn max_size(&self) -> usize {
        if N == 0 {
            return 0;
        }
        self.destinations
            .iter()
            .map(Writer::max_size)
            .min()
            .unwrap_or(0)
            .min(256)
    }

    fn allocate(&mut self, size: usize) -> Option<&mut [u8]> {
        // Discard any previously pending (uncommitted) allocation.
        self.active_mask = 0;
        self.alloc_size = 0;

        if N == 0 || size == 0 || size > 256 {
            return None;
        }

        // Return the staging buffer. We do not pre-allocate in destinations
        // yet — we do that lazily in commit() to avoid holding multiple
        // &mut borrows simultaneously.
        self.alloc_size = size;
        Some(&mut self.buffer[..size])
    }

    fn commit(&mut self) {
        if self.alloc_size == 0 {
            return;
        }
        let size = self.alloc_size;
        self.alloc_size = 0;

        // Broadcast to each destination.
        for i in 0..N {
            if let Some(dst) = self.destinations[i].allocate(size) {
                dst[..size].copy_from_slice(&self.buffer[..size]);
                self.destinations[i].commit();
                self.sent[i] += 1;
            } else {
                self.drops[i] += 1;
            }
        }
    }

    fn flush(&mut self) {
        for dest in &mut self.destinations {
            dest.flush();
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::{BufferedWriter, ForwardingReader, JoinReader, SplitWriter};
    use crate::traits::{Reader, Writer};

    // ── Test helpers ─────────────────────────────────────────────────────────

    /// A simple in-memory byte queue implementing both `Reader` and `Writer`.
    ///
    /// The internal ring buffer holds up to 255 usable bytes (256 slots, one
    /// wasted for the full/empty distinction).  Elements are raw bytes; each
    /// `allocate`/`commit` pair stores exactly one contiguous message whose
    /// length is tracked by a parallel ring of message lengths.
    struct TestQueue {
        /// Raw byte storage.
        data: [u8; 256],
        /// Write cursor (index into `data`).
        write_pos: usize,
        /// Read cursor (index into `data`).
        read_pos: usize,
        /// Sizes of each committed message, in order.
        msg_sizes: [usize; 64],
        /// Index of the next message to read.
        msg_read: usize,
        /// Index of the next message slot to write.
        msg_write: usize,
        /// Size of the in-progress allocation (0 = no allocation pending).
        pending_size: usize,
        /// Maximum message size this queue reports.
        max_msg: usize,
    }

    impl TestQueue {
        fn new(max_msg: usize) -> Self {
            Self {
                data: [0u8; 256],
                write_pos: 0,
                read_pos: 0,
                msg_sizes: [0; 64],
                msg_read: 0,
                msg_write: 0,
                pending_size: 0,
                max_msg,
            }
        }

        fn is_empty(&self) -> bool {
            self.msg_read == self.msg_write
        }
    }

    impl Writer for TestQueue {
        fn max_size(&self) -> usize {
            self.max_msg
        }

        fn allocate(&mut self, size: usize) -> Option<&mut [u8]> {
            if size == 0 || size > self.max_msg {
                return None;
            }
            // Check that there is room in the byte buffer.
            let available = 256_usize.saturating_sub(
                // bytes currently occupied
                if self.write_pos >= self.read_pos {
                    self.write_pos - self.read_pos
                } else {
                    256 - (self.read_pos - self.write_pos)
                },
            );
            // Also need a free message-slot.
            let next_msg_write = (self.msg_write + 1) % 64;
            if size > available || next_msg_write == self.msg_read {
                return None;
            }
            self.pending_size = size;
            let start = self.write_pos;
            // Handle wrap-around: only support contiguous allocations.
            if start + size > 256 {
                // Not enough contiguous space; wrap to 0.
                self.write_pos = 0;
                self.pending_size = size;
            }
            let start = self.write_pos;
            Some(&mut self.data[start..start + size])
        }

        fn commit(&mut self) {
            if self.pending_size == 0 {
                return;
            }
            self.msg_sizes[self.msg_write] = self.pending_size;
            self.msg_write = (self.msg_write + 1) % 64;
            self.write_pos = (self.write_pos + self.pending_size) % 256;
            self.pending_size = 0;
        }

        fn flush(&mut self) {
            // no-op for in-memory queue
        }
    }

    impl Reader for TestQueue {
        fn max_size(&self) -> usize {
            self.max_msg
        }

        fn peek(&self) -> Option<&[u8]> {
            if self.is_empty() {
                return None;
            }
            let size = self.msg_sizes[self.msg_read];
            Some(&self.data[self.read_pos..self.read_pos + size])
        }

        fn release(&mut self) {
            if self.is_empty() {
                return;
            }
            let size = self.msg_sizes[self.msg_read];
            self.read_pos = (self.read_pos + size) % 256;
            self.msg_read = (self.msg_read + 1) % 64;
        }
    }

    // ── Helper: write a byte slice into a writer ──────────────────────────────
    fn write_bytes(w: &mut impl Writer, bytes: &[u8]) -> bool {
        if let Some(buf) = w.allocate(bytes.len()) {
            buf.copy_from_slice(bytes);
            w.commit();
            true
        } else {
            false
        }
    }

    // ── 1. Reader / Writer basic contract ─────────────────────────────────────

    #[test]
    fn reader_writer_basic_contract() {
        let mut q = TestQueue::new(64);
        assert!(q.is_empty());
        assert_eq!(q.peek(), None);

        assert!(write_bytes(&mut q, b"hello"));
        assert!(!q.is_empty());

        let data = q.peek().expect("should have data");
        assert_eq!(data, b"hello");
        // peek again — same data
        let data2 = q.peek().expect("should still have data");
        assert_eq!(data2, b"hello");

        q.release();
        assert!(q.is_empty());
        assert_eq!(q.peek(), None);
    }

    // ── 2. BufferedWriter batches small writes ─────────────────────────────────

    #[test]
    fn buffered_writer_batches_small_writes() {
        let dest = TestQueue::new(64);
        let mut bw: BufferedWriter<TestQueue, 64> = BufferedWriter::new(dest);

        // Write two small messages — they stay in the internal buffer.
        assert!(write_bytes(&mut bw, b"ab"));
        // After commit but before flush, destination is still empty.
        assert!(bw.inner().is_empty(), "destination should be empty before flush");
    }

    // ── 3. BufferedWriter flushes on overflow ──────────────────────────────────

    #[test]
    fn buffered_writer_flushes_on_overflow() {
        let dest = TestQueue::new(128);
        let mut bw: BufferedWriter<TestQueue, 8> = BufferedWriter::new(dest);

        // Fill the buffer with 7 bytes (BUF_SIZE = 8).
        assert!(write_bytes(&mut bw, b"1234567"));
        assert!(bw.inner().is_empty(), "destination still empty before flush");

        // The next write (2 bytes) would overflow the 8-byte buffer, so a
        // flush to the destination should happen first.
        assert!(write_bytes(&mut bw, b"89"));

        // Now destination should have the first 7 bytes.
        assert!(
            !bw.inner().is_empty(),
            "destination should have received flushed data"
        );
    }

    // ── 4. ForwardingReader copies source to destination ──────────────────────

    #[test]
    fn forwarding_reader_copies_source_to_destination() {
        let mut src = TestQueue::new(64);
        let dst = TestQueue::new(64);
        write_bytes(&mut src, b"forward me");

        let mut fr = ForwardingReader::new(src, dst);

        // Peek returns the source data.
        let data = fr.peek().expect("should see source data");
        assert_eq!(data, b"forward me");

        // Release triggers the copy.
        fr.release();

        // Destination should now contain the forwarded bytes.
        let fwd = fr.destination().peek().expect("destination should have data");
        assert_eq!(fwd, b"forward me");
        assert_eq!(fr.failed_allocations, 0);
    }

    // ── 5. ForwardingReader tracks failed allocations ─────────────────────────

    #[test]
    fn forwarding_reader_tracks_failed_allocations() {
        let mut src = TestQueue::new(64);
        // Destination max_msg = 2, so a 5-byte message won't fit.
        let dst = TestQueue::new(2);
        write_bytes(&mut src, b"toolong");

        let mut fr = ForwardingReader::new(src, dst);
        let _ = fr.peek();
        fr.release();

        assert_eq!(fr.failed_allocations, 1);
    }

    // ── 6. JoinReader round-robins across sources ─────────────────────────────

    #[test]
    fn join_reader_round_robin() {
        let mut a = TestQueue::new(64);
        let mut b = TestQueue::new(64);
        write_bytes(&mut a, b"A");
        write_bytes(&mut b, b"B");

        let mut jr = JoinReader::new([a, b]);

        // First peek should come from source 0.
        let d1 = jr.peek().expect("should have data").to_vec();
        jr.release();

        // Second peek should come from source 1 (round-robin advances).
        let d2 = jr.peek().expect("should have data").to_vec();
        jr.release();

        // Both messages must be received, one from each source.
        let mut got = [d1.as_slice() == b"A", d2.as_slice() == b"B"];
        got.sort();
        assert!(got[0] && got[1], "expected A then B");
    }

    // ── 7. JoinReader stats track per-source reads ────────────────────────────

    #[test]
    fn join_reader_stats() {
        let mut a = TestQueue::new(64);
        let mut b = TestQueue::new(64);
        write_bytes(&mut a, b"X");
        write_bytes(&mut a, b"Y");
        write_bytes(&mut b, b"Z");

        let mut jr = JoinReader::new([a, b]);

        // Drain all three messages.
        for _ in 0..3 {
            assert!(jr.peek().is_some());
            jr.release();
        }
        assert_eq!(jr.peek(), None);

        // Total stats must sum to 3.
        assert_eq!(jr.stats[0] + jr.stats[1], 3);
    }

    // ── 8. SplitWriter broadcasts to all destinations ─────────────────────────

    #[test]
    fn split_writer_broadcasts() {
        let d0 = TestQueue::new(64);
        let d1 = TestQueue::new(64);
        let mut sw = SplitWriter::new([d0, d1]);

        assert!(write_bytes(&mut sw, b"broadcast"));
        sw.flush();

        let dests = sw.destinations();
        assert_eq!(dests[0].peek().expect("d0 should have data"), b"broadcast");
        assert_eq!(dests[1].peek().expect("d1 should have data"), b"broadcast");
    }

    // ── 9. SplitWriter tracks sent/drops per destination ──────────────────────

    #[test]
    fn split_writer_tracks_sent() {
        let d0 = TestQueue::new(64);
        let d1 = TestQueue::new(64);
        let mut sw = SplitWriter::new([d0, d1]);

        assert!(write_bytes(&mut sw, b"one"));
        assert!(write_bytes(&mut sw, b"two"));

        assert_eq!(sw.sent[0], 2);
        assert_eq!(sw.sent[1], 2);
        assert_eq!(sw.drops[0], 0);
        assert_eq!(sw.drops[1], 0);
    }

    // ── 10. SplitWriter handles partial failures ──────────────────────────────

    #[test]
    fn split_writer_partial_failure() {
        let d0 = TestQueue::new(64);
        // d1 only accepts 2-byte messages; a 5-byte write will fail for it.
        let d1 = TestQueue::new(2);
        let mut sw = SplitWriter::new([d0, d1]);

        assert!(write_bytes(&mut sw, b"hello")); // d1 will reject

        assert_eq!(sw.sent[0], 1, "d0 should succeed");
        assert_eq!(sw.drops[1], 1, "d1 should report a drop");
        assert_eq!(sw.sent[1], 0, "d1 should not count a send");
    }

    // ── 11. Empty queue peek returns None ─────────────────────────────────────

    #[test]
    fn empty_queue_peek_returns_none() {
        let q = TestQueue::new(64);
        assert_eq!(q.peek(), None);
    }

    // ── 12. Full queue allocate returns None ──────────────────────────────────

    #[test]
    fn full_queue_allocate_returns_none() {
        let mut q = TestQueue::new(64);
        // Fill the queue with many small messages to exhaust the message-slot ring.
        let mut filled = false;
        for _ in 0..65 {
            if q.allocate(1).is_none() {
                filled = true;
                break;
            }
            q.commit();
        }
        assert!(filled, "queue should have reported full at some point");
    }

    // ── 13. release on empty queue is a no-op ─────────────────────────────────

    #[test]
    fn release_on_empty_is_noop() {
        let mut q = TestQueue::new(64);
        // Must not panic.
        q.release();
        assert!(q.is_empty());
    }

    // ── 14. commit without allocate is a no-op ────────────────────────────────

    #[test]
    fn commit_without_allocate_is_noop() {
        let dest = TestQueue::new(64);
        let mut bw: BufferedWriter<TestQueue, 32> = BufferedWriter::new(dest);
        // Must not panic or corrupt state.
        bw.commit();
        assert!(bw.inner().is_empty());
    }
}
