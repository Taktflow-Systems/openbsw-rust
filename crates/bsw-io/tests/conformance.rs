//! Reader/Writer contract conformance (packages D04/D05).
//!
//! One generic writer/reader contract is asserted against every built-in
//! implementation; the cross-thread stress test proves the SPSC queue under
//! sustained wrap and backpressure.

use bsw_io::adapters::{BufferedWriter, ForwardingReader, JoinReader, SplitWriter};
use bsw_io::{MemoryQueue, Reader, Writer};

/// Drive the generic allocate/commit contract:
/// zero/oversize rejection, exact-size slices, re-allocation discarding the
/// pending write, and no-op commit. Leaves `abc` and `ZW` committed.
fn writer_contract<W: Writer>(writer: &mut W) {
    assert!(writer.allocate(0).is_none(), "zero-size allocate must fail");
    assert!(
        writer.allocate(writer.max_size() + 1).is_none(),
        "oversize allocate must fail"
    );
    let slice = writer.allocate(3).expect("allocate 3");
    assert_eq!(slice.len(), 3);
    slice.copy_from_slice(b"abc");
    writer.commit();
    let slice = writer.allocate(2).expect("allocate 2");
    slice.copy_from_slice(b"xy");
    let slice = writer.allocate(2).expect("re-allocate discards pending");
    slice.copy_from_slice(b"ZW");
    writer.commit();
    writer.commit();
    writer.flush();
}

/// Drive the generic peek/release contract over `expected` messages.
fn reader_contract<R: Reader>(reader: &mut R, expected: &[&[u8]]) {
    reader.release();
    for message in expected {
        let first = reader.peek().expect("message available").to_vec();
        let second = reader.peek().expect("peek is stable").to_vec();
        assert_eq!(first, second);
        assert_eq!(&first, message);
        assert!(first.len() <= reader.max_size());
        reader.release();
    }
    assert!(reader.peek().is_none());
    reader.release();
    assert!(reader.peek().is_none());
}

/// Drain every message through the `Reader` trait, flattened to bytes.
fn drain_flat<R: Reader>(reader: &mut R) -> Vec<u8> {
    let mut bytes = Vec::new();
    while let Some(message) = reader.peek() {
        bytes.extend_from_slice(message);
        reader.release();
    }
    bytes
}

#[test]
fn memory_queue_handles_conform() {
    let mut queue: MemoryQueue<64, 16> = MemoryQueue::new();
    let (mut writer, mut reader) = queue.split();
    writer_contract(&mut writer);
    reader_contract(&mut reader, &[b"abc", b"ZW"]);
}

#[test]
fn memory_queue_full_reports_and_recovers() {
    let mut queue: MemoryQueue<18, 16> = MemoryQueue::new();
    let (mut writer, mut reader) = queue.split();
    let slice = writer.allocate(16).unwrap();
    slice.fill(7);
    writer.commit();
    // 18 bytes hold exactly one 16-byte message + 2-byte header.
    assert!(writer.allocate(1).is_none());
    // Backpressure clears once the reader releases.
    assert_eq!(reader.peek().unwrap().len(), 16);
    reader.release();
    assert!(writer.allocate(1).is_some());
}

#[test]
fn buffered_writer_conforms_over_a_queue() {
    let mut queue: MemoryQueue<128, 16> = MemoryQueue::new();
    let (writer, mut reader) = queue.split();
    let mut buffered: BufferedWriter<_, 16> = BufferedWriter::new(writer);
    writer_contract(&mut buffered);
    // Buffering may coalesce message boundaries; the byte stream must be
    // exactly the committed payloads in order.
    assert_eq!(drain_flat(&mut reader), b"abcZW".to_vec());
}

#[test]
fn split_writer_duplicates_to_all_queues() {
    let mut queue_a: MemoryQueue<64, 16> = MemoryQueue::new();
    let mut queue_b: MemoryQueue<64, 16> = MemoryQueue::new();
    let (writer_a, mut reader_a) = queue_a.split();
    let (writer_b, mut reader_b) = queue_b.split();
    let mut split = SplitWriter::new([writer_a, writer_b]);
    writer_contract(&mut split);
    reader_contract(&mut reader_a, &[b"abc", b"ZW"]);
    reader_contract(&mut reader_b, &[b"abc", b"ZW"]);
}

#[test]
fn join_reader_serves_all_sources() {
    let mut queue_a: MemoryQueue<64, 16> = MemoryQueue::new();
    let mut queue_b: MemoryQueue<64, 16> = MemoryQueue::new();
    let (mut writer_a, reader_a) = queue_a.split();
    let (mut writer_b, reader_b) = queue_b.split();
    writer_a.allocate(1).unwrap().copy_from_slice(b"a");
    writer_a.commit();
    writer_b.allocate(1).unwrap().copy_from_slice(b"b");
    writer_b.commit();
    let mut join = JoinReader::new([reader_a, reader_b]);
    let mut seen = Vec::new();
    while let Some(message) = join.peek() {
        seen.push(message.to_vec());
        join.release();
    }
    seen.sort();
    assert_eq!(seen, vec![b"a".to_vec(), b"b".to_vec()]);
}

#[test]
fn forwarding_reader_copies_while_reading() {
    let mut source_queue: MemoryQueue<64, 16> = MemoryQueue::new();
    let mut tap_queue: MemoryQueue<64, 16> = MemoryQueue::new();
    let (mut source_writer, source_reader) = source_queue.split();
    let (tap_writer, mut tap_reader) = tap_queue.split();
    source_writer.allocate(4).unwrap().copy_from_slice(b"data");
    source_writer.commit();
    let mut forwarding = ForwardingReader::new(source_reader, tap_writer);
    reader_contract(&mut forwarding, &[b"data"]);
    assert_eq!(drain_flat(&mut tap_reader), b"data".to_vec());
}

/// D05: cross-thread SPSC stress with wrap and backpressure.
///
/// The writer pushes 10,000 sequence-stamped messages of varying size
/// through a queue that only holds a few messages at a time, forcing
/// continuous wrap-marker traffic and full-queue backpressure. The reader
/// verifies exact FIFO order, sizes, and content.
#[test]
fn spsc_cross_thread_wrap_and_backpressure() {
    const MESSAGES: u32 = 10_000;
    let mut queue: MemoryQueue<61, 16> = MemoryQueue::new();
    let (mut writer, mut reader) = queue.split();

    std::thread::scope(|scope| {
        scope.spawn(move || {
            let mut sequence: u32 = 0;
            while sequence < MESSAGES {
                let size = 4 + (sequence % 12) as usize;
                if let Some(slice) = writer.allocate(size) {
                    slice[..4].copy_from_slice(&sequence.to_le_bytes());
                    for (offset, byte) in slice[4..].iter_mut().enumerate() {
                        *byte = (sequence as u8).wrapping_add(offset as u8);
                    }
                    writer.commit();
                    sequence += 1;
                } else {
                    std::thread::yield_now();
                }
            }
        });

        let mut expected: u32 = 0;
        while expected < MESSAGES {
            if let Some(message) = reader.peek() {
                let sequence = u32::from_le_bytes(message[..4].try_into().unwrap());
                assert_eq!(sequence, expected, "FIFO order violated");
                let size = 4 + (sequence % 12) as usize;
                assert_eq!(message.len(), size, "message length corrupted");
                for (offset, &byte) in message[4..].iter().enumerate() {
                    assert_eq!(byte, (sequence as u8).wrapping_add(offset as u8));
                }
                reader.release();
                expected += 1;
            } else {
                std::thread::yield_now();
            }
        }
        assert!(reader.peek().is_none());
    });
}
