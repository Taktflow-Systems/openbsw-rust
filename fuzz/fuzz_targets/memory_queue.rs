#![no_main]

use bsw_io::MemoryQueue;
use libfuzzer_sys::fuzz_target;
use std::collections::VecDeque;

// Interpret the fuzz input as a script of writer/reader operations against
// a small queue that wraps constantly, and check every observation against
// a shadow FIFO model.
fuzz_target!(|data: &[u8]| {
    let mut queue: MemoryQueue<29, 12> = MemoryQueue::new();
    let (mut writer, mut reader) = queue.split();
    let mut model: VecDeque<Vec<u8>> = VecDeque::new();
    let mut pending: Option<Vec<u8>> = None;
    // Mirrors the reader's internal has_peeked state.
    let mut peeked = false;

    for chunk in data.chunks(2) {
        let op = chunk[0] % 5;
        let arg = usize::from(chunk.get(1).copied().unwrap_or(1));
        match op {
            // allocate + fill; any allocate cancels a pending one.
            0 => {
                let size = arg % 14;
                if let Some(slice) = writer.allocate(size) {
                    assert!((1..=12).contains(&size), "allocate accepted invalid size");
                    for (index, byte) in slice.iter_mut().enumerate() {
                        *byte = (index as u8).wrapping_add(chunk[0]);
                    }
                    pending = Some(slice.to_vec());
                } else {
                    pending = None;
                }
            }
            // commit publishes exactly the pending allocation, if any.
            1 => {
                writer.commit();
                if let Some(message) = pending.take() {
                    model.push_back(message);
                }
            }
            // peek + verify against the model.
            2 => match (reader.peek(), model.front()) {
                (Some(observed), Some(expected)) => {
                    assert_eq!(observed, expected.as_slice(), "FIFO content diverged");
                    peeked = true;
                }
                (Some(_), None) => panic!("queue has data the model does not"),
                (None, Some(_)) => panic!("model has data the queue does not"),
                (None, None) => {}
            },
            // release consumes if and only if a peek is outstanding.
            3 => {
                reader.release();
                if peeked {
                    model.pop_front();
                    peeked = false;
                }
            }
            // release without an outstanding peek must be a no-op.
            _ => {
                if !peeked {
                    let before = model.len();
                    reader.release();
                    assert_eq!(model.len(), before);
                    if let Some(expected) = model.front() {
                        let observed = reader.peek().expect("head must survive no-op release");
                        assert_eq!(observed, expected.as_slice());
                        peeked = true;
                    }
                }
            }
        }
    }

    // Drain and verify everything that is left.
    if peeked {
        reader.release();
        model.pop_front();
    }
    while let Some(expected) = model.pop_front() {
        let observed = reader.peek().expect("model expects another message");
        assert_eq!(observed, expected.as_slice());
        reader.release();
    }
    assert!(reader.peek().is_none());
});
