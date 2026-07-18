//! Typed multi-frame messaging over a byte queue (package D04).
//!
//! Rust port of upstream `io::VariantQueue`: several frame types, each with
//! an optional dynamically sized payload, travel through one
//! [`MemoryQueue`](crate::MemoryQueue) using the wire encoding
//!
//! ```text
//! [ type id : u8 ][ header : HEADER_SIZE bytes ][ payload : 0..N bytes ]
//! ```
//!
//! Upstream derives type ids and header layout from a C++ type list and
//! reinterprets POD structs in place. The native Rust replacement makes
//! both explicit: a frame type implements [`VariantFrame`] with its type id
//! and a byte-exact header codec, which keeps the wire format identical
//! without unaligned pointer casts. Dispatch over the frame types is an
//! ordinary `match` on [`VariantMessage::type_id`] instead of a visitor
//! template.

use crate::traits::{Reader, Writer};

/// One frame type that can travel through a variant queue.
pub trait VariantFrame: Sized {
    /// Wire type id (the first encoded byte). Must be unique per queue.
    const TYPE_ID: u8;

    /// Encoded header length in bytes.
    const HEADER_SIZE: usize;

    /// Encode the header into `out`, which is exactly `HEADER_SIZE` bytes.
    fn encode_header(&self, out: &mut [u8]);

    /// Decode a header from `bytes`, which is exactly `HEADER_SIZE` bytes.
    /// Returns `None` when the bytes do not form a valid header.
    fn decode_header(bytes: &[u8]) -> Option<Self>;
}

/// Failure to enqueue a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VariantWriteError {
    /// The destination queue cannot take the message right now.
    QueueFull,
}

/// Write `frame` plus `payload` into `writer` as one message.
pub fn write_frame<F: VariantFrame, W: Writer>(
    writer: &mut W,
    frame: &F,
    payload: &[u8],
) -> Result<(), VariantWriteError> {
    let total = 1 + F::HEADER_SIZE + payload.len();
    let Some(buffer) = writer.allocate(total) else {
        return Err(VariantWriteError::QueueFull);
    };
    buffer[0] = F::TYPE_ID;
    frame.encode_header(&mut buffer[1..=F::HEADER_SIZE]);
    buffer[1 + F::HEADER_SIZE..].copy_from_slice(payload);
    writer.commit();
    Ok(())
}

/// One received variant message, borrowed from the queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VariantMessage<'a> {
    /// Wire type id of the frame.
    pub type_id: u8,
    /// Header and payload bytes (everything after the type id).
    pub bytes: &'a [u8],
}

impl<'a> VariantMessage<'a> {
    /// Reinterpret a raw queue message as a variant message.
    ///
    /// Returns `None` for an empty message, which cannot carry a type id.
    pub fn parse(raw: &'a [u8]) -> Option<Self> {
        let (&type_id, bytes) = raw.split_first()?;
        Some(Self { type_id, bytes })
    }

    /// Decode this message as frame type `F`.
    ///
    /// Returns the frame and its payload when the type id matches and the
    /// header is well-formed; `None` otherwise.
    pub fn decode<F: VariantFrame>(&self) -> Option<(F, &'a [u8])> {
        if self.type_id != F::TYPE_ID {
            return None;
        }
        let header = self.bytes.get(..F::HEADER_SIZE)?;
        let frame = F::decode_header(header)?;
        Some((frame, &self.bytes[F::HEADER_SIZE..]))
    }
}

/// Peek the next variant message from `reader`.
///
/// The message stays in the queue until [`Reader::release`] is called.
/// Returns `None` when the queue is empty or the head message is too short
/// to carry a type id.
pub fn peek_frame<R: Reader>(reader: &R) -> Option<VariantMessage<'_>> {
    VariantMessage::parse(reader.peek()?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MemoryQueue;

    /// Mirror of upstream test struct `A { uint8_t values[5]; }` with up to
    /// three payload bytes.
    #[derive(Debug, PartialEq, Eq)]
    struct FrameA {
        values: [u8; 5],
    }

    impl VariantFrame for FrameA {
        const TYPE_ID: u8 = 0;
        const HEADER_SIZE: usize = 5;

        fn encode_header(&self, out: &mut [u8]) {
            out.copy_from_slice(&self.values);
        }

        fn decode_header(bytes: &[u8]) -> Option<Self> {
            Some(Self {
                values: bytes.try_into().ok()?,
            })
        }
    }

    /// Mirror of upstream packed `B { be_uint16 x; be_uint32 y; }`.
    #[derive(Debug, PartialEq, Eq)]
    struct FrameB {
        x: u16,
        y: u32,
    }

    impl VariantFrame for FrameB {
        const TYPE_ID: u8 = 1;
        const HEADER_SIZE: usize = 6;

        fn encode_header(&self, out: &mut [u8]) {
            out[..2].copy_from_slice(&self.x.to_be_bytes());
            out[2..].copy_from_slice(&self.y.to_be_bytes());
        }

        fn decode_header(bytes: &[u8]) -> Option<Self> {
            Some(Self {
                x: u16::from_be_bytes(bytes.get(..2)?.try_into().ok()?),
                y: u32::from_be_bytes(bytes.get(2..6)?.try_into().ok()?),
            })
        }
    }

    /// Mirror of upstream empty struct `C` with up to 15 payload bytes.
    #[derive(Debug, PartialEq, Eq)]
    struct FrameC;

    impl VariantFrame for FrameC {
        const TYPE_ID: u8 = 2;
        const HEADER_SIZE: usize = 0;

        fn encode_header(&self, _out: &mut [u8]) {}

        fn decode_header(_bytes: &[u8]) -> Option<Self> {
            Some(Self)
        }
    }

    // Upstream: max element = max(5+3, 6, 0+15) = 16, queue element adds the
    // type id byte.
    const MAX_ELEMENT: usize = 17;
    const CAPACITY: usize = 40;

    /// Port of upstream `VariantQueue.read_write_no_payload`.
    #[test]
    fn read_write_no_payload_matches_upstream_scenario() {
        let mut queue: MemoryQueue<CAPACITY, MAX_ELEMENT> = MemoryQueue::new();
        let (mut writer, mut reader) = queue.split();

        write_frame(
            &mut writer,
            &FrameA {
                values: [0, 1, 2, 3, 4],
            },
            &[],
        )
        .unwrap();
        write_frame(
            &mut writer,
            &FrameB {
                x: 3,
                y: 0xFAAF_1253,
            },
            &[],
        )
        .unwrap();
        write_frame(&mut writer, &FrameC, &[]).unwrap();
        write_frame(&mut writer, &FrameC, &[]).unwrap();
        // Upstream's queue wraps eagerly whenever a full maximum element no
        // longer fits before the buffer end, so its fifth write already
        // fails here. This port wraps lazily and keeps using the tail
        // (intentional native difference recorded in the io parity notes),
        // so exhaustion arrives deterministically a few messages later:
        // 40-byte capacity minus 8+9+3+3 used leaves 17 bytes; two more
        // 3-byte messages fit, a 15-byte message does not.
        write_frame(&mut writer, &FrameC, &[]).unwrap();
        write_frame(&mut writer, &FrameC, &[]).unwrap();
        assert_eq!(
            write_frame(&mut writer, &FrameC, b"0123456789ab"),
            Err(VariantWriteError::QueueFull)
        );

        let message = peek_frame(&reader).unwrap();
        let (frame, payload) = message.decode::<FrameA>().unwrap();
        assert_eq!(
            frame,
            FrameA {
                values: [0, 1, 2, 3, 4]
            }
        );
        assert!(payload.is_empty());
        assert!(message.decode::<FrameB>().is_none(), "type id mismatch");
        Reader::release(&mut reader);

        let message = peek_frame(&reader).unwrap();
        let (frame, _) = message.decode::<FrameB>().unwrap();
        assert_eq!(
            frame,
            FrameB {
                x: 3,
                y: 0xFAAF_1253
            }
        );
        Reader::release(&mut reader);

        let message = peek_frame(&reader).unwrap();
        assert!(message.decode::<FrameC>().is_some());
        Reader::release(&mut reader);
        // Unlike upstream, release without a fresh peek is a no-op here, so
        // the remaining messages are drained peek-by-peek.
        let mut drained = 0;
        while peek_frame(&reader).is_some() {
            Reader::release(&mut reader);
            drained += 1;
        }
        assert_eq!(drained, 3);
        assert!(peek_frame(&reader).is_none());
    }

    /// Port of upstream `VariantQueue.read_write_with_payload`.
    #[test]
    fn read_write_with_payload_roundtrips() {
        let mut queue: MemoryQueue<CAPACITY, MAX_ELEMENT> = MemoryQueue::new();
        let (mut writer, reader) = queue.split();

        write_frame(
            &mut writer,
            &FrameA {
                values: [9, 8, 7, 6, 5],
            },
            &[0xAA, 0xBB],
        )
        .unwrap();
        write_frame(&mut writer, &FrameC, b"0123456789abcde").unwrap();

        let message = peek_frame(&reader).unwrap();
        let (frame, payload) = message.decode::<FrameA>().unwrap();
        assert_eq!(frame.values, [9, 8, 7, 6, 5]);
        assert_eq!(payload, &[0xAA, 0xBB]);
        let mut reader = reader;
        Reader::release(&mut reader);

        let message = peek_frame(&reader).unwrap();
        let (_, payload) = message.decode::<FrameC>().unwrap();
        assert_eq!(payload, b"0123456789abcde");
    }

    #[test]
    fn malformed_messages_decode_to_none() {
        assert!(VariantMessage::parse(&[]).is_none());
        let short = VariantMessage {
            type_id: FrameB::TYPE_ID,
            bytes: &[1, 2, 3],
        };
        assert!(short.decode::<FrameB>().is_none());
    }
}
