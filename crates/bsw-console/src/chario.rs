//! Character I/O contract for embedded console transports (package C18).
//!
//! Board UART drivers implement [`CharOutput`]/[`CharInput`]; the console
//! stack talks only to these traits. [`UartWriter`] adapts a non-blocking
//! [`CharOutput`] to [`core::fmt::Write`] with a bounded staging ring, so a
//! busy or slow UART can never block a caller: bytes that do not fit are
//! dropped and counted.

/// Non-blocking byte output failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CharOutError {
    /// The device cannot accept a byte right now; retry after progress.
    Busy,
}

/// Non-blocking byte sink implemented by UART/virtual-COM drivers.
pub trait CharOutput {
    /// Try to write one byte without blocking.
    fn try_write(&mut self, byte: u8) -> Result<(), CharOutError>;
}

/// Non-blocking byte source implemented by UART/virtual-COM drivers.
pub trait CharInput {
    /// Read one byte if available.
    fn try_read(&mut self) -> Option<u8>;
}

/// Bounded, never-blocking [`core::fmt::Write`] adapter over a
/// [`CharOutput`].
///
/// Formatting stages bytes in an internal ring; [`UartWriter::pump`] moves
/// them to the device as it accepts them. A full ring drops the newest
/// bytes and counts them, which keeps logging and console output safe to
/// call from time-critical code.
pub struct UartWriter<U: CharOutput, const BUF: usize> {
    uart: U,
    ring: [u8; BUF],
    head: usize,
    stored: usize,
    dropped: u32,
}

impl<U: CharOutput, const BUF: usize> UartWriter<U, BUF> {
    /// Wrap a device.
    pub const fn new(uart: U) -> Self {
        Self {
            uart,
            ring: [0; BUF],
            head: 0,
            stored: 0,
            dropped: 0,
        }
    }

    /// Bytes staged and not yet accepted by the device.
    pub const fn pending(&self) -> usize {
        self.stored
    }

    /// Bytes dropped because the staging ring was full.
    pub const fn dropped(&self) -> u32 {
        self.dropped
    }

    /// Access the wrapped device.
    pub fn uart_mut(&mut self) -> &mut U {
        &mut self.uart
    }

    /// Push staged bytes into the device until it reports busy.
    ///
    /// Returns the number of bytes the device accepted. Call from the main
    /// loop or a TX-ready interrupt handler.
    pub fn pump(&mut self) -> usize {
        let mut written = 0;
        while self.stored > 0 {
            let byte = self.ring[self.head];
            match self.uart.try_write(byte) {
                Ok(()) => {
                    self.head = (self.head + 1) % BUF;
                    self.stored -= 1;
                    written += 1;
                }
                Err(CharOutError::Busy) => break,
            }
        }
        written
    }

    fn push(&mut self, byte: u8) {
        if self.stored == BUF {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        let slot = (self.head + self.stored) % BUF;
        self.ring[slot] = byte;
        self.stored += 1;
    }
}

impl<U: CharOutput, const BUF: usize> core::fmt::Write for UartWriter<U, BUF> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &byte in s.as_bytes() {
            self.push(byte);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::fmt::Write as _;

    /// Mock UART with a per-pump acceptance budget and a busy latch.
    struct MockUart {
        accepted: Vec<u8>,
        budget: usize,
        remaining: usize,
        jammed: bool,
    }

    impl MockUart {
        fn new(budget: usize) -> Self {
            Self {
                accepted: Vec::new(),
                budget,
                remaining: budget,
                jammed: false,
            }
        }

        fn recharge(&mut self) {
            self.remaining = self.budget;
        }
    }

    impl CharOutput for MockUart {
        fn try_write(&mut self, byte: u8) -> Result<(), CharOutError> {
            if self.jammed || self.remaining == 0 {
                return Err(CharOutError::Busy);
            }
            self.remaining -= 1;
            self.accepted.push(byte);
            Ok(())
        }
    }

    struct ScriptInput {
        bytes: Vec<u8>,
    }

    impl CharInput for ScriptInput {
        fn try_read(&mut self) -> Option<u8> {
            if self.bytes.is_empty() {
                None
            } else {
                Some(self.bytes.remove(0))
            }
        }
    }

    #[test]
    fn partial_writes_resume_where_they_stopped() {
        let mut writer: UartWriter<MockUart, 16> = UartWriter::new(MockUart::new(3));
        write!(writer, "hello").unwrap();
        assert_eq!(writer.pump(), 3);
        assert_eq!(writer.pending(), 2);
        writer.uart_mut().recharge();
        assert_eq!(writer.pump(), 2);
        assert_eq!(writer.uart_mut().accepted, b"hello");
        assert_eq!(writer.dropped(), 0);
    }

    #[test]
    fn overflow_drops_newest_bytes_and_counts_them() {
        let mut writer: UartWriter<MockUart, 4> = UartWriter::new(MockUart::new(0));
        write!(writer, "abcdef").unwrap();
        assert_eq!(writer.pending(), 4);
        assert_eq!(writer.dropped(), 2);
        writer.uart_mut().recharge();
        writer.uart_mut().budget = 10;
        writer.uart_mut().recharge();
        writer.pump();
        // The oldest bytes survive; the tail was dropped.
        assert_eq!(writer.uart_mut().accepted, b"abcd");
    }

    #[test]
    fn jammed_device_recovers_without_losing_staged_bytes() {
        let mut uart = MockUart::new(100);
        uart.jammed = true;
        let mut writer: UartWriter<MockUart, 8> = UartWriter::new(uart);
        write!(writer, "ok").unwrap();
        assert_eq!(writer.pump(), 0);
        assert_eq!(writer.pending(), 2);
        writer.uart_mut().jammed = false;
        assert_eq!(writer.pump(), 2);
        assert_eq!(writer.uart_mut().accepted, b"ok");
    }

    #[test]
    fn formatting_never_reports_errors() {
        let mut writer: UartWriter<MockUart, 2> = UartWriter::new(MockUart::new(0));
        // Even a hopeless writer returns Ok: loss is counted, not raised.
        assert!(write!(writer, "far too much text").is_ok());
        assert!(writer.dropped() > 0);
    }

    #[test]
    fn char_input_feeds_console_bytes() {
        let mut input = ScriptInput {
            bytes: b"hi\n".to_vec(),
        };
        let mut collected = Vec::new();
        while let Some(byte) = input.try_read() {
            collected.push(byte);
        }
        assert_eq!(collected, b"hi\n");
    }
}
