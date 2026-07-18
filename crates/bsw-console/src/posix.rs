//! POSIX console adapters (package C17).
//!
//! [`InputPump`] moves bytes from any [`std::io::Read`] — stdin, a pipe, a
//! scripted [`std::io::Cursor`] — into a console input queue and wakes the
//! executor that owns the console task. [`IoWriter`] adapts
//! [`std::io::Write`] to the `core::fmt::Write` output contract.
//!
//! Production wiring reads stdin on a dedicated thread:
//! the queue lives in a `static`, the thread owns the `QueueWriter` and a
//! [`bsw_async::posix::Wakeup`], and the main thread runs the executor.
//! Tests replace stdin with a cursor and drive the executor
//! deterministically — the transcript is identical on every run.

use std::io::Read;

use bsw_async::posix::Wakeup;
use bsw_async::TaskId;
use bsw_io::QueueWriter;

/// Chunked reader-to-queue pump.
///
/// Bytes read from the source but not yet accepted by the queue stay in an
/// internal carry buffer, so a full queue never loses input.
pub struct InputPump<R: Read> {
    source: R,
    carry: Vec<u8>,
}

/// Outcome of one pump step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PumpStatus {
    /// Bytes were enqueued and the task was posted.
    Delivered(usize),
    /// The source is exhausted (EOF) and no carried bytes remain.
    Eof,
    /// The queue had no room; the bytes are carried for the next attempt.
    QueueFull,
}

impl<R: Read> InputPump<R> {
    /// Create a pump over `source`.
    pub fn new(source: R) -> Self {
        Self {
            source,
            carry: Vec::new(),
        }
    }

    /// Enqueue one chunk of at most `ELEM` bytes and post `task`.
    ///
    /// A blocking source blocks in `read`; run the pump on its own thread
    /// for interactive input.
    pub fn pump_once<const CAP: usize, const ELEM: usize>(
        &mut self,
        queue: &mut QueueWriter<'_, CAP, ELEM>,
        wakeup: &Wakeup,
        task: TaskId,
    ) -> std::io::Result<PumpStatus> {
        if self.carry.is_empty() {
            let mut chunk = [0u8; 4096];
            let read = self.source.read(&mut chunk)?;
            if read == 0 {
                return Ok(PumpStatus::Eof);
            }
            self.carry.extend_from_slice(&chunk[..read]);
        }
        let take = self.carry.len().min(ELEM);
        let Some(slot) = queue.allocate(take) else {
            return Ok(PumpStatus::QueueFull);
        };
        slot.copy_from_slice(&self.carry[..take]);
        queue.commit();
        self.carry.drain(..take);
        let _ = wakeup.post(task);
        Ok(PumpStatus::Delivered(take))
    }

    /// Pump until EOF, ignoring transient queue-full states by retrying
    /// after the executor drains. Intended for scripted sessions where the
    /// caller alternates pumping and executor runs.
    pub fn pump_to_eof<const CAP: usize, const ELEM: usize>(
        &mut self,
        queue: &mut QueueWriter<'_, CAP, ELEM>,
        wakeup: &Wakeup,
        task: TaskId,
        mut drain: impl FnMut(),
    ) -> std::io::Result<usize> {
        let mut delivered = 0;
        loop {
            match self.pump_once(queue, wakeup, task)? {
                PumpStatus::Delivered(count) => delivered += count,
                PumpStatus::Eof => return Ok(delivered),
                PumpStatus::QueueFull => drain(),
            }
        }
    }
}

/// `core::fmt::Write` adapter over any [`std::io::Write`].
///
/// I/O errors are recorded, not propagated: console output must never
/// abort command execution.
pub struct IoWriter<W: std::io::Write> {
    inner: W,
    errors: u32,
}

impl<W: std::io::Write> IoWriter<W> {
    /// Wrap an output stream.
    pub fn new(inner: W) -> Self {
        Self { inner, errors: 0 }
    }

    /// Number of write errors swallowed so far.
    pub fn errors(&self) -> u32 {
        self.errors
    }

    /// Consume the adapter, returning the stream.
    pub fn into_inner(self) -> W {
        self.inner
    }
}

impl<W: std::io::Write> core::fmt::Write for IoWriter<W> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if self.inner.write_all(s.as_bytes()).is_err() || self.inner.flush().is_err() {
            self.errors = self.errors.saturating_add(1);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::task::ConsoleTask;
    use crate::{Command, CommandStatus, Console};
    use bsw_async::posix::PosixExecutor;
    use bsw_async::Runnable;
    use bsw_io::MemoryQueue;
    use bsw_lifecycle::LifecycleComponent;
    use bsw_time::Instant;
    use core::fmt::Write as _;
    use std::io::Cursor;

    struct Add {
        total: i64,
    }

    impl Command for Add {
        fn name(&self) -> &'static str {
            "add"
        }

        fn help(&self) -> &'static str {
            "add <n> - add to the running total"
        }

        fn execute(&mut self, args: &[&str], out: &mut dyn core::fmt::Write) -> CommandStatus {
            let Some(value) = args.first().and_then(|arg| arg.parse::<i64>().ok()) else {
                return CommandStatus::UsageError;
            };
            self.total += value;
            let _ = writeln!(out, "total={}", self.total);
            CommandStatus::Ok
        }
    }

    fn run_scripted_session(script: &str) -> String {
        let mut add = Add { total: 0 };
        let mut console: Console<'_, 2, 64> = Console::new();
        console.register(&mut add).unwrap();
        let mut queue: MemoryQueue<256, 32> = MemoryQueue::new();
        let (mut writer, reader) = queue.split();
        let mut task = ConsoleTask::new(console, reader, String::new());
        LifecycleComponent::init(&mut task);
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let wakeup = executor.wakeup();
        let mut pump = InputPump::new(Cursor::new(script.as_bytes().to_vec()));
        pump.pump_to_eof(&mut writer, &wakeup, bsw_async::TaskId::new(0), || {
            executor.run_until_idle(Instant::from_nanos(0));
        })
        .unwrap();
        executor.run_until_idle(Instant::from_nanos(0));
        drop(executor);
        task.into_output()
    }

    #[test]
    fn scripted_sessions_are_deterministic() {
        let script = "add 5\nadd 37\nadd nope\nsub 1\n";
        let expected = "total=5\ntotal=42\nusage: add <n> - add to the running total\n\
                        unknown command: sub (try 'help')\n";
        let first = run_scripted_session(script);
        let second = run_scripted_session(script);
        assert_eq!(first, expected);
        assert_eq!(second, expected);
    }

    #[test]
    fn long_scripts_survive_bounded_queues() {
        let mut script = String::new();
        let mut expected = String::new();
        for index in 1..=50 {
            script.push_str("add 1\n");
            let _ = writeln!(expected, "total={index}");
        }
        assert_eq!(run_scripted_session(&script), expected);
    }

    #[test]
    fn io_writer_swallows_stream_errors() {
        struct Broken;
        impl std::io::Write for Broken {
            fn write(&mut self, _buf: &[u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("gone"))
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let mut writer = IoWriter::new(Broken);
        assert!(writeln!(writer, "hello").is_ok());
        assert_eq!(writer.errors(), 1);
    }

    #[test]
    fn io_writer_passes_bytes_through() {
        let mut writer = IoWriter::new(Vec::new());
        write!(writer, "ok {}", 7).unwrap();
        assert_eq!(writer.errors(), 0);
        assert_eq!(writer.into_inner(), b"ok 7");
    }
}
