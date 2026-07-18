//! Console integration with async execution and lifecycle (package C16).
//!
//! [`ConsoleTask`] owns a [`Console`] and drains its input from the read
//! side of a [`bsw_io::MemoryQueue`]. Producers — an ISR byte pump, a POSIX
//! reader thread — enqueue raw bytes through the matching `QueueWriter` and
//! post the task on an executor. Protocol code is never polled directly:
//! the console only runs when input actually arrived.

use bsw_async::{RunContext, Runnable};
use bsw_io::QueueReader;
use bsw_lifecycle::{LifecycleComponent, TransitionResult};

use crate::Console;

/// Runnable, lifecycle-managed console.
///
/// `CAP`/`ELEM` are the input-queue capacity and chunk bounds; `CMDS` and
/// `LINE` bound the registry and line length.
pub struct ConsoleTask<
    'q,
    'c,
    W: core::fmt::Write,
    const CMDS: usize,
    const LINE: usize,
    const CAP: usize,
    const ELEM: usize,
> {
    console: Console<'c, CMDS, LINE>,
    input: QueueReader<'q, CAP, ELEM>,
    out: W,
    active: bool,
}

impl<
        'q,
        'c,
        W: core::fmt::Write,
        const CMDS: usize,
        const LINE: usize,
        const CAP: usize,
        const ELEM: usize,
    > ConsoleTask<'q, 'c, W, CMDS, LINE, CAP, ELEM>
{
    /// Bind a console to its input queue and output writer.
    ///
    /// The task starts inactive; lifecycle `init` activates it.
    pub fn new(
        console: Console<'c, CMDS, LINE>,
        input: QueueReader<'q, CAP, ELEM>,
        out: W,
    ) -> Self {
        Self {
            console,
            input,
            out,
            active: false,
        }
    }

    /// Whether lifecycle has activated input processing.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Access the output writer (test inspection).
    pub fn output(&self) -> &W {
        &self.out
    }

    /// Consume the task, returning the output writer.
    pub fn into_output(self) -> W {
        self.out
    }

    fn drain_input(&mut self) {
        while let Some(chunk) = self.input.peek() {
            for &byte in chunk {
                self.console.feed(byte, &mut self.out);
            }
            self.input.release();
        }
    }
}

impl<
        W: core::fmt::Write,
        const CMDS: usize,
        const LINE: usize,
        const CAP: usize,
        const ELEM: usize,
    > Runnable for ConsoleTask<'_, '_, W, CMDS, LINE, CAP, ELEM>
{
    fn run(&mut self, _context: &mut RunContext) {
        if self.active {
            self.drain_input();
        }
    }
}

impl<
        W: core::fmt::Write,
        const CMDS: usize,
        const LINE: usize,
        const CAP: usize,
        const ELEM: usize,
    > LifecycleComponent for ConsoleTask<'_, '_, W, CMDS, LINE, CAP, ELEM>
{
    fn init(&mut self) -> TransitionResult {
        self.active = true;
        TransitionResult::Done
    }

    fn run(&mut self) -> TransitionResult {
        TransitionResult::Done
    }

    fn shutdown(&mut self) -> TransitionResult {
        // Serve input that arrived before shutdown, then stop accepting.
        self.drain_input();
        self.active = false;
        TransitionResult::Done
    }

    fn name(&self) -> &str {
        "console"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Command, CommandStatus};
    use bsw_async::posix::PosixExecutor;
    use bsw_async::TaskId;
    use bsw_io::MemoryQueue;
    use bsw_time::Instant;

    struct Counter {
        value: u32,
    }

    impl Command for Counter {
        fn name(&self) -> &'static str {
            "count"
        }

        fn help(&self) -> &'static str {
            "count - increment and print"
        }

        fn execute(&mut self, _args: &[&str], out: &mut dyn core::fmt::Write) -> CommandStatus {
            self.value += 1;
            let _ = writeln!(out, "count={}", self.value);
            CommandStatus::Ok
        }
    }

    fn enqueue<const CAP: usize, const ELEM: usize>(
        writer: &mut bsw_io::QueueWriter<'_, CAP, ELEM>,
        bytes: &[u8],
    ) {
        let slot = writer.allocate(bytes.len()).expect("queue full");
        slot.copy_from_slice(bytes);
        writer.commit();
    }

    #[test]
    fn posted_input_executes_commands_without_polling() {
        let mut counter = Counter { value: 0 };
        let mut console: Console<'_, 2, 32> = Console::new();
        console.register(&mut counter).unwrap();
        let mut queue: MemoryQueue<128, 16> = MemoryQueue::new();
        let (mut writer, reader) = queue.split();
        let mut task = ConsoleTask::new(console, reader, String::new());
        assert_eq!(LifecycleComponent::init(&mut task), TransitionResult::Done);

        let output = {
            let mut executor: PosixExecutor<'_, 1, 1> =
                PosixExecutor::new([&mut task as &mut dyn Runnable]);
            let wakeup = executor.wakeup();
            enqueue(&mut writer, b"count\ncou");
            enqueue(&mut writer, b"nt\n");
            wakeup.post(TaskId::new(0)).unwrap();
            // One executor pass serves both chunks; no polling loop exists.
            assert_eq!(executor.run_until_idle(Instant::from_nanos(0)), 1);
            drop(executor);
            task.into_output()
        };
        assert_eq!(output, "count=1\ncount=2\n");
    }

    #[test]
    fn inactive_task_defers_input_until_init() {
        let mut counter = Counter { value: 0 };
        let mut console: Console<'_, 2, 32> = Console::new();
        console.register(&mut counter).unwrap();
        let mut queue: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, reader) = queue.split();
        let mut task = ConsoleTask::new(console, reader, String::new());
        enqueue(&mut writer, b"count\n");
        {
            let mut executor: PosixExecutor<'_, 1, 1> =
                PosixExecutor::new([&mut task as &mut dyn Runnable]);
            executor.post(TaskId::new(0)).unwrap();
            executor.run_until_idle(Instant::from_nanos(0));
        }
        assert_eq!(task.output(), "");
        assert!(!task.is_active());
        LifecycleComponent::init(&mut task);
        {
            let mut executor: PosixExecutor<'_, 1, 1> =
                PosixExecutor::new([&mut task as &mut dyn Runnable]);
            executor.post(TaskId::new(0)).unwrap();
            executor.run_until_idle(Instant::from_nanos(0));
        }
        assert_eq!(task.output(), "count=1\n");
    }

    #[test]
    fn shutdown_flushes_queued_input_then_deactivates() {
        let mut counter = Counter { value: 0 };
        let mut console: Console<'_, 2, 32> = Console::new();
        console.register(&mut counter).unwrap();
        let mut queue: MemoryQueue<64, 16> = MemoryQueue::new();
        let (mut writer, reader) = queue.split();
        let mut task = ConsoleTask::new(console, reader, String::new());
        LifecycleComponent::init(&mut task);
        enqueue(&mut writer, b"count\n");
        assert_eq!(
            LifecycleComponent::shutdown(&mut task),
            TransitionResult::Done
        );
        assert_eq!(task.output(), "count=1\n");
        assert!(!task.is_active());
        assert_eq!(LifecycleComponent::name(&task), "console");
    }
}
