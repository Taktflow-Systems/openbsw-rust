//! Platform-independent, allocation-free runnable and executor interfaces.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod cyclic;
#[cfg(feature = "std")]
pub mod posix;

use bsw_time::Instant;

/// Index of a task in an executor's fixed task table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskId(u16);

impl TaskId {
    /// Construct a task identifier.
    pub const fn new(raw: u16) -> Self {
        Self(raw)
    }

    /// Numeric task-table index.
    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Posting failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PostError {
    /// Task ID does not name an entry in this executor.
    UnknownTask,
}

/// Context passed to one runnable invocation.
pub struct RunContext {
    task: TaskId,
    now: Instant,
    repost: bool,
}

impl RunContext {
    /// Currently executing task.
    pub const fn task(&self) -> TaskId {
        self.task
    }

    /// Time supplied by the executor caller.
    pub const fn now(&self) -> Instant {
        self.now
    }

    /// Request another invocation after the current invocation finishes.
    pub fn repost(&mut self) {
        self.repost = true;
    }
}

/// Unit of cooperatively scheduled work.
pub trait Runnable {
    /// Execute one bounded step.
    fn run(&mut self, context: &mut RunContext);
}

/// Fixed-task, deterministic cooperative executor.
pub struct Executor<'a, const N: usize> {
    tasks: [&'a mut dyn Runnable; N],
    pending: [bool; N],
}

impl<'a, const N: usize> Executor<'a, N> {
    /// Construct an executor from its caller-owned task table.
    pub fn new(tasks: [&'a mut dyn Runnable; N]) -> Self {
        Self {
            tasks,
            pending: [false; N],
        }
    }

    /// Mark a task pending. Repeated posts coalesce into one pending invocation.
    pub fn post(&mut self, task: TaskId) -> Result<(), PostError> {
        let pending = self
            .pending
            .get_mut(usize::from(task.0))
            .ok_or(PostError::UnknownTask)?;
        *pending = true;
        Ok(())
    }

    /// Cancel a pending invocation. Running work is never interrupted.
    pub fn cancel(&mut self, task: TaskId) -> Result<bool, PostError> {
        let pending = self
            .pending
            .get_mut(usize::from(task.0))
            .ok_or(PostError::UnknownTask)?;
        Ok(core::mem::take(pending))
    }

    /// Run the lowest-ID pending task once.
    pub fn run_one(&mut self, now: Instant) -> Option<TaskId> {
        let index = self.pending.iter().position(|pending| *pending)?;
        self.pending[index] = false;
        let task = TaskId(u16::try_from(index).ok()?);
        let mut context = RunContext {
            task,
            now,
            repost: false,
        };
        self.tasks[index].run(&mut context);
        self.pending[index] = context.repost;
        Some(task)
    }

    /// Run at most `budget` task invocations.
    pub fn drain(&mut self, now: Instant, budget: usize) -> usize {
        let mut completed = 0;
        while completed < budget && self.run_one(now).is_some() {
            completed += 1;
        }
        completed
    }

    /// Return whether any task is pending.
    pub fn has_pending(&self) -> bool {
        self.pending.iter().any(|pending| *pending)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct Counter {
        runs: u8,
        repost_until: u8,
    }

    impl Runnable for Counter {
        fn run(&mut self, context: &mut RunContext) {
            self.runs += 1;
            if self.runs < self.repost_until {
                context.repost();
            }
        }
    }

    #[test]
    fn posting_coalesces_and_lowest_id_runs_first() {
        let mut first = Counter::default();
        let mut second = Counter::default();
        {
            let mut executor = Executor::new([
                &mut first as &mut dyn Runnable,
                &mut second as &mut dyn Runnable,
            ]);
            executor.post(TaskId::new(1)).unwrap();
            executor.post(TaskId::new(1)).unwrap();
            executor.post(TaskId::new(0)).unwrap();
            assert_eq!(
                executor.run_one(Instant::from_nanos(1)),
                Some(TaskId::new(0))
            );
            assert_eq!(
                executor.run_one(Instant::from_nanos(1)),
                Some(TaskId::new(1))
            );
            assert!(!executor.has_pending());
        }
        assert_eq!((first.runs, second.runs), (1, 1));
    }

    #[test]
    fn cancellation_and_unknown_ids_are_explicit() {
        let mut task = Counter::default();
        let mut executor = Executor::new([&mut task as &mut dyn Runnable]);
        let id = TaskId::new(0);
        executor.post(id).unwrap();
        assert_eq!(executor.cancel(id), Ok(true));
        assert_eq!(executor.cancel(id), Ok(false));
        assert_eq!(executor.post(TaskId::new(1)), Err(PostError::UnknownTask));
    }

    #[test]
    fn runnable_can_repost_with_a_bounded_drain_budget() {
        let mut task = Counter {
            runs: 0,
            repost_until: 3,
        };
        {
            let mut executor = Executor::new([&mut task as &mut dyn Runnable]);
            executor.post(TaskId::new(0)).unwrap();
            assert_eq!(executor.drain(Instant::from_nanos(9), 2), 2);
            assert!(executor.has_pending());
            assert_eq!(executor.drain(Instant::from_nanos(9), 2), 1);
        }
        assert_eq!(task.runs, 3);
    }
}
