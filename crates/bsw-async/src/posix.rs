//! POSIX host executor with deterministic and threaded operating modes.
//!
//! [`PosixExecutor`] wraps the platform-independent [`Executor`] and
//! [`TimerQueue`] behind two entry points:
//!
//! - [`PosixExecutor::run_until_idle`] is the deterministic single-thread
//!   mode. The caller injects every observed instant (typically from a
//!   [`bsw_time::FakeClock`]), so scheduling decisions are reproducible.
//! - [`PosixExecutor::run_for`] is the real-clock mode. It sleeps on a
//!   condition variable between invocations and is woken by [`Wakeup`]
//!   handles owned by other threads.
//!
//! Both modes share one pending-set semantic: posting an already-pending
//! task coalesces into a single invocation, exactly as in the underlying
//! deterministic executor.

use std::sync::{Arc, Condvar, Mutex};

use bsw_time::{Clock, Duration, Instant, TimerError, TimerHandle, TimerQueue};

use crate::{Executor, PostError, Runnable, TaskId};

/// Cross-thread pending posts protected by [`Shared::condvar`].
struct PendingPosts {
    posts: Vec<bool>,
    wake: bool,
}

struct Shared {
    pending: Mutex<PendingPosts>,
    condvar: Condvar,
}

/// Thread-safe posting handle for a [`PosixExecutor`].
///
/// Cloning is cheap; every clone posts into the same executor. Posting from
/// the executor thread itself is allowed but unnecessary.
#[derive(Clone)]
pub struct Wakeup {
    shared: Arc<Shared>,
}

impl Wakeup {
    /// Mark `task` pending and wake the executor if it is sleeping.
    pub fn post(&self, task: TaskId) -> Result<(), PostError> {
        let mut pending = self.shared.pending.lock().expect("executor mutex poisoned");
        let slot = pending
            .posts
            .get_mut(usize::from(task.get()))
            .ok_or(PostError::UnknownTask)?;
        *slot = true;
        pending.wake = true;
        drop(pending);
        self.shared.condvar.notify_one();
        Ok(())
    }

    /// Wake the executor without posting a task.
    ///
    /// `run_for` re-evaluates its window and timers after a bare wake, which
    /// lets shutdown coordinators interrupt a long sleep.
    pub fn notify(&self) {
        let mut pending = self.shared.pending.lock().expect("executor mutex poisoned");
        pending.wake = true;
        drop(pending);
        self.shared.condvar.notify_one();
    }
}

/// Host executor combining tasks, timers, and cross-thread wakeup.
///
/// `N` is the fixed task-table size; `T` is the timer-queue capacity.
pub struct PosixExecutor<'a, const N: usize, const T: usize> {
    executor: Executor<'a, N>,
    timers: TimerQueue<T>,
    shared: Arc<Shared>,
}

impl<'a, const N: usize, const T: usize> PosixExecutor<'a, N, T> {
    /// Construct an executor from its caller-owned task table.
    pub fn new(tasks: [&'a mut dyn Runnable; N]) -> Self {
        Self {
            executor: Executor::new(tasks),
            timers: TimerQueue::new(),
            shared: Arc::new(Shared {
                pending: Mutex::new(PendingPosts {
                    posts: vec![false; N],
                    wake: false,
                }),
                condvar: Condvar::new(),
            }),
        }
    }

    /// Create a thread-safe posting handle.
    pub fn wakeup(&self) -> Wakeup {
        Wakeup {
            shared: Arc::clone(&self.shared),
        }
    }

    /// Post a task from the executor thread.
    pub fn post(&mut self, task: TaskId) -> Result<(), PostError> {
        self.executor.post(task)
    }

    /// Cancel a pending invocation posted from any thread.
    ///
    /// Returns whether an invocation was actually pending.
    pub fn cancel(&mut self, task: TaskId) -> Result<bool, PostError> {
        let mut pending = self.shared.pending.lock().expect("executor mutex poisoned");
        let slot = pending
            .posts
            .get_mut(usize::from(task.get()))
            .ok_or(PostError::UnknownTask)?;
        let was_shared = core::mem::take(slot);
        drop(pending);
        let was_local = self.executor.cancel(task)?;
        Ok(was_shared || was_local)
    }

    /// Schedule `task` to be posted once `deadline` is reached.
    pub fn schedule(&mut self, task: TaskId, deadline: Instant) -> Result<TimerHandle, TimerError> {
        if usize::from(task.get()) >= N {
            return Err(TimerError::CapacityTooLarge);
        }
        self.timers.register(task.get(), deadline)
    }

    /// Schedule `task` to be posted at `deadline` and then every `period`.
    pub fn schedule_periodic(
        &mut self,
        task: TaskId,
        deadline: Instant,
        period: Duration,
    ) -> Result<TimerHandle, TimerError> {
        if usize::from(task.get()) >= N {
            return Err(TimerError::CapacityTooLarge);
        }
        self.timers.register_periodic(task.get(), deadline, period)
    }

    /// Cancel a scheduled posting. Stale handles return `false`.
    pub fn cancel_scheduled(&mut self, handle: TimerHandle) -> bool {
        self.timers.cancel(handle)
    }

    /// Move cross-thread posts into the deterministic executor.
    fn import_posts(&mut self) {
        let mut pending = self.shared.pending.lock().expect("executor mutex poisoned");
        pending.wake = false;
        for (index, posted) in pending.posts.iter_mut().enumerate() {
            if core::mem::take(posted) {
                let task = TaskId::new(index as u16);
                // The vector length equals N, so the ID is always valid.
                let _ = self.executor.post(task);
            }
        }
    }

    /// Post every timer that is due at `now`.
    fn fire_due_timers(&mut self, now: Instant) {
        while let Some(event) = self.timers.poll(now) {
            let _ = self.executor.post(TaskId::new(event.id));
        }
    }

    /// Deterministic single-thread mode: run until no work remains at `now`.
    ///
    /// Imports cross-thread posts, fires due timers, and drains tasks until
    /// the executor is idle. Time never advances on its own; the caller owns
    /// the clock. Returns the number of task invocations.
    pub fn run_until_idle(&mut self, now: Instant) -> usize {
        let mut invocations = 0;
        loop {
            self.import_posts();
            self.fire_due_timers(now);
            if !self.executor.has_pending() {
                return invocations;
            }
            while self.executor.run_one(now).is_some() {
                invocations += 1;
            }
        }
    }

    /// Real-clock mode: serve posts and timers for one bounded window.
    ///
    /// Runs until `window` has elapsed on `clock`, sleeping between wakeups.
    /// Due timers and cross-thread posts are served as they arrive. Returns
    /// the number of task invocations.
    pub fn run_for<C: Clock>(&mut self, clock: &C, window: Duration) -> usize {
        let start = clock.now();
        let end = start.wrapping_add(window);
        let mut invocations = 0;
        loop {
            let now = clock.now();
            self.import_posts();
            self.fire_due_timers(now);
            while self.executor.run_one(now).is_some() {
                invocations += 1;
            }
            let now = clock.now();
            if now.is_at_or_after(end) {
                return invocations;
            }
            let mut until_end = end.duration_since(now);
            if let Some(deadline) = self.timers.next_deadline(now) {
                let until_deadline = deadline.duration_since(now);
                if until_deadline < until_end {
                    until_end = until_deadline;
                }
            }
            if until_end == Duration::ZERO {
                continue;
            }
            let sleep = std::time::Duration::from_nanos(until_end.as_nanos());
            let pending = self.shared.pending.lock().expect("executor mutex poisoned");
            if !pending.wake {
                let (guard, _timeout) = self
                    .shared
                    .condvar
                    .wait_timeout(pending, sleep)
                    .expect("executor mutex poisoned");
                drop(guard);
            }
        }
    }
}
