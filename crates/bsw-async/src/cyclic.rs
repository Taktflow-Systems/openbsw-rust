//! Cyclic execution on top of the common runnable contract (package C11).
//!
//! [`CyclicExecutor`] adapts the embedded scheduling model — fixed tasks
//! executed at fixed periods from a main-loop tick — into the portable
//! [`Executor`]/[`Runnable`] contract. Protocol components schedule against
//! this type and [`crate::posix::PosixExecutor`] interchangeably; only the
//! platform main loop differs.
//!
//! Differences from the BSP cyclic scheduler it replaces:
//!
//! - deadlines advance drift-free from the scheduled deadline, not from the
//!   observed tick time;
//! - missed periods are skipped and counted per task as overruns;
//! - event (non-periodic) posting and cancellation share the same task
//!   table, so a task can be both cyclic and event-driven.

use bsw_time::{Duration, Instant};

use crate::{Executor, PostError, Runnable, TaskId};

#[derive(Debug, Clone, Copy)]
struct CyclicSlot {
    period: Duration,
    next: Instant,
    overruns: u32,
}

/// Error returned by cyclic registration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CyclicError {
    /// Task ID does not name an entry in this executor.
    UnknownTask,
    /// A zero period would make the task due forever.
    ZeroPeriod,
}

/// Fixed-task executor that runs tasks periodically from a driven tick.
pub struct CyclicExecutor<'a, const N: usize> {
    executor: Executor<'a, N>,
    schedule: [Option<CyclicSlot>; N],
}

impl<'a, const N: usize> CyclicExecutor<'a, N> {
    /// Construct from the caller-owned task table.
    pub fn new(tasks: [&'a mut dyn Runnable; N]) -> Self {
        Self {
            executor: Executor::new(tasks),
            schedule: [None; N],
        }
    }

    /// Run `task` every `period`, first at `first_deadline`.
    ///
    /// Re-registering an already-cyclic task replaces its period and
    /// deadline and clears its overrun count.
    pub fn set_period(
        &mut self,
        task: TaskId,
        period: Duration,
        first_deadline: Instant,
    ) -> Result<(), CyclicError> {
        if period == Duration::ZERO {
            return Err(CyclicError::ZeroPeriod);
        }
        let slot = self
            .schedule
            .get_mut(usize::from(task.get()))
            .ok_or(CyclicError::UnknownTask)?;
        *slot = Some(CyclicSlot {
            period,
            next: first_deadline,
            overruns: 0,
        });
        Ok(())
    }

    /// Stop periodic execution of `task`. Returns whether it was cyclic.
    pub fn clear_period(&mut self, task: TaskId) -> Result<bool, CyclicError> {
        let slot = self
            .schedule
            .get_mut(usize::from(task.get()))
            .ok_or(CyclicError::UnknownTask)?;
        Ok(slot.take().is_some())
    }

    /// Cumulative missed periods for a cyclic task.
    pub fn overruns(&self, task: TaskId) -> Option<u32> {
        self.schedule
            .get(usize::from(task.get()))?
            .map(|slot| slot.overruns)
    }

    /// Post an event invocation, exactly like [`Executor::post`].
    pub fn post(&mut self, task: TaskId) -> Result<(), PostError> {
        self.executor.post(task)
    }

    /// Cancel a pending event invocation.
    pub fn cancel(&mut self, task: TaskId) -> Result<bool, PostError> {
        self.executor.cancel(task)
    }

    /// Serve one main-loop tick at `now`.
    ///
    /// Posts every due cyclic task (skipping and counting missed periods),
    /// then drains at most `N` invocations so a repost-happy task cannot
    /// stall the loop. Returns the number of invocations.
    pub fn tick(&mut self, now: Instant) -> usize {
        for (index, slot) in self.schedule.iter_mut().enumerate() {
            let Some(slot) = slot.as_mut() else {
                continue;
            };
            if !now.is_at_or_after(slot.next) {
                continue;
            }
            // Post once for this tick, then advance past every deadline that
            // is already due; each skipped deadline is one overrun.
            let task = TaskId::new(index as u16);
            let _ = self.executor.post(task);
            slot.next = slot.next.wrapping_add(slot.period);
            while now.is_at_or_after(slot.next) {
                slot.overruns = slot.overruns.saturating_add(1);
                slot.next = slot.next.wrapping_add(slot.period);
            }
        }
        self.executor.drain(now, N)
    }

    /// Whether any event or cyclic invocation is currently pending.
    pub fn has_pending(&self) -> bool {
        self.executor.has_pending()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_time::{Clock, FakeClock};

    const MS: Duration = Duration::from_nanos(1_000_000);

    #[derive(Default)]
    struct Recorder {
        at: [Option<Instant>; 8],
        runs: usize,
    }

    impl Runnable for Recorder {
        fn run(&mut self, context: &mut crate::RunContext) {
            if self.runs < self.at.len() {
                self.at[self.runs] = Some(context.now());
            }
            self.runs += 1;
        }
    }

    fn ms(value: u64) -> Duration {
        Duration::from_nanos(value * 1_000_000)
    }

    #[test]
    fn periodic_tasks_fire_at_their_own_periods() {
        let mut fast = Recorder::default();
        let mut slow = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([
                &mut fast as &mut dyn Runnable,
                &mut slow as &mut dyn Runnable,
            ]);
            executor
                .set_period(TaskId::new(0), MS, Instant::from_nanos(0))
                .unwrap();
            executor
                .set_period(TaskId::new(1), ms(10), Instant::from_nanos(0))
                .unwrap();
            let mut clock = FakeClock::default();
            for _ in 0..10 {
                executor.tick(clock.now());
                clock.advance(MS);
            }
        }
        assert_eq!(fast.runs, 10);
        assert_eq!(slow.runs, 1);
    }

    #[test]
    fn deadlines_do_not_drift_when_ticks_are_late() {
        let mut task = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([&mut task as &mut dyn Runnable]);
            executor
                .set_period(TaskId::new(0), ms(10), Instant::from_nanos(0))
                .unwrap();
            // Tick 1.5ms late every period: invocations happen at the tick,
            // but deadlines stay on the 10ms grid, so every period fires.
            for tick in [ms(1), ms(11), ms(22), ms(33)] {
                executor.tick(Instant::from_nanos(0).wrapping_add(tick));
            }
            assert_eq!(executor.overruns(TaskId::new(0)), Some(0));
        }
        assert_eq!(task.runs, 4);
    }

    #[test]
    fn missed_periods_are_skipped_and_counted() {
        let mut task = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([&mut task as &mut dyn Runnable]);
            executor
                .set_period(TaskId::new(0), ms(10), Instant::from_nanos(0))
                .unwrap();
            executor.tick(Instant::from_nanos(0));
            // 35ms gap: deadlines 10, 20, 30 are all due; one invocation,
            // two skipped periods.
            executor.tick(Instant::from_nanos(0).wrapping_add(ms(35)));
            assert_eq!(executor.overruns(TaskId::new(0)), Some(2));
            // Next deadline is back on the grid at 40ms.
            executor.tick(Instant::from_nanos(0).wrapping_add(ms(40)));
        }
        assert_eq!(task.runs, 3);
    }

    #[test]
    fn event_posts_share_the_task_table() {
        let mut cyclic = Recorder::default();
        let mut event = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([
                &mut cyclic as &mut dyn Runnable,
                &mut event as &mut dyn Runnable,
            ]);
            executor
                .set_period(TaskId::new(0), ms(10), Instant::from_nanos(0))
                .unwrap();
            executor.post(TaskId::new(1)).unwrap();
            executor.tick(Instant::from_nanos(0));
            assert_eq!(executor.cancel(TaskId::new(1)), Ok(false));
        }
        assert_eq!(cyclic.runs, 1);
        assert_eq!(event.runs, 1);
    }

    #[test]
    fn clear_period_stops_execution() {
        let mut task = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([&mut task as &mut dyn Runnable]);
            executor
                .set_period(TaskId::new(0), ms(10), Instant::from_nanos(0))
                .unwrap();
            executor.tick(Instant::from_nanos(0));
            assert_eq!(executor.clear_period(TaskId::new(0)), Ok(true));
            assert_eq!(executor.clear_period(TaskId::new(0)), Ok(false));
            executor.tick(Instant::from_nanos(0).wrapping_add(ms(50)));
        }
        assert_eq!(task.runs, 1);
    }

    #[test]
    fn registration_validates_task_and_period() {
        let mut task = Recorder::default();
        let mut executor = CyclicExecutor::new([&mut task as &mut dyn Runnable]);
        assert_eq!(
            executor.set_period(TaskId::new(1), MS, Instant::from_nanos(0)),
            Err(CyclicError::UnknownTask)
        );
        assert_eq!(
            executor.set_period(TaskId::new(0), Duration::ZERO, Instant::from_nanos(0)),
            Err(CyclicError::ZeroPeriod)
        );
        assert_eq!(executor.overruns(TaskId::new(0)), None);
    }

    #[test]
    fn periods_survive_wraparound() {
        let mut task = Recorder::default();
        {
            let mut executor = CyclicExecutor::new([&mut task as &mut dyn Runnable]);
            let near_wrap = Instant::from_nanos(u64::MAX - ms(5).as_nanos());
            executor
                .set_period(TaskId::new(0), ms(10), near_wrap)
                .unwrap();
            executor.tick(near_wrap);
            let after_wrap = near_wrap.wrapping_add(ms(10));
            executor.tick(after_wrap);
            assert_eq!(executor.overruns(TaskId::new(0)), Some(0));
        }
        assert_eq!(task.runs, 2);
    }
}
