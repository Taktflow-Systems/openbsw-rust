//! Integration tests for the POSIX executor (package C10).
//!
//! The deterministic tests drive every instant through a fake clock; the
//! real-clock tests bound their assertions by ordering rather than exact
//! latency so they stay stable on loaded CI hosts.

use std::sync::mpsc;
use std::thread;

use bsw_async::posix::PosixExecutor;
use bsw_async::{RunContext, Runnable, TaskId};
use bsw_time::{Clock, Duration, FakeClock, Instant, StdClock, TimerError};

const MS: Duration = Duration::from_nanos(1_000_000);

/// Records the instant of every invocation.
#[derive(Default)]
struct Recorder {
    invocations: Vec<Instant>,
}

impl Runnable for Recorder {
    fn run(&mut self, context: &mut RunContext) {
        self.invocations.push(context.now());
    }
}

#[test]
fn fake_clock_schedule_fires_in_deadline_order() {
    let mut early = Recorder::default();
    let mut late = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 2, 4> = PosixExecutor::new([
            &mut early as &mut dyn Runnable,
            &mut late as &mut dyn Runnable,
        ]);
        let mut clock = FakeClock::default();
        executor
            .schedule(TaskId::new(1), Instant::from_nanos(200))
            .unwrap();
        executor
            .schedule(TaskId::new(0), Instant::from_nanos(100))
            .unwrap();

        assert_eq!(executor.run_until_idle(clock.now()), 0);
        clock.advance(Duration::from_nanos(100));
        assert_eq!(executor.run_until_idle(clock.now()), 1);
        clock.advance(Duration::from_nanos(100));
        assert_eq!(executor.run_until_idle(clock.now()), 1);
    }
    assert_eq!(early.invocations, vec![Instant::from_nanos(100)]);
    assert_eq!(late.invocations, vec![Instant::from_nanos(200)]);
}

#[test]
fn fake_clock_periodic_schedule_reposts_every_period() {
    let mut task = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 1, 2> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let mut clock = FakeClock::default();
        executor
            .schedule_periodic(TaskId::new(0), Instant::from_nanos(10), MS)
            .unwrap();
        for _ in 0..3 {
            clock.advance(MS);
            executor.run_until_idle(clock.now());
        }
    }
    assert_eq!(task.invocations.len(), 3);
}

#[test]
fn fake_clock_cancel_scheduled_prevents_posting() {
    let mut task = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 1, 2> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let handle = executor
            .schedule(TaskId::new(0), Instant::from_nanos(5))
            .unwrap();
        assert!(executor.cancel_scheduled(handle));
        assert!(!executor.cancel_scheduled(handle));
        assert_eq!(executor.run_until_idle(Instant::from_nanos(100)), 0);
    }
    assert!(task.invocations.is_empty());
}

#[test]
fn schedule_rejects_unknown_task_ids() {
    let mut task = Recorder::default();
    let mut executor: PosixExecutor<'_, 1, 2> =
        PosixExecutor::new([&mut task as &mut dyn Runnable]);
    assert_eq!(
        executor.schedule(TaskId::new(1), Instant::from_nanos(0)),
        Err(TimerError::CapacityTooLarge)
    );
}

#[test]
fn cross_thread_posts_coalesce_deterministically() {
    let mut task = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let wakeup = executor.wakeup();
        wakeup.post(TaskId::new(0)).unwrap();
        wakeup.post(TaskId::new(0)).unwrap();
        assert_eq!(
            wakeup.post(TaskId::new(1)),
            Err(bsw_async::PostError::UnknownTask)
        );
        assert_eq!(executor.run_until_idle(Instant::from_nanos(7)), 1);
    }
    assert_eq!(task.invocations, vec![Instant::from_nanos(7)]);
}

#[test]
fn cancel_removes_cross_thread_posts() {
    let mut task = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let wakeup = executor.wakeup();
        wakeup.post(TaskId::new(0)).unwrap();
        assert_eq!(executor.cancel(TaskId::new(0)), Ok(true));
        assert_eq!(executor.cancel(TaskId::new(0)), Ok(false));
        assert_eq!(executor.run_until_idle(Instant::from_nanos(0)), 0);
    }
    assert!(task.invocations.is_empty());
}

#[test]
fn real_clock_thread_wakeup_runs_posted_task() {
    let mut task = Recorder::default();
    let invocations = {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let wakeup = executor.wakeup();
        let (ready_tx, ready_rx) = mpsc::channel();
        let poster = thread::spawn(move || {
            ready_rx.recv().expect("executor start signal");
            thread::sleep(std::time::Duration::from_millis(10));
            wakeup.post(TaskId::new(0)).unwrap();
        });
        ready_tx.send(()).unwrap();
        let clock = StdClock::new();
        let invocations = executor.run_for(&clock, Duration::from_millis(400).unwrap());
        poster.join().unwrap();
        invocations
    };
    assert_eq!(invocations, 1);
    assert_eq!(task.invocations.len(), 1);
}

#[test]
fn real_clock_timer_fires_no_earlier_than_deadline() {
    let mut task = Recorder::default();
    let clock = StdClock::new();
    let deadline = clock.now().wrapping_add(Duration::from_millis(20).unwrap());
    {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        executor.schedule(TaskId::new(0), deadline).unwrap();
        let invocations = executor.run_for(&clock, Duration::from_millis(400).unwrap());
        assert_eq!(invocations, 1);
    }
    assert_eq!(task.invocations.len(), 1);
    assert!(task.invocations[0].is_at_or_after(deadline));
}

#[test]
fn real_clock_notify_interrupts_sleep_without_posting() {
    let mut task = Recorder::default();
    let invocations = {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let wakeup = executor.wakeup();
        let handle = thread::spawn(move || {
            thread::sleep(std::time::Duration::from_millis(5));
            wakeup.notify();
        });
        let clock = StdClock::new();
        let invocations = executor.run_for(&clock, Duration::from_millis(60).unwrap());
        handle.join().unwrap();
        invocations
    };
    assert_eq!(invocations, 0);
    assert!(task.invocations.is_empty());
}

#[test]
fn real_clock_many_threads_post_without_losing_wakeups() {
    let mut task = Recorder::default();
    {
        let mut executor: PosixExecutor<'_, 1, 1> =
            PosixExecutor::new([&mut task as &mut dyn Runnable]);
        let mut posters = Vec::new();
        for _ in 0..8 {
            let wakeup = executor.wakeup();
            posters.push(thread::spawn(move || {
                for _ in 0..100 {
                    wakeup.post(TaskId::new(0)).unwrap();
                }
            }));
        }
        let clock = StdClock::new();
        executor.run_for(&clock, Duration::from_millis(200).unwrap());
        for poster in posters {
            poster.join().unwrap();
        }
        // Any posts that raced past the window still coalesce into one
        // deterministic drain.
        executor.run_until_idle(clock.now());
    }
    assert!(!task.invocations.is_empty());
}
