//! Simple cyclic task scheduler for bare-metal BSW applications.
//!
//! Runs tasks at fixed periods (1ms, 10ms, 100ms) driven by the DWT timer.
//! No preemption — tasks run cooperatively in the main loop.

use crate::timer::DwtTimer;

/// Maximum number of tasks that can be registered per period.
const MAX_TASKS_PER_PERIOD: usize = 8;

/// A function pointer for a cyclic runnable.
pub type Runnable = fn();

/// Task periods supported by the scheduler.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Period {
    /// 1 ms cycle (fast control loops, CAN TX polling).
    Ms1,
    /// 10 ms cycle (signal processing, state machines).
    Ms10,
    /// 100 ms cycle (diagnostics, heartbeat, LED blink).
    Ms100,
}

impl Period {
    const fn interval_us(self) -> u64 {
        match self {
            Self::Ms1 => 1_000,
            Self::Ms10 => 10_000,
            Self::Ms100 => 100_000,
        }
    }
}

/// A group of tasks sharing the same period.
struct TaskGroup {
    tasks: [Option<Runnable>; MAX_TASKS_PER_PERIOD],
    count: usize,
    last_run_us: u64,
    interval_us: u64,
    overruns: u32,
}

impl TaskGroup {
    const fn new(period: Period) -> Self {
        Self {
            tasks: [None; MAX_TASKS_PER_PERIOD],
            count: 0,
            last_run_us: 0,
            interval_us: period.interval_us(),
            overruns: 0,
        }
    }

    fn add(&mut self, task: Runnable) -> bool {
        if self.count >= MAX_TASKS_PER_PERIOD {
            return false;
        }
        self.tasks[self.count] = Some(task);
        self.count += 1;
        true
    }

    fn run_if_due(&mut self, now_us: u64) -> bool {
        if now_us.wrapping_sub(self.last_run_us) < self.interval_us {
            return false;
        }

        // Detect overrun (missed more than one period).
        if now_us.wrapping_sub(self.last_run_us) >= self.interval_us * 2 {
            self.overruns += 1;
        }

        self.last_run_us = now_us;

        for i in 0..self.count {
            if let Some(task) = self.tasks[i] {
                task();
            }
        }
        true
    }
}

/// Cooperative cyclic scheduler.
///
/// Register runnables at different periods, then call [`tick`](Self::tick)
/// from the main loop. The scheduler dispatches tasks when their period
/// has elapsed based on the DWT timer.
pub struct Scheduler {
    groups: [TaskGroup; 3],
}

impl Scheduler {
    /// Create a new scheduler with empty task groups.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            groups: [
                TaskGroup::new(Period::Ms1),
                TaskGroup::new(Period::Ms10),
                TaskGroup::new(Period::Ms100),
            ],
        }
    }

    /// Register a runnable at the given period.
    ///
    /// Returns `false` if the task group is full.
    pub fn add_task(&mut self, period: Period, task: Runnable) -> bool {
        let idx = match period {
            Period::Ms1 => 0,
            Period::Ms10 => 1,
            Period::Ms100 => 2,
        };
        self.groups[idx].add(task)
    }

    /// Check all task groups and run any that are due.
    ///
    /// Call this as fast as possible from the main loop.
    /// Returns the number of task groups that ran.
    pub fn tick(&mut self, timer: &mut DwtTimer) -> u32 {
        let now = timer.system_time_us_64();
        let mut ran = 0u32;
        for group in &mut self.groups {
            if group.run_if_due(now) {
                ran += 1;
            }
        }
        ran
    }

    /// Return the cumulative overrun count for a period.
    #[must_use]
    pub fn overruns(&self, period: Period) -> u32 {
        let idx = match period {
            Period::Ms1 => 0,
            Period::Ms10 => 1,
            Period::Ms100 => 2,
        };
        self.groups[idx].overruns
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}
